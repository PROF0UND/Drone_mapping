#!/usr/bin/env python3
"""
MAVLink Telemetry Viewer with RC Channel Bars
Usage: python telemetry_viewer.py [--port /dev/serial0] [--baud 460800]
"""

import argparse
import math
import os
import subprocess
import threading
import time
import sys
from collections import deque
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not found. Install it: pip install pymavlink")
    sys.exit(1)

try:
    from rich.console import Console
    from rich.layout import Layout
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from rich import box as rbox
except ImportError:
    print("rich not found. Install it: pip install rich")
    sys.exit(1)

# ── Configuration ────────────────────────────────────────────────────────────
RC_MIN = 1000
RC_MAX = 2000
BAR_WIDTH = 36
MAX_LOGS = 25

# Camera trigger — CH9 rising edge above this PWM fires a capture
CAM_CHANNEL       = 9
CAM_THRESHOLD     = 1800   # "high" position; raise to 1950+ if you only want >2000
CAM_COOLDOWN_S    = 2.0    # minimum seconds between captures
PHOTO_DIR         = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")
PHOTO_W, PHOTO_H  = 4056, 3040

RC_LABELS = {
    1: "Roll",
    2: "Pitch",
    3: "Throttle",
    4: "Yaw",
    5: "Mode SW",
    6: "CH 6",
    7: "CH 7",
    8: "CH 8",
    9: "Camera",
    10: "CH 10",
    11: "CH 11",
    12: "CH 12",
    13: "CH 13",
    14: "CH 14",
    15: "CH 15",
    16: "CH 16",
}

FLIGHT_MODES = {
    0: "STABILIZE", 1: "ACRO",    2: "ALT_HOLD",
    3: "AUTO",      4: "GUIDED",  5: "LOITER",
    6: "RTL",       7: "CIRCLE",  9: "LAND",
    11: "DRIFT",   13: "SPORT",  15: "AUTOTUNE",
    16: "POSHOLD", 17: "BRAKE",  18: "THROW",
    19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
}

GPS_FIX = {
    0: ("NO GPS",     "red"),
    1: ("NO FIX",     "red"),
    2: ("2D FIX",     "yellow"),
    3: ("3D FIX",     "green"),
    4: ("DGPS",       "bright_green"),
    5: ("RTK FLOAT",  "bright_green"),
    6: ("RTK FIXED",  "bold green"),
}

LOG_COLORS = {
    "OK":        "bold green",
    "INFO":      "white",
    "NOTICE":    "cyan",
    "WARNING":   "yellow",
    "ERROR":     "red",
    "CRITICAL":  "bold red",
    "ALERT":     "bold red",
    "EMERGENCY": "bold red",
    "DEBUG":     "dim white",
}

MAV_SEVERITY = {
    0: "EMERGENCY", 1: "ALERT",   2: "CRITICAL",
    3: "ERROR",     4: "WARNING", 5: "NOTICE",
    6: "INFO",      7: "DEBUG",
}

# ── Shared state ──────────────────────────────────────────────────────────────
state = {
    "rc":        {},   # {ch_num: pwm}
    "attitude":  {},   # roll, pitch, yaw (degrees)
    "gps":       {},   # lat, lon, alt, fix_type, satellites
    "battery":   {},   # voltage, current, remaining
    "heartbeat": {},   # armed, mode, type
    "vfr_hud":   {},   # airspeed, groundspeed, heading, throttle, alt, climb
    "connected": False,
    "last_msg":  0.0,
}
logs: deque = deque(maxlen=MAX_LOGS)
_lock = threading.Lock()


def log(msg: str, level: str = "INFO"):
    ts = datetime.now().strftime("%H:%M:%S")
    with _lock:
        logs.append((ts, level, str(msg)))


# ── Camera state ──────────────────────────────────────────────────────────────
cam = {
    "ch_was_high":  False,   # previous-frame high/low for edge detection
    "capturing":    False,   # True while rpicam-still is running
    "last_capture": 0.0,     # time.time() of last successful trigger
    "photo_count":  0,
    "last_file":    "",
}


def capture_photo(photo_dir: str):
    """Fire rpicam-still in a background thread; called on rising edge only."""
    cam["capturing"]   = True
    cam["photo_count"] += 1
    count = cam["photo_count"]
    ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
    path  = os.path.join(photo_dir, f"photo_{ts}_{count:04d}.jpg")

    def _run():
        try:
            log(f"[Camera] Capturing → {os.path.basename(path)}", "NOTICE")
            result = subprocess.run(
                [
                    "rpicam-still",
                    "-o", path,
                    "--width",  str(PHOTO_W),
                    "--height", str(PHOTO_H),
                    "--nopreview",
                ],
                timeout=20,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                cam["last_file"]    = path
                cam["last_capture"] = time.time()
                log(f"[Camera] Saved  #{count}  {os.path.basename(path)}", "OK")
            else:
                err = result.stderr.strip().splitlines()[-1] if result.stderr.strip() else "unknown"
                log(f"[Camera] Error: {err}", "ERROR")
        except subprocess.TimeoutExpired:
            log("[Camera] Timed out after 20 s", "ERROR")
        except FileNotFoundError:
            log("[Camera] rpicam-still not found — is it installed?", "ERROR")
        except Exception as exc:
            log(f"[Camera] {exc}", "ERROR")
        finally:
            cam["capturing"] = False

    threading.Thread(target=_run, daemon=True).start()


# ── MAVLink receive thread ────────────────────────────────────────────────────
def mav_thread(port: str, baud: int):
    log(f"Connecting → {port}  @{baud} baud", "INFO")
    try:
        conn = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        log(f"Could not open port: {e}", "ERROR")
        return

    log("Waiting for heartbeat …", "INFO")
    try:
        conn.wait_heartbeat(timeout=20)
    except Exception as e:
        log(f"No heartbeat: {e}", "ERROR")
        return

    with _lock:
        state["connected"] = True
    log(f"Heartbeat OK  sys={conn.target_system}  comp={conn.target_component}", "OK")

    # Ask for all streams at 10 Hz
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
    )

    while True:
        try:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            _process(msg)
        except Exception as exc:
            log(f"Recv error: {exc}", "WARNING")
            time.sleep(0.2)


def _process(msg):
    t = msg.get_type()
    with _lock:
        state["last_msg"] = time.time()

        if t in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
            d = msg.to_dict()
            rc = {}
            for i in range(1, 19):
                v = d.get(f"chan{i}_raw", 0)
                if v and v > 0:
                    rc[i] = v
            if rc:
                state["rc"] = rc

        elif t == "ATTITUDE":
            state["attitude"] = {
                "roll":  math.degrees(msg.roll),
                "pitch": math.degrees(msg.pitch),
                "yaw":   math.degrees(msg.yaw),
            }

        elif t in ("GPS_RAW_INT", "GPS2_RAW"):
            state["gps"] = {
                "lat":        msg.lat / 1e7,
                "lon":        msg.lon / 1e7,
                "alt":        msg.alt / 1000.0,
                "fix_type":   msg.fix_type,
                "satellites": msg.satellites_visible,
            }

        elif t == "SYS_STATUS":
            state["battery"] = {
                "voltage":   msg.voltage_battery / 1000.0,
                "current":   msg.current_battery / 100.0,
                "remaining": msg.battery_remaining,
            }

        elif t == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state["heartbeat"] = {
                "armed": armed,
                "mode":  FLIGHT_MODES.get(msg.custom_mode, f"MODE {msg.custom_mode}"),
            }

        elif t == "VFR_HUD":
            state["vfr_hud"] = {
                "airspeed":    msg.airspeed,
                "groundspeed": msg.groundspeed,
                "heading":     msg.heading,
                "throttle":    msg.throttle,
                "alt":         msg.alt,
                "climb":       msg.climb,
            }

        elif t == "STATUSTEXT":
            level = MAV_SEVERITY.get(msg.severity, "INFO")
            logs.append((datetime.now().strftime("%H:%M:%S"), level, msg.text.strip()))


# ── RC bar renderer ───────────────────────────────────────────────────────────
def _rc_bar(pwm: int, width: int = BAR_WIDTH) -> Text:
    pwm = max(RC_MIN, min(RC_MAX, pwm))
    frac = (pwm - RC_MIN) / (RC_MAX - RC_MIN)
    filled = int(frac * width)

    if frac <= 0.10 or frac >= 0.90:
        color = "red"
    elif 0.42 <= frac <= 0.58:
        color = "bright_green"
    else:
        color = "yellow"

    bar = "█" * filled + "░" * (width - filled)
    return Text(bar, style=color)


# ── Panel builders ────────────────────────────────────────────────────────────
def build_rc_panel(rc: dict) -> Panel:
    tbl = Table(box=None, show_header=False, padding=(0, 1), expand=True)
    tbl.add_column("CH",    width=3,          style="dim cyan",   justify="right")
    tbl.add_column("Name",  width=9)
    tbl.add_column("Bar",   width=BAR_WIDTH + 2)
    tbl.add_column("PWM",   width=5,          justify="right",    style="white")
    tbl.add_column("%",     width=4,          justify="right",    style="dim")

    if not rc:
        tbl.add_row("—", "No RC data", Text("Waiting for RC_CHANNELS message…", style="dim"), "", "")
    else:
        for ch in sorted(rc):
            pwm = rc[ch]
            pct = int((pwm - RC_MIN) / (RC_MAX - RC_MIN) * 100)
            label = RC_LABELS.get(ch, f"CH {ch}")
            tbl.add_row(str(ch), label, _rc_bar(pwm), str(pwm), f"{pct}%")

    return Panel(tbl, title="[bold cyan]RC Channels[/bold cyan]", border_style="cyan")


def build_telem_panel(s: dict) -> Panel:
    lines: list[str] = []

    hb = s["heartbeat"]
    if hb:
        arm_txt = ("[bold red]◆ ARMED[/bold red]" if hb["armed"]
                   else "[green]◇ DISARMED[/green]")
        lines.append(f"  {arm_txt}    Mode: [bold white]{hb['mode']}[/bold white]")
    else:
        lines.append("  [dim]Waiting for heartbeat…[/dim]")

    att = s["attitude"]
    if att:
        lines.append(
            f"  Roll [cyan]{att['roll']:+7.1f}°[/cyan]  "
            f"Pitch [cyan]{att['pitch']:+7.1f}°[/cyan]  "
            f"Yaw [cyan]{att['yaw']:+7.1f}°[/cyan]"
        )

    vfr = s["vfr_hud"]
    if vfr:
        lines.append(
            f"  Airspeed [yellow]{vfr['airspeed']:.1f} m/s[/yellow]  "
            f"Groundspeed [yellow]{vfr['groundspeed']:.1f} m/s[/yellow]  "
            f"Throttle [yellow]{vfr['throttle']}%[/yellow]  "
            f"Climb [yellow]{vfr['climb']:+.2f} m/s[/yellow]"
        )
        lines.append(
            f"  Altitude [magenta]{vfr['alt']:.1f} m[/magenta]  "
            f"Heading [magenta]{vfr['heading']}°[/magenta]"
        )

    gps = s["gps"]
    if gps:
        fix_label, fix_color = GPS_FIX.get(gps["fix_type"], ("?", "white"))
        lines.append(
            f"  GPS [{fix_color}]{fix_label}[/{fix_color}]  "
            f"Sats [cyan]{gps['satellites']}[/cyan]  "
            f"[dim]{gps['lat']:.6f}, {gps['lon']:.6f}[/dim]  "
            f"Alt [cyan]{gps['alt']:.1f} m[/cyan]"
        )

    bat = s["battery"]
    if bat:
        rem = bat["remaining"]
        bc = "green" if rem > 50 else ("yellow" if rem > 20 else "red")
        lines.append(
            f"  Battery [{bc}]{bat['voltage']:.2f} V  "
            f"{bat['current']:.1f} A  "
            f"{rem}%[/{bc}]"
        )

    # Staleness warning
    lag = time.time() - s.get("last_msg", 0)
    if s["connected"] and lag > 3:
        lines.append(f"\n  [yellow]⚠  No message for {lag:.0f}s[/yellow]")
    elif not s["connected"]:
        lines.append("\n  [dim]Not connected[/dim]")

    content = "\n".join(lines) if lines else "  [dim]No data yet[/dim]"
    return Panel(
        Text.from_markup(content),
        title="[bold green]Telemetry[/bold green]",
        border_style="green",
    )


def build_log_panel(log_snapshot: list) -> Panel:
    tbl = Table(box=None, show_header=False, padding=(0, 1), expand=True)
    tbl.add_column("Time",  width=8,  style="dim")
    tbl.add_column("Level", width=10)
    tbl.add_column("Message")

    for ts, level, msg in log_snapshot:
        color = LOG_COLORS.get(level, "white")
        tbl.add_row(ts, Text(level, style=color), msg)

    return Panel(tbl, title="[bold yellow]MAVLink Log[/bold yellow]", border_style="yellow")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="MAVLink telemetry + RC viewer")
    parser.add_argument("--port",  default="/dev/serial0", help="Serial port")
    parser.add_argument("--baud",  default=460800, type=int, help="Baud rate")
    args = parser.parse_args()

    threading.Thread(
        target=mav_thread, args=(args.port, args.baud), daemon=True
    ).start()

    layout = Layout()
    layout.split_column(
        Layout(name="telem", ratio=2),
        Layout(name="rc",    ratio=4),
        Layout(name="log",   ratio=3),
    )

    console = Console()
    with Live(layout, console=console, refresh_per_second=15, screen=True):
        try:
            while True:
                with _lock:
                    snap = {k: (dict(v) if isinstance(v, dict) else v)
                            for k, v in state.items()}
                    log_snap = list(logs)

                layout["telem"].update(build_telem_panel(snap))
                layout["rc"].update(build_rc_panel(snap["rc"]))
                layout["log"].update(build_log_panel(log_snap))
                time.sleep(0.067)   # ~15 fps
        except KeyboardInterrupt:
            pass

    print("\nMAVLink viewer closed.")


if __name__ == "__main__":
    main()
