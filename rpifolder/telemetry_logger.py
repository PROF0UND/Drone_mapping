#!/usr/bin/env python3
"""Run MAVProxy on a Raspberry Pi and save flight-controller telemetry logs.

This script wraps MAVProxy with Pi-friendly defaults:
  - reads MAVLink from /dev/serial0 at 921600 baud
  - forwards telemetry to UDP 127.0.0.1:14550
  - stores each run in a timestamped log directory
  - captures MAVProxy stdout/stderr to a console log

Example:
    python3 telemetry_logger.py

Stop with Ctrl+C. For flight use, run it from systemd or a terminal multiplexer.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_MASTER = "/dev/serial0"
DEFAULT_BAUDRATE = 921600
DEFAULT_OUT = "udp:127.0.0.1:14550"
DEFAULT_TELEMETRY_IN = "udpin:127.0.0.1:14550"
DEFAULT_LOG_ROOT = "~/flight_logs"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Start MAVProxy and record telemetry from a flight controller.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--master", default=DEFAULT_MASTER, help="Flight controller serial device.")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="Serial baud rate.")
    parser.add_argument(
        "--out",
        action="append",
        default=None,
        help="MAVProxy output endpoint. May be passed more than once.",
    )
    parser.add_argument(
        "--telemetry-in",
        default=DEFAULT_TELEMETRY_IN,
        help="pymavlink input endpoint used to read the MAVProxy UDP stream.",
    )
    parser.add_argument(
        "--print-rate",
        type=float,
        default=1.0,
        help="How many JSON telemetry lines to print per second.",
    )
    parser.add_argument(
        "--show-mavproxy-console",
        action="store_true",
        help="Also print raw MAVProxy console output. It is always saved to mavproxy_console.log.",
    )
    parser.add_argument(
        "--log-root",
        default=DEFAULT_LOG_ROOT,
        help="Directory where timestamped telemetry sessions are stored.",
    )
    parser.add_argument(
        "--mavproxy",
        default="mavproxy.py",
        help="MAVProxy executable name or full path.",
    )
    parser.add_argument(
        "--daemon",
        action="store_true",
        help="Pass --daemon to MAVProxy. Console capture is limited in daemon mode.",
    )
    parser.add_argument(
        "--restart",
        action="store_true",
        help="Restart MAVProxy if it exits unexpectedly.",
    )
    parser.add_argument(
        "--restart-delay",
        type=float,
        default=3.0,
        help="Seconds to wait before restarting MAVProxy.",
    )
    return parser


@dataclass
class TelemetryState:
    latitude: float = 0.0
    longitude: float = 0.0
    gps_locked: bool = False
    heading_degrees: float = 0.0
    altitude_m: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        latitude = self.latitude if self.gps_locked else 0.0
        longitude = self.longitude if self.gps_locked else 0.0
        return {
            "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds"),
            "gps_locked": self.gps_locked,
            "gps_latitude": round(latitude, 7),
            "gps_longitude": round(longitude, 7),
            "heading_degrees": round(self.heading_degrees, 2),
            "altitude_m": round(self.altitude_m, 2),
        }


def make_session_dir(log_root: str) -> Path:
    root = Path(log_root).expanduser()
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = root / f"telemetry_{timestamp}"
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def build_mavproxy_command(args: argparse.Namespace, session_dir: Path) -> list[str]:
    outputs = args.out if args.out else [DEFAULT_OUT]

    command = [
        args.mavproxy,
        "--non-interactive",
        "--default-modules=",
        "--continue",
        f"--master={args.master}",
        f"--baudrate={args.baudrate}",
        f"--aircraft={session_dir.name}",
        f"--state-basedir={str(session_dir.parent)}",
    ]

    if args.daemon:
        command.append("--daemon")

    for endpoint in outputs:
        command.append(f"--out={endpoint}")

    return command


def write_session_info(session_dir: Path, command: list[str], args: argparse.Namespace) -> None:
    info_path = session_dir / "session_info.txt"
    with info_path.open("w", encoding="utf-8") as info_file:
        info_file.write(f"started_at={dt.datetime.now().isoformat(timespec='seconds')}\n")
        info_file.write(f"master={args.master}\n")
        info_file.write(f"baudrate={args.baudrate}\n")
        info_file.write(f"outputs={','.join(args.out if args.out else [DEFAULT_OUT])}\n")
        info_file.write(f"telemetry_in={args.telemetry_in}\n")
        info_file.write("command=" + " ".join(command) + "\n")


def normalize_heading(heading: float) -> float:
    return heading % 360.0


def update_telemetry_from_message(state: TelemetryState, message: object) -> None:
    message_type = message.get_type()

    if message_type == "GPS_RAW_INT":
        fix_type = getattr(message, "fix_type", 0)
        lat = getattr(message, "lat", 0)
        lon = getattr(message, "lon", 0)
        state.gps_locked = fix_type >= 3 and lat != 0 and lon != 0
        if state.gps_locked:
            state.latitude = lat / 1e7
            state.longitude = lon / 1e7
        altitude_mm = getattr(message, "alt", None)
        if altitude_mm is not None:
            state.altitude_m = altitude_mm / 1000.0

    elif message_type == "GLOBAL_POSITION_INT":
        lat = getattr(message, "lat", 0)
        lon = getattr(message, "lon", 0)
        if lat != 0 and lon != 0:
            state.gps_locked = True
            state.latitude = lat / 1e7
            state.longitude = lon / 1e7

        relative_alt_mm = getattr(message, "relative_alt", None)
        alt_mm = getattr(message, "alt", None)
        if relative_alt_mm is not None:
            state.altitude_m = relative_alt_mm / 1000.0
        elif alt_mm is not None:
            state.altitude_m = alt_mm / 1000.0

        heading_centidegrees = getattr(message, "hdg", 65535)
        if heading_centidegrees != 65535:
            state.heading_degrees = normalize_heading(heading_centidegrees / 100.0)

    elif message_type == "VFR_HUD":
        heading = getattr(message, "heading", None)
        alt = getattr(message, "alt", None)
        if heading is not None:
            state.heading_degrees = normalize_heading(float(heading))
        if alt is not None:
            state.altitude_m = float(alt)

    elif message_type == "ATTITUDE":
        yaw = getattr(message, "yaw", None)
        if yaw is not None:
            state.heading_degrees = normalize_heading(math.degrees(float(yaw)))


def telemetry_reader(endpoint: str, session_dir: Path, print_rate: float, stop_event: threading.Event) -> None:
    from pymavlink import mavutil

    telemetry_log_path = session_dir / "telemetry_json.log"
    state = TelemetryState()
    next_print_at = 0.0
    interval = 1.0 / print_rate if print_rate > 0 else 1.0

    with telemetry_log_path.open("a", encoding="utf-8", buffering=1) as telemetry_log:
        telemetry_log.write(f"\n[{dt.datetime.now().isoformat(timespec='seconds')}] listening on {endpoint}\n")
        mavlink = mavutil.mavlink_connection(endpoint)

        while not stop_event.is_set():
            message = mavlink.recv_match(blocking=False)
            if message is not None:
                update_telemetry_from_message(state, message)

            now = time.monotonic()
            if now >= next_print_at:
                line = json.dumps(state.to_dict())
                print(line, flush=True)
                telemetry_log.write(line + "\n")
                next_print_at = now + interval

            time.sleep(0.02)


def terminate_process(process: subprocess.Popen[str], timeout_seconds: float = 8.0) -> None:
    if process.poll() is not None:
        return

    process.terminate()
    try:
        process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def run_mavproxy(command: list[str], session_dir: Path, args: argparse.Namespace) -> int:
    console_log_path = session_dir / "mavproxy_console.log"
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=telemetry_reader,
        args=(args.telemetry_in, session_dir, args.print_rate, stop_event),
        daemon=True,
    )

    with console_log_path.open("a", encoding="utf-8", buffering=1) as console_log:
        console_log.write(f"\n[{dt.datetime.now().isoformat(timespec='seconds')}] starting MAVProxy\n")
        console_log.write("command: " + " ".join(command) + "\n\n")

        reader_thread.start()

        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )

        stopping = False

        def handle_stop(signum: int, _frame: object) -> None:
            nonlocal stopping
            stopping = True
            stop_event.set()
            console_log.write(f"\n[{dt.datetime.now().isoformat(timespec='seconds')}] signal {signum}; stopping\n")
            terminate_process(process)

        old_sigint = signal.signal(signal.SIGINT, handle_stop)
        old_sigterm = signal.signal(signal.SIGTERM, handle_stop)

        try:
            assert process.stdout is not None
            for line in process.stdout:
                console_log.write(line)
                if args.show_mavproxy_console:
                    print(line, end="")

            return_code = process.wait()
            if stopping:
                return 0
            return return_code
        finally:
            signal.signal(signal.SIGINT, old_sigint)
            signal.signal(signal.SIGTERM, old_sigterm)
            stop_event.set()
            terminate_process(process)
            reader_thread.join(timeout=2.0)
            console_log.write(f"\n[{dt.datetime.now().isoformat(timespec='seconds')}] MAVProxy stopped\n")


def main() -> int:
    args = build_parser().parse_args()
    session_dir = make_session_dir(args.log_root)
    command = build_mavproxy_command(args, session_dir)
    write_session_info(session_dir, command, args)

    print(f"Telemetry session: {session_dir}")
    print("MAVProxy command: " + " ".join(command))
    print("Console log: " + str(session_dir / "mavproxy_console.log"))
    print("Telemetry JSON log: " + str(session_dir / "telemetry_json.log"))

    while True:
        return_code = run_mavproxy(command, session_dir, args)

        if return_code == 0 or not args.restart:
            return return_code

        print(f"MAVProxy exited with code {return_code}; restarting in {args.restart_delay:.1f}s")
        time.sleep(args.restart_delay)


if __name__ == "__main__":
    raise SystemExit(main())
