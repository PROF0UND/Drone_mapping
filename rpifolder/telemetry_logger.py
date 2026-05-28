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
from dataclasses import dataclass, field
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
    parser.add_argument(
        "--capture-images",
        action="store_true",
        help="Capture images from a USB camera and pair each image with latest telemetry.",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="OpenCV camera index for the USB camera. Index 0 usually maps to /dev/video0.",
    )
    parser.add_argument(
        "--capture-interval",
        type=float,
        default=1.0,
        help="Seconds between image captures when --capture-images is enabled.",
    )
    parser.add_argument(
        "--image-width",
        type=int,
        default=None,
        help="Optional camera capture width. Leave unset to use the camera default.",
    )
    parser.add_argument(
        "--image-height",
        type=int,
        default=None,
        help="Optional camera capture height. Leave unset to use the camera default.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=90,
        help="JPEG quality for saved images, from 1 to 100.",
    )
    return parser


@dataclass
class TelemetryState:
    latitude: float = 0.0
    longitude: float = 0.0
    gps_locked: bool = False
    heading_degrees: float = 0.0
    altitude_m: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def to_dict(self) -> dict[str, Any]:
        with self.lock:
            return self._to_dict_unlocked()

    def snapshot(self) -> dict[str, Any]:
        return self.to_dict()

    def _to_dict_unlocked(self) -> dict[str, Any]:
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
        info_file.write(f"capture_images={args.capture_images}\n")
        if args.capture_images:
            info_file.write(f"camera_index={args.camera_index}\n")
            info_file.write(f"capture_interval={args.capture_interval}\n")
            info_file.write(f"image_width={args.image_width}\n")
            info_file.write(f"image_height={args.image_height}\n")
            info_file.write(f"jpeg_quality={args.jpeg_quality}\n")
        info_file.write("command=" + " ".join(command) + "\n")


def normalize_heading(heading: float) -> float:
    return heading % 360.0


def update_telemetry_from_message(state: TelemetryState, message: object) -> None:
    with state.lock:
        update_telemetry_from_message_unlocked(state, message)


def update_telemetry_from_message_unlocked(state: TelemetryState, message: object) -> None:
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


def image_filename(sequence: int) -> str:
    return f"image_{sequence:06d}.jpg"


def make_image_manifest_record(
    sequence: int,
    image_path: Path,
    captured_at: str,
    camera_index: int,
    width: int,
    height: int,
    telemetry: dict[str, Any],
    capture_ok: bool,
    error: str | None = None,
) -> dict[str, Any]:
    record: dict[str, Any] = {
        "sequence": sequence,
        "image_path": str(image_path),
        "captured_at": captured_at,
        "camera_index": camera_index,
        "width": width,
        "height": height,
        "telemetry": telemetry,
        "capture_ok": capture_ok,
    }
    if error:
        record["error"] = error
    return record


def image_capture_worker(
    session_dir: Path,
    args: argparse.Namespace,
    telemetry_state: TelemetryState,
    stop_event: threading.Event,
) -> None:
    image_log_path = session_dir / "image_capture.log"
    manifest_path = session_dir / "image_manifest.jsonl"
    images_dir = session_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    with image_log_path.open("a", encoding="utf-8", buffering=1) as image_log:
        image_log.write(f"\n[{dt.datetime.now().isoformat(timespec='seconds')}] starting image capture\n")

        try:
            import cv2
        except ImportError as exc:
            image_log.write(f"OpenCV import failed: {exc}\n")
            image_log.write("Install OpenCV on Raspberry Pi with: sudo apt install python3-opencv\n")
            return

        camera = cv2.VideoCapture(args.camera_index)
        try:
            if args.image_width is not None:
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
            if args.image_height is not None:
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)

            if not camera.isOpened():
                image_log.write(f"Camera index {args.camera_index} failed to open\n")
                return

            sequence = 1
            next_capture_at = 0.0
            interval = max(args.capture_interval, 0.1)
            jpeg_quality = min(max(args.jpeg_quality, 1), 100)

            with manifest_path.open("a", encoding="utf-8", buffering=1) as manifest:
                while not stop_event.is_set():
                    now = time.monotonic()
                    if now < next_capture_at:
                        stop_event.wait(min(0.05, next_capture_at - now))
                        continue

                    captured_at = dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")
                    image_path = images_dir / image_filename(sequence)
                    telemetry = telemetry_state.snapshot()
                    capture_ok = False
                    error = None
                    width = 0
                    height = 0

                    ok, frame = camera.read()
                    if not ok or frame is None:
                        error = "Camera frame capture failed"
                        image_log.write(f"[{captured_at}] {error}\n")
                    else:
                        height, width = frame.shape[:2]
                        write_ok = cv2.imwrite(
                            str(image_path),
                            frame,
                            [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
                        )
                        if write_ok:
                            capture_ok = True
                        else:
                            error = f"Failed to write image to {image_path}"
                            image_log.write(f"[{captured_at}] {error}\n")

                    record = make_image_manifest_record(
                        sequence=sequence,
                        image_path=image_path,
                        captured_at=captured_at,
                        camera_index=args.camera_index,
                        width=width,
                        height=height,
                        telemetry=telemetry,
                        capture_ok=capture_ok,
                        error=error,
                    )
                    manifest.write(json.dumps(record) + "\n")

                    sequence += 1
                    next_capture_at = now + interval
        finally:
            camera.release()
            image_log.write(f"[{dt.datetime.now().isoformat(timespec='seconds')}] image capture stopped\n")


def telemetry_reader(
    endpoint: str,
    session_dir: Path,
    print_rate: float,
    telemetry_state: TelemetryState,
    stop_event: threading.Event,
) -> None:
    from pymavlink import mavutil

    telemetry_log_path = session_dir / "telemetry_json.log"
    next_print_at = 0.0
    interval = 1.0 / print_rate if print_rate > 0 else 1.0

    with telemetry_log_path.open("a", encoding="utf-8", buffering=1) as telemetry_log:
        telemetry_log.write(f"\n[{dt.datetime.now().isoformat(timespec='seconds')}] listening on {endpoint}\n")
        mavlink = mavutil.mavlink_connection(endpoint)

        while not stop_event.is_set():
            message = mavlink.recv_match(blocking=False)
            if message is not None:
                update_telemetry_from_message(telemetry_state, message)

            now = time.monotonic()
            if now >= next_print_at:
                line = json.dumps(telemetry_state.snapshot())
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
    telemetry_state = TelemetryState()
    reader_thread = threading.Thread(
        target=telemetry_reader,
        args=(args.telemetry_in, session_dir, args.print_rate, telemetry_state, stop_event),
        daemon=True,
    )
    image_thread = None
    if args.capture_images:
        image_thread = threading.Thread(
            target=image_capture_worker,
            args=(session_dir, args, telemetry_state, stop_event),
            daemon=True,
        )

    with console_log_path.open("a", encoding="utf-8", buffering=1) as console_log:
        console_log.write(f"\n[{dt.datetime.now().isoformat(timespec='seconds')}] starting MAVProxy\n")
        console_log.write("command: " + " ".join(command) + "\n\n")

        reader_thread.start()
        if image_thread is not None:
            image_thread.start()

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
            if image_thread is not None:
                image_thread.join(timeout=2.0)
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
    if args.capture_images:
        print("Image directory: " + str(session_dir / "images"))
        print("Image manifest: " + str(session_dir / "image_manifest.jsonl"))
        print("Image capture log: " + str(session_dir / "image_capture.log"))

    while True:
        return_code = run_mavproxy(command, session_dir, args)

        if return_code == 0 or not args.restart:
            return return_code

        print(f"MAVProxy exited with code {return_code}; restarting in {args.restart_delay:.1f}s")
        time.sleep(args.restart_delay)


if __name__ == "__main__":
    raise SystemExit(main())
