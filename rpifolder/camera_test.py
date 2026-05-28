#!/usr/bin/env python3
"""Capture one test image from a USB camera and exit.

Example:
    python3 camera_test.py
    python3 camera_test.py --list-cameras
    python3 camera_test.py --probe-cameras
    python3 camera_test.py --camera-device /dev/video2 --output test.jpg
"""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture one image from a USB camera.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="OpenCV camera index. Index 0 usually maps to /dev/video0.",
    )
    parser.add_argument(
        "--camera-device",
        default=None,
        help="Explicit Linux video device path, such as /dev/video2. Overrides --camera-index.",
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="Print detected /dev/video* devices and exit.",
    )
    parser.add_argument(
        "--probe-cameras",
        action="store_true",
        help="Try each detected /dev/video* device and report whether it returns a frame.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output JPG path. Defaults to camera_test_YYYYMMDD_HHMMSS.jpg.",
    )
    parser.add_argument(
        "--image-width",
        type=int,
        default=None,
        help="Optional capture width. Leave unset to use the camera default.",
    )
    parser.add_argument(
        "--image-height",
        type=int,
        default=None,
        help="Optional capture height. Leave unset to use the camera default.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=90,
        help="JPEG quality, from 1 to 100.",
    )
    return parser


def default_output_path() -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"camera_test_{timestamp}.jpg")


def list_video_devices() -> list[Path]:
    return sorted(Path("/dev").glob("video*"))


def print_video_devices() -> None:
    devices = list_video_devices()
    if not devices:
        print("No /dev/video* devices found.")
        return

    print("Detected video devices:")
    for device in devices:
        print(f"  {device}")


def camera_source(args: argparse.Namespace) -> int | str:
    if args.camera_device:
        return args.camera_device
    return args.camera_index


def open_camera(cv2: object, source: int | str) -> object:
    if isinstance(source, str) and source.startswith("/dev/video"):
        return cv2.VideoCapture(source, cv2.CAP_V4L2)
    return cv2.VideoCapture(source)


def apply_camera_settings(cv2: object, camera: object, args: argparse.Namespace) -> None:
    if args.image_width is not None:
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
    if args.image_height is not None:
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)


def probe_cameras(cv2: object, args: argparse.Namespace) -> int:
    devices = list_video_devices()
    if not devices:
        print("No /dev/video* devices found.")
        return 1

    found_working_camera = False
    print("Probing detected video devices with OpenCV V4L2:")
    for device in devices:
        camera = open_camera(cv2, str(device))
        try:
            apply_camera_settings(cv2, camera, args)
            if not camera.isOpened():
                print(f"  {device}: open failed")
                continue

            ok, frame = camera.read()
            if not ok or frame is None:
                print(f"  {device}: opened, frame read failed")
                continue

            height, width = frame.shape[:2]
            found_working_camera = True
            print(f"  {device}: OK, {width}x{height}")
        finally:
            camera.release()

    return 0 if found_working_camera else 1


def print_open_failure_help(args: argparse.Namespace) -> None:
    print_video_devices()
    print()
    print("Try one of these on the Raspberry Pi:")
    print("  ls -l /dev/video*")
    print("  v4l2-ctl --list-devices")
    print("  python3 camera_test.py --probe-cameras")
    print("  python3 camera_test.py --camera-device /dev/videoX --output test.jpg")
    print()
    print("If no /dev/video* devices appear, unplug/replug the camera and check:")
    print("  dmesg | tail -40")
    print("If devices appear but access fails, try adding your user to the video group:")
    print("  sudo usermod -aG video $USER")
    print("Then log out and back in before retrying.")


def main() -> int:
    args = build_parser().parse_args()

    if args.list_cameras:
        print_video_devices()
        return 0

    output_path = Path(args.output).expanduser() if args.output else default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    jpeg_quality = min(max(args.jpeg_quality, 1), 100)

    try:
        import cv2
    except ImportError as exc:
        print(f"OpenCV import failed: {exc}")
        print("Install OpenCV on Raspberry Pi with: sudo apt install python3-opencv")
        return 1

    if args.probe_cameras:
        return probe_cameras(cv2, args)

    source = camera_source(args)
    camera = open_camera(cv2, source)
    try:
        apply_camera_settings(cv2, camera, args)

        if not camera.isOpened():
            print(f"Camera source {source!r} failed to open.")
            print_open_failure_help(args)
            return 1

        ok, frame = camera.read()
        if not ok or frame is None:
            print("Camera opened, but frame capture failed.")
            return 1

        height, width = frame.shape[:2]
        saved = cv2.imwrite(
            str(output_path),
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
        )
        if not saved:
            print(f"Failed to write image to {output_path}")
            return 1

        print(f"Saved {width}x{height} image to {output_path}")
        return 0
    finally:
        camera.release()


if __name__ == "__main__":
    raise SystemExit(main())
