from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


VIDEO_EXTENSIONS = {".mp4", ".mov", ".avi", ".mkv", ".wmv", ".m4v"}


def load_cv2():
    try:
        import cv2
    except ImportError:
        print(
            "OpenCV is required to extract video frames. Install it with:\n"
            "  python -m pip install opencv-python",
            file=sys.stderr,
        )
        raise SystemExit(1)

    return cv2


def find_video(video_dir: Path) -> Path:
    if not video_dir.exists():
        raise FileNotFoundError(f"Video directory does not exist: {video_dir}")
    if not video_dir.is_dir():
        raise NotADirectoryError(f"Expected a directory: {video_dir}")

    videos = [
        path
        for path in sorted(video_dir.iterdir(), key=lambda item: item.name.lower())
        if path.suffix.lower() in VIDEO_EXTENSIONS
    ]

    if not videos:
        raise FileNotFoundError(f"No video files found in {video_dir}")
    if len(videos) > 1:
        names = ", ".join(path.name for path in videos)
        raise ValueError(f"Found multiple videos in {video_dir}: {names}")

    return videos[0]


def extract_frames(video_path: Path, output_dir: Path, seconds_between_frames: float) -> int:
    cv2 = load_cv2()
    capture = cv2.VideoCapture(str(video_path))

    if not capture.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    if fps <= 0:
        raise RuntimeError("Could not determine the video's FPS.")

    total_seconds = frame_count / fps if frame_count > 0 else 0
    output_dir.mkdir(parents=True, exist_ok=True)

    saved_count = 0
    timestamp = 0.0

    while timestamp <= total_seconds:
        capture.set(cv2.CAP_PROP_POS_MSEC, timestamp * 1000)
        success, frame = capture.read()

        if not success:
            break

        saved_count += 1
        output_path = output_dir / f"image_{saved_count}.jpg"

        if not cv2.imwrite(str(output_path), frame):
            raise RuntimeError(f"Could not write frame: {output_path}")

        timestamp += seconds_between_frames

    capture.release()
    return saved_count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract one frame per second from the video in test_vid."
    )
    parser.add_argument(
        "--video-dir",
        type=Path,
        default=Path("test_vid"),
        help="Directory containing one video file. Defaults to test_vid.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("test_frames"),
        help="Directory where extracted frames will be saved. Defaults to test_frames.",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=5.0,
        help="Seconds between extracted frames. Defaults to 1.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.seconds <= 0 or not math.isfinite(args.seconds):
        print("Error: --seconds must be a positive number.", file=sys.stderr)
        return 1

    try:
        video_path = find_video(args.video_dir)
        saved_count = extract_frames(video_path, args.output_dir, args.seconds)
    except Exception as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    print(f"Extracted {saved_count} frames from {video_path} into {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
