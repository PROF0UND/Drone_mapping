from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


def natural_key(path: Path) -> list[int | str]:
    parts = re.split(r"(\d+)", path.stem.lower())
    return [int(part) if part.isdigit() else part for part in parts]


def load_cv2():
    try:
        import cv2
    except ImportError:
        print(
            "OpenCV is required for image stitching. Install it with:\n"
            "  python -m pip install opencv-python",
            file=sys.stderr,
        )
        raise SystemExit(1)

    return cv2


def image_paths(image_dir: Path) -> list[Path]:
    if not image_dir.exists():
        raise FileNotFoundError(f"Image directory does not exist: {image_dir}")
    if not image_dir.is_dir():
        raise NotADirectoryError(f"Expected a directory: {image_dir}")

    paths = [
        path
        for path in sorted(image_dir.iterdir(), key=natural_key)
        if path.suffix.lower() in IMAGE_EXTENSIONS
    ]

    if len(paths) < 2:
        raise ValueError(f"Need at least 2 images in {image_dir}; found {len(paths)}")

    return paths


def read_images(cv2, paths: list[Path]):
    images = []

    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"Could not read image: {path}")
        images.append(image)

    return images


def create_stitcher(cv2, mode: str):
    if not hasattr(cv2, "Stitcher_create"):
        raise RuntimeError(
            "This OpenCV build does not include cv2.Stitcher_create(). "
            "Try installing opencv-python."
        )

    stitcher_mode = cv2.Stitcher_PANORAMA
    if mode == "scans":
        stitcher_mode = cv2.Stitcher_SCANS

    return cv2.Stitcher_create(stitcher_mode)


def explain_status(cv2, status: int) -> str:
    status_names = {
        getattr(cv2, "Stitcher_OK", 0): "OK",
        getattr(cv2, "Stitcher_ERR_NEED_MORE_IMGS", 1): "Need more matching images",
        getattr(cv2, "Stitcher_ERR_HOMOGRAPHY_EST_FAIL", 2): "Could not align images",
        getattr(cv2, "Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL", 3): (
            "Could not adjust camera parameters"
        ),
    }

    return status_names.get(status, f"Unknown stitcher status: {status}")


def stitch(image_dir: Path, output: Path, mode: str) -> Path:
    cv2 = load_cv2()
    paths = image_paths(image_dir)
    images = read_images(cv2, paths)

    print(f"Loaded {len(images)} images from {image_dir}")
    stitcher = create_stitcher(cv2, mode)
    try:
        status, stitched = stitcher.stitch(images)
    except cv2.error as error:
        raise RuntimeError(
            "OpenCV crashed while matching image features. This often happens when "
            "some images have too few visual details or overlap. Try stitching the "
            "original drone frames instead of processed/label images, or use a "
            "smaller consecutive subset."
        ) from error

    if status != cv2.Stitcher_OK:
        raise RuntimeError(
            f"Stitching failed: {explain_status(cv2, status)}.\n"
            "Try using --mode scans for screenshots or make sure adjacent images overlap."
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), stitched):
        raise RuntimeError(f"Could not write output image: {output}")

    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stitch images from test_images into one output image."
    )
    parser.add_argument(
        "--image-dir",
        type=Path,
        default=Path("test_images"),
        help="Directory containing images to stitch. Defaults to test_images.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("stitched_output.png"),
        help="Where to save the stitched image. Defaults to stitched_output.png.",
    )
    parser.add_argument(
        "--mode",
        choices=("panorama", "scans"),
        default="scans",
        help="OpenCV stitching mode. Use scans for screenshots/maps; panorama for photos.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        output = stitch(args.image_dir, args.output, args.mode)
    except Exception as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    print(f"Saved stitched image to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
