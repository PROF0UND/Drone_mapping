from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


def load_cv2():
    try:
        import cv2
    except ImportError:
        print(
            "OpenCV is required to visualize image matches. Install it with:\n"
            "  python -m pip install opencv-python",
            file=sys.stderr,
        )
        raise SystemExit(1)

    return cv2


def natural_key(path: Path) -> list[int | str]:
    parts = re.split(r"(\d+)", path.stem.lower())
    return [int(part) if part.isdigit() else part for part in parts]


def first_two_images(image_dir: Path) -> tuple[Path, Path]:
    if not image_dir.exists():
        raise FileNotFoundError(f"Image directory does not exist: {image_dir}")
    if not image_dir.is_dir():
        raise NotADirectoryError(f"Expected a directory: {image_dir}")

    images = sorted(
        [
            path
            for path in image_dir.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        ],
        key=natural_key,
    )

    if len(images) < 2:
        raise ValueError(f"Need at least 2 images in {image_dir}; found {len(images)}")

    return images[9], images[10]


def resize_for_matching(cv2, image, max_width: int):
    height, width = image.shape[:2]
    if width <= max_width:
        return image, 1.0

    scale = max_width / width
    resized = cv2.resize(image, (max_width, int(height * scale)), interpolation=cv2.INTER_AREA)
    return resized, scale


def draw_red_match_dots(cv2, image_a, image_b, keypoints_a, keypoints_b, matches):
    height = max(image_a.shape[0], image_b.shape[0])
    width = image_a.shape[1] + image_b.shape[1]
    canvas = cv2.copyMakeBorder(
        image_a,
        top=0,
        bottom=height - image_a.shape[0],
        left=0,
        right=image_b.shape[1],
        borderType=cv2.BORDER_CONSTANT,
        value=(255, 255, 255),
    )
    canvas[0 : image_b.shape[0], image_a.shape[1] : width] = image_b

    offset_x = image_a.shape[1]

    for match in matches:
        point_a = tuple(round(value) for value in keypoints_a[match.queryIdx].pt)
        point_b_raw = keypoints_b[match.trainIdx].pt
        point_b = (round(point_b_raw[0] + offset_x), round(point_b_raw[1]))

        cv2.circle(canvas, point_a, 8, (0, 0, 255), 2)
        cv2.circle(canvas, point_b, 8, (0, 0, 255), 2)
        cv2.line(canvas, point_a, point_b, (0, 0, 255), 1)

    return canvas


def visualize_matches(
    image_dir: Path,
    output: Path,
    max_matches: int,
    max_width: int,
) -> tuple[Path, int, Path, Path]:
    cv2 = load_cv2()
    image_a_path, image_b_path = first_two_images(image_dir)

    image_a = cv2.imread(str(image_a_path))
    image_b = cv2.imread(str(image_b_path))

    if image_a is None:
        raise ValueError(f"Could not read image: {image_a_path}")
    if image_b is None:
        raise ValueError(f"Could not read image: {image_b_path}")

    image_a, _ = resize_for_matching(cv2, image_a, max_width)
    image_b, _ = resize_for_matching(cv2, image_b, max_width)

    gray_a = cv2.cvtColor(image_a, cv2.COLOR_BGR2GRAY)
    gray_b = cv2.cvtColor(image_b, cv2.COLOR_BGR2GRAY)

    detector = cv2.ORB_create(nfeatures=3000)
    keypoints_a, descriptors_a = detector.detectAndCompute(gray_a, None)
    keypoints_b, descriptors_b = detector.detectAndCompute(gray_b, None)

    if descriptors_a is None or descriptors_b is None:
        raise RuntimeError("Could not find enough visual features in the first two images.")

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = matcher.match(descriptors_a, descriptors_b)
    matches = sorted(matches, key=lambda match: match.distance)[:max_matches]

    if not matches:
        raise RuntimeError("No matching features were found between the first two images.")

    visualization = draw_red_match_dots(
        cv2, image_a, image_b, keypoints_a, keypoints_b, matches
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), visualization):
        raise RuntimeError(f"Could not write output image: {output}")

    return output, len(matches), image_a_path, image_b_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Show matching feature points between the first two extracted frames."
    )
    parser.add_argument(
        "--image-dir",
        type=Path,
        default=Path("test_images"),
        help="Directory containing frame images. Defaults to test_frames.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("first_two_frame_matches.jpg"),
        help="Output visualization path. Defaults to first_two_frame_matches.jpg.",
    )
    parser.add_argument(
        "--max-matches",
        type=int,
        default=80,
        help="Maximum number of best matches to draw. Defaults to 80.",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=1200,
        help="Resize each image to this width before matching. Defaults to 1200.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.max_matches <= 0:
        print("Error: --max-matches must be positive.", file=sys.stderr)
        return 1
    if args.max_width <= 0:
        print("Error: --max-width must be positive.", file=sys.stderr)
        return 1

    try:
        output, match_count, image_a, image_b = visualize_matches(
            args.image_dir, args.output, args.max_matches, args.max_width
        )
    except Exception as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    print(f"Matched {match_count} points between {image_a.name} and {image_b.name}")
    print(f"Saved visualization to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
