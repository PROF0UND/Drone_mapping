# Mapping Project

This repository contains the Raspberry Pi logging tools and offline image stitching scripts for the mapping workflow.

The current flow is:

1. Use the Raspberry Pi to collect MAVLink telemetry from the flight controller.
2. Optionally capture USB camera images during the same run.
3. Save each session into a timestamped log folder.
4. Use the stitching tools to extract frames, visualize matches, and create stitched outputs.

## Directory Layout

```text
mapping/
+-- Data/        # Archived datasets, sample images/videos, and external reference code
+-- Stitching/   # Desktop/offline image extraction, matching, and stitching scripts
+-- rpifolder/   # Raspberry Pi telemetry and camera logging scripts
```

## Raspberry Pi Logging

The Pi scripts live in `rpifolder/`.

### Camera Test

Use this first to confirm the USB camera is working:

```bash
cd ~/mapping/rpifolder
python3 camera_test.py --output test.jpg
```

If the default camera does not open, list and probe available camera devices:

```bash
python3 camera_test.py --list-cameras
python3 camera_test.py --probe-cameras
```

Then target a specific device:

```bash
python3 camera_test.py --camera-device /dev/video0 --output test.jpg
```

### Telemetry Only

Run the telemetry logger without image capture:

```bash
cd ~/mapping/rpifolder
python3 telemetry_logger.py
```

By default, it reads MAVLink from `/dev/serial0` at `921600` baud, forwards telemetry to `udp:127.0.0.1:14550`, and saves logs under:

```text
~/flight_logs/telemetry_YYYYMMDD_HHMMSS/
```

Stop the logger with `Ctrl+C`.

### Telemetry With Images

Run telemetry logging and capture one image per second:

```bash
cd ~/mapping/rpifolder
python3 telemetry_logger.py --capture-images --camera-device /dev/video0 --capture-interval 1
```

Useful options:

```bash
python3 telemetry_logger.py --capture-images --capture-interval 5
python3 telemetry_logger.py --capture-images --camera-index 1
python3 telemetry_logger.py --capture-images --camera-device /dev/video0
python3 telemetry_logger.py --capture-images --image-width 1280 --image-height 720
```

Each session can contain:

```text
mavproxy_console.log     # MAVProxy stdout/stderr
telemetry_json.log       # Periodic telemetry JSON lines
session_info.txt         # Run settings and MAVProxy command
images/                  # Captured camera images
image_manifest.jsonl     # One JSON object per captured image
image_capture.log        # Camera capture status/errors
```

Each image manifest row includes the image path, capture timestamp, camera index, image size, capture status, and the latest telemetry snapshot.

### Start Automatically On Boot

Install the included `systemd` service so the Pi starts logging as soon as it powers on:

```bash
cd ~/mapping/rpifolder
sudo bash install_telemetry_service.sh
sudo systemctl start telemetry-logger.service
```

The service runs this command:

```bash
python3 /home/profound/mapping/rpifolder/telemetry_logger.py --capture-images --camera-device /dev/video0 --capture-interval 1 --restart
```

Check whether it is running:

```bash
systemctl status telemetry-logger.service
```

Watch live service logs:

```bash
journalctl -u telemetry-logger.service -f
```

Stop it manually:

```bash
sudo systemctl stop telemetry-logger.service
```

Disable automatic startup:

```bash
sudo systemctl disable telemetry-logger.service
```

## Stitching Workflow

The stitching scripts live in `Stitching/`. Run them from the project root unless you pass custom paths.

### Extract Frames From a Video

Place a video in a folder, then extract frames every N seconds:

```bash
python Stitching/extract_test_frames.py --video-dir Data/test_vid --output-dir Data/test_frames --seconds 5
```

### Visualize Feature Matches

Create an image showing matched feature points between two nearby frames:

```bash
python Stitching/visualize_frame_matches.py --image-dir Data/test_images --output Data/first_two_frame_matches.jpg
```

### Stitch Images

Create a stitched output from a directory of overlapping images:

```bash
python Stitching/test_stitching.py --image-dir Data/test_images --output Data/stitched_output.png --mode scans
```

Use `--mode panorama` for normal photo panoramas and `--mode scans` for flatter map/screenshot-like image sets.

## Dependencies

On the Raspberry Pi:

```bash
sudo apt update
sudo apt install python3-opencv
python3 -m pip install pymavlink MAVProxy
```

On a desktop machine for stitching:

```bash
python -m pip install opencv-python
```

## Tests

Run the telemetry helper tests:

```bash
python -m unittest rpifolder/test_telemetry_logger.py
```

Run a syntax check:

```bash
python -B -m py_compile rpifolder/telemetry_logger.py rpifolder/camera_test.py
```

## Notes

- Use a known-good USB cable for the external camera. A bad cable can make the camera appear as missing or unreadable.
- Keep large generated logs, images, and stitched outputs out of git unless they are intentionally part of the dataset.
- `Ctrl+C` is the normal way to stop telemetry and image capture.
