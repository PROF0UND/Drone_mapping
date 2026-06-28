import json
import tempfile
import unittest
from pathlib import Path

from rpifolder.telemetry_logger import (
    TelemetryState,
    image_filename,
    make_image_manifest_record,
    wait_for_required_devices,
)


class TelemetryLoggerTests(unittest.TestCase):
    def test_telemetry_snapshot_masks_unlocked_gps_coordinates(self) -> None:
        state = TelemetryState(latitude=12.34567891, longitude=-98.76543219, gps_locked=False)

        snapshot = state.snapshot()

        self.assertFalse(snapshot["gps_locked"])
        self.assertEqual(snapshot["gps_latitude"], 0.0)
        self.assertEqual(snapshot["gps_longitude"], 0.0)
        self.assertIn("timestamp", snapshot)

    def test_telemetry_snapshot_rounds_locked_values(self) -> None:
        state = TelemetryState(
            latitude=12.34567891,
            longitude=-98.76543219,
            gps_locked=True,
            heading_degrees=123.456,
            altitude_m=45.678,
        )

        snapshot = state.snapshot()

        self.assertEqual(snapshot["gps_latitude"], 12.3456789)
        self.assertEqual(snapshot["gps_longitude"], -98.7654322)
        self.assertEqual(snapshot["heading_degrees"], 123.46)
        self.assertEqual(snapshot["altitude_m"], 45.68)

    def test_image_filename_uses_six_digit_sequence(self) -> None:
        self.assertEqual(image_filename(1), "image_000001.jpg")
        self.assertEqual(image_filename(42), "image_000042.jpg")

    def test_manifest_record_shape_and_json_serializable(self) -> None:
        telemetry = {
            "timestamp": "2026-05-28T12:00:00.000+00:00",
            "gps_locked": True,
            "gps_latitude": 1.23,
            "gps_longitude": 4.56,
            "heading_degrees": 90.0,
            "altitude_m": 12.0,
        }

        record = make_image_manifest_record(
            sequence=7,
            image_path=Path("images/image_000007.jpg"),
            captured_at="2026-05-28T12:00:00.100+00:00",
            camera_index=0,
            width=640,
            height=480,
            telemetry=telemetry,
            capture_ok=True,
        )

        self.assertEqual(
            record,
            {
                "sequence": 7,
                "image_path": str(Path("images/image_000007.jpg")),
                "captured_at": "2026-05-28T12:00:00.100+00:00",
                "camera_index": 0,
                "width": 640,
                "height": 480,
                "telemetry": telemetry,
                "capture_ok": True,
            },
        )
        json.dumps(record)

    def test_manifest_record_includes_error_only_when_present(self) -> None:
        record = make_image_manifest_record(
            sequence=1,
            image_path=Path("images/image_000001.jpg"),
            captured_at="2026-05-28T12:00:00.100+00:00",
            camera_index=0,
            width=0,
            height=0,
            telemetry={},
            capture_ok=False,
            error="Camera frame capture failed",
        )

        self.assertEqual(record["error"], "Camera frame capture failed")

    def test_wait_for_required_devices_accepts_existing_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            device_path = Path(temp_dir) / "video0"
            device_path.touch()

            self.assertTrue(wait_for_required_devices([str(device_path)], 0.01))

    def test_wait_for_required_devices_rejects_missing_path(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            device_path = Path(temp_dir) / "missing"

            self.assertFalse(wait_for_required_devices([str(device_path)], 0.01))


if __name__ == "__main__":
    unittest.main()
