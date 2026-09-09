from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from camera.ReconnaissanceObjet.ModuleAI import Detection, YoloDetector


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class _FakeBox:
    cls = np.array([0])
    conf = np.array([0.875])
    xyxy = np.array([[10.0, 20.0, 30.0, 60.0]])


class _FakeResult:
    names = {0: "hornet"}
    boxes = [_FakeBox()]


class _FakeModel:
    names = {0: "hornet"}

    def __init__(self) -> None:
        self.predict_arguments = None

    def predict(self, **arguments):
        self.predict_arguments = arguments
        return [_FakeResult()]


class TestYoloDetector(unittest.TestCase):
    def test_detection_returns_centroid_and_annotated_copy(self):
        model = _FakeModel()
        detector = YoloDetector(model=model, confidence=0.5, device="cpu")
        frame = np.zeros((80, 100, 3), dtype=np.uint8)

        annotated, detections = detector.detect(frame)

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].label, "hornet")
        self.assertEqual((detections[0].pixel_x, detections[0].pixel_y), (20, 40))
        self.assertFalse(np.shares_memory(annotated, frame))
        self.assertEqual(model.predict_arguments["device"], "cpu")


class TestGuiLoggingHelpers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from GUI_brouillon_vibecode import append_json_line, build_coordinate_records
        except ImportError as error:
            raise unittest.SkipTest(f"Qt GUI dependencies are not installed: {error}")
        cls.append_json_line = staticmethod(append_json_line)
        cls.build_coordinate_records = staticmethod(build_coordinate_records)

    def test_coordinate_record_contains_pixel_and_calibrated_values(self):
        detection = Detection("hornet", 0.9, 20, 30, (10, 10, 30, 50))

        records = self.build_coordinate_records(
            [detection],
            coefficients=(0.5, 0.25, 0.5, 0.25),
            origin=(10.0, 10.0),
            timestamp="2026-09-09T10:00:00+02:00",
        )

        self.assertEqual(records[0]["pixel"], {"x": 20, "y": 30})
        self.assertEqual(records[0]["calibrated"], {"x": 5.0, "y": 5.0, "unit": "cm"})

    def test_json_logger_appends_one_record_per_line(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            log_path = Path(temporary_directory) / "logs" / "coordinates.txt"
            self.append_json_line(log_path, {"x": 12, "y": 34})
            self.append_json_line(log_path, {"x": 56, "y": 78})

            records = [
                json.loads(line)
                for line in log_path.read_text(encoding="utf-8").splitlines()
            ]

        self.assertEqual(records, [{"x": 12, "y": 34}, {"x": 56, "y": 78}])


class _FakeCamera:
    frame = np.zeros((100, 100, 3), dtype=np.uint8)

    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.released = False

    def is_connected(self):
        return True

    def read(self):
        return True, self.frame.copy()

    def release(self):
        self.released = True


class TestGuiWindow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import GUI_brouillon_vibecode as gui
            from qtpy import QtWidgets
        except ImportError as error:
            raise unittest.SkipTest(f"Qt GUI dependencies are not installed: {error}")
        cls.gui = gui
        cls.QtWidgets = QtWidgets
        cls.application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_window_has_exactly_three_buttons_and_connection_placeholder(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            window = self.gui.HornetDeathStarWindow(log_dir=Path(temporary_directory))
            buttons = window.findChildren(self.QtWidgets.QPushButton)
            self.assertEqual(
                [button.text() for button in buttons],
                ["Test connections", "Calibrate", "Start acquisition"],
            )

            with patch.object(self.gui, "LaCamera", _FakeCamera):
                window.test_connections()

            self.assertIn("PASS", window.camera_status.text())
            self.assertEqual(window.arm_status.text(), "PASS (placeholder)")
            window.close()

    def test_calibration_capture_is_logged(self):
        calibration_frame = np.zeros((100, 100, 3), dtype=np.uint8)
        for point in ((10, 10), (90, 10), (90, 90), (10, 90)):
            cv2.circle(calibration_frame, point, 4, (0, 0, 255), -1)
        _FakeCamera.frame = calibration_frame

        with tempfile.TemporaryDirectory() as temporary_directory:
            log_dir = Path(temporary_directory)
            window = self.gui.HornetDeathStarWindow(
                log_dir=log_dir,
                calibration_distance_cm=80.0,
            )
            with patch.object(self.gui, "LaCamera", _FakeCamera):
                window.run_calibration()

            record = json.loads(
                (log_dir / "calibration_results.txt")
                .read_text(encoding="utf-8")
                .splitlines()[0]
            )
            self.assertTrue(record["success"])
            self.assertEqual(record["origin_pixel"], [10.0, 10.0])
            self.assertIsNotNone(window.calibration_coefficients)
            window.close()

    def test_coordinate_signal_is_always_emitted_but_file_logging_is_optional(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        detection = Detection("hornet", 0.9, 20, 30, (10, 10, 30, 50))

        with tempfile.TemporaryDirectory() as temporary_directory:
            log_dir = Path(temporary_directory)
            window = self.gui.HornetDeathStarWindow(log_dir=log_dir)
            received_records = []
            window.coordinate_ready.connect(received_records.append)

            window._handle_detection_frame(frame, [detection])
            self.assertEqual(received_records[0]["pixel"], {"x": 20, "y": 30})
            self.assertFalse((log_dir / "detection_coordinates.txt").exists())

            window.coordinate_log_toggle.setChecked(True)
            window._handle_detection_frame(frame, [detection])
            self.assertTrue((log_dir / "detection_coordinates.txt").is_file())
            window.close()


if __name__ == "__main__":
    unittest.main()
