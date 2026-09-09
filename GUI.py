"""QtPy control panel for camera checks, calibration, and YOLO acquisition."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np
from qtpy import QtCore, QtGui, QtWidgets

from Bras.ModuleDeDeplacement.PU_PositionInitiale import connecterRobot
from camera.ModuleCam.Calibrage import CoordenéesAuRobo, CoordonnerCalibrage
from camera.ModuleCam.ModuleCamera import LaCamera
from camera.ReconnaissanceObjet.ModuleAI import Detection, YoloDetector


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_LOG_DIR = PROJECT_ROOT / "logs"


def default_model_path() -> Path:
    """Return the newest project model, falling back to the tracked model."""

    candidates = (
        PROJECT_ROOT
        / "camera"
        / "IA"
        / "Model2"
        / "runs"
        / "detect"
        / "train"
        / "weights"
        / "best.pt",
        PROJECT_ROOT / "camera" / "IA" / "Model" / "weights" / "best.pt",
    )
    return next((candidate for candidate in candidates if candidate.is_file()), candidates[1])


def append_json_line(path: Path, payload: dict[str, object]) -> None:
    """Append a machine-readable record to a UTF-8 text file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as log_file:
        log_file.write(json.dumps(payload, ensure_ascii=False) + "\n")


def normalize_camera_indices(camera_indices: Iterable[int]) -> tuple[int, ...]:
    """Validate camera indices while preserving order and removing duplicates."""

    normalized = tuple(dict.fromkeys(int(index) for index in camera_indices))
    if not normalized:
        raise ValueError("At least one camera index is required")
    if any(index < 0 for index in normalized):
        raise ValueError("Camera indices must be zero or greater")
    return normalized


def build_coordinate_records(
    detections: Iterable[Detection],
    coefficients: Sequence[float] | None,
    origin: Sequence[float] | None,
    *,
    camera_index: int | None = None,
    timestamp: str | None = None,
) -> list[dict[str, object]]:
    """Build detection records for text logging or a future ROS publisher."""

    event_time = timestamp or datetime.now().astimezone().isoformat(timespec="milliseconds")
    records: list[dict[str, object]] = []

    for detection in detections:
        calibrated_coordinates = None
        if coefficients is not None and origin is not None:
            calibrated_x, calibrated_y = CoordenéesAuRobo(
                coefficients,
                origin,
                (detection.pixel_x, detection.pixel_y),
            )
            calibrated_coordinates = {
                "x": round(float(calibrated_x), 4),
                "y": round(float(calibrated_y), 4),
                "unit": "cm",
            }

        records.append(
            {
                "timestamp": event_time,
                "camera_index": camera_index,
                "label": detection.label,
                "confidence": round(detection.confidence, 4),
                "pixel": {"x": detection.pixel_x, "y": detection.pixel_y},
                "calibrated": calibrated_coordinates,
            }
        )

    return records


class AcquisitionWorker(QtCore.QThread):
    """Own all configured cameras and one shared YOLO model outside the GUI thread."""

    frame_ready = QtCore.Signal(int, object, object, str)
    status_changed = QtCore.Signal(str)
    acquisition_error = QtCore.Signal(str)

    def __init__(
        self,
        *,
        camera_indices: Sequence[int],
        model_path: Path,
        confidence: float,
        device: str | int | None,
        parent: QtCore.QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.camera_indices = normalize_camera_indices(camera_indices)
        self.model_path = model_path
        self.confidence = confidence
        self.device = device

    def stop(self) -> None:
        self.requestInterruption()

    def run(self) -> None:
        cameras: dict[int, LaCamera] = {}
        try:
            self.status_changed.emit(f"Loading YOLO model: {self.model_path.name}")
            detector = YoloDetector(
                self.model_path,
                confidence=self.confidence,
                device=self.device,
            )

            indices_text = ", ".join(map(str, self.camera_indices))
            self.status_changed.emit(f"Opening cameras {indices_text}")
            for camera_index in self.camera_indices:
                camera = LaCamera(camera_index=camera_index)
                cameras[camera_index] = camera
                if not camera.is_connected():
                    raise RuntimeError(f"Camera {camera_index} could not be opened")

            self.status_changed.emit("Live acquisition running")
            while not self.isInterruptionRequested():
                captured_frames: list[tuple[int, np.ndarray, str]] = []
                # Capture every camera before inference to limit inter-camera skew.
                for camera_index, camera in cameras.items():
                    success, frame = camera.read()
                    captured_at = datetime.now().astimezone().isoformat(
                        timespec="milliseconds"
                    )
                    if not success or frame is None:
                        raise RuntimeError(
                            f"Camera {camera_index} stopped returning frames"
                        )
                    captured_frames.append((camera_index, frame, captured_at))

                for camera_index, frame, captured_at in captured_frames:
                    if self.isInterruptionRequested():
                        break
                    annotated_frame, detections = detector.detect(frame)
                    self.frame_ready.emit(
                        camera_index,
                        annotated_frame,
                        detections,
                        captured_at,
                    )
        except Exception as error:  # Hardware/model errors must reach the UI.
            self.acquisition_error.emit(f"{type(error).__name__}: {error}")
        finally:
            for camera in cameras.values():
                camera.release()


class HornetDeathStarWindow(QtWidgets.QMainWindow):
    """Three-action prototype control panel."""

    # Connect this signal to a ROS bridge later. Each emission is one detection.
    coordinate_ready = QtCore.Signal(object)

    def __init__(
        self,
        *,
        camera_indices: Sequence[int] | None = None,
        camera_index: int | None = None,
        model_path: Path | None = None,
        log_dir: Path | None = None,
        calibration_distance_cm: float = 75.0,
        confidence: float = 0.5,
        device: str | int | None = None,
    ) -> None:
        super().__init__()
        if camera_indices is not None and camera_index is not None:
            raise ValueError("Use camera_indices or camera_index, not both")
        selected_indices = (
            camera_indices
            if camera_indices is not None
            else ((camera_index,) if camera_index is not None else (0, 1))
        )
        self.camera_indices = normalize_camera_indices(selected_indices)
        # Kept as a convenience for code that treats the first camera as primary.
        self.camera_index = self.camera_indices[0]
        self.model_path = (model_path or default_model_path()).expanduser().resolve()
        self.log_dir = (log_dir or DEFAULT_LOG_DIR).expanduser().resolve()
        self.calibration_distance_cm = calibration_distance_cm
        self.confidence = confidence
        self.device = device

        self.calibration_coefficients: dict[int, tuple[float, ...]] = {}
        self.calibration_origins: dict[int, tuple[float, ...]] = {}
        self._worker: AcquisitionWorker | None = None
        self._worker_failed = False
        self._last_images: dict[int, QtGui.QImage] = {}

        self._build_interface()
        self._show_startup_state()

    def _build_interface(self) -> None:
        self.setWindowTitle("Hornet Death Star - Control Panel")
        self.resize(1250, 800)

        central_widget = QtWidgets.QWidget(self)
        root_layout = QtWidgets.QVBoxLayout(central_widget)
        root_layout.setContentsMargins(20, 20, 20, 20)
        root_layout.setSpacing(14)

        title = QtWidgets.QLabel("Hornet Death Star")
        title.setObjectName("title")
        camera_names = ", ".join(f"camera {index}" for index in self.camera_indices)
        subtitle = QtWidgets.QLabel(
            f"Calibration and YOLO acquisition prototype - {camera_names}"
        )
        subtitle.setObjectName("subtitle")
        root_layout.addWidget(title)
        root_layout.addWidget(subtitle)

        status_group = QtWidgets.QGroupBox("System status")
        status_layout = QtWidgets.QFormLayout(status_group)
        self.camera_statuses: dict[int, QtWidgets.QLabel] = {}
        for camera_index in self.camera_indices:
            status = QtWidgets.QLabel("Not tested")
            self.camera_statuses[camera_index] = status
            status_layout.addRow(f"Camera {camera_index}:", status)
        self.arm_status = QtWidgets.QLabel("Not tested")
        self.calibration_statuses: dict[int, QtWidgets.QLabel] = {}
        for camera_index in self.camera_indices:
            status = QtWidgets.QLabel("Not calibrated")
            self.calibration_statuses[camera_index] = status
            status_layout.addRow(f"Calibration {camera_index}:", status)
        self.acquisition_status = QtWidgets.QLabel("Stopped")
        status_layout.addRow("Robot arm:", self.arm_status)
        status_layout.addRow("Acquisition:", self.acquisition_status)
        # Compatibility aliases point at the primary camera's widgets.
        self.camera_status = self.camera_statuses[self.camera_index]
        self.calibration_status = self.calibration_statuses[self.camera_index]
        root_layout.addWidget(status_group)

        button_layout = QtWidgets.QHBoxLayout()
        self.connection_button = QtWidgets.QPushButton("Test connections")
        self.calibration_button = QtWidgets.QPushButton("Calibrate")
        self.acquisition_button = QtWidgets.QPushButton("Start acquisition")
        button_layout.addWidget(self.connection_button)
        button_layout.addWidget(self.calibration_button)
        button_layout.addWidget(self.acquisition_button)
        root_layout.addLayout(button_layout)

        options_layout = QtWidgets.QHBoxLayout()
        self.coordinate_log_toggle = QtWidgets.QCheckBox(
            "Log detection coordinates (x, y)"
        )
        options_layout.addWidget(self.coordinate_log_toggle)
        options_layout.addStretch()
        options_layout.addWidget(QtWidgets.QLabel(f"Logs: {self.log_dir}"))
        root_layout.addLayout(options_layout)

        video_layout = QtWidgets.QGridLayout()
        self.video_labels: dict[int, QtWidgets.QLabel] = {}
        self.detection_summaries: dict[int, QtWidgets.QLabel] = {}
        for position, camera_index in enumerate(self.camera_indices):
            camera_group = QtWidgets.QGroupBox(f"Camera {camera_index}")
            camera_layout = QtWidgets.QVBoxLayout(camera_group)

            video_label = QtWidgets.QLabel("Live acquisition will appear here")
            video_label.setObjectName("videoFrame")
            video_label.setAlignment(QtCore.Qt.AlignCenter)
            video_label.setMinimumSize(420, 260)
            camera_layout.addWidget(video_label, stretch=1)

            detection_summary = QtWidgets.QLabel("No detections")
            camera_layout.addWidget(detection_summary)
            self.video_labels[camera_index] = video_label
            self.detection_summaries[camera_index] = detection_summary
            video_layout.addWidget(camera_group, position // 2, position % 2)

        self.video_label = self.video_labels[self.camera_index]
        self.detection_summary = self.detection_summaries[self.camera_index]
        root_layout.addLayout(video_layout, stretch=1)

        self.setCentralWidget(central_widget)
        self.statusBar().showMessage("Ready")

        self.connection_button.clicked.connect(self.test_connections)
        self.calibration_button.clicked.connect(self.run_calibration)
        self.acquisition_button.clicked.connect(self.toggle_acquisition)

        self.setStyleSheet(
            """
            QMainWindow { background: #f3f5f7; }
            QLabel#title { font-size: 26px; font-weight: 700; color: #18212b; }
            QLabel#subtitle { color: #596675; margin-bottom: 6px; }
            QGroupBox {
                background: white;
                border: 1px solid #d9dfe6;
                border-radius: 8px;
                margin-top: 8px;
                padding: 10px;
                font-weight: 600;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton {
                min-height: 38px;
                border: 0;
                border-radius: 6px;
                background: #265d97;
                color: white;
                font-weight: 600;
                padding: 0 18px;
            }
            QPushButton:hover { background: #1d4f83; }
            QPushButton:disabled { background: #9aa8b6; }
            QLabel#videoFrame {
                background: #111820;
                color: #aab5c0;
                border: 1px solid #2b3743;
                border-radius: 8px;
            }
            QCheckBox {color: black;}
            """
        )

    def _show_startup_state(self) -> None:
        if self.model_path.is_file():
            self.statusBar().showMessage(f"Model: {self.model_path}")
        else:
            self._set_indicator(self.acquisition_status, "Model file missing", "error")
            self.statusBar().showMessage(f"YOLO model not found: {self.model_path}")

    @staticmethod
    def _set_indicator(label: QtWidgets.QLabel, text: str, state: str) -> None:
        colors = {"ok": "#16794a", "warning": "#986f00", "error": "#b42318"}
        label.setText(text)
        label.setStyleSheet(f"color: {colors.get(state, '#596675')}; font-weight: 600;")

    @QtCore.Slot()
    def test_connections(self) -> None:
        """Test every camera and the robot arm, then close all connections."""

        for status in self.camera_statuses.values():
            self._set_indicator(status, "Testing...", "warning")
        self._set_indicator(self.arm_status, "Testing...", "warning")
        QtWidgets.QApplication.processEvents()

        for camera_index in self.camera_indices:
            camera: LaCamera | None = None
            status = self.camera_statuses[camera_index]
            try:
                camera = LaCamera(camera_index=camera_index)
                if camera.is_connected():
                    self._set_indicator(status, "PASS", "ok")
                else:
                    self._set_indicator(status, "FAIL (could not open)", "error")
            except Exception as error:
                self._set_indicator(status, f"FAIL: {error}", "error")
            finally:
                if camera is not None:
                    camera.release()

        robot = None
        try:
            robot = connecterRobot()
            if robot is None:
                self._set_indicator(self.arm_status, "FAIL (connection refused)", "error")
            else:
                self._set_indicator(self.arm_status, "PASS", "ok")
        except Exception as error:
            self._set_indicator(self.arm_status, f"FAIL: {error}", "error")
        finally:
            if robot is not None:
                try:
                    robot.close()
                except Exception as error:
                    self._set_indicator(
                        self.arm_status,
                        f"FAIL (disconnect error: {error})",
                        "error",
                    )

        self.statusBar().showMessage("Connection test finished", 5000)

    @QtCore.Slot()
    def run_calibration(self) -> None:
        """Capture and calibrate a settled frame from every configured camera."""

        self.calibration_button.setEnabled(False)
        for status in self.calibration_statuses.values():
            self._set_indicator(status, "Waiting...", "warning")
        QtWidgets.QApplication.processEvents()

        successful_calibrations = 0
        for camera_index in self.camera_indices:
            status = self.calibration_statuses[camera_index]
            self._set_indicator(status, "Capturing image...", "warning")
            QtWidgets.QApplication.processEvents()
            if self._calibrate_camera(camera_index):
                successful_calibrations += 1

        self.calibration_button.setEnabled(True)
        self.statusBar().showMessage(
            f"Calibration finished: {successful_calibrations}/"
            f"{len(self.camera_indices)} cameras passed",
            8000,
        )

    def _calibrate_camera(self, camera_index: int) -> bool:
        """Calibrate one camera and append its result to the shared log."""

        event_time = datetime.now().astimezone()
        timestamp = event_time.isoformat(timespec="milliseconds")
        filename_time = event_time.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        image_path = (
            self.log_dir / f"calibration_camera_{camera_index}_{filename_time}.jpg"
        )
        log_path = self.log_dir / "calibration_results.txt"
        camera: LaCamera | None = None
        frame: np.ndarray | None = None
        status = self.calibration_statuses[camera_index]

        try:
            camera = LaCamera(camera_index=camera_index)
            if not camera.is_connected():
                raise RuntimeError(f"Camera {camera_index} could not be opened")

            # Discard initial frames so exposure and white balance can settle.
            for _ in range(8):
                success, candidate = camera.read()
                if success and candidate is not None:
                    frame = candidate
            if frame is None:
                raise RuntimeError("No calibration frame was captured")

            self.log_dir.mkdir(parents=True, exist_ok=True)
            image_saved = cv2.imwrite(str(image_path), frame)
            coefficients, origin = CoordonnerCalibrage(
                frame, self.calibration_distance_cm
            )

            success = coefficients is not None and origin is not None
            record: dict[str, object] = {
                "timestamp": timestamp,
                "success": success,
                "camera_index": camera_index,
                "reference_distance_cm": self.calibration_distance_cm,
                "image": str(image_path) if image_saved else None,
                "coefficients": coefficients,
                "origin_pixel": origin,
            }
            if not success:
                record["error"] = "Four red calibration corners were not detected"
            append_json_line(log_path, record)

            self._show_frame(camera_index, frame)
            if success:
                self.calibration_coefficients[camera_index] = tuple(coefficients)
                self.calibration_origins[camera_index] = tuple(origin)
                self._set_indicator(
                    status,
                    f"PASS - origin ({origin[0]:.0f}, {origin[1]:.0f})",
                    "ok",
                )
                return True
            else:
                self.calibration_coefficients.pop(camera_index, None)
                self.calibration_origins.pop(camera_index, None)
                self._set_indicator(
                    status, "FAIL - four red corners not found", "error"
                )
                return False
        except Exception as error:
            self.calibration_coefficients.pop(camera_index, None)
            self.calibration_origins.pop(camera_index, None)
            self._set_indicator(status, f"FAIL: {error}", "error")
            try:
                append_json_line(
                    log_path,
                    {
                        "timestamp": timestamp,
                        "success": False,
                        "camera_index": camera_index,
                        "error": str(error),
                    },
                )
            except OSError:
                pass
            return False
        finally:
            if camera is not None:
                camera.release()

    @QtCore.Slot()
    def toggle_acquisition(self) -> None:
        if self._worker is None:
            self.start_acquisition()
        else:
            self.stop_acquisition()

    def start_acquisition(self) -> None:
        if not self.model_path.is_file():
            self._set_indicator(self.acquisition_status, "YOLO model missing", "error")
            self.statusBar().showMessage(f"Model not found: {self.model_path}", 8000)
            return

        self._worker_failed = False
        self._worker = AcquisitionWorker(
            camera_indices=self.camera_indices,
            model_path=self.model_path,
            confidence=self.confidence,
            device=self.device,
            parent=self,
        )
        self._worker.frame_ready.connect(self._handle_detection_frame)
        self._worker.status_changed.connect(self._handle_worker_status)
        self._worker.acquisition_error.connect(self._handle_worker_error)
        self._worker.finished.connect(self._handle_worker_finished)

        self.connection_button.setEnabled(False)
        self.calibration_button.setEnabled(False)
        self.acquisition_button.setText("Stop acquisition")
        self._set_indicator(self.acquisition_status, "Starting...", "warning")
        self._worker.start()

    def stop_acquisition(self) -> None:
        if self._worker is None:
            return
        self._set_indicator(self.acquisition_status, "Stopping...", "warning")
        self.acquisition_button.setEnabled(False)
        self._worker.stop()

    @QtCore.Slot(str)
    def _handle_worker_status(self, message: str) -> None:
        state = "ok" if message == "Live acquisition running" else "warning"
        self._set_indicator(self.acquisition_status, message, state)
        self.statusBar().showMessage(message)

    @QtCore.Slot(str)
    def _handle_worker_error(self, message: str) -> None:
        self._worker_failed = True
        self._set_indicator(self.acquisition_status, message, "error")
        self.statusBar().showMessage(message, 10000)

    @QtCore.Slot()
    def _handle_worker_finished(self) -> None:
        if not self._worker_failed:
            self._set_indicator(self.acquisition_status, "Stopped", "warning")
            self.statusBar().showMessage("Acquisition stopped", 5000)

        worker = self._worker
        self._worker = None
        self.connection_button.setEnabled(True)
        self.calibration_button.setEnabled(True)
        self.acquisition_button.setEnabled(True)
        self.acquisition_button.setText("Start acquisition")
        if worker is not None:
            worker.deleteLater()

    @QtCore.Slot(int, object, object, str)
    def _handle_detection_frame(
        self,
        camera_index: int,
        frame: np.ndarray,
        detections: list[Detection],
        captured_at: str,
    ) -> None:
        self._show_frame(camera_index, frame)
        detection_summary = self.detection_summaries[camera_index]
        if detections:
            summary = ", ".join(
                f"{detection.label} ({detection.confidence:.2f})"
                for detection in detections
            )
            detection_summary.setText(f"Detections: {summary}")
        else:
            detection_summary.setText("No detections")

        records = build_coordinate_records(
            detections,
            self.calibration_coefficients.get(camera_index),
            self.calibration_origins.get(camera_index),
            camera_index=camera_index,
            timestamp=captured_at,
        )
        for record in records:
            # This works as the hand-off boundary for a later ROS publisher.
            self.coordinate_ready.emit(record)

        if self.coordinate_log_toggle.isChecked():
            coordinate_log = self.log_dir / "detection_coordinates.txt"
            try:
                for record in records:
                    append_json_line(coordinate_log, record)
            except OSError as error:
                self.coordinate_log_toggle.setChecked(False)
                self.statusBar().showMessage(f"Coordinate logging stopped: {error}", 10000)

    def _show_frame(self, camera_index: int, frame: np.ndarray) -> None:
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_frame.shape
        bytes_per_line = channels * width
        self._last_images[camera_index] = QtGui.QImage(
            rgb_frame.data,
            width,
            height,
            bytes_per_line,
            QtGui.QImage.Format_RGB888,
        ).copy()
        self._refresh_pixmap(camera_index)

    def _refresh_pixmap(self, camera_index: int) -> None:
        image = self._last_images.get(camera_index)
        if image is None:
            return
        video_label = self.video_labels[camera_index]
        pixmap = QtGui.QPixmap.fromImage(image)
        video_label.setPixmap(
            pixmap.scaled(
                video_label.size(),
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        for camera_index in self.camera_indices:
            self._refresh_pixmap(camera_index)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self._worker is not None and self._worker.isRunning():
            self._worker.stop()
            if not self._worker.wait(5000):
                self.statusBar().showMessage("Waiting for acquisition to stop...")
                event.ignore()
                return
        event.accept()


def parse_arguments(arguments: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    camera_group = parser.add_mutually_exclusive_group()
    camera_group.add_argument(
        "--camera-indices",
        nargs="+",
        type=int,
        metavar="INDEX",
        help="camera indices to use (default: 0 1)",
    )
    camera_group.add_argument(
        "--camera-index",
        type=int,
        help="single-camera compatibility option",
    )
    parser.add_argument("--model", type=Path, default=default_model_path())
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--distance-cm", type=float, default=75.0)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument(
        "--device",
        default=None,
        help='Ultralytics device, for example "0" for the first GPU or "cpu"',
    )
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] | None = None) -> int:
    args = parse_arguments(arguments)
    camera_indices = (
        args.camera_indices
        if args.camera_indices is not None
        else ([args.camera_index] if args.camera_index is not None else [0, 1])
    )
    application = QtWidgets.QApplication([sys.argv[0]])
    window = HornetDeathStarWindow(
        camera_indices=camera_indices,
        model_path=args.model,
        log_dir=args.log_dir,
        calibration_distance_cm=args.distance_cm,
        confidence=args.confidence,
        device=args.device,
    )
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
