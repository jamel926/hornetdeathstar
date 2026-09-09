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


def build_coordinate_records(
    detections: Iterable[Detection],
    coefficients: Sequence[float] | None,
    origin: Sequence[float] | None,
    *,
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
                "label": detection.label,
                "confidence": round(detection.confidence, 4),
                "pixel": {"x": detection.pixel_x, "y": detection.pixel_y},
                "calibrated": calibrated_coordinates,
            }
        )

    return records


class AcquisitionWorker(QtCore.QThread):
    """Own the camera and YOLO model outside the GUI thread."""

    frame_ready = QtCore.Signal(object, object)
    status_changed = QtCore.Signal(str)
    acquisition_error = QtCore.Signal(str)

    def __init__(
        self,
        *,
        camera_index: int,
        model_path: Path,
        confidence: float,
        device: str | int | None,
        parent: QtCore.QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.camera_index = camera_index
        self.model_path = model_path
        self.confidence = confidence
        self.device = device

    def stop(self) -> None:
        self.requestInterruption()

    def run(self) -> None:
        camera: LaCamera | None = None
        try:
            self.status_changed.emit(f"Loading YOLO model: {self.model_path.name}")
            detector = YoloDetector(
                self.model_path,
                confidence=self.confidence,
                device=self.device,
            )

            self.status_changed.emit(f"Opening camera {self.camera_index}")
            camera = LaCamera(camera_index=self.camera_index)
            if not camera.is_connected():
                raise RuntimeError(f"Camera {self.camera_index} could not be opened")

            self.status_changed.emit("Live acquisition running")
            while not self.isInterruptionRequested():
                success, frame = camera.read()
                if not success or frame is None:
                    raise RuntimeError("The camera stopped returning frames")

                annotated_frame, detections = detector.detect(frame)
                self.frame_ready.emit(annotated_frame, detections)
        except Exception as error:  # Hardware/model errors must reach the UI.
            self.acquisition_error.emit(f"{type(error).__name__}: {error}")
        finally:
            if camera is not None:
                camera.release()


class HornetDeathStarWindow(QtWidgets.QMainWindow):
    """Three-action prototype control panel."""

    # Connect this signal to a ROS bridge later. Each emission is one detection.
    coordinate_ready = QtCore.Signal(object)

    def __init__(
        self,
        *,
        camera_index: int = 1,
        model_path: Path | None = None,
        log_dir: Path | None = None,
        calibration_distance_cm: float = 75.0,
        confidence: float = 0.5,
        device: str | int | None = None,
    ) -> None:
        super().__init__()
        self.camera_index = camera_index
        self.model_path = (model_path or default_model_path()).expanduser().resolve()
        self.log_dir = (log_dir or DEFAULT_LOG_DIR).expanduser().resolve()
        self.calibration_distance_cm = calibration_distance_cm
        self.confidence = confidence
        self.device = device

        self.calibration_coefficients: tuple[float, ...] | None = None
        self.calibration_origin: tuple[float, ...] | None = None
        self._worker: AcquisitionWorker | None = None
        self._worker_failed = False
        self._last_image: QtGui.QImage | None = None

        self._build_interface()
        self._show_startup_state()

    def _build_interface(self) -> None:
        self.setWindowTitle("Hornet Death Star - Control Panel")
        self.resize(1050, 760)

        central_widget = QtWidgets.QWidget(self)
        root_layout = QtWidgets.QVBoxLayout(central_widget)
        root_layout.setContentsMargins(20, 20, 20, 20)
        root_layout.setSpacing(14)

        title = QtWidgets.QLabel("Hornet Death Star")
        title.setObjectName("title")
        subtitle = QtWidgets.QLabel("Camera calibration and YOLO acquisition prototype")
        subtitle.setObjectName("subtitle")
        root_layout.addWidget(title)
        root_layout.addWidget(subtitle)

        status_group = QtWidgets.QGroupBox("System status")
        status_layout = QtWidgets.QFormLayout(status_group)
        self.camera_status = QtWidgets.QLabel("Not tested")
        self.arm_status = QtWidgets.QLabel("Not tested")
        self.calibration_status = QtWidgets.QLabel("Not calibrated")
        self.acquisition_status = QtWidgets.QLabel("Stopped")
        status_layout.addRow("Camera:", self.camera_status)
        status_layout.addRow("Robot arm:", self.arm_status)
        status_layout.addRow("Calibration:", self.calibration_status)
        status_layout.addRow("Acquisition:", self.acquisition_status)
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

        self.video_label = QtWidgets.QLabel("Live acquisition will appear here")
        self.video_label.setObjectName("videoFrame")
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(720, 400)
        root_layout.addWidget(self.video_label, stretch=1)

        self.detection_summary = QtWidgets.QLabel("No detections")
        root_layout.addWidget(self.detection_summary)

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
        """Test the camera and robot arm, then close both connections."""

        self._set_indicator(self.camera_status, "Testing...", "warning")
        self._set_indicator(self.arm_status, "Testing...", "warning")
        QtWidgets.QApplication.processEvents()

        camera: LaCamera | None = None
        try:
            camera = LaCamera(camera_index=self.camera_index)
            if camera.is_connected():
                self._set_indicator(
                    self.camera_status, f"PASS (camera {self.camera_index})", "ok"
                )
            else:
                self._set_indicator(
                    self.camera_status, f"FAIL (camera {self.camera_index})", "error"
                )
        except Exception as error:
            self._set_indicator(self.camera_status, f"FAIL: {error}", "error")
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
        """Capture one settled frame, calibrate it, and append the result to a log."""

        self.calibration_button.setEnabled(False)
        self._set_indicator(self.calibration_status, "Capturing image...", "warning")
        QtWidgets.QApplication.processEvents()

        event_time = datetime.now().astimezone()
        timestamp = event_time.isoformat(timespec="milliseconds")
        filename_time = event_time.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        image_path = self.log_dir / f"calibration_{filename_time}.jpg"
        log_path = self.log_dir / "calibration_results.txt"
        camera: LaCamera | None = None
        frame: np.ndarray | None = None

        try:
            camera = LaCamera(camera_index=self.camera_index)
            if not camera.is_connected():
                raise RuntimeError(f"Camera {self.camera_index} could not be opened")

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
                "camera_index": self.camera_index,
                "reference_distance_cm": self.calibration_distance_cm,
                "image": str(image_path) if image_saved else None,
                "coefficients": coefficients,
                "origin_pixel": origin,
            }
            if not success:
                record["error"] = "Four red calibration corners were not detected"
            append_json_line(log_path, record)

            self._show_frame(frame)
            if success:
                self.calibration_coefficients = tuple(coefficients)
                self.calibration_origin = tuple(origin)
                self._set_indicator(
                    self.calibration_status,
                    f"PASS - origin ({origin[0]:.0f}, {origin[1]:.0f})",
                    "ok",
                )
                self.statusBar().showMessage(f"Calibration logged to {log_path}", 8000)
            else:
                self.calibration_coefficients = None
                self.calibration_origin = None
                self._set_indicator(
                    self.calibration_status, "FAIL - four red corners not found", "error"
                )
                self.statusBar().showMessage(f"Calibration failure logged to {log_path}", 8000)
        except Exception as error:
            self.calibration_coefficients = None
            self.calibration_origin = None
            self._set_indicator(self.calibration_status, f"FAIL: {error}", "error")
            try:
                append_json_line(
                    log_path,
                    {
                        "timestamp": timestamp,
                        "success": False,
                        "camera_index": self.camera_index,
                        "error": str(error),
                    },
                )
            except OSError:
                pass
            self.statusBar().showMessage(str(error), 8000)
        finally:
            if camera is not None:
                camera.release()
            self.calibration_button.setEnabled(True)

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
            camera_index=self.camera_index,
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

    @QtCore.Slot(object, object)
    def _handle_detection_frame(
        self, frame: np.ndarray, detections: list[Detection]
    ) -> None:
        self._show_frame(frame)
        if detections:
            summary = ", ".join(
                f"{detection.label} ({detection.confidence:.2f})"
                for detection in detections
            )
            self.detection_summary.setText(f"Detections: {summary}")
        else:
            self.detection_summary.setText("No detections")

        records = build_coordinate_records(
            detections,
            self.calibration_coefficients,
            self.calibration_origin,
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

    def _show_frame(self, frame: np.ndarray) -> None:
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_frame.shape
        bytes_per_line = channels * width
        self._last_image = QtGui.QImage(
            rgb_frame.data,
            width,
            height,
            bytes_per_line,
            QtGui.QImage.Format_RGB888,
        ).copy()
        self._refresh_pixmap()

    def _refresh_pixmap(self) -> None:
        if self._last_image is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self._last_image)
        self.video_label.setPixmap(
            pixmap.scaled(
                self.video_label.size(),
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._refresh_pixmap()

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
    parser.add_argument("--camera-index", type=int, default=1)
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
    application = QtWidgets.QApplication([sys.argv[0]])
    window = HornetDeathStarWindow(
        camera_index=args.camera_index,
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
