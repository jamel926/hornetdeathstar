"""YOLO inference helpers used by the live acquisition interfaces."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ultralytics import YOLO


@dataclass(frozen=True, slots=True)
class Detection:
    """One object detection expressed in image-pixel coordinates."""

    label: str
    confidence: float
    pixel_x: int
    pixel_y: int
    box_xyxy: tuple[int, int, int, int]

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def _as_numpy(value: Any) -> np.ndarray:
    """Convert either a PyTorch tensor or an array-like object to NumPy."""

    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return np.asarray(value)


class YoloDetector:
    """Load a YOLO model and annotate frames with detections and centroids."""

    def __init__(
        self,
        weights_path: str | Path | None = None,
        *,
        confidence: float = 0.5,
        device: str | int | None = None,
        model: Any | None = None,
    ) -> None:
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")

        if model is None:
            if weights_path is None:
                raise ValueError("weights_path is required when model is not supplied")
            weights = Path(weights_path).expanduser().resolve()
            if not weights.is_file():
                raise FileNotFoundError(f"YOLO weights not found: {weights}")
            model = YOLO(str(weights))

        self.model = model
        self.confidence = confidence
        self.device = device

    def detect(self, frame: np.ndarray) -> tuple[np.ndarray, list[Detection]]:
        """Run inference and return an annotated BGR frame plus detections."""

        predict_args: dict[str, object] = {
            "source": frame,
            "conf": self.confidence,
            "verbose": False,
        }
        if self.device is not None:
            predict_args["device"] = self.device

        results = self.model.predict(**predict_args)
        annotated = frame.copy()
        detections: list[Detection] = []

        for result in results:
            names = getattr(result, "names", None) or self.model.names
            for box in result.boxes:
                class_id = int(_as_numpy(box.cls[0]).item())
                confidence = float(_as_numpy(box.conf[0]).item())
                x1, y1, x2, y2 = map(
                    int, _as_numpy(box.xyxy[0]).round().tolist()
                )
                center_x = round((x1 + x2) / 2)
                center_y = round((y1 + y2) / 2)
                label = (
                    str(names.get(class_id, class_id))
                    if isinstance(names, dict)
                    else str(names[class_id])
                )

                detection = Detection(
                    label=label,
                    confidence=confidence,
                    pixel_x=center_x,
                    pixel_y=center_y,
                    box_xyxy=(x1, y1, x2, y2),
                )
                detections.append(detection)

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(annotated, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(
                    annotated,
                    f"{label} {confidence:.2f}",
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    annotated,
                    f"({center_x}, {center_y})",
                    (center_x + 10, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

        return annotated, detections
