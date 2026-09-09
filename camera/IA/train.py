from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]

data_yaml = (
    ROOT
    / "camera"
    / "IA"
    / "Model2"
    / "deathstar_dataset.v1i.yolov8"
    / "data.yaml"
)
hyp_yaml = ROOT / "runs" / "detect" / "tune" / "best_hyperparameters.yaml"

model = YOLO(str(ROOT / "yolo26n.pt"))

model.train(
    cfg=str(hyp_yaml),
    data=str(data_yaml),
    epochs=50,
    batch=16,
    imgsz=960,
    patience=20,
    optimizer="AdamW",
    workers=4,
)