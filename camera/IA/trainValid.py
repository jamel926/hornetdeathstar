from ultralytics import YOLO
import yaml

# Load YOLOv8 model
model = YOLO("src/camera/IA/Model5 - V2/runs/detect/train/weights/best.pt")

# Load YOLOv8 model
model = YOLO("yolo26n.pt")
PATH="src/camera/IA/Model5 - V2/dataset.v2i.yolov8/data.yaml"
metrics = model.val(data=PATH, split="test", imgsz=640)

print("\n - Résultats test final - ")
print(f"Précision : {metrics.box.mp:.3f}")
print(f"Recall : {metrics.box.mr:.3f}")
print(f"mAP@50 : {metrics.box.map50:.3f}")
print(f"mAP@50-95 : {metrics.box.map:.3f}")
