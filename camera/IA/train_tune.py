from ultralytics import YOLO

# Modèle YOLO de base
model = YOLO("yolo26n.pt")
PATH="src/camera/IA/Model5/dataset.v2i.yolov8/data.yaml"
model.tune(data=PATH, epochs=20, iterations=10, imgsz=640)