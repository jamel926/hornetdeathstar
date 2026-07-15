from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO("yolo26n.pt")
PATH="Chemin vers le .yml du dataset"
model.train(data=PATH, epochs=50, imgsz=640)
