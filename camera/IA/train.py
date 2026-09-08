from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO("yolo26n.pt")
PATH=r"C:\Users\josev\Documents\projects\hornetdeathstar\camera\IA\Model2\deathstar_dataset.v1i.yolov8\data.yaml"
model.train(data=PATH, epochs=50, imgsz=640)
