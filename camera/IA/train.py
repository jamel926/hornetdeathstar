from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO("yolo26n.pt")
PATH=r"C:\Users\josev\Documents\projects\hornetdeathstar\camera\IA\Model2\deathstar_dataset.v1i.yolov8\data.yaml"
model.train(
    data=PATH,
    epochs=50,
    batch=16,       # use -1 for automatic GPU-memory sizing
    imgsz=960,       # GPU 0; use "cpu" if necessary
    patience=20,
    optimizer="AdamW",
    lr0=0.001,
    workers=4,
)
