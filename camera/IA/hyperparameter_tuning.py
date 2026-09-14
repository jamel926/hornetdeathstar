from ultralytics import YOLO

# Initialize the YOLO model
model = YOLO("yolo26n.pt")

# Define search space
search_space = {
    "lr0": (1e-5, 1e-2), #For the adam optimizer, the recommended learning rate is  0.001 by default
    "degrees": (0.0, 45.0),
}

DATA_YAML = r"C:\Users\josev\Documents\projects\hornetdeathstar\camera\IA\Model2\deathstar_dataset.v1i.yolov8\data.yaml"

# Tune hyperparameters on dataset for 30 epochs
model.tune(
    data=DATA_YAML,
    epochs=30,
    iterations=50,
    optimizer="AdamW",
    space=search_space,
    device=0, # If your GPU is not available, you can set device to "cpu"
    batch=-1,
    imgsz=640,
    workers=4,
    amp=True,
    plots=False,
    save=False,
    val=True,
)