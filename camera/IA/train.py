from pathlib import Path

from ultralytics import YOLO
import yaml


def main():
    #with open("src/camera/IA/Model5/runs/detect/tune-2/best_hyperparameters.yaml", "r") as f:
        #yaml_params = yaml.safe_load(f)

    # Load YOLOv8 model
    model = YOLO("yolo26n.pt")
    PATH="src/camera/IA/Model5 - V2/dataset.v2i.yolov8/data.yaml"
    model.train(data=PATH, epochs=150, imgsz=640)     # Add , **yaml_params for tune parameter

if __name__ == "__main__":
    main()