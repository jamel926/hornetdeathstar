from ultralytics import YOLO

DATA_YAML=r"C:\Users\josev\Documents\projects\hornetdeathstar\camera\IA\Model2\deathstar_dataset.v1i.yolov8\data.yaml"
HYP_YAML =r""


def main():
    # Load YOLO model.
    model = YOLO("yolo26n.pt")

    if HYP_YAML:
        model.train(
            cfg=HYP_YAML,
            data=DATA_YAML,
            epochs=50,
            batch=16,       # use -1 for automatic GPU-memory sizing
            imgsz=960,
            patience=20,
            optimizer="AdamW",
            workers=4,
        )
    else:
        model.train(
            data=DATA_YAML,
            epochs=50,
            batch=16,       # use -1 for automatic GPU-memory sizing
            imgsz=960,
            patience=20,
            optimizer="AdamW",
            lr0=0.00594,
            workers=4,
            degrees=0,
        )


if __name__ == "__main__":
    main()
