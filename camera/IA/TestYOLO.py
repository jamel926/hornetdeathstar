import cv2
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO("/home/lucasmay4th/runs/detect/train-4/weights/best.pt")



def detect_objects(frame):
    results = model(frame)
    detected_objects = []

    for r in results:
        for box in r.boxes:
            class_id = int(box.cls[0])  # Get class ID
            confidence = box.conf[0].item()  # Confidence score

            if confidence > 0.5:
                label = model.names[class_id]
                detected_objects.append(label)

                # Draw bounding box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, detected_objects


def main():
    cap = cv2.VideoCapture(0)  # Open webcam

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame, detected_objects = detect_objects(frame)
        cv2.imshow("AI Vision", frame)
        if detected_objects:
            print("Detected objects:", detected_objects)
        key = cv2.waitKey(1) & 0xFF

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()