import cv2
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO(r"yolo26n.pt")





def detect_objects(frame):
    results = model(frame)
    detected_objects = []

    for r in results:
        for box in r.boxes:
            class_id = int(box.cls[0])  # Get class ID
            confidence = box.conf[0].item()  # Confidence score

            if confidence > 0.3:
                label = model.names[class_id]
                detected_objects.append(label)

                # Draw bounding box
                center_x, center_y, _ , _ = (box.xywh[0].cpu().numpy())
                centroid = (round(center_x), round(center_y))
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, centroid, 6, (0, 0, 255), -1)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, detected_objects


def main():
    cap = cv2.VideoCapture(1)  # Open webcam

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame, detected_objects = detect_objects(frame)
        cv2.imshow("AI Vision", frame)
        if detected_objects:
            print("Detected objects:", detected_objects)
        """Vérifie si l'utilisateur a appuyé sur la touche 'q' pour quitter l'affichage du flux vidéo en direct. Si la touche 'q' est pressée, cela signifie que l'utilisateur souhaite quitter, sinon le flux vidéo continue d'être affiché."""
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), ord('Q')):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()