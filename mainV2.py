import cv2
from ultralytics import YOLO
from camera.ModuleCam.ModuleCamera import LaCamera
from camera.ModuleCam.Calibrage import CoordenéesAuRobo, CoordonnerCalibrage
from camera.Prediction.linear_prediction import PositionPredictor
# Load YOLOv8 model
model = YOLO("src/camera/IA/Model5 - V2/runs/detect/train/weights/best.pt")

distanceHaut = 12   # En cm
distanceCote = 11
coffD = []
coordonnerZero = []
coordonnerReel = []

Latence_est = 0.15 # En seconde

def detect_objects(frame):
    results = model(frame)
    detected_objects = []
    position = ()

    for r in results:
        for box in r.boxes:
            class_id = int(box.cls[0])  # Get class ID
            confidence = box.conf[0].item()  # Confidence score

            if confidence > 0.43:
                label = model.names[class_id]
                detected_objects.append(label)

                # Draw bounding box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                position = (int((x1 + x2)/2) , int((y1 + y2)/2))
                cv2.putText(frame, f"({position[0]}, {position[1]})", (position[0] + 20, position[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, detected_objects, position


def main():
    camera = LaCamera(camera_index=1)   # Open cam
    if camera.etatConnexionCamera():
        return()
    
    ret,frame = camera.camera.read()
    if camera.flagCaptureImage(ret):
        return()
    
    # width, height = camera.tailleCamera()

    coffD, coordonnerZero = CoordonnerCalibrage(frame, distanceCote, distanceHaut)

    if coffD is None:
        print("Calibrage impossible : 4 coins rouges non détectés")
        return()

    predictor = PositionPredictor(buffer_size=10, poly_degree=1, timeout=0.5)

    while True:
        ret,frame = camera.read_latest()    # Use read() pour le code normal
        if camera.flagCaptureImage(ret):
            break

        frame, detected_objects, position = detect_objects(frame)

        coordonnerPredite = None

        if position != ():
            coordonnerReel = CoordenéesAuRobo(coffD, coordonnerZero, position)
            predictor.add_detection(coordonnerReel)
        coordonnerPredite = predictor.get_prediction(Latence_est)

        cv2.imshow("AI Vision", frame)
        if detected_objects:
            print("Detected objects:", detected_objects, coordonnerReel, "predit:", coordonnerPredite)

        if camera.quitterAffichage():           
            break

 
    camera.camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()