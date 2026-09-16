# main_trt.py -- copie de main.py ou la detection passe par TensorRT (GPU de la Jetson)
# au lieu d'ultralytics/PyTorch. Seules la detection (detect_objects) et le lancement
# (bas du fichier) changent ; le reste est identique a main.py.
# Mise en place du moteur TensorRT : voir camera/TensorRT/README.md
import cv2
import time
import csv
import matplotlib.pyplot as plt
from camera.ModuleCam.ModuleCamera import LaCamera
from camera.ModuleCam.Calibrage import CoordonneesAuRobo, CoordonnerCalibrage
from camera.Prediction.linear_prediction import PositionPredictor
from camera.TensorRT.ClientTensorRT import DetecteurTensorRT

# Moteur TensorRT construit sur la Jetson a partir de best.pt (voir camera/TensorRT/README.md)
CHEMIN_ENGINE = "camera/IA/Model5/runs/detect/train/weights/best.engine"
SEUIL_CONFIANCE = 0.43

detecteur = None    # cree au lancement, en bas du fichier

distanceHaut = 71   # En cm
distanceCote = 83
coffD = []
coordonnerZero = []
coordonnerReel = []

ListeCoodPredic = []
ListeCoodReel = []
ListeTemps = []

ListeLatente = []

zoneValiditer = 3   # En %

Latence_est = 0.3 # En seconde

def detect_objects(frame):
    # Meme sortie que detect_objects() de main.py : image annotee, labels, position du dernier objet
    debut = time.perf_counter()
    detections = detecteur.detecter(frame)
    duree = time.perf_counter() - debut

    detected_objects = []
    position = ()

    for d in detections:
        # Le seuil de confiance est deja applique par le detecteur
        label = d.label
        detected_objects.append(label)

        # Draw bounding box
        x1, y1, x2, y2 = d.x1, d.y1, d.x2, d.y2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        position = (int((x1 + x2)/2) , int((y1 + y2)/2))
        cv2.putText(frame, f"({position[0]}, {position[1]})", (position[0] + 20, position[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Temps de detection (envoi + GPU + retour), pour mesurer le gain par rapport au CPU
    cv2.putText(frame, f"detection : {duree * 1000:.0f} ms", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return frame, detected_objects, position

def afficherPosition(ListeCoodPredic, ListeCoodReel, ListeTemps, affListe):

    if affListe == True: 
        print(ListeTemps, ListeCoodReel, ListeCoodPredic)

    fig, axs = plt.subplots(2)
    fig.suptitle('Position in the time')     

    axs[0].plot(ListeTemps, [coord[0] for coord in ListeCoodReel], label="Xreel")
    axs[0].plot(ListeTemps, [coord[0] for coord in ListeCoodPredic], label="Xpredic")
    axs[0].grid()
    axs[0].legend()

    axs[1].plot(ListeTemps, [coord[1] for coord in ListeCoodReel], label="Yreel")
    axs[1].plot(ListeTemps, [coord[1] for coord in ListeCoodPredic], label="Ypredic")
    axs[1].grid()
    axs[1].legend() 

    plt.show()

def verification(ListeLatence, zoneValiditer):
    if len(ListeLatence) < 3:
        return False

    lastEvent = ListeLatence[-5:]

    ListeX = [coord[0] for coord in lastEvent]
    ListeY = [coord[1] for coord in lastEvent]

    valeurMinX = min(ListeX)
    valeurMaxX = max(ListeX)

    valeurMinY = min(ListeY)
    valeurMaxY = max(ListeY)

    if valeurMinX == 0:
        ecart_pourcentageX = ((valeurMaxX - valeurMinX) * 100 if valeurMaxX != 0 else 0)
    else:
        ecart_pourcentageX = ((valeurMaxX - valeurMinX) / abs(valeurMinX)) * 100

    if valeurMinY == 0:
        ecart_pourcentageY = ((valeurMaxY - valeurMinY) * 100 if valeurMaxY != 0 else 0)
    else:
        ecart_pourcentageY = ((valeurMaxY - valeurMinY) / abs(valeurMinY)) * 100

    # Vérification des deux axes
    Validiter = (ecart_pourcentageX <= zoneValiditer and ecart_pourcentageY <= zoneValiditer)

    return Validiter

def main():
    start_time = time.perf_counter()

    camera = LaCamera(camera_index=0)

    if camera.etatConnexionCamera():
        return()

    ret, frame = camera.camera.read()

    if not ret or frame is None:
        print("Erreur : impossible de lire une image depuis la caméra")
        camera.release()
        return()

    print("Image reçue :", frame.shape)

    cv2.imshow("Image calibration", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # width, height = camera.tailleCamera()

    coffD, coordonnerZero = CoordonnerCalibrage(frame, distanceCote, distanceHaut)

    if coffD is None:
        print("Calibrage impossible : 4 coins rouges non détectés")
        return()

    predictor = PositionPredictor(buffer_size=10, poly_degree=1, timeout=0.5)

    fichier = open("coordonnees_valides.csv", "w", newline="")
    writer = csv.writer(fichier)

    writer.writerow(["temps", "x", "y"])

    while True:
        ret,frame = camera.read_latest()    # Use read() pour le code normal
        if camera.flagCaptureImage(ret):
            break

        frame, detected_objects, position = detect_objects(frame)

        coordonnerPredite = None

        if position != ():
            coordonnerReel = CoordonneesAuRobo(coffD, coordonnerZero, position)
            predictor.add_detection(coordonnerReel)
            coordonnerPredite = predictor.get_prediction(Latence_est)

            ListeLatente.append(coordonnerReel)

            elapsed_time = time.perf_counter() - start_time

            ListeCoodPredic.append(coordonnerPredite)
            ListeCoodReel.append(coordonnerReel)
            ListeTemps.append(elapsed_time)

        cv2.imshow("AI Vision", frame)
        if detected_objects:
            if verification(ListeLatente, zoneValiditer) == True:
                print("\033[32m Detected objects:", detected_objects, coordonnerReel, "predit:", coordonnerPredite, "\033[0m")
                writer.writerow([elapsed_time, coordonnerReel[0], coordonnerReel[1]])
                fichier.flush()
            else:
                print("\033[31m Detected objects:", detected_objects, coordonnerReel, "predit:", coordonnerPredite, "\033[0m")

        else:
            print("\033[33m No detected objects \033[0m")


        if camera.quitterAffichage():           
            break

    # afficherPosition(ListeCoodPredic, ListeCoodReel, ListeTemps, affListe = False)
    
    camera.camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Le detecteur demarre le serveur TensorRT ; le finally garantit qu'il est arrete
    # meme si le programme plante, sinon il garderait la memoire du GPU.
    detecteur = DetecteurTensorRT(CHEMIN_ENGINE, confiance=SEUIL_CONFIANCE)
    try:
        main()
    finally:
        detecteur.fermer()