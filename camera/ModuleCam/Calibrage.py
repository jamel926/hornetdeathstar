import cv2
import numpy as np



def CoordonnerCalibrage(image, D1, D2):  #Fonction de calibrage pour definir un nouveau répère au niveau des quatre coins rouges autour du carré utile #Elle prends 
#l'image originale, la distance entre les coins
#Elle donne deux listes, la première avec les coef de proportion et la deuxième avec les coordenés d'origine
        image = cv2.imread(image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #Bleu,Green,Red -> Hue, saturation, 
        

        # 2. Détecter les coins rouges (plages HSV pour le rouge)
        lower_red1 = np.array([0, 80, 60])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([165, 80, 60])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Trouver les contours des objets rouges pour extraire leurs centres (les coins)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        corners = []


        for c in contours:
            if cv2.contourArea(c) > 3:  # Filtrer le bruit
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    corners.append((cX, cY))

        # 3. Traitement si les 4 coins sont détectés
        if len(corners) >= 4:
            # Convertir en tableau numpy et ordonner les coins : [haut-gauche, haut-droite, bas-droite, bas-gauche]
            corners = np.array(corners, dtype="float32")
            s = corners.sum(axis=1)
            tl = corners[np.argmin(s)]
            br = corners[np.argmax(s)]
            diff = np.diff(corners, axis=1)
            tr = corners[np.argmin(diff)]
            bl = corners[np.argmax(diff)]
            
            pts_src = np.array([tl, tr, br, bl], dtype="float32")

            # Calculer les distances en pixels entre les côtés
            dist_top = np.linalg.norm(tl - tr)
            dist_right = np.linalg.norm(tr - br)
            dist_bottom = np.linalg.norm(br - bl)
            dist_left = np.linalg.norm(bl - tl)
            
            
            x_o = int(tl[0])
            y_o = int(tl[1])

            return [float(dist_top), float(dist_right), float(dist_bottom), float(dist_left)],[float(x_o), float(y_o)], pts_src

        else:
            print("Erreur : Moins de 4 coins rouges ont été détectés.")
            return None, None, None

def CoordenéesAuRobo(coef, CoordDorigine, CoordPointIA): #Fonction pour calculer les coordenées à envoyer au robo. Elle prends la liste des coeficients, la liste des coordenées de l'origine (x_0, y_0) et la liste de coordenées du point decteté par l'IA 
        return [abs((CoordPointIA[0]-CoordDorigine[0]) / coef[0]), abs((CoordPointIA[1]-CoordDorigine[1]) / coef[1])]
            
def debbugCalibrage(coefProportion, image_input, pts_src, para=True):
    # Charger l'image si un chemin est fourni
    if isinstance(image_input, str):
        image = cv2.imread(image_input)
    else:
        image = image_input

    # Récupérer les coordonnées minimales et maximales des 4 coins
    x_min = int(np.min(pts_src[:, 0]))
    x_max = int(np.max(pts_src[:, 0]))
    y_min = int(np.min(pts_src[:, 1]))
    y_max = int(np.max(pts_src[:, 1]))

    # Découper l'image pour supprimer tout ce qui dépasse
    cropped = image[y_min:y_max, x_min:x_max]

    # Sauvegarder l'image rognée
    cv2.imwrite('C:/Users/Eleves/Documents/MagroHornetDeathStar/Carre_extrait.jpg', cropped)
    print("Image rognée sauvegardée avec succès !")


def main():
    coefProportion, CoordDorigine, pts_src = CoordonnerCalibrage("C:/Users/Eleves/Downloads/KimiMercedez.png", 70.5, 82.5)
    
    # 1. Sécurité : On vérifie que la détection a fonctionné
    if pts_src is None:
        print("Calibrage impossible : les 4 coins rouges n'ont pas été détectés.")
        return

    # 2. On lance le débogage uniquement si pts_src existe
    debbugCalibrage(coefProportion, "C:/Users/Eleves/Downloads/KimiMercedez.png", pts_src, True)
main()