import cv2
import numpy as np


def CoordonnerCalibrage(image, D):  #Fonction de calibrage pour definir un nouveau répère au niveau des quatre coins rouges autour du carré utile #Elle prends l'image originale, la distance entre les coins (suposé. Carré)
#Elle donne deux listes, la première avec les coef de proportion et la deuxième avec les coordenés d'origine

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #Bleu,Green,Red -> Hue, saturation, 
        

        # 2. Détecter les coins rouges (plages HSV pour le rouge)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Trouver les contours des objets rouges pour extraire leurs centres (les coins)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        corners = []


        for c in contours:
            if cv2.contourArea(c) > 10:  # Filtrer le bruit
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

            return [float(D/dist_top), float(D/dist_right), float(D/dist_bottom), float(D/dist_left)],[float(x_o), float(y_o)]

        else:
             return(None, None)

def CoordenéesAuRobo(coef, CoordDorigine, CoordPointIA): #Fonction pour calculer les coordenées à envoyer au robo. Elle prends la liste des coeficients, la listedes coordenées de l'origine (x_0, y_0) et la liste de coordenées du point decteté par l'IA 
        return [(CoordPointIA[0]-CoordDorigine[0])*coef[0], (CoordPointIA[1]-CoordDorigine[1])*coef[1]]
            
   # # 4. Enlever tout ce qui est à l'extérieur (recadrage par perspective)
   # width = max(int(dist_top), int(dist_bottom))
   # height = max(int(dist_left), int(dist_right))
    
   # pts_dst = np.array([
   #     [0, 0],
   #     [width - 1, 0],
   #     [width - 1, height - 1],
   #     [0, height - 1]], dtype="float32")

  
   # matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
   # warped = cv2.warpPerspective(image, matrix, (width, height))

    # Sauvegarder le résultat (extension .jpg ou .png obligatoire)
    #cv2.imwrite('C:/Users/Eleves/Documents/MagroHornetDeathStar/Carre_extrait.jpg', warped)
