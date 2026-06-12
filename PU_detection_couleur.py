import cv2
import numpy as np

#demander le choix de la couleur
def detecterObjet():
    color = "vert"
    cap = cv2.VideoCapture(0) #à changer selon la caméra
    if not cap.isOpened():
        print("Impossible d'ouvrir la caméra")
        exit()
    vu = False
    while not vu:
        ret, frame = cap.read()

        if not ret:
            #break
            print("not ret)")

        # Conversion BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        # HSV = Hue Saturation Value
        if color == "rouge":
            Tmin = 0
            Tmax = 10
        if color == "orange":
            Tmin = 10
            Tmax = 20
        if color == "jaune":
            Tmin = 20
            Tmax = 35
        if color == "vert":
            Tmin = 35
            Tmax = 85
        if color == "cyan":
            Tmin = 85
            Tmax = 100
        if color == "bleu":
            Tmin = 100
            Tmax = 130
        if color == "violet":
            Tmin = 130
            Tmax = 155
        if color == "rose":
            Tmin = 155
            Tmax = 170
        lower_color = np.array([Tmin, 100, 100])
        upper_color = np.array([Tmax, 255, 255])

        # Création du masque
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Recherche des contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:

            # Plus grand contour
            contour = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(contour)

            if area > 500:

                # contour englobant
                x, y, w, h = cv2.boundingRect(contour)

                """# Centre
                cx = x + w // 2
                cy = y + h // 2"""
                M = cv2.moments(contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                # Facteur de conversion
                    mm_par_pixel = 0.34

                    h_img, w_img = frame.shape[:2]

                    x_centre= cx - w_img // 2
                    y_centre= cy - h_img // 2

                # Position réelle en mm
                    x_mm = x_centre * mm_par_pixel
                    y_mm = y_centre * mm_par_pixel

                print(
                    f"Position pixel: ({x_centre}, {y_centre}) | "
                    f"Position réelle: ({x_mm:.1f} mm, {y_mm:.1f} mm) | "
                    f"Taille: {w}x{h} px"
                    )
                """cv2.rectangle(
                    frame,
                    (x, y),
                (x + w, y + h),
                (0, 255, 0),
                    2
                )"""
                #contour rectangle ou contour variable
                cv2.drawContours(
                    frame,
                    [contour],
                    -1,
                    (0, 255, 0),
                    3
                )
                cv2.circle(
                    frame,
                    (cx, cy),
                    5,
                    (255, 0, 0),
                    -1
                )
                vu = True
                return (x_mm, y_mm, 0)

        cv2.imshow("Camera", frame)
        #cv2.imshow("Masque", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'): #appuyer sur q pour arreter le programme
            break
            print("continuer à filmer")

    cap.release()
    cv2.destroyAllWindows()
    return (x_mm, y_mm, 0)

#118x118 pixels --> 4x4 cm à 30 cm de distance --> 1 pixel  = 0.34 mm

def donnerCoords(x_mm, y_mm):
    return (x_mm, y_mm)

detecterObjet()