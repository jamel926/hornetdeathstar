"""Detecte le frelon sur le PC et envoie sa position au bras par le reseau."""

"""
===========================================================
PROCESSUS 1, VERSION RESEAU : LA CAMERA SUR LE PC
===========================================================

    camera -> YOLO -> reperage -> UDP -> Jetson
                               -> positions_frelon.csv (copie locale)

Meme programme que camera_frelon.py, a une difference pres :
chaque position part aussi vers la Jetson par le cable
reseau (voir pont_reseau.py). La camera est branchee sur le
PC, et YOLO tourne sur le PC.

Le journal local est garde : on peut toujours suivre les
detections avec `tail -f abrascamera/positions_frelon.csv`
et rejouer une seance apres coup.


Lancement (sur le PC)
---------------------
    .venv/bin/python abrascamera/camera_frelon_reseau.py --afficher
    .venv/bin/python abrascamera/camera_frelon_reseau.py --camera 2
    .venv/bin/python abrascamera/camera_frelon_reseau.py --ip-bras 10.10.0.103

La camera est trouvee par son NOM (voir trouverCamera), pas
par son index : sur le PC, l'index 0 est la webcam integree,
et l'index de la camera USB change selon l'ordre de
branchement. --camera force un index precis.

Le calibrage des gommettes, le seuil et le modele se reglent
comme dans camera_frelon.py. Le calibrage est enregistre sur
le PC : il faut le refaire une premiere fois ici
(--recalibrer), celui de la Jetson ne suit pas.


Ce qui est recopie de camera_frelon.py
--------------------------------------
La boucle, boucler(), est recopiee : elle ecrivait dans le
journal et doit maintenant aussi envoyer. Les options de la
ligne de commande aussi, avec deux de plus (--ip-bras,
--port). Le reste (ouverture camera, calibrage, choix de la
detection) est importe tel quel.
"""


import argparse
import glob
import os
import sys
import time


DOSSIER = os.path.dirname(os.path.abspath(__file__))

RACINE = os.path.dirname(DOSSIER)


for chemin in (RACINE, DOSSIER):
    if chemin not in sys.path:
        sys.path.insert(0, chemin)


import cv2

import camera_frelon
import pont
import pont_reseau
import reperage

from camera.ReconnaissanceObjet.ModuleAI import YoloDetector


# =========================================================
# 1. PARAMETRES
# =========================================================


"""
Nom sous lequel Linux voit la camera du projet (Realtek
0bda:5a63). Pour le lire : cat /sys/class/video4linux/video*/name
"""
NOM_CAMERA = "USB 2.0 Camera"


"""
YOLO tourne sur le processeur : le PC n'a pas de carte
graphique NVIDIA. Le dire explicitement evite qu'ultralytics
cherche un GPU a chaque lancement.
"""
PERIPHERIQUE_YOLO = "cpu"


"""
Tolerances du controle des quatre coins (voir verifierCoins).

La camera est posee au-dessus du quadrillage : les quatre
gommettes forment un quadrilatere proche d'un rectangle.
Le 17/09, le bon calibrage donnait des angles de 89 a 91
degres et des cotes opposes egaux a 1 % pres. Le faux, avec
une piece rouge du robot prise pour un coin, donnait 123 et
66 degres, et un cote 1,4 fois plus long que son oppose.

Les tolerances laissent une large marge a une camera un peu
inclinee. Si on l'incline expres beaucoup plus, les relever
ici ou lancer avec --forcer-calibrage.
"""
ECART_ANGLE_MAX = 20.0

RAPPORT_COTES_MAX = 1.25


"""
Image de controle ecrite a chaque lancement : la vue de la
camera avec les quatre coins du calibrage dessines dessus.
Un coin qui ne tombe pas sur une gommette s'y voit d'un coup
d'oeil. Les .png sont ignores par git.
"""
CHEMIN_IMAGE_COINS = os.path.join(DOSSIER, "calibrage_camera.png")


# =========================================================
# 2. TROUVER LA CAMERA
# =========================================================


def trouverCamera(nom=NOM_CAMERA):
    """Retourne l'index OpenCV de la camera dont le nom contient `nom`, ou None."""

    """
    Chaque camera USB cree DEUX fichiers /dev/videoN : un
    pour l'image, un pour les metadonnees. Le second ne donne
    aucune image, et OpenCV l'ouvre quand meme sans erreur.
    On garde donc celui dont `index` vaut 0 : c'est le flux
    video.

    Avec le pilote V4L2 de Linux, l'index a passer a OpenCV
    est le N de /dev/videoN.
    """

    trouvees = []

    for dossier in glob.glob("/sys/class/video4linux/video*"):

        try:
            with open(os.path.join(dossier, "name")) as fichier:
                nom_lu = fichier.read().strip()

            with open(os.path.join(dossier, "index")) as fichier:
                sous_index = int(fichier.read().strip())

        except (OSError, ValueError):
            continue

        if nom in nom_lu and sous_index == 0:
            trouvees.append(int(os.path.basename(dossier)[len("video"):]))

    if not trouvees:
        return None

    return min(trouvees)


def choisirCamera(index_force):
    """Retourne l'index a ouvrir : celui de --camera, sinon celui trouve par le nom, sinon None."""

    """
    Pas de repli sur un index par defaut quand la camera n'est
    pas trouvee. Le 17/09, camera USB debranchee, le repli a
    ouvert la webcam integree du PC : le programme a demarre
    sans erreur, applique le calibrage du quadrillage a une
    image qui n'avait rien a voir, et aurait envoye au bras
    tout ce que YOLO croyait y voir. Mieux vaut s'arreter.
    """

    if index_force is not None:
        print("Camera : index %d (force par --camera)" % index_force)
        return index_force

    index = trouverCamera()

    if index is not None:
        print("Camera : \"%s\" trouvee sur /dev/video%d" % (NOM_CAMERA, index))
        return index

    print("Camera \"%s\" introuvable. Est-elle branchee ?" % NOM_CAMERA)
    print("Cameras vues par Linux : cat /sys/class/video4linux/video*/name")
    print("Pour en utiliser une autre, donner son index : --camera N")

    return None


# =========================================================
# 3. CONTROLER LE CALIBRAGE
# =========================================================


def verifierCoins(coins):
    """Retourne la liste des defauts du quadrilatere des coins, vide s'il est plausible."""

    """
    Pourquoi ce controle :
    ----------------------
    reperage.calibrer() prend les quatre plus grosses taches
    rouges de l'image. Il ne sait pas si ce sont des
    gommettes. Le 17/09, une gommette etait cachee par le
    leurre et une piece rouge du robot a pris sa place : le
    calibrage a ete accepte sans message, et le bras ratait
    le frelon de 4 carreaux au centre.

    Un faux coin deforme le quadrilatere. On verifie donc deux
    choses simples, les coins etant ranges haut-gauche,
    haut-droite, bas-droite, bas-gauche (ordonnerCoins) :

      - chaque angle est proche de 90 degres ;
      - chaque cote a peu pres la longueur de son oppose.
    """

    import math

    defauts = []

    for i in range(4):

        precedent = coins[i - 1]
        courant = coins[i]
        suivant = coins[(i + 1) % 4]

        ax = precedent[0] - courant[0]
        ay = precedent[1] - courant[1]
        bx = suivant[0] - courant[0]
        by = suivant[1] - courant[1]

        normes = math.hypot(ax, ay) * math.hypot(bx, by)

        if normes == 0:
            defauts.append("coins %d confondus" % (i + 1))
            continue

        cosinus = max(-1.0, min(1.0, (ax * bx + ay * by) / normes))

        angle = math.degrees(math.acos(cosinus))

        if abs(angle - 90.0) > ECART_ANGLE_MAX:
            defauts.append("angle au coin %d : %.0f degres (attendu 90 +/- %.0f)" % (
                i + 1, angle, ECART_ANGLE_MAX,
            ))

    longueurs = [
        math.hypot(coins[(i + 1) % 4][0] - coins[i][0], coins[(i + 1) % 4][1] - coins[i][1])
        for i in range(4)
    ]

    for i, nom in ((0, "haut / bas"), (1, "droite / gauche")):

        courte = min(longueurs[i], longueurs[i + 2])
        longue = max(longueurs[i], longueurs[i + 2])

        if courte == 0 or longue / courte > RAPPORT_COTES_MAX:
            defauts.append("cotes %s de longueurs trop differentes : %.0f et %.0f pixels" % (
                nom, longueurs[i], longueurs[i + 2],
            ))

    return defauts


def enregistrerImageCoins(camera, coins, chemin=CHEMIN_IMAGE_COINS):
    """Ecrit une image de la camera avec les coins du calibrage dessines dessus."""

    reussi, image = camera.read()

    if not reussi:
        return False

    for i, (x, y) in enumerate(coins):

        centre = (int(round(x)), int(round(y)))

        cv2.circle(image, centre, 12, (0, 255, 0), 3)

        cv2.putText(
            image, str(i + 1), (centre[0] + 14, centre[1]),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
        )

    return cv2.imwrite(chemin, image)


def controlerCalibrage(camera, calibrage, forcer):
    """Verifie le calibrage et enregistre l'image des coins. Retourne True s'il est accepte."""

    coins = calibrage["coins_pixels"]

    if enregistrerImageCoins(camera, coins):
        print("Image des coins : %s" % CHEMIN_IMAGE_COINS)

    defauts = verifierCoins(coins)

    if not defauts:
        return True

    print()
    print("CALIBRAGE SUSPECT : les quatre coins ne forment pas un rectangle plausible.")

    for defaut in defauts:
        print("  - %s" % defaut)

    print()
    print("Cause la plus probable : une gommette cachee (leurre, main) et un autre")
    print("objet rouge pris a sa place. Regarder l'image des coins ci-dessus.")

    if forcer:
        print("--forcer-calibrage : on continue quand meme.")
        return True

    print("Retirer l'objet rouge, degager les gommettes, relancer avec --recalibrer.")
    print("(Camera volontairement tres inclinee : --forcer-calibrage.)")

    return False


# =========================================================
# 4. LA BOUCLE
# =========================================================


class Compteurs(object):
    """Retient ce qui s'est passe pendant la seance, pour le bilan final."""

    def __init__(self):

        self.images = 0

        self.envoyees = 0

        self.hors_grille = 0

        self.duree = 0.0

        self.duree_yolo = 0.0


def boucler(camera, detecteur, calibrage, fichier, writer, emetteur, afficher, compteurs):
    """Detecte, envoie et journalise jusqu'a l'interruption. Remplit les compteurs."""

    print()
    print("Detection en cours. Ctrl+C pour arreter", end="")
    print(", ou 'q' dans la fenetre." if afficher else ".")
    print()

    """
    Les compteurs sont remplis au fil de l'eau plutot que
    retournes a la fin : si la seance est interrompue, le
    bilan dit quand meme combien d'images par seconde on a
    tenu. C'est le chiffre qui dit si YOLO sur le PC va plus
    vite que sur la Jetson.
    """

    depart = time.monotonic()

    try:

        while True:

            reussi, image = camera.read_latest(max_skip=camera_frelon.SAUT_IMAGES)

            if not reussi:
                continue

            compteurs.images += 1

            debut_yolo = time.monotonic()

            annotee, detections = detecteur.detect(image)

            compteurs.duree_yolo += time.monotonic() - debut_yolo

            detection = camera_frelon.meilleureDetection(detections)

            if detection is not None:

                x_pct, y_pct = reperage.enPourcentage(
                    calibrage,
                    detection.pixel_x,
                    detection.pixel_y,
                )

                if reperage.dansLaGrille(x_pct, y_pct):

                    """
                    L'envoi reseau passe AVANT l'ecriture du
                    journal : c'est lui que le bras attend,
                    chaque milliseconde compte.
                    """
                    emetteur.envoyer(x_pct, y_pct, detection.confidence)

                    pont.ecrireDetection(
                        fichier, writer,
                        x_pct, y_pct,
                        detection.confidence,
                    )

                    compteurs.envoyees += 1

                    print("  %6.2f %%  %6.2f %%   confiance %.2f" % (
                        x_pct, y_pct, detection.confidence,
                    ))

                else:
                    compteurs.hors_grille += 1

            if afficher:

                cv2.imshow("Detection frelon", annotee)

                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break

    except KeyboardInterrupt:
        print()
        print("Arret demande.")

    finally:
        compteurs.duree = time.monotonic() - depart


def afficherBilan(compteurs, emetteur):
    """Affiche ce qui s'est passe pendant la seance, cadence comprise."""

    print()
    print("%d images, %d positions envoyees, %d detections hors grille." % (
        compteurs.images, compteurs.envoyees, compteurs.hors_grille,
    ))

    if compteurs.images and compteurs.duree > 0:

        print("Cadence : %.1f images/s sur %.0f s, dont YOLO %.0f ms par image." % (
            compteurs.images / compteurs.duree,
            compteurs.duree,
            1000.0 * compteurs.duree_yolo / compteurs.images,
        ))

    if emetteur.erreurs:
        print("%d envois ont echoue : le cable ou l'adresse du bras est a verifier." % (
            emetteur.erreurs,
        ))


# =========================================================
# 5. POINT D'ENTREE
# =========================================================


def lireArguments():
    """Lit les options de camera_frelon.py, plus l'adresse du bras."""

    analyseur = argparse.ArgumentParser(
        description="Detecte le frelon et envoie sa position au bras par le reseau.",
    )

    analyseur.add_argument("--camera", type=int, default=None,
                           help="index de la camera (defaut : trouvee par son nom, \"%s\")" % NOM_CAMERA)

    analyseur.add_argument("--confiance", type=float, default=camera_frelon.CONFIANCE,
                           help="seuil de confiance YOLO (defaut : %.2f)" % camera_frelon.CONFIANCE)

    analyseur.add_argument("--modele", default=camera_frelon.CHEMIN_MODELE,
                           help="chemin des poids YOLO")

    analyseur.add_argument("--recalibrer", action="store_true",
                           help="refaire le calibrage au lieu de relire celui du disque")

    analyseur.add_argument("--afficher", action="store_true",
                           help="ouvrir une fenetre avec les detections")

    analyseur.add_argument("--forcer-calibrage", action="store_true",
                           help="accepter un calibrage dont les coins ne forment pas un rectangle plausible")

    analyseur.add_argument("--ip-bras", default=pont_reseau.IP_JETSON,
                           help="adresse de la Jetson (defaut : %s)" % pont_reseau.IP_JETSON)

    analyseur.add_argument("--port", type=int, default=pont_reseau.PORT,
                           help="port UDP du bras (defaut : %d)" % pont_reseau.PORT)

    return analyseur.parse_args()


def main():
    """Lance la detection et envoie les positions au bras."""

    options = lireArguments()

    if not os.path.exists(options.modele):
        print("Poids YOLO introuvables :")
        print("   %s" % options.modele)
        return 1

    index = choisirCamera(options.camera)

    if index is None:
        return 1

    camera = camera_frelon.ouvrirCamera(index)

    if camera is None:
        return 1

    fichier = None

    emetteur = pont_reseau.EmetteurPositions(options.ip_bras, options.port)

    compteurs = Compteurs()

    try:

        calibrage = camera_frelon.obtenirCalibrage(camera, options.recalibrer)

        if calibrage is None:
            return 1

        if not controlerCalibrage(camera, calibrage, options.forcer_calibrage):
            return 1

        print()
        print("Chargement du modele...")

        detecteur = YoloDetector(
            options.modele,
            confidence=options.confiance,
            device=PERIPHERIQUE_YOLO,
        )

        fichier, writer = pont.ouvrirJournal()

        print("Journal local : %s" % pont.CHEMIN_POSITIONS)
        print("Envoi vers    : %s:%d (UDP)" % (options.ip_bras, options.port))

        boucler(
            camera, detecteur, calibrage,
            fichier, writer, emetteur,
            options.afficher, compteurs,
        )

    except KeyboardInterrupt:
        print()
        print("Arret demande.")

    finally:

        camera.release()

        emetteur.fermer()

        if fichier is not None:
            fichier.close()

        if options.afficher:
            cv2.destroyAllWindows()

    afficherBilan(compteurs, emetteur)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
