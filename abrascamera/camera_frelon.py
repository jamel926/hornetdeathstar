"""Detecte le frelon a la camera et ecrit sa position dans le journal partage."""

"""
===========================================================
PROCESSUS 1 : LA CAMERA
===========================================================

    camera -> YOLO -> reperage -> positions_frelon.csv

Ce programme ne connait pas le bras. Il ecrit des positions
dans un fichier, un point c'est tout. On peut le lancer seul
et regarder le fichier se remplir avec :

    tail -f abrascamera/positions_frelon.csv


Lancement
---------
    .venv/bin/python abrascamera/camera_frelon.py
    .venv/bin/python abrascamera/camera_frelon.py --afficher
    .venv/bin/python abrascamera/camera_frelon.py --recalibrer

La premiere fois, il faut calibrer : les quatre gommettes
rouges doivent etre visibles et rien d'autre de rouge dans
le champ. Le calibrage est ensuite enregistre et relu tout
seul.


Ce que ce programme ne reprend PAS de main.py
---------------------------------------------
1. La fonction verification() (main.py lignes 71 a 99).
   Elle n'ecrit une position que si les CINQ dernieres sont
   stables a 3 % pres. C'est un filtre "cible immobile" : un
   frelon en vol ne le passe jamais. Ici, une detection
   au-dessus du seuil de confiance est ecrite.

2. Le temps relatif (main.py ligne 153). Voir pont.py.

3. Le mode "w" a l'ouverture (main.py ligne 132), qui
   tronque le journal pendant que le bras le lit.

4. detect_objects() (main.py lignes 28 a 49), remplacee par
   YoloDetector : le seuil y est passe a ultralytics au lieu
   d'etre applique apres coup, et la frame est copiee avant
   annotation.


Limite connue : un seul frelon
------------------------------
On ecrit la detection la plus sure de l'image. S'il y a deux
frelons, le second est ignore. C'est deja le comportement de
main.py (ligne 46, la variable est ecrasee a chaque boite),
sauf qu'ici c'est un choix et qu'il garde le meilleur, pas
le dernier de la liste.
"""


import argparse
import os
import sys


DOSSIER = os.path.dirname(os.path.abspath(__file__))

RACINE = os.path.dirname(DOSSIER)


"""
Les modules du pole image s'importent en paquet depuis la
racine du depot (camera/__init__.py existe).
"""
for chemin in (RACINE, DOSSIER):
    if chemin not in sys.path:
        sys.path.insert(0, chemin)


import cv2

import pont
import reperage

from camera.ModuleCam.ModuleCamera import LaCamera
from camera.ReconnaissanceObjet.ModuleAI import YoloDetector


# =========================================================
# 1. PARAMETRES
# =========================================================


"""
Model5 est le plus recent et le meilleur des cinq jeux de
poids du depot : 150 epoques, une seule classe "frelon",
mAP50 = 0,995 sur son jeu de validation.

GUI.py, lui, charge Model2 (50 epoques) et ne connait meme
pas Model5 -- sa fonction default_model_path() date d'avant.
"""
CHEMIN_MODELE = os.path.join(
    RACINE,
    "camera", "IA", "Model5", "runs", "detect", "train", "weights", "best.pt",
)


"""
Index de la camera. main.py et TestYOLO.py utilisent 1 : sur
l'ordi du robot, 0 est la webcam integree.
"""
INDEX_CAMERA = 1


"""
Seuil de confiance passe a ultralytics. main.py filtre a
0,43 apres coup, YoloDetector prend 0,5 par defaut. On garde
0,5 : en dessous, le modele confond les ombres avec des
frelons.
"""
CONFIANCE = 0.5


"""
Nombre d'images sautees pour prendre la plus recente. Sans
ca, on traite une image vieille de plusieurs dixiemes de
seconde pendant que le tampon de la camera se remplit.
"""
SAUT_IMAGES = 3


# =========================================================
# 2. MISE EN PLACE
# =========================================================


def ouvrirCamera(index):
    """Ouvre la camera et retourne l'objet, ou None si elle ne repond pas."""

    """
    On n'utilise pas etatConnexionCamera() ni
    flagCaptureImage() de LaCamera : leur booleen est
    INVERSE (False veut dire "tout va bien"). Un piege a la
    premiere lecture. is_connected() est franc.
    """

    camera = LaCamera(camera_index=index)

    if not camera.is_connected():
        print("La camera %d ne repond pas." % index)
        print("Verifier le branchement, ou essayer un autre index (--camera).")
        return None

    return camera


def obtenirCalibrage(camera, recalibrer):
    """Relit le calibrage enregistre, ou en fait un nouveau depuis l'image courante."""

    if not recalibrer:

        calibrage = reperage.chargerCalibrage()

        if calibrage is not None:
            print("Calibrage relu depuis %s" % reperage.CHEMIN_CALIBRAGE)
            print("Pour en refaire un : --recalibrer")
            return calibrage

    print()
    print("Calibrage : les quatre gommettes rouges doivent etre visibles,")
    print("et rien d'autre de rouge dans le champ.")
    print()

    """
    Les premieres images d'une camera qui vient de s'ouvrir
    sont sous-exposees, le temps que l'exposition automatique
    se regle. Le rouge y passe sous le seuil de saturation et
    les coins ne sont pas trouves. GUI.py jette huit images
    pour la meme raison (lignes 448 a 451).
    """
    for _ in range(10):
        camera.read()

    reussi, image = camera.read()

    if not reussi:
        print("Lecture de l'image impossible.")
        return None

    calibrage = reperage.calibrer(image)

    if calibrage is None:
        print("Les quatre coins rouges n'ont pas ete trouves.")
        print("Relancer avec --afficher pour voir ce que voit la camera.")
        return None

    reperage.enregistrerCalibrage(calibrage)

    print("Calibrage reussi, enregistre dans %s" % reperage.CHEMIN_CALIBRAGE)

    return calibrage


# =========================================================
# 3. LA BOUCLE
# =========================================================


def meilleureDetection(detections):
    """Retourne la detection la plus sure de l'image, ou None."""

    if not detections:
        return None

    return max(detections, key=lambda d: d.confidence)


def boucler(camera, detecteur, calibrage, fichier, writer, afficher):
    """Detecte et journalise jusqu'a l'interruption. Retourne les compteurs de la seance."""

    ecrites = 0

    images = 0

    hors_grille = 0

    print()
    print("Detection en cours. Ctrl+C pour arreter", end="")
    print(", ou 'q' dans la fenetre." if afficher else ".")
    print()

    """
    Ctrl+C est la facon normale d'arreter ce programme, pas
    un incident : on l'attrape ici plutot que plus haut pour
    que le bilan de la seance s'affiche quand meme.
    """

    try:

        while True:

            reussi, image = camera.read_latest(max_skip=SAUT_IMAGES)

            if not reussi:
                continue

            images += 1

            annotee, detections = detecteur.detect(image)

            detection = meilleureDetection(detections)

            if detection is not None:

                x_pct, y_pct = reperage.enPourcentage(
                    calibrage,
                    detection.pixel_x,
                    detection.pixel_y,
                )

                """
                Un frelon vu en dehors du quadrillage n'est
                pas une cible : le bras ne sait pas pointer
                la-bas, son modele n'y est pas calibre. On le
                compte pour pouvoir le dire a l'arret, mais
                on ne l'ecrit pas.
                """
                if reperage.dansLaGrille(x_pct, y_pct):

                    pont.ecrireDetection(
                        fichier, writer,
                        x_pct, y_pct,
                        detection.confidence,
                    )

                    ecrites += 1

                    print("  %6.2f %%  %6.2f %%   confiance %.2f" % (
                        x_pct, y_pct, detection.confidence,
                    ))

                else:
                    hors_grille += 1

            if afficher:

                cv2.imshow("Detection frelon", annotee)

                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break

    except KeyboardInterrupt:
        print()
        print("Arret demande.")

    return ecrites, images, hors_grille


# =========================================================
# 4. POINT D'ENTREE
# =========================================================


def lireArguments():
    """Lit les options de la ligne de commande."""

    analyseur = argparse.ArgumentParser(
        description="Detecte le frelon et ecrit sa position pour le bras.",
    )

    analyseur.add_argument(
        "--camera", type=int, default=INDEX_CAMERA,
        help="index de la camera (defaut : %d)" % INDEX_CAMERA,
    )

    analyseur.add_argument(
        "--confiance", type=float, default=CONFIANCE,
        help="seuil de confiance YOLO (defaut : %.2f)" % CONFIANCE,
    )

    analyseur.add_argument(
        "--modele", default=CHEMIN_MODELE,
        help="chemin des poids YOLO",
    )

    analyseur.add_argument(
        "--recalibrer", action="store_true",
        help="refaire le calibrage au lieu de relire celui du disque",
    )

    analyseur.add_argument(
        "--afficher", action="store_true",
        help="ouvrir une fenetre avec les detections",
    )

    return analyseur.parse_args()


def main():
    """Lance la detection et journalise les positions."""

    options = lireArguments()

    if not os.path.exists(options.modele):
        print("Poids YOLO introuvables :")
        print("   %s" % options.modele)
        return 1

    camera = ouvrirCamera(options.camera)

    if camera is None:
        return 1

    fichier = None

    try:

        calibrage = obtenirCalibrage(camera, options.recalibrer)

        if calibrage is None:
            return 1

        print()
        print("Chargement du modele...")

        detecteur = YoloDetector(options.modele, confidence=options.confiance)

        fichier, writer = pont.ouvrirJournal()

        print("Journal : %s" % pont.CHEMIN_POSITIONS)

        ecrites, images, hors_grille = boucler(
            camera, detecteur, calibrage,
            fichier, writer,
            options.afficher,
        )

    except KeyboardInterrupt:
        print()
        print("Arret demande.")
        return 0

    finally:

        camera.release()

        if fichier is not None:
            fichier.close()

        if options.afficher:
            cv2.destroyAllWindows()

    print()
    print("%d images, %d positions ecrites, %d detections hors grille." % (
        images, ecrites, hors_grille,
    ))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
