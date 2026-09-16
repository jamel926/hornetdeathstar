"""Repere le quadrillage par ses quatre coins rouges et convertit les pixels en pourcentage."""

"""
===========================================================
DES PIXELS VERS LE POURCENTAGE DE GRILLE
===========================================================

Ce module repond a une seule question : ce point detecte par
YOLO, a quel endroit de la grille est-il, en pourcentage ?

    (0, 0)       coin origine
    (100, 100)   coin oppose

C'est le contrat convenu avec le pole image.


D'ou vient ce code
------------------
La detection des quatre coins rouges est RECOPIEE de

    camera/ModuleCam/Calibrage.py, fonction
    CoordonnerCalibrage, lignes 8 a 62

Meme plages HSV, meme filtre de surface, meme tri des coins.
Le code n'est pas importe mais recopie, pour deux raisons.


Raison 1 : l'original bloque
----------------------------
Calibrage.py lignes 20 a 24 fait, A L'INTERIEUR de la
fonction :

    cv2.imshow("Image originale", image)
    cv2.imshow("Masque rouge", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

Un waitKey(0) attend une touche, indefiniment. Appeler cette
fonction depuis une boucle de detection fige tout. C'est
utile pour mettre au point le masque a la main, pas pour
tourner en service.


Raison 2 : l'original ne sort pas ce qu'il annonce
--------------------------------------------------
CoordonneesAuRobo (ligne 67) divise des pixels par des
pixels :

    y = (point[0] - origine[0]) / coef[0]
    x = (point[1] - origine[1]) / coef[1]

Le resultat est une FRACTION SANS DIMENSION, entre 0 et 1.
Ce ne sont pas des centimetres, malgre ce qu'annoncent
main.py ("distanceHaut = 71  # En cm"), GUI.py ("unit":
"cm") et le commentaire de suivi_frelon.py. Les parametres
D1 et D2 de CoordonnerCalibrage ne sont d'ailleurs utilises
nulle part dans le corps de la fonction : il n'y a aucune
conversion en centimetres dans tout le pole image.

Une fraction entre 0 et 1, multipliee par 100, c'est
exactement le pourcentage de grille demande. La sortie de ce
module est donc la meme grandeur que l'originale, dans une
unite assumee.

Note sur les axes : l'original croise x et y (le pixel
horizontal alimente le y retourne). Ici on ne croise pas --
x_pct suit l'horizontale de l'image, y_pct la verticale. Si
le bras pointe en miroir au banc, c'est le drapeau
ECHANGER_XY de bras_frelon.py qu'il faut lever, pas ce
fichier.
"""


import json
import os

import cv2
import numpy as np


DOSSIER = os.path.dirname(os.path.abspath(__file__))


CHEMIN_CALIBRAGE = os.path.join(DOSSIER, "calibrage_camera.json")


# =========================================================
# 1. PARAMETRES DE DETECTION DES COINS
# =========================================================


"""
Plages HSV du rouge, recopiees telles quelles. Le rouge est
a cheval sur les deux bouts de la roue des teintes, d'ou les
deux masques que l'on reunit.
"""
ROUGE_BAS_1 = np.array([0, 80, 60])
ROUGE_HAUT_1 = np.array([15, 255, 255])

ROUGE_BAS_2 = np.array([165, 80, 60])
ROUGE_HAUT_2 = np.array([180, 255, 255])


"""
Surface minimale d'une tache pour etre prise pour un coin.
Valeur de l'original. Elle est basse : 5 pixels, c'est du
bruit rougeatre sur une image un peu bruitee.

Consequence a connaitre : si un cinquieme objet rouge traine
dans le champ -- un stylo, un vetement -- il devient un coin
et le reperage est faux SANS AUCUN MESSAGE. Verifier le
champ avant de calibrer.
"""
SURFACE_MINIMALE = 5


"""
Un quadrilatere de coins doit couvrir au moins cette
fraction de l'image pour etre pris au serieux.

Pourquoi ce garde-fou existe :
------------------------------
Le seuil de 5 pixels ci-dessus laisse passer a peu pres
n'importe quel pixel rougeatre. Teste sur une webcam
ordinaire, sans aucune gommette dans le champ, la detection
"reussit" et renvoie quatre coins -- pris dans le bruit.

Le calibrage est alors FAUX, et rien ne le dit. Le bras
pointe n'importe ou, on cherche l'erreur dans la conversion,
dans le modele, dans les drapeaux d'inversion, et elle est
en amont de tout ca.

Quatre taches de bruit tombent presque toujours au meme
endroit de l'image, donc le quadrilatere qu'elles forment
est minuscule. Quatre vraies gommettes posees aux coins du
quadrillage en couvrent une bonne part. C'est ce qui les
separe.
"""
FRACTION_MINIMALE = 0.10


"""
Deux coins plus proches que cette distance sont forcement la
meme tache vue deux fois, ou du bruit.
"""
ECART_MINIMAL = 20.0


# =========================================================
# 2. TROUVER LES QUATRE COINS
# =========================================================


def trouverCoins(image):
    """Retourne les 4 coins rouges ordonnes (hg, hd, bd, bg) en pixels, ou None."""

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    masque = (
        cv2.inRange(hsv, ROUGE_BAS_1, ROUGE_HAUT_1)
        | cv2.inRange(hsv, ROUGE_BAS_2, ROUGE_HAUT_2)
    )

    contours, _ = cv2.findContours(
        masque,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    centres = []

    for contour in contours:

        if cv2.contourArea(contour) <= SURFACE_MINIMALE:
            continue

        moments = cv2.moments(contour)

        if moments["m00"] == 0:
            continue

        centres.append((
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]),
        ))

    if len(centres) < 4:
        return None

    return ordonnerCoins(centres)


def ordonnerCoins(centres):
    """Trie des centres en haut-gauche, haut-droite, bas-droite, bas-gauche."""

    """
    Le tri par somme et difference des coordonnees, recopie
    de l'original : le coin haut-gauche minimise x + y, le
    bas-droite le maximise ; le haut-droite minimise y - x,
    le bas-gauche le maximise.

    Ca ne marche que si la camera n'est pas tournee de plus
    d'environ 45 degres. C'est le cas ici.
    """

    points = np.array(centres, dtype="float32")

    somme = points.sum(axis=1)

    difference = np.diff(points, axis=1)

    return np.array([
        points[np.argmin(somme)],
        points[np.argmin(difference)],
        points[np.argmax(somme)],
        points[np.argmax(difference)],
    ], dtype="float32")


def aireQuadrilatere(coins):
    """Retourne l'aire en pixels du quadrilatere forme par les quatre coins ordonnes."""

    """
    Formule du lacet. Les coins etant deja ordonnes dans le
    sens horaire, elle donne directement l'aire.
    """

    aire = 0.0

    for indice in range(4):

        x1, y1 = coins[indice]
        x2, y2 = coins[(indice + 1) % 4]

        aire += x1 * y2 - x2 * y1

    return abs(aire) / 2.0


def pourquoiInvraisemblable(coins, forme_image):
    """Retourne la raison de rejeter ces coins, ou None s'ils sont plausibles."""

    hauteur, largeur = forme_image[0], forme_image[1]

    for indice in range(4):
        for autre in range(indice + 1, 4):

            ecart = float(np.linalg.norm(coins[indice] - coins[autre]))

            if ecart < ECART_MINIMAL:
                return (
                    "deux coins a %.0f pixels l'un de l'autre, "
                    "c'est la meme tache ou du bruit" % ecart
                )

    aire = aireQuadrilatere(coins)

    fraction = aire / float(hauteur * largeur)

    if fraction < FRACTION_MINIMALE:
        return (
            "le quadrilatere ne couvre que %.1f %% de l'image "
            "(minimum %.0f %%)" % (fraction * 100.0, FRACTION_MINIMALE * 100.0)
        )

    return None


# =========================================================
# 3. LE CALIBRAGE
# =========================================================


def calibrer(image):
    """Construit la matrice qui envoie les pixels de la grille sur un carre 0-100."""

    """
    Pourquoi une homographie et pas une simple division :
    -----------------------------------------------------
    L'original divise par la longueur de deux cotes. Ca
    suppose que la grille apparait comme un rectangle a
    l'image, donc que la camera la regarde perpendiculaire-
    ment. Toute inclinaison fait un trapeze, et l'erreur
    grandit en s'eloignant du coin d'origine.

    Les quatre coins sont deja mesures : getPerspectiveTrans-
    form redresse le trapeze pour rien de plus. Le bloc est
    d'ailleurs ecrit, mais commente, a la fin de
    Calibrage.py (lignes 72 a 84) -- quelqu'un y avait deja
    pense.

    On envoie directement sur 0-100 plutot que sur des
    pixels : le pourcentage devient le resultat de la
    transformation, sans division supplementaire.
    """

    coins = trouverCoins(image)

    if coins is None:
        return None

    probleme = pourquoiInvraisemblable(coins, image.shape)

    if probleme is not None:
        print("Coins rouges trouves, mais rejetes : %s" % probleme)
        return None

    cible = np.array([
        [0.0, 0.0],
        [100.0, 0.0],
        [100.0, 100.0],
        [0.0, 100.0],
    ], dtype="float32")

    matrice = cv2.getPerspectiveTransform(coins, cible)

    return {
        "matrice": matrice.tolist(),
        "coins_pixels": coins.tolist(),
    }


def enPourcentage(calibrage, x_pixel, y_pixel):
    """Convertit un point de l'image en pourcentage de grille, ou None si hors cadre."""

    matrice = np.array(calibrage["matrice"], dtype="float64")

    point = np.array([[[float(x_pixel), float(y_pixel)]]], dtype="float32")

    transforme = cv2.perspectiveTransform(point, matrice)

    x_pct = float(transforme[0][0][0])
    y_pct = float(transforme[0][0][1])

    return (x_pct, y_pct)


def dansLaGrille(x_pct, y_pct, marge=5.0):
    """Dit si un point en pourcentage tombe dans la grille, avec une marge de tolerance."""

    """
    La marge absorbe le cas frequent du frelon pose sur le
    bord : la detection deborde un peu, le point sort a
    -2 %. Le refuser serait perdre une cible valable.
    """

    return (
        -marge <= x_pct <= 100.0 + marge
        and -marge <= y_pct <= 100.0 + marge
    )


# =========================================================
# 4. PERSISTANCE
# =========================================================


def enregistrerCalibrage(calibrage, chemin=CHEMIN_CALIBRAGE):
    """Ecrit le calibrage sur disque pour ne pas avoir a le refaire au prochain lancement."""

    """
    Ni main.py ni GUI.py ne relisent leur calibrage : les
    deux recalibrent a chaque demarrage. GUI.py ecrit bien un
    JSON dans logs/, mais ne le relit jamais -- c'est un
    journal, pas un etat.

    Tant que la camera et le quadrillage ne bougent pas, le
    calibrage reste valable. Le relire fait gagner la minute
    de mise en place a chaque lancement.
    """

    with open(chemin, "w") as fichier:
        json.dump(calibrage, fichier, indent=2)

    return chemin


def chargerCalibrage(chemin=CHEMIN_CALIBRAGE):
    """Relit un calibrage enregistre, ou None s'il n'y en a pas."""

    if not os.path.exists(chemin):
        return None

    try:
        with open(chemin, "r") as fichier:
            calibrage = json.load(fichier)

    except (ValueError, OSError):
        return None

    if "matrice" not in calibrage:
        return None

    return calibrage


# =========================================================
# 5. VERIFICATION RAPIDE
# =========================================================


def main():
    """Fabrique une image de test avec quatre coins rouges et verifie la conversion."""

    image = np.zeros((480, 640, 3), dtype=np.uint8)

    coins = [(80, 60), (560, 60), (560, 420), (80, 420)]

    for x, y in coins:
        cv2.circle(image, (x, y), 10, (0, 0, 255), -1)

    calibrage = calibrer(image)

    if calibrage is None:
        print("ECHEC : les quatre coins n'ont pas ete trouves.")
        return 1

    print("Coins trouves :")

    for coin in calibrage["coins_pixels"]:
        print("   %7.1f  %7.1f" % (coin[0], coin[1]))

    print()
    print("Conversion de quelques points :")
    print()
    print("   %-18s %s" % ("pixels", "pourcentage"))

    essais = [
        (80, 60),
        (320, 240),
        (560, 420),
        (320, 60),
    ]

    for x, y in essais:

        x_pct, y_pct = enPourcentage(calibrage, x, y)

        print("   (%4d, %4d)       (%6.2f, %6.2f)   %s" % (
            x, y, x_pct, y_pct,
            "dans la grille" if dansLaGrille(x_pct, y_pct) else "DEHORS",
        ))

    print()
    print("Attendu : le centre de l'image tombe sur (50, 50).")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
