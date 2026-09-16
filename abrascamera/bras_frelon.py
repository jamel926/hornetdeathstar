"""Lit la derniere position de frelon et pointe le laser dessus, en continu."""

"""
===========================================================
PROCESSUS 2 : LE BRAS
===========================================================

    positions_frelon.csv -> carreaux -> J4/J5 -> send_angles

Ce programme ne connait pas la camera. Il lit un fichier. On
peut donc le tester sans camera, en ecrivant une ligne a la
main :

    printf 'horodatage,x_pct,y_pct,confiance\\n%s,50,50,0.9\\n' \\
        "$(date +%s)" > abrascamera/positions_frelon.csv


LE LASER
--------
Ce programme allume la sortie du pointeur. Monter un
POINTEUR ROUGE DE CLASSE 2, pas le laser de puissance : les
conditions d'inhibition du tir (etape E6) ne sont pas
ecrites, et un 3B/4 abime un oeil avant qu'on ait le temps
de fermer la paupiere.

Le pointeur est eteint dans un finally qui couvre aussi
Ctrl+C.


BASE FIGEE
----------
J1, J2 et J3 sont lus une fois apres la mise en position
initiale, puis renvoyes tels quels dans chaque consigne.
Seuls J4 et J5 bougent : le bras se comporte en tourelle.
C'est la phase 1 decidee le 07/09.


UN SEUL PROCESSUS SUR /dev/ttyTHS1
----------------------------------
Si la connexion est refusee, c'est presque toujours qu'un
autre script du projet tourne encore. Ne jamais importer
Bras/ModuleGPIO/PU_GPIO.py : il ouvre le port des l'import.
"""


import argparse
import os
import sys
import time


DOSSIER = os.path.dirname(os.path.abspath(__file__))

RACINE = os.path.dirname(DOSSIER)

CALIBRATION_V3 = os.path.join(
    RACINE,
    "Bras", "ModuleDeDeplacement", "Calibration_v3",
)


for chemin in (CALIBRATION_V3, DOSSIER):
    if chemin not in sys.path:
        sys.path.insert(0, chemin)


import pont

import suivi_frelon
import suivi_frelon_v3

from calibration_grille import (
    COTE_CARREAU,
    allerPositionInitiale,
    allumerPointeur,
    connecterRobot,
    eteindrePointeur,
    lireAngles,
)


# =========================================================
# 1. LE REPERE : POURCENTAGE VERS CARREAUX
# =========================================================


"""
La camera donne x et y entre 0 et 100, en proportion de la
grille. Le bras travaille en CARREAUX, et un carreau vaut
3,0 cm.

    >>> COTE_CARREAU = 3.0, et surtout PAS 2.5. <<<

PU_Pointage.py declare 2,5 : cette valeur est fausse et la
prendre ferait pointer 20 % a cote sans qu'aucun message ne
le signale. Elle est archivee, ne pas la ressortir.

L'etendue ci-dessous est celle de la zone reellement
calibree, lue dans resultats/modele_grille_v2.json :
u de 0 a 24 carreaux, v de -0,5 a 27. Soit 72 cm sur 82,5.
"""

U_ORIGINE = 0.0

V_ORIGINE = 0.0

U_ETENDUE = 24.0

V_ETENDUE = 27.0


"""
LES TROIS DRAPEAUX A REGLER AU BANC
-----------------------------------
Personne n'a jamais mesure comment le repere de la camera
est pose par rapport au quadrillage du bras. Ce n'est pas un
oubli de ce fichier : suivi_frelon.py le dit deja, ligne 820
-- "ca ne se devine pas : il faudra le mesurer une fois, sur
la zone reelle".

Tant que ce n'est pas fait, LE LASER POINTERA A COTE. C'est
normal et c'est reglable en trois essais avec
reglage_repere.py :

  ECHANGER_XY   la camera croise-t-elle les axes ? C'est ce
                que fait CoordonneesAuRobo dans le pole
                image (le pixel horizontal y alimente le y
                retourne), sans qu'on sache si c'est voulu.

  INVERSER_U    le 0 % de la camera est-il du cote du 0
  INVERSER_V    carreau, ou du cote oppose ?

Methode : poser un objet dans un coin connu, lire ce
qu'affiche reglage_repere.py, et basculer le drapeau qui
remet le point au bon endroit. Un drapeau a la fois.
"""

ECHANGER_XY = False

INVERSER_U = False

INVERSER_V = False


def enCarreaux(x_pct, y_pct):
    """Convertit une position en pourcentage de grille vers les carreaux du bras."""

    if ECHANGER_XY:
        x_pct, y_pct = y_pct, x_pct

    if INVERSER_U:
        x_pct = 100.0 - x_pct

    if INVERSER_V:
        y_pct = 100.0 - y_pct

    u = U_ORIGINE + (x_pct / 100.0) * U_ETENDUE

    v = V_ORIGINE + (y_pct / 100.0) * V_ETENDUE

    return (u, v)


# =========================================================
# 2. PARAMETRES DU SUIVI
# =========================================================


"""
Cadence d'envoi des consignes. FREQUENCE_FLUX vaut 10 Hz
dans suivi_frelon.py, decrit comme "l'ordre de grandeur
realiste pour une detection camera".
"""
FREQUENCE = suivi_frelon.FREQUENCE_FLUX


"""
En dessous de cette confiance, la detection est ignoree. Le
seuil YOLO filtre deja a 0,5 cote camera ; celui-ci permet
de resserrer sans relancer la camera.
"""
CONFIANCE_MINIMALE = 0.0


# =========================================================
# 3. MISE EN PLACE DU BRAS
# =========================================================


def preparerModele():
    """Branche le v3 et construit le contexte de conversion attendu par suivi_frelon."""

    """
    brancherV3() remplace des noms DANS les globales du
    module suivi_frelon : construireModeleCalib2,
    anglesPourPoint, verifierBase, le dossier de resultats.
    Il doit donc tourner AVANT tout appel a ces fonctions,
    sans quoi on utilise silencieusement le modele v1.

    Il appelle lui-meme brancherModeleV2() en premier :
    l'ordre compte, et il est deja gere.
    """

    suivi_frelon_v3.brancherV3()

    modele = suivi_frelon.construireModeleCalib2()

    if modele is None:
        return None

    return {
        "modele": modele,
        "correction": suivi_frelon.chargerModeleCorrection(),
    }


def preparerRobot(contexte):
    """Connecte le robot, le met en position initiale et fige la base. Retourne (mc, base, j6)."""

    mc = connecterRobot()

    if mc is None:
        return (None, None, None)

    allerPositionInitiale(mc)

    print()
    print("Referencement (environ 30 secondes, le bras bouge seul)...")
    print()

    """
    En v3, verifierBase a ete remplace par
    verifierBaseEtReferencer : c'est lui qui mesure le
    decalage de commande, et il REFUSE de continuer s'il
    n'y arrive pas. C'est voulu -- sans referencement, le v3
    ne vaut pas mieux que le v2.

    Le decalage derive de 0,7 a 1,25 degre d'une seance a
    l'autre, donc on le refait plutot que de recharger celui
    de la veille.
    """

    if not suivi_frelon.verifierBase(mc, contexte["modele"]):
        return (None, None, None)

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return (None, None, None)

    return (mc, angles[0:3], angles[5])


# =========================================================
# 4. LA BOUCLE DE POINTAGE
# =========================================================


class Compteurs(object):
    """Retient ce qui s'est passe pendant la seance, pour le bilan final."""

    def __init__(self):

        self.consignes = 0

        self.perdues = 0

        self.inatteignables = 0

        self.refusees = 0


def pointer(mc, base, j6, contexte, lecteur, compteurs, frequence, bavard):
    """Suit la derniere position connue jusqu'a l'interruption."""

    """
    L'horloge est ABSOLUE : chaque echeance est calculee
    depuis le depart, pas en ajoutant un sleep au tour
    precedent. Sinon le temps de calcul de chaque tour
    s'accumule et la cadence derive lentement.

    C'est le schema de suivreFlux() (lignes 1017 a 1087).
    On ne l'appelle pas directement parce qu'il accumule un
    releve par point dans une liste sans borne : parfait
    pour une campagne de 200 tirs, pas pour un service qui
    tourne une apres-midi.
    """

    periode = 1.0 / frequence

    depart = time.time()

    numero = 0

    laser_allume = False

    derniere_consigne = None

    while True:

        echeance = depart + numero * periode

        attente = echeance - time.time()

        if attente > 0:
            time.sleep(attente)

        numero += 1

        position = lecteur.lire()

        # ---------------------------------------------
        # Pas de cible, ou cible perimee
        # ---------------------------------------------

        if position is None or position[3] > pont.AGE_MAX:

            if laser_allume:
                eteindrePointeur(mc)
                laser_allume = False

                if bavard:
                    print("  cible perdue, pointeur eteint")

            if position is not None:
                compteurs.perdues += 1

            continue

        x_pct, y_pct, confiance, age = position

        if confiance < CONFIANCE_MINIMALE:
            continue

        # ---------------------------------------------
        # Conversion
        # ---------------------------------------------

        u, v = enCarreaux(x_pct, y_pct)

        consigne = suivi_frelon.convertirPosition(contexte, u, v)

        """
        convertirPosition rend None quand le point sort de
        l'enveloppe calibree ou des bornes J4/J5. On saute
        le point sans arreter la boucle : c'est ce que fait
        suivi_frelon, et c'est le bon reflexe -- une cible
        inatteignable n'est pas une panne.
        """
        if consigne is None:
            compteurs.inatteignables += 1
            continue

        # ---------------------------------------------
        # Envoi
        # ---------------------------------------------

        if not laser_allume:
            allumerPointeur(mc)
            laser_allume = True

        pose = [
            base[0], base[1], base[2],
            consigne["j4"], consigne["j5"], j6,
        ]

        try:
            mc.send_angles(pose, suivi_frelon.VITESSE_SUIVI)

        except Exception as erreur:
            compteurs.refusees += 1

            if bavard:
                print("  consigne refusee : %s" % erreur)

            continue

        compteurs.consignes += 1

        derniere_consigne = consigne

        if bavard:
            print("  %6.2f %% %6.2f %%  ->  %5.1f %5.1f carreaux"
                  "  ->  J4 %+7.2f  J5 %+7.2f   (age %.2f s)" % (
                      x_pct, y_pct,
                      consigne["x"], consigne["y"],
                      consigne["j4"], consigne["j5"],
                      age,
                  ))

    return derniere_consigne


# =========================================================
# 5. POINT D'ENTREE
# =========================================================


def lireArguments():
    """Lit les options de la ligne de commande."""

    analyseur = argparse.ArgumentParser(
        description="Pointe le laser sur la derniere position de frelon journalisee.",
    )

    analyseur.add_argument(
        "--frequence", type=float, default=FREQUENCE,
        help="cadence des consignes en Hz (defaut : %.1f)" % FREQUENCE,
    )

    analyseur.add_argument(
        "--journal", default=pont.CHEMIN_POSITIONS,
        help="fichier de positions a suivre",
    )

    analyseur.add_argument(
        "--silencieux", action="store_true",
        help="ne pas afficher chaque consigne",
    )

    return analyseur.parse_args()


def afficherBilan(compteurs):
    """Affiche ce qui s'est passe pendant la seance."""

    print()
    print("-" * 57)
    print("BILAN")
    print("-" * 57)
    print()
    print("  consignes envoyees        %d" % compteurs.consignes)
    print("  cibles perimees           %d" % compteurs.perdues)
    print("  points inatteignables     %d" % compteurs.inatteignables)
    print("  consignes refusees        %d" % compteurs.refusees)

    if compteurs.inatteignables > compteurs.consignes:
        print()
        print("  Plus de points refuses qu'envoyes : le repere est")
        print("  probablement mal regle. Lancer reglage_repere.py.")

    print()


def main():
    """Lance le suivi de la derniere position connue."""

    options = lireArguments()

    print("Zone calibree : %.0f x %.0f carreaux, soit %.0f x %.0f cm." % (
        U_ETENDUE, V_ETENDUE,
        U_ETENDUE * COTE_CARREAU, V_ETENDUE * COTE_CARREAU,
    ))

    contexte = preparerModele()

    if contexte is None:
        print("Modele de calibration introuvable. Lancer autotest.py du v3.")
        return 1

    mc, base, j6 = preparerRobot(contexte)

    if mc is None:
        return 1

    print()
    print("Base verrouillee :", [round(a, 2) for a in base])
    print("Journal suivi    :", options.journal)
    print()
    print("MONTER UN POINTEUR ROUGE DE CLASSE 2, pas le laser de puissance.")
    print()

    input("Entree pour lancer le suivi (Ctrl+C pour arreter) > ")

    lecteur = pont.LecteurDernierePosition(options.journal)

    compteurs = Compteurs()

    print()
    print("Suivi en cours...")
    print()

    try:
        pointer(
            mc, base, j6, contexte, lecteur, compteurs,
            options.frequence,
            not options.silencieux,
        )

    except KeyboardInterrupt:
        print()
        print("Suivi interrompu.")

    finally:
        """
        Le pointeur s'eteint quoi qu'il arrive, y compris sur
        Ctrl+C et sur une exception non prevue. C'est la
        seule chose de ce fichier qui n'a pas le droit
        d'echouer.
        """
        eteindrePointeur(mc)

    afficherBilan(compteurs)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
