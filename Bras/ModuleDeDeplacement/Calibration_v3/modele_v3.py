"""
=========================================================
MODELE V3 - NOYAU COMMUN
=========================================================

Ce que le v3 ajoute au v2 :
--------------------------
Rien sur la geometrie. Le modele geometrique du v2 est bon,
la seance du 15/09 l'a montre : depuis les angles REELLEMENT
ATTEINTS, il retrouve l'impact a 1,19 cm rms, conforme a ce
qu'il annonce.

Ce qui manquait, c'est ailleurs : **le bras ne va pas ou on
lui dit**. Sur les 21 tirs de cette seance, l'ecart entre
angle demande et angle atteint vaut

    J4  +1,248 deg      (ecart-type 0,054)
    J5  +0,645 deg      (ecart-type 0,168)

soit un manque quasi parfaitement constant. C'est lui, et
lui seul, qui a transforme 1,28 cm annonces en 2,54 cm
mesures.

Pourquoi ca passait avant :
--------------------------
viserAngles() ne fait plus qu'UN mouvement, sans rattrapage
-- son parametre `passes` ne sert plus a rien, la boucle a
ete retiree. Son docstring le dit : "La compensation des
erreurs mecaniques doit etre faite en amont, par le modele
de correction X/Y."

Le polynome de degre 4 faisait donc DEUX jobs : rattraper le
biais du modele geometrique, et rattraper ce manque
mecanique. Le v2 a remplace le premier et supprime le
second. D'ou la regression.

La separation que tient le v3 :
------------------------------
    GEOMETRIE        ou tombe le faisceau pour des angles
                     donnes. C'est le modele v2, inchange.

    COMMANDE         quels angles le bras atteint quand on
                     lui en demande d'autres. C'est le
                     decalage mesure par referencement.py.

Les deux ne se melangent pas. Le decalage n'entre ni dans le
modele, ni dans anglesPourPointCorrige() : il s'applique au
dernier moment, juste avant d'envoyer la consigne.

C'est ce qui garde les mesures reutilisables. Un tir
enregistre les angles ATTEINTS : la correspondance
(angles, impact) reste vraie quel que soit le decalage
applique ce jour la, donc les seances continuent de nourrir
l'ajustement du modele.

Pourquoi on mesure au lieu de figer une constante :
--------------------------------------------------
Le manque derive d'une seance a l'autre. Sur les relevees
de suivi, J4_consigne - J4_atteint vaut

    14/09 apres-midi    +0,66 a +0,70 deg
    15/09 matin         +0,88 deg
    15/09 apres-midi    +0,98 a +1,10 deg   (1,25 au pointage)

Une constante ecrite en dur serait donc juste un jour et
fausse la semaine suivante, sans que rien ne previenne.
Mesurer coute quelques secondes et se voit.

Ce module ne recopie rien du v2 :
--------------------------------
Il ajoute Calibration_v2 au chemin et reexporte ses
fonctions. Une seule definition de chaque formule dans tout
le projet.
"""


# =========================================================
# 1. IMPORTS ET CHEMIN
# =========================================================

import os
import sys
import json
import glob
import math


DOSSIER = os.path.dirname(os.path.abspath(__file__))

DOSSIER_PARENT = os.path.dirname(DOSSIER)

DOSSIER_V2 = os.path.join(DOSSIER_PARENT, "Calibration_v2")


for chemin in (DOSSIER_V2, DOSSIER_PARENT):
    if chemin not in sys.path:
        sys.path.insert(0, chemin)


"""
Reexport du noyau v2. Les scripts v3 importent tout depuis
ici : ils n'ont jamais a savoir que le v2 existe, et le jour
ou le v2 bouge, il n'y a qu'un fichier a regarder.
"""
from modele_v2 import (                                      # noqa: E402
    CHEMIN_MODELE_V2,
    DOSSIER_RESULTATS_V2,
    U_MAX,
    V_MAX,
    anglesPourPointCorrige,
    chargerModeleV2,
    enCentimetres,
    evaluerResidu,
    predireImpactCorrige,
    resumerModele,
    rmsEtBiais,
    toleranceBase,
    verifierBaseV2,
)

from calibration_grille import (                             # noqa: E402
    COTE_CARREAU,
    J4_MAX,
    J4_MIN,
    J5_MAX,
    J5_MIN,
)


# =========================================================
# 2. CHEMINS
# =========================================================

DOSSIER_RESULTATS_V3 = os.path.join(DOSSIER, "resultats")


MOTIF_DECALAGE = os.path.join(DOSSIER_RESULTATS_V3, "decalage_commande_*.json")


# Au dela, un decalage mesure lors d'une autre seance n'est
# plus une reference : il a eu le temps de deriver.
AGE_DECALAGE_SUSPECT_HEURES = 4.0


# =========================================================
# 3. LE DECALAGE DE COMMANDE
# =========================================================

def decalageNul():
    """Retourne un decalage sans effet, pour tourner sans compensation."""

    return {
        "j4": 0.0,
        "j5": 0.0,
        "ecart_type_j4": 0.0,
        "ecart_type_j5": 0.0,
        "nb_points": 0,
        "date": None,
        "fiable": True,
    }


def compenserCommande(j4, j5, decalage):
    """Ajoute le decalage de commande aux angles vises, et retourne la consigne a envoyer."""

    """
    Le sens du signe : le decalage vaut "demande moins
    atteint", donc positif quand le bras s'arrete EN DESSOUS.
    Pour qu'il arrive la ou on veut, on lui demande plus --
    on AJOUTE.

    Le bornage n'est pas un detail. Aux angles extremes de la
    zone, ajouter 1,25 deg peut franchir la limite
    constructeur ; bougerTete refuserait le mouvement et le
    tir serait perdu. On serre plutot que de refuser : au
    pire la compensation est incomplete sur ce point, et le
    residu le dira.
    """

    if decalage is None:
        return (j4, j5)

    consigne_j4 = min(max(j4 + decalage.get("j4", 0.0), J4_MIN), J4_MAX)
    consigne_j5 = min(max(j5 + decalage.get("j5", 0.0), J5_MIN), J5_MAX)

    return (consigne_j4, consigne_j5)


def resumerDecalage(decalage):
    """Affiche le decalage de commande et ce qu'il vaut au sol."""

    if decalage is None or decalage.get("nb_points", 0) == 0:
        print("Decalage de commande : AUCUN, les consignes partent telles quelles.")
        return

    print("Decalage de commande mesure sur %d point(s) :" % decalage["nb_points"])
    print("  J4 %+.3f deg  (ecart-type %.3f)"
          % (decalage["j4"], decalage.get("ecart_type_j4", 0.0)))
    print("  J5 %+.3f deg  (ecart-type %.3f)"
          % (decalage["j5"], decalage.get("ecart_type_j5", 0.0)))

    if not decalage.get("fiable", True):
        print("  ATTENTION : le manque n'est PAS constant sur la zone.")
        print("  Un decalage unique ne le corrigera qu'en moyenne.")


def enregistrerDecalage(decalage, dossier=None):
    """Ecrit le decalage mesure dans son fichier horodate, et retourne le chemin ou None."""

    """
    Un fichier par mesure, jamais ecrase. C'est l'historique
    qui a de la valeur : c'est lui, et rien d'autre, qui
    montrera que le manque derive -- donc que le bras se
    degrade.
    """

    if dossier is None:
        dossier = DOSSIER_RESULTATS_V3

    from datetime import datetime

    try:
        os.makedirs(dossier, exist_ok=True)

        chemin = os.path.join(
            dossier,
            "decalage_commande_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"),
        )

        with open(chemin, "w") as fichier:
            json.dump(decalage, fichier, indent=2)

        return chemin

    except Exception as e:
        print("Impossible d'enregistrer le decalage :", e)
        return None


def chargerDernierDecalage(motif=None):
    """Relit le dernier decalage mesure et le retourne, ou None s'il n'y en a aucun."""

    if motif is None:
        motif = MOTIF_DECALAGE

    fichiers = sorted(glob.glob(motif))

    if not fichiers:
        return None

    try:
        with open(fichiers[-1], "r") as fichier:
            decalage = json.load(fichier)

    except Exception as e:
        print("Dernier decalage illisible :", e)
        return None

    decalage["fichier"] = fichiers[-1]

    return decalage


def ageDecalageHeures(decalage):
    """Retourne l'age du decalage en heures, ou None si sa date est inconnue."""

    if not decalage or not decalage.get("date"):
        return None

    from datetime import datetime

    try:
        mesure = datetime.fromisoformat(decalage["date"])

    except ValueError:
        return None

    return (datetime.now() - mesure).total_seconds() / 3600.0


def decalagePerime(decalage):
    """Indique si le decalage date d'assez longtemps pour ne plus etre une reference."""

    age = ageDecalageHeures(decalage)

    return age is not None and age > AGE_DECALAGE_SUSPECT_HEURES


# =========================================================
# 4. SENSIBILITE - POUR LIRE LES DEGRES EN CARREAUX
# =========================================================

def sensibilite(modele, j4, j5):
    """Retourne (carreaux par degre de J4, carreaux par degre de J5) autour d'une pose."""

    """
    Un degre ne veut rien dire tant qu'on ne sait pas ce
    qu'il fait au sol, et ca varie d'un facteur trois sur la
    zone : 0,33 carreau par degre de J4 pres du bras (y=0),
    1,02 au fond (y=27).

    C'est exactement pour ca que le manque CONSTANT de
    1,25 deg produit une erreur CROISSANTE avec y. Ce que la
    sensibilite predit, le long de x=12 :

        y= 0   0,33 carreau/deg  ->  1,25 deg = 0,41 carreau
        y=13   0,49 carreau/deg  ->             0,61 carreau
        y=20   0,71 carreau/deg  ->             0,88 carreau
        y=27   1,02 carreau/deg  ->             1,27 carreau

    Mesure du 15/09 : -0,30 carreau sur la bande y 0-5,
    -1,10 sur la bande y 22-27. Le compte y est.

    Sans cette fonction, cette signature reste invisible et
    on cherche un defaut de modele la ou il n'y en a pas.
    """

    origine = predireImpactCorrige(modele, j4, j5)

    if origine is None:
        return None

    voisin_j4 = predireImpactCorrige(modele, j4 + 1.0, j5)
    voisin_j5 = predireImpactCorrige(modele, j4, j5 + 1.0)

    if voisin_j4 is None or voisin_j5 is None:
        return None

    return (
        math.hypot(voisin_j4[0] - origine[0], voisin_j4[1] - origine[1]),
        math.hypot(voisin_j5[0] - origine[0], voisin_j5[1] - origine[1]),
    )
