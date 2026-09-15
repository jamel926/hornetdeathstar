"""
=========================================================
MODELE V2 - NOYAU COMMUN
=========================================================

Ce module est le seul endroit du dossier qui sait :

    - ou trouver le modele v2 ;
    - comment l'evaluer et l'inverser, residu compris ;
    - comment verifier que le bras est dans la pose de base
      de la calibration.

Les trois scripts du dossier l'importent, personne ne
recopie ces formules.

Ce qui change par rapport au modele d'origine :
----------------------------------------------
Le modele v2 est ajuste sur 112 correspondances au lieu de
35 : les noeuds pointes a la main PLUS les tirs deja
annotes dans validation_pointage_Calib_2.csv. Voir
recalibration_tirs.py pour le detail de l'ajustement.

Il porte en plus un bloc "residu_quadratique" : deux
surfaces de degre 2 qui rattrapent ce que l'homographie
n'explique pas. calibration_grille.py ignore ce bloc, donc
le fichier reste un remplacant direct de modele_grille.json
-- mais pour en profiter il faut passer par les deux
fonctions definies ici.

    predireImpactCorrige(modele, j4, j5)   angles  -> point
    anglesPourPointCorrige(modele, u, v)   point   -> angles

Erreur attendue :
    modele d'origine             1,75 cm rms
    v2 sans le residu            1,40 cm rms
    v2 avec le residu            1,28 cm rms

Chemin d'import :
----------------
Le dossier parent est ajoute au chemin AVANT d'importer
calibration_grille, puisque ce module vit un cran plus bas.
C'est la seule raison pour laquelle les imports ne sont pas
tous groupes en tete de fichier.
"""


# =========================================================
# 1. IMPORTS ET CHEMIN
# =========================================================

import os
import sys
import json
import math


DOSSIER = os.path.dirname(os.path.abspath(__file__))

DOSSIER_PARENT = os.path.dirname(DOSSIER)

if DOSSIER_PARENT not in sys.path:
    sys.path.insert(0, DOSSIER_PARENT)


from calibration_grille import (
    COTE_CARREAU,
    anglesPourPoint,
    lireAngles,
    predireImpact,
)


# =========================================================
# 2. CHEMINS
# =========================================================

DOSSIER_RESULTATS_V2 = os.path.join(DOSSIER, "resultats")


CHEMIN_MODELE_V2 = os.path.join(DOSSIER_RESULTATS_V2, "modele_grille_v2.json")


# Fichiers de la campagne d'origine, un cran au dessus.
DOSSIER_RESULTATS_ORIGINE = os.path.join(DOSSIER_PARENT, "resultats")

CHEMIN_MODELE_ORIGINE = os.path.join(DOSSIER_RESULTATS_ORIGINE, "modele_grille.json")


# Etendue du quadrillage, en carreaux.
U_MAX = 24
V_MAX = 27


# Tours de point fixe pour inverser le residu. Il vaut
# quelques dixiemes de carreau et varie lentement : deux ou
# trois tours suffisent, six est une marge confortable.
TOURS_INVERSION = 6


# =========================================================
# 3. CHARGEMENT
# =========================================================

def chargerModeleV2(chemin=None, silencieux=False):
    """Charge le modele v2 et le retourne, ou None s'il est absent ou illisible."""

    if chemin is None:
        chemin = CHEMIN_MODELE_V2

    if not os.path.exists(chemin):
        print("Modele v2 absent :", chemin)
        print("Le regenerer avec : python3 recalibration_tirs.py")
        return None

    try:
        with open(chemin, "r") as fichier:
            modele = json.load(fichier)

    except Exception as e:
        print("Modele v2 illisible :", e)
        return None

    if not silencieux:
        resumerModele(modele)

    return modele


def resumerModele(modele):
    """Affiche en quelques lignes sur quoi le modele a ete ajuste et ce qu'il vaut."""

    source = modele.get("source_donnees", {})

    print("Modele v2 charge.")
    print("  ajuste sur      : %d correspondances (%d noeuds + %d tirs annotes)"
          % (modele.get("nb_points", 0),
             source.get("noeuds_pointes", 0),
             source.get("tirs_annotes", 0)))

    enveloppe = modele.get("enveloppe", {})

    print("  zone calibree   : x %.1f..%.1f, y %.1f..%.1f carreaux"
          % (enveloppe.get("u_min", 0.0), enveloppe.get("u_max", 0.0),
             enveloppe.get("v_min", 0.0), enveloppe.get("v_max", 0.0)))

    residu = modele.get("residu_quadratique")

    if residu is None:
        print("  residu          : aucun (homographie seule)")
    else:
        print("  residu          : degre %d, actif" % residu.get("degre", 0))

    base = modele.get("base")

    if base:
        print("  pose de base    : J1 %+.2f  J2 %+.2f  J3 %+.2f" % tuple(base))


# =========================================================
# 4. LE RESIDU
# =========================================================

def termesResidu(u, v, degre):
    """Retourne les monomes du residu pour un point, en coordonnees reduites."""

    """
    Coordonnees reduites : on ramene le quadrillage dans
    [-1, 1] avant d'elever aux puissances. Sans cela u^2
    vaut jusqu'a 576 et le systeme normal devient mal
    conditionne.
    """

    a = (u - U_MAX / 2.0) / (U_MAX / 2.0)
    b = (v - V_MAX / 2.0) / (V_MAX / 2.0)

    termes = []

    for ordre in range(degre + 1):
        for i in range(ordre + 1):
            termes.append((a ** (ordre - i)) * (b ** i))

    return termes


def evaluerResidu(residu, u, v):
    """Retourne la correction (du, dv) a ajouter a une position predite."""

    if residu is None:
        return (0.0, 0.0)

    termes = termesResidu(u, v, residu["degre"])

    return (
        sum(c * t for c, t in zip(residu["coefficients_u"], termes)),
        sum(c * t for c, t in zip(residu["coefficients_v"], termes)),
    )


# =========================================================
# 5. PREDICTION ET INVERSION
# =========================================================

def predireImpactCorrige(modele, j4, j5):
    """Retourne (u, v) en carreaux pour un couple d'angles, residu compris."""

    prediction = predireImpact(modele, j4, j5)

    if prediction is None:
        return None

    du, dv = evaluerResidu(modele.get("residu_quadratique"), prediction[0], prediction[1])

    return (prediction[0] + du, prediction[1] + dv)


def anglesPourPointCorrige(modele, u, v, marge=None):
    """Retourne (J4, J5) pour viser le point demande, residu compris, ou None."""

    """
    L'inversion de l'homographie seule reste analytique. Le
    residu, lui, s'inverse par point fixe : on cherche le
    point brut q tel que q + residu(q) tombe sur la cible.

        q <- (u, v) - residu(q)

    On s'arrete des que le pas passe sous le millieme de
    carreau, soit 30 microns au sol -- trois ordres de
    grandeur sous ce que le bras sait faire.
    """

    residu = modele.get("residu_quadratique")

    cible_u = u
    cible_v = v

    for _ in range(TOURS_INVERSION):

        du, dv = evaluerResidu(residu, cible_u, cible_v)

        suivant_u = u - du
        suivant_v = v - dv

        pas = math.hypot(suivant_u - cible_u, suivant_v - cible_v)

        cible_u = suivant_u
        cible_v = suivant_v

        if pas < 1e-3:
            break

    if marge is None:
        return anglesPourPoint(modele, cible_u, cible_v)

    return anglesPourPoint(modele, cible_u, cible_v, marge)


# =========================================================
# 6. POSE DE BASE
# =========================================================

def toleranceBase(modele, marge=0.3):
    """Retourne la tolerance a appliquer sur J1/J2/J3, deduite de la campagne."""

    """
    La tolerance de calibration_grille.py vaut 0,2 degre.
    Elle est faite pour une SEANCE : entre deux points, la
    base ne doit pas bouger, et 0,2 degre est le bon ordre
    de grandeur.

    Entre deux SEANCES, c'est une autre affaire. Sur la
    campagne Calib 2 la base relue s'etale sur 0,88 degre
    en J2 : le bras ne revient pas exactement au meme
    endroit en position initiale. Exiger 0,2 degre ici
    refuserait de tirer alors que la pose est parfaitement
    normale.

    On prend donc l'etendue reellement observee, plus une
    marge. Ce n'est pas un relachement du garde-fou : c'est
    le calibrer sur ce que le bras fait vraiment.

    Ce meme etalement est le premier suspect pour les
    0,75 cm d'erreur qui separent deux seances, et que
    AUCUN reajustement du modele ne peut enlever.
    """

    observee = modele.get("base_observee")

    if not observee:
        return 0.2 + marge

    return max(observee.get("etendue", [0.0, 0.0, 0.0])) + marge


def verifierBaseV2(mc, modele, tolerance=None):
    """Compare la pose J1/J2/J3 actuelle a celle de la calibration, et retourne True si elle tient."""

    reference = modele.get("base")

    if not reference:
        print("ATTENTION : pas de pose de base dans le modele, verification impossible.")
        return True

    if tolerance is None:
        tolerance = toleranceBase(modele)

    mesure = lireAngles(mc)

    if mesure is None:
        print("Lecture des angles impossible.")
        return False

    ecarts = [abs(mesure[i] - reference[i]) for i in range(3)]

    print()
    print("Base de calibration :", [round(a, 2) for a in reference])
    print("Base courante       :", [round(mesure[i], 2) for i in range(3)])
    print("Ecarts              :", [round(e, 2) for e in ecarts])
    print("Tolerance           : %.2f deg (etendue de la campagne + marge)" % tolerance)

    if max(ecarts) > tolerance:
        print()
        print("REFUS : la base est a %.2f deg de la pose de calibration." % max(ecarts))
        print("C'est plus que ce que la campagne a jamais observe.")
        print("Le modele ne decrit plus la geometrie actuelle.")
        print("Relancer la position initiale, ou refaire une campagne.")
        return False

    print("Base conforme.")

    return True


# =========================================================
# 7. PETITS OUTILS PARTAGES
# =========================================================

def enCentimetres(carreaux):
    """Convertit une distance en carreaux vers des centimetres."""

    return carreaux * COTE_CARREAU


def rmsEtBiais(ecarts):
    """Retourne (rms, biais_u, biais_v, dispersion) pour une liste d'ecarts (du, dv)."""

    if not ecarts:
        return None

    nombre = len(ecarts)

    biais_u = sum(e[0] for e in ecarts) / nombre
    biais_v = sum(e[1] for e in ecarts) / nombre

    rms = math.sqrt(sum(e[0] ** 2 + e[1] ** 2 for e in ecarts) / nombre)

    dispersion = math.sqrt(
        sum((e[0] - biais_u) ** 2 + (e[1] - biais_v) ** 2 for e in ecarts) / nombre
    )

    return (rms, biais_u, biais_v, dispersion)
