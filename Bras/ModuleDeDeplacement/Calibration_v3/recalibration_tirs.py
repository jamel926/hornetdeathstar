"""
=========================================================
RECALIBRATION A PARTIR DES ERREURS DE POINTAGE ANNOTEES
=========================================================

Objectif :
---------
Reajuster le modele grille -> angles en utilisant, EN PLUS
des noeuds pointes a la main, les tirs de validation deja
annotes dans resultats/validation_pointage_Calib_2.csv.

Pourquoi ces tirs sont des mesures de calibration :
--------------------------------------------------
Dans correction_calib_mc_mtm.py, la colonne J4/J5 du
fichier de validation n'est pas la consigne : c'est la pose
RELUE sur le robot apres rattrapage (mesure[3], mesure[4]).
Chaque ligne annotee dit donc : "avec le poignet a ces deux
angles la, le faisceau est tombe sur ce carreau la".

C'est exactement la meme information qu'une ligne de
calibration_grille.csv, obtenue dans l'autre sens : au lieu
d'amener le spot sur un noeud connu et de lire les angles,
on impose les angles et on lit le carreau. Le modele ne
fait pas la difference, il ajuste une correspondance
angles <-> impact. Quelle que soit la correction appliquee
a la cible ce jour la, la ligne reste valide : la correction
change la cible visee, pas le couple (angles, impact).

Ce que l'ajout change :
----------------------
Le modele actuel (modele_grille.json) est ajuste sur les 35
noeuds de Calib 2 uniquement. Il colle a ces 35 points
(rms 0,31 carreau) mais laisse un biais sur les 67 tirs :
il annonce le spot 0,45 carreau trop haut en v. C'est le
"-1 carreau en Y" note a la main dans apprentissage_erreurs.csv.

Ce biais est le seul defaut vraiment reparable, et le
reajustement le supprime.

Ce que l'ajout NE change pas :
-----------------------------
Deux termes restent, et aucun modele ne les enlevera :

1. Le decalage d'une seance a l'autre, 0,25 carreau rms
   (0,75 cm). La pose de base relue pendant la campagne
   Calib 2 varie de 0,9 degre sur J2 : le bras ne revient
   pas exactement au meme endroit. Seul un recalage en
   debut de seance le supprimerait.

2. Le bruit de lecture. Les impacts sont notes au carreau
   entier : l'arrondi seul vaut 0,41 carreau rms (1,22 cm),
   soit exactement le residu qui reste sur les tirs apres
   reajustement. Autrement dit ces annotations ne peuvent
   PLUS rien mesurer en dessous de 1,2 cm.

C'est aussi pour cela que le polynome de degre 4 de
modele_correction_Calib_2.json (30 coefficients pour 41
points a erreurs entieres) ajuste surtout cet arrondi. En
validation croisee, le degre 2 est le dernier qui apporte
quelque chose ; au dela, le gain disparait.

Resultat :
---------
                                  erreur attendue (rms)
    modele actuel                   0,58 carreau  1,75 cm
    homographie reajustee           0,47 carreau  1,40 cm
    reajustee + residu degre 2      0,43 carreau  1,28 cm

Ce que le script lit :
---------------------
    resultats/calibration_grille.csv               les noeuds
    resultats/validation_pointage_Calib_2.csv      les tirs annotes
    resultats/tirs_v2_*.csv                        les seances
                                                   faites depuis
                                                   avec test_calibrage_v2.py

Ce que le script produit :
-------------------------
    resultats/modele_grille_v2.json   le nouveau modele
    resultats/grille_angles_v2.csv    J4/J5 pour chaque
                                      noeud du quadrillage

La boucle du dossier : tirer avec test_calibrage_v2.py,
annoter les impacts, relancer ce script. Chaque seance
grossit l'ajustement.

Le JSON garde exactement le format de modele_grille.json :
le copier par dessus suffit, tout le code existant le relit
sans modification et gagne le reajustement.

Le bloc "residu_quadratique" ajoute en fin de fichier est
ignore par calibration_grille.py. Pour en profiter il faut
passer par predireImpactCorrige() et anglesPourPointCorrige()
definis ici.

Utilisation :
------------
    python3 recalibration_tirs.py             # ajuste, compare, ecrit
    python3 recalibration_tirs.py --diagnostic # compare sans rien ecrire

Aucun robot n'est necessaire : le script ne lit que des
fichiers de resultats.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
import json
import math
import sys
import glob

from datetime import datetime


"""
modele_v2 est importe EN PREMIER : c'est lui qui ajoute le
dossier parent au chemin de recherche, sans quoi
calibration_grille resterait introuvable depuis ce
sous-dossier.

Les fonctions d'evaluation et d'inversion du residu vivent
la-bas, pas ici : ce script les ajuste, les trois autres les
utilisent.
"""
from modele_v2 import (
    CHEMIN_MODELE_ORIGINE,
    CHEMIN_MODELE_V2,
    DOSSIER_RESULTATS_ORIGINE,
    DOSSIER_RESULTATS_V2,
    U_MAX,
    V_MAX,
    anglesPourPointCorrige,
    evaluerResidu,
    predireImpactCorrige,
    termesResidu,
)

from calibration_grille import (
    COTE_CARREAU,
    ajusterModele,
    anglesPourPoint,
    calculerResidus,
    moindresCarres,
    predireImpact,
)


# =========================================================
# 2. CONFIGURATION
# =========================================================

SERIE = "Calib 2"


FICHIER_NOEUDS = os.path.join(
    DOSSIER_RESULTATS_ORIGINE,
    "calibration_grille.csv",
)


FICHIER_TIRS = os.path.join(
    DOSSIER_RESULTATS_ORIGINE,
    "validation_pointage_Calib_2.csv",
)


"""
Les seances de test faites avec test_calibrage_v2.py
deposent leurs fichiers ici. Ils ont exactement le meme
entete que le fichier de validation d'origine, donc ils
sont relus par la meme fonction et viennent GROSSIR
l'ajustement.

C'est la boucle du dossier : tirer, annoter, relancer ce
script, le modele s'ameliore.
"""
MOTIF_TIRS_V2 = os.path.join(
    DOSSIER_RESULTATS_V2,
    "tirs_v2_*.csv",
)


FICHIER_MODELE_ACTUEL = CHEMIN_MODELE_ORIGINE


FICHIER_MODELE_NEUF = CHEMIN_MODELE_V2


FICHIER_GRILLE_ANGLES = os.path.join(
    DOSSIER_RESULTATS_V2,
    "grille_angles_v2.csv",
)


# Degre du residu ajoute par dessus l'homographie.
# 2 est le dernier degre qui survive a la validation croisee.
DEGRE_RESIDU = 2


# =========================================================
# 3. LECTURE DES DONNEES
# =========================================================

def chargerNoeuds(chemin=None, serie=SERIE):
    """Lit les noeuds pointes a la main et retourne la liste des correspondances angles / impact."""

    """
    Ici l'operateur a amene le spot sur un noeud connu du
    quadrillage : l'impact est exact, ce sont les angles qui
    portent l'incertitude.
    """

    if chemin is None:
        chemin = FICHIER_NOEUDS

    mesures = []

    with open(chemin, "r", newline="") as fichier:

        for ligne in csv.DictReader(fichier):

            if serie is not None and ligne.get("serie") != serie:
                continue

            try:
                mesures.append({
                    "j4": float(ligne["J4"]),
                    "j5": float(ligne["J5"]),
                    "u": float(ligne["u_carreaux"]),
                    "v": float(ligne["v_carreaux"]),
                    "origine": "noeud",
                    "seance": ligne["horodatage"][:10],
                    "base": [float(ligne["J1"]), float(ligne["J2"]), float(ligne["J3"])],
                })

            except (KeyError, TypeError, ValueError):
                continue

    return mesures


def chargerTousLesTirs():
    """Lit le fichier de tirs d'origine plus toutes les seances faites depuis, et retourne la liste."""

    """
    Les seances de test_calibrage_v2.py s'ajoutent sans rien
    a changer ici : elles ecrivent le meme entete, dans le
    meme dossier, sous un nom qui commence par tirs_v2_.
    """

    tirs = []

    if os.path.exists(FICHIER_TIRS):
        tirs += chargerTirs(FICHIER_TIRS)

    for chemin in sorted(glob.glob(MOTIF_TIRS_V2)):
        ajout = chargerTirs(chemin)
        if ajout:
            print("  seance de test relue : %s (%d tirs)"
                  % (os.path.basename(chemin), len(ajout)))
            tirs += ajout

    return tirs


def chargerTirs(chemin=None):
    """Lit les tirs annotes a la main et retourne la liste des correspondances angles / impact."""

    """
    Ici c'est l'inverse : les angles sont relus sur le robot,
    donc exacts a la repetabilite pres, et c'est l'impact qui
    porte l'incertitude puisqu'il est note au carreau entier.

    Les lignes sans impact note sont les tirs abandonnes :
    on les saute.
    """

    if chemin is None:
        chemin = FICHIER_TIRS

    mesures = []

    with open(chemin, "r", newline="") as fichier:

        for ligne in csv.DictReader(fichier):

            try:
                u = float(ligne["x_reel_carreaux"])
                v = float(ligne["y_reel_carreaux"])
                j4 = float(ligne["J4"])
                j5 = float(ligne["J5"])

            except (KeyError, TypeError, ValueError):
                continue

            mesures.append({
                "j4": j4,
                "j5": j5,
                "u": u,
                "v": v,
                "origine": "tir",
                "seance": ligne["horodatage"][:10],
                "u_demande": float(ligne["x_demande_carreaux"]),
                "v_demande": float(ligne["y_demande_carreaux"]),
            })

    return mesures


# =========================================================
# 4. AJUSTEMENT DU RESIDU POLYNOMIAL
# =========================================================

def ajusterResidu(modele, mesures, degre=DEGRE_RESIDU):
    """Ajuste ce que l'homographie n'explique pas et retourne le bloc residu, ou None."""

    """
    L'homographie suppose un point d'emission fixe. Le laser
    etant decale du centre de rotation du poignet, ce point
    bouge un peu quand la tete tourne : il reste une derive
    douce, surtout en v. Deux surfaces de degre 2 la
    rattrapent.

    Le residu est fonction de la position PREDITE, pas de la
    position mesuree : c'est la seule des deux qui soit
    connue au moment de viser.
    """

    lignes = []
    ecarts_u = []
    ecarts_v = []

    for m in mesures:

        prediction = predireImpact(modele, m["j4"], m["j5"])

        if prediction is None:
            continue

        lignes.append(termesResidu(prediction[0], prediction[1], degre))
        ecarts_u.append(m["u"] - prediction[0])
        ecarts_v.append(m["v"] - prediction[1])

    coefficients_u = moindresCarres(lignes, ecarts_u)
    coefficients_v = moindresCarres(lignes, ecarts_v)

    if coefficients_u is None or coefficients_v is None:
        return None

    return {
        "degre": degre,
        "u_max": U_MAX,
        "v_max": V_MAX,
        "coefficients_u": coefficients_u,
        "coefficients_v": coefficients_v,
        "nb_points": len(lignes),
    }


# =========================================================
# 5. DIAGNOSTIC
# =========================================================

def ecartsModele(modele, mesures, avec_residu=True):
    """Retourne la liste des ecarts signes (predit - mesure) pour chaque point."""

    fonction = predireImpactCorrige if avec_residu else predireImpact

    ecarts = []

    for m in mesures:

        prediction = fonction(modele, m["j4"], m["j5"])

        if prediction is None:
            continue

        ecarts.append((prediction[0] - m["u"], prediction[1] - m["v"], m))

    return ecarts


def decomposerErreur(modele, mesures, avec_residu=True):
    """Separe l'erreur en biais global, decalage de seance et residu interne."""

    """
    Les trois termes ne se corrigent pas de la meme facon :

      - le biais global part avec un reajustement, c'est le
        seul que ce script repare ;
      - le decalage de seance demanderait un recalage en
        debut de seance ;
      - le residu interne est ce qui reste, bruit de lecture
        compris.

    Ils s'ajoutent en carres, d'ou l'erreur attendue en bout
    de tableau.
    """

    ecarts = ecartsModele(modele, mesures, avec_residu)

    if not ecarts:
        return None

    nombre = len(ecarts)

    biais_u = sum(e[0] for e in ecarts) / nombre
    biais_v = sum(e[1] for e in ecarts) / nombre

    # -----------------------------------------------------
    # Moyenne par seance, une fois le biais global retire
    # -----------------------------------------------------

    par_seance = {}

    for du, dv, m in ecarts:
        par_seance.setdefault(m["seance"], []).append((du, dv))

    decalages = {}

    for seance, liste in par_seance.items():
        decalages[seance] = (
            sum(e[0] for e in liste) / len(liste) - biais_u,
            sum(e[1] for e in liste) / len(liste) - biais_v,
        )

    dispersion_seance = math.sqrt(
        sum(d[0] ** 2 + d[1] ** 2 for d in decalages.values()) / len(decalages)
    )

    # -----------------------------------------------------
    # Ce qui reste a l'interieur d'une seance
    # -----------------------------------------------------

    interne = []

    for du, dv, m in ecarts:
        cu = biais_u + decalages[m["seance"]][0]
        cv = biais_v + decalages[m["seance"]][1]
        interne.append((du - cu) ** 2 + (dv - cv) ** 2)

    residu_interne = math.sqrt(sum(interne) / len(interne))

    biais_global = math.hypot(biais_u, biais_v)

    attendu = math.sqrt(biais_global ** 2 + dispersion_seance ** 2 + residu_interne ** 2)

    return {
        "nb_points": nombre,
        "biais_global": biais_global,
        "biais_u": biais_u,
        "biais_v": biais_v,
        "decalages_seance": decalages,
        "dispersion_seance": dispersion_seance,
        "residu_interne": residu_interne,
        "erreur_attendue": attendu,
    }


def afficherDecomposition(titre, decomposition):
    """Affiche le detail d'une decomposition d'erreur."""

    print(titre)

    if decomposition is None:
        print("  aucun point exploitable.")
        return

    def enCm(valeur):
        return valeur * COTE_CARREAU

    print("  points                  : %d" % decomposition["nb_points"])
    print("  biais global            : %.3f carreau  (%.2f cm)   u %+.3f  v %+.3f"
          % (decomposition["biais_global"], enCm(decomposition["biais_global"]),
             decomposition["biais_u"], decomposition["biais_v"]))
    print("  decalage entre seances  : %.3f carreau  (%.2f cm)"
          % (decomposition["dispersion_seance"], enCm(decomposition["dispersion_seance"])))

    for seance in sorted(decomposition["decalages_seance"]):
        d = decomposition["decalages_seance"][seance]
        print("      %s   du %+.3f   dv %+.3f" % (seance, d[0], d[1]))

    print("  residu interne          : %.3f carreau  (%.2f cm)"
          % (decomposition["residu_interne"], enCm(decomposition["residu_interne"])))
    print("  erreur attendue (rms)   : %.3f carreau  (%.2f cm)"
          % (decomposition["erreur_attendue"], enCm(decomposition["erreur_attendue"])))


def bruitDeLecture():
    """Retourne l'ecart type, en carreaux, du seul arrondi au carreau entier."""

    """
    Un impact note au carreau entier porte une erreur
    uniforme sur [-1/2, +1/2] par axe, donc de variance
    1/12 par axe et 2/12 pour la distance. C'est le
    plancher de ce que ces annotations peuvent mesurer.
    """

    return math.sqrt(2.0 / 12.0)


def poseDeBase(mesures):
    """Retourne la pose J1/J2/J3 moyenne des releves et son etendue, ou None."""

    """
    J1 a J3 ne sont jamais commandes : le bras les retrouve
    en revenant en position initiale, et le modele suppose
    qu'il les retrouve a l'identique. Mesurer leur etendue
    sur la campagne dit si cette hypothese tient.

    C'est le meilleur suspect pour le decalage d'une seance
    a l'autre : c'est le seul terme de la geometrie que le
    modele ne controle pas.
    """

    poses = [m["base"] for m in mesures if "base" in m]

    if not poses:
        return None

    nombre = len(poses)

    moyenne = [sum(p[i] for p in poses) / nombre for i in range(3)]
    mini = [min(p[i] for p in poses) for i in range(3)]
    maxi = [max(p[i] for p in poses) for i in range(3)]

    return {
        "moyenne": moyenne,
        "min": mini,
        "max": maxi,
        "etendue": [maxi[i] - mini[i] for i in range(3)],
        "nb_releves": nombre,
    }


# =========================================================
# 6. ECRITURE
# =========================================================

def ecrireModele(modele, chemin=None):
    """Ecrit le nouveau modele au format JSON, et retourne True si l'ecriture a reussi."""

    if chemin is None:
        chemin = FICHIER_MODELE_NEUF

    try:
        os.makedirs(os.path.dirname(chemin), exist_ok=True)

        with open(chemin, "w") as fichier:
            json.dump(modele, fichier, indent=2)

        return True

    except Exception as e:
        print("Impossible d'ecrire le modele.")
        print("Erreur :", e)
        return False


def ecrireGrilleAngles(modele, ancien=None, chemin=None):
    """Ecrit la table J4/J5 de chaque noeud du quadrillage, et retourne le nombre de lignes."""

    """
    C'est la grille au sens litteral : une ligne par noeud,
    les deux angles a envoyer pour l'atteindre, et l'ecart
    avec l'ancien modele pour voir d'un coup d'oeil ce que
    le reajustement a change.

    Les noeuds hors enveloppe sortent avec des champs vides
    plutot que d'etre omis : la table reste rectangulaire et
    dit explicitement ou le modele ne repond pas.
    """

    if chemin is None:
        chemin = FICHIER_GRILLE_ANGLES

    entete = [
        "u_carreaux",
        "v_carreaux",
        "x_cm",
        "y_cm",
        "J4",
        "J5",
        "delta_J4_vs_ancien",
        "delta_J5_vs_ancien",
    ]

    lignes = 0

    os.makedirs(os.path.dirname(chemin), exist_ok=True)

    with open(chemin, "w", newline="") as fichier:

        redacteur = csv.writer(fichier)
        redacteur.writerow(entete)

        for v in range(V_MAX + 1):
            for u in range(U_MAX + 1):

                neuf = anglesPourPointCorrige(modele, float(u), float(v))

                if neuf is None:
                    redacteur.writerow([u, v, u * COTE_CARREAU, v * COTE_CARREAU, "", "", "", ""])
                    continue

                delta_j4 = ""
                delta_j5 = ""

                if ancien is not None:
                    vieux = anglesPourPoint(ancien, float(u), float(v))
                    if vieux is not None:
                        delta_j4 = round(neuf[0] - vieux[0], 3)
                        delta_j5 = round(neuf[1] - vieux[1], 3)

                redacteur.writerow([
                    u,
                    v,
                    u * COTE_CARREAU,
                    v * COTE_CARREAU,
                    round(neuf[0], 3),
                    round(neuf[1], 3),
                    delta_j4,
                    delta_j5,
                ])

                lignes += 1

    return lignes


# =========================================================
# 7. PROGRAMME PRINCIPAL
# =========================================================

def construireModele(mesures, ancien, degre=DEGRE_RESIDU):
    """Ajuste l'homographie puis le residu sur toutes les mesures, et retourne le modele, ou None."""

    observee = poseDeBase(mesures)

    modele = ajusterModele(
        mesures,
        etiquette_robot=ancien.get("robot", "") if ancien else "",
        base=(observee["moyenne"] if observee is not None
              else (ancien.get("base") if ancien else None)),
        commentaire=(
            "reajuste le %s sur les noeuds pointes a la main PLUS les tirs "
            "annotes de validation_pointage_%s.csv"
            % (datetime.now().date().isoformat(), SERIE.replace(" ", "_"))
        ),
    )

    if modele is None:
        return None

    if degre and degre > 0:

        residu = ajusterResidu(modele, mesures, degre)

        if residu is not None:
            modele["residu_quadratique"] = residu

    if observee is not None:
        modele["base_observee"] = observee

    modele["source_donnees"] = {
        "noeuds_pointes": sum(1 for m in mesures if m["origine"] == "noeud"),
        "tirs_annotes": sum(1 for m in mesures if m["origine"] == "tir"),
        "seances": sorted(set(m["seance"] for m in mesures)),
    }

    return modele


def main():
    """Ajuste le nouveau modele, le compare a l'ancien et ecrit les deux fichiers de sortie."""

    diagnostic_seul = "--diagnostic" in sys.argv

    print("=" * 57)
    print("RECALIBRATION A PARTIR DES ERREURS DE POINTAGE")
    print("=" * 57)
    print()

    # -----------------------------------------------------
    # Lecture
    # -----------------------------------------------------

    noeuds = chargerNoeuds()
    tirs = chargerTousLesTirs()

    if not noeuds and not tirs:
        print("ERREUR : aucune mesure lue. Verifier les fichiers de resultats.")
        return 1

    mesures = noeuds + tirs

    print("Donnees :")
    print("  noeuds pointes a la main (%s) : %d" % (SERIE, len(noeuds)))
    print("  tirs annotes a la main         : %d" % len(tirs))
    print("  total                          : %d correspondances" % len(mesures))
    print()

    observee = poseDeBase(mesures)

    if observee is not None:
        print("Pose de base relevee pendant la campagne (jamais commandee) :")
        print("  moyenne  J1 %+.2f   J2 %+.2f   J3 %+.2f"
              % tuple(observee["moyenne"]))
        print("  etendue  J1 %5.2f   J2 %5.2f   J3 %5.2f  degres"
              % tuple(observee["etendue"]))
        print("  C'est la seule partie de la geometrie que le modele ne")
        print("  controle pas : premier suspect pour le decalage de seance.")
        print()

    try:
        with open(FICHIER_MODELE_ACTUEL, "r") as fichier:
            ancien = json.load(fichier)
    except Exception:
        print("ATTENTION : modele_grille.json illisible, pas de comparaison possible.")
        ancien = None

    # -----------------------------------------------------
    # Ajustement
    # -----------------------------------------------------

    neuf = construireModele(mesures, ancien)

    if neuf is None:
        print("ERREUR : ajustement impossible.")
        return 1

    sans_residu = dict(neuf)
    sans_residu.pop("residu_quadratique", None)

    # -----------------------------------------------------
    # Comparaison
    # -----------------------------------------------------

    print("-" * 57)
    print("COMPARAISON SUR LES %d CORRESPONDANCES" % len(mesures))
    print("-" * 57)
    print()

    if ancien is not None:
        afficherDecomposition("Modele actuel (ajuste sur les %d noeuds seuls) :"
                              % ancien.get("nb_points", 0),
                              decomposerErreur(ancien, mesures, avec_residu=False))
        print()

    afficherDecomposition("Homographie reajustee sur tout :",
                          decomposerErreur(sans_residu, mesures, avec_residu=False))
    print()

    afficherDecomposition("Reajustee + residu de degre %d :" % DEGRE_RESIDU,
                          decomposerErreur(neuf, mesures, avec_residu=True))
    print()

    # -----------------------------------------------------
    # Ou est le plancher
    # -----------------------------------------------------

    ecarts_tirs = ecartsModele(neuf, tirs, avec_residu=True)

    if ecarts_tirs:

        nombre = len(ecarts_tirs)
        bu = sum(e[0] for e in ecarts_tirs) / nombre
        bv = sum(e[1] for e in ecarts_tirs) / nombre

        disperse = math.sqrt(
            sum((e[0] - bu) ** 2 + (e[1] - bv) ** 2 for e in ecarts_tirs) / nombre
        )

        plancher = bruitDeLecture()

        print("-" * 57)
        print("CE QUE LES ANNOTATIONS PEUVENT ENCORE MESURER")
        print("-" * 57)
        print()
        print("  dispersion restante sur les tirs : %.3f carreau (%.2f cm)"
              % (disperse, disperse * COTE_CARREAU))
        print("  arrondi au carreau entier seul   : %.3f carreau (%.2f cm)"
              % (plancher, plancher * COTE_CARREAU))

        reste = disperse ** 2 - plancher ** 2

        if reste <= 0.0:
            print()
            print("  La dispersion restante est AU NIVEAU du seul arrondi de")
            print("  lecture. Ces annotations ne portent plus d'information")
            print("  exploitable : en rajouter du meme type ne fera pas mieux.")
            print("  Pour descendre, il faut noter l'impact au demi-carreau")
            print("  ou au centimetre, pas au carreau.")
        else:
            print()
            print("  part non expliquee par la lecture : %.3f carreau (%.2f cm)"
                  % (math.sqrt(reste), math.sqrt(reste) * COTE_CARREAU))

        print()

    # -----------------------------------------------------
    # Ecriture
    # -----------------------------------------------------

    if diagnostic_seul:
        print("Mode diagnostic : aucun fichier ecrit.")
        return 0

    neuf["residus"] = calculerResidus(neuf, mesures)

    if not ecrireModele(neuf):
        return 1

    lignes = ecrireGrilleAngles(neuf, ancien)

    print("-" * 57)
    print("FICHIERS ECRITS")
    print("-" * 57)
    print()
    print("  %s" % FICHIER_MODELE_NEUF)
    print("      nouveau modele, meme format que modele_grille.json.")
    print("      Le copier par dessus pour que tout le code existant")
    print("      en profite (sans le residu, qui demande les fonctions")
    print("      de ce fichier).")
    print()
    print("  %s" % FICHIER_GRILLE_ANGLES)
    print("      %d noeuds du quadrillage avec leurs angles J4/J5," % lignes)
    print("      et l'ecart avec l'ancien modele.")
    print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
