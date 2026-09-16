"""
=========================================================
TEST DE LA CALIBRATION - POINTER UN POINT DEMANDE
=========================================================

Objectif :
---------
Verifier la calibration produite par calibration_grille.py :
on donne des coordonnees sur le quadrillage, le robot
pointe dessus.

Le script sert aussi d'outil de MESURE : apres chaque tir,
on peut saisir l'endroit ou le spot est reellement tombe.
L'ecart est enregistre, et c'est ce chiffre qui dit ce que
vaut la calibration.

Pourquoi la base est verifiee au demarrage :
-------------------------------------------
Le modele decrit la position du spot pour une pose de base
donnee. Si J1, J2 ou J3 ont bouge depuis la calibration,
tout le modele est faux. Le script refuse alors de
continuer plutot que de viser a cote sans le dire.

Pourquoi le tir corrige sa propre consigne :
-------------------------------------------
Le bras ne s'arrete pas sur la consigne, mais un peu en
dessous. Le 11/09, sur les trois tirs enregistres, le
deficit valait 0,7 a 1,8 degre sur les deux axes, soit 1,5
a 2,3 cm au sol -- plus que l'erreur du modele lui-meme.

Le tir passe donc par viserAngles(), qui relit la pose
atteinte et rallonge la consigne tant que l'angle demande
n'est pas tenu. L'ecart residuel affiche apres chaque tir
ne vient plus de la mecanique : c'est celui du modele.

Utilisation :
------------
    python3 calibration_test.py

ATTENTION :
----------
Pointeur rouge classe 2 uniquement. Le laser de puissance
reste DEBRANCHE.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
import re
from datetime import datetime

"""
Les fonctions du modele et la couche robot viennent du
script de calibration, qui est dans le meme dossier. Son
code executable est sous if __name__ == "__main__" : cet
import n'ouvre donc aucun port serie et ne declenche
aucun mouvement.
"""

from calibration_grille import (
    COTE_CARREAU,
    DOSSIER_RESULTATS,
    J4_MIN, J4_MAX, J5_MIN, J5_MAX,
    MARGE_ENVELOPPE,
    TOLERANCE_BASE,
    TOLERANCE_POINTAGE,
    allumerPointeur,
    anglesPourPoint,
    viserAngles,
    ajusterModele,
    calculerResidus,
    chargerMesures,
    connecterRobot,
    dansEnveloppe,
    eteindrePointeur,
    lireAngles,
    lireTexte,
    predireImpact,
)

from PU_PositionInitiale import allerPositionInitiale

# =========================================================
# 2. PARAMETRES
# =========================================================

FICHIER_VALIDATION = os.path.join(DOSSIER_RESULTATS, "validation_pointage.csv")
FICHIER_MESURES = os.path.join(DOSSIER_RESULTATS, "calibration_grille.csv")

ENTETE_VALIDATION = [
    "horodatage", "u_demande_carreaux", "v_demande_carreaux",
    "J4", "J5", "u_reel_carreaux", "v_reel_carreaux",
    "ecart_carreaux",
]


# =========================================================
# 3. VERIFICATIONS
# =========================================================

def verifierBase(mc, modele, tolerance=TOLERANCE_BASE):
    """Compare la base courante a celle de la calibration et retourne True si elle n'a pas bouge."""

    reference = modele.get("base")

    if not reference:
        print("ATTENTION : le modele ne contient pas la pose de base.")
        print("Impossible de verifier que le bras n'a pas bouge depuis la calibration.")
        return True

    mesure = lireAngles(mc)

    if mesure is None:
        print("Lecture des angles impossible.")
        return False

    ecarts = [abs(mesure[i] - reference[i]) for i in range(3)]

    print()
    print("Base de calibration :", [round(a, 2) for a in reference])
    print("Base courante       :", [round(mesure[i], 2) for i in range(3)])
    print("Ecarts              :", [round(e, 2) for e in ecarts])

    if max(ecarts) > tolerance:
        print()
        print("REFUS : la base a bouge de %.2f deg depuis la calibration." % max(ecarts))
        print("Le modele ne decrit plus la geometrie actuelle.")
        print("Remettre le bras dans sa pose de calibration, ou refaire une campagne.")
        return False

    print("Base conforme.")
    return True


# =========================================================
# 4. ENREGISTREMENT DES TIRS
# =========================================================

def enregistrerTir(demande, angles, reel, chemin=None):
    """Ajoute une ligne au fichier de validation et retourne True si l'ecriture a reussi."""

    if reel is None:
        ecart = ""
        u_reel = ""
        v_reel = ""
    else:
        ecart = round(((reel[0] - demande[0]) ** 2 + (reel[1] - demande[1]) ** 2) ** 0.5, 3)
        u_reel = round(reel[0], 3)
        v_reel = round(reel[1], 3)

    try:
        os.makedirs(DOSSIER_RESULTATS, exist_ok=True)

        nouveau = not os.path.exists(FICHIER_VALIDATION)

        with open(FICHIER_VALIDATION, "a", newline="") as fichier:

            redacteur = csv.writer(fichier)

            if nouveau:
                redacteur.writerow(ENTETE_VALIDATION)

            redacteur.writerow([
                datetime.now().isoformat(timespec="seconds"),
                round(demande[0], 3), round(demande[1], 3),
                round(angles[0], 3), round(angles[1], 3),
                u_reel, v_reel, ecart,
            ])

        return True

    except Exception as e:
        print("Impossible d'enregistrer le tir :", e)
        return False


# =========================================================
# 5. SELECTION DE LA CALIBRATION
# =========================================================

def nomFichierSecurise(serie):
    """Transforme le nom d'une serie en nom de fichier sans caracteres dangereux."""
    propre = re.sub(r"[^A-Za-z0-9._-]+", "_", serie).strip("._")
    return propre or "calibration"


def listerSeriesCalibrations():
    """Retourne les series presentes dans le CSV, avec quelques statistiques utiles."""
    series = {}

    try:
        with open(FICHIER_MESURES, "r", newline="") as fichier:
            for ligne in csv.DictReader(fichier):
                serie = (ligne.get("serie") or "").strip()
                usage = (ligne.get("usage") or "").strip()
                if not serie:
                    continue

                if serie not in series:
                    series[serie] = {
                        "ajustement": 0,
                        "validation": 0,
                        "u_min": None,
                        "u_max": None,
                        "v_min": None,
                        "v_max": None,
                    }

                info = series[serie]
                if usage in ("ajustement", "validation"):
                    info[usage] += 1

                try:
                    if ligne.get("u_carreaux") and ligne.get("v_carreaux"):
                        u = float(ligne["u_carreaux"])
                        v = float(ligne["v_carreaux"])
                    elif ligne.get("u_cm") and ligne.get("v_cm"):
                        u = float(ligne["u_cm"]) / COTE_CARREAU
                        v = float(ligne["v_cm"]) / COTE_CARREAU
                    else:
                        continue

                    info["u_min"] = u if info["u_min"] is None else min(info["u_min"], u)
                    info["u_max"] = u if info["u_max"] is None else max(info["u_max"], u)
                    info["v_min"] = v if info["v_min"] is None else min(info["v_min"], v)
                    info["v_max"] = v if info["v_max"] is None else max(info["v_max"], v)
                except (ValueError, TypeError):
                    continue

    except FileNotFoundError:
        print("Fichier de mesures introuvable :", FICHIER_MESURES)
        return {}
    except Exception as e:
        print("Impossible de lire les calibrations :", e)
        return {}

    return series


def chargerBaseSerie(serie):
    """Lit J1/J2/J3 dans la campagne choisie pour verifier la base du robot."""
    try:
        with open(FICHIER_MESURES, "r", newline="") as fichier:
            for ligne in csv.DictReader(fichier):
                if (ligne.get("serie") or "").strip() != serie:
                    continue
                try:
                    return [float(ligne["J1"]), float(ligne["J2"]), float(ligne["J3"])]
                except (ValueError, KeyError):
                    continue
    except Exception as e:
        print("Impossible de lire la base de la calibration :", e)

    return None


def construireModeleSerie(serie):
    """Reconstruit en memoire le modele de la serie choisie a partir du CSV."""
    ajustement = chargerMesures(usage="ajustement", serie=serie)
    validation = chargerMesures(usage="validation", serie=serie)

    if len(ajustement) < 5:
        print("ERREUR : la calibration '%s' ne contient que %d point(s) d'ajustement." %
              (serie, len(ajustement)))
        print("Il en faut au moins 5 pour reconstruire le modele.")
        return None

    base_reference = chargerBaseSerie(serie)
    modele = ajusterModele(ajustement, base=base_reference)

    if modele is None:
        print("ERREUR : impossible de reconstruire la calibration '%s'." % serie)
        return None

    # Le nom de serie est ajoute au modele pour l'affichage et le suivi.
    modele["serie"] = serie

    if validation:
        modele["residus_validation"] = calculerResidus(modele, validation)

    return modele


def choisirCalibration():
    """Affiche les calibrations disponibles et retourne la serie choisie."""
    series = listerSeriesCalibrations()

    if not series:
        print("Aucune calibration trouvee dans :", FICHIER_MESURES)
        return None

    noms = list(series.keys())

    print()
    print("=========================================")
    print("CHOIX DE LA CALIBRATION")
    print("=========================================")
    print("Les calibrations sont selectionnees par leur nom de serie.")
    print()

    for indice, serie in enumerate(noms, 1):
        info = series[serie]
        if info["u_min"] is not None:
            zone = "u %.1f..%.1f / v %.1f..%.1f carreaux" % (
                info["u_min"], info["u_max"], info["v_min"], info["v_max"])
        else:
            zone = "zone inconnue"

        print("  %d) %s" % (indice, serie))
        print("     ajustement : %d | validation : %d | %s" %
              (info["ajustement"], info["validation"], zone))

    print()
    print("Taper le numero ou le nom exact de la calibration.")
    print("Taper fin pour quitter.")

    while True:
        reponse = lireTexte("calibration > ").strip()

        if reponse.lower() in ("fin", "f", "q"):
            return None

        if reponse.isdigit():
            indice = int(reponse)
            if 1 <= indice <= len(noms):
                return noms[indice - 1]

        for nom in noms:
            if reponse.lower() == nom.lower():
                return nom

        print("  Choix invalide. Entrer un numero entre 1 et %d, ou le nom exact." % len(noms))


# =========================================================
# 5. DIALOGUE
# =========================================================

def lireCible():
    """Lit le point a viser en carreaux et le retourne, ou None pour terminer."""

    """
    Tout se compte en carreaux, comme pendant la
    calibration. Accepter deux unites dans une meme chaine
    de mesure finit par produire un facteur d'echelle que
    rien ne signale -- ici il vaudrait 3.
    """

    print()
    print("Point a viser : deux nombres en CARREAUX (ex : 4 -2).")
    print("Taper fin pour terminer.")

    while True:

        reponse = lireTexte("cible > ")

        if reponse.lower() in ("fin", "f", "q"):
            return None

        morceaux = reponse.replace(",", " ").split()

        if len(morceaux) != 2:
            print("  Il faut deux nombres separes par un espace.")
            continue

        try:
            return (float(morceaux[0]), float(morceaux[1]))

        except ValueError:
            print("  Saisie illisible.")


def lireImpactReel(demande):
    """Demande ou le spot est reellement tombe, en carreaux, et le retourne ou None."""

    print("  Ou le spot est-il tombe ? deux nombres en carreaux,")
    print("  ou Entree si le releve n'est pas fait.")

    reponse = lireTexte("  impact reel > ")

    if not reponse:
        return None

    morceaux = reponse.replace(",", " ").split()

    if len(morceaux) != 2:
        print("  Saisie ignoree.")
        return None

    try:
        reel = (float(morceaux[0]), float(morceaux[1]))

    except ValueError:
        print("  Saisie ignoree.")
        return None

    ecart = ((reel[0] - demande[0]) ** 2 + (reel[1] - demande[1]) ** 2) ** 0.5

    print("  Ecart de pointage : %.2f carreaux (%.1f cm)" % (ecart, ecart * COTE_CARREAU))

    return reel



# =========================================================
# 6. PROGRAMME
# =========================================================

def main():
    """Demande la calibration, verifie la base, puis pointe les points demandes."""

    print("=========================================")
    print("TEST DE LA CALIBRATION")
    print("=========================================")
    print()
    print("SECURITE : pointeur rouge classe 2, laser de puissance DEBRANCHE.")

    # ---------------------------------------------------------
    # Choix de la calibration AVANT tout mouvement du robot.
    # ---------------------------------------------------------
    serie = choisirCalibration()

    if serie is None:
        print("Aucune calibration selectionnee.")
        return 0

    print()
    print("Calibration selectionnee :", serie)
    print("Reconstruction du modele depuis les points de cette serie...")

    modele = construireModeleSerie(serie)

    if modele is None:
        return 1

    print()
    print("Calibration '%s' chargee : %d points d'ajustement" %
          (serie, modele.get("nb_points", 0)))
    print("Zone calibree : u %.1f..%.1f, v %.1f..%.1f carreaux" % (
        modele["enveloppe"]["u_min"], modele["enveloppe"]["u_max"],
        modele["enveloppe"]["v_min"], modele["enveloppe"]["v_max"]))

    residus = modele.get("residus")
    if residus:
        print("Erreur ajustement : rms %.2f carreaux, max %.2f carreaux" %
              (residus["rms_carreaux"], residus["max_carreaux"]))

    residus_validation = modele.get("residus_validation")
    if residus_validation:
        print("Erreur validation  : rms %.2f carreaux, max %.2f carreaux" %
              (residus_validation["rms_carreaux"], residus_validation["max_carreaux"]))
    else:
        print("Pas de validation pour cette calibration.")

    # ---------------------------------------------------------
    # Connexion au robot seulement apres le choix de calibration.
    # ---------------------------------------------------------
    mc = connecterRobot()
    allerPositionInitiale(mc)

    if mc is None:
        return 1

    if not verifierBase(mc, modele):
        return 1

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    base = angles[0:3]
    j6 = angles[5]

    fichier_validation = os.path.join(
        DOSSIER_RESULTATS,
        "validation_pointage_%s.csv" % nomFichierSecurise(serie),
    )

    print()
    print("Les tirs de cette session seront enregistres dans :")
    print(" ", fichier_validation)

    # ---------------------------------------------------------
    # Correction apprise, s'il y en a une.
    # ---------------------------------------------------------

    """
    L'import est fait ici, et pas en tete de fichier, parce
    que apprentissage_correction importe ce module : un
    import au chargement serait circulaire.
    """

    from apprentissage_correction import chargerCorrection, cibleCorrigee

    correction = chargerCorrection()

    if correction is None:
        print()
        print("Pas de correction apprise : le modele est utilise tel quel.")

    else:
        print()
        print("Correction apprise '%s' appliquee (%s, %d points)."
              % (correction.get("type", "?"), correction.get("serie", "serie inconnue"),
                 correction.get("nb_points", 0)))
        print("Pour viser sans elle, renommer", os.path.basename(
            os.path.join(DOSSIER_RESULTATS, "correction_pointage.json")))

    try:
        if not allumerPointeur(mc):
            print("Le pointeur ne s'allume pas.")

        while True:

            demande = lireCible()

            if demande is None:
                break

            u, v = demande

            # -------------------------------------------------
            # Verification explicite de l'enveloppe choisie.
            # -------------------------------------------------
            if not dansEnveloppe(modele, u, v):
                e = modele["enveloppe"]
                print("  Point hors de la zone de la calibration '%s'." % serie)
                print("  Zone autorisee : u %.1f..%.1f, v %.1f..%.1f carreaux."
                      % (e["u_min"], e["u_max"], e["v_min"], e["v_max"]))
                print("  Si ce point appartient a une autre zone, relancer le programme")
                print("  et choisir la calibration correspondante.")
                continue

            # La correction deplace la cible donnee au modele ;
            # le modele lui-meme n'est pas touche.
            vise_u, vise_v = cibleCorrigee(correction, u, v)

            if (vise_u, vise_v) != (u, v):
                print("  Cible corrigee : (%.2f, %.2f) carreaux" % (vise_u, vise_v))

            resultat = anglesPourPoint(modele, vise_u, vise_v)

            if resultat is None:
                print("  Impossible de calculer J4/J5 pour ce point avec cette calibration.")
                print("  Le point est peut-etre hors du domaine angulaire inverse.")
                continue

            j4, j5 = resultat

            print("  Angles calcules : J4 = %+.2f   J5 = %+.2f" % (j4, j5))

            vise, mesure = viserAngles(mc, base, j4, j5, j6)

            if mesure is None:
                print("  Mouvement echoue.")
                continue

            if not vise:
                print("  ATTENTION : la tete se repose a %+.2f / %+.2f deg de l'angle demande"
                      % (mesure[3] - j4, mesure[4] - j5))
                print("  (tolerance %.2f deg). Le rattrapage n'a pas converge."
                      % TOLERANCE_POINTAGE)

            # -------------------------------------------------
            # Ecart de reproduction
            # -------------------------------------------------

            prediction = predireImpact(modele, mesure[3], mesure[4])

            if prediction is not None:
                # Comparer a la cible REELLEMENT donnee au modele :
                # sinon la correction se lirait comme une derive.
                derive = ((prediction[0] - vise_u) ** 2 + (prediction[1] - vise_v) ** 2) ** 0.5
                if derive > 0.1:
                    print("  Angles atteints : J4 = %+.2f  J5 = %+.2f  -> %.2f carreaux de derive mecanique"
                          % (mesure[3], mesure[4], derive))

            reel = lireImpactReel(demande)

            enregistrerTir(
                demande,
                (mesure[3], mesure[4]),
                reel,
                chemin=fichier_validation,
            )

    finally:

        eteindrePointeur(mc)
        print()
        print("Pointeur eteint. Moteurs laisses sous tension.")
        print("Calibration utilisee :", serie)
        print("Tirs enregistres dans :", fichier_validation)

    return 0


# =========================================================
# 7. EXECUTION AUTONOME
# =========================================================

if __name__ == "__main__":

    try:
        code_retour = main()

    except KeyboardInterrupt:
        print()
        print("Programme interrompu par l'operateur.")
        code_retour = 1

    print()
    print("Programme termine.")

    raise SystemExit(code_retour)