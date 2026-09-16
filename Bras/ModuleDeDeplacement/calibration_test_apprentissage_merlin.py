"""
TEST DE LA CALIBRATION - POINTER UN POINT DEMANDE

Cette version enregistre egalement les mesures dans :
    resultats/apprentissage_erreurs.csv

Ces donnees sont utilisees ensuite par calib_apprentissage.py
pour apprendre un modele polynomial de correction.

Principe de correction appris :
    erreur_x = x_mesuree - x_visee
    erreur_y = y_mesuree - y_visee

Pour une prochaine cible :
    x_corrige = x_visee - erreur_x_modele(x_visee, y_visee)
    y_corrige = y_visee - erreur_y_modele(x_visee, y_visee)
"""

import os
import csv
import re
import json
from datetime import datetime

import matplotlib.pyplot as plt

from calibration_grille import (
    COTE_CARREAU,
    DOSSIER_RESULTATS,
    J4_MIN,
    J4_MAX,
    J5_MIN,
    J5_MAX,
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

from calib_test_merlin import verifierBase
from PU_PositionInitiale import allerPositionInitiale

# =========================================================
SERIE = "Calib 2"


def chemin_modele_correction():
    nom = "Calib_2"

    return os.path.join(
        DOSSIER_RESULTATS,
        "modele_correction_%s.json" % nom
    )


def chargerModeleCorrection():
    chemin = chemin_modele_correction()

    if not os.path.exists(chemin):
        print()
        print("ATTENTION : aucun modele de correction trouve.")
        print("Fichier attendu :")
        print(" ", chemin)
        print("Le ciblage fonctionnera sans correction.")
        return None

    try:
        with open(chemin, "r") as fichier:
            modele = json.load(fichier)

        print()
        print("Modele de correction charge :")
        print(" ", chemin)
        print(
            "Degre polynomial :",
            modele["degre"]
        )
        print(
            "Nombre de mesures :",
            modele["nb_points"]
        )

        return modele

    except Exception as e:
        print(
            "Impossible de charger le modele de correction :",
            e
        )
        return None
    

def predireCorrection(modele, x, y):
    """
    Retourne l'erreur X et Y predite par le modele
    pour la position visee (x, y).
    """

    if modele is None:
        return 0.0, 0.0

    centre_x = float(modele["centre_x"])
    centre_y = float(modele["centre_y"])

    echelle_x = float(modele["echelle_x"])
    echelle_y = float(modele["echelle_y"])

    x_n = (x - centre_x) / echelle_x
    y_n = (y - centre_y) / echelle_y

    termes = modele["termes"]

    valeurs = []

    for i, j in termes:
        valeurs.append(
            (x_n ** int(i)) *
            (y_n ** int(j))
        )

    erreur_x = sum(
        float(c) * v
        for c, v in zip(
            modele["coefficients_erreur_x"],
            valeurs
        )
    )

    erreur_y = sum(
        float(c) * v
        for c, v in zip(
            modele["coefficients_erreur_y"],
            valeurs
        )
    )

    return (
        float(erreur_x),
        float(erreur_y)
    )

# =========================================================
# 1. FICHIERS
# =========================================================

FICHIER_VALIDATION = os.path.join(
    DOSSIER_RESULTATS,
    "validation_pointage.csv",
)

FICHIER_MESURES = os.path.join(
    DOSSIER_RESULTATS,
    "calibration_grille.csv",
)

FICHIER_APPRENTISSAGE = os.path.join(
    DOSSIER_RESULTATS,
    "apprentissage_erreurs.csv",
)

ENTETE_VALIDATION = [
    "horodatage",
    "x_demande_carreaux",
    "y_demande_carreaux",
    "J4",
    "J5",
    "x_reel_carreaux",
    "y_reel_carreaux",
    "ecart_carreaux",
]

ENTETE_APPRENTISSAGE = [
    "horodatage",
    "serie",
    "x_vise",
    "y_vise",
    "x_mesure",
    "y_mesure",
    "erreur_x",
    "erreur_y",
]


# =========================================================
# 2. MODELE DE CORRECTION
# =========================================================

def nom_fichier_securise(serie):
    propre = re.sub(
        r"[^A-Za-z0-9._-]+",
        "_",
        serie,
    ).strip("._")

    return propre or "calibration"


def chemin_modele_correction(serie):
    return os.path.join(
        DOSSIER_RESULTATS,
        "modele_correction_%s.json"
        % nom_fichier_securise(serie),
    )


def chargerModeleCorrection(serie):
    """
    Charge le modele polynomial appris pour une serie.

    Retourne None si aucun modele n'a encore ete appris.
    """
    chemin = chemin_modele_correction(
        serie
    )

    if not os.path.exists(chemin):
        return None

    try:
        with open(
            chemin,
            "r",
        ) as fichier:

            modele = json.load(fichier)

        if modele.get("serie") != serie:
            print(
                "ATTENTION : le modele de correction ne correspond "
                "pas a la serie courante."
            )
            return None

        return modele

    except Exception as e:
        print(
            "Impossible de charger le modele de correction :",
            e,
        )

        return None


def predireCorrection(modele, x, y):
    """
    Calcule l'erreur predite par le modele polynomial.

    Retourne :
        erreur_x_predite, erreur_y_predite

    en CARREAUX.
    """
    if modele is None:
        return 0.0, 0.0

    centre_x = float(
        modele["centre_x"]
    )
    centre_y = float(
        modele["centre_y"]
    )

    echelle_x = float(
        modele["echelle_x"]
    )
    echelle_y = float(
        modele["echelle_y"]
    )

    x_n = (
        (x - centre_x)
        / echelle_x
    )

    y_n = (
        (y - centre_y)
        / echelle_y
    )

    termes = modele["termes"]

    base = []

    for i, j in termes:
        base.append(
            (x_n ** int(i))
            * (y_n ** int(j))
        )

    erreur_x = float(
        sum(
            float(c) * v
            for c, v in zip(
                modele["coefficients_erreur_x"],
                base,
            )
        )
    )

    erreur_y = float(
        sum(
            float(c) * v
            for c, v in zip(
                modele["coefficients_erreur_y"],
                base,
            )
        )
    )

    return (
        erreur_x,
        erreur_y,
    )


def corrigerCible(modele, x, y):
    """
    Compense l'erreur predite.

    Si le modele predit :
        mesure = consigne + erreur

    alors la nouvelle consigne devient :
        consigne_corrigee = consigne - erreur
    """
    erreur_x, erreur_y = predireCorrection(
        modele,
        x,
        y,
    )

    x_corrige = x - erreur_x
    y_corrige = y - erreur_y

    return (
        x_corrige,
        y_corrige,
        erreur_x,
        erreur_y,
    )


# =========================================================
# 3. ENREGISTREMENT
# =========================================================

def enregistrerTir(
    demande,
    angles,
    reel,
    chemin=None,
):
    if reel is None:
        ecart = ""
        x_reel = ""
        y_reel = ""
    else:
        ecart = round(
            (
                (reel[0] - demande[0]) ** 2
                + (reel[1] - demande[1]) ** 2
            ) ** 0.5,
            3,
        )

        x_reel = round(
            reel[0],
            3,
        )

        y_reel = round(
            reel[1],
            3,
        )

    try:
        os.makedirs(
            DOSSIER_RESULTATS,
            exist_ok=True,
        )

        fichier_cible = (
            chemin
            if chemin is not None
            else FICHIER_VALIDATION
        )

        nouveau = not os.path.exists(
            fichier_cible
        )

        with open(
            fichier_cible,
            "a",
            newline="",
        ) as fichier:

            redacteur = csv.writer(
                fichier
            )

            if nouveau:
                redacteur.writerow(
                    ENTETE_VALIDATION
                )

            redacteur.writerow([
                datetime.now().isoformat(
                    timespec="seconds"
                ),
                round(demande[0], 3),
                round(demande[1], 3),
                round(angles[0], 3),
                round(angles[1], 3),
                x_reel,
                y_reel,
                ecart,
            ])

        return True

    except Exception as e:
        print(
            "Impossible d'enregistrer le tir :",
            e,
        )
        return False


def enregistrerMesureApprentissage(
    serie,
    demande,
    reel,
):
    """
    Enregistre une observation complete pour l'apprentissage.
    """
    if reel is None:
        return False

    x_vise, y_vise = demande
    x_mesure, y_mesure = reel

    erreur_x = (
        x_mesure
        - x_vise
    )

    erreur_y = (
        y_mesure
        - y_vise
    )

    try:
        os.makedirs(
            DOSSIER_RESULTATS,
            exist_ok=True,
        )

        nouveau = not os.path.exists(
            FICHIER_APPRENTISSAGE
        )

        with open(
            FICHIER_APPRENTISSAGE,
            "a",
            newline="",
        ) as fichier:

            redacteur = csv.writer(
                fichier
            )

            if nouveau:
                redacteur.writerow(
                    ENTETE_APPRENTISSAGE
                )

            redacteur.writerow([
                datetime.now().isoformat(
                    timespec="seconds"
                ),
                serie,
                round(x_vise, 5),
                round(y_vise, 5),
                round(x_mesure, 5),
                round(y_mesure, 5),
                round(erreur_x, 5),
                round(erreur_y, 5),
            ])

        return True

    except Exception as e:
        print(
            "Impossible d'enregistrer la mesure d'apprentissage :",
            e,
        )
        return False


# =========================================================
# 4. SELECTION DE LA CALIBRATION
# =========================================================

def listerSeriesCalibrations():
    series = {}

    try:
        with open(
            FICHIER_MESURES,
            "r",
            newline="",
        ) as fichier:

            for ligne in csv.DictReader(
                fichier
            ):
                serie = (
                    ligne.get("serie") or ""
                ).strip()

                usage = (
                    ligne.get("usage") or ""
                ).strip()

                if not serie:
                    continue

                if serie not in series:
                    series[serie] = {
                        "ajustement": 0,
                        "validation": 0,
                        "x_min": None,
                        "x_max": None,
                        "y_min": None,
                        "y_max": None,
                    }

                info = series[serie]

                if usage in (
                    "ajustement",
                    "validation",
                ):
                    info[usage] += 1

                try:
                    if (
                        ligne.get("u_carreaux")
                        and ligne.get("v_carreaux")
                    ):
                        x = float(
                            ligne["u_carreaux"]
                        )
                        y = float(
                            ligne["v_carreaux"]
                        )

                    elif (
                        ligne.get("u_cm")
                        and ligne.get("v_cm")
                    ):
                        x = (
                            float(ligne["u_cm"])
                            / COTE_CARREAU
                        )
                        y = (
                            float(ligne["v_cm"])
                            / COTE_CARREAU
                        )
                    else:
                        continue

                    info["x_min"] = (
                        x
                        if info["x_min"] is None
                        else min(
                            info["x_min"],
                            x,
                        )
                    )

                    info["x_max"] = (
                        x
                        if info["x_max"] is None
                        else max(
                            info["x_max"],
                            x,
                        )
                    )

                    info["y_min"] = (
                        y
                        if info["y_min"] is None
                        else min(
                            info["y_min"],
                            y,
                        )
                    )

                    info["y_max"] = (
                        y
                        if info["y_max"] is None
                        else max(
                            info["y_max"],
                            y,
                        )
                    )

                except (
                    ValueError,
                    TypeError,
                ):
                    continue

    except FileNotFoundError:
        print(
            "Fichier de mesures introuvable :",
            FICHIER_MESURES,
        )
        return {}

    except Exception as e:
        print(
            "Impossible de lire les calibrations :",
            e,
        )
        return {}

    return series


def chargerBaseSerie(serie):
    try:
        with open(
            FICHIER_MESURES,
            "r",
            newline="",
        ) as fichier:

            for ligne in csv.DictReader(
                fichier
            ):

                if (
                    ligne.get("serie") or ""
                ).strip() != serie:
                    continue

                try:
                    return [
                        float(ligne["J1"]),
                        float(ligne["J2"]),
                        float(ligne["J3"]),
                    ]
                except (
                    ValueError,
                    KeyError,
                ):
                    continue

    except Exception as e:
        print(
            "Impossible de lire la base de la calibration :",
            e,
        )

    return None


def construireModeleSerie(serie):
    ajustement = chargerMesures(
        usage="ajustement",
        serie=serie,
    )

    validation = chargerMesures(
        usage="validation",
        serie=serie,
    )

    if len(ajustement) < 5:
        print(
            "ERREUR : la calibration '%s' ne contient "
            "que %d point(s) d'ajustement."
            % (
                serie,
                len(ajustement),
            )
        )

        print(
            "Il en faut au moins 5 pour reconstruire le modele."
        )
        return None

    base_reference = chargerBaseSerie(
        serie
    )

    modele = ajusterModele(
        ajustement,
        base=base_reference,
    )

    if modele is None:
        print(
            "ERREUR : impossible de reconstruire "
            "la calibration '%s'."
            % serie
        )
        return None

    modele["serie"] = serie

    if validation:
        modele["residus_validation"] = (
            calculerResidus(
                modele,
                validation,
            )
        )

    return modele


def choisirCalibration():
    series = listerSeriesCalibrations()

    if not series:
        print(
            "Aucune calibration trouvee dans :",
            FICHIER_MESURES,
        )
        return None

    noms = list(
        series.keys()
    )

    print()
    print("=========================================")
    print("CHOIX DE LA CALIBRATION")
    print("=========================================")

    for indice, serie in enumerate(
        noms,
        1,
    ):
        info = series[serie]

        if info["x_min"] is not None:
            zone = (
                "x %.1f..%.1f / y %.1f..%.1f carreaux"
                % (
                    info["x_min"],
                    info["x_max"],
                    info["y_min"],
                    info["y_max"],
                )
            )
        else:
            zone = "zone inconnue"

        print(
            "  %d) %s"
            % (
                indice,
                serie,
            )
        )

        print(
            "     ajustement : %d | validation : %d | %s"
            % (
                info["ajustement"],
                info["validation"],
                zone,
            )
        )

    print()
    print(
        "Taper le numero ou le nom exact de la calibration."
    )
    print(
        "Taper fin pour quitter."
    )

    while True:
        reponse = lireTexte(
            "calibration > "
        ).strip()

        if reponse.lower() in (
            "fin",
            "f",
            "q",
        ):
            return None

        if reponse.isdigit():
            indice = int(
                reponse
            )

            if (
                1 <= indice <= len(noms)
            ):
                return noms[
                    indice - 1
                ]

        for nom in noms:
            if (
                reponse.lower()
                == nom.lower()
            ):
                return nom

        print(
            "  Choix invalide. Entrer un numero "
            "entre 1 et %d, ou le nom exact."
            % len(noms)
        )


# =========================================================
# 5. DIALOGUE
# =========================================================

def lireCible():
    print()
    print(
        "Point a viser : deux nombres en CARREAUX (ex : 4 -2)."
    )
    print(
        "Taper fin pour terminer."
    )

    while True:
        reponse = lireTexte(
            "cible > "
        )

        if reponse.lower() in (
            "fin",
            "f",
            "q",
        ):
            return None

        morceaux = (
            reponse
            .replace(",", " ")
            .split()
        )

        if len(morceaux) != 2:
            print(
                "  Il faut deux nombres separes par un espace."
            )
            continue

        try:
            return (
                float(morceaux[0]),
                float(morceaux[1]),
            )
        except ValueError:
            print(
                "  Saisie illisible."
            )


def lireImpactReel(demande):
    print(
        "  Ou le spot est-il tombe ? deux nombres en carreaux,"
    )

    print(
        "  ou Entree si le releve n'est pas fait."
    )

    reponse = lireTexte(
        "  impact reel > "
    )

    if not reponse:
        return None

    morceaux = (
        reponse
        .replace(",", " ")
        .split()
    )

    if len(morceaux) != 2:
        print(
            "  Saisie ignoree."
        )
        return None

    try:
        reel = (
            float(morceaux[0]),
            float(morceaux[1]),
        )
    except ValueError:
        print(
            "  Saisie ignoree."
        )
        return None

    ecart = (
        (
            (reel[0] - demande[0]) ** 2
            + (reel[1] - demande[1]) ** 2
        ) ** 0.5
    )

    print(
        "  Ecart de pointage : %.2f carreaux (%.1f cm)"
        % (
            ecart,
            ecart * COTE_CARREAU,
        )
    )

    return reel


# =========================================================
# 6. GRAPHIQUES 3D
# =========================================================

def tracerErreurs3D(donnees):
    if not donnees:
        print(
            "Aucune mesure disponible pour le graphique."
        )
        return

    x_vise = [
        d["x_vise"]
        for d in donnees
    ]

    y_vise = [
        d["y_vise"]
        for d in donnees
    ]

    erreur_x = [
        d["x_mesure"] - d["x_vise"]
        for d in donnees
    ]

    erreur_y = [
        d["y_mesure"] - d["y_vise"]
        for d in donnees
    ]

    fig = plt.figure(
        figsize=(14, 7)
    )

    ax1 = fig.add_subplot(
        121,
        projection="3d",
    )

    ax1.plot(
        x_vise,
        y_vise,
        erreur_x,
        marker="o",
        linewidth=2,
    )

    ax1.set_xlabel(
        "X visee (carreaux)"
    )

    ax1.set_ylabel(
        "Y visee (carreaux)"
    )

    ax1.set_zlabel(
        "Erreur X (carreaux)"
    )

    ax1.set_title(
        "Erreur X = X mesuree - X visee"
    )

    ax2 = fig.add_subplot(
        122,
        projection="3d",
    )

    ax2.plot(
        x_vise,
        y_vise,
        erreur_y,
        marker="o",
        linewidth=2,
    )

    ax2.set_xlabel(
        "X visee (carreaux)"
    )

    ax2.set_ylabel(
        "Y visee (carreaux)"
    )

    ax2.set_zlabel(
        "Erreur Y (carreaux)"
    )

    ax2.set_title(
        "Erreur Y = Y mesuree - Y visee"
    )

    plt.tight_layout()

    print()
    print(
        "Affichage des courbes 3D..."
    )

    print(
        "Fermer la fenetre graphique pour terminer le programme."
    )

    plt.show()


# =========================================================
# 7. PROGRAMME
# =========================================================

def main():
    print("=========================================")
    print("TEST DE LA CALIBRATION")
    print("=========================================")
    print()

    print(
        "SECURITE : pointeur rouge classe 2, "
        "laser de puissance DEBRANCHE."
    )

    serie = choisirCalibration()

    if serie is None:
        print(
            "Aucune calibration selectionnee."
        )
        return 0

    print()
    print(
        "Calibration selectionnee :",
        serie,
    )

    print(
        "Reconstruction du modele depuis "
        "les points de cette serie..."
    )

    modele = construireModeleSerie(
        serie
    )

    if modele is None:
        return 1

    print()
    print(
        "Calibration '%s' chargee : %d points d'ajustement"
        % (
            serie,
            modele.get(
                "nb_points",
                0,
            ),
        )
    )

    print(
        "Zone calibree : x %.1f..%.1f, y %.1f..%.1f carreaux"
        % (
            modele["enveloppe"]["u_min"],
            modele["enveloppe"]["u_max"],
            modele["enveloppe"]["v_min"],
            modele["enveloppe"]["v_max"],
        )
    )

    residus = modele.get(
        "residus"
    )

    if residus:
        print(
            "Erreur ajustement : rms %.2f carreaux, max %.2f carreaux"
            % (
                residus["rms_carreaux"],
                residus["max_carreaux"],
            )
        )

    residus_validation = modele.get(
        "residus_validation"
    )

    if residus_validation:
        print(
            "Erreur validation  : rms %.2f carreaux, max %.2f carreaux"
            % (
                residus_validation["rms_carreaux"],
                residus_validation["max_carreaux"],
            )
        )
    else:
        print(
            "Pas de validation pour cette calibration."
        )

    # -----------------------------------------------------
    # Chargement du modele d'apprentissage
    # -----------------------------------------------------

    modele_correction = chargerModeleCorrection(
        serie
    )

    if modele_correction is None:
        print()
        print(
            "Aucun modele de correction appris pour cette serie."
        )
        print(
            "Les prochaines cibles seront donc utilisees "
            "sans correction."
        )
    else:
        print()
        print(
            "Modele de correction charge :",
            chemin_modele_correction(serie),
        )

        print(
            "  Degre polynomial :",
            modele_correction.get(
                "degre",
                "?",
            )
        )

        print(
            "  Points d'apprentissage :",
            modele_correction.get(
                "nb_points",
                "?",
            )
        )

        print(
            "  RMSE X : %.3f carreaux"
            % modele_correction.get(
                "rmse_erreur_x",
                float("nan"),
            )
        )

        print(
            "  RMSE Y : %.3f carreaux"
            % modele_correction.get(
                "rmse_erreur_y",
                float("nan"),
            )
        )

    # -----------------------------------------------------
    # Connexion robot
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        return 1

    allerPositionInitiale(
        mc
    )

    if not verifierBase(
        mc,
        modele
    ):
        return 1

    angles = lireAngles(
        mc
    )

    if angles is None:
        print(
            "Lecture des angles impossible."
        )
        return 1

    base = angles[0:3]
    j6 = angles[5]

    fichier_validation = os.path.join(
        DOSSIER_RESULTATS,
        "validation_pointage_%s.csv"
        % nom_fichier_securise(serie),
    )

    print()
    print(
        "Les tirs de cette session seront enregistres dans :"
    )
    print(
        " ",
        fichier_validation,
    )

    donnees_graphique = []

    try:
        if not allumerPointeur(mc):
            print(
                "Le pointeur ne s'allume pas."
            )

        while True:
            demande = lireCible()

            if demande is None:
                print()
                print(
                    "Fin de la campagne de mesure."
                )

                tracerErreurs3D(
                    donnees_graphique
                )

                break

            x, y = demande

            # -------------------------------------------------
            # Verification de l'enveloppe sur la cible d'origine.
            # -------------------------------------------------

            if not dansEnveloppe(
                modele,
                x,
                y,
            ):
                e = modele["enveloppe"]

                print(
                    "  Point hors de la zone de la calibration '%s'."
                    % serie
                )

                print(
                    "  Zone autorisee : "
                    "x %.1f..%.1f, y %.1f..%.1f carreaux."
                    % (
                        e["u_min"],
                        e["u_max"],
                        e["v_min"],
                        e["v_max"],
                    )
                )

                continue

            # -------------------------------------------------
            # Correction apprise
            # -------------------------------------------------

            (
                x_commande,
                y_commande,
                erreur_x_predite,
                erreur_y_predite,
            ) = corrigerCible(
                modele_correction,
                x,
                y,
            )

            if modele_correction is not None:
                print()
                print(
                    "  Correction apprise :"
                )

                print(
                    "    erreur X predite = %+0.3f carreaux"
                    % erreur_x_predite
                )

                print(
                    "    erreur Y predite = %+0.3f carreaux"
                    % erreur_y_predite
                )

                print(
                    "    cible initiale    = (%+.3f, %+.3f)"
                    % (
                        x,
                        y,
                    )
                )

                print(
                    "    cible corrigee    = (%+.3f, %+.3f)"
                    % (
                        x_commande,
                        y_commande,
                    )
                )

                # La correction ne doit pas sortir du domaine
                # de validite de la calibration.
                if not dansEnveloppe(
                    modele,
                    x_commande,
                    y_commande,
                ):
                    print(
                        "  ATTENTION : la cible corrigee sort "
                        "de l'enveloppe de calibration."
                    )
                    print(
                        "  Tir annule pour eviter une extrapolation "
                        "du modele."
                    )
                    continue

            # -------------------------------------------------
            # Calcul des angles sur la cible corrigee
            # -------------------------------------------------

            resultat = anglesPourPoint(
                modele,
                x_commande,
                y_commande,
            )

            if resultat is None:
                print(
                    "  Impossible de calculer J4/J5 "
                    "pour cette cible corrigee."
                )
                continue

            j4, j5 = resultat

            print(
                "  Angles calcules : "
                "J4 = %+.2f   J5 = %+.2f"
                % (
                    j4,
                    j5,
                )
            )

            vise, mesure = viserAngles(
                mc,
                base,
                j4,
                j5,
                j6,
            )

            if mesure is None:
                print(
                    "  Mouvement echoue."
                )
                continue

            if not vise:
                print(
                    "  ATTENTION : la tete se repose "
                    "a %+.2f / %+.2f deg de l'angle demande"
                    % (
                        mesure[3] - j4,
                        mesure[4] - j5,
                    )
                )

                print(
                    "  (tolerance %.2f deg). "
                    "Le rattrapage n'a pas converge."
                    % TOLERANCE_POINTAGE
                )

            # -------------------------------------------------
            # Predicteur du modele mecanique
            #
            # Attention :
            # On compare ici la prediction du modele initial
            # a la cible UTILISATEUR, et non a la cible corrigee.
            # -------------------------------------------------

            prediction = predireImpact(
                modele,
                mesure[3],
                mesure[4],
            )

            if prediction is not None:

                derive = (
                    (
                        (prediction[0] - x) ** 2
                        + (prediction[1] - y) ** 2
                    ) ** 0.5
                )

                if derive > 0.1:
                    print(
                        "  Prediction modele : "
                        "(%.3f, %.3f)"
                        " -> derive %.2f carreaux"
                        % (
                            prediction[0],
                            prediction[1],
                            derive,
                        )
                    )

            # -------------------------------------------------
            # Mesure reelle du spot
            # -------------------------------------------------

            reel = lireImpactReel(
                demande
            )

            # -------------------------------------------------
            # Stockage pour l'apprentissage
            #
            # IMPORTANT :
            # on stocke toujours la cible demandee par
            # l'utilisateur et la position REELLEMENT mesuree.
            #
            # On ne stocke pas x_commande/y_commande comme cible
            # d'apprentissage, sinon le modele apprendrait la
            # correction deja appliquee.
            # -------------------------------------------------

            if reel is not None:

                donnees_graphique.append({
                    "x_vise": x,
                    "y_vise": y,
                    "x_mesure": reel[0],
                    "y_mesure": reel[1],
                })

                enregistrerMesureApprentissage(
                    serie,
                    demande,
                    reel,
                )

            enregistrerTir(
                demande,
                (
                    mesure[3],
                    mesure[4],
                ),
                reel,
                chemin=fichier_validation,
            )

    finally:

        eteindrePointeur(
            mc
        )

        print()
        print(
            "Pointeur eteint. Moteurs laisses sous tension."
        )

        print(
            "Calibration utilisee :",
            serie,
        )

        print(
            "Tirs enregistres dans :",
            fichier_validation,
        )

        print(
            "Mesures d'apprentissage :",
            FICHIER_APPRENTISSAGE,
        )

    return 0


if __name__ == "__main__":
    try:
        code_retour = main()

    except KeyboardInterrupt:
        print()
        print(
            "Programme interrompu par l'operateur."
        )
        code_retour = 1

    print()
    print(
        "Programme termine."
    )

    raise SystemExit(
        code_retour
    )
