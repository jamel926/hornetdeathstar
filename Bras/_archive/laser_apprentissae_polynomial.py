"""
POINTAGE LASER AVEC CORRECTION POLYNOMIALE APPRENDUE

Principe :
    1. Lire resultats/apprentissage_erreurs.csv
    2. Ajuster un polynôme pour l'erreur X.
    3. Ajuster un polynôme pour l'erreur Y.
    4. L'utilisateur choisit le degré de chaque polynôme.
    5. Pour une cible (X, Y), prédire :
           erreur_X = f(X, Y)
           erreur_Y = g(X, Y)
    6. Corriger la cible :
           X_corrige = X - erreur_X
           Y_corrige = Y - erreur_Y
    7. Faire viser le robot sur cette cible corrigée.
    8. Allumer le pointeur uniquement après le mouvement.
    9. L'utilisateur indique si le laser est au bon endroit.
       - oui : la mesure réelle est considérée égale à la cible demandée ;
       - non : l'utilisateur saisit la vraie position.
   10. Ajouter la mesure au fichier apprentissage_erreurs.csv.

IMPORTANT :
    - Pas de campagne aléatoire.
    - Pas de vérification de la base de calibration.
    - Calibration géométrique utilisée : Calib 2 uniquement.
    - Toutes les distances X/Y sont en CARREAUX.
    - Le programme réajuste les modèles à partir du CSV au démarrage.
      Pour utiliser les nouvelles mesures ajoutées pendant une même
      session, choisir "r" pour recalculer les modèles.
"""

import os
import csv
import numpy as np
import pandas as pd

from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score

from datetime import datetime

from calibration_grille import (
    DOSSIER_RESULTATS,
    COTE_CARREAU,
    TOLERANCE_POINTAGE,
    allumerPointeur,
    anglesPourPoint,
    viserAngles,
    ajusterModele,
    chargerMesures,
    connecterRobot,
    dansEnveloppe,
    eteindrePointeur,
    lireAngles,
    lireTexte,
)

from PU_PositionInitiale import allerPositionInitiale


# ==========================================================
# PARAMETRES
# ==========================================================

SERIE = "Calib 2"

FICHIER_MESURES = os.path.join(
    DOSSIER_RESULTATS,
    "calibration_grille.csv",
)

FICHIER_APPRENTISSAGE = os.path.join(
    DOSSIER_RESULTATS,
    "apprentissage_erreurs.csv",
)

FICHIER_VALIDATION = os.path.join(
    DOSSIER_RESULTATS,
    "validation_pointage_Calib_2.csv",
)

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

ENTETE_VALIDATION = [
    "horodatage",
    "x_demande_carreaux",
    "y_demande_carreaux",
    "x_commande_carreaux",
    "y_commande_carreaux",
    "erreur_x_predite",
    "erreur_y_predite",
    "J4",
    "J5",
    "J4_atteint",
    "J5_atteint",
    "x_reel_carreaux",
    "y_reel_carreaux",
    "ecart_carreaux",
]


# ==========================================================
# LECTURE DU CSV D'APPRENTISSAGE
# ==========================================================

def chargerDonneesApprentissage():
    """Charge les données de Calib 2 depuis apprentissage_erreurs.csv."""

    if not os.path.exists(FICHIER_APPRENTISSAGE):
        raise FileNotFoundError(
            "Fichier d'apprentissage introuvable :\n"
            f"{FICHIER_APPRENTISSAGE}"
        )

    df = pd.read_csv(FICHIER_APPRENTISSAGE)

    colonnes = {
        "serie",
        "x_vise",
        "y_vise",
        "erreur_x",
        "erreur_y",
    }

    manquantes = colonnes - set(df.columns)
    if manquantes:
        raise ValueError(
            "Colonnes manquantes dans apprentissage_erreurs.csv : "
            + ", ".join(sorted(manquantes))
        )

    df = df[
        df["serie"].astype(str).str.strip() == SERIE
    ].copy()

    for colonne in [
        "x_vise",
        "y_vise",
        "erreur_x",
        "erreur_y",
    ]:
        df[colonne] = pd.to_numeric(
            df[colonne],
            errors="coerce",
        )

    df = df.dropna(
        subset=[
            "x_vise",
            "y_vise",
            "erreur_x",
            "erreur_y",
        ]
    )

    return df


# ==========================================================
# MODELE POLYNOMIAL
# ==========================================================

def entrerDegre(message, minimum=1, maximum=8):
    while True:
        reponse = lireTexte(message).strip()

        if reponse.lower() in {"q", "quit", "quitter"}:
            return None

        try:
            degre = int(reponse)
        except ValueError:
            print("  Entrez un nombre entier.")
            continue

        if minimum <= degre <= maximum:
            return degre

        print(
            f"  Le degré doit être compris entre {minimum} et {maximum}."
        )


def entrerOuiNon(message):
    while True:
        reponse = lireTexte(message).strip().lower()

        if reponse in {"o", "oui", "y", "yes"}:
            return True

        if reponse in {"n", "non", "no"}:
            return False

        print("  Répondez par oui ou non.")


def ajusterPolynome(df, colonne_erreur, degre):
    """Ajuste exactement le même type de polynôme que dans les programmes fournis."""

    X = np.column_stack([
        df["x_vise"].to_numpy(dtype=float),
        df["y_vise"].to_numpy(dtype=float),
    ])

    Z = df[colonne_erreur].to_numpy(dtype=float)

    poly = PolynomialFeatures(
        degree=degre,
        include_bias=True,
    )

    X_poly = poly.fit_transform(X)

    nombre_coefficients = X_poly.shape[1]

    if len(df) < nombre_coefficients:
        raise ValueError(
            f"Le degré {degre} nécessite au moins "
            f"{nombre_coefficients} mesures, mais le CSV n'en contient que "
            f"{len(df)} pour {colonne_erreur}."
        )

    modele = LinearRegression()
    modele.fit(X_poly, Z)

    Z_pred = modele.predict(X_poly)

    r2 = r2_score(Z, Z_pred)
    residus = np.abs(Z - Z_pred)

    return {
        "degre": degre,
        "poly": poly,
        "modele": modele,
        "r2": float(r2),
        "erreur_moyenne": float(residus.mean()),
        "erreur_max": float(residus.max()),
        "colonne": colonne_erreur,
    }


def nomCoefficientVersTexte(nom):
    """Convertit par exemple X^2 Y en X²Y pour l'affichage."""
    mapping = {
        "X": "X",
        "Y": "Y",
    }

    morceaux = nom.split(" ")
    resultat = []

    for morceau in morceaux:
        if not morceau:
            continue

        if "^" in morceau:
            variable, puissance = morceau.split("^", 1)
            resultat.append(
                mapping.get(variable, variable)
                + _superScript(puissance)
            )
        else:
            resultat.append(
                mapping.get(morceau, morceau)
            )

    return "".join(resultat) or "1"


def _superScript(valeur):
    chiffres = str(valeur).translate(
        str.maketrans(
            "0123456789-",
            "⁰¹²³⁴⁵⁶⁷⁸⁹⁻",
        )
    )
    return chiffres


def afficherEquation(modele_info, nom_erreur):
    poly = modele_info["poly"]
    modele = modele_info["modele"]

    noms = poly.get_feature_names_out(["X", "Y"])
    coefficients = modele.coef_

    print()
    print("Équation du modèle", nom_erreur, ":")

    equation = f"{modele.intercept_:.8f}"

    for coefficient, nom in zip(coefficients[1:], noms[1:]):
        if abs(coefficient) < 1e-10:
            continue

        terme = nomCoefficientVersTexte(nom)
        equation += f" + ({coefficient:+.8f})*{terme}"

    print("  ", equation)


def afficherModele(modele_info, nom):
    print()
    print("----------------------------------------")
    print(nom)
    print("----------------------------------------")
    print("Degré :", modele_info["degre"])
    print("R²    :", f"{modele_info['r2']:.6f}")
    print(
        "Erreur moyenne :",
        f"{modele_info['erreur_moyenne']:.6f} carreau(x)"
    )
    print(
        "Erreur max     :",
        f"{modele_info['erreur_max']:.6f} carreau(x)"
    )

    afficherEquation(
        modele_info,
        nom,
    )


def predireErreur(modele_info, x, y):
    XY = np.array([
        [x, y],
    ], dtype=float)

    XY_poly = modele_info["poly"].transform(XY)

    return float(
        modele_info["modele"].predict(XY_poly)[0]
    )


# ==========================================================
# CALIBRATION GEOMETRIQUE CALIB 2
# ==========================================================

def chargerBaseSerie():
    """Charge J1/J2/J3 de Calib 2 sans vérifier la base courante."""

    try:
        with open(
            FICHIER_MESURES,
            "r",
            newline="",
        ) as fichier:
            for ligne in csv.DictReader(fichier):
                serie = (
                    ligne.get("serie") or ""
                ).strip()

                if serie != SERIE:
                    continue

                try:
                    return [
                        float(ligne["J1"]),
                        float(ligne["J2"]),
                        float(ligne["J3"]),
                    ]
                except (ValueError, KeyError):
                    continue

    except Exception as e:
        print(
            "Impossible de lire la base de Calib 2 :",
            e,
        )

    return None


def construireModeleCalib2():
    ajustement = chargerMesures(
        usage="ajustement",
        serie=SERIE,
    )

    if len(ajustement) < 5:
        raise ValueError(
            f"Calib 2 contient seulement {len(ajustement)} "
            "points d'ajustement."
        )

    base_reference = chargerBaseSerie()

    modele = ajusterModele(
        ajustement,
        base=base_reference,
    )

    if modele is None:
        raise RuntimeError(
            "Impossible de construire le modèle géométrique de Calib 2."
        )

    modele["serie"] = SERIE
    return modele


# ==========================================================
# ENREGISTREMENT DES MESURES
# ==========================================================

def enregistrerMesureApprentissage(x_vise, y_vise, x_mesure, y_mesure):
    erreur_x = x_mesure - x_vise
    erreur_y = y_mesure - y_vise

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
        redacteur = csv.writer(fichier)

        if nouveau:
            redacteur.writerow(
                ENTETE_APPRENTISSAGE
            )

        redacteur.writerow([
            datetime.now().isoformat(
                timespec="seconds"
            ),
            SERIE,
            round(x_vise, 5),
            round(y_vise, 5),
            round(x_mesure, 5),
            round(y_mesure, 5),
            round(erreur_x, 5),
            round(erreur_y, 5),
        ])

    print()
    print("Mesure ajoutée à apprentissage_erreurs.csv :")
    print("  X visé    = %+.5f" % x_vise)
    print("  Y visé    = %+.5f" % y_vise)
    print("  X mesuré  = %+.5f" % x_mesure)
    print("  Y mesuré  = %+.5f" % y_mesure)
    print("  Erreur X  = %+.5f" % erreur_x)
    print("  Erreur Y  = %+.5f" % erreur_y)


def enregistrerValidation(
    demande,
    commande,
    erreurs_predites,
    angles,
    mesure,
    reel,
):
    if reel is None:
        x_reel = ""
        y_reel = ""
        ecart = ""
    else:
        x_reel = round(reel[0], 5)
        y_reel = round(reel[1], 5)
        ecart = round(
            np.hypot(
                reel[0] - demande[0],
                reel[1] - demande[1],
            ),
            5,
        )

    nouveau = not os.path.exists(
        FICHIER_VALIDATION
    )

    with open(
        FICHIER_VALIDATION,
        "a",
        newline="",
    ) as fichier:
        redacteur = csv.writer(fichier)

        if nouveau:
            redacteur.writerow(
                ENTETE_VALIDATION
            )

        redacteur.writerow([
            datetime.now().isoformat(
                timespec="seconds"
            ),
            round(demande[0], 5),
            round(demande[1], 5),
            round(commande[0], 5),
            round(commande[1], 5),
            round(erreurs_predites[0], 5),
            round(erreurs_predites[1], 5),
            round(angles[0], 5),
            round(angles[1], 5),
            round(mesure[3], 5),
            round(mesure[4], 5),
            x_reel,
            y_reel,
            ecart,
        ])


# ==========================================================
# DIALOGUE UTILISATEUR
# ==========================================================

def lireCible():
    print()
    print(
        "Entrez la cible X Y en carreaux "
        "(exemple : 4 -2)."
    )
    print("Tapez 'fin' pour terminer.")

    while True:
        reponse = lireTexte("cible > ").strip()

        if reponse.lower() in {
            "fin",
            "f",
            "q",
            "quit",
        }:
            return None

        morceaux = (
            reponse
            .replace(",", " ")
            .split()
        )

        if len(morceaux) != 2:
            print("  Entrez deux nombres.")
            continue

        try:
            return (
                float(morceaux[0]),
                float(morceaux[1]),
            )
        except ValueError:
            print("  Saisie illisible.")


def lirePositionReelle(demande):
    """Demande si le laser est correct, puis la vraie position si nécessaire."""

    bonne = entrerOuiNon(
        "Le laser est-il sur la bonne position ? [o/n] > "
    )

    if bonne:
        return demande

    while True:
        reponse = lireTexte(
            "Donnez la vraie position du laser X Y > "
        ).strip()

        morceaux = (
            reponse
            .replace(",", " ")
            .split()
        )

        if len(morceaux) != 2:
            print("  Entrez deux nombres en carreaux.")
            continue

        try:
            return (
                float(morceaux[0]),
                float(morceaux[1]),
            )
        except ValueError:
            print("  Saisie illisible.")


# ==========================================================
# CHOIX / RECALCUL DES MODELES
# ==========================================================

def demanderDegres():
    print()
    print("========================================")
    print("DEGRE DES POLYNOMES")
    print("========================================")
    print(
        "Le modèle X corrige l'erreur X et le modèle Y "
        "corrige l'erreur Y."
    )
    print("Vous pouvez choisir un degré différent pour X et Y.")

    degre_x = entrerDegre(
        "Degré du modèle d'erreur X [1-8] > ",
        1,
        8,
    )

    if degre_x is None:
        return None

    degre_y = entrerDegre(
        "Degré du modèle d'erreur Y [1-8] > ",
        1,
        8,
    )

    if degre_y is None:
        return None

    return degre_x, degre_y


def construireModeles(df, degre_x, degre_y):
    modele_x = ajusterPolynome(
        df,
        "erreur_x",
        degre_x,
    )

    modele_y = ajusterPolynome(
        df,
        "erreur_y",
        degre_y,
    )

    return modele_x, modele_y


def afficherResumeDonnees(df):
    print()
    print("========================================")
    print("DONNEES D'APPRENTISSAGE")
    print("========================================")
    print("Série :", SERIE)
    print("Fichier :")
    print(" ", FICHIER_APPRENTISSAGE)
    print("Nombre de mesures :", len(df))

    if len(df) > 0:
        print(
            "Zone X : %.3f à %.3f carreaux"
            % (
                df["x_vise"].min(),
                df["x_vise"].max(),
            )
        )
        print(
            "Zone Y : %.3f à %.3f carreaux"
            % (
                df["y_vise"].min(),
                df["y_vise"].max(),
            )
        )


def apprendreEtAfficher():
    df = chargerDonneesApprentissage()
    afficherResumeDonnees(df)

    choix = demanderDegres()
    if choix is None:
        return None, None

    degre_x, degre_y = choix

    modele_x, modele_y = construireModeles(
        df,
        degre_x,
        degre_y,
    )

    afficherModele(
        modele_x,
        "ERREUR X = f(X, Y)",
    )

    afficherModele(
        modele_y,
        "ERREUR Y = g(X, Y)",
    )

    return modele_x, modele_y


# ==========================================================
# PROGRAMME PRINCIPAL
# ==========================================================

def main():
    print("========================================")
    print("POINTAGE LASER - CORRECTION POLYNOMIALE")
    print("========================================")
    print("Calibration géométrique :", SERIE)
    print("Toutes les positions : CARREAUX")
    print()
    print("Aucune campagne aléatoire.")
    print("Une cible à la fois, validation manuelle.")

    # ------------------------------------------------------
    # Apprentissage initial
    # ------------------------------------------------------
    try:
        modele_x, modele_y = apprendreEtAfficher()
    except Exception as e:
        print()
        print("ERREUR pendant l'apprentissage :")
        print(" ", e)
        return 1

    if modele_x is None or modele_y is None:
        return 0

    # ------------------------------------------------------
    # Calibration géométrique Calib 2
    # ------------------------------------------------------
    try:
        modele_geom = construireModeleCalib2()
    except Exception as e:
        print()
        print("ERREUR calibration géométrique Calib 2 :")
        print(" ", e)
        return 1

    print()
    print("Calibration géométrique Calib 2 chargée.")

    # ------------------------------------------------------
    # Connexion robot
    # ------------------------------------------------------
    mc = connecterRobot()
    if mc is None:
        return 1

    allerPositionInitiale(mc)

    angles = lireAngles(mc)
    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    # On ne vérifie volontairement PAS la base par rapport à la calibration.
    base = angles[0:3]
    j6 = angles[5]

    print()
    print("Base courante utilisée pour le mouvement :")
    print(
        "  J1 = %+.2f   J2 = %+.2f   J3 = %+.2f"
        % (base[0], base[1], base[2])
    )

    print()
    print("Validation :", FICHIER_VALIDATION)
    print("Apprentissage :", FICHIER_APPRENTISSAGE)

    try:
        while True:
            demande = lireCible()

            if demande is None:
                print()
                print("Fin du programme.")
                break

            x, y = demande

            # --------------------------------------------------
            # Vérification de la zone calibrée
            # --------------------------------------------------
            if not dansEnveloppe(
                modele_geom,
                x,
                y,
            ):
                e = modele_geom["enveloppe"]
                print()
                print("CIBLE HORS DE LA ZONE CALIBREE.")
                print(
                    "  X : %.2f à %.2f carreaux"
                    % (e["u_min"], e["u_max"])
                )
                print(
                    "  Y : %.2f à %.2f carreaux"
                    % (e["v_min"], e["v_max"])
                )
                continue

            # --------------------------------------------------
            # Prédiction des erreurs
            # --------------------------------------------------
            erreur_x_predite = predireErreur(
                modele_x,
                x,
                y,
            )

            erreur_y_predite = predireErreur(
                modele_y,
                x,
                y,
            )

            # Pour une erreur mesurée = position réelle - cible,
            # la compensation est cible - erreur prédite.
            x_commande = x - erreur_x_predite
            y_commande = y - erreur_y_predite

            print()
            print("----------------------------------------")
            print("CIBLE")
            print("----------------------------------------")
            print(
                "  Demandée  : X = %+.3f   Y = %+.3f carreaux"
                % (x, y)
            )
            print(
                "  Erreur X prévue : %+.5f carreau"
                % erreur_x_predite
            )
            print(
                "  Erreur Y prévue : %+.5f carreau"
                % erreur_y_predite
            )
            print(
                "  Commandée : X = %+.3f   Y = %+.3f carreaux"
                % (x_commande, y_commande)
            )

            # --------------------------------------------------
            # La correction ne doit pas sortir du domaine calibré.
            # --------------------------------------------------
            if not dansEnveloppe(
                modele_geom,
                x_commande,
                y_commande,
            ):
                print()
                print(
                    "ATTENTION : la cible corrigée sort de "
                    "l'enveloppe de Calib 2."
                )
                print("Mouvement annulé pour éviter l'extrapolation.")
                continue

            # --------------------------------------------------
            # Calcul des angles
            # --------------------------------------------------
            resultat = anglesPourPoint(
                modele_geom,
                x_commande,
                y_commande,
            )

            if resultat is None:
                print(
                    "Impossible de calculer J4/J5 pour la cible corrigée."
                )
                continue

            j4, j5 = resultat

            print()
            print(
                "Angles demandés : J4 = %+.2f   J5 = %+.2f"
                % (j4, j5)
            )

            # Laser éteint pendant le mouvement.
            eteindrePointeur(mc)

            # --------------------------------------------------
            # Mouvement unique / fonction de viserAngles existante
            # --------------------------------------------------
            vise, mesure = viserAngles(
                mc,
                base,
                j4,
                j5,
                j6,
            )

            if mesure is None:
                print("Mouvement échoué.")
                eteindrePointeur(mc)
                continue

            print()
            print(
                "Angles atteints : J4 = %+.2f   J5 = %+.2f"
                % (mesure[3], mesure[4])
            )

            if not vise:
                print(
                    "Attention : écart angulaire résiduel "
                    "J4 = %+.3f°, J5 = %+.3f°."
                    % (
                        mesure[3] - j4,
                        mesure[4] - j5,
                    )
                )
                print(
                    "Tolérance : %.3f°"
                    % TOLERANCE_POINTAGE
                )

            # --------------------------------------------------
            # Laser ON après arrivée
            # --------------------------------------------------
            if not allumerPointeur(mc):
                print("ATTENTION : impossible d'allumer le pointeur.")

            # --------------------------------------------------
            # Validation humaine de la position réelle
            # --------------------------------------------------
            reel = lirePositionReelle(
                demande
            )

            # Éteindre après l'observation.
            eteindrePointeur(mc)

            # --------------------------------------------------
            # Calcul / affichage de l'erreur réelle
            # --------------------------------------------------
            erreur_x_reelle = reel[0] - x
            erreur_y_reelle = reel[1] - y

            print()
            print("Mesure réelle :")
            print(
                "  X réel = %+.5f   Y réel = %+.5f carreaux"
                % (reel[0], reel[1])
            )
            print(
                "  Erreur X réelle = %+.5f carreau"
                % erreur_x_reelle
            )
            print(
                "  Erreur Y réelle = %+.5f carreau"
                % erreur_y_reelle
            )
            print(
                "  Écart total = %.5f carreau (%.3f cm)"
                % (
                    np.hypot(
                        erreur_x_reelle,
                        erreur_y_reelle,
                    ),
                    np.hypot(
                        erreur_x_reelle,
                        erreur_y_reelle,
                    ) * COTE_CARREAU,
                )
            )

            # --------------------------------------------------
            # Ajout au CSV d'apprentissage
            # --------------------------------------------------
            enregistrerMesureApprentissage(
                x,
                y,
                reel[0],
                reel[1],
            )

            # --------------------------------------------------
            # Validation
            # --------------------------------------------------
            enregistrerValidation(
                demande=(x, y),
                commande=(x_commande, y_commande),
                erreurs_predites=(
                    erreur_x_predite,
                    erreur_y_predite,
                ),
                angles=(j4, j5),
                mesure=mesure,
                reel=reel,
            )

            print()
            print("Mesure enregistrée.")
            print(
                "Pour que cette mesure influence les prochaines "
                "prédictions, relancez l'apprentissage avec 'r' "
                "dans le menu prévu après la prochaine cible."
            )

            # --------------------------------------------------
            # Recalcul optionnel immédiat
            # --------------------------------------------------
            recalculer = entrerOuiNon(
                "Recalculer immédiatement les polynômes avec cette nouvelle mesure ? [o/n] > "
            )

            if recalculer:
                try:
                    df = chargerDonneesApprentissage()
                    modele_x, modele_y = construireModeles(
                        df,
                        modele_x["degre"],
                        modele_y["degre"],
                    )

                    afficherModele(
                        modele_x,
                        "ERREUR X = f(X, Y)",
                    )
                    afficherModele(
                        modele_y,
                        "ERREUR Y = g(X, Y)",
                    )

                except Exception as e:
                    print(
                        "Impossible de recalculer les modèles :",
                        e,
                    )

    except KeyboardInterrupt:
        print()
        print("Programme interrompu par l'opérateur.")

    finally:
        eteindrePointeur(mc)
        print()
        print("Pointeur éteint.")
        print("Moteurs laissés sous tension.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
