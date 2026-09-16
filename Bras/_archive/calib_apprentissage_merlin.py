"""
calib_apprentissage.py

Apprend un modele polynomial 2D de correction du pointage du laser.

Principe :
    erreur_x = x_mesure - x_visee
    erreur_y = y_mesure - y_visee

On ajuste deux polynomes en fonction de (x_visee, y_visee) :

    erreur_x = f_x(x, y)
    erreur_y = f_y(x, y)

Pour une nouvelle cible (x, y), la cible corrigee devient :

    x_corrige = x - f_x(x, y)
    y_corrige = y - f_y(x, y)

Le programme lit les mesures du fichier :
    resultats/apprentissage_erreurs.csv

et produit :
    resultats/modele_correction_<serie>.json

Toutes les distances sont en CARREAUX.
"""

import argparse
import csv
import json
import math
import os
import re
from datetime import datetime

import numpy as np

# On recupere le dossier de resultats depuis le script de calibration.
from calibration_grille import DOSSIER_RESULTATS


FICHIER_APPRENTISSAGE = os.path.join(
    DOSSIER_RESULTATS,
    "apprentissage_erreurs.csv",
)


def nom_fichier_securise(serie):
    propre = re.sub(r"[^A-Za-z0-9._-]+", "_", serie).strip("._")
    return propre or "calibration"


def generer_termes_polynomial(degre):
    """
    Retourne les couples (i, j) correspondant aux termes :

        x^i * y^j

    avec i + j <= degre.

    Exemple degre=2 :
        1, x, y, x², x*y, y²
    """
    termes = []

    for total in range(degre + 1):
        for i in range(total, -1, -1):
            j = total - i
            termes.append((i, j))

    return termes


def matrice_polynomiale(x, y, degre):
    """
    Construit la matrice des termes polynomiaux.
    x et y sont deja normalises.
    """
    termes = generer_termes_polynomial(degre)

    colonnes = []
    for i, j in termes:
        colonnes.append((x ** i) * (y ** j))

    return np.column_stack(colonnes)


def lire_donnees(serie):
    """
    Lit les mesures d'apprentissage pour une serie donnee.
    """
    if not os.path.exists(FICHIER_APPRENTISSAGE):
        raise FileNotFoundError(
            f"Fichier introuvable : {FICHIER_APPRENTISSAGE}"
        )

    donnees = []

    with open(FICHIER_APPRENTISSAGE, "r", newline="") as fichier:
        lecteur = csv.DictReader(fichier)

        champs = lecteur.fieldnames or []

        requis = {
            "serie",
            "x_vise",
            "y_vise",
            "x_mesure",
            "y_mesure",
        }

        manquants = requis - set(champs)
        if manquants:
            raise ValueError(
                "Colonnes manquantes dans le fichier "
                f"{FICHIER_APPRENTISSAGE} : {sorted(manquants)}"
            )

        for ligne in lecteur:
            if (ligne.get("serie") or "").strip() != serie:
                continue

            try:
                x_vise = float(ligne["x_vise"])
                y_vise = float(ligne["y_vise"])
                x_mesure = float(ligne["x_mesure"])
                y_mesure = float(ligne["y_mesure"])
            except (TypeError, ValueError):
                continue

            donnees.append({
                "x_vise": x_vise,
                "y_vise": y_vise,
                "x_mesure": x_mesure,
                "y_mesure": y_mesure,
                "erreur_x": x_mesure - x_vise,
                "erreur_y": y_mesure - y_vise,
            })

    return donnees


def ajuster_polynome(donnees, degre):
    """
    Ajuste simultanement les deux fonctions d'erreur.

    Une normalisation x/y est utilisee avant l'ajustement afin
    de limiter les problemes numeriques lorsque les coordonnees
    sont de grande amplitude.

    Retourne un dictionnaire directement serialisable en JSON.
    """
    x = np.array([d["x_vise"] for d in donnees], dtype=float)
    y = np.array([d["y_vise"] for d in donnees], dtype=float)

    erreur_x = np.array(
        [d["erreur_x"] for d in donnees],
        dtype=float,
    )

    erreur_y = np.array(
        [d["erreur_y"] for d in donnees],
        dtype=float,
    )

    centre_x = float(np.mean(x))
    centre_y = float(np.mean(y))

    echelle_x = float(np.std(x))
    echelle_y = float(np.std(y))

    # Eviter une division par zero si toutes les valeurs sont
    # identiques sur un axe.
    if echelle_x < 1e-12:
        echelle_x = 1.0

    if echelle_y < 1e-12:
        echelle_y = 1.0

    x_n = (x - centre_x) / echelle_x
    y_n = (y - centre_y) / echelle_y

    A = matrice_polynomiale(
        x_n,
        y_n,
        degre,
    )

    # Nombre minimal de points : au moins autant que de coefficients.
    nb_coefficients = A.shape[1]
    if len(donnees) < nb_coefficients:
        raise ValueError(
            f"Le degre {degre} necessite au moins "
            f"{nb_coefficients} points, mais seulement "
            f"{len(donnees)} sont disponibles."
        )

    coeff_x, _, rang, _ = np.linalg.lstsq(
        A,
        erreur_x,
        rcond=None,
    )

    coeff_y, _, rang_y, _ = np.linalg.lstsq(
        A,
        erreur_y,
        rcond=None,
    )

    prediction_x = A @ coeff_x
    prediction_y = A @ coeff_y

    residu_x = erreur_x - prediction_x
    residu_y = erreur_y - prediction_y

    rmse_x = float(np.sqrt(np.mean(residu_x ** 2)))
    rmse_y = float(np.sqrt(np.mean(residu_y ** 2)))

    max_x = float(np.max(np.abs(residu_x)))
    max_y = float(np.max(np.abs(residu_y)))

    modele = {
        "version": 1,
        "date_creation": datetime.now().isoformat(timespec="seconds"),
        "serie": None,
        "unite": "carreaux",
        "degre": int(degre),
        "centre_x": centre_x,
        "centre_y": centre_y,
        "echelle_x": echelle_x,
        "echelle_y": echelle_y,
        "termes": [
            [int(i), int(j)]
            for i, j in generer_termes_polynomial(degre)
        ],
        "coefficients_erreur_x": [
            float(v) for v in coeff_x
        ],
        "coefficients_erreur_y": [
            float(v) for v in coeff_y
        ],
        "nb_points": int(len(donnees)),
        "rang_matrice_x": int(rang),
        "rang_matrice_y": int(rang_y),
        "rmse_erreur_x": rmse_x,
        "rmse_erreur_y": rmse_y,
        "max_residu_erreur_x": max_x,
        "max_residu_erreur_y": max_y,
        "erreur_moyenne_x": float(np.mean(erreur_x)),
        "erreur_moyenne_y": float(np.mean(erreur_y)),
    }

    return modele


def sauvegarder_modele(modele, chemin):
    os.makedirs(
        os.path.dirname(chemin),
        exist_ok=True,
    )

    with open(chemin, "w") as fichier:
        json.dump(
            modele,
            fichier,
            indent=2,
            ensure_ascii=False,
        )


def afficher_resultats(donnees, modele):
    print()
    print("=========================================")
    print("APPRENTISSAGE DE LA CORRECTION")
    print("=========================================")

    print(
        f"Serie              : {modele['serie']}"
    )

    print(
        f"Nombre de mesures  : {modele['nb_points']}"
    )

    print(
        f"Degre polynomial   : {modele['degre']}"
    )

    print()
    print(
        "Erreur moyenne X   : "
        f"{modele['erreur_moyenne_x']:+.4f} carreaux"
    )

    print(
        "Erreur moyenne Y   : "
        f"{modele['erreur_moyenne_y']:+.4f} carreaux"
    )

    print()
    print(
        "RMSE modele X      : "
        f"{modele['rmse_erreur_x']:.4f} carreaux"
    )

    print(
        "RMSE modele Y      : "
        f"{modele['rmse_erreur_y']:.4f} carreaux"
    )

    print(
        "Max residu X       : "
        f"{modele['max_residu_erreur_x']:.4f} carreaux"
    )

    print(
        "Max residu Y       : "
        f"{modele['max_residu_erreur_y']:.4f} carreaux"
    )

    # Diagnostic simple : erreur moyenne brute.
    erreur_totale_moyenne = math.sqrt(
        modele["erreur_moyenne_x"] ** 2
        + modele["erreur_moyenne_y"] ** 2
    )

    print()
    print(
        "Norme de l'offset moyen : "
        f"{erreur_totale_moyenne:.4f} carreaux"
    )

    print()
    print(
        "Interpretation : pour une cible (x, y), le programme"
    )
    print(
        "utilisera ensuite :"
    )
    print(
        "    x_corrige = x - erreur_x_modele(x, y)"
    )
    print(
        "    y_corrige = y - erreur_y_modele(x, y)"
    )


SERIE = "Calib 2"


def demander_degre():
    """Demande interactivement le degre du polynome."""
    print()
    print("=========================================")
    print("CHOIX DU DEGRE DU MODELE")
    print("=========================================")
    print("1 : lineaire")
    print("2 : quadratique")
    print("3 : cubique")
    print("4 : degre 4")
    print()

    while True:
        reponse = input("Degre du polynome [1-4] > ").strip()
        try:
            degre = int(reponse)
        except ValueError:
            print("  Saisie invalide. Entrer 1, 2, 3 ou 4.")
            continue

        if degre in (1, 2, 3, 4):
            return degre

        print("  Saisie invalide. Entrer 1, 2, 3 ou 4.")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Apprentissage d'un modele polynomial 2D "
            "pour corriger les erreurs de pointage de Calib 2."
        )
    )

    # Le degre est choisi interactively au lancement.
    args = parser.parse_args()

    try:
        degre = demander_degre()

        donnees = lire_donnees(
            SERIE
        )

        if not donnees:
            print(
                "Aucune mesure trouvee pour la serie :",
                SERIE,
            )
            print(
                "Verifie que calibration_test.py a bien "
                "enregistre des mesures dans :"
            )
            print(
                " ",
                FICHIER_APPRENTISSAGE,
            )
            return 1

        modele = ajuster_polynome(
            donnees,
            degre,
        )

        modele["serie"] = SERIE

        chemin_sortie = os.path.join(
            DOSSIER_RESULTATS,
            "modele_correction_%s.json"
            % nom_fichier_securise(SERIE),
        )

        sauvegarder_modele(
            modele,
            chemin_sortie,
        )

        afficher_resultats(
            donnees,
            modele,
        )

        print()
        print(
            "Modele sauvegarde dans :"
        )
        print(
            " ",
            chemin_sortie,
        )

        return 0

    except Exception as e:
        print(
            "ERREUR pendant l'apprentissage :",
            e,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
