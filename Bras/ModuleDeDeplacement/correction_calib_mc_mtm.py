"""
=========================================================
TEST DE LA CALIBRATION - CALIB 2 + CORRECTION APPRENTIE
=========================================================

Principe :
----------
1. L'utilisateur donne une cible X/Y en carreaux.
2. Le modele de correction de Calib 2 predit :
       erreur_x = f(x, y)
       erreur_y = g(x, y)
3. La cible est corrigee :
       x_corrige = x - erreur_x
       y_corrige = y - erreur_y
4. La calibration geometrique calcule J4/J5 a partir
   de la cible corrigee.
5. viserAngles() ameliore l'atteinte des angles.
6. L'utilisateur mesure le point reel.
7. L'erreur est toujours calculee par rapport a la cible
   ORIGINALE afin de continuer l'apprentissage.

Toutes les positions sont en CARREAUX.

Calibration utilisee :
    Calib 2

Correction appliquee :
    X : +0 carreau
    Y : +1 carreau

Mesures d'apprentissage :
    resultats/apprentissage_erreurs.csv
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv

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


from PU_PositionInitiale import allerPositionInitiale


# =========================================================
# 2. CONFIGURATION
# =========================================================

SERIE = "Calib 2"


FICHIER_VALIDATION = os.path.join(
    DOSSIER_RESULTATS,
    "validation_pointage_Calib_2.csv",
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
# 3. CORRECTION FIXE
# =========================================================

OFFSET_X = 0.0
OFFSET_Y = 1.0


def corrigerCible(x, y):
    """Applique une correction fixe : X + 0, Y + 1 carreau."""
    erreur_x_predite = 0.0
    erreur_y_predite = -1.0
    x_corrige = x + OFFSET_X
    y_corrige = y + OFFSET_Y
    return x_corrige, y_corrige, erreur_x_predite, erreur_y_predite


# =========================================================
## 4. ENREGISTREMENT DE LA MESURE
# =========================================================

def enregistrerTir(
    demande,
    angles,
    reel,
    chemin=None
):
    """
    Enregistre la mesure dans le fichier de validation.
    """

    if reel is None:

        ecart = ""
        x_reel = ""
        y_reel = ""

    else:

        ecart = round(
            (
                (reel[0] - demande[0]) ** 2
                +
                (reel[1] - demande[1]) ** 2
            ) ** 0.5,
            3
        )

        x_reel = round(
            reel[0],
            3
        )

        y_reel = round(
            reel[1],
            3
        )

    try:

        os.makedirs(
            DOSSIER_RESULTATS,
            exist_ok=True
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
            newline=""
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
                round(
                    demande[0],
                    3
                ),
                round(
                    demande[1],
                    3
                ),
                round(
                    angles[0],
                    3
                ),
                round(
                    angles[1],
                    3
                ),
                x_reel,
                y_reel,
                ecart,
            ])

        return True

    except Exception as e:

        print(
            "Impossible d'enregistrer le tir :",
            e
        )

        return False


def enregistrerMesureApprentissage(
    demande,
    reel
):
    """
    Enregistre les donnees necessaires a un prochain
    apprentissage.

    IMPORTANT :
    la cible est la cible ORIGINALE de l'utilisateur,
    pas la cible corrigee envoyee au robot.
    """

    if reel is None:

        return False

    x_vise = demande[0]
    y_vise = demande[1]

    x_mesure = reel[0]
    y_mesure = reel[1]

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
            exist_ok=True
        )

        nouveau = not os.path.exists(
            FICHIER_APPRENTISSAGE
        )

        with open(
            FICHIER_APPRENTISSAGE,
            "a",
            newline=""
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
                SERIE,
                round(
                    x_vise,
                    5
                ),
                round(
                    y_vise,
                    5
                ),
                round(
                    x_mesure,
                    5
                ),
                round(
                    y_mesure,
                    5
                ),
                round(
                    erreur_x,
                    5
                ),
                round(
                    erreur_y,
                    5
                ),
            ])

        return True

    except Exception as e:

        print(
            "Impossible d'enregistrer la mesure d'apprentissage :",
            e
        )

        return False


# =========================================================
# 6. CALIBRATION GEOMETRIQUE : CALIB 2 UNIQUEMENT
# =========================================================

def chargerBaseSerie():
    """
    Charge J1/J2/J3 de Calib 2.
    """

    try:

        with open(
            FICHIER_MESURES,
            "r",
            newline=""
        ) as fichier:

            for ligne in csv.DictReader(
                fichier
            ):

                serie = (
                    ligne.get("serie")
                    or ""
                ).strip()

                if serie != SERIE:

                    continue

                try:

                    return [
                        float(
                            ligne["J1"]
                        ),
                        float(
                            ligne["J2"]
                        ),
                        float(
                            ligne["J3"]
                        )
                    ]

                except (
                    ValueError,
                    KeyError
                ):

                    continue

    except Exception as e:

        print(
            "Impossible de lire la base de la calibration :",
            e
        )

    return None


def construireModeleCalib2():
    """
    Reconstruit exclusivement la calibration 'Calib 2'.
    """

    ajustement = chargerMesures(
        usage="ajustement",
        serie=SERIE
    )

    validation = chargerMesures(
        usage="validation",
        serie=SERIE
    )

    if len(ajustement) < 5:

        print()
        print(
            "ERREUR : Calib 2 contient seulement %d "
            "points d'ajustement."
            % len(ajustement)
        )

        print(
            "Il en faut au moins 5."
        )

        return None

    base_reference = chargerBaseSerie()

    modele = ajusterModele(
        ajustement,
        base=base_reference
    )

    if modele is None:

        print(
            "ERREUR : impossible de reconstruire Calib 2."
        )

        return None

    modele["serie"] = SERIE

    if validation:

        modele["residus_validation"] = (
            calculerResidus(
                modele,
                validation
            )
        )

    return modele


# =========================================================
# 7. DIALOGUE UTILISATEUR
# =========================================================

def lireCible():
    """
    Demande une cible X/Y en carreaux.
    """

    print()
    print(
        "Point a viser : deux nombres en CARREAUX "
        "(ex : 4 -2)."
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
            "q"
        ):

            return None

        morceaux = (
            reponse
            .replace(
                ",",
                " "
            )
            .split()
        )

        if len(morceaux) != 2:

            print(
                "  Il faut deux nombres separes par un espace."
            )

            continue

        try:

            x = float(
                morceaux[0]
            )

            y = float(
                morceaux[1]
            )

            return (
                x,
                y
            )

        except ValueError:

            print(
                "  Saisie illisible."
            )


def lireImpactReel(
    demande
):
    """
    Demande la position reelle du spot en carreaux.
    """

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
        .replace(
            ",",
            " "
        )
        .split()
    )

    if len(morceaux) != 2:

        print(
            "  Saisie ignoree."
        )

        return None

    try:

        reel = (
            float(
                morceaux[0]
            ),
            float(
                morceaux[1]
            )
        )

    except ValueError:

        print(
            "  Saisie ignoree."
        )

        return None

    ecart = (
        (
            (reel[0] - demande[0]) ** 2
            +
            (reel[1] - demande[1]) ** 2
        ) ** 0.5
    )

    print(
        "  Ecart de pointage : %.2f carreaux (%.1f cm)"
        % (
            ecart,
            ecart * COTE_CARREAU
        )
    )

    return reel


# =========================================================
# 8. GRAPHIQUES 3D
# =========================================================

def tracerErreurs3D(
    donnees
):
    """
    Trace deux courbes 3D :
        1) erreur X
        2) erreur Y

    Abscisse  = X visee
    Profondeur = Y visee
    Hauteur = erreur
    """

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
        d["x_mesure"]
        - d["x_vise"]
        for d in donnees
    ]

    erreur_y = [
        d["y_mesure"]
        - d["y_vise"]
        for d in donnees
    ]

    fig = plt.figure(
        figsize=(
            14,
            7
        )
    )

    # -----------------------------------------------------
    # Erreur X
    # -----------------------------------------------------

    ax1 = fig.add_subplot(
        121,
        projection="3d"
    )

    ax1.plot(
        x_vise,
        y_vise,
        erreur_x,
        marker="o",
        linewidth=2
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

    # -----------------------------------------------------
    # Erreur Y
    # -----------------------------------------------------

    ax2 = fig.add_subplot(
        122,
        projection="3d"
    )

    ax2.plot(
        x_vise,
        y_vise,
        erreur_y,
        marker="o",
        linewidth=2
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
        "Fermer la fenetre graphique "
        "pour terminer le programme."
    )

    plt.show()


# =========================================================
# 9. PROGRAMME PRINCIPAL
# =========================================================

def main():

    print(
        "========================================="
    )

    print(
        "TEST DE LA CALIBRATION - CALIB 2"
    )

    print(
        "========================================="
    )

    print()

    print(
        "SECURITE : pointeur rouge classe 2, "
        "laser de puissance DEBRANCHE."
    )

    print()

    print(
        "Calibration utilisee :",
        SERIE
    )

    print()
    print(
        "Correction fixe : X + %.1f, Y + %.1f carreau"
        % (
            OFFSET_X,
            OFFSET_Y
        )
    )

    # -----------------------------------------------------
    # Construction de Calib 2
    # -----------------------------------------------------

    print()
    print(
        "Reconstruction de Calib 2..."
    )

    modele = construireModeleCalib2()

    if modele is None:

        return 1

    print()

    print(
        "Calib 2 chargee : %d points d'ajustement"
        % modele.get(
            "nb_points",
            0
        )
    )

    print(
        "Zone calibree : "
        "x %.1f..%.1f, y %.1f..%.1f carreaux"
        % (
            modele["enveloppe"]["u_min"],
            modele["enveloppe"]["u_max"],
            modele["enveloppe"]["v_min"],
            modele["enveloppe"]["v_max"]
        )
    )

    residus = modele.get(
        "residus"
    )

    if residus:

        print(
            "Erreur ajustement : "
            "rms %.2f carreaux, max %.2f carreaux"
            % (
                residus["rms_carreaux"],
                residus["max_carreaux"]
            )
        )

    residus_validation = modele.get(
        "residus_validation"
    )

    if residus_validation:

        print(
            "Erreur validation : "
            "rms %.2f carreaux, max %.2f carreaux"
            % (
                residus_validation["rms_carreaux"],
                residus_validation["max_carreaux"]
            )
        )

    else:

        print(
            "Pas de validation pour Calib 2."
        )


    # -----------------------------------------------------
    # Connexion au robot
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:

        return 1

    allerPositionInitiale(
        mc
    )

    angles = lireAngles(
        mc
    )

    if angles is None:

        print(
            "Lecture des angles impossible."
        )

        return 1

    base = angles[
        0:3
    ]

    j6 = angles[
        5
    ]

    print()

    print(
        "Les mesures seront enregistrees dans :"
    )

    print(
        " ",
        FICHIER_VALIDATION
    )

    print()

    print(
        "Les mesures d'apprentissage seront "
        "enregistrees dans :"
    )

    print(
        " ",
        FICHIER_APPRENTISSAGE
    )

    donnees_graphique = []

    try:

        if not allumerPointeur(
            mc
        ):

            print(
                "Le pointeur ne s'allume pas."
            )

        # =================================================
        # BOUCLE DE CIBLAGE
        # =================================================

        while True:

            demande = lireCible()

            # -------------------------------------------------
            # FIN
            # -------------------------------------------------

            if demande is None:

                print()
                print(
                    "Fin de la campagne de mesure."
                )

                tracerErreurs3D(
                    donnees_graphique
                )

                break

            # -------------------------------------------------
            # CIBLE ORIGINALE
            # -------------------------------------------------

            x = demande[0]
            y = demande[1]

            print()
            print(
                "-----------------------------------------"
            )

            print(
                "Cible demandee : "
                "X = %+.3f   Y = %+.3f carreaux"
                % (
                    x,
                    y
                )
            )

            # -------------------------------------------------
            # Verification de l'enveloppe sur la cible
            # originale.
            # -------------------------------------------------

            if not dansEnveloppe(
                modele,
                x,
                y
            ):

                e = modele[
                    "enveloppe"
                ]

                print(
                    "  Point hors de la zone de "
                    "Calib 2."
                )

                print(
                    "  Zone autorisee : "
                    "x %.1f..%.1f, "
                    "y %.1f..%.1f carreaux."
                    % (
                        e["u_min"],
                        e["u_max"],
                        e["v_min"],
                        e["v_max"]
                    )
                )

                continue

            # =================================================
            # CORRECTION FIXE
            # =================================================

            (
                x_corrige,
                y_corrige,
                erreur_x_predite,
                erreur_y_predite
            ) = corrigerCible(
                x,
                y
            )

            print()
            print(
                "Correction fixe appliquee :"
            )

            print(
                "  Offset X : %+.3f carreau"
                % OFFSET_X
            )

            print(
                "  Offset Y : %+.3f carreau"
                % OFFSET_Y
            )

            print()
            print(
                "Cible corrigee : "
                "X = %+.3f   Y = %+.3f carreaux"
                % (
                    x_corrige,
                    y_corrige
                )
            )

            # -------------------------------------------------
            # Verification de l'enveloppe de la cible corrigee
            # -------------------------------------------------

            if not dansEnveloppe(
                modele,
                x_corrige,
                y_corrige
            ):

                print()
                print(
                    "ATTENTION : la cible corrigee sort "
                    "de l'enveloppe de Calib 2."
                )

                print(
                    "Tir annule pour eviter "
                    "une extrapolation."
                )

                continue

            # =================================================
            # CALCUL DES ANGLES
            # =================================================

            resultat = anglesPourPoint(
                modele,
                x_corrige,
                y_corrige
            )

            if resultat is None:

                print(
                    "Impossible de calculer J4/J5 "
                    "pour la cible corrigee."
                )

                continue

            j4, j5 = resultat

            print()
            print(
                "Angles calcules sur la cible corrigee :"
            )

            print(
                "  J4 = %+.2f"
                % j4
            )

            print(
                "  J5 = %+.2f"
                % j5
            )

            # =================================================
            # POSITIONNEMENT AVEC RATTRAPAGE
            # =================================================

            vise, mesure = viserAngles(
                mc,
                base,
                j4,
                j5,
                j6
            )

            if mesure is None:

                print(
                    "Mouvement echoue."
                )

                continue

            if not vise:

                print()

                print(
                    "ATTENTION : la tete se repose "
                    "a %+.2f / %+.2f deg de l'angle demande."
                    % (
                        mesure[3] - j4,
                        mesure[4] - j5
                    )
                )

                print(
                    "Tolerance : %.2f deg."
                    % TOLERANCE_POINTAGE
                )

            # =================================================
            # PREDICTION DU MODELE GEOMETRIQUE
            # =================================================

            prediction = predireImpact(
                modele,
                mesure[3],
                mesure[4]
            )

            if prediction is not None:

                print()
                print(
                    "Prediction geometrique sur les "
                    "angles atteints :"
                )

                print(
                    "  X = %+.3f"
                    % prediction[0]
                )

                print(
                    "  Y = %+.3f"
                    % prediction[1]
                )

            # =================================================
            # MESURE DU SPOT REEL
            # =================================================

            reel = lireImpactReel(
                demande
            )

            # =================================================
            # APPRENTISSAGE
            # =================================================

            if reel is not None:

                # -------------------------------------------------
                # IMPORTANT :
                # la comparaison se fait avec la cible originale.
                # -------------------------------------------------

                erreur_x_reelle = (
                    reel[0]
                    - x
                )

                erreur_y_reelle = (
                    reel[1]
                    - y
                )

                print()
                print(
                    "Erreur reellement mesuree :"
                )

                print(
                    "  Erreur X = %+.3f carreaux"
                    % erreur_x_reelle
                )

                print(
                    "  Erreur Y = %+.3f carreaux"
                    % erreur_y_reelle
                )

                # Donnees pour les deux graphiques 3D.
                donnees_graphique.append({
                    "x_vise": x,
                    "y_vise": y,
                    "x_mesure": reel[0],
                    "y_mesure": reel[1],
                })

                # Donnees pour un futur apprentissage.
                enregistrerMesureApprentissage(
                    demande,
                    reel
                )

            # =================================================
            # ENREGISTREMENT DE VALIDATION
            # =================================================

            enregistrerTir(
                demande,
                (
                    mesure[3],
                    mesure[4]
                ),
                reel,
                chemin=FICHIER_VALIDATION
            )

    finally:

        eteindrePointeur(
            mc
        )

        print()
        print(
            "Pointeur eteint. "
            "Moteurs laisses sous tension."
        )

        print(
            "Calibration utilisee :",
            SERIE
        )

        print(
            "Tirs enregistres dans :"
        )

        print(
            " ",
            FICHIER_VALIDATION
        )

        print(
            "Apprentissage disponible dans :"
        )

        print(
            " ",
            FICHIER_APPRENTISSAGE
        )

    return 0


# =========================================================
# 10. EXECUTION
# =========================================================

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

