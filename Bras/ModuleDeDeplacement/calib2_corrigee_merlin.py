"""
=========================================================
TEST DE LA CALIBRATION - CALIB 2 + CORRECTION APPRENTIE
                 CAMPAGNE ALEATOIRE
=========================================================

Principe :
----------

1. L'utilisateur choisit le nombre de positions a tester.
2. Le programme genere des positions X/Y aleatoires
   dans l'enveloppe de Calib 2.
3. Le modele de correction de Calib 2 predit :
       erreur_x = f(x, y)
       erreur_y = g(x, y)
4. La cible est corrigee :
       x_corrige = x - erreur_x
       y_corrige = y - erreur_y
5. La calibration geometrique calcule J4/J5.
6. Le robot se deplace vers la cible corrigee.
7. Le temps de deplacement est mesure.
8. Le laser reste ETEINT pendant le mouvement.
9. Si le robot atteint correctement la position,
   le laser est allume.
10. L'utilisateur mesure alors la position reelle du spot.
11. Le laser est eteint avant le deplacement suivant.
12. Toutes les donnees sont enregistrees dans un CSV.

Toutes les positions sont en CARREAUX.

Calibration utilisee :
    Calib 2

Modele de correction :
    resultats/modele_correction_Calib_2.json

Fichier de campagne :
    resultats/campagne_aleatoire_Calib_2.csv
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
import re
import json
import random
import time

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
    predireImpact,
)


from PU_PositionInitiale import allerPositionInitiale


# =========================================================
# 2. CONFIGURATION
# =========================================================

SERIE = "Calib 2"


FICHIER_MESURES = os.path.join(
    DOSSIER_RESULTATS,
    "calibration_grille.csv",
)


FICHIER_MODELE_CORRECTION = os.path.join(
    DOSSIER_RESULTATS,
    "modele_correction_Calib_2.json",
)


FICHIER_CAMPAGNE = os.path.join(
    DOSSIER_RESULTATS,
    "campagne_aleatoire_Calib_2.csv",
)


ENTETE_CAMPAGNE = [
    "horodatage",
    "numero_tir",
    "x_demande",
    "y_demande",
    "erreur_x_predite",
    "erreur_y_predite",
    "x_corrige",
    "y_corrige",
    "J4_consigne",
    "J5_consigne",
    "J4_atteint",
    "J5_atteint",
    "temps_deplacement_s",
    "position_atteinte",
    "x_mesure",
    "y_mesure",
    "erreur_x",
    "erreur_y",
    "erreur_totale",
]


# =========================================================
# 3. UTILITAIRES
# =========================================================

def nomFichierSecurise(serie):
    """
    Transforme un nom de serie en nom de fichier
    sans caracteres dangereux.
    """

    propre = re.sub(
        r"[^A-Za-z0-9._-]+",
        "_",
        serie
    ).strip("._")

    return propre or "calibration"


# =========================================================
# 4. CHARGEMENT DU MODELE DE CORRECTION
# =========================================================

def chargerModeleCorrection():
    """
    Charge automatiquement le modele polynomial de Calib 2.
    """

    print()
    print("=========================================")
    print("MODELE DE CORRECTION")
    print("=========================================")

    print(
        "Calibration utilisee :",
        SERIE
    )

    print(
        "Recherche du modele :"
    )

    print(
        " ",
        FICHIER_MODELE_CORRECTION
    )

    if not os.path.exists(
        FICHIER_MODELE_CORRECTION
    ):

        print()
        print(
            "ATTENTION : aucun modele de correction n'a ete trouve."
        )

        print(
            "Le robot fonctionnera sans correction apprises."
        )

        return None

    try:

        with open(
            FICHIER_MODELE_CORRECTION,
            "r"
        ) as fichier:

            modele = json.load(
                fichier
            )

    except Exception as e:

        print()
        print(
            "ERREUR : impossible de lire le modele de correction :",
            e
        )

        return None

    serie_modele = (
        modele.get("serie") or ""
    ).strip()

    if serie_modele != SERIE:

        print()
        print(
            "ERREUR : le modele charge ne correspond pas a Calib 2."
        )

        print(
            "Serie attendue :",
            SERIE
        )

        print(
            "Serie trouvee  :",
            serie_modele
        )

        return None

    champs_requis = [
        "degre",
        "centre_x",
        "centre_y",
        "echelle_x",
        "echelle_y",
        "termes",
        "coefficients_erreur_x",
        "coefficients_erreur_y",
    ]

    manquants = [
        champ
        for champ in champs_requis
        if champ not in modele
    ]

    if manquants:

        print()
        print(
            "ERREUR : le modele est incomplet."
        )

        print(
            "Champs manquants :",
            manquants
        )

        return None

    print()
    print(
        "Modele de correction charge."
    )

    print(
        "  Degre polynomial :",
        modele["degre"]
    )

    if "nb_points" in modele:

        print(
            "  Nombre de points d'apprentissage :",
            modele["nb_points"]
        )

    if "rmse_erreur_x" in modele:

        print(
            "  RMSE erreur X : %.4f carreaux"
            % modele["rmse_erreur_x"]
        )

    if "rmse_erreur_y" in modele:

        print(
            "  RMSE erreur Y : %.4f carreaux"
            % modele["rmse_erreur_y"]
        )

    return modele


# =========================================================
# 5. PREDICTION DE L'ERREUR
# =========================================================

def predireCorrection(
    modele,
    x,
    y
):
    """
    Calcule l'erreur predite pour la cible (x, y).
    """

    if modele is None:

        return (
            0.0,
            0.0
        )

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

    if abs(echelle_x) < 1e-12:

        echelle_x = 1.0

    if abs(echelle_y) < 1e-12:

        echelle_y = 1.0

    x_normalise = (
        x - centre_x
    ) / echelle_x

    y_normalise = (
        y - centre_y
    ) / echelle_y

    valeurs = []

    for terme in modele["termes"]:

        i = int(
            terme[0]
        )

        j = int(
            terme[1]
        )

        valeur = (
            (x_normalise ** i)
            *
            (y_normalise ** j)
        )

        valeurs.append(
            valeur
        )

    coefficients_x = (
        modele["coefficients_erreur_x"]
    )

    erreur_x = sum(
        float(coefficient) * valeur
        for coefficient, valeur
        in zip(
            coefficients_x,
            valeurs
        )
    )

    coefficients_y = (
        modele["coefficients_erreur_y"]
    )

    erreur_y = sum(
        float(coefficient) * valeur
        for coefficient, valeur
        in zip(
            coefficients_y,
            valeurs
        )
    )

    return (
        float(erreur_x),
        float(erreur_y)
    )


# =========================================================
# 6. CALCUL DE LA CIBLE CORRIGEE
# =========================================================

def corrigerCible(
    modele,
    x,
    y
):
    """
    Calcule la cible corrigee.

        x_corrige = x - erreur_x
        y_corrige = y - erreur_y
    """

    erreur_x, erreur_y = predireCorrection(
        modele,
        x,
        y
    )

    x_corrige = (
        x - erreur_x
    )

    y_corrige = (
        y - erreur_y
    )

    return (
        x_corrige,
        y_corrige,
        erreur_x,
        erreur_y
    )


# =========================================================
# 7. VERIFICATION DE LA BASE
# =========================================================

def verifierBase(
    mc,
    modele,
    tolerance=TOLERANCE_BASE
):
    """
    Compare la base courante a celle de la calibration.
    """

    reference = modele.get(
        "base"
    )

    if not reference:

        print(
            "ATTENTION : le modele ne contient pas la pose de base."
        )

        print(
            "Impossible de verifier que le bras n'a pas bouge."
        )

        return True

    mesure = lireAngles(
        mc
    )

    if mesure is None:

        print(
            "Lecture des angles impossible."
        )

        return False

    ecarts = [
        abs(
            mesure[i]
            - reference[i]
        )
        for i in range(3)
    ]

    print()
    print(
        "Base de calibration :",
        [
            round(a, 2)
            for a in reference
        ]
    )

    print(
        "Base courante       :",
        [
            round(mesure[i], 2)
            for i in range(3)
        ]
    )

    print(
        "Ecarts              :",
        [
            round(e, 2)
            for e in ecarts
        ]
    )

    if max(ecarts) > tolerance:

        print()
        print(
            "REFUS : la base a bouge de %.2f deg depuis la calibration."
            % max(ecarts)
        )

        print(
            "Le modele ne decrit plus la geometrie actuelle."
        )

        print(
            "Remettre le bras dans sa pose de calibration,"
        )

        print(
            "ou refaire une campagne."
        )

        return False

    print(
        "Base conforme."
    )

    return True


# =========================================================
# 8. CHARGEMENT DE CALIB 2
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
    Reconstruit exclusivement la calibration Calib 2.
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
# 9. GENERATION DES POSITIONS ALEATOIRES
# =========================================================

def demanderNombrePositions():
    """
    Demande combien de positions aleatoires seront testees.
    """

    print()
    print("=========================================")
    print("CAMPAGNE ALEATOIRE")
    print("=========================================")

    while True:

        reponse = input(
            "Nombre de positions a tester > "
        ).strip()

        try:

            nombre = int(
                reponse
            )

        except ValueError:

            print(
                "  Saisir un nombre entier positif."
            )

            continue

        if nombre > 0:

            return nombre

        print(
            "  Le nombre doit etre superieur a 0."
        )


def genererPositionsAleatoires(
    modele,
    nombre
):
    """
    Genere des positions entieres aleatoires en carreaux.

    Les doublons sont evites autant que possible.
    """

    enveloppe = modele["enveloppe"]

    x_min = int(
        round(
            enveloppe["u_min"]
        )
    )

    x_max = int(
        round(
            enveloppe["u_max"]
        )
    )

    y_min = int(
        round(
            enveloppe["v_min"]
        )
    )

    y_max = int(
        round(
            enveloppe["v_max"]
        )
    )

    toutes_positions = [
        (
            x,
            y
        )
        for x in range(
            x_min,
            x_max + 1
        )
        for y in range(
            y_min,
            y_max + 1
        )
        if dansEnveloppe(
            modele,
            x,
            y
        )
    ]

    if not toutes_positions:

        raise RuntimeError(
            "Aucune position entiere disponible "
            "dans l'enveloppe de Calib 2."
        )

    if nombre > len(toutes_positions):

        print()
        print(
            "ATTENTION : %d positions demandees, "
            "mais seulement %d positions distinctes "
            "sont disponibles."
            % (
                nombre,
                len(toutes_positions)
            )
        )

        print(
            "Les positions seront reutilisees."
        )

        positions = []

        while len(positions) < nombre:

            positions.append(
                random.choice(
                    toutes_positions
                )
            )

        return positions

    return random.sample(
        toutes_positions,
        nombre
    )


# =========================================================
# 10. ENREGISTREMENT CSV
# =========================================================

def initialiserFichierCampagne():
    """
    Cree le fichier CSV et ecrit l'en-tete s'il est nouveau.
    """

    os.makedirs(
        DOSSIER_RESULTATS,
        exist_ok=True
    )

    nouveau = not os.path.exists(
        FICHIER_CAMPAGNE
    )

    if nouveau:

        with open(
            FICHIER_CAMPAGNE,
            "w",
            newline=""
        ) as fichier:

            redacteur = csv.writer(
                fichier
            )

            redacteur.writerow(
                ENTETE_CAMPAGNE
            )


def enregistrerResultat(
    numero,
    demande,
    erreur_predite,
    cible_corrigee,
    angles_consigne,
    angles_atteints,
    temps_deplacement,
    position_atteinte,
    reel
):
    """
    Enregistre toutes les informations d'un point dans le CSV.
    """

    if reel is None:

        x_mesure = ""
        y_mesure = ""
        erreur_x = ""
        erreur_y = ""
        erreur_totale = ""

    else:

        x_mesure = reel[0]
        y_mesure = reel[1]

        erreur_x = (
            x_mesure
            - demande[0]
        )

        erreur_y = (
            y_mesure
            - demande[1]
        )

        erreur_totale = (
            (
                erreur_x ** 2
                +
                erreur_y ** 2
            ) ** 0.5
        )

    with open(
        FICHIER_CAMPAGNE,
        "a",
        newline=""
    ) as fichier:

        redacteur = csv.writer(
            fichier
        )

        redacteur.writerow([
            datetime.now().isoformat(
                timespec="seconds"
            ),

            numero,

            round(
                demande[0],
                5
            ),

            round(
                demande[1],
                5
            ),

            round(
                erreur_predite[0],
                5
            ),

            round(
                erreur_predite[1],
                5
            ),

            round(
                cible_corrigee[0],
                5
            ),

            round(
                cible_corrigee[1],
                5
            ),

            round(
                angles_consigne[0],
                5
            ),

            round(
                angles_consigne[1],
                5
            ),

            round(
                angles_atteints[0],
                5
            ),

            round(
                angles_atteints[1],
                5
            ),

            round(
                temps_deplacement,
                5
            ),

            int(
                position_atteinte
            ),

            x_mesure,
            y_mesure,
            erreur_x,
            erreur_y,
            erreur_totale,
        ])


# =========================================================
# 11. GRAPHIQUES 3D
# =========================================================

def tracerErreurs3D(
    donnees
):
    """
    Trace les erreurs mesurees pendant la campagne.
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
        "Affichage des courbes 3D."
    )

    print(
        "Fermer la fenetre pour terminer."
    )

    plt.show()


# =========================================================
# 12. PROGRAMME PRINCIPAL
# =========================================================

def main():

    print(
        "========================================="
    )

    print(
        "CAMPAGNE ALEATOIRE - CALIB 2"
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

    # -----------------------------------------------------
    # Reconstruction de Calib 2
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

    # -----------------------------------------------------
    # Chargement correction
    # -----------------------------------------------------

    modele_correction = (
        chargerModeleCorrection()
    )

    # -----------------------------------------------------
    # Nombre de positions
    # -----------------------------------------------------

    nombre_positions = (
        demanderNombrePositions()
    )

    # -----------------------------------------------------
    # Generation aleatoire
    # -----------------------------------------------------

    try:

        positions = genererPositionsAleatoires(
            modele,
            nombre_positions
        )

    except Exception as e:

        print(
            "ERREUR generation positions :",
            e
        )

        return 1

    print()
    print(
        "%d positions aleatoires generees."
        % len(positions)
    )

    # -----------------------------------------------------
    # Initialisation CSV
    # -----------------------------------------------------

    initialiserFichierCampagne()

    print()
    print(
        "Les resultats seront enregistres dans :"
    )

    print(
        " ",
        FICHIER_CAMPAGNE
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

    base = angles[
        0:3
    ]

    j6 = angles[
        5
    ]

    donnees_graphique = []

    # =====================================================
    # BOUCLE PRINCIPALE
    # =====================================================

    try:

        # Le laser est explicitement eteint au depart.
        eteindrePointeur(
            mc
        )

        for numero, demande in enumerate(
            positions,
            1
        ):

            print()
            print()
            print(
                "========================================="
            )

            print(
                "POSITION %d / %d"
                % (
                    numero,
                    len(positions)
                )
            )

            print(
                "========================================="
            )

            x = demande[0]
            y = demande[1]

            print()
            print(
                "Position demandee : "
                "X = %+.3f   Y = %+.3f carreaux"
                % (
                    x,
                    y
                )
            )

            # -------------------------------------------------
            # Correction
            # -------------------------------------------------

            (
                erreur_x_predite,
                erreur_y_predite
            ) = predireCorrection(
                modele_correction,
                x,
                y
            )

            (
                x_corrige,
                y_corrige,
                _,
                _
            ) = corrigerCible(
                modele_correction,
                x,
                y
            )

            print()
            print(
                "Erreur predite : "
                "X = %+.3f   Y = %+.3f carreaux"
                % (
                    erreur_x_predite,
                    erreur_y_predite
                )
            )

            print(
                "Position corrigee : "
                "X = %+.3f   Y = %+.3f carreaux"
                % (
                    x_corrige,
                    y_corrige
                )
            )

            # -------------------------------------------------
            # Verification enveloppe cible corrigee
            # -------------------------------------------------

            if not dansEnveloppe(
                modele,
                x_corrige,
                y_corrige
            ):

                print()
                print(
                    "ATTENTION : la position corrigee sort "
                    "de l'enveloppe."
                )

                print(
                    "Tir ignore."
                )

                continue

            # -------------------------------------------------
            # Calcul angles
            # -------------------------------------------------

            resultat = anglesPourPoint(
                modele,
                x_corrige,
                y_corrige
            )

            if resultat is None:

                print(
                    "Impossible de calculer les angles."
                )

                continue

            j4, j5 = resultat

            print()
            print(
                "Consigne angulaire :"
            )

            print(
                "  J4 = %+.2f deg"
                % j4
            )

            print(
                "  J5 = %+.2f deg"
                % j5
            )

            # -------------------------------------------------
            # SECURITE :
            # laser eteint avant chaque mouvement.
            # -------------------------------------------------

            eteindrePointeur(
                mc
            )

            print()
            print(
                "Laser eteint."
            )

            print(
                "Deplacement en cours..."
            )

            # -------------------------------------------------
            # CHRONOMETRAGE DU DEPLACEMENT
            # -------------------------------------------------

            debut = time.perf_counter()

            vise, mesure = viserAngles(
                mc,
                base,
                j4,
                j5,
                j6
            )

            fin = time.perf_counter()

            temps_deplacement = (
                fin - debut
            )

            # -------------------------------------------------
            # Mouvement echoue
            # -------------------------------------------------

            if mesure is None:

                print()
                print(
                    "Mouvement echoue."
                )

                print(
                    "Temps de deplacement : %.3f s"
                    % temps_deplacement
                )

                enregistrerResultat(
                    numero,
                    demande,
                    (
                        erreur_x_predite,
                        erreur_y_predite
                    ),
                    (
                        x_corrige,
                        y_corrige
                    ),
                    (
                        j4,
                        j5
                    ),
                    (
                        float("nan"),
                        float("nan")
                    ),
                    temps_deplacement,
                    False,
                    None
                )

                # Laser reste eteint.
                eteindrePointeur(
                    mc
                )

                continue

            # -------------------------------------------------
            # Temps de mouvement
            # -------------------------------------------------

            print()
            print(
                "Temps de deplacement : %.3f secondes"
                % temps_deplacement
            )

            print(
                "Angles atteints : "
                "J4 = %+.2f   J5 = %+.2f"
                % (
                    mesure[3],
                    mesure[4]
                )
            )

            # -------------------------------------------------
            # VERIFICATION ATTEINTE
            # -------------------------------------------------

            if not vise:

                print()
                print(
                    "ATTENTION : la position demandee "
                    "n'a pas ete atteinte dans la tolerance."
                )

                print(
                    "Le laser reste ETEINT."
                )

                print(
                    "Ecart angulaire : "
                    "J4 = %+.3f deg / J5 = %+.3f deg"
                    % (
                        mesure[3] - j4,
                        mesure[4] - j5
                    )
                )

                # Securite supplementaire.
                eteindrePointeur(
                    mc
                )

                enregistrerResultat(
                    numero,
                    demande,
                    (
                        erreur_x_predite,
                        erreur_y_predite
                    ),
                    (
                        x_corrige,
                        y_corrige
                    ),
                    (
                        j4,
                        j5
                    ),
                    (
                        mesure[3],
                        mesure[4]
                    ),
                    temps_deplacement,
                    False,
                    None
                )

                continue

            # =================================================
            # ROBOT ARRIVE :
            # ALLUMER LE LASER MAINTENANT SEULEMENT
            # =================================================

            print()
            print(
                "Position atteinte."
            )

            print(
                "Laser allume."
            )

            laser_ok = allumerPointeur(
                mc
            )

            if not laser_ok:

                print(
                    "ATTENTION : impossible d'allumer le laser."
                )

                enregistrerResultat(
                    numero,
                    demande,
                    (
                        erreur_x_predite,
                        erreur_y_predite
                    ),
                    (
                        x_corrige,
                        y_corrige
                    ),
                    (
                        j4,
                        j5
                    ),
                    (
                        mesure[3],
                        mesure[4]
                    ),
                    temps_deplacement,
                    True,
                    None
                )

                continue

            # -------------------------------------------------
            # Mesure du spot
            # -------------------------------------------------

            print()
            print(
                "Le laser est maintenant allume."
            )

            print(
                "Mesurer la position du spot."
            )

            while True:

                reponse = input(
                    "Position mesuree X Y > "
                ).strip()

                if not reponse:

                    print(
                        "  Une mesure est necessaire."
                    )

                    continue

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
                        "  Entrer deux nombres."
                    )

                    continue

                try:

                    x_mesure = float(
                        morceaux[0]
                    )

                    y_mesure = float(
                        morceaux[1]
                    )

                    reel = (
                        x_mesure,
                        y_mesure
                    )

                    break

                except ValueError:

                    print(
                        "  Saisie illisible."
                    )

            # -------------------------------------------------
            # ERREUR REELLE
            # -------------------------------------------------

            erreur_x_reelle = (
                reel[0]
                - x
            )

            erreur_y_reelle = (
                reel[1]
                - y
            )

            erreur_totale = (
                (
                    erreur_x_reelle ** 2
                    +
                    erreur_y_reelle ** 2
                ) ** 0.5
            )

            print()
            print(
                "Position mesuree : "
                "X = %+.3f   Y = %+.3f"
                % (
                    reel[0],
                    reel[1]
                )
            )

            print(
                "Erreur X : %+.3f carreaux"
                % erreur_x_reelle
            )

            print(
                "Erreur Y : %+.3f carreaux"
                % erreur_y_reelle
            )

            print(
                "Erreur totale : %.3f carreaux"
                % erreur_totale
            )

            # -------------------------------------------------
            # Sauvegarde graphique
            # -------------------------------------------------

            donnees_graphique.append({
                "x_vise": x,
                "y_vise": y,
                "x_mesure": reel[0],
                "y_mesure": reel[1],
            })

            # -------------------------------------------------
            # CSV
            # -------------------------------------------------

            enregistrerResultat(
                numero,
                demande,
                (
                    erreur_x_predite,
                    erreur_y_predite
                ),
                (
                    x_corrige,
                    y_corrige
                ),
                (
                    j4,
                    j5
                ),
                (
                    mesure[3],
                    mesure[4]
                ),
                temps_deplacement,
                True,
                reel
            )

            # -------------------------------------------------
            # IMPORTANT :
            # extinction du laser AVANT le point suivant.
            # -------------------------------------------------

            eteindrePointeur(
                mc
            )

            print()
            print(
                "Laser eteint."
            )

    finally:

        # -----------------------------------------------------
        # SECURITE :
        # le laser doit etre eteint quoi qu'il arrive.
        # -----------------------------------------------------

        eteindrePointeur(
            mc
        )

        print()
        print(
            "========================================="
        )

        print(
            "FIN DE LA CAMPAGNE"
        )

        print(
            "Laser eteint."
        )

        print(
            "Resultats enregistres dans :"
        )

        print(
            " ",
            FICHIER_CAMPAGNE
        )

        print(
            "========================================="
        )

    # -----------------------------------------------------
    # Graphiques
    # -----------------------------------------------------

    tracerErreurs3D(
        donnees_graphique
    )

    return 0


# =========================================================
# 13. EXECUTION
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

