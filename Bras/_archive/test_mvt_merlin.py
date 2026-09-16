"""
=========================================================
CAMPAGNE ALEATOIRE - CALIB 2 + CORRECTION APPRENTIE
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
6. Le robot effectue UN SEUL mouvement vers la cible.
7. Le temps de deplacement est mesure.
8. Le laser reste ETEINT pendant tout le mouvement.
9. Si le mouvement a reussi et qu'une pose finale est
   disponible, le laser est allume pour permettre
   la mesure du spot.
10. L'utilisateur saisit la position reelle du spot.
11. Le laser est eteint avant le point suivant.
12. Toutes les donnees sont enregistrees dans un CSV.
13. Les deux graphiques 3D sont affiches a la fin.

IMPORTANT :
-----------

La tolerance angulaire ne bloque PAS le laser.

Elle sert uniquement a indiquer si les angles reels
sont proches de la consigne.

Un mouvement peut donc etre :

    mouvement_reussi = True
    precision_angulaire = False

Dans ce cas, le laser est quand meme allume afin que
la position reelle du spot puisse etre mesuree.

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

    "mouvement_reussi",
    "precision_angulaire_ok",
    "laser_allume",

    "x_mesure",
    "y_mesure",

    "erreur_x",
    "erreur_y",
    "erreur_totale",
]


# =========================================================
# 3. CHARGEMENT DU MODELE DE CORRECTION
# =========================================================

def chargerModeleCorrection():
    """
    Charge le modele polynomial de Calib 2.
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
# 4. PREDICTION DE L'ERREUR
# =========================================================

def predireCorrection(
    modele,
    x,
    y
):
    """
    Calcule l'erreur predite pour la position (x, y).
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

        valeurs.append(
            (x_normalise ** i)
            *
            (y_normalise ** j)
        )

    erreur_x = sum(
        float(c)
        * valeur

        for c, valeur in zip(
            modele["coefficients_erreur_x"],
            valeurs
        )
    )

    erreur_y = sum(
        float(c)
        * valeur

        for c, valeur in zip(
            modele["coefficients_erreur_y"],
            valeurs
        )
    )

    return (
        float(erreur_x),
        float(erreur_y)
    )


def corrigerCible(
    modele,
    x,
    y
):
    """
    Calcule la cible corrigee.
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
# 5. VERIFICATION DE LA BASE
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

        return False

    print(
        "Base conforme."
    )

    return True


# =========================================================
# 6. CHARGEMENT DE CALIB 2
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
                        float(ligne["J1"]),
                        float(ligne["J2"]),
                        float(ligne["J3"]),
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
    Reconstruit exclusivement Calib 2.
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
# 7. POSITIONS ALEATOIRES
# =========================================================

def demanderNombrePositions():
    """
    Demande le nombre de positions a tester.
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
    Genere des positions entieres aleatoires dans Calib 2.
    """

    enveloppe = modele[
        "enveloppe"
    ]

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

    if nombre <= len(
        toutes_positions
    ):

        return random.sample(
            toutes_positions,
            nombre
        )

    print()
    print(
        "ATTENTION : %d positions demandees, "
        "mais seulement %d positions distinctes sont disponibles."
        % (
            nombre,
            len(toutes_positions)
        )
    )

    print(
        "Certaines positions seront reutilisees."
    )

    return [
        random.choice(
            toutes_positions
        )

        for _ in range(nombre)
    ]


# =========================================================
# 8. CSV
# =========================================================

def initialiserFichierCampagne():
    """
    Cree le CSV s'il n'existe pas.
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

            csv.writer(
                fichier
            ).writerow(
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
    mouvement_reussi,
    precision_angulaire_ok,
    laser_allume,
    reel
):
    """
    Enregistre un tir complet.
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

    j4_atteint = angles_atteints[0]
    j5_atteint = angles_atteints[1]

    with open(
        FICHIER_CAMPAGNE,
        "a",
        newline=""
    ) as fichier:

        csv.writer(
            fichier
        ).writerow([
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

            j4_atteint,
            j5_atteint,

            round(
                temps_deplacement,
                5
            ),

            int(
                mouvement_reussi
            ),

            int(
                precision_angulaire_ok
            ),

            int(
                laser_allume
            ),

            x_mesure,
            y_mesure,

            erreur_x,
            erreur_y,
            erreur_totale,
        ])


# =========================================================
# 9. GRAPHIQUES
# =========================================================

def tracerErreurs3D(
    donnees
):
    """
    Affiche les erreurs X et Y en 3D.
    """

    if not donnees:

        print()
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
# 10. PROGRAMME PRINCIPAL
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
    # Reconstruction calibration
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
    # Modele polynomial
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
    # Positions aleatoires
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
    # CSV
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
    # Robot
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

    try:

        # -------------------------------------------------
        # Toujours commencer laser eteint.
        # -------------------------------------------------

        eteindrePointeur(
            mc
        )

        # =================================================
        # CAMPAGNE
        # =================================================

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
            # Correction spatiale
            # -------------------------------------------------

            (
                x_corrige,
                y_corrige,
                erreur_x_predite,
                erreur_y_predite
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
            # Verification de l'enveloppe
            # -------------------------------------------------

            if not dansEnveloppe(
                modele,
                x_corrige,
                y_corrige
            ):

                print()
                print(
                    "ATTENTION : la position corrigee "
                    "sort de l'enveloppe."
                )

                print(
                    "Aucun mouvement effectue."
                )

                continue

            # -------------------------------------------------
            # Calcul J4/J5
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
            # Laser OFF avant mouvement
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
            # Chronometrage
            # -------------------------------------------------

            debut = time.perf_counter()

            try:

                vise, mesure = viserAngles(
                    mc,
                    base,
                    j4,
                    j5,
                    j6
                )

            except Exception as e:

                print()
                print(
                    "ERREUR pendant le mouvement :",
                    e
                )

                vise = False
                mesure = None

            fin = time.perf_counter()

            temps_deplacement = (
                fin - debut
            )

            # -------------------------------------------------
            # Aucun retour robot
            # -------------------------------------------------

            if mesure is None:

                print()
                print(
                    "Mouvement echoue ou lecture impossible."
                )

                print(
                    "Temps de deplacement : %.3f secondes"
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
                    False,
                    False,
                    None
                )

                eteindrePointeur(
                    mc
                )

                continue

            # -------------------------------------------------
            # Mouvement reussi
            # -------------------------------------------------

            mouvement_reussi = True

            j4_atteint = mesure[3]
            j5_atteint = mesure[4]

            print()
            print(
                "Mouvement termine."
            )

            print(
                "Temps de deplacement : %.3f secondes"
                % temps_deplacement
            )

            print(
                "Angles atteints : "
                "J4 = %+.2f   J5 = %+.2f"
                % (
                    j4_atteint,
                    j5_atteint
                )
            )

            # -------------------------------------------------
            # Precision angulaire
            # -------------------------------------------------

            ecart_j4 = (
                j4
                - j4_atteint
            )

            ecart_j5 = (
                j5
                - j5_atteint
            )

            erreur_angulaire = max(
                abs(ecart_j4),
                abs(ecart_j5)
            )

            precision_angulaire_ok = (
                erreur_angulaire
                <= TOLERANCE_POINTAGE
            )

            if precision_angulaire_ok:

                print()
                print(
                    "Position angulaire dans la tolerance."
                )

            else:

                print()
                print(
                    "ATTENTION : erreur angulaire residuelle."
                )

                print(
                    "  dJ4 = %+.3f deg"
                    % ecart_j4
                )

                print(
                    "  dJ5 = %+.3f deg"
                    % ecart_j5
                )

                print(
                    "Cette erreur n'empeche PAS la mesure."
                )

            # =================================================
            # IMPORTANT :
            # Le mouvement est termine.
            # On peut maintenant allumer le laser.
            # =================================================

            print()
            print(
                "Robot en position finale."
            )

            print(
                "Allumage du laser pour la mesure."
            )

            laser_ok = allumerPointeur(
                mc
            )

            if not laser_ok:

                print()
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
                        j4_atteint,
                        j5_atteint
                    ),
                    temps_deplacement,
                    mouvement_reussi,
                    precision_angulaire_ok,
                    False,
                    None
                )

                eteindrePointeur(
                    mc
                )

                continue

            print(
                "Laser allume."
            )

            # -------------------------------------------------
            # Verification geometrique
            # -------------------------------------------------

            prediction = predireImpact(
                modele,
                j4_atteint,
                j5_atteint
            )

            if prediction is not None:

                print()
                print(
                    "Prediction geometrique : "
                    "X = %+.3f   Y = %+.3f"
                    % (
                        prediction[0],
                        prediction[1]
                    )
                )

            # -------------------------------------------------
            # Mesure manuelle
            # -------------------------------------------------

            print()
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

                    reel = (
                        float(
                            morceaux[0]
                        ),
                        float(
                            morceaux[1]
                        )
                    )

                    break

                except ValueError:

                    print(
                        "  Saisie illisible."
                    )

            # -------------------------------------------------
            # Erreur spatiale par rapport a la cible originale
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
            # Donnees pour graphiques
            # -------------------------------------------------

            donnees_graphique.append({
                "x_vise": x,
                "y_vise": y,
                "x_mesure": reel[0],
                "y_mesure": reel[1],
            })

            # -------------------------------------------------
            # Enregistrement CSV
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
                    j4_atteint,
                    j5_atteint
                ),
                temps_deplacement,
                mouvement_reussi,
                precision_angulaire_ok,
                True,
                reel
            )

            # -------------------------------------------------
            # Laser OFF avant le prochain point
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
        # laser TOUJOURS eteint a la fin.
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

        print()
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
# 11. EXECUTION
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
