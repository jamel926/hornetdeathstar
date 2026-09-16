"""
=========================================================
TEST DE LA CALIBRATION - POINTER UN POINT DEMANDE
=========================================================

Objectif :
---------
Verifier la calibration produite par calibration_grille.py :
on donne des coordonnees X et Y sur le quadrillage,
le robot pointe dessus.

Le script sert aussi d'outil de MESURE : apres chaque tir,
on peut saisir l'endroit ou le spot est reellement tombe.
L'ecart est enregistre, et c'est ce chiffre qui dit ce
que vaut la calibration.

Pourquoi la base est verifiee au demarrage :
-------------------------------------------
Le modele decrit la position du spot pour une pose de base
donnee. Si J1, J2 ou J3 ont bouge depuis la calibration,
tout le modele est faux. Le script refuse alors de
continuer plutot que de viser a cote sans le dire.

Pourquoi le tir corrige sa propre consigne :
--------------------------------------------
Le bras ne s'arrete pas sur la consigne, mais un peu en
dessous. Le tir passe donc par viserAngles(), qui relit
la pose atteinte et rallonge la consigne tant que l'angle
demande n'est pas tenu. L'ecart residuel affiche apres
chaque tir ne vient plus de la mecanique : c'est celui
du modele.

Graphiques en fin de session :
------------------------------
Lorsque l'utilisateur tape "fin", deux courbes 3D sont
affichees :

    1) X visee, Y visee, erreur X
       erreur X = X mesuree - X visee

    2) X visee, Y visee, erreur Y
       erreur Y = Y mesuree - Y visee

Toutes les coordonnees sont exprimees en CARREAUX.

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

import matplotlib.pyplot as plt

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

FICHIER_VALIDATION = os.path.join(
    DOSSIER_RESULTATS,
    "validation_pointage.csv"
)

FICHIER_MESURES = os.path.join(
    DOSSIER_RESULTATS,
    "calibration_grille.csv"
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


# =========================================================
# 3. VERIFICATIONS
# =========================================================

def verifierBase(mc, modele, tolerance=TOLERANCE_BASE):
    """
    Compare la base courante a celle de la calibration
    et retourne True si elle n'a pas bouge.
    """

    reference = modele.get("base")

    if not reference:
        print("ATTENTION : le modele ne contient pas la pose de base.")
        print("Impossible de verifier que le bras n'a pas bouge depuis la calibration.")
        return True

    mesure = lireAngles(mc)

    if mesure is None:
        print("Lecture des angles impossible.")
        return False

    ecarts = [
        abs(mesure[i] - reference[i])
        for i in range(3)
    ]

    print()
    print(
        "Base de calibration :",
        [round(a, 2) for a in reference]
    )

    print(
        "Base courante       :",
        [round(mesure[i], 2) for i in range(3)]
    )

    print(
        "Ecarts              :",
        [round(e, 2) for e in ecarts]
    )

    if max(ecarts) > tolerance:
        print()
        print(
            "REFUS : la base a bouge de %.2f deg depuis la calibration."
            % max(ecarts)
        )
        print("Le modele ne decrit plus la geometrie actuelle.")
        print(
            "Remettre le bras dans sa pose de calibration, "
            "ou refaire une campagne."
        )
        return False

    print("Base conforme.")
    return True


# =========================================================
# 4. ENREGISTREMENT DES TIRS
# =========================================================

def enregistrerTir(demande, angles, reel, chemin=None):
    """
    Ajoute une ligne au fichier de validation
    et retourne True si l'ecriture a reussi.

    Les coordonnees utilisees dans ce programme sont X et Y,
    en carreaux.
    """

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
            3
        )

        x_reel = round(reel[0], 3)
        y_reel = round(reel[1], 3)

    try:
        os.makedirs(DOSSIER_RESULTATS, exist_ok=True)

        # Utiliser le fichier fourni par l'appelant si disponible.
        fichier_cible = (
            chemin
            if chemin is not None
            else FICHIER_VALIDATION
        )

        nouveau = not os.path.exists(fichier_cible)

        with open(fichier_cible, "a", newline="") as fichier:

            redacteur = csv.writer(fichier)

            if nouveau:
                redacteur.writerow(
                    ENTETE_VALIDATION
                )

            redacteur.writerow([
                datetime.now().isoformat(timespec="seconds"),
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
            e
        )
        return False


# =========================================================
# 5. SELECTION DE LA CALIBRATION
# =========================================================

def nomFichierSecurise(serie):
    """
    Transforme le nom d'une serie en nom de fichier
    sans caracteres dangereux.
    """

    propre = re.sub(
        r"[^A-Za-z0-9._-]+",
        "_",
        serie
    ).strip("._")

    return propre or "calibration"


def listerSeriesCalibrations():
    """
    Retourne les series presentes dans le CSV,
    avec quelques statistiques utiles.

    Le fichier calibration_grille.csv conserve ses champs
    internes u/v. Dans ce programme, ils sont traduits en
    x/y pour l'affichage et l'utilisation utilisateur.
    """

    series = {}

    try:
        with open(
            FICHIER_MESURES,
            "r",
            newline=""
        ) as fichier:

            for ligne in csv.DictReader(fichier):

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
                    "validation"
                ):
                    info[usage] += 1

                try:

                    # Les donnees du fichier de calibration
                    # utilisent encore u/v.
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
                        else min(info["x_min"], x)
                    )

                    info["x_max"] = (
                        x
                        if info["x_max"] is None
                        else max(info["x_max"], x)
                    )

                    info["y_min"] = (
                        y
                        if info["y_min"] is None
                        else min(info["y_min"], y)
                    )

                    info["y_max"] = (
                        y
                        if info["y_max"] is None
                        else max(info["y_max"], y)
                    )

                except (
                    ValueError,
                    TypeError
                ):
                    continue

    except FileNotFoundError:

        print(
            "Fichier de mesures introuvable :",
            FICHIER_MESURES
        )

        return {}

    except Exception as e:

        print(
            "Impossible de lire les calibrations :",
            e
        )

        return {}

    return series


def chargerBaseSerie(serie):
    """
    Lit J1/J2/J3 dans la campagne choisie
    pour verifier la base du robot.
    """

    try:
        with open(
            FICHIER_MESURES,
            "r",
            newline=""
        ) as fichier:

            for ligne in csv.DictReader(fichier):

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
                    KeyError
                ):
                    continue

    except Exception as e:

        print(
            "Impossible de lire la base de la calibration :",
            e
        )

    return None


def construireModeleSerie(serie):
    """
    Reconstruit en memoire le modele de la serie choisie
    a partir du CSV.
    """

    ajustement = chargerMesures(
        usage="ajustement",
        serie=serie
    )

    validation = chargerMesures(
        usage="validation",
        serie=serie
    )

    if len(ajustement) < 5:

        print(
            "ERREUR : la calibration '%s' ne contient "
            "que %d point(s) d'ajustement."
            % (
                serie,
                len(ajustement)
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
        base=base_reference
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
                validation
            )
        )

    return modele


def choisirCalibration():
    """
    Affiche les calibrations disponibles
    et retourne la serie choisie.
    """

    series = listerSeriesCalibrations()

    if not series:

        print(
            "Aucune calibration trouvee dans :",
            FICHIER_MESURES
        )

        return None

    noms = list(
        series.keys()
    )

    print()
    print("=========================================")
    print("CHOIX DE LA CALIBRATION")
    print("=========================================")
    print(
        "Les calibrations sont selectionnees "
        "par leur nom de serie."
    )
    print()

    for indice, serie in enumerate(
        noms,
        1
    ):

        info = series[serie]

        if info["x_min"] is not None:

            zone = (
                "x %.1f..%.1f / y %.1f..%.1f carreaux"
                % (
                    info["x_min"],
                    info["x_max"],
                    info["y_min"],
                    info["y_max"]
                )
            )

        else:
            zone = "zone inconnue"

        print(
            "  %d) %s"
            % (
                indice,
                serie
            )
        )

        print(
            "     ajustement : %d | validation : %d | %s"
            % (
                info["ajustement"],
                info["validation"],
                zone
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
            "q"
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
# 6. DIALOGUE
# =========================================================

def lireCible():
    """
    Lit le point a viser en CARREAUX
    et le retourne, ou None pour terminer.

    Format :
        x y

    Exemple :
        4 -2
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
            .replace(",", " ")
            .split()
        )

        if len(morceaux) != 2:

            print(
                "  Il faut deux nombres "
                "separes par un espace."
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


def lireImpactReel(demande):
    """
    Demande ou le spot est reellement tombe,
    en CARREAUX.

    Retourne :
        (x_reel, y_reel)

    ou None.
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
        .replace(",", " ")
        .split()
    )

    if len(morceaux) != 2:

        print(
            "  Saisie ignoree."
        )

        return None

    try:

        x_reel = float(
            morceaux[0]
        )

        y_reel = float(
            morceaux[1]
        )

    except ValueError:

        print(
            "  Saisie ignoree."
        )

        return None

    ecart = (
        (
            (x_reel - demande[0]) ** 2
            + (y_reel - demande[1]) ** 2
        ) ** 0.5
    )

    print(
        "  Ecart de pointage : %.2f carreaux (%.1f cm)"
        % (
            ecart,
            ecart * COTE_CARREAU
        )
    )

    return (
        x_reel,
        y_reel
    )


# =========================================================
# 7. GRAPHIQUES 3D
# =========================================================

def tracerErreurs3D(donnees):
    """
    Trace deux courbes 3D dans deux sous-graphiques.

    SUBPLOT 1 :
        abscisse = X visee
        profondeur = Y visee
        hauteur = erreur X

        erreur X = X mesuree - X visee

    SUBPLOT 2 :
        abscisse = X visee
        profondeur = Y visee
        hauteur = erreur Y

        erreur Y = Y mesuree - Y visee

    Toutes les coordonnees sont en CARREAUX.
    """

    if not donnees:

        print()
        print(
            "Aucune mesure disponible pour le graphique."
        )

        return

    # -----------------------------------------------------
    # Extraction des donnees
    # -----------------------------------------------------

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

    # -----------------------------------------------------
    # Creation de la figure
    # -----------------------------------------------------

    fig = plt.figure(
        figsize=(14, 7)
    )

    # =====================================================
    # SUBPLOT 1 : ERREUR X
    # =====================================================

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

    # =====================================================
    # SUBPLOT 2 : ERREUR Y
    # =====================================================

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
# 8. PROGRAMME
# =========================================================

def main():
    """
    Demande la calibration, verifie la base,
    puis pointe les points demandes.
    """

    print(
        "========================================="
    )

    print(
        "TEST DE LA CALIBRATION"
    )

    print(
        "========================================="
    )

    print()

    print(
        "SECURITE : pointeur rouge classe 2, "
        "laser de puissance DEBRANCHE."
    )

    # -----------------------------------------------------
    # Choix de la calibration AVANT tout mouvement
    # du robot.
    # -----------------------------------------------------

    serie = choisirCalibration()

    if serie is None:

        print(
            "Aucune calibration selectionnee."
        )

        return 0

    print()

    print(
        "Calibration selectionnee :",
        serie
    )

    print(
        "Reconstruction du modele "
        "depuis les points de cette serie..."
    )

    modele = construireModeleSerie(
        serie
    )

    if modele is None:
        return 1

    print()

    print(
        "Calibration '%s' chargee : "
        "%d points d'ajustement"
        % (
            serie,
            modele.get(
                "nb_points",
                0
            )
        )
    )

    # Le modele interne conserve encore u/v.
    # On les affiche ici sous la forme X/Y pour l'utilisateur.
    print(
        "Zone calibree : x %.1f..%.1f, "
        "y %.1f..%.1f carreaux"
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
            "Erreur ajustement : rms %.2f carreaux, "
            "max %.2f carreaux"
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
            "Erreur validation  : rms %.2f carreaux, "
            "max %.2f carreaux"
            % (
                residus_validation["rms_carreaux"],
                residus_validation["max_carreaux"]
            )
        )

    else:

        print(
            "Pas de validation pour cette calibration."
        )

    # -----------------------------------------------------
    # Connexion au robot seulement apres le choix
    # de calibration.
    # -----------------------------------------------------

    mc = connecterRobot()

    allerPositionInitiale(
        mc
    )

    if mc is None:

        return 1

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
        % nomFichierSecurise(
            serie
        )
    )

    print()

    print(
        "Les tirs de cette session seront enregistres dans :"
    )

    print(
        " ",
        fichier_validation
    )

    # -----------------------------------------------------
    # Donnees conservées pour les courbes 3D
    # -----------------------------------------------------

    donnees_graphique = []

    try:

        if not allumerPointeur(
            mc
        ):

            print(
                "Le pointeur ne s'allume pas."
            )

        while True:

            # -------------------------------------------------
            # Saisie de la cible
            # -------------------------------------------------

            demande = lireCible()

            if demande is None:

                print()
                print(
                    "Fin de la campagne de mesure."
                )

                # Affichage des courbes 3D
                tracerErreurs3D(
                    donnees_graphique
                )

                break

            x, y = demande

            # -------------------------------------------------
            # Verification explicite de l'enveloppe
            # choisie.
            # -------------------------------------------------

            if not dansEnveloppe(
                modele,
                x,
                y
            ):

                e = modele["enveloppe"]

                print(
                    "  Point hors de la zone "
                    "de la calibration '%s'."
                    % serie
                )

                print(
                    "  Zone autorisee : "
                    "x %.1f..%.1f, y %.1f..%.1f carreaux."
                    % (
                        e["u_min"],
                        e["u_max"],
                        e["v_min"],
                        e["v_max"]
                    )
                )

                print(
                    "  Si ce point appartient a "
                    "une autre zone, relancer le programme"
                )

                print(
                    "  et choisir la calibration correspondante."
                )

                continue

            # -------------------------------------------------
            # Calcul des angles
            # -------------------------------------------------

            resultat = anglesPourPoint(
                modele,
                x,
                y
            )

            if resultat is None:

                print(
                    "  Impossible de calculer J4/J5 "
                    "pour ce point avec cette calibration."
                )

                print(
                    "  Le point est peut-etre hors "
                    "du domaine angulaire inverse."
                )

                continue

            j4, j5 = resultat

            print(
                "  Angles calcules : "
                "J4 = %+.2f   J5 = %+.2f"
                % (
                    j4,
                    j5
                )
            )

            # -------------------------------------------------
            # Deplacement avec rattrapage mecanique
            # -------------------------------------------------

            vise, mesure = viserAngles(
                mc,
                base,
                j4,
                j5,
                j6
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
                        mesure[4] - j5
                    )
                )

                print(
                    "  (tolerance %.2f deg). "
                    "Le rattrapage n'a pas converge."
                    % TOLERANCE_POINTAGE
                )

            # -------------------------------------------------
            # Ecart de reproduction
            # -------------------------------------------------

            prediction = predireImpact(
                modele,
                mesure[3],
                mesure[4]
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
                        "  Angles atteints : "
                        "J4 = %+.2f  J5 = %+.2f"
                        "  -> %.2f carreaux de derive mecanique"
                        % (
                            mesure[3],
                            mesure[4],
                            derive
                        )
                    )

            # -------------------------------------------------
            # Saisie du point reel
            # -------------------------------------------------

            reel = lireImpactReel(
                demande
            )

            # -------------------------------------------------
            # Sauvegarde pour les graphiques
            # -------------------------------------------------

            if reel is not None:

                donnees_graphique.append({
                    "x_vise": x,
                    "y_vise": y,
                    "x_mesure": reel[0],
                    "y_mesure": reel[1],
                })

            # -------------------------------------------------
            # Enregistrement du tir
            # -------------------------------------------------

            enregistrerTir(
                demande,
                (
                    mesure[3],
                    mesure[4]
                ),
                reel,
                chemin=fichier_validation
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
            serie
        )

        print(
            "Tirs enregistres dans :",
            fichier_validation
        )

    return 0


# =========================================================
# 9. EXECUTION AUTONOME
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
