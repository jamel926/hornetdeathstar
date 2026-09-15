"""
=========================================================
CHASSE AU FRELON - VERSION SIMPLE (X/Y -> J4/J5, SANS CSV)
=========================================================

Objectif :
---------
L'operateur entre une position (x, y) en carreaux, le script
la convertit en angles J4/J5 via le modele geometrique deja
calibre, deplace la tete, allume le laser une duree fixe,
l'eteint, puis fait danser le bras.

Contrairement a calibration_test.py / chasse_frelon.py
(version precedente), ce script :
    - NE lit ni n'ecrit AUCUN fichier CSV
    - NE demande PAS de choisir une serie de calibration
    - N'enregistre RIEN sur disque

Le seul fichier lu est resultats/modele_grille.json, deja
produit une fois par calibration_grille.py (fonction
enregistrerModele) : c'est un JSON, pas un CSV, et il est
charge directement par chargerModele() sans aucune question
posee a l'operateur.

Si un fichier resultats/correction_pointage.json existe
(modele de correction appris via apprentissage_correction.py),
il est applique automatiquement. Sinon, les angles bruts du
modele geometrique sont utilises tels quels : ce n'est pas
bloquant.

ATTENTION SECURITE :
--------------------
Le laser pilote ici est celui de allumerPointeur/eteindrePointeur
dans calibration_grille.py (broche 33), le meme que dans tous
les autres scripts du depot -- PAS un laser de puissance cable
a part. DUREE_TIR et SEQUENCE_DANSE sont a ajuster en tete de
fichier selon vos besoins reels.

Utilisation :
------------
    python3 chasse_frelon.py
"""

import time

from calibration_grille import (
    TOLERANCE_BASE,
    TOLERANCE_POINTAGE,
    allerPositionInitiale,
    allumerPointeur,
    anglesPourPoint,
    chargerModele,
    connecterRobot,
    dansEnveloppe,
    eteindrePointeur,
    lireAngles,
    lireTexte,
    viserAngles,
)

"""
IMPORTANT : allerPositionInitiale est pris ici dans
calibration_grille.py, PAS dans PU_PositionInitiale.py.

La version de calibration_grille.py ne depend pas de
mc.is_moving() pour savoir qu'un mouvement a demarre : elle
relit directement les angles mesures et compare a la cible,
avec un timeout reel (TIMEOUT_INIT). Si mc.is_moving() ne
remonte jamais a 1 sur ce bras (ce qui provoquait le blocage
silencieux observe avec PU_PositionInitiale.allerPositionInitiale),
cette version ne peut pas rester bloquee indefiniment : au pire
elle affiche "ATTENTION : la pose initiale n'a pas ete atteinte"
apres 25 s et continue.

Elle a quand meme besoin de ANGLES_POSITION_INITIALE, importe
par calibration_grille.py depuis PU_PositionInitiale.py au
demarrage (dans un try/except) : si ce fichier est absent ou en
erreur, le bras part simplement de sa pose courante, avec un
message clair -- jamais de blocage silencieux.
"""

# La correction apprise est optionnelle : si le module ou son
# fichier JSON est absent, on continue avec les angles bruts.
try:
    from apprentissage_correction import chargerCorrection, cibleCorrigee
except Exception:
    chargerCorrection = None
    cibleCorrigee = None


# =========================================================
# PARAMETRES A AJUSTER
# =========================================================

# Duree pendant laquelle le laser reste allume sur la cible.
# A AJUSTER selon le temps reel necessaire.
DUREE_TIR = 3.0   # secondes

# Sequence de danse : deplacements (dJ4, dJ5) en degres,
# appliques en absolu autour de la position de tir. Amplitude
# volontairement petite pour rester loin des butees ; le
# dernier point revient a (0, 0).
SEQUENCE_DANSE = [
    (10, 0),
    (-10, 10),
    (0, -10),
    (-10, 0),
    (10, -10),
    (0, 10),
    (0, 0),
]

PAUSE_ENTRE_PAS_DANSE = 0.15   # secondes


# =========================================================
# VERIFICATION DE LA BASE (sans lire/ecrire aucun fichier)
# =========================================================

def verifierBase(mc, modele, tolerance=TOLERANCE_BASE):
    """Compare la base courante a celle enregistree dans le modele charge."""

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
# DANSE DE VICTOIRE
# =========================================================

def danserVictoire(mc, base, j4_tir, j5_tir, j6):
    """Fait osciller la tete autour de la position de tir, puis revient dessus."""

    print()
    print("Le frelon est mort. Petite danse de victoire...")

    for dj4, dj5 in SEQUENCE_DANSE:

        cible_j4 = j4_tir + dj4
        cible_j5 = j5_tir + dj5

        # viserAngles() refuse tout seul un mouvement hors des
        # bornes constructeur (J4_MIN/MAX, J5_MIN/MAX).
        arrive, mesure = viserAngles(mc, base, cible_j4, cible_j5, j6)

        if mesure is None:
            print("  Mouvement de danse refuse, arret de la sequence.")
            return

        time.sleep(PAUSE_ENTRE_PAS_DANSE)

    print("Danse terminee.")


# =========================================================
# SAISIE DE LA POSITION DU FRELON
# =========================================================

def lireCibleFrelon():
    """Lit la position du frelon en carreaux et la retourne, ou None pour terminer."""

    print()
    print("Position du frelon : deux nombres en CARREAUX (ex : 4 -2).")
    print("Taper fin pour terminer.")

    while True:

        reponse = lireTexte("frelon > ")

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


# =========================================================
# PROGRAMME PRINCIPAL
# =========================================================

def main():
    print("=========================================")
    print("CHASSE AU FRELON")
    print("=========================================")
    print()
    print("SECURITE : verifier que c'est bien le pointeur rouge classe 2")
    print("qui est branche sur la broche du laser avant de continuer.")

    # -----------------------------------------------------
    # Modele geometrique : un seul fichier JSON, deja calibre.
    # Pas de choix de serie, pas de lecture de CSV.
    # -----------------------------------------------------

    modele = chargerModele()

    if modele is None:
        # chargerModele() a deja affiche le chemin attendu.
        return 1

    mc = connecterRobot()

    if mc is None:
        return 1

    allerPositionInitiale(mc)

    if not verifierBase(mc, modele):
        return 1

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    base = angles[0:3]
    j6 = angles[5]

    # -----------------------------------------------------
    # Correction apprise, si disponible (optionnelle).
    # -----------------------------------------------------

    correction = chargerCorrection() if chargerCorrection is not None else None

    if correction is None:
        print()
        print("Pas de correction apprise : angles bruts du modele geometrique.")
    else:
        print()
        print(
            "Correction apprise '%s' appliquee (%s, %d points)."
            % (
                correction.get("type", "?"),
                correction.get("serie", "serie inconnue"),
                correction.get("nb_points", 0),
            )
        )

    try:
        while True:

            demande = lireCibleFrelon()

            if demande is None:
                break

            u, v = demande

            # -------------------------------------------------
            # Verification de l'enveloppe sur la cible ORIGINALE.
            # -------------------------------------------------

            if not dansEnveloppe(modele, u, v):
                e = modele["enveloppe"]
                print("  Point hors de la zone calibree.")
                print(
                    "  Zone autorisee : u %.1f..%.1f, v %.1f..%.1f carreaux."
                    % (e["u_min"], e["u_max"], e["v_min"], e["v_max"])
                )
                continue

            if correction is not None and cibleCorrigee is not None:
                vise_u, vise_v = cibleCorrigee(correction, u, v)
            else:
                vise_u, vise_v = u, v

            if (vise_u, vise_v) != (u, v):
                print("  Cible corrigee : (%.2f, %.2f) carreaux" % (vise_u, vise_v))

            resultat = anglesPourPoint(modele, vise_u, vise_v)

            if resultat is None:
                print("  Impossible de calculer J4/J5 pour ce point avec ce modele.")
                continue

            j4, j5 = resultat

            print("  Angles calcules : J4 = %+.2f   J5 = %+.2f" % (j4, j5))

            vise, mesure = viserAngles(mc, base, j4, j5, j6)

            if mesure is None:
                print("  Mouvement echoue.")
                continue

            if not vise:
                print(
                    "  ATTENTION : la tete se repose a %+.2f / %+.2f deg de l'angle demande"
                    % (mesure[3] - j4, mesure[4] - j5)
                )
                print("  (tolerance %.2f deg)." % TOLERANCE_POINTAGE)

            # -------------------------------------------------
            # Tir : allumage, duree fixe, extinction
            # -------------------------------------------------

            if not allumerPointeur(mc):
                print("  Le pointeur ne s'allume pas, tir annule.")
                continue

            print("  Laser allume, tir en cours (%.1f s)..." % DUREE_TIR)
            time.sleep(DUREE_TIR)

            eteindrePointeur(mc)
            print("  Laser eteint : frelon considere mort.")

            # -------------------------------------------------
            # Danse de victoire, puis retour sur la cible
            # -------------------------------------------------

            danserVictoire(mc, base, mesure[3], mesure[4], j6)

            viserAngles(mc, base, j4, j5, j6)

    finally:

        eteindrePointeur(mc)
        print()
        print("Pointeur eteint. Moteurs laisses sous tension.")

    return 0


# =========================================================
# EXECUTION AUTONOME
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
