"""
=========================================================
TESTS UNITAIRES - CALIBRATION DE LA GRILLE
=========================================================

Tourne sans robot : pymycobot est remplace par un faux
module, et le bras par une classe ecrite a la main.

Le test central est celui du modele. On fabrique des
points avec une geometrie de tourelle CONNUE, on ajuste
dessus, et on verifie que le modele la retrouve
exactement. Si l'algebre est juste, le residu doit etre
numeriquement nul : ce n'est pas une tolerance choisie au
jugé, c'est zero.

Lancement, depuis Bras/ModuleDeDeplacement/ :

    python3 -m unittest test_unitaires.tests_unitaire_calibration -v
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import unittest
from unittest.mock import MagicMock
import sys
import os
import math
import json
import shutil
import tempfile

## Remplacement de pymycobot par un faux module (absent hors Jetson)
faux_pymycobot = MagicMock()
sys.modules["pymycobot"] = faux_pymycobot
sys.modules["pymycobot.mycobot280"] = faux_pymycobot.mycobot280

## importation des modules a tester
dossier_racine = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_racine)
import calibration_grille


# =========================================================
# 2. SIMULATEUR DE GEOMETRIE
# =========================================================

def rotation(alpha, beta, gamma):
    """Retourne la matrice Rz(gamma) Ry(beta) Rx(alpha), angles en degres."""

    a, b, g = math.radians(alpha), math.radians(beta), math.radians(gamma)

    rz = [[math.cos(g), -math.sin(g), 0.0], [math.sin(g), math.cos(g), 0.0], [0.0, 0.0, 1.0]]
    ry = [[math.cos(b), 0.0, math.sin(b)], [0.0, 1.0, 0.0], [-math.sin(b), 0.0, math.cos(b)]]
    rx = [[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]]

    def produit(m1, m2):
        return [[sum(m1[i][k] * m2[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    return produit(produit(rz, ry), rx)


class TourelleSimulee:
    """Tourelle ideale : point d'emission fixe, deux rotations, impact sur un plan horizontal."""

    def __init__(self, emission, orientation, offset_j5):
        self.emission = emission
        self.orientation = orientation
        self.offset_j5 = offset_j5

    def impact(self, j4, j5):
        """Retourne le point d'impact sur le plan z = 0, ou None si le faisceau ne l'atteint pas."""

        p = calibration_grille.directionFaisceau(j4, j5 + self.offset_j5)

        d = [sum(self.orientation[i][k] * p[k] for k in range(3)) for i in range(3)]

        if d[2] >= -1e-9:
            return None

        t = -self.emission[2] / d[2]

        return (self.emission[0] + t * d[0], self.emission[1] + t * d[1])


def tourelleDeReference():
    """Retourne une tourelle plausible : emission 30 cm au-dessus du plan, base legerement inclinee."""

    return TourelleSimulee(
        emission=[1.5, -2.0, 30.0],
        orientation=rotation(2.0, 95.0, 4.0),
        offset_j5=7.5,
    )


def mesuresSynthetiques(tourelle, pas_j4=20.0, pas_j5=20.0):
    """Fabrique une grille 5 x 4 de points a partir d'une tourelle simulee."""

    mesures = []

    for i in range(5):
        for k in range(4):

            j4 = -40.0 + i * pas_j4
            j5 = -30.0 + k * pas_j5

            point = tourelle.impact(j4, j5)

            if point is not None:
                mesures.append({"j4": j4, "j5": j5, "u": point[0], "v": point[1]})

    return mesures


# =========================================================
# 3. FAUX BRAS
# =========================================================

class FauxBras:
    """Bras dont J3 est bloque a -124 degres : il ignore toute consigne sur cet axe."""

    J3_BLOQUE = -124.0

    def __init__(self):
        self.pose = [0.0, 50.0, self.J3_BLOQUE, 40.0, 0.0, 0.0]
        self.consignes = []
        self.sorties = []
        self.relaches = set()
        self.focalises = []
        self.couleurs = []
        self.release_all_appele = False

    def release_servo(self, servo_id, mode=None):
        self.relaches.add(servo_id)
        return 1

    def focus_servo(self, servo_id):
        self.relaches.discard(servo_id)
        self.focalises.append(servo_id)
        return 1

    def release_all_servos(self):
        # Piege : ce script ne doit JAMAIS couper les six servos.
        self.release_all_appele = True
        self.relaches.update([1, 2, 3, 4, 5, 6])
        return 1

    def set_color(self, r, g, b):
        self.couleurs.append((r, g, b))
        return 1

    def send_angles(self, angles, vitesse):
        self.consignes.append(list(angles))
        self.pose = list(angles)
        self.pose[2] = self.J3_BLOQUE
        return 1

    def get_angles(self):
        return list(self.pose)

    def set_pin_mode(self, broche, mode):
        return 1

    def set_digital_output(self, broche, niveau):
        self.sorties.append((broche, niveau))
        return 1


# =========================================================
# 4. LE MODELE
# =========================================================

class TestModeleGeometrique(unittest.TestCase):
    """Le modele doit decrire exactement une tourelle ideale."""

    def setUp(self):
        self.tourelle = tourelleDeReference()
        self.mesures = mesuresSynthetiques(self.tourelle)

    def test_la_grille_synthetique_est_complete(self):
        """Les vingt points de la grille doivent tous atteindre le plan."""
        self.assertEqual(len(self.mesures), 20)

    def test_residu_numeriquement_nul(self):
        """Sur une tourelle ideale, l'ajustement doit tomber juste a la precision machine."""

        modele = calibration_grille.ajusterModele(self.mesures)

        self.assertIsNotNone(modele)
        self.assertLess(modele["residus"]["max_cm"], 1e-6)

    def test_decalage_j5_retrouve(self):
        """Le decalage de J5 est identifiable : il doit etre retrouve au centieme de degre."""

        modele = calibration_grille.ajusterModele(self.mesures)

        self.assertAlmostEqual(modele["offset_j5"], self.tourelle.offset_j5, places=2)

    def test_inversion_rend_les_angles_de_depart(self):
        """anglesPourPoint doit retrouver les angles qui ont produit chaque point."""

        modele = calibration_grille.ajusterModele(self.mesures)

        for m in self.mesures:

            resultat = calibration_grille.anglesPourPoint(modele, m["u"], m["v"])

            self.assertIsNotNone(resultat)
            self.assertAlmostEqual(resultat[0], m["j4"], places=4)
            self.assertAlmostEqual(resultat[1], m["j5"], places=4)

    def test_precision_sur_des_points_non_ajustes(self):
        """Le modele doit rester juste sur des points qui n'ont pas servi a l'ajustement."""

        modele = calibration_grille.ajusterModele(self.mesures)

        controle = []
        for j4, j5 in ((-33.0, -21.0), (7.0, 13.0), (28.0, -7.0), (-11.0, 25.0)):
            point = self.tourelle.impact(j4, j5)
            controle.append({"j4": j4, "j5": j5, "u": point[0], "v": point[1]})

        residus = calibration_grille.calculerResidus(modele, controle)

        self.assertLess(residus["max_cm"], 1e-6)


class TestComparaisonBilineaire(unittest.TestCase):
    """Le temoin bilineaire justifie le choix du modele geometrique."""

    def test_le_bilineaire_decroche_sur_la_zone_large(self):
        """Sur la meme zone large, le modele bilineaire doit etre nettement moins bon."""

        """
        C'est la raison d'etre du changement de modele. Le
        test la verifie au lieu de la supposer : si un jour
        cet ecart disparait, le modele simple suffirait.
        """

        mesures = mesuresSynthetiques(tourelleDeReference())

        modele = calibration_grille.ajusterModele(mesures)
        temoin = calibration_grille.residusBilineaire(
            calibration_grille.ajusterBilineaire(mesures), mesures)

        self.assertGreater(temoin["max_cm"], 1.0)
        self.assertGreater(temoin["max_cm"], 100.0 * modele["residus"]["max_cm"])


class TestRefus(unittest.TestCase):
    """Le modele doit refuser plutot que de repondre n'importe quoi."""

    def test_trop_peu_de_points(self):
        """Moins de cinq points ne permet pas d'ajuster : le modele doit valoir None."""

        mesures = mesuresSynthetiques(tourelleDeReference())[0:3]

        self.assertIsNone(calibration_grille.ajusterModele(mesures))

    def test_points_alignes(self):
        """Une grille degeneree en ligne doit etre refusee, malgre un residu nul."""

        """
        Le piege : ce cas n'est PAS singulier. Le modele
        s'ajuste parfaitement le long de la ligne, avec un
        residu de l'ordre de 1e-13, et se trompe partout
        ailleurs. C'est le controle d'etendue qui doit le
        rattraper, pas le solveur.
        """

        tourelle = tourelleDeReference()

        mesures = []
        for i in range(8):
            point = tourelle.impact(-40.0 + i * 10.0, 0.0)
            mesures.append({"j4": -40.0 + i * 10.0, "j5": 0.0, "u": point[0], "v": point[1]})

        self.assertIsNone(calibration_grille.ajusterModele(mesures))

    def test_hors_enveloppe(self):
        """Un point hors de la zone relevee doit etre refuse, pas extrapole."""

        modele = calibration_grille.ajusterModele(mesuresSynthetiques(tourelleDeReference()))

        e = modele["enveloppe"]

        self.assertIsNone(calibration_grille.anglesPourPoint(modele, e["u_max"] + 50.0, 0.0))
        self.assertIsNone(calibration_grille.anglesPourPoint(modele, 0.0, e["v_min"] - 50.0))

    def test_dans_enveloppe(self):
        """Le centre de la zone relevee doit rester accessible."""

        modele = calibration_grille.ajusterModele(mesuresSynthetiques(tourelleDeReference()))

        e = modele["enveloppe"]
        centre_u = (e["u_min"] + e["u_max"]) / 2.0
        centre_v = (e["v_min"] + e["v_max"]) / 2.0

        self.assertIsNotNone(calibration_grille.anglesPourPoint(modele, centre_u, centre_v))


class TestResidus(unittest.TestCase):
    """Le biais et la dispersion doivent etre separes."""

    def test_un_biais_pur_ne_compte_pas_comme_dispersion(self):
        """Un decalage constant doit ressortir en biais, avec une dispersion nulle."""

        tourelle = tourelleDeReference()
        mesures = mesuresSynthetiques(tourelle)

        modele = calibration_grille.ajusterModele(mesures)

        decalees = [{"j4": m["j4"], "j5": m["j5"], "u": m["u"] - 2.0, "v": m["v"] + 1.0}
                    for m in mesures]

        residus = calibration_grille.calculerResidus(modele, decalees)

        self.assertAlmostEqual(residus["biais_u_cm"], 2.0, places=4)
        self.assertAlmostEqual(residus["biais_v_cm"], -1.0, places=4)
        self.assertLess(residus["dispersion_cm"], 1e-6)


# =========================================================
# 5. LE ROBOT
# =========================================================

class TestMouvementTete(unittest.TestCase):
    """La couche robot ne doit ni commander la base, ni attendre un axe mort."""

    def setUp(self):
        self.bras = FauxBras()
        self.base = [0.0, 50.0, FauxBras.J3_BLOQUE]

        self.sleep_origine = calibration_grille.time.sleep
        calibration_grille.time.sleep = lambda duree: None

    def tearDown(self):
        calibration_grille.time.sleep = self.sleep_origine

    def test_arrivee_malgre_j3_bloque(self):
        """L'attente ne porte que sur J4 et J5 : elle doit aboutir bien que J3 soit mort."""

        arrive, mesure = calibration_grille.bougerTete(self.bras, self.base, 30.0, -10.0, 0.0)

        self.assertTrue(arrive)
        self.assertEqual(mesure[2], FauxBras.J3_BLOQUE)

    def test_la_base_est_renvoyee_telle_quelle(self):
        """La pose envoyee doit reprendre les angles de base relus, jamais une consigne."""

        calibration_grille.bougerTete(self.bras, self.base, 30.0, -10.0, 0.0)

        self.assertEqual(self.bras.consignes[-1][0:3], self.base)

    def test_consigne_hors_limites_refusee(self):
        """Une consigne hors limites constructeur doit etre refusee, pas rabotee."""

        arrive, mesure = calibration_grille.bougerTete(self.bras, self.base, 400.0, 0.0, 0.0)

        self.assertFalse(arrive)
        self.assertIsNone(mesure)
        self.assertEqual(self.bras.consignes, [])

    def test_approche_toujours_du_meme_cote(self):
        """L'approche doit passer par un recul avant d'atteindre la consigne."""

        calibration_grille.approcher(self.bras, self.base, 30.0, -10.0, 0.0)

        self.assertGreaterEqual(len(self.bras.consignes), 2)

        intermediaire = self.bras.consignes[-2]
        finale = self.bras.consignes[-1]

        self.assertLess(intermediaire[3], finale[3])
        self.assertLess(intermediaire[4], finale[4])
        self.assertEqual(finale[3], 30.0)
        self.assertEqual(finale[4], -10.0)

    def test_recul_borne_par_les_limites(self):
        """Pres d'une butee, le recul est reduit au lieu de sortir des limites."""

        calibration_grille.approcher(self.bras, self.base,
                                     calibration_grille.J4_MIN + 1.0, 0.0, 0.0)

        for consigne in self.bras.consignes:
            self.assertGreaterEqual(consigne[3], calibration_grille.J4_MIN)


class TestPointeur(unittest.TestCase):
    """Le pointeur doit s'allumer et s'eteindre sur la broche retenue."""

    def setUp(self):
        self.bras = FauxBras()
        self.sleep_origine = calibration_grille.time.sleep
        calibration_grille.time.sleep = lambda duree: None

    def tearDown(self):
        calibration_grille.time.sleep = self.sleep_origine

    def test_allumage_puis_extinction(self):
        """L'allumage et l'extinction doivent ecrire les deux niveaux sur la bonne broche."""

        calibration_grille.allumerPointeur(self.bras)
        calibration_grille.eteindrePointeur(self.bras)

        self.assertEqual(self.bras.sorties[0],
                         (calibration_grille.BROCHE_LASER, calibration_grille.NIVEAU_ALLUME))
        self.assertEqual(self.bras.sorties[-1],
                         (calibration_grille.BROCHE_LASER, calibration_grille.NIVEAU_ETEINT))


class FauxDialogue:
    """Remplace les saisies clavier par une liste de reponses preparees."""

    def __init__(self, reponses):
        self.reponses = list(reponses)
        self.questions = []

    def __call__(self, message, defaut=""):
        self.questions.append(message)
        if self.reponses:
            return self.reponses.pop(0)
        return defaut


class TestRelachementDeLaTete(unittest.TestCase):
    """Relacher la tete ne doit jamais couper le couple de l'epaule ni du coude."""

    def setUp(self):
        self.bras = FauxBras()
        self.sleep_origine = calibration_grille.time.sleep
        calibration_grille.time.sleep = lambda duree: None

    def tearDown(self):
        calibration_grille.time.sleep = self.sleep_origine

    def test_seuls_j4_et_j5_sont_relaches(self):
        """C'est la ligne dangereuse du script : relacher les six ferait tomber le bras."""

        calibration_grille.relacherTete(self.bras)

        self.assertEqual(self.bras.relaches, {4, 5})
        self.assertFalse(self.bras.release_all_appele)

    def test_verrouillage_redonne_le_couple(self):
        """Le verrouillage doit refocaliser les deux servos et figer la pose lue."""

        calibration_grille.relacherTete(self.bras)

        self.bras.pose = [0.0, 50.0, FauxBras.J3_BLOQUE, 55.0, -12.0, 0.0]

        pose = calibration_grille.verrouillerTete(self.bras)

        self.assertEqual(self.bras.relaches, set())
        self.assertEqual(sorted(self.bras.focalises), [4, 5])
        self.assertEqual(pose[3], 55.0)
        self.assertEqual(self.bras.consignes[-1][3], 55.0)
        self.assertEqual(self.bras.consignes[-1][4], -12.0)

    def test_la_tete_est_verrouillee_meme_si_on_termine(self):
        """Taper f pour terminer ne doit pas laisser la tete molle."""

        origine = calibration_grille.lireTexte
        calibration_grille.lireTexte = FauxDialogue(["o", "f"])

        try:
            issue, pose = calibration_grille.pointageManuel(self.bras)

        finally:
            calibration_grille.lireTexte = origine

        self.assertEqual(issue, "fin")
        self.assertEqual(self.bras.relaches, set())

    def test_la_tete_est_verrouillee_meme_si_on_saute(self):
        """Sauter un point ne doit pas laisser la tete molle."""

        origine = calibration_grille.lireTexte
        calibration_grille.lireTexte = FauxDialogue(["o", "s"])

        try:
            issue, pose = calibration_grille.pointageManuel(self.bras)

        finally:
            calibration_grille.lireTexte = origine

        self.assertEqual(issue, "saut")
        self.assertEqual(self.bras.relaches, set())

    def test_refus_de_relacher_ne_relache_rien(self):
        """Repondre non a la demande de confirmation ne doit couper aucun servo."""

        origine = calibration_grille.lireTexte
        calibration_grille.lireTexte = FauxDialogue(["n"])

        try:
            issue, pose = calibration_grille.pointageManuel(self.bras)

        finally:
            calibration_grille.lireTexte = origine

        self.assertEqual(issue, "saut")
        self.assertEqual(self.bras.relaches, set())

    def test_validation_rend_la_pose_tenue_a_la_main(self):
        """La pose retournee doit etre celle lue pendant que la tete etait libre."""

        origine = calibration_grille.lireTexte
        calibration_grille.lireTexte = FauxDialogue(["o", ""])

        self.bras.pose = [0.0, 50.0, FauxBras.J3_BLOQUE, 62.0, -25.0, 0.0]

        try:
            issue, pose = calibration_grille.pointageManuel(self.bras)

        finally:
            calibration_grille.lireTexte = origine

        self.assertEqual(issue, "ok")
        self.assertEqual(pose[3], 62.0)
        self.assertEqual(pose[4], -25.0)


class TestPositionInitiale(unittest.TestCase):
    """La mise en position ne doit pas attendre un axe mort."""

    def setUp(self):
        self.bras = FauxBras()
        self.sleep_origine = calibration_grille.time.sleep
        calibration_grille.time.sleep = lambda duree: None
        self.dialogue_origine = calibration_grille.lireTexte
        calibration_grille.lireTexte = FauxDialogue(["o"])

    def tearDown(self):
        calibration_grille.time.sleep = self.sleep_origine
        calibration_grille.lireTexte = self.dialogue_origine

    def test_arrivee_confirmee_malgre_j3(self):
        """J3 reste loin de sa consigne : l'arrivee doit quand meme etre reconnue."""

        cible = [10.0, 50.0, -145.0, 40.0, 0.0, 0.0]

        arrive, mesure = calibration_grille.allerPositionInitiale(self.bras, cible)

        self.assertTrue(arrive)
        self.assertNotAlmostEqual(mesure[2], cible[2])

    def test_annulation_ne_bouge_pas(self):
        """Repondre non a la question de securite ne doit envoyer aucune consigne."""

        calibration_grille.lireTexte = FauxDialogue(["n"])

        arrive, mesure = calibration_grille.allerPositionInitiale(self.bras)

        self.assertFalse(arrive)
        self.assertEqual(self.bras.consignes, [])


# =========================================================
# 6. FICHIERS
# =========================================================

class TestFichiers(unittest.TestCase):
    """Les points ecrits doivent etre relus a l'identique, et les usages separes."""

    def setUp(self):
        self.dossier = tempfile.mkdtemp()
        self.chemin = os.path.join(self.dossier, "mesures.csv")

    def tearDown(self):
        shutil.rmtree(self.dossier, ignore_errors=True)

    def ecrireQuelquesPoints(self):
        """Ecrit deux points d'ajustement et un de validation dans le fichier temporaire."""

        pose = [0.0, 50.0, -124.0, 30.0, -10.0, 0.0]

        calibration_grille.enregistrerPoint("s1", "ajustement", 1, (30.0, -10.0),
                                            pose, (6.0, -3.0), "", self.chemin)
        calibration_grille.enregistrerPoint("s1", "ajustement", 2, (35.0, -5.0),
                                            pose, (9.0, 0.0), "", self.chemin)
        calibration_grille.enregistrerPoint("s1", "validation", 3, (20.0, 5.0),
                                            pose, (3.0, 3.0), "hors grille", self.chemin)

    def test_relecture(self):
        """Les points relus doivent porter les angles mesures et l'impact en centimetres."""

        self.ecrireQuelquesPoints()

        mesures = calibration_grille.chargerMesures(self.chemin)

        self.assertEqual(len(mesures), 3)
        self.assertAlmostEqual(mesures[0]["j4"], 30.0)
        self.assertAlmostEqual(mesures[0]["u"], 6.0)

    def test_separation_des_usages(self):
        """Les points de validation ne doivent jamais entrer dans l'ajustement."""

        self.ecrireQuelquesPoints()

        ajustement = calibration_grille.chargerMesures(self.chemin, usage="ajustement")
        validation = calibration_grille.chargerMesures(self.chemin, usage="validation")

        self.assertEqual(len(ajustement), 2)
        self.assertEqual(len(validation), 1)

    def test_aller_retour_du_modele(self):
        """Un modele ecrit puis relu doit predire exactement les memes impacts."""

        modele = calibration_grille.ajusterModele(mesuresSynthetiques(tourelleDeReference()))

        chemin = os.path.join(self.dossier, "modele.json")
        calibration_grille.enregistrerModele(modele, chemin)

        relu = calibration_grille.chargerModele(chemin)

        avant = calibration_grille.predireImpact(modele, 12.0, -8.0)
        apres = calibration_grille.predireImpact(relu, 12.0, -8.0)

        self.assertAlmostEqual(avant[0], apres[0], places=9)
        self.assertAlmostEqual(avant[1], apres[1], places=9)

    def test_fichier_absent(self):
        """Un fichier de mesures absent doit donner une liste vide, pas une exception."""

        self.assertEqual(calibration_grille.chargerMesures(
            os.path.join(self.dossier, "rien.csv")), [])


class TestSeanceComplete(unittest.TestCase):
    """Le deroule d'une seance doit enchainer relachement, approche et ecriture."""

    """
    Test d'integration : il couvre le chemin que les tests
    unitaires laissent passer, celui ou les fonctions
    marchent chacune de leur cote mais s'enchainent mal.
    C'est lui qui a rattrape une constante manquante.
    """

    def setUp(self):
        self.bras = FauxBras()
        self.dossier = tempfile.mkdtemp()

        self.sleep_origine = calibration_grille.time.sleep
        self.dialogue_origine = calibration_grille.lireTexte
        self.fichier_origine = calibration_grille.FICHIER_MESURES

        calibration_grille.time.sleep = lambda duree: None
        calibration_grille.FICHIER_MESURES = os.path.join(self.dossier, "mesures.csv")

    def tearDown(self):
        calibration_grille.time.sleep = self.sleep_origine
        calibration_grille.lireTexte = self.dialogue_origine
        calibration_grille.FICHIER_MESURES = self.fichier_origine
        shutil.rmtree(self.dossier, ignore_errors=True)

    def test_deux_points_puis_fin(self):
        """Deux points releves puis fin : deux lignes ecrites, tete verrouillee."""

        calibration_grille.lireTexte = FauxDialogue([
            "o", "", "3 -2", "",
            "o", "", "-1 4", "essai",
            "o", "f",
        ])

        base = [0.0, 50.0, FauxBras.J3_BLOQUE]

        nombre = calibration_grille.seanceDeReleve(self.bras, base, 0.0, "s1", "ajustement")

        self.assertEqual(nombre, 2)
        self.assertEqual(self.bras.relaches, set())
        self.assertFalse(self.bras.release_all_appele)

        mesures = calibration_grille.chargerMesures(calibration_grille.FICHIER_MESURES)
        self.assertEqual(len(mesures), 2)
        self.assertAlmostEqual(mesures[0]["u"], 9.0)
        self.assertAlmostEqual(mesures[1]["v"], 12.0)

    def test_base_deplacee_signalee(self):
        """Si la base a bouge, le point doit etre propose au rejet."""

        self.bras.pose = [30.0, 50.0, FauxBras.J3_BLOQUE, 40.0, 0.0, 0.0]

        # 3e reponse : on refuse de garder le point ; puis on termine.
        calibration_grille.lireTexte = FauxDialogue(["o", "", "n", "o", "f"])

        base = [0.0, 50.0, FauxBras.J3_BLOQUE]

        nombre = calibration_grille.seanceDeReleve(self.bras, base, 0.0, "s1", "ajustement")

        self.assertEqual(nombre, 0)


# =========================================================
# 7. ALGEBRE
# =========================================================

class TestAlgebre(unittest.TestCase):
    """Le solveur doit resoudre juste et signaler les cas singuliers."""

    def test_systeme_simple(self):
        """Un systeme 2x2 connu doit etre resolu exactement."""

        solution = calibration_grille.resoudreSysteme([[2.0, 1.0], [1.0, 3.0]], [5.0, 10.0])

        self.assertAlmostEqual(solution[0], 1.0)
        self.assertAlmostEqual(solution[1], 3.0)

    def test_systeme_singulier(self):
        """Un systeme a deux lignes identiques doit donner None."""

        self.assertIsNone(calibration_grille.resoudreSysteme(
            [[1.0, 2.0], [2.0, 4.0]], [3.0, 6.0]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
