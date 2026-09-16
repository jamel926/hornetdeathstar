"""
=========================================================
TESTS UNITAIRES - POINTAGE ET VALIDATION
=========================================================

Ces tests s'executent sans robot et sans la bibliotheque
pymycobot. Le modele de pointage est verifie sur des
donnees SYNTHETIQUES dont on connait la reponse : on
fabrique des impacts avec une geometrie choisie, et on
verifie que l'ajustement la retrouve.

Lancement :
    python3 -m unittest discover -s test_unitaires -v
"""

import unittest
from unittest.mock import MagicMock
import csv
import shutil
import sys
import os
import tempfile


## Remplacement de pymycobot par un faux module (absent hors Jetson)
faux_pymycobot = MagicMock()
sys.modules["pymycobot"] = faux_pymycobot
sys.modules["pymycobot.mycobot280"] = faux_pymycobot.mycobot280


## importation des modules a tester
dossier_racine = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_racine)
import PU_Pointage
import PU_Tete
import Bras.ModuleDeDeplacement.tests_files.test_validation_pointage as validation


def impactSynthetique(j4, j5):
    """Geometrie de reference des tests : une correspondance bilineaire connue."""

    u = 1.5 + 0.40 * j4 - 0.10 * j5 + 0.002 * j4 * j5
    v = -2.0 + 0.05 * j4 + 0.35 * j5 - 0.001 * j4 * j5

    return u, v


def mesuresSynthetiques(angles_j4=(60.0, 75.0, 90.0), angles_j5=(-10.0, 0.0, 10.0)):
    """Fabrique une grille de mesures parfaites a partir de la geometrie de reference."""

    mesures = []

    for j4 in angles_j4:
        for j5 in angles_j5:
            u, v = impactSynthetique(j4, j5)
            mesures.append({"j4": j4, "j5": j5, "u": u, "v": v})

    return mesures


class TestAlgebre(unittest.TestCase):
    """Tests du solveur lineaire ecrit a la main."""

    def test_systeme_simple(self):
        """Un systeme 2x2 connu est resolu exactement."""
        solution = PU_Pointage.resoudreSysteme([[2.0, 1.0], [1.0, 3.0]], [5.0, 10.0])
        self.assertAlmostEqual(solution[0], 1.0)
        self.assertAlmostEqual(solution[1], 3.0)

    def test_pivot_necessaire(self):
        """Un zero en diagonale n'empeche pas la resolution, le pivot partiel le gere."""
        solution = PU_Pointage.resoudreSysteme([[0.0, 2.0], [1.0, 1.0]], [4.0, 3.0])
        self.assertAlmostEqual(solution[0], 1.0)
        self.assertAlmostEqual(solution[1], 2.0)

    def test_systeme_singulier(self):
        """Un systeme singulier retourne None au lieu de diviser par zero."""
        self.assertIsNone(
            PU_Pointage.resoudreSysteme([[1.0, 2.0], [2.0, 4.0]], [3.0, 6.0])
        )

    def test_moindres_carres_exact(self):
        """Sur des donnees sans bruit, les moindres carres retrouvent la solution exacte."""
        lignes = [[1.0, 0.0], [1.0, 1.0], [1.0, 2.0]]
        valeurs = [3.0, 5.0, 7.0]
        solution = PU_Pointage.moindresCarres(lignes, valeurs)
        self.assertAlmostEqual(solution[0], 3.0, places=6)
        self.assertAlmostEqual(solution[1], 2.0, places=6)


class TestAjustement(unittest.TestCase):
    """Tests de l'ajustement du modele de pointage."""

    def test_modele_retrouve_la_geometrie(self):
        """Le modele ajuste predit les impacts de la geometrie qui a servi a les fabriquer."""
        modele = PU_Pointage.ajusterModele(mesuresSynthetiques(), "test")

        for j4, j5 in ((70.0, -5.0), (80.0, 5.0), (75.0, 0.0)):
            attendu = impactSynthetique(j4, j5)
            obtenu = PU_Pointage.predireImpact(modele, j4, j5)
            self.assertAlmostEqual(obtenu[0], attendu[0], places=4)
            self.assertAlmostEqual(obtenu[1], attendu[1], places=4)

    def test_residus_nuls_sans_bruit(self):
        """Des mesures parfaites donnent des residus negligeables."""
        modele = PU_Pointage.ajusterModele(mesuresSynthetiques(), "test")
        self.assertLess(modele["residus"]["max_cm"], 1e-6)

    def test_trop_peu_de_points(self):
        """Moins de quatre points ne permet pas d'ajuster quatre coefficients."""
        self.assertIsNone(PU_Pointage.ajusterModele(mesuresSynthetiques()[:3], "test"))

    def test_grille_degeneree(self):
        """Une grille ou J5 ne varie pas rend le systeme singulier et l'ajustement echoue."""
        mesures = mesuresSynthetiques(angles_j5=(0.0,))
        self.assertIsNone(PU_Pointage.ajusterModele(mesures, "test"))

    def test_enveloppe_enregistree(self):
        """L'enveloppe retenue est celle des points reellement balayes."""
        modele = PU_Pointage.ajusterModele(mesuresSynthetiques(), "test")
        self.assertAlmostEqual(modele["enveloppe"]["j4_min"], 60.0)
        self.assertAlmostEqual(modele["enveloppe"]["j4_max"], 90.0)


class TestInversion(unittest.TestCase):
    """Tests de la recherche des angles pour un point vise."""

    def setUp(self):
        """Modele ajuste sur la geometrie de reference."""
        self.modele = PU_Pointage.ajusterModele(mesuresSynthetiques(), "test")

    def test_aller_retour(self):
        """Predire puis inverser redonne les angles de depart."""
        for j4, j5 in ((70.0, -5.0), (85.0, 7.0), (75.0, 0.0)):
            u, v = PU_Pointage.predireImpact(self.modele, j4, j5)
            angles = PU_Pointage.anglesPourPoint(self.modele, u, v)
            self.assertIsNotNone(angles)
            self.assertAlmostEqual(angles[0], j4, places=3)
            self.assertAlmostEqual(angles[1], j5, places=3)

    def test_refus_hors_enveloppe(self):
        """Un point qui demanderait des angles hors zone balayee est refuse."""
        u, v = PU_Pointage.predireImpact(self.modele, 150.0, 40.0)
        self.assertIsNone(PU_Pointage.anglesPourPoint(self.modele, u, v))

    def test_marge_acceptee(self):
        """Une petite sortie de zone, dans la marge, reste acceptee."""
        u, v = PU_Pointage.predireImpact(self.modele, 91.0, 0.0)
        self.assertIsNotNone(PU_Pointage.anglesPourPoint(self.modele, u, v))

    def test_dans_enveloppe(self):
        """Le test d'enveloppe suit les bornes balayees, marge comprise."""
        self.assertTrue(PU_Pointage.dansEnveloppe(self.modele, 75.0, 0.0))
        self.assertFalse(PU_Pointage.dansEnveloppe(self.modele, 120.0, 0.0))


class TestChargementMesures(unittest.TestCase):
    """Tests de la relecture du fichier de balayage."""

    def setUp(self):
        """Ecrit un CSV de balayage temporaire contenant les trois cas a distinguer."""
        self.dossier = tempfile.mkdtemp()
        self.chemin = os.path.join(self.dossier, "balayage.csv")

        with open(self.chemin, "w", newline="") as f:
            r = csv.writer(f)
            r.writerow(PU_Pointage.ENTETE_BALAYAGE)
            ## point d'ajustement normal
            r.writerow(["d", "c1", "ajustement", 1, 70, 0,
                        2, 48, -144, 70.1, 0.2, 1, 30.0, 5.0, ""])
            ## point saute pendant le balayage : pas d'impact
            r.writerow(["d", "c1", "ajustement", 2, 75, 0,
                        2, 48, -144, 75.0, 0.0, 1, "", "", "saute"])
            ## point garde pour la validation
            r.writerow(["d", "c1", "validation", 3, 80, 5,
                        2, 48, -144, 80.2, 5.1, 1, 35.0, 8.0, ""])

    def tearDown(self):
        """Supprime le dossier temporaire."""
        shutil.rmtree(self.dossier, ignore_errors=True)

    def test_filtre_usage(self):
        """Seuls les points d'ajustement sont charges par defaut."""
        mesures = PU_Pointage.chargerMesures(self.chemin, usage="ajustement")
        self.assertEqual(len(mesures), 1)

    def test_point_saute_ignore(self):
        """Un point sans impact est ignore sans faire echouer la lecture."""
        mesures = PU_Pointage.chargerMesures(self.chemin, usage="ajustement")
        self.assertAlmostEqual(mesures[0]["u"], 30.0)

    def test_angles_mesures_et_non_consigne(self):
        """Le modele est nourri des angles mesures, pas des angles commandes."""
        mesures = PU_Pointage.chargerMesures(self.chemin, usage="ajustement")
        self.assertAlmostEqual(mesures[0]["j4"], 70.1)
        self.assertAlmostEqual(mesures[0]["j5"], 0.2)

    def test_validation_separee(self):
        """Les points de validation se chargent a part."""
        mesures = PU_Pointage.chargerMesures(self.chemin, usage="validation")
        self.assertEqual(len(mesures), 1)

    def test_fichier_absent(self):
        """Un fichier absent retourne une liste vide au lieu de lever."""
        self.assertEqual(
            PU_Pointage.chargerMesures(os.path.join(self.dossier, "rien.csv")), []
        )


class FauxBrasTete:
    """Bras qui memorise les consignes recues, pour verifier le chemin d'approche."""

    def __init__(self):
        self.pose = [2.0, 48.0, -144.0, 75.0, 0.0, 1.0]
        self.consignes = []

    def is_moving(self):
        return 0

    def stop(self):
        pass

    def send_angles(self, angles, vitesse):
        self.consignes.append(list(angles))
        self.pose = list(angles)
        return 1

    def get_angles(self):
        return list(self.pose)


class TestApproche(unittest.TestCase):
    """Tests de l'approche constante, qui rattrape le jeu toujours du meme cote."""

    def test_passage_par_la_pose_d_approche(self):
        """La tete passe par la pose decalee avant de rejoindre sa cible."""
        bras = FauxBrasTete()
        PU_Tete.bougerTeteAvecApproche(bras, [80.0, 10.0, 1.0], timeout=2)

        self.assertEqual(len(bras.consignes), 2)
        self.assertAlmostEqual(bras.consignes[0][3], 80.0 + PU_Tete.APPROCHE_TETE[0])
        self.assertAlmostEqual(bras.consignes[1][3], 80.0)

    def test_arrivee_sur_la_cible(self):
        """La derniere consigne est bien la pose demandee."""
        bras = FauxBrasTete()
        arrive, mesure = PU_Tete.bougerTeteAvecApproche(bras, [80.0, 10.0, 1.0], timeout=2)

        self.assertTrue(arrive)
        self.assertAlmostEqual(mesure[4], 10.0)

    def test_approche_hors_limites(self):
        """Si la pose d'approche sort des limites, la cible est rejointe directement."""
        bras = FauxBrasTete()
        ## J4 = 145 : l'approche a 153 depasserait la limite de 150
        PU_Tete.bougerTeteAvecApproche(bras, [145.0, 10.0, 1.0], timeout=2)

        self.assertEqual(len(bras.consignes), 1)
        self.assertAlmostEqual(bras.consignes[0][3], 145.0)


class TestValidation(unittest.TestCase):
    """Tests de la lecture des erreurs de visee."""

    def test_biais_pur(self):
        """Une erreur identique sur tous les points est un biais, sans dispersion."""
        resultat = validation.analyserErreurs([(2.0, 1.0), (2.0, 1.0), (2.0, 1.0)])
        self.assertAlmostEqual(resultat["biais_u_cm"], 2.0)
        self.assertAlmostEqual(resultat["dispersion_u_cm"], 0.0)

    def test_dispersion_pure(self):
        """Des erreurs symetriques donnent un biais nul mais une dispersion reelle."""
        resultat = validation.analyserErreurs([(2.0, 0.0), (-2.0, 0.0)])
        self.assertAlmostEqual(resultat["biais_u_cm"], 0.0)
        self.assertGreater(resultat["dispersion_u_cm"], 1.0)

    def test_erreur_maximale(self):
        """L'erreur maximale est bien une distance, pas une composante."""
        resultat = validation.analyserErreurs([(3.0, 4.0), (0.0, 0.0)])
        self.assertAlmostEqual(resultat["erreur_max_cm"], 5.0)

    def test_points_tires_dans_la_zone(self):
        """Les points de validation tires restent visables par le modele."""
        modele = PU_Pointage.ajusterModele(mesuresSynthetiques(), "test")
        points = validation.tirerPoints(modele, 5)

        self.assertGreater(len(points), 0)
        for u, v in points:
            self.assertIsNotNone(PU_Pointage.anglesPourPoint(modele, u, v))


if __name__ == "__main__":
    unittest.main()
