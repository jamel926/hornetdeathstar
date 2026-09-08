"""
=========================================================
TESTS UNITAIRES - PU_Visee
=========================================================

Ces tests s'executent sans robot et sans la bibliotheque
pymycobot : le module pymycobot est remplace par un faux
module avant l'import, et le robot par un MagicMock.

Lancement :
    python3 -m unittest discover -s test_unitaires -v
"""

import unittest
from unittest.mock import MagicMock
import sys
import os


## Remplacement de pymycobot par un faux module (absent hors Jetson)
faux_pymycobot = MagicMock()
sys.modules["pymycobot"] = faux_pymycobot
sys.modules["pymycobot.mycobot280"] = faux_pymycobot.mycobot280


## importation du module a tester
dossier_racine = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_racine)
import PU_Visee


## Point de test et solution articulaire associee
POINT_TEST = [150.0, 0.0, 200.0, 0.0, 0.0, 0.0]
ANGLES_TEST = [0.0, -30.0, -60.0, 0.0, 90.0, 0.0]


def fauxRobot(coords=None, angles=None):
    """Construit un faux robot qui atteint la consigne et resout la cinematique inverse."""

    mc = MagicMock()
    mc.is_moving.return_value = 0
    mc.get_coords.return_value = list(coords if coords is not None else POINT_TEST)
    mc.get_angles.return_value = list(angles if angles is not None else ANGLES_TEST)
    mc.solve_inv_kinematics.return_value = list(ANGLES_TEST)

    return mc


class TestVerifierCoords(unittest.TestCase):
    """Tests de la validation des coordonnees cartesiennes."""

    def test_point_valide(self):
        """Un point dans la boite de travail est accepte."""
        self.assertTrue(PU_Visee.verifierCoords(POINT_TEST))

    def test_x_hors_limite(self):
        """X = 300 est refuse : la limite est 281.45."""
        self.assertFalse(
            PU_Visee.verifierCoords([300.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        )

    def test_z_negatif_autorise(self):
        """Z = -50 est accepte : la limite basse est -70, sous le plan de la base."""
        self.assertTrue(
            PU_Visee.verifierCoords([150.0, 0.0, -50.0, 0.0, 0.0, 0.0])
        )

    def test_z_trop_bas(self):
        """Z = -100 est refuse : la limite basse est -70."""
        self.assertFalse(
            PU_Visee.verifierCoords([150.0, 0.0, -100.0, 0.0, 0.0, 0.0])
        )

    def test_mauvaise_longueur(self):
        """Une liste de 3 valeurs est refusee : verifierCoords attend les 6 coordonnees."""
        self.assertFalse(PU_Visee.verifierCoords([150.0, 0.0, 200.0]))

    def test_valeur_non_numerique(self):
        """Une valeur qui n'est pas un nombre est refusee."""
        self.assertFalse(
            PU_Visee.verifierCoords([150.0, "0", 200.0, 0.0, 0.0, 0.0])
        )


class TestEcart(unittest.TestCase):
    """Tests des mesures d'ecart."""

    def test_ecart_angulaire_simple(self):
        """L'ecart entre 10 et 40 degres vaut 30."""
        self.assertAlmostEqual(PU_Visee.ecartAngulaire(10.0, 40.0), 30.0)

    def test_ecart_angulaire_repliement(self):
        """179 et -179 ne sont distants que de 2 degres, pas de 358."""
        self.assertAlmostEqual(PU_Visee.ecartAngulaire(179.0, -179.0), 2.0)

    def test_ecart_position(self):
        """L'ecart de position est la distance euclidienne sur X, Y, Z."""
        ecart_position, _ = PU_Visee.ecartVisee(
            [3.0, 4.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.assertAlmostEqual(ecart_position, 5.0)

    def test_ecart_orientation(self):
        """L'ecart d'orientation est le plus grand ecart sur RX, RY, RZ."""
        _, ecart_orientation = PU_Visee.ecartVisee(
            [0.0, 0.0, 0.0, 1.0, 12.0, 3.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.assertAlmostEqual(ecart_orientation, 12.0)


class TestCalculerAngles(unittest.TestCase):
    """Tests du controle de faisabilite avant mouvement."""

    def test_point_resolvable(self):
        """Le firmware renvoie une solution valide, elle est retournee telle quelle."""
        mc = fauxRobot()
        self.assertEqual(PU_Visee.calculerAngles(mc, POINT_TEST), ANGLES_TEST)

    def test_aucun_mouvement_declenche(self):
        """La resolution ne doit jamais faire bouger le bras."""
        mc = fauxRobot()
        PU_Visee.calculerAngles(mc, POINT_TEST)

        mc.sync_send_coords.assert_not_called()
        mc.send_coords.assert_not_called()
        mc.send_angles.assert_not_called()

    def test_point_non_resolvable(self):
        """Une reponse inexploitable du firmware signifie point hors espace de travail."""
        mc = fauxRobot()
        mc.solve_inv_kinematics.return_value = -1

        self.assertIsNone(PU_Visee.calculerAngles(mc, POINT_TEST))

    def test_solution_hors_limites(self):
        """Une solution qui sort des limites articulaires est refusee."""
        mc = fauxRobot()
        mc.solve_inv_kinematics.return_value = [0.0, 145.0, 0.0, 0.0, 0.0, 0.0]

        self.assertIsNone(PU_Visee.calculerAngles(mc, POINT_TEST))


class TestViser(unittest.TestCase):
    """Tests de la sequence de visee complete."""

    def test_visee_reussie(self):
        """Le robot atteint le point : la fonction retourne True."""
        mc = fauxRobot()

        self.assertTrue(PU_Visee.allerAuPoint(mc, POINT_TEST))
        mc.sync_send_coords.assert_called_once()

    def test_orientation_courante_conservee(self):
        """Avec un point [X, Y, Z], l'orientation courante de l'outil est reprise."""
        mc = fauxRobot(coords=[0.0, 0.0, 0.0, -90.0, 5.0, 45.0])
        mc.get_coords.return_value = [150.0, 0.0, 200.0, -90.0, 5.0, 45.0]

        PU_Visee.allerAuPoint(mc, [150.0, 0.0, 200.0])

        coords_envoyees = mc.sync_send_coords.call_args[0][0]

        self.assertEqual(coords_envoyees[3:6], [-90.0, 5.0, 45.0])

    def test_vitesse_envoyee_en_entier(self):
        """pymycobot exige un int pour la vitesse : une vitesse flottante est convertie."""
        mc = fauxRobot()

        PU_Visee.allerAuPoint(mc, POINT_TEST, vitesse=20.0)

        vitesse_envoyee = mc.sync_send_coords.call_args[0][1]

        self.assertIsInstance(vitesse_envoyee, int)
        self.assertEqual(vitesse_envoyee, 20)

    def test_point_hors_limites_non_envoye(self):
        """Un point hors de la boite de travail est refuse avant tout envoi."""
        mc = fauxRobot()

        self.assertFalse(
            PU_Visee.allerAuPoint(mc, [300.0, 0.0, 200.0])
        )
        mc.sync_send_coords.assert_not_called()

    def test_point_inatteignable_non_envoye(self):
        """Un point que le firmware ne sait pas resoudre ne declenche aucun mouvement."""
        mc = fauxRobot()
        mc.solve_inv_kinematics.return_value = -1

        self.assertFalse(PU_Visee.allerAuPoint(mc, POINT_TEST))
        mc.sync_send_coords.assert_not_called()

    def test_point_non_atteint(self):
        """sync_send_coords retourne 1 meme sur timeout : l'ecart mesure doit faire echouer."""
        mc = fauxRobot()
        mc.get_coords.return_value = [100.0, 0.0, 200.0, 0.0, 0.0, 0.0]

        self.assertFalse(PU_Visee.allerAuPoint(mc, POINT_TEST))

    def test_orientation_non_atteinte(self):
        """Un ecart d'orientation trop grand fait echouer la visee, meme position correcte."""
        mc = fauxRobot()
        mc.get_coords.return_value = [150.0, 0.0, 200.0, 45.0, 0.0, 0.0]

        self.assertFalse(PU_Visee.allerAuPoint(mc, POINT_TEST))

    def test_arret_du_mouvement_en_cours(self):
        """Un mouvement deja lance est stoppe avant la visee."""
        mc = fauxRobot()
        mc.is_moving.return_value = 1

        PU_Visee.allerAuPoint(mc, POINT_TEST)

        mc.stop.assert_called_once()

    def test_erreur_serie(self):
        """Une erreur de communication ne fait pas planter l'appelant."""
        mc = fauxRobot()
        mc.sync_send_coords.side_effect = Exception("port ferme")

        self.assertFalse(PU_Visee.allerAuPoint(mc, POINT_TEST))

    def test_point_mal_forme(self):
        """Un point de 4 valeurs est refuse."""
        mc = fauxRobot()

        self.assertFalse(PU_Visee.allerAuPoint(mc, [1.0, 2.0, 3.0, 4.0]))


if __name__ == "__main__":
    unittest.main()
