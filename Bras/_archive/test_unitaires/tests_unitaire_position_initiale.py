"""
=========================================================
TESTS UNITAIRES - PU_PositionInitiale
=========================================================

Ces tests s'executent sans robot et sans la bibliotheque
pymycobot : le module pymycobot est remplace par un faux
module avant l'import de PU_PositionInitiale, et le robot
lui-meme par un MagicMock.

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
import PU_PositionInitiale


class TestVerifierAngles(unittest.TestCase):
    """Tests de la validation des angles articulaires."""

    def test_pose_valide(self):
        """Une pose dans les limites est acceptee."""
        self.assertTrue(
            PU_PositionInitiale.verifierAngles([0.0, -30.0, -60.0, 0.0, 90.0, 0.0])
        )

    def test_position_initiale_par_defaut_valide(self):
        """La constante ANGLES_POSITION_INITIALE du module est elle-meme valide."""
        self.assertTrue(
            PU_PositionInitiale.verifierAngles(
                PU_PositionInitiale.ANGLES_POSITION_INITIALE
            )
        )

    def test_j2_hors_limite(self):
        """J2 = 145 est refuse : la limite reelle est 140, pas 150."""
        self.assertFalse(
            PU_PositionInitiale.verifierAngles([0.0, 145.0, 0.0, 0.0, 0.0, 0.0])
        )

    def test_j5_hors_limite(self):
        """J5 = -160 est refuse : la limite basse reelle est -155."""
        self.assertFalse(
            PU_PositionInitiale.verifierAngles([0.0, 0.0, 0.0, 0.0, -160.0, 0.0])
        )

    def test_j5_asymetrique(self):
        """J5 = 158 est accepte : la limite haute reelle est 160, l'articulation est asymetrique."""
        self.assertTrue(
            PU_PositionInitiale.verifierAngles([0.0, 0.0, 0.0, 0.0, 158.0, 0.0])
        )

    def test_mauvaise_longueur(self):
        """Une liste de 5 angles est refusee."""
        self.assertFalse(
            PU_PositionInitiale.verifierAngles([0.0, 0.0, 0.0, 0.0, 0.0])
        )

    def test_valeur_non_numerique(self):
        """Une valeur qui n'est pas un nombre est refusee."""
        self.assertFalse(
            PU_PositionInitiale.verifierAngles([0.0, "30", 0.0, 0.0, 0.0, 0.0])
        )


class TestAllerPositionInitiale(unittest.TestCase):
    """Tests du deplacement vers la position initiale."""

    def setUp(self):
        """Prepare un faux robot immobile qui atteint toujours la consigne."""
        self.cible = [0.0, -30.0, -60.0, 0.0, 90.0, 0.0]

        self.mc = MagicMock()
        self.mc.is_moving.return_value = 0
        self.mc.get_angles.return_value = list(self.cible)

    def test_deplacement_reussi(self):
        """Le robot atteint la consigne : la fonction retourne True."""
        self.assertTrue(
            PU_PositionInitiale.allerPositionInitiale(self.mc, self.cible)
        )
        self.mc.sync_send_angles.assert_called_once()

    def test_vitesse_envoyee_en_entier(self):
        """pymycobot exige un int pour la vitesse : une vitesse flottante est convertie."""
        PU_PositionInitiale.allerPositionInitiale(self.mc, self.cible, vitesse=20.0)

        vitesse_envoyee = self.mc.sync_send_angles.call_args[0][1]

        self.assertIsInstance(vitesse_envoyee, int)
        self.assertEqual(vitesse_envoyee, 20)

    def test_consigne_invalide_non_envoyee(self):
        """Une consigne hors limites est refusee avant tout envoi au robot."""
        self.assertFalse(
            PU_PositionInitiale.allerPositionInitiale(
                self.mc, [0.0, 145.0, 0.0, 0.0, 0.0, 0.0]
            )
        )
        self.mc.sync_send_angles.assert_not_called()

    def test_position_non_atteinte(self):
        """sync_send_angles retourne 1 meme sur timeout : l'ecart mesure doit faire echouer."""
        self.mc.get_angles.return_value = [0.0, -30.0, -20.0, 0.0, 90.0, 0.0]

        self.assertFalse(
            PU_PositionInitiale.allerPositionInitiale(self.mc, self.cible)
        )

    def test_arret_du_mouvement_en_cours(self):
        """Un mouvement deja lance est stoppe avant la mise en position."""
        self.mc.is_moving.return_value = 1

        PU_PositionInitiale.allerPositionInitiale(self.mc, self.cible)

        self.mc.stop.assert_called_once()

    def test_robot_muet(self):
        """Si le robot ne renvoie plus sa position, la mise en position est declaree en echec."""
        self.mc.get_angles.return_value = None

        self.assertFalse(
            PU_PositionInitiale.allerPositionInitiale(self.mc, self.cible)
        )

    def test_erreur_serie(self):
        """Une erreur de communication ne fait pas planter l'appelant."""
        self.mc.sync_send_angles.side_effect = Exception("port ferme")

        self.assertFalse(
            PU_PositionInitiale.allerPositionInitiale(self.mc, self.cible)
        )


class TestEstEnPositionInitiale(unittest.TestCase):
    """Tests du controle de position sans mouvement."""

    def setUp(self):
        """Prepare un faux robot et une pose de reference."""
        self.cible = [0.0, -30.0, -60.0, 0.0, 90.0, 0.0]
        self.mc = MagicMock()

    def test_bras_en_place(self):
        """Un ecart inferieur a la tolerance est considere comme en place."""
        self.mc.get_angles.return_value = [0.5, -30.0, -59.0, 0.0, 90.0, 0.0]

        self.assertTrue(
            PU_PositionInitiale.estEnPositionInitiale(self.mc, self.cible)
        )

    def test_bras_hors_place(self):
        """Un ecart superieur a la tolerance est detecte."""
        self.mc.get_angles.return_value = [0.0, -30.0, -60.0, 0.0, 45.0, 0.0]

        self.assertFalse(
            PU_PositionInitiale.estEnPositionInitiale(self.mc, self.cible)
        )

    def test_aucun_mouvement_declenche(self):
        """Le controle ne doit jamais envoyer de commande de deplacement."""
        self.mc.get_angles.return_value = list(self.cible)

        PU_PositionInitiale.estEnPositionInitiale(self.mc, self.cible)

        self.mc.sync_send_angles.assert_not_called()
        self.mc.send_angles.assert_not_called()


if __name__ == "__main__":
    unittest.main()
