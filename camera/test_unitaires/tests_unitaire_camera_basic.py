import unittest
from unittest.mock import patch, MagicMock
import sys
import os
import cv2
import sys
import io

## importation de la classe à tester
dossier_racine = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_racine)
from camera.Module_Camera import LaCamera


class TestLaCamera(unittest.TestCase):

    @patch('cv2.VideoCapture')
    def setUp(self, mock_video_capture):
        """
        Initialisation avant chaque test.
        On 'mock' (simule) cv2.VideoCapture pour éviter que Python n'essaie 
        d'allumer votre vraie webcam à chaque test.
        """
        self.mock_camera_instance = MagicMock()
        mock_video_capture.return_value = self.mock_camera_instance
        self.camera_app = LaCamera()

    @patch('cv2.waitKey')
    def test_quitterAffichage_touche_q(self, mock_wait_key):
        # On simule l'appui sur la touche 'q'
        mock_wait_key.return_value = ord('q')
        resultat = self.camera_app.quitterAffichage()
        self.assertTrue(resultat)

    @patch('cv2.waitKey')
    def test_quitterAffichage_autre_touche(self, mock_wait_key):
        # On simule l'appui sur la touche 'a'
        mock_wait_key.return_value = ord('a')
        resultat = self.camera_app.quitterAffichage()
        self.assertFalse(resultat)

    def test_etatConnexionCamera_ouverte(self):
        # On simule une caméra fonctionnelle
        self.camera_app.camera.isOpened.return_value = True
        resultat = self.camera_app.etatConnexionCamera()
        self.assertFalse(resultat) # False signifie "pas d'erreur" dans votre code

    def test_etatConnexionCamera_fermee(self):
        # On simule une erreur de connexion à la caméra
        self.camera_app.camera.isOpened.return_value = False
        resultat = self.camera_app.etatConnexionCamera()
        self.assertTrue(resultat) # True signifie "erreur" dans votre code

    def test_flagCaptureImage_succes(self):
        # On simule la réussite de la lecture d'une frame (ret = True)
        resultat = self.camera_app.flagCaptureImage(True)
        self.assertFalse(resultat)

    def test_flagCaptureImage_echec(self):
        # On simule l'échec de la lecture d'une frame (ret = False)
        resultat = self.camera_app.flagCaptureImage(False)
        self.assertTrue(resultat)

    @patch('cv2.destroyAllWindows')
    @patch('cv2.imshow')
    def test_afficherLive_boucle_complete(self, mock_imshow, mock_destroy):
        # 1. On simule que la caméra est bien connectée
        self.camera_app.camera.isOpened.return_value = True
        
        # 2. On simule la lecture d'une image : (True, 'fausse_image_matrice')
        self.camera_app.camera.read.return_value = (True, "frame_fictive")
        
        # 3. On mock quitterAffichage pour qu'il retourne True à la première boucle
        # Cela permet d'entrer dans la boucle `while(True)`, de faire un tour, et de quitter
        with patch.object(self.camera_app, 'quitterAffichage', return_value=True):
            self.camera_app.afficherLive()
            
        # VÉRIFICATIONS (Rapports intégrés)
        # On vérifie que imshow a bien été appelé avec notre image fictive
        mock_imshow.assert_called_once_with('Image en direct', 'frame_fictive')
        # On vérifie que la caméra a bien été libérée
        self.camera_app.camera.release.assert_called_once()
        # On vérifie que les fenêtres ont été fermées
        mock_destroy.assert_called_once()

    def test_afficherLive_erreur_connexion(self):
        # On simule une caméra non connectée
        self.camera_app.camera.isOpened.return_value = False
        
        # On lance la méthode
        self.camera_app.afficherLive()
        
        # La boucle n'a jamais dû se lancer, donc read() ne doit jamais être appelé
        self.camera_app.camera.read.assert_not_called()

if __name__ == '__main__':
    unittest.main()
