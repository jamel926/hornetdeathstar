import time
import cv2

class LaCamera:
    def __init__(self , camera_index = 0):
        self.portCamera=camera_index
        self.camera = cv2.VideoCapture(self.portCamera)


    def quitterAffichage(self):
        """Vérifie si l'utilisateur a appuyé sur la touche 'q' pour quitter l'affichage du flux vidéo en direct. Si la touche 'q' est pressée, cela signifie que l'utilisateur souhaite quitter, sinon le flux vidéo continue d'être affiché."""
        return (cv2.waitKey(1) & 0xFF) in (ord('q'), ord('Q'))

    def enregistrerImage(self, frame):
        """Enregistre une image capturée depuis le flux vidéo."""
        nom_fichier = f'image_capturee{time.time()}.jpg'
        if cv2.imwrite(nom_fichier, frame):
            print(f"Image enregistrée : {nom_fichier}")
            return True
        print("Erreur lors de l'enregistrement de l'image")
        return False

    def etatConnexionCamera(self):
        """Vérifie si la caméra est connectée et prête à être utilisée. Si la caméra est ouverte, cela signifie qu'elle est connectée et prête à être utilisée, sinon il y a une erreur de connexion."""
        if self.camera.isOpened():
            print("La camera est connectée. Appuyez sur Espace ou s pour une photo, q pour quitter")
            return False
        else:
            print("Erreur de connexion")
            return True


    def flagCaptureImage(self,ret):
        """Vérifie si la capture de l'image a réussi. Si ret est True, cela signifie que la capture a réussi, sinon il y a eu une erreur."""
        if ret:
            return False
        else:
            print("Erreur à la capture de l'image")
            return True


    def afficherLive(self):
        """Affiche le flux vidéo. Appuyez sur Espace ou 's' pour une photo, 'q' pour quitter."""
        if self.etatConnexionCamera():
            return
        while(True):
            ret,frame = self.camera.read()

            if self.flagCaptureImage(ret):
                break
            cv2.imshow('Image en direct',frame)

            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord('q'), ord('Q')):
                break
            if touche in (ord('s'), ord('S'), 32):
                self.enregistrerImage(frame)
        self.camera.release()
        cv2.destroyAllWindows()

    def capturerImage(self):
        """Capture une image de la caméra et l'affiche. Appuyez sur 'q' pour quitter."""
        if self.etatConnexionCamera():
            return
        ret,frame = self.camera.read()

        if self.flagCaptureImage(ret):
            print("Erreur lors de la capture de l'image")
            self.camera.release()
            return 
        
        self.enregistrerImage(frame)

        while(True):
            if self.quitterAffichage():
                break
        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera = LaCamera(camera_index=1)
    camera.afficherLive()