
import cv2

class LaCamera:
    def __init__(self , camera_index = 0):
        self.portCamera=camera_index
        self.camera = cv2.VideoCapture(self.portCamera)


    def quitterAffichage(self):
        """Vérifie si l'utilisateur a appuyé sur la touche 'q' pour quitter l'affichage du flux vidéo en direct. Si la touche 'q' est pressée, cela signifie que l'utilisateur souhaite quitter, sinon le flux vidéo continue d'être affiché."""
        return cv2.waitKey(1) & 0xFF == ord('q')

    def etatConnexionCamera(self):
        """Vérifie si la caméra est connectée et prête à être utilisée. Si la caméra est ouverte, cela signifie qu'elle est connectée et prête à être utilisée, sinon il y a une erreur de connexion."""
        if self.camera.isOpened():
            print("La camera est connectée. Appuyez sur q pour quitter")
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
        """Affiche le flux vidéo en direct de la caméra. Appuyez sur 'q' pour quitter."""
        if self.etatConnexionCamera():
            return
        while(True):
            ret,frame = self.camera.read()

            if self.flagCaptureImage(ret):
                break
            cv2.imshow('Image en direct',frame)

            if self.quitterAffichage():
                break
        self.camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = LaCamera(camera_index=0)
    camera.afficherLive()