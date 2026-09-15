import time
from datetime import datetime
import cv2

class LaCamera:
    def __init__(self , camera_index = 0):
        self.portCamera=camera_index
        self.camera = cv2.VideoCapture(self.portCamera)

    def is_connected(self):
        """Return whether OpenCV successfully opened the camera."""
        return self.camera.isOpened()

    def read(self):
        """Read and return ``(success, frame)`` from the camera."""
        return self.camera.read()

    def read_latest(self, max_skip=1):
        for _ in range(max_skip):
            if not self.camera.grab():
                break
        return self.camera.retrieve()
    
    def release(self):
        """Release the underlying OpenCV camera handle."""
        self.camera.release()


    def quitterAffichage(self):
        """Vérifie si l'utilisateur a appuyé sur la touche 'q' pour quitter l'affichage du flux vidéo en direct. Si la touche 'q' est pressée, cela signifie que l'utilisateur souhaite quitter, sinon le flux vidéo continue d'être affiché."""
        return (cv2.waitKey(1) & 0xFF) in (ord('q'), ord('Q'))

    def Screener(self):
        """Vérifie si l'utilisateur a appuyé sur la touche 'space' pour screnner l'affichage du flux vidéo en direct. Si la touche 'space' est pressée, cela signifie que l'utilisateur souhaite capturer, sinon le flux vidéo continue d'être affiché."""
        return (cv2.waitKey(1) & 0xFF) == ord(' ')
    
    def enregistrerImage(self, frame):
        """Enregistre une image capturée depuis le flux vidéo."""
        import os
        if not os.path.exists("imgs"):
            os.mkdir("imgs")
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")[:-3]
        nom_fichier = f'imgs/image_capturee_{current_time}.jpg'
        if cv2.imwrite(nom_fichier, frame):
            print(f"Image enregistrée : {nom_fichier}")
            return True
        print("Erreur lors de l'enregistrement de l'image")
        return False

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

    def capturerImageDirect(self):
        """Affiche le flux vidéo en direct de la caméra. Appuyez sur 'q' pour quitter."""
        if self.etatConnexionCamera():
            return
        while(True):
            ret,frame = self.camera.read()

            if self.flagCaptureImage(ret):
                break
            cv2.imshow('Image en direct',frame)

            if self.Screener():
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
        
        cv2.imwrite(f'image_capturee{time.time()}.jpg', frame)

        while(True):
            if self.quitterAffichage():
                break
        self.camera.release()
        cv2.destroyAllWindows()

    def tailleCamera(self):
        width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return(width,height)

if __name__ == "__main__":
    camera = LaCamera(camera_index=1)
    camera.capturerImageDirect()
