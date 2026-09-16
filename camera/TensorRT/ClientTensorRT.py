"""
ClientTensorRT.py -- detection YOLO sur GPU depuis le .venv du projet, sans PyTorch.

Remplace `YOLO(...)` d'ultralytics sur la Jetson Nano. Le calcul est fait par
ServeurTensorRT.py, qui tourne avec le Python du systeme (le seul ou TensorRT
est disponible). Ce module :
    1. demarre le serveur s'il ne tourne pas deja, et attend qu'il soit pret ;
    2. redimensionne chaque image au format du reseau (letterbox, comme ultralytics) ;
    3. l'envoie au serveur, recoit la sortie brute du reseau ;
    4. la convertit en detections dans les coordonnees de l'image d'origine.

Utilisation :
    from camera.TensorRT.ClientTensorRT import DetecteurTensorRT

    with DetecteurTensorRT("chemin/best.engine", confiance=0.43) as detecteur:
        detections = detecteur.detecter(frame)
        for d in detections:
            print(d.label, d.confiance, d.x1, d.y1, d.x2, d.y2)

Les deux formats de sortie d'un export YOLO sont geres automatiquement :
    - "end2end" (YOLO26 par defaut) : (1, 300, 6) = x1, y1, x2, y2, score, classe -> pas de NMS
    - classique (end2end=False)     : (1, 4 + nb_classes, N) = cx, cy, w, h, scores -> NMS ici
"""

import json
import socket
import struct
import subprocess
import time
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np

DOSSIER_MODULE = Path(__file__).resolve().parent
CHEMIN_SERVEUR = DOSSIER_MODULE / "ServeurTensorRT.py"


class Detection(NamedTuple):
    """Une detection, en pixels de l'image d'origine."""

    label: str
    confiance: float
    x1: int
    y1: int
    x2: int
    y2: int


def recevoirExactement(connexion, taille):
    """Recoit exactement `taille` octets (recv peut en renvoyer moins)."""
    tampon = bytearray(taille)
    vue = memoryview(tampon)
    recu = 0
    while recu < taille:
        n = connexion.recv_into(vue[recu:], taille - recu)
        if n == 0:
            raise ConnectionError("le serveur TensorRT a ferme la connexion")
        recu += n
    return tampon


def letterbox(image, hauteur, largeur):
    """
    Redimensionne sans deformer et complete avec du gris (114), comme ultralytics.
    Renvoie l'image, le facteur d'echelle et le decalage (dx, dy) pour revenir a l'image d'origine.
    """
    h, w = image.shape[:2]
    echelle = min(hauteur / h, largeur / w)
    nouvelleH, nouvelleW = round(h * echelle), round(w * echelle)
    dx, dy = (largeur - nouvelleW) // 2, (hauteur - nouvelleH) // 2

    sortie = np.full((hauteur, largeur, 3), 114, dtype=np.uint8)
    sortie[dy:dy + nouvelleH, dx:dx + nouvelleW] = cv2.resize(image, (nouvelleW, nouvelleH), interpolation=cv2.INTER_LINEAR)
    return sortie, echelle, dx, dy


class DetecteurTensorRT:
    """Client du serveur TensorRT. Voir la documentation en tete de fichier."""

    def __init__(
        self,
        cheminEngine,
        confiance=0.43,
        seuilIou=0.45,
        fichierClasses=None,
        hote="127.0.0.1",
        port=5005,
        pythonSysteme="/usr/bin/python3",
        lancerServeur=True,
        delaiDemarrage=300.0,
    ):
        self.cheminEngine = Path(cheminEngine).expanduser().resolve()
        if not self.cheminEngine.is_file():
            raise FileNotFoundError(
                f"moteur TensorRT introuvable : {self.cheminEngine}\n"
                "Le construire d'abord (voir camera/TensorRT/README.md)."
            )
        self.confiance = confiance
        self.seuilIou = seuilIou
        self.noms = self._chargerClasses(fichierClasses)
        self.processus = None

        self.connexion = self._seConnecter(hote, port)
        if self.connexion is None:
            if not lancerServeur:
                raise ConnectionError(f"aucun serveur TensorRT sur {hote}:{port}")
            self._demarrerServeur(pythonSysteme, hote, port)
            self.connexion = self._attendreServeur(hote, port, delaiDemarrage)

        self.connexion.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.hauteur, self.largeur = struct.unpack(">II", recevoirExactement(self.connexion, 8))
        print(f"[ClientTensorRT] connecte, entree du reseau {self.largeur}x{self.hauteur}")

    ### --- Demarrage --- ###
    def _chargerClasses(self, fichierClasses):
        """Lit best_classes.json (ecrit par export_onnx.py) : le .engine ne contient pas les noms."""
        chemin = Path(fichierClasses) if fichierClasses else self.cheminEngine.with_name(self.cheminEngine.stem + "_classes.json")
        if not chemin.is_file():
            print(f"[ClientTensorRT] {chemin.name} absent : les classes seront affichees par leur numero")
            return {}
        with open(chemin, encoding="utf-8") as fichier:
            return {int(k): v for k, v in json.load(fichier).items()}

    @staticmethod
    def _seConnecter(hote, port):
        try:
            return socket.create_connection((hote, port), timeout=None)
        except OSError:
            return None

    def _demarrerServeur(self, pythonSysteme, hote, port):
        print(f"[ClientTensorRT] demarrage du serveur avec {pythonSysteme} (chargement du moteur : 10 a 60 s)")
        # stdout/stderr herites : les messages et erreurs du serveur s'affichent dans ce terminal.
        self.processus = subprocess.Popen(
            [pythonSysteme, str(CHEMIN_SERVEUR), "--engine", str(self.cheminEngine), "--hote", hote, "--port", str(port)]
        )

    def _attendreServeur(self, hote, port, delai):
        limite = time.monotonic() + delai
        while time.monotonic() < limite:
            if self.processus.poll() is not None:
                raise RuntimeError(
                    f"le serveur TensorRT s'est arrete (code {self.processus.returncode}), voir ses messages ci-dessus"
                )
            connexion = self._seConnecter(hote, port)
            if connexion is not None:
                return connexion
            time.sleep(0.5)
        self.fermer()
        raise TimeoutError(f"le serveur TensorRT n'a pas repondu en {delai:.0f} s")

    ### --- Detection --- ###
    def detecter(self, image):
        """Renvoie la liste des Detection trouvees dans une image BGR de n'importe quelle taille."""
        entree, echelle, dx, dy = letterbox(image, self.hauteur, self.largeur)
        sortie = self._envoyer(entree)
        boites, scores, classes = self._decoder(sortie)

        # Retour aux coordonnees de l'image d'origine.
        h, w = image.shape[:2]
        boites[:, [0, 2]] = np.clip((boites[:, [0, 2]] - dx) / echelle, 0, w - 1)
        boites[:, [1, 3]] = np.clip((boites[:, [1, 3]] - dy) / echelle, 0, h - 1)

        return [
            Detection(self.noms.get(int(c), str(int(c))), float(s), *map(int, np.round(b)))
            for b, s, c in zip(boites, scores, classes)
        ]

    def _envoyer(self, entree):
        h, w, c = entree.shape
        self.connexion.sendall(struct.pack(">III", h, w, c) + np.ascontiguousarray(entree).tobytes())

        (statut,) = struct.unpack(">B", recevoirExactement(self.connexion, 1))
        if statut != 0:
            (longueur,) = struct.unpack(">I", recevoirExactement(self.connexion, 4))
            raise RuntimeError("serveur TensorRT : " + recevoirExactement(self.connexion, longueur).decode("utf-8"))

        (ndim,) = struct.unpack(">I", recevoirExactement(self.connexion, 4))
        forme = struct.unpack(f">{ndim}I", recevoirExactement(self.connexion, 4 * ndim))
        donnees = recevoirExactement(self.connexion, int(np.prod(forme)) * 4)
        return np.frombuffer(donnees, dtype=np.float32).reshape(forme)

    def _decoder(self, sortie):
        """Sortie brute du reseau -> (boites xyxy, scores, classes) apres filtrage, en pixels du letterbox."""
        tableau = sortie[0] if sortie.ndim == 3 else sortie

        if tableau.shape[-1] == 6:
            # Format end2end : le reseau a deja fait le tri, il reste a appliquer le seuil de confiance.
            garder = tableau[:, 4] >= self.confiance
            t = tableau[garder]
            return t[:, :4].astype(np.float64), t[:, 4], t[:, 5]

        # Format classique : (4 + nb_classes, N) -> (N, 4 + nb_classes)
        if tableau.shape[0] < tableau.shape[1]:
            tableau = tableau.T
        scoresClasses = tableau[:, 4:]
        classes = scoresClasses.argmax(axis=1)
        scores = scoresClasses.max(axis=1)
        garder = scores >= self.confiance
        cxcywh, scores, classes = tableau[garder, :4], scores[garder], classes[garder]

        if len(scores) == 0:
            return np.zeros((0, 4)), scores, classes

        xywh = cxcywh.copy()
        xywh[:, 0] -= xywh[:, 2] / 2
        xywh[:, 1] -= xywh[:, 3] / 2
        indices = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), self.confiance, self.seuilIou)
        indices = np.array(indices, dtype=int).reshape(-1)

        boites = xywh[indices].astype(np.float64)
        boites[:, 2] += boites[:, 0]
        boites[:, 3] += boites[:, 1]
        return boites, scores[indices], classes[indices]

    ### --- Fermeture --- ###
    def fermer(self):
        """Ferme la connexion et arrete le serveur s'il a ete demarre par ce client."""
        if getattr(self, "connexion", None) is not None:
            self.connexion.close()
            self.connexion = None
        if self.processus is not None and self.processus.poll() is None:
            self.processus.terminate()
            try:
                self.processus.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.processus.kill()
        self.processus = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.fermer()
