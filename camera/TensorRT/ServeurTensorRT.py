"""
ServeurTensorRT.py -- processus de detection YOLO sur le GPU de la Jetson Nano.

Pourquoi un processus separe :
    Les bindings Python de TensorRT fournis par JetPack sont compiles pour le
    Python du SYSTEME (/usr/bin/python3, 3.6 sur JetPack 4), alors que le projet
    tourne dans un .venv en Python 3.10. Ce script tourne donc avec le Python du
    systeme, charge le moteur TensorRT (.engine) et repond aux demandes de
    detection envoyees par ClientTensorRT.py via un socket local.

Ce script n'est normalement pas lance a la main : ClientTensorRT le demarre tout
seul. Pour le lancer a la main (utile pour lire ses erreurs) :
    /usr/bin/python3 camera/TensorRT/ServeurTensorRT.py --engine chemin/best.engine

Dependances (Python du systeme uniquement) : tensorrt et numpy, deja fournis par
JetPack. Pas besoin de pycuda : la memoire GPU est geree directement avec
libcudart via ctypes.

ATTENTION : ce fichier doit rester compatible Python 3.6 : pas de dataclass, pas de
"from __future__ import annotations", pas d'annotations du type list[int].

Protocole (tous les entiers en big-endian) :
    1. a la connexion, le serveur envoie la taille d'entree du reseau : ">II" (hauteur, largeur)
    2. le client envoie une image deja redimensionnee : ">III" (h, w, 3) puis h*w*3 octets BGR uint8
    3. le serveur repond :
         ">B" 0, puis ">I" nombre de dimensions, puis ">I" par dimension, puis les float32 de la sortie
       ou en cas d'erreur :
         ">B" 1, puis ">I" longueur, puis le message en UTF-8
"""

import argparse
import ctypes
import signal
import socket
import struct
import sys
import time

import numpy as np
import tensorrt as trt


### --- Acces minimal a la memoire GPU via libcudart --- ###
# On importe tensorrt AVANT de charger libcudart : TensorRT a deja charge sa
# propre libcudart, et dlopen renvoie alors la meme bibliotheque (meme contexte CUDA).
CUDA_MEMCPY_HOTE_VERS_GPU = 1
CUDA_MEMCPY_GPU_VERS_HOTE = 2


def chargerCudart():
    """Charge libcudart en essayant les noms usuels sur Jetson."""
    noms = [
        "libcudart.so.10.2",
        "libcudart.so",
        "/usr/local/cuda/lib64/libcudart.so",
    ]
    for nom in noms:
        try:
            cudart = ctypes.CDLL(nom)
            break
        except OSError:
            continue
    else:
        raise RuntimeError("libcudart introuvable (essaye : %s)" % ", ".join(noms))

    cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
    cudart.cudaMalloc.restype = ctypes.c_int
    cudart.cudaFree.argtypes = [ctypes.c_void_p]
    cudart.cudaFree.restype = ctypes.c_int
    cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
    cudart.cudaMemcpy.restype = ctypes.c_int
    return cudart


def verifierCuda(code, action):
    """Leve une erreur lisible si un appel CUDA a echoue."""
    if code != 0:
        raise RuntimeError("erreur CUDA %d pendant : %s" % (code, action))


### --- Moteur TensorRT --- ###
class MoteurTensorRT:
    """Charge un fichier .engine et execute une inference sur une image."""

    def __init__(self, cheminEngine):
        self.cudart = chargerCudart()
        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, "")

        # Le runtime est garde en attribut : il doit vivre au moins aussi longtemps que le moteur.
        self.runtime = trt.Runtime(self.logger)
        with open(cheminEngine, "rb") as fichier:
            self.engine = self.runtime.deserialize_cuda_engine(fichier.read())
        if self.engine is None:
            raise RuntimeError(
                "impossible de charger %s. Le .engine doit etre construit sur CETTE "
                "Jetson avec CETTE version de TensorRT (voir construire_engine.sh)." % cheminEngine
            )
        self.context = self.engine.create_execution_context()

        # Pour chaque entree/sortie : un tableau numpy cote CPU et un bloc de memoire cote GPU.
        self.entrees = []
        self.sorties = []
        self.bindings = []
        self.pointeursGpu = []
        for i in range(self.engine.num_bindings):
            forme = tuple(self.engine.get_binding_shape(i))
            if any(d < 0 for d in forme):
                raise RuntimeError("forme dynamique %s non geree : exporter avec dynamic=False" % (forme,))
            typeNumpy = trt.nptype(self.engine.get_binding_dtype(i))
            tableauHote = np.empty(forme, dtype=typeNumpy)

            pointeur = ctypes.c_void_p()
            verifierCuda(self.cudart.cudaMalloc(ctypes.byref(pointeur), tableauHote.nbytes), "allocation GPU")
            self.pointeursGpu.append(pointeur)
            self.bindings.append(pointeur.value)

            binding = (self.engine.get_binding_name(i), tableauHote, pointeur)
            if self.engine.binding_is_input(i):
                self.entrees.append(binding)
            else:
                self.sorties.append(binding)

        if len(self.entrees) != 1 or len(self.sorties) < 1:
            raise RuntimeError("le moteur doit avoir 1 entree et au moins 1 sortie")

        nom, tableau, _ = self.entrees[0]
        # Forme attendue : (1, 3, hauteur, largeur)
        self.hauteur = tableau.shape[2]
        self.largeur = tableau.shape[3]
        print("[ServeurTensorRT] entree %s %s" % (nom, tableau.shape))
        for nom, tableau, _ in self.sorties:
            print("[ServeurTensorRT] sortie %s %s" % (nom, tableau.shape))
        if len(self.sorties) > 1:
            print("[ServeurTensorRT] plusieurs sorties : seule la premiere est renvoyee au client")

    def inferer(self, imageBgr):
        """imageBgr : uint8 (hauteur, largeur, 3) deja au format du reseau. Renvoie la premiere sortie."""
        _, entree, pointeurEntree = self.entrees[0]

        # BGR -> RGB, HWC -> CHW, [0, 255] -> [0, 1] : pretraitement standard d'ultralytics.
        entree[0] = imageBgr[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        verifierCuda(
            self.cudart.cudaMemcpy(pointeurEntree, entree.ctypes.data, entree.nbytes, CUDA_MEMCPY_HOTE_VERS_GPU),
            "copie de l'image vers le GPU",
        )

        if not self.context.execute_v2(self.bindings):
            raise RuntimeError("execute_v2 a echoue")

        _, sortie, pointeurSortie = self.sorties[0]
        verifierCuda(
            self.cudart.cudaMemcpy(sortie.ctypes.data, pointeurSortie, sortie.nbytes, CUDA_MEMCPY_GPU_VERS_HOTE),
            "copie du resultat depuis le GPU",
        )
        return sortie

    def liberer(self):
        """Libere la memoire GPU."""
        for pointeur in self.pointeursGpu:
            self.cudart.cudaFree(pointeur)
        self.pointeursGpu = []


### --- Reseau --- ###
def recevoirExactement(connexion, taille):
    """Recoit exactement `taille` octets (recv peut en renvoyer moins)."""
    tampon = bytearray(taille)
    vue = memoryview(tampon)
    recu = 0
    while recu < taille:
        n = connexion.recv_into(vue[recu:], taille - recu)
        if n == 0:
            raise ConnectionError("client deconnecte")
        recu += n
    return tampon


def envoyerErreur(connexion, message):
    donnees = message.encode("utf-8")
    connexion.sendall(struct.pack(">BI", 1, len(donnees)) + donnees)


def servirClient(connexion, moteur):
    """Traite les images d'un client jusqu'a sa deconnexion."""
    connexion.sendall(struct.pack(">II", moteur.hauteur, moteur.largeur))
    nbImages = 0
    debut = time.time()
    while True:
        h, w, c = struct.unpack(">III", recevoirExactement(connexion, 12))
        image = np.frombuffer(recevoirExactement(connexion, h * w * c), dtype=np.uint8).reshape(h, w, c)

        if (h, w, c) != (moteur.hauteur, moteur.largeur, 3):
            envoyerErreur(connexion, "image %dx%dx%d recue, %dx%dx3 attendue" % (h, w, c, moteur.hauteur, moteur.largeur))
            continue

        try:
            sortie = moteur.inferer(image)
        except Exception as erreur:
            envoyerErreur(connexion, str(erreur))
            continue

        sortie = np.ascontiguousarray(sortie, dtype=np.float32)
        entete = struct.pack(">BI", 0, sortie.ndim) + struct.pack(">%dI" % sortie.ndim, *sortie.shape)
        connexion.sendall(entete + sortie.tobytes())

        nbImages += 1
        if nbImages % 100 == 0:
            print("[ServeurTensorRT] %d images, %.1f images/s" % (nbImages, nbImages / (time.time() - debut)))


def main():
    parser = argparse.ArgumentParser(description="Serveur de detection YOLO TensorRT (Python systeme).")
    parser.add_argument("--engine", required=True, help="fichier .engine construit par construire_engine.sh")
    parser.add_argument("--hote", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    # Un SIGTERM (envoye par le client a sa fermeture) doit passer par le finally pour liberer le GPU.
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))

    print("[ServeurTensorRT] TensorRT %s, chargement de %s ..." % (trt.__version__, args.engine))
    moteur = MoteurTensorRT(args.engine)

    # Une inference a vide : la premiere est toujours lente (initialisation CUDA).
    moteur.inferer(np.zeros((moteur.hauteur, moteur.largeur, 3), dtype=np.uint8))

    serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serveur.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        serveur.bind((args.hote, args.port))
        serveur.listen(1)
        print("[ServeurTensorRT] pret sur %s:%d" % (args.hote, args.port))
        while True:
            connexion, adresse = serveur.accept()
            connexion.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("[ServeurTensorRT] client connecte %s:%d" % adresse)
            try:
                servirClient(connexion, moteur)
            except ConnectionError:
                print("[ServeurTensorRT] client deconnecte")
            finally:
                connexion.close()
    except KeyboardInterrupt:
        pass
    finally:
        serveur.close()
        moteur.liberer()
        print("[ServeurTensorRT] arrete")


if __name__ == "__main__":
    main()
