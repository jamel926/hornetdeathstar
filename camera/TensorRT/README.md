# Detection YOLO sur GPU avec TensorRT (Jetson Nano)

## Pourquoi ce dossier

Sur la Jetson Nano, `main.py` plante avec `Illegal instruction (core dumped)` :

- le PyTorch installe par pip (2.x, CUDA 13) ne peut pas utiliser le GPU de la Nano, dont le pilote est en CUDA 10.2 (JetPack 4) ;
- sur le CPU, il utilise des instructions que le processeur de la Nano (Cortex-A57, ARMv8.0) ne possede pas.

Il n'existe pas de PyTorch GPU officiel pour Python 3.10 sur cette carte. La solution retenue : **ne plus utiliser PyTorch pour la detection** et faire tourner le modele avec **TensorRT**, le moteur d'inference de NVIDIA deja installe par JetPack.

Probleme : TensorRT n'est utilisable que depuis le Python **du systeme** (`/usr/bin/python3`), pas depuis le `.venv` en 3.10. La detection tourne donc dans un **processus separe** :

```
main_trt.py (.venv, Python 3.10)                    ServeurTensorRT.py (/usr/bin/python3)
  camera -> image -> ClientTensorRT  --- socket --->  TensorRT sur le GPU
             detections <-----------  --- local  <---  sortie brute du reseau
```

Le client demarre et arrete le serveur tout seul : `main_trt.py` se lance comme `main.py`.

## Contenu

| Fichier | Ou le lancer | Role |
|---|---|---|
| `export_onnx.py` | **PC** | `best.pt` -> `best.onnx` + `best_classes.json` |
| `construire_engine.sh` | **Jetson** (chaque robot) | `best.onnx` -> `best.engine` (FP16) |
| `ServeurTensorRT.py` | Jetson, Python systeme | Execute le modele sur le GPU (lance automatiquement) |
| `ClientTensorRT.py` | Jetson, `.venv` | Classe `DetecteurTensorRT` qui remplace `YOLO(...)` |
| `../../main_trt.py` | Jetson, `.venv` | Copie de `main.py` utilisant `DetecteurTensorRT` |

## Mise en place

### 1. Sur le PC : export ONNX

```bash
pip install "ultralytics>=8.4" onnx      # 8.4 minimum : le modele est un YOLO26
python camera/TensorRT/export_onnx.py camera/IA/Model5/runs/detect/train/weights/best.pt
```

Copier `best.onnx` et `best_classes.json` sur la Jetson, dans `camera/IA/Model5/runs/detect/train/weights/`.

### 2. Sur la Jetson : verifier TensorRT

Hors de tout environnement (`deactivate`, `conda deactivate`) :

```bash
/usr/bin/python3 -c "import tensorrt, numpy; print('TensorRT', tensorrt.__version__)"
```

S'il manque `tensorrt` : `sudo apt install python3-libnvinfer`. S'il manque `numpy` : `sudo apt install python3-numpy`.

### 3. Sur la Jetson : construire le moteur (5 a 15 min)

```bash
./camera/TensorRT/construire_engine.sh camera/IA/Model5/runs/detect/train/weights/best.onnx
```

A la fin, trtexec affiche une ligne `Throughput: ... qps` : c'est le nombre d'images par seconde que le GPU peut traiter.

**Si la construction echoue sur une couche non supportee** (TopK, GatherElements, ...) : la tete end2end de YOLO26 est trop recente pour TensorRT 8.2. Refaire l'etape 1 avec `--sans-end2end`, puis l'etape 3. Le client fait alors le tri des boites (NMS) lui-meme, rien d'autre a changer.

### 4. Lancer

```bash
source .venv/bin/activate
python3 main_trt.py
```

Le temps de detection de chaque image est affiche en haut a gauche de la fenetre.

## Depannage

| Symptome | Cause / solution |
|---|---|
| `moteur TensorRT introuvable` | L'etape 3 n'a pas ete faite, ou `CHEMIN_ENGINE` dans `main_trt.py` pointe ailleurs |
| `le serveur TensorRT s'est arrete` | Lire les messages `[ServeurTensorRT]` juste au-dessus. Pour les voir seuls : `/usr/bin/python3 camera/TensorRT/ServeurTensorRT.py --engine chemin/best.engine` |
| `impossible de charger ... .engine` | Moteur construit sur une autre machine ou une autre version de TensorRT : relancer l'etape 3 sur ce robot |
| `Killed` pendant l'etape 3 | Memoire insuffisante : fermer VS Code et le navigateur, relancer |
| `Address already in use` | Un ancien serveur tourne encore : `pkill -f ServeurTensorRT.py` |
| Classes affichees en chiffres | `best_classes.json` absent a cote du `.engine` |

## A savoir

- **Ne pas commiter les `.engine`** : ils dependent du GPU et de la version de TensorRT, chaque robot construit le sien.
- **Apres un nouvel entrainement**, refaire les etapes 1 et 3 : le `.engine` ne se met pas a jour tout seul.
- Le seuil de confiance (0.43) est dans `main_trt.py` (`SEUIL_CONFIANCE`).
