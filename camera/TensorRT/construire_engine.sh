#!/usr/bin/env bash
#
# construire_engine.sh -- ETAPE 2, a lancer SUR LA JETSON (sur chaque robot).
#
# Transforme le fichier .onnx (produit sur PC par export_onnx.py) en moteur
# TensorRT .engine optimise pour le GPU de CETTE carte, en demi-precision (FP16).
#
# Un .engine n'est pas portable : il depend du GPU et de la version de TensorRT.
# Il faut donc le construire sur chaque robot, et ne pas le commiter.
#
# Usage :
#   ./camera/TensorRT/construire_engine.sh camera/IA/Model5/runs/detect/train/weights/best.onnx
#
# Duree : 5 a 15 minutes sur une Jetson Nano. Si le processus est tue ("Killed"),
# c'est un manque de memoire : fermer VS Code et le navigateur, puis relancer.
set -euo pipefail

if [ $# -ne 1 ]; then
    echo "Usage : $0 chemin/vers/modele.onnx" >&2
    exit 1
fi

ONNX="$1"
ENGINE="${ONNX%.onnx}.engine"
TRTEXEC="/usr/src/tensorrt/bin/trtexec"

[ -f "$ONNX" ] || { echo "ERREUR : $ONNX introuvable" >&2; exit 1; }
[ -x "$TRTEXEC" ] || { echo "ERREUR : trtexec introuvable ($TRTEXEC). TensorRT (JetPack) est-il installe ?" >&2; exit 1; }

CLASSES="${ONNX%.onnx}_classes.json"
[ -f "$CLASSES" ] || echo "/!\\  $CLASSES absent : les detections afficheront un numero de classe au lieu du nom."

# Memoire de travail autorisee pendant la construction : 1 Go.
# L'option a change de nom avec TensorRT 8.4 (--workspace -> --memPoolSize).
if "$TRTEXEC" --help 2>&1 | grep -q -- "--memPoolSize"; then
    OPTION_MEMOIRE="--memPoolSize=workspace:1024"
else
    OPTION_MEMOIRE="--workspace=1024"
fi

echo "==> Construction de $ENGINE (FP16), patience..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 "$OPTION_MEMOIRE"

echo
echo "==> Moteur pret : $ENGINE"
echo "    Le debit mesure par trtexec est affiche plus haut (ligne 'Throughput')."
