#!/usr/bin/env bash
#
# Lance la chaine complete : camera d'un cote, bras de
# l'autre, les deux en parallele.
#
# Les deux programmes communiquent par positions_frelon.csv
# et ne se connaissent pas autrement. Ce script ne fait que
# les demarrer ensemble et les arreter ensemble.
#
# Usage :
#     ./abrascamera/lancer.sh
#     ./abrascamera/lancer.sh --recalibrer
#
# Toute option donnee ici est passee a camera_frelon.py.

set -u

DOSSIER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RACINE="$(dirname "$DOSSIER")"
PYTHON="$RACINE/.venv/bin/python"

JOURNAL="$DOSSIER/positions_frelon.csv"

if [ ! -x "$PYTHON" ]; then
    echo "Environnement introuvable : $PYTHON"
    echo "Lancer d'abord Environement/install_env.sh"
    exit 1
fi

# On archive le journal de la seance precedente plutot que
# de le tronquer : les deux programmes ouvrent le fichier en
# ajout, et une seance passee peut toujours servir a rejouer
# une trajectoire.
if [ -s "$JOURNAL" ]; then
    ARCHIVE="$DOSSIER/journaux/positions_$(date +%Y%m%d_%H%M%S).csv"
    mkdir -p "$DOSSIER/journaux"
    mv "$JOURNAL" "$ARCHIVE"
    echo "Journal precedent archive : $ARCHIVE"
fi

CAMERA_PID=""

# Le bras doit eteindre son pointeur avant de mourir : on lui
# envoie un SIGINT, qu'il attrape, et on lui laisse le temps
# de passer dans son finally.
arreter() {
    echo
    echo "Arret de la chaine..."

    if [ -n "$CAMERA_PID" ] && kill -0 "$CAMERA_PID" 2>/dev/null; then
        kill -INT "$CAMERA_PID" 2>/dev/null
    fi

    wait 2>/dev/null
    echo "Termine."
}

trap arreter INT TERM

echo "==========================================="
echo " CHAINE CAMERA -> BRAS"
echo "==========================================="
echo
echo "MONTER UN POINTEUR ROUGE DE CLASSE 2."
echo "Le laser de puissance reste debranche."
echo

"$PYTHON" "$DOSSIER/camera_frelon.py" "$@" &
CAMERA_PID=$!

# Laisser la camera calibrer et charger YOLO avant de
# demarrer le bras : sinon le bras fait son referencement de
# 30 secondes pendant que la camera attend encore.
sleep 3

if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
    echo
    echo "La camera s'est arretee au demarrage. Le bras n'est pas lance."
    echo "Relancer camera_frelon.py seul pour voir le message."
    exit 1
fi

# Le bras tourne au premier plan : c'est lui qui demande une
# confirmation avant de bouger, et lui qu'on veut voir.
"$PYTHON" "$DOSSIER/bras_frelon.py"

arreter
