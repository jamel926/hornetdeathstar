"""
export_onnx.py -- ETAPE 1, a lancer sur un PC (PAS sur la Jetson).

Convertit les poids YOLO (.pt) en ONNX, format que TensorRT sait lire, et ecrit
a cote la liste des classes (le moteur TensorRT ne la conserve pas).

Prerequis sur le PC : pip install "ultralytics>=8.4" onnx
(8.4 minimum car le modele est un YOLO26 ; la version CPU de PyTorch suffit.)

Utilisation (depuis la racine du projet) :
    python camera/TensorRT/export_onnx.py camera/IA/Model5/runs/detect/train/weights/best.pt

Produit, a cote du .pt :
    best.onnx           -> a copier sur la Jetson
    best_classes.json   -> a copier sur la Jetson, dans le meme dossier que le .onnx

Options :
    --imgsz 480 640   taille d'entree du reseau : hauteur largeur (defaut), ou une seule valeur pour un carre.
                      480 640 = format de la camera : pas de bandes grises, 25 % de calcul en moins qu'en 640x640.
    --opset 12        version ONNX. TensorRT 8.2 (JetPack 4.6) accepte au plus 13.
    --sans-end2end    exporte la sortie classique (NMS fait par le client) si TensorRT refuse la tete end2end
"""

import argparse
import json
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="Export YOLO .pt -> ONNX pour TensorRT (Jetson Nano).")
    parser.add_argument("poids", help="fichier .pt entraine")
    parser.add_argument("--imgsz", type=int, nargs="+", default=[480, 640], help="hauteur largeur, ou une valeur pour un carre")
    parser.add_argument("--opset", type=int, default=12)
    parser.add_argument("--sans-end2end", action="store_true", help="sortie classique, NMS cote client")
    args = parser.parse_args()

    poids = Path(args.poids).resolve()
    modele = YOLO(str(poids))

    imgsz = args.imgsz[0] if len(args.imgsz) == 1 else args.imgsz
    options = dict(format="onnx", imgsz=imgsz, opset=args.opset, simplify=True, dynamic=False, half=False, batch=1)
    if args.sans_end2end:
        options["end2end"] = False
    cheminOnnx = Path(modele.export(**options))

    # Les noms de classes ne passent pas dans le .engine : on les sauve a cote.
    cheminClasses = cheminOnnx.with_name(cheminOnnx.stem + "_classes.json")
    with open(cheminClasses, "w", encoding="utf-8") as fichier:
        json.dump({str(k): v for k, v in modele.names.items()}, fichier, ensure_ascii=False, indent=2)

    # Affichage de la forme de sortie : (1, 300, 6) = end2end, (1, 4+nb_classes, N) = classique.
    try:
        import onnx

        graphe = onnx.load(str(cheminOnnx)).graph
        for sortie in graphe.output:
            dims = [d.dim_value for d in sortie.type.tensor_type.shape.dim]
            print(f"Sortie ONNX {sortie.name} : {dims}")
    except ImportError:
        pass

    print(f"\nA copier sur la Jetson, dans le meme dossier :\n  {cheminOnnx}\n  {cheminClasses}")


if __name__ == "__main__":
    main()
