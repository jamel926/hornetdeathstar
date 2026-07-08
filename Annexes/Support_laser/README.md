# Support de Fixation Laser pour myCobot 280

Ce dépôt contient les fichiers de conception 3D et les instructions de fabrication du support d'interface mécanique pour le module laser de ciblage des frelons asiatiques.

## Logiciels Utilisés & Spécifications

* **Conception CAO :** FreeCAD v0.18 puis v1.0
    * *Ateliers principaux :* `PartDesign` (Modélisation paramétrique) et `TechDraw` (Mises en plan).
* **Tranchage (Slicing) :** ideamaker v5.2.4
    * *Format d'export intermédiaire :* `.STL` ou `.3MF`

---

## Conception Mécanique

La pièce a été conçue pour faire l'interface entre l'effecteur terminal du bras **myCobot 280 (tête M5ATOM)** et une diode laser cylindrique de diamètre **7.02mm**.

### Choix techniques :
* **Fixation Robot :** Empreinte de 3 trous traversants pour LEGO : 2780, alignés sur les entraxes d'origine du myCobot.
* **Maintien du Laser :** Système de serrage par une vis.
* **Contrainte de masse :** Charge du robot limitée à 250g.

---

## Guide de Fabrication (Impression 3D)

La pièce a été validée et fabriquée par dépôt de fil fondu (FDM).

### Matériel utilisé :
* **Imprimante 3D :** Raise3D Pro 3 Plus
* **Filament (Matériau) :** **PLA** de couleur noire

---

## Instructions d'Assemblage

1. Élargir les trous de perçage si nécessaire après l'impression.
2. Mettre le laser en place ainsi que la carte PCB associée.
3. Fixer le support laser sur la tête du myCobot à l'aide de 3 pièces LEGO : 2780.
4. Connecter les fils à la tête du bras robot. (GND, 3V3, G33)


## Historique des Versions

### [V3.0] - Version Actuelle (Juillet 2026)
* **Modifications :** Ajout d'un emplacement pour une carte PCB, perçage d'un trou pour faire sortir les fils sur le côté.
* **Justification :** Le laser nécessitait une carte PCB afin d'être correctement contrôlé par la tête du bras robot.

### [V2.0] - Version Intermédiaire (11 Juin 2026)
* **Modifications :** Ajout d'un trou de fixation, réduction de la taille du support, optimisation du profil de la gorge (pente adoucie) pour supprimer les points de friction critiques.
* **Justification :** La réduction du support permet de pouvoir accéder au bout du laser et régler la lentille du laser. La nouvelle gorge permet de mieux protéger le câblage

### [V1.0] - Prototype Initial (9 Juin 2026)
* Première version fonctionnelle avec fourreau lisse pour le laser et deux trous de fixation.
* *Problème identifié :* Manque de stabilité et pliage des câbles trop abrupte.