# Carte d'Interface Électronique - Contrôle Laser

Ce dépôt contient la documentation et le schéma de câblage de la carte d'interface électronique permettant de piloter le module laser de ciblage à partir des broches GPIO du bras robot **myCobot 280**.

## Spécifications Électriques & Logiques

Le but principal de cette carte est de permettre l'allumage et l'extinction logicielle du laser en utilisant les niveaux logiques du microcontrôleur du robot.

* **Alimentation & Signal :** $3.3\text{ V}$ (fourni par la tête M5ATOM du myCobot)
* **Contrôle :** Broche GPIO `G33`
* **Consommation Laser :** < 40 mA

---

## Évolution de la Conception (Optimisation)

Initialement, le module laser exigeait une tension nominale de $3\text{ V}$. La première version de la carte intégrait donc un composant de régulation de tension pour abaisser le $3.3\text{ V}$ du robot en $3\text{ V}$.

> **Optimisation V2 :** Après tests, il a été validé que la diode laser fonctionnait parfaitement et de manière stable directement sous $3.3\text{ V}$. Le circuit a donc été simplifié en supprimant l'étage de régulation inutile. Le laser est désormais piloté directement par la GPIO, ce qui réduit la taille du PCB, le coût et le risque de panne.

---

## Schéma de Connexion (Brochage)

Le raccordement entre la tête M5ATOM du myCobot 280, la carte PCB et le laser suit le schéma suivant :

| Broche myCobot | Fonction | Connexion sur le PCB | Destination Finale |
| :--- | :--- | :--- | :--- |
| **3V3** | Alimentation positive | Entrée VCC | Pôle (+) du Laser |
| **GND** | Masse / Terre | Entrée Masse | Pôle (-) du Laser (via transistor) |
| **G33** | Commande GPIO | Entrée Signal / Target | Commande de l'interrupteur électronique |

*Note : La broche G33 agit comme un interrupteur. Lorsqu'elle est à l'état HAUT (`HIGH`), le circuit se ferme et le laser s'allume. À l'état BAS (`LOW`), le laser s'éteint.*

---

## Instructions de Montage & Soudure

1. Souder les éléments nécessaires au bon fonctionnement de la carte
2. Insérer la carte PCB dans le logement dédié du **Support de Fixation Laser V3** (voir dossier `Support_laser`).
3. Vérifier l'absence de court-circuit au multimètre avant la mise sous tension du bras robot.

---

## Historique des Versions (Évolution du Design)

### Version 2 : Connexion Directe GPIO $3.3\text{ V}$ (Actuelle)
* **Objectif :** Simplifier le schéma électrique et réduire l'encombrement.
* **Description :** Après des tests, il a été validé que la diode laser accepte parfaitement le $3.3\text{ V}$ en direct de manière stable et sans surchauffe. 
* **Amélioration :** Suppression complète de l'étage de régulation devenu inutile. Le laser est désormais piloté en direct via un simple interrupteur électronique (transistor) connecté à la GPIO.
* **Bilan :** Réduction des coûts, gain de place crucial pour l'intégration dans le support imprimé en 3D, et diminution du risque de panne.

### Version 1 : Prototype avec Régulateur de Tension (Initiale)
* **Objectif :** Adapter la tension pour protéger le laser.
* **Description :** Le module laser fonctionnant initialement en $3\text{ V}$, le circuit intégrait un régulateur de tension pour abaisser le $3.3\text{ V}$ délivré par la GPIO du myCobot.
* **Problématique :** Circuit plus complexe, encombrement plus important sur le PCB et composants supplémentaires à souder.