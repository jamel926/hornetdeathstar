# Demo en reseau — la camera sur un PC, le bras sur la Jetson

Ce guide part de zero : cloner le depot sur un PC Linux, le brancher au myCobot, et faire
pointer le laser sur un leurre pose sur le quadrillage.

```
PC portable                                     myCobot 280 (Jetson Nano)
camera USB -> YOLO -> reperage  --- UDP --->    carreaux -> J4/J5 -> bras
   camera_frelon_reseau.py      cable reseau       bras_frelon_reseau.py
```

La detection (camera + YOLO) tourne sur le PC, la Jetson ne fait que bouger le bras. Seule la
**position de la cible** passe dans le cable, jamais une commande moteur : la calibration du
bras et son referencement restent sur la Jetson.

> **LE LASER** — monter un **pointeur rouge de classe 2**, jamais le laser de puissance.
> Les conditions d'inhibition du tir ne sont pas ecrites.

| Fichier | Machine | Role |
|---|---|---|
| `camera_frelon_reseau.py` | PC | camera + YOLO, controle du calibrage, envoi des positions |
| `bras_frelon_reseau.py` | Jetson | recoit les positions et pointe le laser |
| `pont_reseau.py` | les deux | le format d'echange UDP, et un outil pour tester le cable |

La version ou tout tourne sur la Jetson (`camera_frelon.py`, `bras_frelon.py`, `lancer.sh`,
voir `README.md`) reste utilisable et n'a pas ete modifiee.

---

## 0. Ce qu'il faut

| | |
|---|---|
| PC | Linux (teste sur Pop!_OS 24.04), Python **3.10 ou plus**, ~2 Go libres. Pas besoin de carte graphique NVIDIA |
| Camera | la camera USB du projet (nom Linux : « USB 2.0 Camera »), fixee au-dessus du quadrillage |
| Robot | le myCobot 280 Jetson Nano, avec le depot et son `.venv` deja installes (voir 3.) |
| Reseau | un **cable Ethernet** entre le PC et le robot (adaptateur USB-Ethernet si le PC n'a pas de prise) |
| Banc | le quadrillage 24 x 27 carreaux de 3 cm, les **4 gommettes rouges** aux coins, un leurre de frelon |

---

## 1. Sur le PC : cloner et installer

```bash
git clone git@gitlab.ecole.ensicaen.fr:denoual/hornetdeathstar.git
cd hornetdeathstar
git checkout camerabras
```

Le PC n'a besoin que de la partie camera : ni `pymycobot`, ni les GPIO. On installe PyTorch
**en version processeur** (quelques centaines de Mo au lieu de ~3 Go de bibliotheques CUDA
inutiles sans carte NVIDIA).

```bash
python3 -m venv .venv
.venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
.venv/bin/pip install ultralytics==8.3.203 numpy==2.2.6
```

**L'ordre compte** : installer `ultralytics` en premier lui fait telecharger PyTorch avec CUDA.

Si `python3 -m venv` echoue sur « ensurepip is not available » (paquet `python3-venv` absent,
et pas de droits sudo) :

```bash
python3 -m venv --without-pip .venv
curl -sSL https://bootstrap.pypa.io/get-pip.py | .venv/bin/python
```

Verifier :

```bash
.venv/bin/python -c "import torch, cv2, ultralytics; print(torch.__version__, 'CUDA:', torch.cuda.is_available())"
```

Les poids YOLO (`camera/IA/Model5/.../best.pt`) sont dans le depot, rien a telecharger.

---

## 2. Le cable entre le PC et le robot — une fois par PC

Personne ne distribue d'adresse sur un cable direct. C'est **le PC qui le fait**, en
connexion partagee : le robot reste sur son profil filaire par defaut (DHCP) et **n'a rien a
configurer**.

Brancher le cable, puis trouver le nom de l'interface filaire du PC :

```bash
nmcli device          # la ligne de type "ethernet", par exemple enp3s0 ou enxfc19...
```

Creer le profil (une seule fois) :

```bash
nmcli connection add type ethernet ifname <interface> con-name cobot \
    ipv4.method shared ipv4.addresses 10.10.0.1/24 ipv6.method disabled \
    connection.autoconnect no
```

`autoconnect no` : le profil ne s'active que quand on le demande, le PC se comporte
normalement sur les autres reseaux filaires. Le Wi-Fi peut rester actif.

**A chaque seance**, cable branche :

```bash
nmcli connection up cobot
ip neigh show dev <interface>      # adresse du robot, en general 10.10.0.103
ping 10.10.0.103
```

Le PC donne aussi acces a Internet au robot a travers lui (pratique pour un `git pull`).

Acces SSH au robot (utilisateur `er`, mot de passe du robot : le demander a l'equipe).
Installer sa cle une fois pour ne plus le taper :

```bash
ssh-copy-id er@10.10.0.103
```

---

## 3. Sur le robot : mettre le depot a jour

Le depot est deja clone sur le robot dans `~/Documents/hornetdeathstar`.

```bash
ssh er@10.10.0.103
cd ~/Documents/hornetdeathstar
git checkout camerabras
git pull
```

> Si `git pull` refuse parce que des fichiers `abrascamera/*_reseau.py` « non suivis seraient
> ecrases » : ce sont d'anciennes copies manuelles. Les supprimer
> (`rm abrascamera/*_reseau.py abrascamera/README_reseau.md`) puis relancer `git pull`.

Robot neuf, sans `.venv` : `./Environement/install_env.sh` (environnement de base, **sans**
`--avec-ia` : le robot ne fait pas tourner YOLO dans cette demo).

Verifier que le cable transporte bien les positions, **avant** de faire bouger le bras :

```bash
# sur le robot
.venv/bin/python abrascamera/pont_reseau.py --ecouter

# sur le PC, dans un autre terminal : 3 secondes de positions factices
.venv/bin/python -c "import sys, time; sys.path.insert(0, 'abrascamera'); import pont_reseau as p; e = p.EmetteurPositions(); [(e.envoyer(50, 50, 0.9), time.sleep(0.1)) for _ in range(30)]"
```

Le robot doit afficher des positions `50.00 % 50.00 %`. Ctrl+C pour arreter l'ecoute.

---

## 4. Calibrer la camera — a chaque fois que la camera ou le plateau bouge

La camera doit savoir ou est le quadrillage dans l'image. Elle le trouve grace aux **4
gommettes rouges** collees aux coins.

1. **Degager les 4 gommettes** : aucun leurre, aucune main dessus.
2. **Aucun autre objet rouge dans le champ.** Attention a la **piece rouge du robot**, sur le
   cote gauche de l'image : la cacher (papier, scotch blanc).
3. Sur le PC :

```bash
.venv/bin/python abrascamera/camera_frelon_reseau.py --afficher --recalibrer
```

Le programme enregistre `abrascamera/calibrage_camera.json` et une image de controle,
**`abrascamera/calibrage_camera.png`** : la vue de la camera avec les 4 coins trouves. Les 4
cercles verts doivent etre **sur les 4 gommettes**.

Il **refuse** un calibrage dont les coins ne forment pas un rectangle plausible, et dit
pourquoi :

```
CALIBRAGE SUSPECT : les quatre coins ne forment pas un rectangle plausible.
  - angle au coin 1 : 123 degres (attendu 90 +/- 20)
```

C'est presque toujours une gommette cachee remplacee par un autre objet rouge. Ouvrir l'image
de controle, retirer l'objet, relancer avec `--recalibrer`.

Le calibrage est relu tout seul aux lancements suivants.

---

## 5. Lancer la demo

**Terminal 1 — le robot**

```bash
ssh er@10.10.0.103
cd ~/Documents/hornetdeathstar
.venv/bin/python abrascamera/bras_frelon_reseau.py
```

1. `Zone degagee ? [O/n]` : verifier que rien ni personne n'est dans la zone du bras, et
   que c'est bien le **pointeur de classe 2** qui est monte. Repondre `O`.
2. **Referencement d'environ 30 secondes** : le bras bouge seul, c'est normal.
3. `Entree pour lancer le suivi` : appuyer sur Entree.

**Terminal 2 — le PC**

```bash
cd hornetdeathstar
.venv/bin/python abrascamera/camera_frelon_reseau.py --afficher
```

Une fenetre montre la camera avec les detections (cadre vert). Poser le leurre sur le
quadrillage : le point rouge doit tomber dessus.

**Arreter** : `q` dans la fenetre de la camera, **Ctrl+C** dans le terminal du robot (le
pointeur s'eteint). Si la camera s'arrete ou si le cable est debranche, le bras s'immobilise
et eteint le pointeur au bout d'une seconde.

A l'arret, la camera affiche sa cadence (images par seconde, millisecondes de YOLO par image).

---

## 6. Ce qu'on doit obtenir

Mesure au banc le 17/09/2026, leurre immobile, PC i7 13e generation sans GPU :

| | |
|---|---|
| Cadence camera | 7 a 9 images/s (YOLO ~80 ms par image) |
| Centre du quadrillage | le point **touche** le leurre |
| Bords | le point tombe a **0,5 a 1 carreau** (1,5 a 3 cm) |

Un frelon **en vol** sera vise avec un peu de retard : a ~8 images/s, la position recue a
jusqu'a 0,1 a 0,2 s.

---

## 7. Quand ca ne marche pas

**On ne sait pas si c'est la camera ou le bras** : les tester separement.

- *La camera seule* (bras eteint) : lancer la camera, poser le leurre **immobile** sur un
  carreau connu, et lire ou elle le place :
  ```bash
  .venv/bin/python -c "import sys, runpy; sys.path[:0] = ['abrascamera']; import bras_frelon, bras_frelon_reseau as r; [setattr(bras_frelon, k, v) for k, v in r.REPERE_RESEAU.items()]; sys.argv = ['reglage_repere.py']; runpy.run_path('abrascamera/reglage_repere.py', run_name='__main__')"
  ```
  (lance `reglage_repere.py` avec le repere de la version reseau). Le carreau affiche doit
  etre a moins d'un carreau du vrai. Sinon : recalibrer (4.).
- *Le bras seul* (camera arretee, bras en suivi) : envoyer une cible connue depuis le PC.
  Pour viser le carreau (u, v), envoyer `x = v / 27 * 100` et `y = u / 24 * 100` :
  ```bash
  .venv/bin/python -c "import sys, time; sys.path.insert(0, 'abrascamera'); import pont_reseau as p; e = p.EmetteurPositions(); [(e.envoyer(50, 50, 0.9), time.sleep(0.1)) for _ in range(150)]"
  ```
  (ici le centre, 12 ; 13,5, pendant 15 s). Le point doit tomber a environ un carreau pres.

| Symptome | Cause probable |
|---|---|
| `Camera "USB 2.0 Camera" introuvable` | camera debranchee. Le programme refuse volontairement d'ouvrir une autre camera ; `--camera N` pour en forcer une |
| `CALIBRAGE SUSPECT` | gommette cachee, autre objet rouge pris pour un coin (4.) |
| Le point rate de plusieurs carreaux, surtout vers un bord | calibrage faux : regarder `calibrage_camera.png`, recalibrer |
| Le point part en miroir ou sur le mauvais axe | camera tournee depuis le 17/09 : reperer avec `--sans-echange-xy`, `--inverser-u`, `--inverser-v` (une option a la fois), puis reporter le bon reglage dans `REPERE_RESEAU` en tete de `bras_frelon_reseau.py` |
| Le robot ne recoit rien (`pont_reseau.py --ecouter` muet) | `nmcli connection up cobot` oublie, adresse du robot differente (`--ip-bras` cote PC), pare-feu du robot (`sudo ufw allow 5005/udp`) |
| `Impossible d'ecouter le port UDP 5005` | un autre `bras_frelon_reseau.py` tourne encore sur le robot |
| Le bras refuse de se connecter | un autre script du projet tient `/dev/ttyTHS1` |
| Beaucoup de « detections hors grille » dans le bilan | calibrage faux ou camera deplacee |

---

## Comment ca marche, en bref

- **Format** : un datagramme UDP par detection, `x_pct,y_pct,confiance` en texte. `x` et `y`
  vont de 0 a 100 % du quadrillage.
- **UDP plutot que TCP** : le bras n'a besoin que de la derniere position ; une position
  perdue est remplacee 0,1 s plus tard, et TCP retarderait les suivantes pour la renvoyer.
- **Age des positions mesure par le robot**, a la reception : les horloges du PC et du robot
  ne sont pas synchronisees. Au-dela d'1 seconde sans position, la cible est perdue.
- **Repere** : la camera croise les axes par rapport au bras (x camera -> v bras, y camera ->
  u bras). C'est `REPERE_RESEAU` dans `bras_frelon_reseau.py`, mesure au banc.
- Le detail est en tete de chaque fichier.
