# abrascamera — la chaine camera vers bras

Le chainon manquant entre les deux poles : la camera detecte un frelon, ecrit sa position,
le bras va chercher la derniere position et pointe le laser dessus.

```
camera  ->  YOLO  ->  reperage  ->  positions_frelon.csv  ->  carreaux  ->  J4/J5  ->  bras
        camera_frelon.py                                    bras_frelon.py
```

Deux programmes separes qui tournent en parallele et ne se connaissent pas. Chacun se teste
tout seul, et le fichier au milieu se lit avec `tail -f` pendant que ca tourne.

> **LE LASER** — monter un **pointeur rouge de classe 2**, pas le laser de puissance.
> Les conditions d'inhibition du tir (etape E6) ne sont pas ecrites.

---

## Demarrage rapide

```bash
cd ~/02_Scolaire/ENSICAEN_3A/Projet_imersif

# La premiere fois : verifier que la conversion tient debout
.venv/bin/python abrascamera/reglage_repere.py --hors-ligne

# Puis la chaine complete
./abrascamera/lancer.sh --recalibrer
```

Au premier lancement, la camera demande un calibrage : les **quatre gommettes rouges**
doivent etre visibles, et **rien d'autre de rouge** dans le champ. Le resultat est
enregistre dans `calibrage_camera.json` et relu tout seul ensuite.

Le bras fait un **referencement d'environ 30 secondes** au demarrage — il bouge seul,
c'est normal, aucune action a faire.

---

## Les fichiers

| Fichier | Role |
|---|---|
| `pont.py` | le format d'echange, en un seul endroit. Ecriture et lecture du journal |
| `reperage.py` | les quatre coins rouges, et la conversion pixels vers pourcentage de grille |
| `camera_frelon.py` | **processus 1** : camera + YOLO, ecrit dans le journal |
| `bras_frelon.py` | **processus 2** : lit le journal, pointe le laser |
| `reglage_repere.py` | l'outil pour caler le repere camera sur le quadrillage |
| `lancer.sh` | demarre les deux ensemble, les arrete ensemble |

Chaque fichier s'explique en tete. Les pourquoi sont la-bas, pas ici.

---

## Le format d'echange

`positions_frelon.csv`, ouvert en ajout, un `flush()` par ligne.

```
horodatage,x_pct,y_pct,confiance
1789458231.482,43.7,61.2,0.91
```

| Champ | Unite |
|---|---|
| `horodatage` | `time.time()`, epoch **absolu** — c'est ce qui permet au bras de savoir si la cible est fraiche |
| `x_pct`, `y_pct` | 0 a 100, proportion de la grille. (0,0) = origine, (100,100) = coin oppose |
| `confiance` | 0 a 1, sortie du modele |

Au-dela de **1 seconde** sans nouvelle ligne, la cible est perdue : le bras s'immobilise et
eteint le pointeur.

Cote bras, un carreau vaut **3,0 cm** et la zone calibree fait 24 x 27 carreaux, soit
72 x 81 cm.

---

## Tester chaque etage separement

C'est tout l'interet des deux processus : quand ca ne marche pas, on sait ou regarder.

```bash
# 1. le pont, sans rien d'autre
.venv/bin/python abrascamera/pont.py

# 2. le reperage, sur une image de test
.venv/bin/python abrascamera/reperage.py

# 3. la conversion, sans camera ni robot
.venv/bin/python abrascamera/reglage_repere.py --hors-ligne

# 4. la camera seule
.venv/bin/python abrascamera/camera_frelon.py --afficher
tail -f abrascamera/positions_frelon.csv          # dans un autre terminal

# 5. le bras seul, sur une position ecrite a la main
printf 'horodatage,x_pct,y_pct,confiance\n%s,50,50,0.9\n' "$(date +%s)" \
    > abrascamera/positions_frelon.csv
.venv/bin/python abrascamera/bras_frelon.py
```

L'etape 5 doit faire tomber le point rouge **au centre du quadrillage**. Si ce n'est pas le
cas, c'est le repere qu'il faut regler.

---

## Regler le repere — a faire une fois

**Au premier essai, le laser pointera a cote. C'est normal.**

Personne n'a jamais mesure comment le repere de la camera est pose par rapport au
quadrillage du bras. `suivi_frelon.py` le dit deja ligne 820 : *« ca ne se devine pas : il
faudra le mesurer une fois, sur la zone reelle »*. C'est cette mesure.

Trois drapeaux en tete de `bras_frelon.py` :

```python
ECHANGER_XY = False    # la camera croise-t-elle les axes ?
INVERSER_U  = False    # le 0 % est-il du cote du carreau 0 ?
INVERSER_V  = False
```

La methode :

1. Lancer `camera_frelon.py` d'un cote, `reglage_repere.py` de l'autre.
2. Poser un leurre dans le coin que le bras appelle **(0, 0)**.
3. Lire ce qu'affiche `reglage_repere.py` :
   - environ (0, 0) carreaux : rien a faire ;
   - le coin oppose : lever les deux inversions ;
   - un coin adjacent : une seule.
4. Modifier **un** drapeau, relancer, verifier.
5. Recommencer avec le leurre au centre, puis dans un autre coin.

Un drapeau a la fois. Changer les trois ensemble et regarder si « ca marche mieux » est le
moyen le plus sur de tourner en rond.

---

## Pieges connus

**Un seul processus sur `/dev/ttyTHS1`.** Si le bras refuse de se connecter, un autre script
du projet tourne encore. Ne jamais importer `Bras/ModuleGPIO/PU_GPIO.py` : il ouvre le port
des l'import.

**Un cinquieme objet rouge dans le champ** devient un coin, et le reperage est faux **sans
aucun message**. Verifier le champ avant de calibrer.

**Le calibrage ne survit pas a un deplacement de la camera.** Si quelqu'un la bouge,
relancer avec `--recalibrer`.

**`COTE_CARREAU` vaut 3,0 cm.** `PU_Pointage.py` declare 2,5 : cette valeur est fausse et
ferait pointer 20 % a cote. Le fichier est archive, ne pas le ressortir.

---

## Ce qui vient de l'existant, et ce qui a ete recopie

Reutilises tels quels, par import :

- `LaCamera.read_latest()` — `camera/ModuleCam/ModuleCamera.py`
- `YoloDetector.detect()` — `camera/ReconnaissanceObjet/ModuleAI.py`
- tout le pole bras — `Bras/ModuleDeDeplacement/Calibration_v3/`

Recopie plutot qu'importe : la detection des quatre coins rouges, de
`camera/ModuleCam/Calibrage.py`. La fonction d'origine contient un `cv2.waitKey(0)`
**bloquant** en plein milieu (lignes 20 a 24), ce qui la rend inutilisable dans une boucle.
L'origine est citee en tete de `reperage.py`.

Modele YOLO charge : `camera/IA/Model5/runs/detect/train/weights/best.pt` — une classe
`frelon`, 150 epoques, mAP50 0,995. C'est le meilleur du depot.

---

## Deux choses a dire au pole image

**`CoordonneesAuRobo()` ne sort pas des centimetres.** Elle divise des pixels par des pixels
(`Calibrage.py` lignes 67 a 70) : le resultat est une fraction sans dimension. Les
parametres `D1`/`D2` ne sont utilises nulle part dans le corps de la fonction. Le
`"unit": "cm"` de `GUI.py` ligne 87 est donc faux, et le commentaire de `suivi_frelon.py`
qui parle de centimetres l'est aussi. Ce module travaille en pourcentage, une unite assumee.

**`GUI.py` ne demarre pas.** Trois bugs : l'import `CoordenéesAuRobo` (ligne 17) ne
correspond a aucune fonction — l'originale s'appelle `CoordonneesAuRobo` ; `CoordonnerCalibrage`
est appelee avec 2 arguments au lieu de 3 (ligne 457) ; et l'import du pole bras pointe un
chemin qui a change. Les tests de `tests/test_gui_services.py` **sautent silencieusement**
sur `ImportError`, donc le depot parait vert alors que le GUI ne se lance pas.
