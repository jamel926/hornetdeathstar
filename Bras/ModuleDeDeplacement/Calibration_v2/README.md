# Calibration v2

Calibration du pointage reajustee, et les deux scripts pour la tester sur le bras.

**Aucun fichier existant n'est modifie.** Tout est dans ce dossier. Si le v2 ne
convient pas, supprimer le dossier suffit : le projet retrouve son etat d'avant.

---

## Le probleme

Le modele actuel (`../resultats/modele_grille.json`) est ajuste sur les **35 noeuds**
de Calib 2 uniquement. Il colle a ces 35 points, mais sur les 67 tirs deja annotes a la
main il annonce le spot **0,45 carreau trop haut en y**. C'est exactement le
« **-1 carreau en Y** » note dans `apprentissage_erreurs.csv`.

Le polynome de correction de degre 4 (`modele_correction_Calib_2.json`) rattrapait ce
biais apres coup. 30 coefficients pour 41 points dont les erreurs valent 0 ou +/-1 entier :
il ajuste surtout l'arrondi de lecture.

## Ce qui a ete fait

Les lignes de `validation_pointage_Calib_2.csv` sont **des mesures de calibration a part
entiere**. La colonne J4/J5 n'est pas la consigne : `correction_calib_mc_mtm.py` y ecrit
`mesure[3], mesure[4]`, la pose **relue sur le robot**. Chaque ligne dit donc « poignet a
ces angles, spot sur ce carreau » -- la meme information qu'une ligne de
`calibration_grille.csv`, obtenue dans l'autre sens. Et elle reste valable quelle que soit
la correction appliquee ce jour la : la correction change la cible, pas le couple
(angles, impact).

La calibration passe donc de **35 a 112 points**, et le biais disparait.

| Modele | Biais global | Ecart entre seances | Residu interne | **Erreur attendue** |
|---|---|---|---|---|
| Actuel (35 noeuds) | 0,84 cm | 0,78 cm | 1,33 cm | **1,75 cm** |
| v2, homographie seule | 0 | 0,75 cm | 1,19 cm | **1,40 cm** |
| v2 + residu degre 2 | 0 | 0,73 cm | 1,05 cm | **1,28 cm** |

## Trois choses a savoir avant de tester

**1. Le degre 4 ne sert a rien.** En validation croisee (5 plis x 20 repetitions) :
degre 1 -> 1,46 cm, degre 2 -> 1,35 cm, degres 3 et 4 -> plus rien. Le v2 garde le
degre 2, qui ne rapporte que 0,12 cm. Le gain est dans le reajustement, pas dans la
correction plaquee par dessus.

**2. Les annotations au carreau entier ont atteint leur plancher.** Un impact note au
carreau entier porte un arrondi de **0,41 carreau rms (1,22 cm)**. Apres reajustement,
la dispersion qui reste sur les 67 tirs vaut 0,38 carreau. Il ne reste **rien de
mesurable**. Annoter 100 tirs de plus au carreau n'ameliorera pas la calibration --
c'est pour ca que `test_calibrage_v2.py` demande le **demi-carreau** et previent si on
l'oublie.

**3. La pose de base s'etale sur 0,88 degre en J2** pendant la campagne (J1 0,44, J3 0,70).
J1 a J3 ne sont jamais commandes : le bras est cense les retrouver en position initiale,
il ne les retrouve pas. C'est le suspect n°1 pour les **0,75 cm d'ecart entre seances**,
le seul terme qu'aucun reajustement n'enlevera. Au passage, le champ `base` du modele
d'origine (`0.61, 50.53, -144.93`) ne correspond a **aucune** pose relevee pendant
Calib 2 (moyenne reelle `0.78, 51.07, -145.48`) ; le v2 le corrige.

---

## Les fichiers

| Fichier | Role |
|---|---|
| `modele_v2.py` | Noyau commun : chargement, evaluation, inversion du residu, verification de la base. Les trois autres l'importent, personne ne recopie ces formules. |
| `recalibration_tirs.py` | Refait l'ajustement et regenere le modele + la grille. Tourne **sans robot**. |
| `test_calibrage_v2.py` | Campagne de tirs sur le bras, pour mesurer ce que vaut le v2 et produire de nouvelles donnees. |
| `suivi_frelon_v2.py` | Le suivi de `suivi_frelon.py`, inchange, mais avec la chaine de visee du v2. |
| `resultats/modele_grille_v2.json` | Le modele. **Meme format** que `modele_grille.json`. |
| `resultats/grille_angles_v2.csv` | La grille : les 700 noeuds (x 0-24, y 0-27) avec leurs J4/J5 et l'ecart avec l'ancien modele. |

---

## Faire les tests

### 1. Verifier que le modele tient (sans robot, 2 secondes)

```bash
cd Bras/ModuleDeDeplacement/Calibration_v2
python3 recalibration_tirs.py --diagnostic
```

Affiche la comparaison des trois modeles sans rien ecrire.

### 2. Tester le pointage sur le bras

```bash
python3 test_calibrage_v2.py
```

Mode **GRILLE** recommande : 21 noeuds repartis sur toute la zone, bords et coins compris.
Une erreur mesuree seulement au centre ne dit rien des bords.

Pour chaque tir : le bras vise, on note ou le spot est tombe **au demi-carreau au moins**.
Le bilan (biais, dispersion) s'affiche apres chaque tir.

> **A lire pendant la seance** : le *biais* dit que le modele est decale, ca se rattrape.
> La *dispersion* dit que le bras ne repete pas, ca ne se rattrape pas par du calcul.
> Si la seance sort nettement au dessus de **1,3 cm rms**, ce n'est pas le modele : la base
> a bouge, le quadrillage a ete deplace, ou le pointeur n'est plus au meme endroit.

La seance ecrit `resultats/tirs_v2_<horodatage>.csv`.

### 3. Reinjecter la seance dans le modele

```bash
python3 recalibration_tirs.py
```

Relit tout seul les `tirs_v2_*.csv` du dossier et les ajoute a l'ajustement. C'est la
boucle : **tirer, annoter, reajuster, recommencer.**

### 4. Tester le suivi de frelon

```bash
python3 suivi_frelon_v2.py
```

Memes questions, memes modes, memes sorties que `suivi_frelon.py` -- elles atterrissent
dans `Calibration_v2/resultats/` pour ne pas se melanger aux anciennes. Comparer les
colonnes `erreur_carreaux` des deux CSV.

> **Ne pas attendre de miracle en mode flux.** Un modele plus juste enleve l'erreur de
> **visee**, pas celle de **commande**. En flux, la pose est lue en vol et le jeu des
> reducteurs n'est plus repris du meme cote : le gros de l'ecart restera. Le gain se verra
> surtout en mode enchaine.

---

## Passer le v2 en production

Une fois les tests concluants :

```bash
cp resultats/modele_grille_v2.json ../resultats/modele_grille.json
```

Le format est identique, tout le code existant le relit sans modification et gagne le
reajustement (1,40 cm). Le bloc `residu_quadratique` en plus est **ignore** par
`calibration_grille.py` : pour les 1,28 cm il faut passer par `predireImpactCorrige()` et
`anglesPourPointCorrige()` de `modele_v2.py`.

Penser aussi a **retirer le polynome de correction** des scripts qui l'appliquent
(`correction_calib_mc_mtm.py`, `suivi_frelon.py`, `calib2_corrigee_alea.py`) : avec le v2
il corrigerait deux fois.

## Un point a trancher

L'enveloppe du v2 descend a `v_min = -1` parce qu'un tir annote a ete lu a y = -1. Ca
autorise un carreau de plus vers le bas. Si ca gene, remonter la borne a la main dans
`resultats/modele_grille_v2.json`.
