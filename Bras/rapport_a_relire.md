# Rapport de semaine — pole bras : de la calibration au pointage du frelon

> **BROUILLON A RELIRE.** Redige a partir de l'historique git, des fichiers de resultats et
> des mesures faites au banc. Les passages `[A COMPLETER]` et `[A VERIFIER]` demandent une
> information que les fichiers ne donnent pas. Semaine du **jeudi 10 au jeudi 17 septembre
> 2026**. [A VERIFIER : bornes de la semaine a couvrir]

---

## 1. Objectif de la semaine

Le systeme Hornet Death Star doit detecter un frelon au-dessus d'une ruche et le viser avec un
laser porte par un bras myCobot 280 (Jetson Nano). En debut de semaine, le pole bras savait
amener le bras dans une pose initiale, mais **ne savait pas viser un point precis** du plan de
la ruche, et rien ne reliait la camera au bras.

Objectif : **qu'un leurre pose sur la zone soit vise par le pointeur**, de bout en bout.

Choix de depart, decide le 07/09 et conserve toute la semaine : **base figee**. Le bras se met
en pose initiale, J1 a J3 ne bougent plus, et seuls **J4 et J5** orientent le laser, comme une
tourelle. Il ne reste que deux angles a calculer.

Toute la semaine, les essais se sont faits avec un **pointeur rouge de classe 2**, jamais avec
le laser de puissance.

Le banc d'essai : un quadrillage de **24 x 27 carreaux de 3 cm** (72 x 81 cm) pose sous le
bras, qui sert a la fois de cible et d'unite de mesure.

---

## 2. Deroulement de la semaine

### Jeudi 10 et vendredi 11 — un modele de visee

Le modele de pointage existant (`PU_Pointage.py`, bilineaire) etait donne par sa propre
documentation a **3,8 cm d'erreur au pire** sur 30 degres, alors que couvrir la zone demande
51 degres. Densifier les points n'y changeait rien : c'est la forme du modele qui limitait.

Constat qui a guide la suite : le faisceau part d'un point fixe et frappe un plan. La relation
entre la direction du laser (J4, J5) et le point d'impact est donc **exactement une
homographie** (une transformation projective 3x3). Neuf parametres, un ajustement lineaire,
une inversion analytique, sans numpy ni scipy (non garantis sur la Jetson).

- `calibration_grille.py` : on amene le pointeur a la main sur des noeuds du quadrillage, le
  script releve les angles et ajuste le modele.
- `calibration_test.py` : le bras vise un point demande et on mesure l'ecart.
- Le bras approche toujours sa cible **du meme cote**, pour rattraper le jeu des reducteurs.
- 23 tests unitaires, sans robot.

Campagnes de releve les 10 et 11/09 (20 puis 35 points).

### Lundi 14 — deux defauts caches, et le suivi de trajectoire

**Defaut 1 : les unites.** L'entete du CSV de calibration n'etait ecrit qu'a sa creation. La
campagne du 11/09, saisie en carreaux, s'est ajoutee sous des colonnes restees en centimetres,
et la relecture a tout divise par trois **sans message**. Consequence : une zone calibree
annoncee a 8 x 9 carreaux au lieu de 24 x 27, deux tiers des points refuses, et des erreurs
qui paraissaient trois fois plus petites qu'en realite. Corrige : CSV converti, et le script
refuse maintenant d'ecrire sous un entete different.

Apres correction : **erreur du modele 0,31 carreau (0,94 cm)**, la meme a 20 et a 35 points.
Ajouter des points ne gagnait donc plus rien.

**Defaut 2 : l'ecart de repos.** Le bras s'arrete **sous sa consigne** : 0,7 a 1,8 degre de
deficit sur J4 et J5, soit 1,5 a 2,3 cm au sol, plus que l'erreur du modele. La tolerance du
firmware (2 degres) le declarait pourtant « arrive ». `viserAngles()` relit maintenant la pose
atteinte et rallonge la consigne tant que l'angle n'est pas tenu.

**Apprentissage de la correction** (`apprentissage_correction.py`) : le robot vise lui-meme
les carreaux, l'operateur saisit l'impact reel, et le script apprend une correction. Garde-fou
principal : aucune correction n'est adoptee si elle ne bat pas nettement l'absence de
correction en validation croisee. Sinon, le script dit que l'erreur restante est de la
dispersion, donc pas apprenable.

**Suivi de trajectoire** (`suivi_frelon.py`) : enchainer une liste de positions pour simuler
le vol d'un frelon, en mode *enchaine* (on attend chaque point) ou *flux* (cadence fixe, sans
attendre, le mode destine a la camera).

[A COMPLETER : travail des autres membres de l'equipe le 14/09 — l'historique montre des
cartes d'erreur 3D (Maelle Chevennement), un nouveau modele entraine (Jose Alves), des
mises a jour du main et de la prediction (Enzo Musseau), extraction de donnees (M. Diallo)]

### Mardi 15 — calibration v2 puis v3 : l'erreur divisee par cinq

**Calibration v2.** Les tirs annotes pendant les validations precedentes contiennent les
angles **relus sur le robot**, pas la consigne : ce sont donc des mesures de calibration
comme les autres. En les ajoutant, on passe de 35 a 110 points d'ajustement, et le biais en y
que corrigeait un polynome de degre 4 disparait. Le polynome est retire.

Seance de test v2 (25 tirs) : **2,53 cm rms**, avec un biais de -0,66 carreau en y. Moins bien
qu'attendu.

**Le diagnostic.** Depuis les angles **reellement atteints**, le modele retrouve l'impact a
1,19 cm : la geometrie n'etait pas en cause. L'ecart venait encore de la commande : le bras
s'arretait **1,25 degre sous sa consigne sur J4** et 0,65 sur J5. Le polynome retire en v2
compensait en fait aussi ce manque, sans que ce soit ecrit nulle part.

**Calibration v3.** Au demarrage de chaque seance, le bras vise 8 points de reference, mesure
son propre manque sur ses encodeurs (sans aucune saisie) et decale ses consignes en
consequence. On mesure a chaque seance plutot que de figer une constante, parce que ce manque
derive : de 0,66 a 1,25 degre en deux jours.

Seance de test v3 (21 tirs, lus au demi-carreau) : **0,46 cm rms (0,15 carreau), 1,5 cm au
pire, biais nul.**

| Seance du 15/09 | Tirs | Erreur rms | Pire tir | Biais y |
|---|---|---|---|---|
| Calibration v2 | 25 | 2,53 cm | 4,5 cm | -0,66 carreau |
| **Calibration v3** | 21 | **0,46 cm** | **1,5 cm** | 0 |

> A nuancer : les impacts sont lus au demi-carreau (1,5 cm). Une erreur rms de 0,46 cm est
> donc au niveau de la precision de lecture : le v3 est au moins aussi bon que ce qu'on sait
> mesurer, sans qu'on puisse dire de combien.

Suivi de trajectoire avec le v3 : erreur moyenne de **0,5 carreau** (1,5 cm) sur un cercle de
50 points et sur une trajectoire de frelon de 10 points, environ 0,5 s par point.

### Mercredi 16 — rangement, premiere chaine camera -> bras, TensorRT

**Rangement du module bras.** Deux dossiers autonomes : `Calibration_v3/` (tout ce qu'il faut
pour calibrer et suivre) et `Version_Finale/` (pose initiale et visee). Le reste est deplace
dans `Bras/_archive/`, rien n'est supprime. Au passage : les imports de `Version_Finale`
etaient casses, et un marqueur de conflit git (`<<<<<<< HEAD`) empechait
`test_calibrage_v3.py` de compiler depuis un merge.

Nouvelle seance de suivi en cercle : erreur moyenne **0,39 carreau** (40 points).

**Premiere chaine camera -> bras** (`abrascamera/`). Deux programmes qui tournent en
parallele sur la Jetson et communiquent par un fichier CSV :
- `camera_frelon.py` : camera, YOLO, conversion en position sur la grille, ecriture ;
- `bras_frelon.py` : lecture de la derniere position, conversion en angles, visee.

L'unite d'echange est le **pourcentage de la grille** (0 a 100 sur chaque axe), avec un
horodatage absolu. Point releve sur le code du pole image : `CoordonneesAuRobo()` ne sort pas
des centimetres mais une fraction sans dimension (pixels divises par pixels).

**YOLO sur la Jetson.** Le PyTorch installe par pip plante sur la Nano (« Illegal
instruction ») : il ne peut utiliser ni le GPU (pilote CUDA 10.2) ni le processeur ARMv8.0.
La detection passe donc par **TensorRT** : export ONNX sur PC, moteur construit sur le robot,
et un petit serveur local, parce que TensorRT n'existe que pour le Python du systeme.
[A COMPLETER : le moteur TensorRT a-t-il tourne, et a quelle cadence ?]

### Jeudi 17 — la camera sur un PC, et la chasse a l'erreur

**Deporter la detection sur un PC.** Le PC portable fait tourner la camera et YOLO, la Jetson
ne fait plus que bouger le bras. Les deux sont relies par un **cable Ethernet direct**. Seule
la **position de la cible** passe dans le cable, jamais une commande moteur : le referencement
et la boucle fermee sur J4/J5 restent sur la Jetson, sans latence reseau.

Sur ce cable, les positions passent en **UDP** et non en TCP : le bras n'a besoin que de la
derniere position, et TCP retarderait les suivantes pour renvoyer une position perdue, deja
perimee. L'age d'une position est mesure par la Jetson **a la reception**, parce que les
horloges des deux machines ne sont pas synchronisees.

Premier essai de la chaine complete : **le pointeur suivait, mais ratait le leurre de
plusieurs carreaux**, et visait en miroir.

- **Le miroir** : la camera croise les axes par rapport au bras (x camera = v bras). Regle en
  un essai.
- **Les carreaux manques** : on a separe les deux moities de la chaine pour savoir laquelle se
  trompait.

**Test A, camera seule** (bras eteint, leurre immobile) : au centre, la camera placait le
leurre a **4 carreaux** de sa position reelle, presque juste pres d'un coin, et le rejetait
hors grille au coin oppose. Une erreur tordue, pas un simple decalage : la signature d'un coin
de calibrage faux.

**La cause** : la camera repere le quadrillage grace a 4 gommettes rouges. Le leurre en
cachait une, et une **piece rouge du robot**, au bord de l'image, a ete prise a sa place.
Le calibrage l'a accepte **sans aucun message**. Piece cachee, gommettes degagees, nouveau
calibrage :

| Leurre (carreaux) | Vu avant | Vu apres | Ecart apres |
|---|---|---|---|
| centre (12 ; 14) | (15,9 ; 18,5) | (12,1 ; 14,2) | 0,2 carreau |
| (23 ; 0) | rejete hors grille | (23,3 ; 0,8) | 0,8 carreau |
| (23 ; 26) | — | (23,3 ; 26,4) | 0,4 carreau |

**Test B, bras seul** (camera arretee, cibles envoyees a la main) :

| Cible (carreaux) | Point laser | Ecart |
|---|---|---|
| (12 ; 14) | (12 ; 14) | 0 |
| (1 ; 14) | (1 ; 14) | 0 |
| (23 ; 0) | (22 ; 0) | 1 carreau |
| (23 ; 26) | (22 ; 25,5) | 1 carreau |
| (1 ; 26) | (0 ; 25,5) | 1 carreau |

Juste sur la ligne du milieu, environ un carreau pres des bords. Ces cinq points ont ete lus
au carreau entier : l'ecart est au niveau de la precision de lecture, et deux explications
mecaniques proposees en cours de test ont ete dementies par le point suivant. **On a donc
choisi de ne pas corriger** : ajuster une correction sur du bruit degrade le pointage sans
que ca se voie.

**Chaine complete, leurre immobile** :

| Leurre (carreaux) | Point laser | Resultat |
|---|---|---|
| centre (12 ; 14) | sur le leurre | **touche** |
| (23 ; 0) | (23 ; 1) | 1 carreau (3 cm) |
| (1 ; 26) | (1 ; 25,5) | 1/2 carreau (1,5 cm) |

Deux garde-fous ajoutes a la suite :
- le calibrage camera est **refuse** si les 4 coins ne forment pas un rectangle plausible, et
  une image montrant les coins trouves est enregistree a chaque lancement ;
- si la camera USB est absente, le programme **s'arrete** au lieu d'ouvrir une autre camera
  (il ouvrait la webcam du PC sans prevenir).

Un guide `abrascamera/README_reseau.md` explique comment cloner le depot et relancer la demo.

---

## 3. Comment ca marche

### La chaine complete

```
 PC portable                                          Jetson Nano (myCobot 280)
 ------------                                         -------------------------
 camera USB                                           reception UDP
    |                                                    |  position la plus recente,
 YOLO (Model5, une classe "frelon")                      |  perimee apres 1 s
    |  centre de la boite, en pixels                     |
 reperage : 4 gommettes rouges                        conversion en carreaux (u, v)
    |  pixels -> % de la grille (homographie)            |  echange des axes x/y
    |                                                    |
 envoi UDP  "x_pct,y_pct,confiance"  --- cable --->   modele de visee v3
                                                         |  carreaux -> angles J4/J5
                                                         |  + decalage mesure au demarrage
                                                         |
                                                      send_angles(J1-J3 figes, J4, J5, J6)
                                                         |
                                                      pointeur rouge
```

### Etape par etape

**1. Detection.** YOLO (modele `Model5`, 150 epoques, une classe) donne une boite autour du
frelon. On garde la plus sure de l'image et son centre en pixels. Environ 80 ms par image sur
le processeur du PC, soit **7 a 9 images par seconde**.

**2. De l'image a la grille.** Les 4 gommettes rouges aux coins du quadrillage donnent 4
correspondances entre pixels et coins de la grille. On en deduit une homographie, qui corrige
la perspective de la camera. Chaque detection devient une position en **pourcentage de la
grille**, independante de la resolution et de la position de la camera.

**3. Transport.** Un datagramme UDP par detection, en texte. Si le cable est debranche ou si
la camera s'arrete, plus rien n'arrive : au bout d'une seconde, le bras s'immobilise et eteint
le pointeur.

**4. De la grille aux carreaux du bras.** Le pourcentage est converti en carreaux (0 a 24 en
u, 0 a 27 en v), avec l'echange d'axes mesure au banc.

**5. Des carreaux aux angles.** Le modele de visee est lui aussi une homographie, dans l'autre
sens : il relie les angles J4/J5 au point d'impact sur le plan. Elle est ajustee sur 110
points (45 noeuds pointes a la main et 65 tirs annotes), avec une petite correction de degre 2
par-dessus. On l'inverse pour obtenir les angles qui visent un carreau donne.

**6. La compensation v3.** Au demarrage, le bras vise 8 points de reference et mesure de
combien il s'arrete sous ses consignes. Ce decalage est ajoute a toutes les consignes de la
seance.

**7. Le mouvement.** Seuls J4 et J5 changent ; J1 a J3 sont relus au depart et renvoyes tels
quels. Cadence d'envoi des consignes : 10 par seconde.

### Pourquoi une homographie, deux fois

Dans les deux cas, on projette un point d'un plan sur un autre depuis un centre fixe : le
centre optique de la camera d'un cote, le point d'emission du laser de l'autre. Tant que ce
centre ne bouge pas, la relation est exactement projective. C'est aussi pour ca que la base
du bras est figee : si le bras bougeait, le point d'emission bougerait et le modele ne
tiendrait plus.

---

## 4. Resultats

| Etape | Mesure | Resultat |
|---|---|---|
| Modele de visee (10–14/09) | erreur d'ajustement | 0,31 carreau (0,94 cm) |
| Calibration v2 (15/09) | 25 tirs | 2,53 cm rms |
| **Calibration v3 (15/09)** | 21 tirs | **0,46 cm rms**, 1,5 cm au pire |
| Suivi en cercle v3 (16/09) | 40 points | 0,39 carreau en moyenne (1,2 cm) |
| Camera seule (17/09) | 3 points, leurre immobile | 0,2 a 0,8 carreau |
| Bras seul (17/09) | 5 points | 0 au centre, 1 carreau aux bords |
| **Chaine complete (17/09)** | 3 points, leurre immobile | **touche au centre**, 0,5 a 1 carreau aux bords |
| Cadence detection (PC) | | 7 a 9 images/s |

Un frelon mesure environ 3 cm. [A VERIFIER : taille du leurre et du frelon reel]

---

## 5. Problemes rencontres et ce qu'on en retient

| Probleme | Consequence | Ce qu'on en retient |
|---|---|---|
| CSV avec des colonnes en cm sous des donnees en carreaux | erreurs 3 fois trop optimistes, sans message | un format de fichier se verifie a l'ecriture, pas seulement a la lecture |
| Le bras s'arrete sous sa consigne, et le firmware le dit « arrive » | 1,5 a 2,3 cm d'erreur | ne jamais croire l'accuse de reception du firmware : relire la pose |
| Le polynome retire en v2 compensait aussi ce manque, sans le dire | v2 moins bon que prevu (2,53 cm) | une correction doit dire ce qu'elle corrige |
| Marqueur de conflit git laisse dans un fichier | fichier qui ne compile plus | verifier apres chaque merge |
| PyTorch par pip inutilisable sur la Jetson Nano | detection impossible sur le robot tel quel | TensorRT sur la Jetson, ou detection deportee sur un PC |
| Piece rouge du robot prise pour une gommette | 4 carreaux d'erreur, calibrage accepte sans message | controle automatique des 4 coins et image de verification |
| Camera USB absente : ouverture de la webcam du PC | positions envoyees depuis une mauvaise image | refuser plutot que se rabattre sur un choix par defaut |
| Tentation de corriger l'erreur du bras sur 5 points | risque de degrader le pointage | mesurer au demi-carreau, sur assez de points, avant de corriger |

Methode retenue : **tester chaque moitie de la chaine separement**. Sans le test A (camera
seule) et le test B (bras seul), l'erreur de 4 carreaux aurait pu etre attribuee au bras, et
une nouvelle campagne de calibration du bras n'aurait rien change.

---

## 6. Limites et suite

**Limites actuelles**
- Resultats obtenus sur un **leurre immobile**. Sur un frelon en vol, la position arrive avec
  0,1 a 0,2 s de retard (7 a 9 images/s), sans anticipation.
- Peu de points de validation pour la chaine complete (3). Il faut une vraie serie.
- **Securite et inhibition du tir non ecrites** : aucun essai au laser de puissance tant que ce
  n'est pas fait.
- Le suivi lit le modele `modele_grille_v2.json`, alors que l'outil de reajustement v3 ecrit
  `modele_grille_v3.json` : a harmoniser avant une nouvelle campagne de calibration du bras.
- Le calibrage camera reste sensible a tout objet rouge dans le champ.

**Suite**
1. **Dix essais** sur un leurre immobile, a des positions variees, en notant combien touchent.
2. **Conditions d'inhibition du tir** (bras hors pose, cible perdue, personne dans la zone...),
   testees une par une, avant tout essai au laser de puissance.
3. Cible mobile : mesurer le retard reel et decider s'il faut anticiper la trajectoire.
4. Cadence : comparer TensorRT sur la Jetson et le PC. Sur le PC, essayer un export OpenVINO.
5. Integrer la detection sonore (confirmation abeille / frelon). [A COMPLETER : etat du pole son]

[A COMPLETER : repartition du travail dans l'equipe cette semaine]
