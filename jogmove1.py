from pymycobot.mycobot import MyCobot
import time
import PU_pointer_vers
###definition constante
POSITION_INIT=[73.6, -63.0, 300.1, 179.9, -0.78, -88.41]
ANGLES_INIT=[0,0,0,0,0,0]
SPEED = 60
P=['X','Y','Z']
mc = MyCobot('/dev/ttyTHS1',1000000)
### degager l'espace
print("Attention ! Degagez l'espace autour du bras !")


### recuperation de la position actuelle
position_actuelle = mc.get_coords()
angles_actuels=mc.get_angles()
print(angles_actuels)
print(position_actuelle)


### test si different de la position initiale

if angles_actuels!=ANGLES_INIT :
    reponse = input("Retour a la position initiale ? (oui/non) : ")

    if reponse.lower() == "oui" : #lower permet a l'utilisateur de taper oui Oui oUi OUI...
        #mc.send_angles(ANGLES_INIT,SPEED)
        mc.send_angles(ANGLES_INIT,SPEED)
        #POSITION_INIT = mc.get_coords()
        time.sleep(2)
        #print(mc.get_angles())        
        print("Position Initiale =",mc.get_coords())

### boucle tant que l'utilisateur ne tape pas quit/echap
quitter =' '

while quitter.lower()!= 'q':

    ### demander a l'utilisateur la position souhaitee
    position_souhaitee=[]
    for i in range(1,4) :        
        J=float(input(f"Entrez la coordonnee {P[i-1]} : "))
        position_souhaitee.append(J)
    position_souhaitee=position_souhaitee+[179.9, -0.78, -88.41]
    print("Position souhaitee =",position_souhaitee)
        
    confirmation = input("Confirmation mouvement ? (oui/non) : ")
    if confirmation.lower() == "oui" :
      PU_pointer_vers.verspointer(200, 100, 250,20)
      mc.send_coords(position_souhaitee,SPEED)
      
      time.sleep(3)
      print("Position obtenue =", mc.get_coords())
      print(mc.get_angles())
    quitter=input("Pour quitter taper q sinon taper entrer: ")