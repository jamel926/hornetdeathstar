from pymycobot.mycobot280 import MyCobot280

mc = MyCobot280('/dev/ttyTHS1', 1000000)

### Broche de commande du laser sur la carte Basic ###
PIN_LASER = 2

### Dernier etat commande sur la broche ###
### pymycobot ne permet pas de relire une sortie (set_basic_output est en ecriture seule), ###
### on memorise donc ce que l'on a envoye. ###
_etat_gpio = 0

def toggleOn():
    """ On met le pin 2 à 1 pour allumer la GPIO """
    global _etat_gpio
    mc.set_basic_output(PIN_LASER, 1)
    _etat_gpio = 1
    print("La GPIO est allumé.")

def toggleOff():
    """ On met le pin 2 à 0 pour éteindre la GPIO """
    global _etat_gpio
    mc.set_basic_output(PIN_LASER, 0)
    _etat_gpio = 0
    print("La GPIO est éteint.")

def afficheEtatGPIO():
    """ Retourne le dernier état commandé sur la GPIO """
    if _etat_gpio == 1:
        print("La GPIO est allumé.")
    else:
        print("La GPIO est éteint.")
