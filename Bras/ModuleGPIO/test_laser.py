from pymycobot.mycobot import MyCobot

mc = MyCobot('/dev/ttyTHS1', 1000000)

mc.set_basic_output(2, 0)
input("Le laser devrait être allumé. Vérifie puis appuie sur Entrée.")

mc.set_basic_output(2, 1)
input("Le laser devrait être éteint. Vérifie puis appuie sur Entrée.")