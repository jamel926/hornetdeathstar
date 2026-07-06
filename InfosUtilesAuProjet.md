# Convention de Nommage en Python

| //             | Convention              |
| -------------- | ----------------------- |
| constante      | snake_case              |
| variable       | SCREAMING_SNAKE_CASE    |
| fonction       | camelCase               |
| classe-objet   | PascalCase/CapWord      |
| attribut privé | underscore + snake_case |
# Convention pour documenter une fonction
## Convention :
```python
def nomDeLaFonction(parametre_1,parametre_2):
	"""Description de la fonction"""
	#reste de la fonction
	# … 
```

## Exemple

```python
def saluer(nom):
    """Retourne une salutation avec le nom fourni."""
    return f"Bonjour {nom} !"
```


# Configuration du bras

Port COM = "THS1"
débit = 1Mb/s