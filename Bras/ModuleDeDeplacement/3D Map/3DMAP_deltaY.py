import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score

# ==========================================================
# PARAMETRES
# ==========================================================
df = pd.read_csv("/Users/maellechevennement/Documents/apprentissage_erreurs.csv")
DEGRE_MAX = 6     # degré maximal testé

# ==========================================================
# GENERATION DE DONNEES TEST
# ==========================================================
#


x_vise = df["x_vise"].values
y_vise = df["y_vise"].values
erreur_x = df["erreur_y"].values

# Pour le reste du programme
X = x_vise
Y = y_vise
Z = erreur_x

# ajout de bruit de mesure



# ==========================================================
# RECHERCHE AUTOMATIQUE DU DEGRE
# ==========================================================

XY = np.column_stack((X, Y))

print("\nRecherche du meilleur degré\n")

meilleur_modele = None
meilleur_poly = None
meilleur_r2 = -999
meilleur_degre = 1

for degre in range(1, DEGRE_MAX + 1):

    poly = PolynomialFeatures(degree=degre)

    XY_poly = poly.fit_transform(XY)

    modele = LinearRegression()
    modele.fit(XY_poly, Z)

    Z_pred = modele.predict(XY_poly)

    r2 = r2_score(Z, Z_pred)

    print(f"Degré {degre} : R² = {r2:.5f}")

    if r2 > meilleur_r2:
        meilleur_r2 = r2
        meilleur_modele = modele
        meilleur_poly = poly
        meilleur_degre = degre

print("\n--------------------------------")
print("Meilleur degré :", meilleur_degre)
print("R² =", round(meilleur_r2, 5))
print("--------------------------------")

# ==========================================================
# EQUATION TROUVEE
# ==========================================================

noms = meilleur_poly.get_feature_names_out(["X", "Y"])

print("\nEquation approchée :\n")

equation = f"Z = {meilleur_modele.intercept_:.5f}"

for coef, nom in zip(meilleur_modele.coef_[1:], noms[1:]):

    if abs(coef) > 1e-5:
        equation += f" + ({coef:.5f})*{nom}"

print(equation)

# ==========================================================
# SURFACE DE REGRESSION
# ==========================================================

xg = np.linspace(X.min(), X.max(), 50)
yg = np.linspace(Y.min(), Y.max(), 50)

Xg, Yg = np.meshgrid(xg, yg)

XY_grid = np.column_stack([
    Xg.ravel(),
    Yg.ravel()
])

XY_grid_poly = meilleur_poly.transform(XY_grid)

Zg = meilleur_modele.predict(XY_grid_poly)

Zg = Zg.reshape(Xg.shape)

# =====================================================
# AFFICHAGE INTERACTIF PLOTLY
# =====================================================

import plotly.graph_objects as go


fig = go.Figure()

# Surface de régression
fig.add_trace(
    go.Surface(
        x=Xg,
        y=Yg,
        z=Zg,
        colorscale='Viridis',
        opacity=0.8,
        name='Régression'
    )
)

# Points expérimentaux
fig.add_trace(
    go.Scatter3d(
        x=X,
        y=Y,
        z=Z,
        mode='markers',
        marker=dict(
            symbol = 'cross',
            size=4,
            color='red'
        ),
        name='Mesures'
    )
)

fig.update_layout(
    title=f"Régression polynomiale (R²={r2:.4f})",
    scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='erreur en Y'
    ),
    width=1000,
    height=800
)

fig.show()

# ==========================================================
# COMPARAISON MESURE / PREDICTION
# ==========================================================

XY_poly = meilleur_poly.transform(XY)

Z_calcule = meilleur_modele.predict(XY_poly)

erreur = np.abs(Z - Z_calcule)

print("\nRésultats")
print("----------")
print("Erreur moyenne :", erreur.mean())
print("Erreur max     :", erreur.max())