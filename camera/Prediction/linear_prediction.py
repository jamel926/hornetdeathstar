import time
import numpy as np
from collections import deque

class PositionPredictor:
    def __init__(self, buffer_size=10, poly_degree=1, timeout=0.5):
        """
        buffer_size : Nombre de positions à retenir.
        poly_degree : 1 (linéaire) ou 2 (courbes).
        timeout : Temps en secondes avant de considérer l'objet comme "perdu".
        """
        self.history = deque(maxlen=buffer_size)
        self.poly_degree = poly_degree
        self.timeout = timeout

    def add_detection(self, coords):
        """
        L'IA peut appeler cette fonction avec (x, y) ou None. 
        Elle peut aussi simplement ne pas l'appeler si elle ne détecte rien.
        """
        if coords is None:
            return
            
        self.history.append((time.time(), coords[0], coords[1]))

    def get_prediction(self, latency_t0):
        if len(self.history) == 0:
            return None
            
        current_time = time.time()
        
        # Sécurité contre les données périmées
        last_detection_time = self.history[-1][0]
        if current_time - last_detection_time > self.timeout:
            self.history.clear()  # Purge du buffer
            return None           # On signale au robot qu'on a perdu la cible

        # Pas assez de points pour une ligne, on renvoie la dernière position
        if len(self.history) < 3:
            return (self.history[-1][1], self.history[-1][2])

        times = np.array([pt[0] for pt in self.history])
        x_vals = np.array([pt[1] for pt in self.history])
        y_vals = np.array([pt[2] for pt in self.history])

        target_time = current_time + latency_t0

        x_pred, y_pred = self._compute_extrapolation(times, x_vals, y_vals, target_time)

        # Contrainte aux limites [0, 1]
        x_pred = max(0.0, min(1.0, x_pred))
        y_pred = max(0.0, min(1.0, y_pred))

        return (x_pred, y_pred)

    def _compute_extrapolation(self, t, x, y, target_t):
        t_norm = t - t[0]
        target_t_norm = target_t - t[0]

        degree = min(self.poly_degree, len(t) - 1)

        poly_x = np.polyfit(t_norm, x, degree)
        poly_y = np.polyfit(t_norm, y, degree)

        x_pred = np.polyval(poly_x, target_t_norm)
        y_pred = np.polyval(poly_y, target_t_norm)

        return x_pred, y_pred