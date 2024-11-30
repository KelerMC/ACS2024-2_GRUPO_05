import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class PendulumSystem:
    def __init__(self):
        # Parámetros del sistema
        self.g = 9.81  # Gravedad
        self.l = 1.0   # Longitud del péndulo
        self.m = 1.0   # Masa
        self.b = 0.1   # Coeficiente de fricción

