import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
import control as ctrl

class CarControllerPID:
    def __init__(self, tiempo_sim, angulo_inicial_grados, M, m, l, g):
        self.tiempo_sim = tiempo_sim
        self.posicion_inicial = np.radians(angulo_inicial_grados)
        self.M = M
        self.m = m
        self.l = l
        self.g = g
        self.KP_v = []
        self.KI_v = []
        self.KD_v = []
    

def main():
    angulo_inicial_grados = 10
    tiempo_sim = 20
    M, m, l, g = 1.0, 0.1, 0.5, 9.81  # Parámetros del sistema del carro
    
    car_controller_pid = CarControllerPID(tiempo_sim, angulo_inicial_grados, M, m, l, g)
    
if __name__ == "__main__":
    main()
