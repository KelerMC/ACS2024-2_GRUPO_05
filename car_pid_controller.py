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
 # Definir variables simbólicas
        self.s = sp.Symbol('s')  # Variable de Laplace
        self.X, self.U = sp.symbols('X U')
        
        # Ecuación 1 (del carro) en dominio de Laplace
        self.eq1 = (self.M + self.m)*self.s**2*self.X - self.m*self.l*self.s**2*self.X - self.U
        
        # Función de transferencia del carro: X(s)/U(s)
        self.transfer_function = sp.solve(self.eq1, self.X)[0]/self.U
        self.transfer_function_simplified = sp.simplify(self.transfer_function)
        
        # Convertir la función de transferencia a una forma usable por la librería control
        self.num, self.den = sp.fraction(self.transfer_function_simplified)
        self.num = np.array([float(coef) for coef in self.num.as_coefficients_dict().values()])
        self.den = np.array([float(coef) for coef in self.den.as_coefficients_dict().values()])
        
        # Crear la función de transferencia en Python usando control
        self.system = ctrl.TransferFunction(self.num, self.den)
        
def simulate(self, KP, KI, KD):   
        # Simula el sistema PID con los parámetros dados de KP, KI, y KD.
        # Crear el controlador PID
        pid_controller = ctrl.TransferFunction([KD, KP, KI], [1, 0])
        
        # Sistema cerrado (feedback)
        closed_loop_system = ctrl.feedback(pid_controller * self.system)
        
        # Simular la respuesta del sistema para una entrada escalón (U=1)
        time = np.linspace(0, self.tiempo_sim, 1000)
        time, yout = ctrl.forced_response(closed_loop_system, T=time, U=1)
        
        return time, yout
    
def main():
    angulo_inicial_grados = 10
    tiempo_sim = 20
    M, m, l, g = 1.0, 0.1, 0.5, 9.81  # Parámetros del sistema del carro
    
    car_controller_pid = CarControllerPID(tiempo_sim, angulo_inicial_grados, M, m, l, g)
    
if __name__ == "__main__":
    main()
