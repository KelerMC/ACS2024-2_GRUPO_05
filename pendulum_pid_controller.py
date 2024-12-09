# pendulum_pid_controller.py
import numpy as np
from scipy.integrate import odeint

class PendulumSystem:
    def __init__(self):
        # Parámetros del sistema
        self.g = 9.81  # Gravedad
        self.l = 1.0  # Longitud del péndulo
        self.m = 1.0  # Masa
        self.b = 0.1  # Coeficiente de fricción

    def simulate(self, t, x0, kp, ki, kd):
        def system_equations(state, t):
            theta, omega, integral_error = state

            # Calcular error y PID
            error = 0 - theta  # El objetivo es mantener el péndulo en 0
            integral_error += error * 0.01  # dt = 0.01
            derivative = -omega

            # Señal de control PID
            u = kp * error + ki * integral_error + kd * derivative

            # Ecuaciones del péndulo
            dtheta = omega
            domega = (-self.b * omega + self.m * self.g * self.l * np.sin(theta) + u) / (self.m * self.l ** 2)

            return [dtheta, domega, error]

        # Tiempo de simulación
        t = np.linspace(0, t, int(t / 0.01))

        # Resolver ecuaciones diferenciales
        solution = odeint(system_equations, [x0, 0, 0], t)

        # Convertir radianes a grados
        angles = np.degrees(solution[:, 0])

        return t, angles
