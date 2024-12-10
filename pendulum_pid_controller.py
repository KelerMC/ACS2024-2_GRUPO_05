import numpy as np
import matplotlib.pyplot as plt
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
        

def plot_responses(system, time_sim, initial_angle, param_name, param_values, base_kp=30, base_ki=5.52, base_kd=3.66):
    plt.figure(figsize=(10, 6))

    for value in param_values:
        kp, ki, kd = base_kp, base_ki, base_kd

        if param_name == 'kp':
            kp = value
        elif param_name == 'ki':
            ki = value
        else:
            kd = value

        t, angles = system.simulate(time_sim, np.radians(initial_angle), kp, ki, kd)
        plt.plot(t, angles, label=f'{param_name}={value}')

    plt.title(f'Ángulo del péndulo vs tiempo (variando {param_name})')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Ángulo (º)')
    plt.legend()
    plt.grid(True)
    plt.show()

#Permite generar un grafico individual segun los parametros indicados
def plot_simulation(system, time_sim, initial_angle, kp, ki, kd):
    t, angles = system.simulate(time_sim, np.radians(initial_angle), kp, ki, kd)
    plt.figure(figsize=(10, 6))
    plt.plot(t, angles)
    plt.title(f'Respuesta del sistema con Kp={kp}, Ki={ki}, Kd={kd}')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Ángulo (º)')
    plt.grid(True)
    plt.show()

def main():
    # Inicialización
    system = PendulumSystem()
    initial_angle = 10  # grados
    time_sim = 5       # segundos

   # Valores para sintonización
    kp_values = np.arange(30, 110, 10)
    ki_values = np.arange(0, 16, 2)
    kd_values = np.arange(0, 16, 2)

    # Graficar variando KP
    print("Sintonizando KP...")
    plot_responses(system, time_sim, initial_angle, 'kp', kp_values)

    # Graficar variando KI
    print("Sintonizando KI...")
    plot_responses(system, time_sim, initial_angle, 'ki', ki_values, base_kp=40)

    # Graficar variando KD
    print("Sintonizando KD...")
    plot_responses(system, time_sim, initial_angle, 'kd', kd_values, base_kp=40, base_ki=0)

    # Resultados finales con valores optimizados
    print("Graficando resultados finales...")
    t, angles = system.simulate(time_sim, np.radians(initial_angle), kp=40, ki=0, kd=4)

    plt.figure(figsize=(10, 6))
    plt.plot(t, angles)
    plt.title('Respuesta del sistema con Kp=40 y Kd=4')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Ángulo (º)')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
