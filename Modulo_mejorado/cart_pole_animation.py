import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


class CartPoleSystem:
    def __init__(self):
        # Parámetros físicos del sistema
        self.M = 1.0  # Masa del carro (kg)
        self.m = 0.1  # Masa del péndulo (kg)
        self.l = 0.5  # Longitud del péndulo (m)
        self.g = 9.81  # Gravedad (m/s^2)

        # Referencias del sistema
        self.x_ref = 0.0  # Posición deseada del carro
        self.theta_ref = np.pi  # Posición vertical hacia arriba

        # Parámetros de control conservadores para estabilidad a largo plazo
        self.k_energy = 0.5  # Ganancia de energía más conservadora

        # Control de estabilización con ganancias más moderadas
        self.k_theta = 200.0  # Ganancia proporcional para ángulo
        self.k_theta_dot = 20.0  # Ganancia derivativa para ángulo
        self.k_x = 0.8  # Ganancia proporcional para posición
        self.k_x_dot = 1.2  # Ganancia derivativa para posición

        # Parámetros para amortiguamiento adaptativo
        self.base_damping = 0.1  # Amortiguamiento base
        self.extra_damping = 0.5  # Amortiguamiento adicional cerca del equilibrio

        # Umbral para cambio de control
        self.capture_threshold = 0.8  # Radianes, región más amplia de captura

    def calculate_energy(self, theta, theta_dot):
        """Calcula la energía mecánica total del péndulo"""
        E_p = self.m * self.g * self.l * (np.cos(theta) - 1)
        E_k = 0.5 * self.m * (self.l * theta_dot) ** 2
        return E_p + E_k

    def system_dynamics(self, state, t):
        x, theta, x_dot, theta_dot = state

        # Normalización del error angular
        theta_error = (theta - self.theta_ref + np.pi) % (2 * np.pi) - np.pi

        # Cálculo de energía
        current_energy = self.calculate_energy(theta, theta_dot)

        # Factor de proximidad al equilibrio usando una función suave
        proximity = 0.5 * (1 + np.cos(theta_error))

        # Decisión de control basada en la región
        if abs(theta_error) < self.capture_threshold:
            # Región de estabilización
            # Control LQR-like con ganancias conservadoras
            F = (-self.k_theta * theta_error
                 - self.k_theta_dot * theta_dot * (1 + 0.5 * np.cos(theta))  # Amortiguamiento no lineal
                 - self.k_x * x * (1 - 0.5 * abs(theta_error))  # Reducción de control de posición lejos del equilibrio
                 - self.k_x_dot * x_dot)

            # Compensación de gravedad suave
            F += self.m * self.g * np.sin(theta) * proximity
        else:
            # Región de swing-up
            # Control de energía con modulación suave
            energy_error = current_energy
            F = self.k_energy * energy_error * np.cos(theta) * theta_dot

            # Control mínimo de posición durante swing-up
            F += -0.1 * x - 0.2 * x_dot

        # Limitación de fuerza más conservadora
        F = np.clip(F, -20, 20)

        # Amortiguamiento adaptativo que aumenta cerca del equilibrio
        damping = self.base_damping + self.extra_damping * proximity

        # Términos trigonométricos
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        # Dinámica del sistema con amortiguamiento adaptativo
        den = self.M + self.m * sin_theta ** 2

        # Amortiguamiento natural del sistema
        cart_damping = damping * x_dot
        pendulum_damping = damping * theta_dot

        x_ddot = (F + self.m * self.l * theta_dot ** 2 * sin_theta -
                  self.m * self.g * sin_theta * cos_theta - cart_damping) / den

        theta_ddot = (-F * cos_theta - self.m * self.l * theta_dot ** 2 * sin_theta * cos_theta +
                      (self.M + self.m) * self.g * sin_theta - pendulum_damping) / (self.l * den)

        return [x_dot, theta_dot, x_ddot, theta_ddot]

    def simulate(self, t_span, initial_state):
        dt = 0.01
        n_points = int(t_span / dt) + 1
        t = np.linspace(0, t_span, n_points)

        solution = odeint(self.system_dynamics,
                          initial_state,
                          t,
                          rtol=1e-8,
                          atol=1e-8)

        return t, solution

    def plot_results(self, t, solution):
        plt.figure(figsize=(12, 10))

        # Gráfica de posición del carro
        plt.subplot(3, 1, 1)
        plt.plot(t, solution[:, 0], 'b-', label='Posición del Carro')
        plt.plot(t, np.ones_like(t) * self.x_ref, 'r--', label='Referencia')
        plt.grid(True)
        plt.legend()
        plt.ylabel('Posición (m)')
        plt.title('Respuesta del Sistema Carro-Péndulo Mejorado')

        # Gráfica del ángulo del péndulo
        plt.subplot(3, 1, 2)
        angles = solution[:, 1] % (2 * np.pi)
        plt.plot(t, np.degrees(angles), 'g-', label='Ángulo del Péndulo')
        plt.plot(t, np.degrees(np.ones_like(t) * self.theta_ref), 'r--',
                 label='Referencia')
        plt.grid(True)
        plt.legend()
        plt.ylabel('Ángulo (grados)')

        # Gráfica de energía
        plt.subplot(3, 1, 3)
        energies = [self.calculate_energy(state[1], state[3])
                    for state in solution]
        plt.plot(t, energies, 'm-', label='Energía Total')
        plt.plot(t, [0] * len(t), 'r--', label='Energía Referencia')
        plt.grid(True)
        plt.legend()
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Energía (J)')

        plt.tight_layout()
        plt.show()


def main():
    system = CartPoleSystem()

    # Condiciones iniciales
    initial_state = [0.0, 0.2, 0.0, 0.0]
    t_span = 100.0

    t, solution = system.simulate(t_span, initial_state)
    system.plot_results(t, solution)


if __name__ == "__main__":
    main()