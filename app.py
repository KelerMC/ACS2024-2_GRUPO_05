import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import pandas as pd
from deap import base, creator, tools, algorithms
import random
import multiprocessing
import array


# Definimos la clase del sistema de Péndulo Invertido con Control PID
class CartPoleSystem:
    def __init__(self, pendulum_pid, cart_pid):
        self.M = 1.0  # Masa del carro (kg)
        self.m = 0.1  # Masa del péndulo (kg)
        self.l = 0.5  # Longitud del péndulo (m)
        self.g = 9.81  # Gravedad (m/s^2)

        # Controladores PID para el péndulo y el carro
        self.pendulum_pid = pendulum_pid
        self.cart_pid = cart_pid

        # Posiciones de referencia
        self.x_ref = 0.0  # Posición deseada del carro
        self.theta_ref = 0.0  # Ángulo deseado del péndulo

    def system_dynamics(self, state, t):
        x, theta, x_dot, theta_dot = state

        theta_error = theta - self.theta_ref
        x_error = x - self.x_ref

        # Actualización de los errores integrales
        dt = 0.01
        self.pendulum_pid['integral_error'] += theta_error * dt
        self.cart_pid['integral_error'] += x_error * dt

        # Señales de control PID
        pendulum_control = (self.pendulum_pid['kp'] * theta_error +
                            self.pendulum_pid['ki'] * self.pendulum_pid['integral_error'] +
                            self.pendulum_pid['kd'] * theta_dot)

        cart_control = (self.cart_pid['kp'] * x_error +
                        self.cart_pid['ki'] * self.cart_pid['integral_error'] +
                        self.cart_pid['kd'] * x_dot)

        # Fuerza combinada
        F = cart_control + pendulum_control

        # Ecuaciones del sistema
        M = np.array([[self.M + self.m, self.m * self.l * np.cos(theta)],
                      [self.m * self.l * np.cos(theta), self.m * self.l ** 2]])
        C = np.array([[-self.m * self.l * theta_dot ** 2 * np.sin(theta) + F],
                      [self.m * self.g * self.l * np.sin(theta)]])
        acc = np.linalg.solve(M, C)
        x_ddot = acc[0][0]
        theta_ddot = acc[1][0]

        return [x_dot, theta_dot, x_ddot, theta_ddot]

    def simulate(self, t_span, initial_state):
        t = np.linspace(0, t_span, int(t_span / 0.01))
        solution = odeint(self.system_dynamics, initial_state, t)
        return t, solution

# Función para optimizar los parámetros PID usando algoritmos genéticos
def evaluate(individual):
    system = CartPoleSystem(
        {'kp': individual[0], 'ki': 0, 'kd': individual[1], 'integral_error': 0},
        {'kp': individual[2], 'ki': 0, 'kd': individual[3], 'integral_error': 0}
    )

    t_span = 10.0
    initial_state = [0.0, np.radians(30.0), 0.0, 0.0]
    t = np.linspace(0, t_span, int(t_span / 0.01))

    try:
        solution = odeint(system.system_dynamics, initial_state, t)

        # Error cuadrático y penalización de oscilaciones
        x_error = np.sum((solution[:, 0] - system.x_ref) ** 2)
        theta_error = np.sum((solution[:, 1] - system.theta_ref) ** 2)
        x_oscillation = np.sum(np.diff(solution[:, 0]) ** 2)
        theta_oscillation = np.sum(np.diff(solution[:, 1]) ** 2)

        fitness = x_error + 10 * theta_error + 0.1 * x_oscillation + 0.1 * theta_oscillation
        return (fitness,)
    except:
        return (float('inf'),)


# Crear el algoritmo genético
def genetic_algorithm():
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", array.array, typecode='d', fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    pool = multiprocessing.Pool()
    toolbox.register("map", pool.map)

    # Genes: [pendulum_kp, pendulum_kd, cart_kp, cart_kd]
    toolbox.register("attr_float", random.uniform, 0, 100)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=4)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("evaluate", evaluate)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=10, indpb=0.2)
    toolbox.register("select", tools.selTournament, tournsize=3)

    population = toolbox.population(n=50)
    ngen = 20

    result, logbook = algorithms.eaSimple(population, toolbox, cxpb=0.7, mutpb=0.3, ngen=ngen, verbose=True)
    pool.close()

    best = tools.selBest(result, k=1)[0]
    return best

# Streamlit para la interfaz interactiva
st.title("Simulador de Péndulo Invertido con Control PID")

# Definición de los sliders para los controladores PID
kp_pendulum = st.slider("KP del péndulo", 0, 100, 40)
ki_pendulum = st.slider("KI del péndulo", 0, 10, 0)
kd_pendulum = st.slider("KD del péndulo", 0, 50, 4)

kp_cart = st.slider("KP del carro", 0, 50, 1)
ki_cart = st.slider("KI del carro", 0, 10, 0)
kd_cart = st.slider("KD del carro", 0, 50, 1)

# Botón para ejecutar simulación
if st.button("Ejecutar simulación"):
    pendulum_pid = {'kp': kp_pendulum, 'ki': ki_pendulum, 'kd': kd_pendulum, 'integral_error': 0}
    cart_pid = {'kp': kp_cart, 'ki': ki_cart, 'kd': kd_cart, 'integral_error': 0}

    system = CartPoleSystem(pendulum_pid, cart_pid)

    # Estado inicial del sistema
    initial_state = [0.0, np.radians(30.0), 0.0, 0.0]
    
    # Simulación
    t, solution = system.simulate(40.0, initial_state)

    # Mostrar los resultados
    fig, ax = plt.subplots(2, 1, figsize=(10, 6))

    ax[0].plot(t, solution[:, 0], label="Posición del carro")
    ax[0].axhline(y=system.x_ref, color='r', linestyle='--', label="Referencia")
    ax[0].set_ylabel("Posición (m)")
    ax[0].set_title("Posición del Carro")
    ax[0].legend()

    ax[1].plot(t, np.degrees(solution[:, 1]), label="Ángulo del péndulo")
    ax[1].axhline(y=0, color='r', linestyle='--', label="Referencia")
    ax[1].set_ylabel("Ángulo (grados)")
    ax[1].set_title("Ángulo del Péndulo")
    ax[1].legend()

    st.pyplot(fig)

    # Guardar resultados en CSV
    results = pd.DataFrame({
        "Tiempo": t,
        "Posición del carro (m)": solution[:, 0],
        "Ángulo del péndulo (grados)": np.degrees(solution[:, 1]),
    })

    # Opción de descarga
    st.download_button(
        label="Descargar resultados como CSV",
        data=results.to_csv(index=False),
        file_name="simulation_results.csv",
        mime="text/csv",
    )

# Botón para optimizar parámetros con algoritmo genético
if st.button("Optimizar parámetros PID"):
    best_gains = genetic_algorithm()

    st.write(f"Mejores ganancias encontradas:")
    st.write(f"Péndulo: KP = {best_gains[0]:.2f}, KD = {best_gains[1]:.2f}")
    st.write(f"Carro: KP = {best_gains[2]:.2f}, KD = {best_gains[3]:.2f}")
