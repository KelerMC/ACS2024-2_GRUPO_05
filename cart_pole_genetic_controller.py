import numpy as np
from deap import base, creator, tools, algorithms
import random
from scipy.integrate import odeint
import multiprocessing
import array


# Reutilizamos la clase CartPoleSystem
class CartPoleSystem:
    def __init__(self, gains):
        self.M = 1.0
        self.m = 0.1
        self.l = 0.5
        self.g = 9.81

        # Extraemos las ganancias del individuo
        self.pendulum_pid = {
            'kp': gains[0],
            'ki': 0,
            'kd': gains[1],
            'integral_error': 0
        }

        self.cart_pid = {
            'kp': gains[2],
            'ki': 0,
            'kd': gains[3],
            'integral_error': 0
        }

        self.x_ref = 0.0
        self.theta_ref = 0.0

    def system_dynamics(self, state, t):
        x, theta, x_dot, theta_dot = state

        theta_error = theta - self.theta_ref
        x_error = x - self.x_ref

        dt = 0.01
        self.pendulum_pid['integral_error'] += theta_error * dt
        self.cart_pid['integral_error'] += x_error * dt

        pendulum_control = (self.pendulum_pid['kp'] * theta_error +
                            self.pendulum_pid['kd'] * theta_dot)

        cart_control = (self.cart_pid['kp'] * x_error +
                        self.cart_pid['kd'] * x_dot)

        F = cart_control + pendulum_control

        M = np.array([[self.M + self.m, self.m * self.l * np.cos(theta)],
                      [self.m * self.l * np.cos(theta), self.m * self.l ** 2]])

        C = np.array([[-self.m * self.l * theta_dot ** 2 * np.sin(theta) + F],
                      [self.m * self.g * self.l * np.sin(theta)]])

        acc = np.linalg.solve(M, C)
        x_ddot = acc[0][0]
        theta_ddot = acc[1][0]

        return [x_dot, theta_dot, x_ddot, theta_ddot]


def evaluate(individual):
    # Configurar sistema con las ganancias del individuo
    system = CartPoleSystem(individual)

    # Parámetros de simulación
    t_span = 10.0  # Reducido para acelerar la evaluación
    initial_state = [0.0, np.radians(30.0), 0.0, 0.0]
    t = np.linspace(0, t_span, int(t_span / 0.01))

    try:
        # Simular sistema
        solution = odeint(system.system_dynamics, initial_state, t)

        # Calcular error cuadrático
        x_error = np.sum((solution[:, 0] - system.x_ref) ** 2)
        theta_error = np.sum((solution[:, 1] - system.theta_ref) ** 2)

        # Penalizar oscilaciones excesivas
        x_oscillation = np.sum(np.diff(solution[:, 0]) ** 2)
        theta_oscillation = np.sum(np.diff(solution[:, 1]) ** 2)

        # Función de fitness (menor es mejor)
        fitness = x_error + 10 * theta_error + 0.1 * x_oscillation + 0.1 * theta_oscillation

        return (fitness,)
    except:
        return (float('inf'),)
        
def main():
    # Configuración del algoritmo genético
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", array.array, typecode='d', fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    
    # Paralelización
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
    
    # Algoritmo
    population = toolbox.population(n=50)
    ngen = 20
    
    result, logbook = algorithms.eaSimple(population, toolbox, 
                                        cxpb=0.7, mutpb=0.3, 
                                        ngen=ngen, verbose=True)
    
    pool.close()
    
    # Obtener mejor individuo
    best = tools.selBest(result, k=1)[0]
    print("\nMejores ganancias encontradas:")
    print(f"Péndulo: kp={best[0]:.2f}, kd={best[1]:.2f}")
    print(f"Carro: kp={best[2]:.2f}, kd={best[3]:.2f}")
    
    return best

if __name__ == "__main__":
    best_gains = main()
