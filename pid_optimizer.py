# pid_optimizer.py
import random
from deap import base, creator, tools, algorithms
import numpy as np

class PIDOptimizer:
    def __init__(self, system):
        self.system = system  # Instancia del sistema de péndulo
        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))  # Queremos minimizar el error
        creator.create("Individual", list, fitness=creator.FitnessMin)

    def fitness(self, individual):
        Kp, Ki, Kd = individual
        time, angles = self.system.simulate(20, np.radians(10), Kp, Ki, Kd)  # 20 segundos, ángulo inicial de 10 grados
        error = np.sum(np.square(angles))  # Error cuadrático total
        return error,

    def create_individual(self):
        # Asegúrate de que el individuo sea del tipo creator.Individual, no solo una lista
        individual = [random.uniform(0, 100), random.uniform(0, 100), random.uniform(0, 100)]  # Inicializar para Kp, Ki, Kd
        return creator.Individual(individual)

    def optimize(self, population_size=20, generations=50):
        # Crear la población inicial con el tipo correcto
        population = [self.create_individual() for _ in range(population_size)]

        # Operadores de selección, cruzamiento y mutación
        toolbox = base.Toolbox()
        toolbox.register("mate", tools.cxBlend, alpha=0.5)
        toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=5, indpb=0.2)
        toolbox.register("select", tools.selTournament, tournsize=3)
        toolbox.register("evaluate", self.fitness)

        # Algoritmo evolutivo
        algorithms.eaSimple(population, toolbox, cxpb=0.7, mutpb=0.2, ngen=generations, verbose=True)

        # Obtener el mejor individuo (mejores valores de PID)
        best_individual = tools.selBest(population, 1)[0]
        return best_individual
