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
