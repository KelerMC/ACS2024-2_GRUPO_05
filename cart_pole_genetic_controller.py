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