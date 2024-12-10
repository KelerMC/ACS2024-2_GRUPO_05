import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


class CartPoleSystem:
    def __init__(self):
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 0.1  # Mass of pendulum (kg)
        self.l = 0.5  # Length of pendulum (m)
        self.g = 9.81  # Gravity (m/s^2)

        # Controller parameters for pendulum
        self.pendulum_pid = {
            'kp': 40,
            'ki': 0,
            'kd': 4,
            'integral_error': 0
        }

        # Controller parameters for cart
        self.cart_pid = {
            'kp': 1,
            'ki': 0,
            'kd': 1,
            'integral_error': 0
        }

        # Reference positions
        self.x_ref = 0.0  # Desired cart position
        self.theta_ref = 0.0  # Desired pendulum angle

    def system_dynamics(self, state, t):
        x, theta, x_dot, theta_dot = state

        # Calculate errors
        theta_error = theta - self.theta_ref
        x_error = x - self.x_ref

        # Update integral errors
        dt = 0.01  # Small time step for integral calculation
        self.pendulum_pid['integral_error'] += theta_error * dt
        self.cart_pid['integral_error'] += x_error * dt

        # Calculate PID control signals
        pendulum_control = (self.pendulum_pid['kp'] * theta_error +
                            self.pendulum_pid['ki'] * self.pendulum_pid['integral_error'] +
                            self.pendulum_pid['kd'] * theta_dot)

        cart_control = (self.cart_pid['kp'] * x_error +
                        self.cart_pid['ki'] * self.cart_pid['integral_error'] +
                        self.cart_pid['kd'] * x_dot)

        # Combined control force
        F = cart_control + pendulum_control

        # System matrices
        M = np.array([[self.M + self.m, self.m * self.l * np.cos(theta)],
                      [self.m * self.l * np.cos(theta), self.m * self.l ** 2]])

        C = np.array([[-self.m * self.l * theta_dot ** 2 * np.sin(theta) + F],
                      [self.m * self.g * self.l * np.sin(theta)]])

        # Solve for accelerations
        acc = np.linalg.solve(M, C)
        x_ddot = acc[0][0]
        theta_ddot = acc[1][0]

        return [x_dot, theta_dot, x_ddot, theta_ddot]

    def simulate(self, t_span, initial_state):
        t = np.linspace(0, t_span, int(t_span / 0.01))
        solution = odeint(self.system_dynamics, initial_state, t)
        return t, solution
