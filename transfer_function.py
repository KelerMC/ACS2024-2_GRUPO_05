import sympy as sp
import matplotlib.pyplot as plt

def display_equation(expr, title=""):
    """Función para mostrar ecuaciones en LaTeX"""
    plt.figure(figsize=(10, 2))
    plt.text(0.5, 0.5, f"${sp.latex(expr)}$",
             fontsize=14, ha='center', va='center')
    plt.axis('off')
    if title:
        plt.title(title)
    plt.show()

# Definir variables simbólicas
s = sp.Symbol('s')  # Variable de Laplace
M, m, l, g = sp.symbols('M m l g')  # Parámetros del sistema
X, U, Theta = sp.symbols('X U Theta')  # Variables en dominio de Laplace