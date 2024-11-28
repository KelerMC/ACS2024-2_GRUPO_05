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

# Ecuación 1 (del carro) en dominio de Laplace:
# (M+m)s²X(s) - mls²Theta(s) = U(s)
eq1 = (M + m)*s**2*X - m*l*s**2*Theta - U

# Ecuación 1 (del carro) en dominio de Laplace:
# (M+m)s²X(s) - mls²Theta(s) = U(s)
eq1 = (M + m)*s**2*X - m*l*s**2*Theta - U
print("Ecuación 1 (carro):")
display_equation(sp.Eq(eq1, 0))