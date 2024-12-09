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
print("Ecuación 1 (carro):")
display_equation(sp.Eq(eq1, 0))
# Ecuación 2 (del péndulo) en dominio de Laplace:
# mls²X(s) - ml²s²Theta(s) + mglTheta(s) = 0
eq2 = m*l*s**2*X - m*l**2*s**2*Theta + m*g*l*Theta
print("Ecuación 2 (péndulo):")
display_equation(sp.Eq(eq2, 0))

# Resolver el sistema de ecuaciones para Theta/U
# Primero despejamos X de eq2
X_solved = sp.solve(eq2, X)[0]


print("X despejado:")
display_equation(sp.Eq(X, X_solved))

# Sustituimos X en eq1
eq_final = eq1.subs(X, X_solved)

# Resolvemos para Theta/U
transfer_function = -sp.solve(eq_final, Theta)[0]/U

# Simplificar la expresión
transfer_function_simplified = sp.simplify(transfer_function)

print("Función de transferencia Theta(s)/U(s):")
display_equation(sp.Eq(Theta/U, transfer_function_simplified))

# Encontrar polos (denominador = 0)
denominator = sp.fraction(transfer_function_simplified)[1]
poles = sp.solve(denominator, s)

print("\nPolos del sistema:")
for i, pole in enumerate(poles, 1):
    display_equation(sp.Eq(sp.Symbol(f's_{i}'), pole))

# Factor forma de la función de transferencia
factored_form = sp.factor(transfer_function_simplified)
print("\nForma factorizada de la función de transferencia:")
display_equation(sp.Eq(Theta/U, factored_form))