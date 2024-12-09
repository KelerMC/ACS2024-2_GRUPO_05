# main.py
import matplotlib.pyplot as plt
from pendulum_pid_controller import PendulumSystem
from pid_optimizer import PIDOptimizer

class Main:
    def __init__(self):
        # Crear el sistema de péndulo
        self.system = PendulumSystem()

        # Crear el optimizador de PID
        self.optimizer = PIDOptimizer(self.system)

    def run(self):
        # 1. Ejecutar la simulación del sistema sin optimización
        Kp_manual, Ki_manual, Kd_manual = 30, 5.52, 3.66  # Valores manuales para PID
        print(f"Simulación con PID ajustado manualmente: Kp={Kp_manual}, Ki={Ki_manual}, Kd={Kd_manual}")

        time, angles_manual = self.system.simulate(20, 10, Kp_manual, Ki_manual, Kd_manual)  # Valores manuales para PID

        # Visualizar resultados con PID ajustado manualmente
        plt.plot(time, angles_manual, label="PID Ajustado Manualmente")
        plt.title('Respuesta del Sistema con PID Ajustado Manualmente')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Ángulo (º)')
        plt.legend()
        plt.grid(True)
        plt.show()

        # 2. Optimizar los parámetros PID usando un algoritmo genético
        print("Optimizando los parámetros PID...")
        best_pid = self.optimizer.optimize()
        print(f"Mejores parámetros PID optimizados: Kp = {best_pid[0]}, Ki = {best_pid[1]}, Kd = {best_pid[2]}")

        # 3. Ejecutar la simulación del sistema con los parámetros optimizados
        Kp_opt, Ki_opt, Kd_opt = best_pid
        time, angles_optimized = self.system.simulate(20, 10, Kp_opt, Ki_opt, Kd_opt)

        # Visualizar resultados con PID optimizado
        plt.plot(time, angles_optimized, label=f"PID Optimizado: Kp={Kp_opt}, Ki={Ki_opt}, Kd={Kd_opt}")
        plt.title('Respuesta del Sistema con PID Optimizado')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Ángulo (º)')
        plt.legend()
        plt.grid(True)
        plt.show()

        # 4. Visualizar resultados comparando PID ajustado manualmente y PID optimizado
        plt.figure(figsize=(10, 6))
        plt.plot(time, angles_manual, label="PID Ajustado Manualmente (Kp=30, Ki=5.52, Kd=3.66)")
        plt.plot(time, angles_optimized, label=f"PID Optimizado (Kp={Kp_opt}, Ki={Ki_opt}, Kd={Kd_opt})")
        plt.title('Comparación de Respuestas del Sistema')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Ángulo (º)')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    # Crear la clase Main y ejecutar el flujo completo
    main_app = Main()
    main_app.run()
