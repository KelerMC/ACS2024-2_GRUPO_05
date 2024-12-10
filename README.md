# ProyectoFinal_Automatizacion_G5
Intregantes:
- Benites Leiva, Yordy Marlon 
- Cruces Salhuana, Diego Axel
- Moore Salazar, Jhon Antony
- Quispe Fajardo, Adrian Ismael
- Vise Chumpitaz, Daniel Seth

# Controlador PID para Sistema de Carro Invertido

Este proyecto implementa un controlador PID para un sistema de carro invertido utilizando la librería `control` de Python. El objetivo es simular y analizar cómo diferentes combinaciones de parámetros PID afectan la respuesta del sistema, comparando las respuestas en un solo gráfico.

## Descripción del Proyecto

En este proyecto se modela un sistema de carro invertido en el que se busca controlar la posición de un carro sobre una pista utilizando un sistema de retroalimentación (feedback) PID. El controlador PID tiene tres parámetros: 
- **KP (Proporcional)**
- **KI (Integrador)**
- **KD (Derivativo)**

Se realiza una simulación para diferentes combinaciones de estos parámetros y se grafica la respuesta del sistema con el fin de analizar cuál combinación de parámetros produce el mejor desempeño.

## Funcionamiento del Código

1. **Definición de Parámetros**:
   El código define las siguientes constantes físicas del sistema:
   - `M`: Masa del carro (kg)
   - `m`: Masa del péndulo (kg)
   - `l`: Longitud del péndulo (m)
   - `g`: Aceleración debido a la gravedad (m/s²)

2. **Modelo Matemático**:
   El sistema se modela utilizando la dinámica de un carro invertido en un plano vertical. La ecuación resultante se transforma en una función de transferencia que describe la relación entre la entrada (fuerza aplicada al carro) y la salida (posición del carro).

3. **Controlador PID**:
   Un controlador PID es utilizado para intentar controlar la posición del carro sobre la pista. Los tres parámetros del controlador (KP, KI, KD) se ajustan en el sistema para observar cómo afectan la respuesta.

4. **Simulación y Comparación**:
   Se utilizan combinaciones predefinidas de parámetros PID y se simula el comportamiento del sistema para cada una. Luego, se grafican todas las respuestas en un solo gráfico, lo que permite comparar visualmente cómo cada combinación de parámetros afecta la posición del carro.

## Resultados

El código genera un gráfico único que muestra cómo la posición del carro varía a lo largo del tiempo para cada combinación de parámetros PID. Cada curva en el gráfico corresponde a una combinación diferente de `KP`, `KI`, y `KD`, y la leyenda del gráfico muestra qué parámetros fueron utilizados para cada una.

## Requisitos

Para ejecutar este código, asegúrate de tener instaladas las siguientes librerías de Python:

- `numpy`
- `matplotlib`
- `sympy`
- `control`

Puedes instalar las dependencias utilizando :

```
pip install -r requirements.txt
```
## Link de la interfaz interactiva:
 ( [https://www.canva.com/design/DAGY32gyPno/w0TLAXpDX43sVcCNskPOIg/edit] )
