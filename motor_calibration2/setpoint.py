# Realiza el ajuste lineal de los datos obtenidos por steady_state.py y muestra una grafica.
# Tambien muestra por la consola los valores de a_ff y b_ff.

import csv
import matplotlib.pyplot as plt
import numpy as np

# Nombre del archivo CSV
csv_file = input("Introduzca archivo .csv: ")
csv_file = f"./Data/{csv_file}"

# Listas para almacenar los datos
r_izq = []
r_der = []
PWM = []

# Leer datos desde el archivo CSV
with open(csv_file, 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Saltar la fila de encabezado si existe
    for row in reader:
        r_izq.append(float(row[0]))
        r_der.append(float(row[1]))
        PWM.append(float(row[2]))


def ajuste_lineal(x, y):
    
    # Realizar ajuste lineal
    coefficients = np.polyfit(x, y, 1)  # 1 indica ajuste lineal
    x_fit = np.linspace(min(x), max(x), 100)  # Generar valores x para el ajuste
    y_fit = np.polyval(coefficients, x_fit)  # Calcular valores y para el ajuste

    # Trazar la línea de ajuste
    plt.plot(x_fit, y_fit, label=f'Ajuste Lineal: {coefficients[0]:.2f}x + {coefficients[1]:.2f}', color='red', linestyle='--')

    return coefficients





# Hacer la gráfica izquierda
plt.figure(1)
plt.plot(r_izq, PWM)
plt.xlabel('RPS (Rueda Izquierda)')
plt.ylabel('PWM')
plt.title('Setpoint Rueda Izquierda')
m_izq, n_izq = ajuste_lineal(r_izq, PWM)
print(f'Para la rueda izquierda.    a_ff = {n_izq}, b_ff = {1/m_izq}, , x = {(PWM[-1] - PWM[0])/m_izq}')
plt.legend()
plt.grid(True)
plt.show()





# Hacer las gráfica derecha
plt.figure(2)
plt.plot(r_der, PWM)
plt.xlabel('RPS (Rueda Derecha)')
plt.ylabel('PWM')
plt.title('Setpoint Rueda Derecha')
m_der, n_der = ajuste_lineal(r_der, PWM)
print(f'Para la rueda derecha.    a_ff = {n_der}, b_ff = {1/m_der}, x = {(PWM[-1] - PWM[0])/m_der}')
plt.grid(True)
plt.legend()
plt.show()
