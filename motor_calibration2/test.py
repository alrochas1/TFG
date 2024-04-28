import serial
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
#import keyboard


# Función para encontrar el nombre de archivo único ----
def unique_file_name(base_name, extension):
    index = 1
    while True:
        file_name = f"{base_name}{index}.{extension}"
        if not os.path.exists(file_name):
            return file_name
        index += 1


# Función para escribir en el archivo CSV
def write_to_csv(data):
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_header)  # Escribe los encabezados en el archivo CSV
        writer.writerows(data)  # Escribe los datos en el archivo CSV

# Funcion para detectar si se ha presionado una tecla
#def key_pressed(tecla):
#    return keyboard.is_pressed(tecla)

# -------------------------------------------------

# Parametros del programa -------------------------
csv_file = unique_file_name('./Data/data', 'csv')
csv_header = ['Rueda Izq', 'Rueda Dere', 'Setpoint']
N_data = len(csv_header)
data_array = np.zeros((1, N_data))

# -----------------------------------------

# Abre el puerto serial
arduino_port = '/dev/ttyACM0'  # Puerto serial de Arduino en Linux
baud_rate = 115200

try:
    serialConexion = serial.Serial(arduino_port, baud_rate)
    print('Conexión establecida con Arduino en el puerto', arduino_port)
except serial.SerialException as e:
    print('Error al conectar con Arduino:', e)
    exit()


# Lee y manda datos a traves del puerto serial
high_value = 3        # Este es el valor mas alto del test
stepP = np.zeros((1, N_data))

try:
    while True:
        line = serialConexion.readline().decode().strip()  # Lee una línea de datos del puerto serial
        print('Dato recibido:', line)

        # Divide la línea
        data_list = line.split()
        if len(data_list) == N_data:
            data = np.array([float(x) for x in data_list])
            data_array = np.vstack([data_array, data])  # Agrega los datos al array


        if data_array[-1, 2] == high_value:  # Empieza a guardar datos
            stepP = np.vstack([stepP, data_array[-1, :]])
        else :
            if data_array[-2, 2] == high_value:   # Cuando haya probado los dos valores
                t = np.linspace(0, 0.025*len(stepP[:, 0]), len(stepP[:, 0]))
                plt.figure()
                plt.xlabel('Tiempo (s)')
                plt.ylabel('RPS')
                plt.title('Respuesta de las Ruedas con controlador')
                plt.plot(t, stepP[:, 0], label=f'Velocidad del motor izquierdo')
                plt.plot(t, stepP[:, 1], label=f'Velocidad del motor derecho')
                plt.plot(t, stepP[:, 2], label=f'Setpoint del motor')
                plt.legend()
                plt.grid(True)
                stepP = np.zeros((1, N_data))
            else :
                stepP = np.vstack([stepP, data_array[-1, :]])
                #print("Guardando dato...")

            


except KeyboardInterrupt:   # Ctrl + C para cerrar
    print('Programa interrumpido. Cerrando conexión y archivo CSV.')

    print("Pintando graficas...")
    plt.show()
    

    serialConexion.close()  # Cierra la conexión serial al finalizar el programa




# Escribe los datos almacenados en el array en el archivo CSV antes de finalizar el programa
write_to_csv(data_array)
