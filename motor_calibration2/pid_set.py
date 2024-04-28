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
csv_header = ['Rueda Izq', 'Rueda Dere', 'Kp', 'Ki', 'Kd']
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

motor = 1;      # Motor a ajustar

stepP = np.zeros((1, N_data))
plt.figure(1)
plt.xlabel('Tiempo (s)')
plt.ylabel('RPS')
plt.title('Respuesta de la Rueda Izquierda con controlador')
try:
    while True:
        line = serialConexion.readline().decode().strip()  # Lee una línea de datos del puerto serial
        print('Dato recibido:', line)

        # Divide la línea
        data_list = line.split()
        if len(data_list) == N_data:
            data = np.array([float(x) for x in data_list])
            data_array = np.vstack([data_array, data])  # Agrega los datos al array


        if data_array[-1, motor] != 0:  # Empieza a guardar datos para pintar cuando la respuesta es mayor a 0 (comienza la grafica)
            
            stepP = np.vstack([stepP, data_array[-1, :]])
            
            if kp_value == -1:
                kp_value = str(stepP[-1, 2])  # Convertir a cadena
                ki_value = str(stepP[-1, 3])  # Convertir a cadena
                kd_value = str(stepP[-1, 4])  # Convertir a cadena

        else :  # Si es 0 es porque ha terminado un ciclo o porque ya era 0
            if len(stepP[:, motor]) > 5:    # Si ha terminado un ciclo, habra un array de valores (el 5 es el umbral, para evitar problemas de ruido)
                t = np.linspace(0, 0.025*len(stepP[:, 0]), len(stepP[:, motor]))
                if stepP[-1, 3] == 0 :
                    plt.plot(t, stepP[:, motor], label=f'Kp = {kp_value}', lw=1)
                elif stepP[-1, 4] == 0 :
                    plt.plot(t, stepP[:, motor], label=f'Kp = {kp_value}, Ki = {ki_value}', lw=1)
                else :
                    plt.plot(t, stepP[:, motor], label=f'Kp = {kp_value}, Ki = {ki_value}, Kd = {kd_value}', lw=1)

            stepP = np.zeros((1, N_data))
            # Esto es necesario para que se guarde bien el valor (seguramente hay una manera mejor)
            kp_value = -1
            ki_value = -1
            kd_value = -1

except KeyboardInterrupt:   # Ctrl + C para cerrar
    print('Programa interrumpido. Cerrando conexión y archivo CSV.')

    print("Pintando graficas...")
    plt.legend()
    plt.grid(True)
    plt.show()
    

    serialConexion.close()  # Cierra la conexión serial al finalizar el programa




# Escribe los datos almacenados en el array en el archivo CSV antes de finalizar el programa
write_to_csv(data_array)



