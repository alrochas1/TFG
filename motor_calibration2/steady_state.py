# Lee los datos del serial monitor y los guarda en un archivo csv. Despues hace una
# media para cada valor de PWM y lo guarda en otro csv.
# Todos los csv se guardan en ./Data/{archivo}.csv


import serial
import csv
import os
import numpy as np

# Abre el puerto serial
arduino_port = '/dev/ttyACM0'  # Puerto serial de Arduino en Linux
baud_rate = 115200

try:
    serialConexion = serial.Serial(arduino_port, baud_rate)
    print('Conexión establecida con Arduino en el puerto', arduino_port)
except serial.SerialException as e:
    print('Error al conectar con Arduino:', e)
    exit()


# Función para encontrar el nombre de archivo único
def unique_file_name(base_name, extension):
    index = 1
    while True:
        file_name = f"{base_name}{index}.{extension}"
        if not os.path.exists(file_name):
            return file_name
        index += 1

# Parametros del programa
csv_file = unique_file_name('./Data/data', 'csv')
csv_header = ['Rueda Izq', 'Rueda Dere', 'PWM']
N_data = len(csv_header)
data_array = []
# ----------------


# Función para escribir en el archivo CSV
def write_to_csv(data):
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_header)  # Escribe los encabezados en el archivo CSV
        writer.writerows(data)  # Escribe los datos en el archivo CSV


try:
    while True:
        line = serialConexion.readline().decode().strip()  # Lee una línea de datos del puerto serial
        print('Dato recibido:', line)

        # Divide la línea en tiempo y dato
        data = line.split()
        if len(data) == N_data:
            data_array.append(data)  # Agrega los datos al array

except KeyboardInterrupt:   # Ctrl + C para cerrar
    print('Programa interrumpido. Cerrando conexión y archivo CSV.')
    serialConexion.close()  # Cierra la conexión serial al finalizar el programa

# Escribe los datos almacenados en el array en el archivo CSV antes de finalizar el programa
write_to_csv(data_array)

data_array = np.array(data_array)      # Convertir data_array en un array NumPy
data_array = data_array.astype(float)  # Convertir a tipo de datos float
PWM_VALUES = data_array[:, 2]
PWM_VALUES, indices = np.unique(PWM_VALUES, return_index=True)    # Devuelve los valores unicos de PWM y sus posiciones

data_processed = np.zeros((len(PWM_VALUES), 3));
indices = np.append(indices, len(data_array[:, 1]))
for i in range(0, len(PWM_VALUES)):
    j = indices[i]
    while j < indices[i+1]:
        data_processed[i, 0] = data_processed[i, 0] + data_array[j, 0];
        data_processed[i, 1] = data_processed[i, 1] + data_array[j, 1];
        j=j+1
    data_processed[i, 0] = data_processed[i, 0]/(j-indices[i])
    data_processed[i, 1] = data_processed[i, 1]/(j-indices[i])
    data_processed[i, 2] = PWM_VALUES[i]


csv_file = unique_file_name('./Data/steady_state', 'csv')
csv_header = ['Rueda Izq', 'Rueda Dere', 'PWM']
write_to_csv(data_processed)


