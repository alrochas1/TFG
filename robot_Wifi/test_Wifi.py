
import struct
import sys
import socket


# def main():

#     # struct.pack() para empaquetar datos en formato binario. 
#     data=(struct.pack('d',2) + struct.pack('d',2) + struct.pack('d',2 ) 
#     + struct.pack('d', 1) + struct.pack('d', 1) + struct.pack('d', 1))
#     len = 48
#     cabecera = struct.pack('H',112) + struct.pack('H',22)+ struct.pack('H', 9) + struct.pack('H',48)
#     message=cabecera + data
#     '''data= (struct.pack('H', 112) + struct.pack('H', 22) + struct.pack('H', 9) + 
#     struct.pack('H', 48) + struct.pack('d',2) + struct.pack('d',2) + struct.pack('d',2 ) 
#     + struct.pack('d', 1) + struct.pack('d', 1) + struct.pack('d', 1) )'''

#     print(message)
#     print(struct.unpack('HHHHdddddd',message))


# if __name__ == "__main__":
#     main()

def send_message():

    # Definir el número que quieres enviar
    number = 240

    # Estructura del mensaje https://docs.python.org/3/library/struct.html
    id = 1
    op = 1      # Código de operación para saludo
    len = 8    # Longitud de datos, un double
    header = struct.pack("HHH", id, op, len)
    number_bytes = struct.pack('B', number)
    # Crear el mensaje completo
    message = header + number_bytes
    print(message)
    
    # Dirección IP y puerto del Arduino
    arduino_ip = '192.168.10.6'  # Cambiar a la dirección IP correcta
    arduino_port = 4244  # Puerto definido en el código del Arduino (destino)
    
    # Crear un socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # Enviar el mensaje al Arduino
        sock.sendto(message, (arduino_ip, arduino_port))
        print("Mensaje enviado al Arduino")
    except Exception as e:
        print("Error al enviar el mensaje:", e)
    finally:
        sock.close()

if __name__ == "__main__":
    send_message()

