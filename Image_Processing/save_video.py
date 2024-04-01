# Con este codigo se puede salvar un video para convertirlo luego en la imagen
# El video se guarda con el nombre elegido, y no sobreescribe -> ./Images/{nombre_video}x.avi

import cv2 as cv
import numpy as np
import time
import os
import subprocess

def configurar_camara(res, frames, formato_fourcc):

    indice_camara = int(input("Ingrese el índice de la cámara (usa 2 por defecto): "))
    while True:
        cap = cv.VideoCapture(indice_camara)

        if not cap.isOpened():
            print("No se pudo abrir la cámara. Mostrando cámaras disponibles:")
            # Ejecutar comando en caso de que la cámara no esté disponible
            try:
                subprocess.run(["v4l2-ctl", "--list-devices"])
            except FileNotFoundError:
                print("El comando v4l2-ctl no está disponible en este sistema.")

            indice_camara = int(input("Ingrese el índice de la cámara (usa -1 para salir): "))
            if indice_camara == -1:
                exit()
        else:
            break
    

    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*formato_fourcc))

    formato = int(cap.get(cv.CAP_PROP_FOURCC))
    formato_ascii = (formato.to_bytes(4, byteorder='little')).decode('ascii')
    print(f"Formato ASCII: {formato_ascii}")

    cap.set(cv.CAP_PROP_FRAME_WIDTH, res[0])
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, res[1])
    res = [cap.get(cv.CAP_PROP_FRAME_WIDTH), cap.get(cv.CAP_PROP_FRAME_HEIGHT)]

    cap.set(cv.CAP_PROP_FPS, frames)
    frames = cap.get(cv.CAP_PROP_FPS)

    print(f"FPS: {frames}")
    print(f"Resolucion: {res[0]} x {res[1]}")

    return cap


def configurar_video(ruta, res, FPS, formato_fourcc):

    i = 1
    nombre_archivo = f"{ruta}{i}.avi"
    while os.path.exists(nombre_archivo):
        i += 1
        nombre_archivo = f"{ruta}{i}.avi"

    fourcc = cv.VideoWriter_fourcc(*formato_fourcc)                                    # XVID es el codec recomendado para linux, MJPG puede ocupar mas
    out = cv.VideoWriter(nombre_archivo, fourcc, FPS, (int(res[0]), int(res[1])))


    return out, nombre_archivo



####### Video y propiedades ########
resolution = [1920, 1080]
FPS = 30
cap = configurar_camara(resolution, FPS, 'MJPG')
out, ruta_buffer = configurar_video('./Images/delete/buffer', resolution, FPS, 'MJPG')


nFrames = 0
contadorFrames = 0
lastTime = time.time()
initialTime = lastTime


while True:

    # Capture frame-by-frame
    ret, frame = cap.read()             # En ret devuelve un true/false

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Our operations on the frame come here
    if time.time() - lastTime > 1 :
        lastTime = time.time()
        print(f"FPS reales: {contadorFrames}")
        contadorFrames = 1
    else:
        contadorFrames += 1

    if out is not None:
        out.write(frame)
        nFrames += 1

    cv.imshow('Video', frame)    

    

    if cv.waitKey(1) == ord('q'):
        break


# Liberar todo y close all
cap.release()
out.release()
cv.destroyAllWindows()

videoDuration = lastTime - initialTime
print(f"Numero Total de Frames: {nFrames}")
print(f"Duracion del video: {videoDuration}")




# Aqui vuelve a procesar el video, para que este al numero de fps adecuado ####
print("\n")
cap = cv.VideoCapture(ruta_buffer)
FPS = nFrames/videoDuration

nombre_archivo = input("Ingrese el nombre del archivo de video: ")
nombre_archivo = f"./Images/{nombre_archivo}"
video, ruta_video = configurar_video(nombre_archivo, resolution, FPS, 'MJPG')

if not cap.isOpened():
    print("Cannot open video")
    print(ruta_buffer)
    exit()

print("Procesando video ...")
while True:

    # Capture frame-by-frame
    ret, frame = cap.read()             # En ret devuelve un true/false

    # if frame is read correctly ret is True
    if not ret:
        print("Video Finalizado")
        break

    video.write(frame)

# When everything done, release the capture
cap.release()
video.release()
cv.destroyAllWindows()

print(f"HECHO. Video guardado como {ruta_video}")
print("\n")

