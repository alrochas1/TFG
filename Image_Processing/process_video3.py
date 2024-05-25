# Sirve para procesar un video grabado con save_video.py. El video procesado debe estar en ./Images/{video}.avi
# La imagen guardada tendra el mismo nombre que el video (cuidado con no sobreescribir nada)
#
# Esta versión elimina el fondo de la imagen, usando BackgroundSubtractorMOG2 (ver enlace).
# Con esto crea una mascara que elimina el fondo de cada imagen y con el resultado realiza
# el mismo proceso que process_video.py (se queda con el maximo).
# Ademas, esta version aplica un filtro de la mediana para eliminar un poco ruido
# (esta version solo aplica el filtro a la imagen final, para aplicar a cada fotograma 
# usar process_video3_HARD.py)

# https://github.com/PacktPublishing/Python-Image-Processing-Cookbook/blob/master/Chapter%2001/Chapter01.ipynb
# https://docs.opencv.org/4.x/d1/dc5/tutorial_background_subtraction.html
# https://docs.opencv.org/4.x/dc/dd3/tutorial_gausian_median_blur_bilateral_filter.html

import cv2 as cv
import numpy as np
    

# Abre el video
ruta_archivo = input("Ingrese la ruta del archivo de video (sin extensión): ")
extensiones = ['.avi', '.mp4', '.mkv']

for extension in extensiones:
    cap = cv.VideoCapture(f"./Images/{ruta_archivo}{extension}")
    if cap.isOpened():
        break

if not cap.isOpened():
        print("Error: No se pudo abrir el archivo de video.")
        exit()

# Obtiene los parametros del video (resolucion y FPS)
res = [cap.get(cv.CAP_PROP_FRAME_WIDTH), cap.get(cv.CAP_PROP_FRAME_HEIGHT)]
n_frames = cap.get(cv.CAP_PROP_FRAME_COUNT)


# Carga la imagen de fondo
background = cv.imread("./Images/back.jpg")
if background is None:
    print("Error: No se pudo abrir la imagen del fondo.")
    exit()


# Aqui calcula el fondo
N = 1
backSub = cv.createBackgroundSubtractorMOG2(history=10, varThreshold=200)   #history=10, varThreshold=1024, detectShadows=True
backSub.apply(background, learningRate=1)   # Usa el fondo basico que ya tenemos

# COMIENZA EL PROCESADO
while True:

    # Captura frama a frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # Muestra el frame por pantalla (se puede comentar si molesta)
    cv.imshow('Frame del video procesado', frame)

    # Every frame is used both for calculating the foreground mask and for updating the background.
    # If you want to change the learning rate used for updating the background model, 
    # it is possible to set a specific learning rate by passing a parameter to the apply method.
    fgMask = backSub.apply(frame)

    # Aplica la máscara para obtener solo el primer plano
    frame_bs = cv.bitwise_and(frame, frame, mask=fgMask)

    #cv.imshow('Frame', frame)
    cv.imshow('Mascara', fgMask)
    cv.imshow('Fotograma con la mascara extraida', frame_bs)

    # Se queda con el valor maximo del frame (si es el primero, no hay maximo)
    # Borra el primer fotograma, ya que a veces no se procesa bien
    if N == 2 :
        final_image = frame_bs
    elif N>2:
        final_image = cv.max(final_image, frame_bs)

    print(f"{N/n_frames * 100:.2f} % ...")
    N += 1
    
    # Para salir (si el video acaba sale solo)
    if cv.waitKey(1) == ord('q'):
        break


cap.release()
cv.destroyAllWindows()

final_image = cv.medianBlur(final_image, 5)

cv.imshow('Imagen final', final_image)
cv.imwrite(f"./Images/{ruta_archivo}.jpg", final_image)

k = cv.waitKey(0)
cv.destroyAllWindows()

