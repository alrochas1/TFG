# Sirve para procesar un video grabado con save_video.py. El video procesado debe estar en ./Images/{video}.avi
# La imagen guardada tendra el mismo nombre que el video (cuidado con no sobreescribir nada)
#
# Esta versión realiza lo basico. Se queda con el valor maximo de la imagen.  

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

# Aqui se almacenara la imagen final
final_image = np.zeros([int(res[1]), int(res[0]), 3])
N = 1

# COMIENZA EL PROCESADO
while True:

    # Captura frame a frame
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # Muestra el frame por pantalla (se puede comentar si molesta)
    cv.imshow('Frame del video procesado', frame)

    # Se queda con el valor maximo del frame (si es el primero, no hay maximo)
    if N == 1 :
        final_image = frame
    else :
        final_image = cv.max(final_image, frame)
    N += 1

    # Para salir (si el video acaba sale solo)
    if cv.waitKey(1) == ord('q'):
        break


cap.release()
cv.destroyAllWindows()

# Muestra el resultado y guarda la imagen (usa el mismo nombre que el video)
cv.imshow('Imagen final', final_image)
cv.imwrite(f"./Images/{ruta_archivo}.jpg", final_image)

k = cv.waitKey(0)
cv.destroyAllWindows()



