# Sirve para procesar un video grabado con save_video.py. El video procesado debe estar en ./Images/{video}.avi
# La imagen guardada tendra el mismo nombre que el video (cuidado con no sobreescribir nada)

import cv2 as cv
import numpy as np
    

ruta_archivo = input("Ingrese la ruta del archivo de video (sin extensión): ")
cap = cv.VideoCapture(f"./Images/{ruta_archivo}.avi")
if not cap.isOpened():
    print("Error: No se pudo abrir el archivo de video.")
    exit()
res = [cap.get(cv.CAP_PROP_FRAME_WIDTH), cap.get(cv.CAP_PROP_FRAME_HEIGHT)]

n_frames = cap.get(cv.CAP_PROP_FRAME_COUNT)

final_image = np.zeros([int(res[1]), int(res[0]), 3])
N = 1


while True:

    # Capture frame-by-frame
    ret, frame = cap.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # Operaciones
    cv.imshow('Frame del video procesado', frame)

    if N == 1 :
        final_image = frame
    else :
        final_image = cv.max(final_image, frame)

    N += 1

    if cv.waitKey(1) == ord('q'):
        break


cap.release()
cv.destroyAllWindows()

cv.imshow('Imagen final', final_image)
cv.imwrite(f"./Images/{ruta_archivo}.jpg", final_image)

k = cv.waitKey(0)
cv.destroyAllWindows()



