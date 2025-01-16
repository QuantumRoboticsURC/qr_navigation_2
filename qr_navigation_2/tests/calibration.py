#!/usr/bin/env python
import cv2
import numpy as np


def nothing(x):
    pass

# Inicializar la cámara1
cap = cv2.VideoCapture(0)

# Crear una ventana para los controles deslizantes
cv2.namedWindow('Trackbars')

# Crear trackbars para los umbrales de color en el espacio de color HSV
cv2.createTrackbar('orangeLowHue', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('orangeHighHue', 'Trackbars', 22, 179, nothing)
cv2.createTrackbar('orangeLowSaturation', 'Trackbars', 130, 255, nothing)
cv2.createTrackbar('orangeHighSaturation', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('orangeLowValue', 'Trackbars', 160, 255, nothing)
cv2.createTrackbar('orangeHighValue', 'Trackbars', 255, 255, nothing)

while True:
    # Capturar un fotograma del flujo de video
    ret, frame = cap.read()

    if ret:
        # Convertir el fotograma al espacio de color HSV
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Obtener los valores actuales de los trackbars
        orange_low_hue = cv2.getTrackbarPos('orangeLowHue', 'Trackbars')
        orange_high_hue = cv2.getTrackbarPos('orangeHighHue', 'Trackbars')
        orange_low_saturation = cv2.getTrackbarPos('orangeLowSaturation', 'Trackbars')
        orange_high_saturation = cv2.getTrackbarPos('orangeHighSaturation', 'Trackbars')
        orange_low_value = cv2.getTrackbarPos('orangeLowValue', 'Trackbars')
        orange_high_value = cv2.getTrackbarPos('orangeHighValue', 'Trackbars')

        # Definir los umbrales de color naranja en el espacio de color HSV
        lower_orange = np.array([orange_low_hue, orange_low_saturation, orange_low_value])
        upper_orange = np.array([orange_high_hue, orange_high_saturation, orange_high_value])

        # Crear una máscara para detectar objetos de color naranja
        mask = cv2.inRange(frame_hsv, lower_orange, upper_orange)

        # Encontrar contornos en la máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Rectángulo verde alrededor de los objetos detectados como naranja
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Filtrar contornos pequeños para eliminar el ruido
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Dibujar un rectángulo verde alrededor del objeto
                cv2.putText(frame, 'Naranja', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Mostrar el fotograma original
        cv2.imshow('Frame', frame)
        #cv2.imshow('Mask', mask)

    # Salir del bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar todas las ventanas
cap.release()
cv2.destroyAllWindows()