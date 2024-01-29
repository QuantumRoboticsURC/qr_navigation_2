import cv2
import numpy as np

# Inicializar la cámara
cap = cv2.VideoCapture(0)

def nothing(x):
    pass

# Crear una ventana para los controles deslizantes
cv2.namedWindow('Trackbars')

# Crear trackbars para los umbrales de color en el espacio de color HSV
cv2.createTrackbar('orangeLowHue', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('orangeHighHue', 'Trackbars', 22, 179, nothing)
cv2.createTrackbar('orangeLowSaturation', 'Trackbars', 130, 255, nothing)
cv2.createTrackbar('orangeHighSaturation', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('orangeLowValue', 'Trackbars', 160, 255, nothing)
cv2.createTrackbar('orangeHighValue', 'Trackbars', 255, 255, nothing)

thresh = 100
while True:
    # Capturar un fotograma del flujo de video
    ret, img = cap.read()
    ret, img2 = cap.read()

    if ret:
        ht, wd = img.shape[:2]
        ht, wd = img2.shape[:2]

        # Convertir el fotograma al espacio de color HSV
        frame_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        frame_hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

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

        # Actualizar la imagen incluso si no hay contornos
        img = np.copy(img2)

        # Definir las variables crp_r0, crp_r1, crp_c0, crp_c1
        crp_c0 = 0
        crp_c1 = wd
        crp_r0 = 0
        crp_r1 = ht

        if contours:
            # Encontrar el contorno más grande
            max_contour = max(contours, key=cv2.contourArea)

            # Obtener el rectángulo delimitador
            x, y, w, h = cv2.boundingRect(max_contour)

            # Calcular el centro del objeto naranja
            centro_objeto = (x + w // 2, y + h // 2)

            # Calcular el centro de la imagen
            centro_imagen = (wd // 2, ht // 2)

            # Definir las variables crp_r0, crp_r1, crp_c0, crp_c1
            crp_c0 = x - thresh if x - thresh > 0 else 0
            crp_c1 = x + w + thresh if x + w + thresh < wd else wd
            crp_r0 = y - thresh if y - thresh > 0 else 0 
            crp_r1 = y + h + thresh if y + h + thresh < ht else ht

            # Determinar la posición relativa
            if centro_objeto[0] < centro_imagen[0] - 50:
                posicion = "A la izquierda"
                print("A la izquierda")
            elif centro_objeto[0] > centro_imagen[0] + 50:
                posicion = "A la derecha"
                print("A la derecha")
            else:
                posicion = "Centrado"
                print("Centrado")

            # Dibujar un rectángulo alrededor del objeto naranja
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.putText(img, 'Naranja', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Dibujar rectángulo de información
        rect_height = 80
        rect_top_left = (10, 10)
        rect_bottom_right = (wd - 10, rect_height)
        rect_thickness = -1  # Relleno del rectángulo
        rect_alpha = 0.3  # Opacidad del rectángulo

        # Crear una máscara para el rectángulo
        mask_rect = np.zeros_like(img)
        cv2.rectangle(mask_rect, rect_top_left, rect_bottom_right, (255, 255, 255), rect_thickness)

        # Fusionar la imagen original y la máscara del rectángulo
        img = cv2.addWeighted(img, 1.0, mask_rect, rect_alpha, 0)

        cv2.putText(img, f"Posicion: {posicion if contours else 'null'}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv2.putText(img, f"Objeto naranja encontrado: {'Si' if contours else 'No'}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        # Dibujar un rectángulo centrado en la parte superior
        cv2.rectangle(img, (10, 10), (wd - 10, 80), (0, 255, 0), 2, cv2.LINE_AA)

        # Mostrar el fotograma original con las detecciones y centrado
        cv2.imshow('Center Stage', img[crp_r0:crp_r1, crp_c0:crp_c1])
        cv2.imshow('Camera with Orange Detection', img)

    # Salir del bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar todas las ventanas
cap.release()
cv2.destroyAllWindows()