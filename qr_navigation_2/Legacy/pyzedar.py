import cv2
import numpy as np
import math
import pyzed.sl as sl  # Importar la librería PyZED

parametros = cv2.aruco.DetectorParameters()
diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

# Configurar la cámara ZED
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
zed = sl.Camera()
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Error al abrir la cámara ZED")
    exit(-1)

# Poner como constante el tamaño físico del marcador ArUco en centímetros
marcador_cm = 20.0

while True:
    # Capturar fotograma de la cámara ZED
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Obtener la imagen izquierda (puedes usar la derecha si prefieres)
        image_zed = sl.Mat()
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        frame = image_zed.get_data()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        esquinas, ids, _ = cv2.aruco.detectMarkers(gray, diccionario, parameters=parametros)

        if np.all(ids is not None):
            for i, esquina in enumerate(esquinas):
                id_marcador = ids[i][0]
                x, y = map(int, esquina[0][0])
                cv2.putText(frame, str(id_marcador), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                marcador_pixeles = max(esquina[0][0]) - min(esquina[0][0])
                distancia_aprox_cm = (marcador_cm * 100) / marcador_pixeles

                centro_x = int((esquina[0][0][0] + esquina[0][2][0]) / 2)
                centro_y = int((esquina[0][0][1] + esquina[0][2][1]) / 2)

                punto_izquierda = (int(esquina[0][0][0]), centro_y)
                punto_derecha = (int(esquina[0][2][0]), centro_y)

                distancia_centro_izquierda = math.sqrt((centro_x - punto_izquierda[0]) ** 2 + (centro_y - punto_izquierda[1]) ** 2)
                distancia_centro_derecha = math.sqrt((centro_x - punto_derecha[0]) ** 2 + (centro_y - punto_derecha[1]) ** 2)

                cv2.putText(frame, f'Distancia: {distancia_aprox_cm:.2f} cm', (x, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(frame, (centro_x, centro_y), 4, (0, 0, 255), -1)
                cv2.circle(frame, punto_izquierda, 4, (255, 0, 0), -1)
                cv2.circle(frame, punto_derecha, 4, (255, 0, 0), -1)

                if distancia_centro_izquierda < distancia_centro_derecha:
                    cv2.putText(frame, f'Ve a la izquierda', (150, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                elif distancia_centro_derecha < distancia_centro_izquierda:
                    cv2.putText(frame, f'Ve a la derecha', (150, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, f'Estás centrado', (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame, f'Distancia Centro a Izquierda: {distancia_centro_izquierda:.2f} px', (x, y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f'Distancia Centro a Derecha: {distancia_centro_derecha:.2f} px', (x, y + 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Frame", frame)

    # Terminar el programa con la tecla 'esc'
    k = cv2.waitKey(1)
    if k == 27:
        break

zed.close()
cv2.destroyAllWindows()
