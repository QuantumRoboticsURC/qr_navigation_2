import cv2
import numpy as np
import pyzed.sl as sl
import math
import sys

# Inicializar la cámara ZED
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.METER
init_params.camera_resolution = sl.RESOLUTION.HD720

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

runtime_parameters = sl.RuntimeParameters()
#runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD
#runtime_parameters.confidence_threshold = 100
#runtime_parameters.textureness_confidence_threshold = 100


#image_size = zed.get_camera_information().camera_resolution
image_size = zed.get_camera_information().camera_configuration.resolution
image_size.width = image_size.width // 2
image_size.height = image_size.height // 2

image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
point_cloud = sl.Mat()



# Inicializar la cámara OpenCV
#cap = cv2.VideoCapture("/dev/video1")

# Crear ventana para controles deslizantes
cv2.namedWindow('Trackbars')

# Crear trackbars para umbrales de color en espacio de color HSV
cv2.createTrackbar('orangeLowHue', 'Trackbars', 0, 179, lambda x: None)
cv2.createTrackbar('orangeHighHue', 'Trackbars', 22, 179, lambda x: None)
cv2.createTrackbar('orangeLowSaturation', 'Trackbars', 130, 255, lambda x: None)
cv2.createTrackbar('orangeHighSaturation', 'Trackbars', 255, 255, lambda x: None)
cv2.createTrackbar('orangeLowValue', 'Trackbars', 160, 255, lambda x: None)
cv2.createTrackbar('orangeHighValue', 'Trackbars', 255, 255, lambda x: None)

thresh = 100

i = 0
while True:
    # Capturar un fotograma del flujo de video ZED
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, image_size)
        #image_ocv = zed.retrieve_image(sl.VIEW.LEFT, sl.MEM.CPU, image_size).get_data()
        image_ocv = image_zed.get_data()


        # Capturar un fotograma del flujo de video OpenCV
        #ret, img = cap.read()
        #ret, img2 = cap.read()

        # Convertir el fotograma al espacio de color HSV
        frame_hsv = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2HSV)

        # Obtener los valores actuales de los trackbars
        orange_low_hue = cv2.getTrackbarPos('orangeLowHue', 'Trackbars')
        orange_high_hue = cv2.getTrackbarPos('orangeHighHue', 'Trackbars')
        orange_low_saturation = cv2.getTrackbarPos('orangeLowSaturation', 'Trackbars')
        orange_high_saturation = cv2.getTrackbarPos('orangeHighSaturation', 'Trackbars')
        orange_low_value = cv2.getTrackbarPos('orangeLowValue', 'Trackbars')
        orange_high_value = cv2.getTrackbarPos('orangeHighValue', 'Trackbars')

        lower_orange = np.array([orange_low_hue, orange_low_saturation, orange_low_value])
        upper_orange = np.array([orange_high_hue, orange_high_saturation, orange_high_value])

        mask = cv2.inRange(frame_hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            centro_objeto = (x + w // 2, y + h // 2)

            # Definir las variables de recorte
            crp_c0 = x - thresh if x - thresh > 0 else 0
            crp_c1 = x + w + thresh if x + w + thresh < image_size.width else image_size.width
            crp_r0 = y - thresh if y - thresh > 0 else 0 
            crp_r1 = y + h + thresh if y + h + thresh < image_size.height else image_size.height

            # Obtener la posición 3D del objeto naranja en el punto central
            err,point_value = point_cloud.get_value(centro_objeto[0], centro_objeto[1])
            x_3d,y_3d,z_3d,_ = point_value
            # Calcular la distancia euclidiana tridimensional
            distancia_objeto = math.sqrt(x_3d**2 + y_3d**2 + z_3d**2)

            # Imprimir la posición relativa y la distancia al objeto naranja
            print("Distancia al objeto naranja: {:1.3} m".format(distancia_objeto), end="\r")
            if centro_objeto[0] < image_size.width // 2 - 50:
                print("A la izquierda")
            elif centro_objeto[0] > image_size.width // 2 + 50:
                print("A la derecha")
            else:
                print("Centrado")

            # Dibujar un rectángulo alrededor del objeto naranja
            cv2.rectangle(image_ocv, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Escribir "Color Naranja" sobre el rectángulo
            cv2.putText(image_ocv, 'Naranja', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow("Image", image_ocv)

            # Mostrar el fotograma original con las detecciones y centrado
           # cv2.imshow('Center Stage', img2[crp_r0:crp_r1, crp_c0:crp_c1])
            #cv2.imshow('Camera with Orange Detection', img)

        

        # Salir del bucle si se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Liberar la cámara ZED y cerrar todas las ventanas
zed.close()
cv2.destroyAllWindows()
cap.release()