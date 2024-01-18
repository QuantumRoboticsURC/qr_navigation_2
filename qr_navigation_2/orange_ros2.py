import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Int8,Float64,Header,Int32
from geometry_msgs.msg import Twist,Point
from sensor_msgs.msg import Image
from custom_interfaces.msg import CA
from cv_bridge import CvBridge
import numpy as np
import time
import cv2
import pyzed.sl as sl
import math

class Detect_Object(Node):

    def __init__(self):
        super().__init__("orange_node")
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.center_approach = self.create_publisher(CA,"center_approach",10)
        self.twist = Twist()
        self.found = self.create_publisher(Bool, "detected_orange", 1)
        self.CA = CA()
        self.bridge = CvBridge()
        
        self.vel_x = 0.33
        self.vel_y = 0
        self.vel_theta = 0.1
        
        self.x = 0
        self.y = 0
        self.distance = None
        self.contador = 0
        self.orange_dis_dis = False
        self.is_center = False

        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
        self.init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

        # Open the camera
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
            print("Camera Open : "+repr(status)+". Exit program.")
            exit()

        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        
        self.image_ocv = self.image.get_data()
        self.image_ocv = self.image_ocv[:,:-1]
        self.depth_ocv = self.image.get_data()
        
        self.mirror_ref = sl.Transform()
        self.mirror_ref.set_translation(sl.Translation(2.75,4.0,0)) 

        self.timer = self.create_timer(0.001,self.detect)

    def orange_display(self, contours, image):

        if contours:
            self.orange_dis = True
            self.contador += 1
            
            for (idx, contour) in enumerate(contours):
                x, y, w, h = cv2.boundingRect(contour)

                corners = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype = np.int32)
                corners = corners.reshape((-1, 1, 2))

                cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                cx = int(x + w / 2.0)
                cy = int(y + h / 2.0)
                cv2.circle(image, (cx, cy),4, (0, 0, 255), -1)
                print(f"x,y: {cx}, {cy}")
                self.x = cx
                self.y = cy

        else:
            self.orange_dis = False
            self.contador = 0
            print("Objeto naranja no detectado")

        return image

    def cv2_to_imgmsg(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding = 'bgra8')
        return msg

    def nothing(x):
        pass

    def trackbars(self, nothing):
        # Crear una ventana para los controles deslizantes
        cv2.namedWindow('Trackbars')

        # Crear trackbars para los umbrales de color en el espacio de color HSV
        cv2.createTrackbar('orangeLowHue', 'Trackbars', 0, 179, nothing)
        cv2.createTrackbar('orangeHighHue', 'Trackbars', 22, 179, nothing)
        cv2.createTrackbar('orangeLowSaturation', 'Trackbars', 130, 255, nothing)
        cv2.createTrackbar('orangeHighSaturation', 'Trackbars', 255, 255, nothing)
        cv2.createTrackbar('orangeLowValue', 'Trackbars', 160, 255, nothing)
        cv2.createTrackbar('orangeHighValue', 'Trackbars', 255, 255, nothing)

    def detect(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.image_ocv = self.image.get_data()
            # Retrieve depth map. Depth is aligned on the left image
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            self.depth_ocv = self.depth.get_data()
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

            detected_orange = self.orange_display(self.image_ocv)

            tresh = 100
            while True:
                ret, img = self.image_ocv()
                ret, img2 = self.image_ocv()

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

                    err, point_cloud_value = self.point_cloud.get_value(self.x, self.y)
                    #distance = 0
                    if math.isfinite(point_cloud_value[2]):
                        detected = Bool()
                        if self.contador >= 2:
                            detected.data = True
                            self.found.publish(detected)
                            self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                                point_cloud_value[1] * point_cloud_value[1] +
                                                point_cloud_value[2] * point_cloud_value[2])
                            print(f"Distance to Object at {{{self.x};{self.y}}}: {self.distance}")
                            print(f"Contador: {self.contador}")
                            
                            self.x_zed = round(self.image.get_width() / 2)
                            self.y_zed = round(self.image.get_height() / 2)
                            cv2.circle(detected_orange, (self.x_zed, self.y_zed),4,(0,0,255),-1)
                            
                            print(f"x_z: {self.x_zed} y_z: {self.y_zed}")
                            
                            self.CA.distance = self.distance
                            self.CA.x = self.x - self.x_zed
                            if self.x > (self.x_zed+20):
                                print(f"Objeto a la derecha por: {self.x_zed - self.x} pixeles")
                                self.CA.detected = False
                            elif self.x < (self.x_zed-20):
                                print(f"Objeto a la izquierda por: {self.x - self.x_zed} pixeles")
                                self.CA.detected = False
                            elif self.x >= (self.x_zed-20) and self.x <= (self.x_zed+20):
                                print(f"Objeto al centro")
                                cv2.putText(detected_orange, f"Centro", (self.x, self.y -80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                                self.CA.detected = True
                            self.center_approach.publish(self.CA)
                        else:
                            self.distance=None
                            print("Not detected ",self.orange_dis)

                    cv2.putText(detected_markers, f"Distancia: {self.distance}", (self.x, self.y -70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    self.cv2_to_imgmsg(detected_orange)
                    self.publisher_.publish(self.cv2_to_imgmsg(detected_orange))
                    self.get_logger().info("Publicando video")
                    
        def main(args=None):
            rclpy.init(args=args)
            detect = Detect_Object()
            rclpy.spin(detect)
            detect.destroy_node()
            rclpy.shutdown()
            
        if __name__=="__main__":
            main()

                    




