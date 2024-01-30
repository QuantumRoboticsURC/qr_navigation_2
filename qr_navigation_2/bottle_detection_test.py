import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Header
from geometry_msgs.msg import Image
from custom_interfaces.msg import CA
from cv_bridge import CvBridge
import pyzed.sl as sl
import numpy as np
import time
import cv2
import math
from ultralytics import YOLO
import logging


class Detect_Bottle(Node):

    def __init__(self):
        super().__init__("bottle_node")
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.center_approach = self.create_publisher(CA, "center_approach", 10)
        self.found = self.create_publisher(Bool, "detected_bottle", 1)
        self.CA = CA()
        self.bridge = CvBridge()

        self.vel_x = 0.33
        self.vel_y = 0
        self.vel_theta = 0.1

        self.x = 0
        self.y = 0
        self.distance = None
        self.posicion = ""
        self.contador = 0
        self.bottle_dis = False
        self.is_center = False

        self.zed = sl.Camera()

        logging.getLogger('ultralytics').setLevel(logging.CRITICAL)

        # Crear un objeto InitParameters y establecer parámetros de configuración
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Usar modo de profundidad ULTRA
        self.init_params.coordinate_units = sl.UNIT.MILLIMETER  # Usar unidades de milímetros (para medidas de profundidad)

        # Abrir la cámara ZED
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open: " + repr(status) + ". Exit program.")
            exit()

        self.runtime_parameters = sl.RuntimeParameters()
        self.image_zed = sl.Mat()
        self.point_cloud = sl.Mat()

        self.cap = cv2.VideoCapture(0)
        self.model = YOLO("yolov8m.pt")

        self.timer = self.create_timer(0.001, self.detect)

    def bottle_display(self, frame, bboxes, classes):
        if bboxes.any():
            self.bottle_dis = True
            self.contador += 1

            for bbox, cls in zip(bboxes, classes):
                if cls == 39:
                    (x, y, x2, y2) = bbox

                    cv2.rectangle(frame, (x, y), (x2, y2), (0, 0, 225), 2)
                    cv2.putText(frame, "plastic_bottle", (x, y - 5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 225), 2)

                    cx = int(x + (x2 - x) / 2.0)
                    cy = int(y + (y2 - y) / 2.0)
                    print(f"x,y: {cx}, {cy}")
                    self.x = cx
                    self.y = cy
                    print("Botella detectada")

        else:
            self.bottle_dis = False
            self.contador = 0
            print("Botella no detectada")

        return frame

    def cv2_to_imgmsg(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        return msg

    def detect(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Recuperar la imagen izquierda
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            image_zed_ocv = self.image_zed.get_data()

            # Recuperar el mapa de profundidad. La profundidad está alineada en la imagen izquierda.
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

            results = self.model(image_zed_ocv)

            result = results[0]
            bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
            classes = np.array(result.boxes.cls.cpu(), dtype="int")

            detected_bottle = self.bottle_display(image_zed_ocv, bboxes, classes)

            if bboxes.any():

                ht, wd = image_zed_ocv.shape[:2]

                # Encontrar el contorno más grande
                max_bbox = max(bboxes, key=lambda box: (box[2] - box[0]) * (box[3] - box[1]))

                # Calcular el centro de la botella
                centro_objeto = ((max_bbox[0] + max_bbox[2]) // 2, (max_bbox[1] + max_bbox[3]) // 2)

                # Calcular el centro de la imagen
                centro_imagen = (wd // 2, ht // 2)

                # Determinar la posición relativa
                if centro_objeto[0] < centro_imagen[0] - 50:
                    self.posicion = "A la izquierda"
                    print("A la izquierda")
                elif centro_objeto[0] > centro_imagen[0] + 50:
                    self.posicion = "A la derecha"
                    print("A la derecha")
                else:
                    self.posicion = "Centrado"
                    print("Centrado")

                # Dibujar un rectángulo alrededor de la botella
                cv2.rectangle(image_zed_ocv, (max_bbox[0], max_bbox[1]), (max_bbox[2], max_bbox[3]), (0, 255, 0), 2)

                cv2.putText(image_zed_ocv, 'Botella', (max_bbox[0], max_bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 255, 0), 2)

                # Publicar información sobre la botella detectada
                err, point_cloud_value = self.point_cloud.get_value(self.x, self.y)
                if math.isfinite(point_cloud_value[2]):
                    detected = Bool()
                    if self.contador >= 2:
                        detected.data = True
                        self.found.publish(detected)
                        self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                            point_cloud_value[1] * point_cloud_value[1] +
                                            point_cloud_value[2] * point_cloud_value[2])
                        print(f"Distance to Bottle at {{{self.x};{self.y}}}: {self.distance}")
                        print(f"Contador: {self.contador}")
                        
                        self.x_zed = round(self.image.get_width() / 2)
                        self.y_zed = round(self.image.get_height() / 2)
                        cv2.circle(detected_bottle, (self.x_zed, self.y_zed),4,(0,0,255),-1)
                        
                        print(f"x_z: {self.x_zed} y_z: {self.y_zed}")
                        
                        self.CA.distance = self.distance
                        self.CA.x = self.x - self.x_zed
                        if self.x > (self.x_zed+20):
                            print(f"Botella a la derecha por: {self.x_zed - self.x} pixeles")
                            self.CA.detected = False
                        elif self.x < (self.x_zed-20):
                            print(f"Botella a la izquierda por: {self.x - self.x_zed} pixeles")
                            self.CA.detected = False
                        elif self.x >= (self.x_zed-20) and self.x <= (self.x_zed+20):
                            print(f"Botella al centro")
                            cv2.putText(detected_bottle, f"Centro", (self.x, self.y -80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                            self.CA.detected = True
                        self.center_approach.publish(self.CA)
                    else:
                        self.distance=None
                        print("Not detected ",self.bottle_dis)

                cv2.putText(detected_bottle, f"Distancia: {self.distance}", (max_bbox[0], max_bbox[1] - 64), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.putText(detected_bottle, f"Posicion: {self.posicion}", (max_bbox[0], max_bbox[1] - 37), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                self.cv2_to_imgmsg(detected_bottle)
                self.publisher_.publish(self.cv2_to_imgmsg(detected_bottle))
                self.get_logger().info("Publicando video")
                
            else:      
                self.cv2_to_imgmsg(detected_bottle)
                self.publisher_.publish(self.cv2_to_imgmsg(detected_bottle))
                self.get_logger().info("Publicando video sin deteccion")
                    
def main(args=None):
    rclpy.init(args=args)
    detect = Detect_Bottle()
    rclpy.spin(detect)
    detect.destroy_node()
    rclpy.shutdown()
            
if __name__=="__main__":
    main()