import rclpy
import argparse
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Header
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from custom_interfaces.msg import CA
import pyzed.sl as sl
from cv_bridge import CvBridge
import numpy as np
import time
import cv2
import math
from ultralytics import YOLO

class Detect_Bottle(Node):

    def __init__(self):
        super().__init__("bottle_node")
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.center_approach = self.create_publisher(CA, "center_approach", 10)
        self.found = self.create_publisher(Bool, "detected_bottle", 1)
        self.CA = CA()
        self.bridge = CvBridge()

        parser = argparse.ArgumentParser()
        parser.add_argument('--svo', type=str, default=None, help='optional svo file')
        args = parser.parse_args()

        self.vel_x = 0.33
        self.vel_y = 0
        self.vel_theta = 0.1
        self.model = YOLO("yolov8n.pt")

        self.x = 0
        self.y = 0
        self.distance = None
        self.posicion = ""
        self.contador = 0
        self.bottle_dis = False
        self.is_center = False

        self.zed = sl.Camera()

        input_type = sl.InputType()
        if args.svo is not None:
            input_type.set_from_svo_file(args.svo)

        # Crear un objeto InitParameters y establecer parámetros de configuración
        self.init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA # Usar modo de profundidad ULTRA
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

        self.init_params.coordinate_units = sl.UNIT.MILLIMETER # Usar unidades de milímetros (para medidas de profundidad)

        # Abrir la cámara ZED
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open: " + repr(status) + ". Exit program.")
            exit()

        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()

        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()

        self.image_ocv = self.image.get_data()
        self.image_ocv = self.image_ocv[:, :-1]
        self.depth_ocv = self.image.get_data()

        self.mirror_ref = sl.Transform()
        self.mirror_ref.set_translation(sl.Translation(2.75, 4.0, 0))

        self.timer = self.create_timer(1e-90, self.detect)

    def bottle_display(self, frame, bboxes, classes):
        if bboxes.any():
            self.bottle_dis = True
            self.contador += 1

            for bbox, cls in zip(bboxes, classes):
                if cls == 39:
                    (x, y, x2, y2) = bbox

                    # Dibujar un rectángulo alrededor de la botella
                    cv2.rectangle(frame, (x, y), (x2, y2), (0, 255, 0), 2)

                    cv2.putText(frame, 'Botella', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

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

    def cv2_to_imgmsg_resized(self, frame, scale_percent):
        widht = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (widht, height)
        resized_image = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
        msg = self.bridge.cv2_to_imgmsg(resized_image, encoding = "bgr8")
        return msg

    def detect(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Recuperar la imagen izquierda
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.image_ocv = self.image.get_data()
            self.image_col = cv2.cvtColor(self.image_ocv, cv2.COLOR_BGRA2RGB)

            # Recuperar el mapa de profundidad. La profundidad está alineada en la imagen izquierda.
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

            results = self.model(self.image_col)

            result = results[0]
            bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
            classes = np.array(result.boxes.cls.cpu(), dtype="int")

            detected_bottle = self.bottle_display(self.image_col, bboxes, classes)

            if bboxes.any():
                ht, wd = self.image_col.shape[:2]

                for bbox, cls in zip(bboxes, classes):
                    # Solo procesar detecciones de clase 39 (botella) <IMPORTANTE>
                    if cls == 39:
                        
                        # Encontrar el contorno más grande
                        max_bbox = bbox

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

                        cv2.putText(detected_bottle, f"Distancia: {self.distance}", (max_bbox[0], max_bbox[1] - 64),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        cv2.putText(detected_bottle, f"Posicion: {self.posicion}", (max_bbox[0], max_bbox[1] - 37),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        self.publisher_.publish(self.cv2_to_imgmsg_resized(detected_bottle, 30))
                        self.get_logger().info("Publicando video")

                    else:
                        self.distance = None
                        print("Not detected ", self.bottle_dis)
                        self.publisher_.publish(self.cv2_to_imgmsg_resized(detected_bottle, 30))
                        self.get_logger().info("Publicando video sin deteccion")

            else:
                self.publisher_.publish(self.cv2_to_imgmsg_resized(detected_bottle, 30))
                self.get_logger().info("Publicando video sin deteccion")

def main(args=None):
    rclpy.init(args=args)
    detect = Detect_Bottle()
    rclpy.spin(detect)
    detect.zed.close()
    detect.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
