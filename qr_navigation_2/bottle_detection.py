import cv2
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from custom_interfaces.msg import CA
import pyzed.sl as sl
from cv_bridge import CvBridge

from PIL import Image as im 

import numpy as np
import math
import scipy.misc

from ultralytics import YOLO
class DetectObject(Node):

    def __init__(self):
        super().__init__('object_detection_node')
        self.publisher_image = self.create_publisher(Image, 'camera/image', 10)
        self.publisher_detection = self.create_publisher(Bool, 'detected_object', 1)
        self.publisher_center_approach = self.create_publisher(CA, 'center_approach', 10)
        self.publisher_detected_b = self.create_publisher(Bool, 'detected_b', 1)  # Nuevo publicador booleano
        self.bridge = CvBridge()
        self.x = 0.0
        self.y = 0.0
        self.model = YOLO("yolov8m.pt")
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
        self.width = self.image.get_width()
        self.height = self.image.get_height()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.image_ocv = self.image.get_data()
        self.image_ocv = self.image_ocv[:,:-1]
        self.depth_ocv = self.image.get_data()
        
        self.mirror_ref = sl.Transform()
        self.mirror_ref.set_translation(sl.Translation(2.75,4.0,0)) 
        
        self.timer = self.create_timer(0.01,self.detect)

    def cv2_to_imgmsg(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgra8')
        return msg

    def detect_object(self, frame):
        frame = np.asarray(frame)
        frame = im.fromarray(frame, "RGB")
        results = self.model(frame)
        result = results[0]
        bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
        classes = np.array(result.boxes.cls.cpu(), dtype="int")

        object_detected = False
        center_approach_msg = CA()

        for cls, bbox in zip(classes, bboxes):
            if cls == 39:  # Clase correspondiente a botella de plástico
                object_detected = True
                self.publisher_detected_b.publish(Bool(data=True))  # Publica True si se detecta una botella

                (x, y, x2, y2) = bbox
                self.x,self.y = x,y
                cv2.rectangle(frame, (x, y), (x2, y2), (0, 0, 225), 2)
                cv2.putText(frame, "plastic_bottle", (x, y - 5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 225), 2)

                center_x = (x + x2) // 2
                center_y = (y + y2) // 2
                distance = 0  

                center_approach_msg.detected = True
                center_approach_msg.distance = distance
                center_approach_msg.x = center_x
                center_approach_msg.y = center_y
                self.publisher_center_approach.publish(center_approach_msg)

        return frame, object_detected
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
    
    
            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            self.image_ocv,self.detected = self.detect_object(self.image_callback)
            err, point_cloud_value = self.point_cloud.get_value(self.x, self.y)
            #distance = 0
            
            if math.isfinite(point_cloud_value[2]):
                self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                print(f"Distance to Aruco at {{{self.x};{self.y}}}: {self.distance}")
                print(f"Contador: {self.contador}")
            self.publisher_image.publish(self.cv2_to_imgmsg(self.image_ocv))     
           
            

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Realizar la detección del objeto
        frame_detected, object_detected = self.detect_object(frame)

        # Publicar la imagen con las detecciones
        self.publisher_image.publish(self.cv2_to_imgmsg(frame_detected))

        # Publicar la información de detección
        detection_msg = Bool()
        detection_msg.data = object_detected
        self.publisher_detection.publish(detection_msg)

        
        detection_b_msg = Bool(data=object_detected)
        self.publisher_detected_b.publish(detection_b_msg)

def main(args=None):
    rclpy.init(args=args)
    detect_object_node = DetectObject()
    detect_object_node.create_subscription(Image, 'image_topic', detect_object_node.image_callback, 10)
    rclpy.spin(detect_object_node)
    detect_object_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
