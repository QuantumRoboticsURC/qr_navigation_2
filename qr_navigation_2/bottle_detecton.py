import cv2
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from custom_interfaces.msg import CA
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class DetectObject(Node):

    def __init__(self):
        super().__init__('object_detection_node')
        self.publisher_image = self.create_publisher(Image, 'camera/image', 10)
        self.publisher_detection = self.create_publisher(Bool, 'detected_object', 1)
        self.publisher_center_approach = self.create_publisher(CA, 'center_approach', 10)
        self.publisher_detected_b = self.create_publisher(Bool, 'detected_b', 1)  # Nuevo publicador booleano
        self.bridge = CvBridge()
        self.model = YOLO("yolov8m.pt")

    def cv2_to_imgmsg(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgra8')
        return msg

    def detect_object(self, frame):
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
