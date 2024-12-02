#!/usr/bin/env python
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .submodules.constants import PIXEL_ANGLE_ERROR

class Center(Node):
    def __init__(self):
        super().__init__("Center")
        self.controller = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.midpoint = 0.0
        self.x, self.y = 0.0,0.0

        self.first_detected = False
        self.count = 0
        self.threshold =10
        self.twist = Twist()

        self.angular_velocity=0.33
        self.cap = cv2.VideoCapture(0)

# Crear una ventana para los controles deslizantes
        cv2.namedWindow('Trackbars')

    def draw(self,mask,frame):
        #En esta función se trabaja con Open CV para la detección del color de las rocas
        contornos, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contornos:
            area = cv2.contourArea(contour)
            if area > 1000:  # Filtrar contornos pequeños para eliminar el ruido
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Dibujar un rectángulo verde alrededor del objeto
                cv2.putText(frame, 'Naranja', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                self.x,self.y=x,y
                return True
        return False
    
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        self.midpoint=frame.shape[1]/2

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
            # Rectángulo verde alrededor de los objetos detectados como naranja
            detected = self.draw(mask,frame)
            if(detected):
                self.count+=1
                if(self.first_detected):
                    if self.midpoint+PIXEL_ANGLE_ERROR > self.x and self.midpoint-PIXEL_ANGLE_ERROR < self.x:
                        print("Centro")
                        self.twist.linear.x=0.0
                        self.twist.angular.z=0.0
                        #Service approach
                    elif self.midpoint > self.x:
                        print("Izquierda")
                        self.twist.linear.x=0.0
                        self.twist.angular.z=self.angular_velocity
                    elif self.midpoint < self.x:
                        print("Derecha")
                        self.twist.linear.x=0.0
                        self.twist.angular.z=-self.angular_velocity
                if(self.count>self.threshold):
                    self.first_detected=True

                self.controller.publish(self.twist)



def main(args=None):
    rclpy.init(args=args)

    p = Center()
    rclpy.spin(p)

    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


def nothing(x):
    pass

# Inicializar la cámara1


# Crear una ventana para los controles deslizantes
cv2.namedWindow('Trackbars')
#Crear trackbars para los umbrales de color en el espacio de color HSV
cv2.createTrackbar('orangeLowHue', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('orangeHighHue', 'Trackbars', 22, 179, nothing)
cv2.createTrackbar('orangeLowSaturation', 'Trackbars', 130, 255, nothing)
cv2.createTrackbar('orangeHighSaturation', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('orangeLowValue', 'Trackbars', 160, 255, nothing)
cv2.createTrackbar('orangeHighValue', 'Trackbars', 255, 255, nothing)
cv2.destroyAllWindows()