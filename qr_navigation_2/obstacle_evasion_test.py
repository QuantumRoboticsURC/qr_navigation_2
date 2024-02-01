import rclpy
import math
import time
import cv2
from rclpy.node import Node
import pyzed.sl as sl
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool,Int8,Float64,Header,Int32
from geometry_msgs.msg import Twist,Point
from geometry_msg.msg import Quaternion

class Evade(Node):
    def __init__ (self):
        super().__init__("Evade_node")
        self.publisher_ = self.create_publisher(Bool, 'detected_obstacle', 10)
        self.s = self.create_subscription (Imu, "/bno055/imu", self.callback, 10)


zed = sl.camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.MILLIMETER
        
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print("Camera Open: " + repr(status) +". Exit program.")
    exit()

runtime_parameters = sl.RuntimeParameters()

i = 0
image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()
depth_ocv = image.get_data()
mirror_ref = sl.Transform()
mirror_ref.set_translation(sl.Translation(2.75, 4.0, 0))
distance = 0

while True:
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        image_ocv = image.get_data()
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        depth_ocv = depth.get_data()
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        grayimg = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2GRAY)

        err, point_cloud_value = point_cloud.get_value(x, y)

        if math.isfinite(point_cloud_value[2]):
            distance 

