import pyzed as sl
import math
import numpy as np
import sys
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians

class Obstacle_evasion_test_2(Node):
    def __init__ (self):
        super().__init__("Evade_node_2")
        self.publisher_ = self.create_publisher(Bool, 'detected_obstacle', 10)
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel_fg', 10)
        self.imu = self.create_subscription(Imu, "/bno055/imu", self.callback, 10)

        self.zed = sl.camera()
        self.twist = Twist()
        self.linear_velocity = 0.16
        self.angular_velocity = 0.1
        self.x_rover, self.y_rover, self.angle = 0.0,0.0,0.0

        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE mode
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_resolution = sl.RESOLUTION.HD1080

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open: " + repr(status) + ". Exit program.")
            exit()
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD
        self.runtime_parameters.confindence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100

        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()

        self.mirror_ref = sl.Transform()
        self.mirror_ref.set_translation(sl.Translation(2.75, 4.0, 0))
        self.tr_np = self.mirror_ref.m

        self.time = self.create_timer(0.0001,self.self.timer)
        
        
    def timer(self):


        while True:
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW_LEFT)
                self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

                self.x = round(self.image.get_width()/2)
                self.y = round(self.image.get_height()/2)

                err, point_cloud_value = self.point_cloud.get_value(self.x, self.y)

                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                        point_cloud_value[1] * point_cloud_value[1] +
                                        point_cloud_value[2] * point_cloud_value[2])
                
                point_cloud_np = self.point_cloud.get_data()
                point_cloud_np.dot(self.tr_np)

                distance_f = float(distance)

                if distance_f < 0.5:
                    
                    self.twist.linear.x = 10
                    self.twist.linear.y = 10
                    self.cmd_vel.publish(self.twist)
                
                
                quat = Quaternion()
                quat = msg.orientation

                
            


    