import rclpy 
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu 
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

class imu (Node): 
    def __init__(self):
        super().__init__("test_imu")
        self.s= self.create_subscription(Imu, "/bno055/imu", self.callback, 10)    
    def callback(self, data):
        quat= Quaternion()
        quat=data.orientation
        angle_x,angle_y,angle_z = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
        self.angle = (angle_z+2*math.pi)%2*math.pi

        print (self.angle)

def main(args=None):
	rclpy.init(args=args)
	gps = imu()
	rclpy.spin(gps)
	gps.destroy_node()
	rclpy.shutdown()

    
if __name__=="__main__":
    main()
    
