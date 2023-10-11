import rclcpy
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu
from rclpy import Node 
from alvinxy import *
import numpy
import math 

class FollowGPS(Node):
	def __init__(self):
		super().__init__('follow_gps')
		
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
		self.timer = self.create_timer(0.5,self.follow)
		self.my_rover_position = self.create_subscription(NavSatFix,'gps',self.update_position,10)
		self.my_rover_angle = self.create_subscription(Imu,'Imu',self.update_angle,10)
		self.my_rover_angle = None
		self.gps_coordinates = (0.0,0.0)
		self.rover_orientation = (0.0,0.0,0.0)
		self.x_rover,self.y_rover,self.angle = 0.0,0.0,0.0
		self.orglong = 0.0
		self.orglat = 0.0
		self.twist = Twist()

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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

	def follow(self,longitude,latitud):
		self.x,self.y = ll2xy(longitude,latitud,self.orglong,self.orglat)
		angle = (np.atan2(self.y,self.x)+2*math.pi)%math.pi

		if(self.angle>angle):
			self.twist.angular.z = -0.2
			while(self.angle>angle):
				self.cmd_vel.publish(self.twist)
			self.twist.angular.z=0.0
			self.cmd_vel.publish(self.twist)
		else:
			self.twist.angular.z = 0.2
			while(self.angle>angle):
				self.cmd_vel.publish(self.twist)
			self.twist.angular.z=0.0
			self.cmd_vel.publish(self.twist)
		

	def update_angle(self,msg):
		quat = Quaternion()
		quat = msg.orientation
		angle_x,angle_y,angle_z = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		self.angle = angle_z

	

	def update_position(self,msg):
		self.gps_coordinates[0] = msg.latitud
		self.gps_coordinates[1] = msg.longitude
		x,y = ll2xy(msg.longitude,msg.latitud,self.orglong,self.orglat)
		self.x_rover= x
		self.y_rover = y

	def set_origin(orglat,orglong):
		self.orglat = orglat
		self.orglong = orglong
