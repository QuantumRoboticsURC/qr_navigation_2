import rclpy
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu
from rclpy import Node 
from .submodules.alvinxy import *
from ublox_msgs.msg import UBXNavHPPosLLH
import numpy
import math 
from custom_interfaces.srv import FollowGPS

class FollowGPS(Node):
	def __init__(self):
		super().__init__('follow_gps')
		#Probably will be replaced for a service 
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
		self.srv = self.create_service(FollowGPS, 'follow_gps', self.FollowGPS_callback)
		self.my_rover_position = self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.update_position,10)
		self.my_rover_angle = self.create_subscription(Imu,'imu',self.update_angle,10)
		self.twist = Twist()

		self.gps_coordinates = [0.0,0.0]
		self.rover_orientation = [0.0,0.0,0.0]
		self.x_rover,self.y_rover,self.angle = 0.0,0.0,0.0

		self.orglong = 0.0
		self.orglat = 0.0

		self.linear_velocity = 0.33
		self.angular_velocity = 0.2


	def FollowGPS_callback(self,request,response):
		
		x,y = ll2xy(request.latitude,request.longitude,self.orglat,self.orglong)
		target_angle = (np.arctan2(y,x)+2*math.pi)%2*math.pi
		if(self.angle>target_angle):
			while(self.angle>target_angle):
				self.twist.angular.z = -(abs((self.angle - 0)) * (self.angular_velocity- 0.08) / (2*math.pi - 0) + 0.08)
				self.cmd_vel.publish(self.twist)
		else:
			while(self.angle<target_angle):
				self.twist.angular.z = (abs((self.angle - 0)) * (self.angular_velocity- 0.08) / (2*math.pi - 0) + 0.08)
				self.cmd_vel.publish(self.twist)
		
		self.twist.angular.z=0.0
		self.cmd_vel.publish(self.twist)

		distance = math.sqrt(math.pow(x-self.x_rover,2)+math.pow(y-self.y_rover))
		control = distance
		
		while(distance>0):
			distance = math.sqrt(math.pow(x-self.x_rover,2)+math.pow(y-self.y_rover,2))
			self.twist.linear.x = (abs((distance - 0)) * (self.linear_velocity- 0.08) / (control - 0) + 0.08)
			self.cmd_vel.publish(self.twist)

		self.twist.linear.x = 0.0
		self.cmd_vel.publish(self.twist)
		self.orglong = self.gps_coordinates[1]
		self.orglat = self.gps_coordinates[0]
		response.arrived = True 
  
		return response

	def euler_from_quaternion(self,x, y, z, w):
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
	
	def update_angle(self,msg):
		quat = Quaternion()
		quat = msg.orientation
		angle_x,angle_y,angle_z = self.euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		self.angle = (np.arctan2(angle_z)+2*math.pi)%2*math.pi
  


	def update_position(self,msg):
		self.gps_coordinates[0] = msg.latitude
		self.gps_coordinates[1] = msg.longitude
		self.x_rover,self.y_rover = ll2xy(msg.latitude,msg.longitude,self.orglat,self.orglong)


	def set_origin(self,orglat,orglong):
		self.orglat = orglat
		self.orglong = orglong

	def set_target_coordinates(self,latitude,longitude):
		self.target_latitude=latitude
		self.target_longitude=longitude

def main(args=None):
    gps = FollowGPS()
    rclpy.init(args=args)
    rclpy.spin(gps)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
    
