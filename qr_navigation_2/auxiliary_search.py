from threading import Thread
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import rclpy
from rclpy.node import Node
from rclpy.qos import *
from .submodules.alvinxy import *
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu
from std_msgs.msg import Int8,Bool
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from custom_interfaces.srv import FollowGPS
from custom_interfaces.msg import TargetCoordinates,Coordinates
from std_msgs.msg import Float64
import numpy
import math 
import time 


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

class SearchAuxiliar(Node):
	def __init__(self):
		super().__init__("auxiliary_search")

		timer_group = MutuallyExclusiveCallbackGroup()
		listener_group = ReentrantCallbackGroup()

		self.create_subscription(Bool, "detected_aruco", self.aruco, 1,callback_group=listener_group)
		self.create_subscription(Bool,"detected_orange",self.orange,1,callback_group=listener_group)
		self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.update_coords,qos_profile_sensor_data,callback_group=listener_group)
		self.create_subscription(Imu, "/bno055/imu", self.update_angle, 10,callback_group=listener_group)    
		self.create_subscription(Int8,"/state",self.update_state,1,callback_group=listener_group)
		
		self.cmd_vel = self.create_publisher(Twist,"/cmd_vel_sr",10)
		
		
		self.twist = Twist()

		self.x_rover = 0.0
		self.y_rover = 0.0
		self.yaw_angle = 0.0

		self.angular_velocity = 0.1
		self.linear_velocity = 0.33

		self.margin = 1.0

		self.route = [(0,0),(0,-4),(-4,4),(-4,0),(-4,4),(0,4),(4,4),(4,0),(4,-4),(4,-8),(0,-8),(-4,-8),(-8,-8),(-8,-4),(-8,8),(-8,12),(-4,12),(0,12),(4,12),(8,12),(12,12)]
		self.current_point = self.route[0]
		self.found = False
		self.timer = self.create_timer(0.001,self.routine,callback_group=timer_group)

	def update_state(self,msg):
		self.state=msg.data

	def aruco(self, msg): 
		'''Sets found to true if an aruco was found'''
		self.found = msg.data
		
	def orange(self,msg):
		'''Sets found to true if an orange object is found'''
		self.found = msg.data
		
	def update_position(self):
		'''Updates the rover's position relative to the origin'''
		self.x_rover,self.y_rover = ll2xy(self.gps_coordinates[0] ,self.gps_coordinates[1] ,self.orglat,self.orglong)
	
	def update_coords(self,data):
		'''Updates the coordinates based on the data given by the GPS'''
		if(self.state in [2,3,4]):
			if(self.HAS_STARTED):
				self.orglong = data.lon/(10000000)
				self.orglat = data.lat/(10000000)
				self.HAS_STARTED = False
			self.gps_coordinates[0]=data.lat/(10000000)
			self.gps_coordinates[1]=data.lon/(10000000)
			if(not self.HAS_STARTED):
				self.update_position()

	def update_angle(self,msg):
		'''Updates the angle with the Imu's readings'''
		quat = Quaternion()
		quat = msg.orientation
		angle_x,angle_y,angle_z = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		self.yaw_angle = (angle_z+2*math.pi)%(2*math.pi)
	
	def calc_angle(self,x,y):
		'''Calculates the target angle with the target position and the current position'''
		target_angle = (math.atan2(y-self.y_rover,x-self.x_rover))
		return target_angle


	def direction_planner2(self,target_angle):
		'''Decides the best direction to rotate towards the target angle'''
		ang_error = target_angle-self.yaw_angle
		ang_error_adj=math.atan2(math.sin(ang_error),math.cos(ang_error))
		if(ang_error_adj>0):
			return -1
		return 1
			

	def angle_correction(self,target_angle):
		'''Corrects the rover's angle based on its current position and target angle'''
		self.get_logger().info("Correcting angle")
		self.twist.linear.x = 0.0
		sign = self.direction_planner2(target_angle)

		while(not ((self.yaw_angle > (target_angle-self.angle_error)) and (self.yaw_angle < (target_angle+self.angle_error)))):
			self.twist.angular.z = sign*self.angular_velocity
			self.cmd_vel.publish(self.twist) 

		self.twist.angular.z=0.0
		self.cmd_vel.publish(self.twist)

	def move_distance(self,x,y):
		relative_distance = math.sqrt(math.pow(x-self.x_rover,2)+math.pow(y-self.y_rover,2))
		while(relative_distance>1.5):
			self.twist.linear.x=self.linear_velocity
			self.cmd_vel.publish(self.twist)

	def rotate(self):
		target = ((self.yaw_angle+(math.pi))+2*math.pi)%(2*math.pi)
		target2 = ((self.yaw_angle-(math.pi))+2*math.pi)%(2*math.pi)
		self.angle_correction(target)
		self.angle_correction(target2)

	def rotate2(self):
		target = (self.yaw_angle + 2*math.pi) % (2*math.pi)
		self.angle_correction(target)
		
	def go_to_point(self,x,y):
		self.rotate2()
		self.angle_correction(self.calc_angle(x,y))
		self.move_distance(x,y)

	def routine(self):
		if(self.state == 2 or self.state==3 or self.state==4):
			if(self.current_point!=self.route[len(self.route)-1]):
				if(not self.found):
					for x,y in self.route:
						self.current_point = (x,y)
						self.go_to_point(x,y)
			if(self.current_point==self.route[len(self.route)-1]):
					for i in range(len(self.route)-1,-1,-1):
						self.current_point = self.route[i]
						self.go_to_point(x,y)
       
		
	
def main(args=None):
	rclpy.init(args=args)
	gps = SearchAuxiliar()
	executor = MultiThreadedExecutor()
	executor.add_node(gps)
	executor.spin()
	gps.destroy_node()
	rclpy.shutdown()
	
if __name__=="__main__":
	main()
