import rclpy
from rclpy.node import Node
from rclpy.qos import * 
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Thread

from .submodules.alvinxy import *
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu
from std_msgs.msg import Int8,Bool,Int32
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from custom_interfaces.msg import TargetCoordinates
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

class Follow_GPS(Node):
	
	def __init__(self):
		super().__init__('gps8')

		#State publishers to give feedback to the node controller
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel_fg',10)
		self.arrived_pub = self.create_publisher(Bool,'arrived_fg',1)
		self.state_pub = self.create_publisher(Int8,'state',1)
		#Subscribers to the node controller's publisher of the target coordinates
		self.target_coords = self.create_subscription(TargetCoordinates,"/target_coordinates",self.update_target,1)
		self.reset_coords = self.create_publisher(TargetCoordinates,"/target_coordinates",1)
		#Subscribers to the sensors' data
		self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.update_coords,qos_profile_sensor_data)
		self.create_subscription(Imu, "/bno055/imu", self.update_angle, 10)    
		#Subscriber to the state topic of the node controller
		self.create_subscription(Int8,"/state",self.update_state,1)
		self.create_subscription(Bool,"/object_detected",self.obstacle_detection,1)
		self.create_subscription(Int32,"/distance",self.distance_obstacle,1)
		self.create_subscription(Float64,"/zed_angle",self.update_zed_angle,1)

		self.zed_yaw = 0.0
		self.obstacle_distance = None
		#Velocity data
		self.twist = Twist()
		self.linear_velocity = 0.25
		self.angular_velocity = 0.1
		self.obstacle_detected = False
		
		#Coordinates and position on the plane
		self.gps_coordinates = [0.0,0.0]
		self.target_coordinates = [None,None]
		self.x_rover,self.y_rover,self.yaw_angle = 0.0,0.0,0.0
		#Target x,y coordinates
		self.x_target = 0.0
		self.y_target = 0.0
		#Origin's latitude and logitude to map the plane using the alvinxy library
		self.orglong = 0.0
		self.orglat = 0.0
		#Constant for time checker
		self.time_constant=1.0
		#Flag for the initial coordinate registered
		self.HAS_STARTED = True
		#Default value of the state
		self.state = -1
		#Errors for the distance and coordinate stoppage routines
		self.coordinate_error = 0.000005  
		self.distance_error = 1.2
		#Angle error for correction
		self.angle_error = 0.08
		#Variable for range distance
		self.range_distance = 1.0
		self.just_started = False
		#Main
		self.timer = self.create_timer(0.01,self.followGPS)
		


	def update_target(self,msg):
		'''Sets the target coordinates to the given value'''
		self.target_coordinates[0]=msg.latitude
		self.target_coordinates[1]=msg.longitude

	def update_position(self):
		'''Updates the rover's position relative to the origin'''
		self.x_rover,self.y_rover = ll2xy(self.gps_coordinates[0] ,self.gps_coordinates[1] ,self.orglat,self.orglong)
	
	def update_coords(self,data):
		'''Updates the coordinates based on the data given by the GPS'''
		if(self.HAS_STARTED):
			self.orglong = data.lon/(10000000)
			self.orglat = data.lat/(10000000)
			self.HAS_STARTED = False
		self.gps_coordinates[0]=data.lat/(10000000)
		self.gps_coordinates[1]=data.lon/(10000000)
  
		if(not self.HAS_STARTED):
			self.update_position()


	def update_state(self,msg):
		'''updates the state variable after receiving data from the controller'''
		self.state=msg.data
		
	def update_angle(self,msg):
		'''Updates the angle with the Imu's readings'''
		quat = Quaternion()
		quat = msg.orientation
		angle_x,angle_y,angle_z = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		self.yaw_angle = angle_z
		

	def calc_angle(self):
		'''Calculates the target angle with the target position and the current position'''
		target_angle = (math.atan2(self.y_target-self.y_rover,self.x_target-self.x_rover))
		return target_angle


	def direction_planner(self,target_angle):
		'''Decides the best direction to rotate towards the target angle'''
		ang_error = target_angle-self.yaw_angle
		ang_error_adj=math.atan2(math.sin(ang_error),math.cos(ang_error))
		if(ang_error_adj>0):
			return 1
		return -1
			
	
	def angle_correction(self,target_angle):
		'''Corrects the rover's angle based on its current position and target angle'''
		sign = self.direction_planner(target_angle)
		self.twist.angular.z = sign*self.angular_velocity
		self.twist.linear.x = 0.0	

	def check_coord_precision(self):
		'''Boolean expression for the coordinate precision stoppage routine'''
		var = (self.gps_coordinates[0]>(self.target_coordinates[0]-self.coordinate_error) and self.gps_coordinates[0]<(self.target_coordinates[0]+self.coordinate_error)) and (self.gps_coordinates[1]>(self.target_coordinates[1]-self.coordinate_error) and self.gps_coordinates[1]<(self.target_coordinates[1]+self.coordinate_error))
		return var

	def check_distance_precision(self):
		'''Boolean expression for the distance precision stoppage routine'''
		var = (self.x_rover>(self.x_target-self.distance_error) and self.x_rover<(self.x_target+self.distance_error)) and (self.y_rover>(self.x_target-self.distance_error) and self.y_rover<(self.y_target+self.distance_error))
		return var


	def followGPSFunction(self,target_angle,distance):
		if(not((self.yaw_angle>(target_angle-self.angle_error*2)) and (self.yaw_angle<(target_angle+self.angle_error*2)))):
			self.angle_correction(target_angle)
		elif(distance > 2.5):
			self.twist.linear.x = self.linear_velocity
			self.twist.angular.z = 0.0




	def followGPS(self):
		
		if(self.state==0): #Checks if the state is the one assigned to FGPS
			state = Int8()
			arrived = Bool()
   
			if(not self.just_started):
				self.get_logger().info("Entered Follow GPS v8.1")
				self.just_started=True
    
			if(self.target_coordinates[0]!=None and self.target_coordinates[1]!=None): #Checks that the target coordinates are not null
				self.get_logger().info(f"The target coordinates are {self.target_coordinates}")
				if(self.gps_coordinates[0]!=0.0 and self.gps_coordinates[1]!=0.0): #Checks that the gps readings are valid
					
					#calculates the target x,y using the target coords and the origin
					self.x_target,self.y_target = ll2xy(self.target_coordinates[0],self.target_coordinates[1],self.orglat,self.orglong)
					#calculates the distance between the current position and the target position
					distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
					#calculates the target angle
					target_angle = self.calc_angle()
					#Corrects the rover's position towards the target angle
					self.angle_correction(target_angle)

					self.followGPSFunction(target_angle,distance)


					self.cmd_vel.publish(self.twist)

					if self.check_coord_precision() or self.check_distance_precision():
						self.twist.linear.x = 0.0
						self.twist.angular.z = 0.0
						state.data = -1
						self.target_coordinates[0]=None
						self.target_coordinates[1]=None
						self.HAS_STARTED=False
						arrived.data=True
						self.arrived_pub.publish(arrived)
						self.state_pub.publish(state)
						self.cmd_vel.publish(self.twist)
						time.sleep(2)



def main(args=None):
	rclpy.init(args=args)
	gps = Follow_GPS()
	executor = MultiThreadedExecutor()
	executor.add_node(gps)
	executor.spin()
	gps.destroy_node()
	rclpy.shutdown()
	
if __name__=="__main__":
	main()
