import rclpy
from rclpy.node import Node
from rclpy.qos import * 
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Thread

from .submodules.alvinxy import *
from .submodules.constants import *

from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu,LaserScan
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
		self.create_subscription(Float64,'latitude',self.update_coords_latitude,10)
		self.create_subscription(Float64,'longitude',self.update_coords_longitude,10)
		self.create_subscription(Imu, "/bno055/imu", self.update_angle, 10) 
		self.create_subscription(LaserScan,"/scan",self.lidar_callback,10)
		self.create_subscription(Int8,"/state",self.update_state,1)


		#Velocity data
		self.twist = Twist()
		self.linear_velocity = 0.25
		self.angular_velocity = 0.1
		self.obstacle_detected = False
		self.obstacle_routine = -1
		self.obstacle_min_distance = 0.0
		
		#Coordinates and position on the plane
		self.gps_coordinates = [0.0,0.0]
		self.target_coordinates = [None,None]
		self.x_rover,self.y_rover,self.yaw_angle,self.pitch_angle = 0.0,0.0,0.0,0.0
		#Target x,y coordinates
		self.x_target = 0.0
		self.y_target = 0.0
		#Origin's latitude and logitude to map the plane using the alvinxy library
		self.orglong = None
		self.orglat = None
		#Flag for the initial coordinate registered
		self.HAS_STARTED = True
		#Default value of the state
		self.state = -1
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
			self.orglong = data.lon/(10000000.0)
			self.orglat = data.lat/(10000000.0)
			self.HAS_STARTED = False
		self.gps_coordinates[0]=data.lat/(10000000.0)
		self.gps_coordinates[1]=data.lon/(10000000.0)

		if(not self.HAS_STARTED):
			self.update_position()

	def update_coords_latitude(self,data):
		'''Updates the latitude of the rover's position'''
		if not self.orglat:
			self.orglat = data.data/10000000.0
		else:
			self.gps_coordinates[0] = data.data/10000000.0

		if(not self.HAS_STARTED):
			self.update_position()

	def update_coords_longitude(self,data):
		'''Updates the longitude of the rover's position'''
		if not self.orglong:
			self.orglong = data.data/10000000.0
		else:
			self.gps_coordinates[1] = data.data/10000000.0

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
		self.pitch_angle = angle_y

	def lidar_callback(self,msg):
		'''Updates the range distance with the lidar's readings'''
		ranges = msg.ranges
		angle_min = msg.angle_min  # Starting angle of the scan
		angle_increment = msg.angle_increment  # Increment per beam

		# Find the closest object
		min_distance = min(ranges)  # Minimum range value (distance to closest object)
		closest_index = ranges.index(min_distance)
		closest_angle = angle_min + closest_index * angle_increment  # Angle to closest object
		self.obstacle_min_distance = min_distance

	def calc_angle(self):
		'''Calculates the target angle with the target position and the current position'''
		target_angle = (math.atan2(self.y_target-self.y_rover,self.x_target-self.x_rover))
		return target_angle


	def direction_planner(self,target_angle):
		'''Decides the best direction to rotate towards the target angle'''
		ang_error = target_angle-self.yaw_angle
		ang_error_adj=math.atan2(math.sin(ang_error),math.cos(ang_error))
		return ang_error_adj/abs(ang_error_adj)

	
	def check_for_obstacles(self):
		if self.pitch_angle > abs(0.5):
			self.obstacle_routine = 0
			self.obstacle_detected = True
		elif self.obstacle_min_distance < self.range_distance:
			self.obstacle_routine = 1
			self.obstacle_detected = True
	
	def angle_correction(self,target_angle):
		'''Corrects the rover's angle based on its current position and target angle'''
		sign = self.direction_planner(target_angle)
		self.twist.angular.z = sign*self.angular_velocity
		self.twist.linear.x = 0.0	

	def check_coord_precision(self):
		'''Boolean expression for the coordinate precision stoppage routine'''
		var = (
				(self.gps_coordinates[0]>(self.target_coordinates[0]-COORDINATE_ERROR) and self.gps_coordinates[0]<(self.target_coordinates[0]+COORDINATE_ERROR)) 
				and (self.gps_coordinates[1]>(self.target_coordinates[1]-COORDINATE_ERROR) and self.gps_coordinates[1]<(self.target_coordinates[1]+COORDINATE_ERROR))
		)		
		return var

	def check_distance_precision(self):
		'''Boolean expression for the distance precision stoppage routine'''
		var = ((self.x_rover>(self.x_target-DISTANCE_ERROR) and self.x_rover<(self.x_target+DISTANCE_ERROR))
		 	and (self.y_rover>(self.x_target-DISTANCE_ERROR) and self.y_rover<(self.y_target+DISTANCE_ERROR)))
		return var

	def check_angle_precision(self,target_angle):
		return not((self.yaw_angle>(target_angle-ANGLE_ERROR*2)) and (self.yaw_angle<(target_angle+ANGLE_ERROR*2)))

	def obstacle_evader(self):
		if self.obstacle_routine == 1:
			print("Coco")
		elif self.obstacle_routine == 2:
			print("Andrés")

	def followGPSFunction(self,target_angle,distance):
		if(self.obstacle_detected):
			self.obstacle_evader()
		elif(self.check_angle_precision(target_angle)):
			self.angle_correction(target_angle)
		elif(distance > 2.5):
			self.twist.linear.x = self.linear_velocity
			self.twist.angular.z = 0.0
		else:
			if self.check_coord_precision() or self.check_distance_precision():
				state = Int8()
				arrived = Bool()
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				state.data = -1
				self.target_coordinates[0]=None
				self.target_coordinates[1]=None
				self.HAS_STARTED=True
				self.orglat = None
				self.orglong = None
				arrived.data=True
				self.arrived_pub.publish(arrived)
				self.state_pub.publish(state)
				self.cmd_vel.publish(self.twist)
				time.sleep(2)
	
	def followGPS(self):
		
		if(self.state==0): #Checks if the state is the one assigned to FGPS

			if(not self.just_started):
				self.get_logger().info("Entered Follow GPS v8.1")
				self.just_started=True
	
			if(self.target_coordinates[0]!=None and self.target_coordinates[1]!=None): #Checks that the target coordinates are not null
				self.get_logger().info(f"The target coordinates are {self.target_coordinates}")
				if((self.gps_coordinates[0]!=0.0 and self.gps_coordinates[1]!=0.0 )or (self.gps_coordinates[0] is not None and self.gps_coordinates[1] is not None)): #Checks that the gps readings are valid
					
					#calculates the target x,y using the target coords and the origin
					self.x_target,self.y_target = ll2xy(self.target_coordinates[0],self.target_coordinates[1],self.orglat,self.orglong)
					#calculates the distance between the current position and the target position
					distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
					target_angle = self.calc_angle()
					self.check_for_obstacles()
					self.followGPSFunction(target_angle,distance)
					self.cmd_vel.publish(self.twist)





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
