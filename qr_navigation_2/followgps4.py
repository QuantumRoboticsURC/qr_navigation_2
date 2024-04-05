import rclpy
from rclpy.node import Node
from rclpy.qos import *
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Thread

from .submodules.alvinxy import *
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu
from std_msgs.msg import Int8,Bool
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
		super().__init__('gps4')
		#Probably will be replaced for a service 
		timer_group = MutuallyExclusiveCallbackGroup()
		listener_group = ReentrantCallbackGroup()
  
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel_fg',10)
		self.arrived_pub = self.create_publisher(Bool,'arrived_fg',1)
		self.state_pub = self.create_publisher(Int8,'state',10)
		self.target_coords = self.create_subscription(TargetCoordinates,"/target_coordinates",self.update_target,1,callback_group=listener_group)
		self.reset_coords = self.create_publisher(TargetCoordinates,"/target_coordinates",1)
		#self.subscription = self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.update_coords,qos_profile_sensor_data,callback_group=listener_group)
		self.my_rover_angle = self.create_subscription(Imu, "/bno055/imu", self.update_angle, 10,callback_group=listener_group)    
		self.state_subscription = self.create_subscription(Int8,"/state",self.update_state,10,callback_group=listener_group)
		self.lat= self.create_subscription(Float64,'/latitude',self.update_lon,10)
		self.lon= self.create_subscription(Float64,'/longitude',self.update_lat,10)
		self.twist = Twist()
		self.linear_velocity = 0.16
		self.angular_velocity = 0.1
		
		
		self.gps_coordinates = [0.0,0.0]
		self.target_coordinates = [None,None]
		self.x_rover,self.y_rover,self.angle = 0.0,0.0,0.0
		self.orglong = 0.0
		self.orglat = 0.0
		self.HAS_STARTED = True
		
		self.state = -1
		self.coordinate_error = 0.000002
		self.timer = self.create_timer(0.01,self.followGPS2,callback_group=timer_group)
		

	def update_target(self,msg):
		self.target_coordinates[0]=msg.latitude
		self.target_coordinates[1]=msg.longitude

	def update_position(self):
		self.x_rover,self.y_rover = ll2xy(self.gps_coordinates[0] ,self.gps_coordinates[1] ,self.orglat,self.orglong)
	
	def update_coords(self,data):
		if(self.HAS_STARTED):
			self.orglong = data.lon/(10000000)
			self.orglat = data.lat/(10000000)
			self.HAS_STARTED = False
		self.gps_coordinates[0]=data.lat/(10000000)
		self.gps_coordinates[1]=data.lon/(10000000)
		if(not self.HAS_STARTED):
			self.update_position()
   
	def update_lon(self,data):
		self.gps_coordinates[1]=data.data
	def update_lat(self,data):
		self.gps_coordinates[0]=data.data
		
  
	def update_state(self,msg):
		self.state=msg.data
		
	def update_angle(self,msg):
		quat = Quaternion()
		quat = msg.orientation
		angle_x,angle_y,angle_z = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		self.angle = (angle_z+2*math.pi)%(2*math.pi)
		#self.angle += math.pi
		#self.angle = (angle_z+2*math.pi)%(2*math.pi)

	def calc_angle(self):
		dX,dY = ll2xy(self.target_coordinates[0],self.target_coordinates[1],self.gps_coordinates[0],self.gps_coordinates[1])
		#print(self.target_coordinates[0],self.target_coordinates[1],self.gps_coordinates[0],self.gps_coordinates[1])
		#dY = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.gps_coordinates[1])
		#dX = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.gps_coordinates[0],self.target_coordinates[1])
		#dY,dX = self.distance_correction(dX,dY)
		target_angle = ((math.atan2(dY,dX))+2*math.pi)%(2*math.pi)
		print(self.gps_coordinates)
		print(self.target_coordinates)
		print(f"dx,dy, dT {dX,dY,target_angle}")
		return target_angle

	def angle_correction(self,target_angle):
		#print("CORRECTION-------------------------")
		print(f"Target angle {target_angle} | Current angle {self.angle}")
		self.twist.linear.x = 0.0
		while(not (self.angle > target_angle-0.05 and self.angle < target_angle+0.05)):
			self.twist.angular.z = -(abs((self.angle - 0.0)) * (self.angular_velocity- 0.08) / (2*math.pi - 0) + 0.08)
			self.cmd_vel.publish(self.twist) 

		self.twist.angular.z=0.0
		self.cmd_vel.publish(self.twist)

	def check_coord_precision(self):
		var = (self.gps_coordinates[0]>self.target_coordinates[0]-self.coordinate_error and self.gps_coordinates[0]<self.target_coordinates[0]+self.coordinate_error) and (self.gps_coordinates[1]>self.target_coordinates[1]-self.coordinate_error and self.gps_coordinates[1]<self.target_coordinates[1]+self.coordinate_error)
		return var
	
	def followGPS2(self):
		if(self.state==0 and self.target_coordinates[0] != None and self.target_coordinates[1] != None):
			print("Entered Follow GPS 4")
			#print(f"Current coords {self.gps_coordinates} | \nTarget coords {self.target_coordinates}")
			state = Int8()
			arrived = Bool()
			
			target_angle = self.calc_angle()
			self.angle_correction(target_angle)
	
			distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
			
			start_time = time.time()
			WITHIN_RANGE = False
	
			while(distance>1.5): 
				#print(f"Distance = {distance}")
				distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
				current_time = time.time()
				target_angle = self.calc_angle()
				if((current_time-start_time)%5==0 and not WITHIN_RANGE):
					#print("Making correction ")
					if(not (self.angle>target_angle-0.05 and self.angle<target_angle+0.05)):
						self.angle_correction(target_angle)
				else:
					distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
					if(distance < 20):
						#print("WITHIN RANGE")
						WITHIN_RANGE=True
						if(not WITHIN_RANGE):
							self.angle_correction(self.calc_angle())
						self.twist.linear.x = (abs((distance - 0)) * (self.linear_velocity- 0.08) / (20 - 0) + 0.08)
					else:
						self.twist.linear.x = self.linear_velocity
					if(self.check_coord_precision()):
						break

					self.cmd_vel.publish(self.twist)
			

			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			arrived.data=True
			state.data = -1

			self.arrived_pub.publish(arrived)
			self.target_coordinates[0]=None
			self.target_coordinates[1]=None
			
			self.state_pub.publish(state)
			self.cmd_vel.publish(self.twist)

			self.HAS_STARTED=False
			time.sleep(5)


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
	
