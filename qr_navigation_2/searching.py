#Program: Searching Routine
#Version: 1.2a
#Developer: @emvivas (Emiliano Vivas Rodríguez)
#Contact: a01424732@tec.mx

import rclpy, math, bisect, copy, numpy, time
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int8, Float64, Bool
from custom_interfaces.msg import Coordinates, TargetCoordinates, NavigationCoordinates
from rclpy.qos import * 
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Thread
from .submodules.alvinxy import *
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu
from ublox_ubx_msgs.msg import UBXNavHPPosLLH


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

class Searching(Node):

	def __init__(self):
		super().__init__('searching')
		self.orglatitude = 19.5970212
		self.orglongitude = -99.227144
		self.state = None
		self.is_found = None
		self.is_stopped = None
		self.is_finished = None
		self.coordinates = Coordinates()
		self.point = Point()
		self.target_point = Point()
	
	#TODO: Change archimedean spiral paramethers
		self.scale = 7
		self.turns = 2
		self.points_distance = 150
		self.continuous_points_number = 500
		self.navigation_point_uncertainty = 10

		self.set_default_configuration()
		self.sub_state = self.create_subscription(Int8, '/state', self.state_callback, 10)
		self.sub_found = self.create_subscription(Int8, '/found', self.found_callback, 10)
		self.sub_stopped = self.create_subscription(Int8, '/stopped', self.stopped_callback, 10)
		self.sub_finished = self.create_subscription(Int8, '/finished', self.finished_callback, 10)
		self.sub_coordinates = self.create_subscription(Coordinates, '/coordinates', self.coordinates_callback, 10)
		self.pub_target_navigation_coordinates_list = self.create_publisher(NavigationCoordinates, '/target_navigation_coordinates_list', 10)
		self.pub_target_navigation_coordinates = self.create_publisher(TargetCoordinates, '/target_navigation_coordinates', 10)
		self.pub_target_navigation_coordinates_remaining_distance = self.create_publisher(Float64, '/target_navigation_coordinates_remaining_distance', 10)
		self.pub_autonomous_navigation_route_times = self.create_publisher(Int8, '/autonomous_navigation_route_times', 10)

		timer_group = MutuallyExclusiveCallbackGroup()
		listener_group = ReentrantCallbackGroup()
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel_fg',10)
		self.arrived_pub = self.create_publisher(Bool,'arrived_fg',1)
		self.state_pub = self.create_publisher(Int8,'state',1)
  
		#TODO: Change "/target_coordinates" to "/target_navigation_coordinates"
		self.target_coords = self.create_subscription(TargetCoordinates,"/target_coordinates",self.update_target,1,callback_group=listener_group)
		
		self.reset_coords = self.create_publisher(TargetCoordinates,"/target_coordinates",1)
		self.subscription = self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.update_coords,qos_profile_sensor_data,callback_group=listener_group)
		self.my_rover_angle = self.create_subscription(Imu, "/bno055/imu", self.update_angle, 10,callback_group=listener_group)    
		self.state_subscription = self.create_subscription(Int8,"/state",self.update_state,1,callback_group=listener_group)
		#self.lat= self.create_subscription(Float64,'/latitude',self.update_lat,10)
		#self.lon= self.create_subscription(Float64,'/longitude',self.update_lon,10)
		self.twist = Twist()
		self.linear_velocity = 0.16
		self.angular_velocity = 0.1
		self.gps_coordinates = [0.0,0.0]
		self.target_coordinates = [None,None]
		self.x_rover,self.y_rover,self.angle = 0.0,0.0,0.0
		self.dX = 0.0
		self.dY = 0.0
		self.orglong = 0.0
		self.orglat = 0.0
		self.HAS_STARTED = True
		self.FIRST_LAT = False
		self.FIRST_LON = False
		self.state = -1
		self.coordinate_error = 0.00005  
		self.distance_error = 1.0

		self.timer = self.create_timer(0.001, self.timer_callback,callback_group=timer_group)

	
	def set_default_configuration(self):
		self.times = Int8()
		self.continuous_route = []
		self.discrete_route = []
		self.navigation_coordinates = NavigationCoordinates()
		self.navigation_coordinates_auxiliar = None
		self.target_navigation_coordinates = None
		self.is_calculated_route = False
		self.remaining_distance = Float64()
	
	def state_callback(self,msg):
		self.state = msg.data
	
	def found_callback(self,msg):
		self.is_found = msg.data
	
	def stopped_callback(self,msg):
		self.is_stopped = msg.data

	def finished_callback(self,msg):
		self.is_finished = msg.data
	
	def coordinates_callback(self,msg):
		self.coordinates.latitude,self.coordinates.longitude = msg.latitude, msg.longitude
		self.point.x, self.point.y = ll2xy(msg.latitude, msg.longitude,self.orglatitude,self.orglongitude)

	def timer_callback(self):
		if self.is_finished == 1: #TODO
			self.set_default_configuration()
			pass
		elif self.is_stopped == 1: #TODO
			pass
		elif self.is_found == 1: #TODO
			pass
		elif self.state == 2 or self.state==3 or self.state==4:
			if not self.is_calculated_route:
				self.calculate_continuous_route()
				self.calculate_discrete_route()
				self.navigation_coordinates_auxiliar = copy.copy(self.navigation_coordinates.coordinates)
				self.navigation_coordinates_auxiliar.pop(0)
				self.target_navigation_coordinates = self.navigation_coordinates_auxiliar.pop(0)
				self.times.data = 0
				self.is_calculated_route = True
			elif self.target_navigation_coordinates:
				self.target_point.x, self.target_point.y = ll2xy(self.target_navigation_coordinates.latitude, self.target_navigation_coordinates.longitude, self.orglatitude, self.orglongitude)
				self.remaining_distance.data = distanceBetweenCoords(self.coordinates.latitude, self.coordinates.longitude, self.target_navigation_coordinates.latitude, self.target_navigation_coordinates.longitude)
				if self.point.x >= self.target_point.x - self.navigation_point_uncertainty and self.point.x <= self.target_point.x + self.navigation_point_uncertainty and self.point.y >= self.target_point.y - self.navigation_point_uncertainty and self.point.y <= self.target_point.y + self.navigation_point_uncertainty:
					self.target_navigation_coordinates = self.navigation_coordinates_auxiliar.pop(0) if len(self.navigation_coordinates_auxiliar)>0 else None
			elif self.is_calculated_route:
				self.navigation_coordinates_auxiliar = copy.copy(self.navigation_coordinates.coordinates)
				self.target_navigation_coordinates = self.navigation_coordinates_auxiliar.pop(0)
				self.times.data += 1
			
			#FollowGPS
			
			if(self.target_coordinates[0] != None and self.target_coordinates[1] != None and self.gps_coordinates[0]!=0.0 and self.gps_coordinates[1]!=0.0):
				print("Entered Follow GPS 5")
				self.dX,self.dY = ll2xy(self.target_coordinates[0],self.target_coordinates[1],self.orglat,self.orglong)
				#print(f"Current coords {self.gps_coordinates} | \nTarget coords {self.target_coordinates}")
				state = Int8()
				arrived = Bool()
				
				target_angle = self.calc_angle()

				self.angle_correction(target_angle)
		
				distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
				
				start_time = time.time()
				WITHIN_RANGE = False
		
				while(distance>1.5): 
					self.update_position()
					self.dX,self.dY = ll2xy(self.target_coordinates[0],self.target_coordinates[1],self.orglat,self.orglong)
					distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
					current_time = time.time()
					target_angle = self.calc_angle()
					if((int(current_time)-int(start_time))%8.0==0.0):
						start_time = time.time()
						if(not((self.angle>(target_angle-0.1)) and (self.angle<(target_angle+0.1)))):
							print("Distancia ",distance)
							self.angle_correction(target_angle)
					else:
						distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
						self.twist.linear.x = self.linear_velocity
						
						#if(distance < 20):
							#print("WITsHIN RANGE")
						#	WITHIN_RANGE=True
						#	if(not WITHIN_RANGE):
						#		self.angle_correction(self.calc_angle())
						#	self.twist.linear.x = (abs((distance - 0)) * (self.linear_velocity- 0.08) / (20 - 0) + 0.08)
						#else:
						#	self.twist.linear.x = self.linear_velocity
						if(self.check_coord_precision()):
							print("Coord precision stopped the process")
							print(f"Finished at {self.x_rover,self.y_rover} \nTarget position was {self.dX, self.dY}")
							break
						if(self.check_distance_precision()):
							print(f"Finished at {self.x_rover,self.y_rover} \nTarget position was {self.dX, self.dY}")
							break					

						self.cmd_vel.publish(self.twist)
				
				print("Distance was : ",distance)
				print(f"Finished at {self.x_rover,self.y_rover} \nTarget position was {self.dX, self.dY}")
				
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
				
				#TODO: Check this time.sleep()
				time.sleep(5)
   
		self.pub_target_navigation_coordinates_list.publish(self.navigation_coordinates)
		self.pub_target_navigation_coordinates.publish(self.target_navigation_coordinates if self.target_navigation_coordinates else TargetCoordinates())
		self.pub_target_navigation_coordinates_remaining_distance.publish(self.remaining_distance)
		self.pub_autonomous_navigation_route_times.publish(self.times)

	def calculate_continuous_route(self):
		self.continuous_route = []
		angle_max = 2 * math.pi * self.turns
		points_number = int(angle_max * self.continuous_points_number)
		delta_angle = angle_max / points_number
		for index in range(points_number):
			angle = index * delta_angle
			self.continuous_route.append((int(self.scale * angle * math.cos(angle) + self.point.x), int(self.scale * angle * math.sin(angle) + self.point.y)))
		
	def calculate_discrete_route(self):
		self.discrete_route = []
		distances = [0]
		for index in range(1, len(self.continuous_route)):
			distance = math.sqrt((self.continuous_route[index][0] - self.continuous_route[index - 1][0])**2 +
								(self.continuous_route[index][1] - self.continuous_route[index - 1][1])**2)
			distances.append(distances[-1] + distance)
		for index in range(0, int(distances[-1]), self.points_distance):
			subindex = bisect.bisect(distances, index)
			current_point = self.continuous_route[subindex]
			self.discrete_route.append(current_point)
			target_coordinates = TargetCoordinates()
			target_coordinates.latitude,target_coordinates.longitude = xy2ll(current_point[0],current_point[1],self.orglatitude,self.orglongitude)
			self.navigation_coordinates.coordinates.append(target_coordinates)
	
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
   
	def update_lat(self,data):
		self.gps_coordinates[0]=data.data
		if not self.FIRST_LAT:
			self.orglat  = data.data
			self.FIRST_LAT = True
		if(self.FIRST_LAT and self.FIRST_LON):
			self.update_position()
   
	def update_lon(self,data):
		self.gps_coordinates[1]=data.data
		if not self.FIRST_LON:
			self.orglong = data.data
			self.FIRST_LON = True
		
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
		target_angle = ((math.atan2(self.dY-self.y_rover,self.dX-self.x_rover))+2*math.pi)%(2*math.pi)
		return target_angle

	def angle_correction(self,target_angle):
		print("------------------CORRECTION2-------------------------")
		print(f"Target angle {target_angle} | Current angle {self.angle}")
		print("Coord objetivo ", self.dX,self.dY)
		print("Coord actual ", self.x_rover,self.y_rover)
		self.twist.linear.x = 0.0
		if(target_angle>self.angle):      
			difference_from_one = ((2*math.pi)-target_angle)+self.angle
			difference_from_two = target_angle-self.angle
			print(f"Difference1 {difference_from_one} | Difference2 {difference_from_two}")
			if(difference_from_two>difference_from_one):
				sign=1
				print("antihorario")
			else:
				sign=-1
				print("horario")
		else:
			difference_from_one = ((2*math.pi)-self.angle)+target_angle
			difference_from_two = self.angle-target_angle
			print(f"Difference1 {difference_from_one} | Difference2 {difference_from_two}")
			if(difference_from_two>difference_from_one):
				sign=1
				print("antihorario")
			else:
				sign=-1
				print("horario")
		while(not ((self.angle > (target_angle-0.05)) and (self.angle < (target_angle+0.05)))):
			self.twist.angular.z = sign*(abs((abs(self.angle-target_angle) - 0.0)) * (self.angular_velocity- 0.08) / (2*math.pi - 0) + 0.08)
			self.cmd_vel.publish(self.twist) 

		self.twist.angular.z=0.0
		self.cmd_vel.publish(self.twist)

	def check_coord_precision(self):
		var = (self.gps_coordinates[0]>(self.target_coordinates[0]-self.coordinate_error) and self.gps_coordinates[0]<(self.target_coordinates[0]+self.coordinate_error)) and (self.gps_coordinates[1]>(self.target_coordinates[1]-self.coordinate_error) and self.gps_coordinates[1]<(self.target_coordinates[1]+self.coordinate_error))
		return var

	def check_distance_precision(self):
		var = (self.x_rover>(self.dX-self.distance_error) and self.x_rover<(self.dX+self.distance_error)) and (self.y_rover>(self.dY-self.distance_error) and self.y_rover<(self.dY+self.distance_error))
		return var

def main(args=None):
	rclpy.init(args=args)
	searching = Searching()
	executor = MultiThreadedExecutor()
	executor.add_node(searching)
	executor.spin()
	searching.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
