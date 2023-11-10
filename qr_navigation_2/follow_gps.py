import rclpy
from rclpy.node import Node
from rclpy.qos import *
from .submodules.alvinxy import *
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu
from std_msgs.msg import Int8,Bool
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from custom_interfaces.srv import FollowGPS
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
		super().__init__('go_to_gps')
		#Probably will be replaced for a service 
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel_fg',10)
		self.arrived_pub = self.create_publisher(Bool,'arrived_fg',1)
		self.state_pub = self.create_publisher(Int8,'/state',10)
		self.target_coordinates = self.create_subscription(TargetCoordinates,"/target_coordinates",self.update_target,1)
		self.subscription = self.create_subscription(UBXNavHPPosLLH,'/gps_rover/ubx_nav_hp_pos_llh',self.update_coords,qos_profile_sensor_data)
		self.my_rover_angle = self.create_subscription(Imu, "/bno055/imu", self.update_angle, 10)    
		self.state_subscription = self.create_subscription(Int8,"/state",self.update_state,10)

		self.twist = Twist()
		self.linear_velocity = 0.33
		self.angular_velocity = 0.2
		
		self.gps_coordinates = [0.0,0.0]
		self.target_coords = [0.0,0.0]
		self.x_rover,self.y_rover,self.angle = 0.0,0.0,0.0
		self.orglong = 0.0
		self.orglat = 0.0
		self.HAS_STARTED = True
		self.state = -1
		self.timer = self.create_timer(0.01,self.followGPS2)
		

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
		print(f"coordinates: {self.gps_coordinates}")
  
	def update_state(self,msg):
		self.state=msg.data
		
	def update_angle(self,msg):
		quat = Quaternion()
		quat = msg.orientation
		angle_x,angle_y,angle_z = euler_from_quaternion(quat.x,quat.y,quat.z,quat.w)
		self.angle = (angle_z+2*math.pi)%2*math.pi
		print(f"Angle: {self.angle}")

	def followGPS2(self):
		if(self.state==0):
			state = Int8()
			arrived = Bool()
			
			x,y = ll2xy(self.target_coordinates[0],self.target_coordinates[1],self.orglat,self.orglong)
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
			arrived.data=True
			state.data = -1

			self.arrived_pub.publish(arrived)
			self.state_pub.publish(state)
			self.cmd_vel.publish(self.twist)
   
			time.sleep(1)
   


def main(args=None):
	rclpy.init(args=args)
	gps = Follow_GPS()
	rclpy.spin(gps)
	gps.destroy_node()
	rclpy.shutdown()

    
if __name__=="__main__":
    main()
    
