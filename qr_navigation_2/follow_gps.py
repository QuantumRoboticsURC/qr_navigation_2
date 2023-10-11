import rclcpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from rclpy import Node 
from alvinxy import *
import numpy

class FollowGPS(Node):
	def __init__(self):
		super().__init__('follow_gps')
		
		self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
		self.timer = self.create_timer(0.5,self.follow)
		self.my_rover_position = self.create_subscription(NavSatFix,'gps',self.update_position,10)
		self.my_rover_angle = None
		self.gps_coordinates = (0.0,0.0)
		self.rover_orientation = (0.0,0.0,0.0)
		self.x,self.y = 0.0,0.0
		self.orglong = 0.0
		self.orglat = 0.0



	def follow(self,longitude,latitud):
		self.x,self.y = ll2xy(longitude,latitud,self.orglong,self.orglat)


	def update_position(self,msg):
		self.gps_coordinates[0] = msg.latitud
		self.gps_coordinates[1] = msg.longitude
		x,y = ll2xy(msg.longitude,msg.latitud,self.orglong,self.orglat)
		self.rover_orientation[0] = x
		self.rover_orientation[1] = y

	def set_origin(orglat,orglong):
		self.orglat = orglat
		self.orglong = orglong
