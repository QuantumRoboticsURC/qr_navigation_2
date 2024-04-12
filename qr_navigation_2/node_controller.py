import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Int8
from geometry_msgs.msg import Twist
import time

class NodeController(Node):
	def __init__(self):
		super().__init__('node_controller')

		# Suscripciones
		self.create_subscription(Twist, 'cmd_vel_ca', self.cmd_vel_ca_callback, 10)
		self.create_subscription(Bool, 'arrived_ca', self.arrived_ca_callback, 10)
		self.s = self.create_subscription(Int8, 'target_type', self.target_type_callback, 10)
		self.create_subscription(Twist, 'cmd_vel_fg', self.cmd_vel_fg_callback, 10)
		self.test = self.create_subscription(Bool, 'arrived_fg', self.arrived_fg_callback, 10)
		self.create_subscription(Bool, 'arrived_sr', self.arrived_sr_callback, 10)
		self.create_subscription(Twist, 'cmd_vel_sr', self.cmd_vel_sr_callback, 10)
		self.create_subscription(Bool, '/go', self.go_start, 10)

		# Publicadores
		self.pub_arrived = self.create_publisher(Bool, 'arrived', 10)
		self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
		self.pub_state = self.create_publisher(Int8, '/state', 1)
		self.pub_go = self.create_publisher(Bool, '/go', 10)

		# Variables de estado
		self.arrived = False
		self.cmd_vel = Twist()
		self.state = Int8()
		self.start = False

		# Contenedores
		self.parameters = {0: "gps_only", 1: "gps_aruco", 2: "gps_hammer", 3: "gps_bottle"}
		self.target_function = ""
		self.timer = self.create_timer(0.05, self.controller)

	# Callbacks para los tópicos suscritos
	def go_start(self, msg):
		self.start = msg.data

	def cmd_vel_ca_callback(self, msg):
		if self.arrived:
			self.cmd_vel = msg
			if self.cmd_vel.linear.x<=0.2:
				self.pub_cmd_vel.publish(self.cmd_vel)
			else:
				c =Twist()
				c.linear.x=0.2
				c.angular.z = self.cmd_vel.angular.z
				self.pub_cmd_vel.publish(c)

	def arrived_ca_callback(self, msg):
		print("Center and Approach callback, ",msg.data)
		self.check_arrived(msg)

	def target_type_callback(self, msg):
		print(f"Received target_type: {msg.data}")
		self.target_function = self.parameters.get(msg.data, "")
		print(f"Setting target_function: {self.target_function}")
		self.start = True
		print("Setting self.start to True")
		self.state.data=0

	def cmd_vel_fg_callback(self, msg):
		if self.state.data == 0:
			self.cmd_vel = msg
			if self.cmd_vel.linear.x > 0.2:
				modified_cmd_vel = Twist()
				modified_cmd_vel.linear.x = 0.2
				modified_cmd_vel.angular.z = self.cmd_vel.angular.z
				self.pub_cmd_vel.publish(modified_cmd_vel)
			else:
				self.pub_cmd_vel.publish(self.cmd_vel)


	def arrived_fg_callback(self, msg):
		print("Follow GPS callback, ",msg.data)
		self.check_arrived(msg)

	def arrived_sr_callback(self, msg):
		print("Search routine callback, ",msg.data)
		self.check_arrived(msg)

	def cmd_vel_sr_callback(self, msg):
		if self.state.data in [1, 2, 3] and not self.arrived:
			if msg.linear.x > 0.2:
				modified_cmd_vel = Twist()
				modified_cmd_vel.linear.x = 0.2
				modified_cmd_vel.angular.z = msg.angular.z
				self.pub_cmd_vel.publish(modified_cmd_vel)
			else:
				self.pub_cmd_vel.publish(msg)


	def check_arrived(self, msg):
		print("Entered check_arrived ")
		#print (f"{self.target_function}")
		#print (f"print :{self.state.data}")
		if msg.data:
			self.arrived = True
			arrived_msg = Bool()
			arrived_msg.data = True
			if self.target_function == "gps_only":
				print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
				self.pub_arrived.publish(arrived_msg)
				print(f"Finished {self.target_function}")
				self.target_function=""
				self.start = False
				print("Finished gps_only")
			elif self.target_function == "gps_aruco" and self.state.data==0:
				self.state.data = 4
				self.pub_state.publish(self.state)
				print (f"{self.state.data}")
			elif self.target_function == "gps_aruco" and self.state.data==4:
				print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
				self.pub_arrived.publish(arrived_msg)
				print(f"Finished {self.target_function}")
				self.target_function=""
				self.start = False
				print("Finished gps_aruco")
			elif self.target_function == "gps_hammer" and self.state.data==0:
				self.state.data = 3
				self.pub_state.publish(self.state)
				print (f"{self.state.data}")
			elif self.target_function == "gps_hammer" and self.state.data==3:
				self.target_function=""
				self.start = False
				print("Finished gps_hammer")
			elif self.target_function == "gps_bottle" and self.state.data==0:
				self.state.data = 2
				self.pub_state.publish(self.state)
				print (f"{self.state.data}")
			elif self.target_function == "gps_bottle" and self.state.data==2:
				self.target_function=""
				self.start = False
				print("Finished gps_hammer")
				
				
		else:
			self.arrived = False
		print("Self.arrived: ",self.arrived)

	def controller(self):
		
		if self.start is True:
			arrive_msg=Bool()
			arrive_msg.data=False
			self.pub_arrived.publish(arrive_msg)
			print(f"Received start signal. Target function: {self.target_function}")
			
			
			if self.target_function == "gps_only":
				
				print("Performing actions for target function 'gps_only'")
				self.pub_state.publish(self.state)
				self.has_started = True
				print("Waiting ...")
				
				

			elif self.target_function == "gps_aruco":
				print("Performing actions for target function 'gps_aruco'")
				self.pub_state.publish(self.state)
				self.arrived = False
				print("Waiting ...")
				print(f"Excecuting {self.state.data}")
				
				

			elif self.target_function == "gps_hammer":
				print("Performing actions for target function 'gps_hammer'")
				self.pub_state.publish(self.state)
				self.arrived = False
				print("Waiting ...")
				print(f"Excecuting {self.state.data}")

			elif self.target_function == "gps_bottle":
				print("Performing actions for target function 'gps_bottle'")
				self.pub_state.publish(self.state)
				self.arrived = False
				print("Waiting ...")
				print(f"Excecuting {self.state.data}")
		#started = Bool()
		#started.data = False
		#self.pub_go.publish(started)


def main(args=None):
	rclpy.init(args=args)
	node_controller = NodeController()
	rclpy.spin(node_controller)
	node_controller.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
