#!/usr/bin/env python3
# Controlador
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
        self.s = self.create_subscription(Int32, 'target_type', self.target_type_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_fg', self.cmd_vel_fg_callback, 10)
        self.test = self.create_subscription(Bool, 'arrived_fg', self.arrived_fg_callback, 10)
        self.create_subscription(Bool, 'arrived_sr', self.arrived_sr_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_sr', self.cmd_vel_sr_callback, 10)
        self.create_subscription(Bool, '/go', self.go_start, 10)

        # Publicadores
        self.pub_arrived = self.create_publisher(Bool, 'arrived', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_state = self.create_publisher(Int32, 'state', 10)
        self.pub_go = self.create_publisher(Bool, '/go', 10)

        # Variables de estado
        self.arrived = False
        self.cmd_vel = Twist()
        self.state = Int32()
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
            self.pub_cmd_vel.publish(self.cmd_vel)

    def arrived_ca_callback(self, msg):
        print("Center and Approach callback, ",msg.data)
        self.check_arrived(msg)

    def target_type_callback(self, msg):
        print(f"Received target_type: {msg.data}")
        self.target_function = self.parameters.get(msg.data, "")
        print(f"Setting target_function: {self.target_function}")
        self.start = True
        print("Setting self.start to True")

    def cmd_vel_fg_callback(self, msg):
        if self.state.data == 0:
            self.cmd_vel = msg
            self.pub_cmd_vel.publish(self.cmd_vel)

    def arrived_fg_callback(self, msg):
        print("Follow GPS callback, ",msg.data)
        self.check_arrived(msg)

    def arrived_sr_callback(self, msg):
        print("Search routine callback, ",msg.data)
        self.check_arrived(msg)

    def cmd_vel_sr_callback(self, msg):
        if self.state.data in [1, 2, 3] and not self.arrived:
            self.cmd_vel = msg
            self.pub_cmd_vel.publish(self.cmd_vel)

    def check_arrived(self, msg):
        print("Entered check_arrived ")
        if msg.data:
            self.arrived = True
            arrived_msg = Bool()
            arrived_msg.data = True
            self.pub_arrived.publish(arrived_msg)
            print(f"Finished {self.target_function}")
            self.target_function=""
        else:
            self.arrived = False
        print("Self.arrived: ",self.arrived)

    def controller(self):
        if self.start is True:
            print(f"Received start signal. Target function: {self.target_function}")

            if self.target_function == "gps_only":
                print("Performing actions for target function 'gps_only'")
                self.state.data = 0
                self.pub_state.publish(self.state)
                self.has_started = True
                print("Waiting ...")
                
                

            elif self.target_function == "gps_aruco":
                print("Performing actions for target function 'gps_aruco'")
                self.state.data = 0
                self.pub_state.publish(self.state)
                self.arrived = False
                self.state.data = 4
                self.pub_state.publish(self.state)
                

            elif self.target_function == "gps_hammer":
                print("Performing actions for target function 'gps_hammer'")
                self.state.data = 0
                self.pub_state.publish(self.state)
                while not self.arrived:
                    pass
                self.arrived = False
                self.state.data = 3
                self.pub_state.publish(self.state)
                while not self.arrived:
                    pass

            elif self.target_function == "gps_bottle":
                print("Performing actions for target function 'gps_bottle'")
                self.state.data = 0
                self.pub_state.publish(self.state)
                while not self.arrived:
                    pass
                self.arrived = False
                self.state.data = 2
                while not self.arrived:
                    pass
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
