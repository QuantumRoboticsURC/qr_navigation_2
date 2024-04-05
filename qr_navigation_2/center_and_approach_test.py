import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Header, Float64,Int8
from geometry_msgs.msg import Twist
from custom_interfaces.msg import CA

class Center_approach(Node):
    def __init__(self):
        super().__init__("center")
        self.create_subscription(CA, "center_approach", self.callback,10)
        self.create_subscription(Bool, "detected_aruco", self.aruco, 1)
        self.create_subscription(Bool,"detected_orange",self.orange,1)
        self.arrived = self.create_publisher(Bool, "arrived_ca", 10)
        self.cmd_vel_ca = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub = self.create_publisher(Int8,"/state",1)
        self.Twist = Twist()
        
        self.vel_x = 0.16
        self.vel_y = 0.0
        self.vel_theta = 0.06
        self.distance = 0.0
        self.found = False
        self.x = 0.0
        self.center = False
        self.finish = False
        self.timer = self.create_timer(0.01, self.get_aruco)
        
    def callback(self,msg):
        self.distance = msg.distance
        self.x = msg.x
        self.center = msg.detected
        
    def aruco(self, msg): 
        self.found = msg.data
    def orange(self,msg):
        self.found = msg.data
        
    def approach(self):
        if (self.distance > 1300):
            self.Twist.linear.x = self.vel_x
            self.Twist.angular.z = 0.0
        else:
            self.Twist.linear.x = 0.0
            self.Twist.angular.z = 0.0
            self.found = False
            self.arrived_ca = True
            print("Terminado")
            arrived = Bool()
            arrived.data = True
            state = Int8()
            state.data = -1
            self.finish=True
            self.state_pub.publish(state)
            self.arrived.publish(arrived)
        
        self.cmd_vel_ca.publish(self.Twist) 

    def get_aruco(self):
        if (self.found and not self.finish):
            if (self.center):
                self.approach()
                print("Centro")
            elif (self.x+50*(1200/self.distance)  < 0):
                self.Twist.linear.x = 0.0
                self.Twist.angular.z = self.vel_theta
                print("Izquierda")
            elif (self.x-50*(1200/self.distance) > 0):
                self.Twist.linear.x = 0.0
                self.Twist.angular.z = -self.vel_theta
                print("Derecha")
            self.cmd_vel_ca.publish(self.Twist)    
    
def main(args=None):
	rclpy.init(args=args)
	ca = Center_approach()
	rclpy.spin(ca)
	ca.destroy_node()
	rclpy.shutdown()

    
if __name__=="__main__":
    main()
    
