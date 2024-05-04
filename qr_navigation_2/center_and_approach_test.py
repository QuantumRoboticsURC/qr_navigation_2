import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,Header, Float64,Int8
from geometry_msgs.msg import Twist
from custom_interfaces.msg import CA

class Center_approach(Node):
    def __init__(self):
        super().__init__("center_approach")
        #subscriptions to the messages on the node controller and the node detection
        self.create_subscription(CA, "/center_approach", self.callback,10)
        self.create_subscription(Bool, "/detected_aruco", self.aruco, 1)
        self.create_subscription(Bool,"/detected_orange",self.orange,1)
        self.create_subscription(Int8,"/state",self.update_state,1)
        
        #Publishers to give feedback to the controller
        self.arrived = self.create_publisher(Bool, "/arrived_ca", 10)
        self.cmd_vel_ca = self.create_publisher(Twist, "/cmd_vel_ca", 10)
        self.state_pub = self.create_publisher(Int8,"/state",1)
        #default state value
        self.state = -1
        #Velocity values
        self.Twist = Twist()
        self.vel_x = 0.16
        self.vel_y = 0.0
        self.vel_theta = 0.06
        #CA topic variables
        self.distance = 0.0
        self.x = 0.0
        self.center = False
        #flags
        self.found = False
        self.finish = False
        #Constants 
        self.pixel_constante = 50
        self.center_distance_constant = 1200
        self.min_distance_constant = 1300
        self.relation = {2:" bottle ",3:" hammer ",4:" aruco"}
        
        self.timer = self.create_timer(0.0001, self.center_and_approach)
    
    
    def update_state(self,msg):
        '''Updates the state, with the date of the node controller'''
        self.state=msg.data
        
    def callback(self,msg):
        '''Gets the data from the node detection CA topic and assigns it to the vars'''
        self.distance = msg.distance
        self.x = msg.x
        self.center = msg.detected
        
    def aruco(self, msg): 
        '''Sets found to true if an aruco was found'''
        self.found = msg.data
        
    def orange(self,msg):
        '''Sets found to true if an orange object is found'''
        self.found = msg.data
        
    def approach(self):
        '''Approaches the object while the distance to its is above the threshold'''
        self.Twist.angular.z = 0.0
        arrived = Bool()
        state = Int8()
        if (self.distance > self.min_distance_constant):
            self.Twist.linear.x = self.vel_x
        else:
            self.Twist.linear.x = 0.0
            
            self.found = False
            self.arrived_ca = True
            self.finish= True
            arrived.data = True
            state.data = -1
            
            self.state_pub.publish(state)
            self.arrived.publish(arrived)
        
        self.cmd_vel_ca.publish(self.Twist) 

    def center_and_approach(self):
        if(self.state in [2,3,4]):
            if (self.found):
                self.get_logger().info(f"Estoy en center and approachs")
                if(not self.finish):
                    if (self.center):
                        self.get_logger().info(f"The {self.relation[self.state]} is centered")
                        self.approach()
                    elif (self.x+self.pixel_constante*(self.center_distance_constant/(self.distance+0.01))  < 0):
                        self.Twist.linear.x = 0.0
                        self.Twist.angular.z = self.vel_theta
                        self.get_logger().info(f"The {self.relation[self.state]} is on the left")
                    elif (self.x-self.pixel_constante*(self.center_distance_constant/(self.distance+0.01)) > 0):
                        self.Twist.linear.x = 0.0
                        self.Twist.angular.z = -self.vel_theta
                        self.get_logger().info(f"The {self.relation[self.state]}  is on the right")
                    self.cmd_vel_ca.publish(self.Twist)    
    
def main(args=None):
	rclpy.init(args=args)
	ca = Center_approach()
	rclpy.spin(ca)
	ca.destroy_node()
	rclpy.shutdown()

    
if __name__=="__main__":
    main()
    
