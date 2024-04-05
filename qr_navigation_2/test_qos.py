import rclpy
from rclpy.node import Node
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from rclpy.qos import *
from .submodules.alvinxy import *

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.HAS_STARTED = True

        self.subscription = self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.callback,qos_profile_sensor_data)
        self.x_rover,self.y_rover,self.angle = 0.0,0.0,0.0
        self.orglong = 0.0
        self.orglat = 0.0
        self.gps_coordinates = [0.0,0.0]
    
    def update_position(self):
        self.x_rover,self.y_rover = ll2xy(self.gps_coordinates[0] ,self.gps_coordinates[1] ,self.orglat,self.orglong)
    
    def callback(self, data):
        if(self.HAS_STARTED):
            self.orglong = data.lon/(10000000)
            self.orglat = data.lat/(10000000)
            self.HAS_STARTED = False
        self.gps_coordinates[0]=data.lat/(10000000)
        self.gps_coordinates[1]=data.lon/(10000000)
        if(not self.HAS_STARTED):
            self.update_position()
        print("Origin: ",self.orglat," ",self.orglong)
        print("Current: ",self.gps_coordinates)
        print("x,y coordinates ",self.x_rover,self.y_rover)
        

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
