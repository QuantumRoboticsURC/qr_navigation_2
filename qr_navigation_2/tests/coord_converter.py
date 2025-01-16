import rclpy
from rclpy.node import Node
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from rclpy.qos import *
from .submodules.alvinxy import *

from sensor_msgs.msg import NavSatFix

class MySubscriber(Node):

    def __init__(self):
        super().__init__('coord')
        self.subscription = self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.callback,qos_profile_sensor_data)
        self.gps = self.create_publisher(NavSatFix,'/coordinates',10)
        self.gps_coords = NavSatFix()
    
    def update_position(self):
        self.x_rover,self.y_rover = ll2xy(self.gps_coordinates[0] ,self.gps_coordinates[1] ,self.orglat,self.orglong)
    
    def callback(self, data):
        self.gps_coords.longitude=data.lon/(10000000)
        self.gps_coords.latitude=data.lat/(10000000)
        self.gps_coords.altitude=data.height
        self.gps.publish(self.gps_coords)
        


        

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
