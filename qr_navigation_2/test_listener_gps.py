import rclpy
from rclpy import Node 
from .submodules.alvinxy import *
from ublox_msgs.msg import UBXNavHPPosLLH

class Listener_gps(Node):
    def __init__(self):
        super().__init__("test_gps")
        self.listener = self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.update_position,10)
        
    def update_position(self,msg):
        print(msg.lat)
        print(msg.lon)
        
def main(args=None):
    gps = Listener_gps()
    rclpy.init(args=args)
    rclpy.spin(gps)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
    