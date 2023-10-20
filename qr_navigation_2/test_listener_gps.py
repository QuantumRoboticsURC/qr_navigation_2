import rclpy
from rclpy.node import Node 
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from std_msgs.msg import Float64
class Listener_gps(Node):
    def __init__(self):
        super().__init__('test_gps')
        self.s= self.create_subscription(UBXNavHPPosLLH,'/gps_rover/ubx_nav_hp_pos_llh',self.actualizarCallBack,10)
        self.s
        self.sub = self.create_subscription(Float64,'test',self.callBack,10)
        #self.timer = self.create_timer(0.05, self.control)
        self.longitude = 0.0


    def actualizarCallBack(self,msg):
        print("AAAAAAAAAAAAAa")
        self.longitude = msg.lon

    def callBack(self,msg):
        print(msg.data)

    def control(self):
        print(self.longitude)

        
def main(args=None):
    print("hola2")
    rclpy.init(args=args)
    L= Listener_gps()
    print("hola3")
    rclpy.spin(L)
    L.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()  
