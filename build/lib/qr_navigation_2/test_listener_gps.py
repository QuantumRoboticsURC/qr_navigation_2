import rclpy
from rclpy.node import Node 
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from std_msgs.msg import Float64
from rclpy.qos import qos_profile_sensor_data

class Listener_gps(Node):
    def __init__(self):
        super().__init__('test_gps')
        self.lat= self.create_subscription(Float64,'/latitude',self.update_lon,10)
        self.lon= self.create_subscription(Float64,'/longitude',self.update_lat,10)
        self.timer = self.create_timer(0.05, self.control)
        self.longitude = 0.0
        self.latitude = 0.0
        self.coords = [0.0,0.0]
    
    def update_coords(self):
        self.coords[0]=self.latitude
        self.coords[1]=self.longitude

    def update_lat(self,data):
        self.latitude = data.data
        self.update_coords()
        
    def update_lon(self,data):
        self.longitude=data.data
        self.update_coords()
        
    def control(self):
        print(self.coords)

        
def main(args=None):
    rclpy.init(args=args)
    L= Listener_gps()
    rclpy.spin(L)
    L.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()  
