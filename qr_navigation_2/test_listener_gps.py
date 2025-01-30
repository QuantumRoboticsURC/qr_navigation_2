import rclpy
from rclpy.node import Node 
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from std_msgs.msg import Float64
from rclpy.qos import qos_profile_sensor_data
from .submodules.alvinxy import *
class Listener_gps(Node):
    def __init__(self):
        super().__init__('test_gps')
        self.lat= self.create_subscription(Float64,'/latitude',self.update_lon,10)
        self.lon= self.create_subscription(Float64,'/longitude',self.update_lat,10)
        self.timer = self.create_timer(0.01, self.control)
        self.longitude = 0.0
        self.latitude = 0.0
        self.coords = [0.0,0.0]
        self.orglat = 0.0
        self.orglong = 0.0
        self.start = [0.0,0.0]
        self.started = False
    
    def update_coords(self):
        self.coords[0]=self.latitude
        self.coords[1]=self.longitude

    def update_lat(self,data):
        if self.orglat == 0.0:
            self.orglat = data.data/10000000.0
        self.latitude = data.data/10000000.0
        self.update_coords()
        
    def update_lon(self,data):
        if self.orglong == 0.0:
            self.orglong = data.data/10000000.0
        self.longitude=data.data/10000000.0
        self.update_coords()
        
    def control(self):
        x,y=ll2xy(self.coords[0] ,self.coords[1] ,self.orglat,self.orglong)
        if self.start[0] ==0.0 and self.start[1] ==0.0:
            self.start = [x,y]
            self.started = True
        print(f"Positions {x},{y}")
        print(f"Start={self.start}")
        print(self.coords)
        if self.started:
            try:
                print(f"Error {(x-self.start[0])},{(y-self.start[1])}")
            except Exception as e:
                print(e)
        
def main(args=None):
    rclpy.init(args=args)
    L= Listener_gps()
    rclpy.spin(L)
    L.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()  
