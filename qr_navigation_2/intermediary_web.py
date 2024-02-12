import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,Bool,Int8
from custom_interfaces.msg import TargetCoordinates

class Intermediary_web(Node):
    def __init__(self):
        super().__init__('node_web_intermediary')
        self.listener = self.create_subscription(Float64MultiArray, 'ublox/gps_goal', self.callback_coordinates, 1)
        self.publisher = self.create_publisher(TargetCoordinates, 'target_coordinates', 1)
        self.coords = TargetCoordinates()
        self.litenerarrived=self.create_subscription(Bool, 'arrived', self.callback_arrived, 10)
        self.publisherarrived=self.create_publisher(Int8,'matrix_signal',10)
        self.color=Int8()
        self.color.data=2
        self.publisherarrived.publish(self.color)

    def callback_coordinates(self, data):
        self.coords.latitude = data.data[0]
        self.coords.longitude = data.data[1]
        self.publisher.publish(self.coords)
    
    def callback_arrived(self,data):
        print(data.data)
        if (data.data):
            self.color.data=3
            self.publisherarrived.publish(self.color)
        else:
            self.color.data=2
            self.publisherarrived.publish(self.color)
    
        
def main(args=None):
    rclpy.init(args=args)
    node_intermediate = Intermediary_web()
    rclpy.spin(node_intermediate)
    node_intermediate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
