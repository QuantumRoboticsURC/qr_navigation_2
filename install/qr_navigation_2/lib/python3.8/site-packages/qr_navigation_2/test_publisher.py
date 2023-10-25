import rclpy
from rclpy.node import Node 
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from std_msgs.msg import Float64

class sPublisher(Node):
    def __init__(self):
        super().__init__('test')
        self.pub = self.create_publisher(Float64, 'test',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        p = Float64()
    def timer_callback(self):
        p = Float64()
        p.data = 4.67
        self.pub.publish(p)




def main(args=None):
    rclpy.init(args=args)

    p = sPublisher()

    rclpy.spin(p)

    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()