import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64

class Suscriptor(Node):
    def __init__(self):
        super().__init__("sus")
        self.listener = self.create_subscription(Float64,'test',self.callback,10)

    def callback(self,data):
        print(data.data)
    

def main(args=None):
    rclpy.init(args=args)
    p = Suscriptor()
    rclpy.spin(p)
    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()