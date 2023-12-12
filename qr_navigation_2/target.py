import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8

class Target(Node):
    def __init__ (self):
        super().__init__('node_controller')

        # Publicador
        self.pub_target_type = self.create_publisher(Int8, 'target_t', 10)

        # Variables porque me pierdo xd
        self.target_type = 0

        def ask_target_type(self, msg):
            int valor = input ('Tipo de objetivo: ')
            self.pub_target_type.publish(valor)
        
    def main(args=None):
        rclpy.init(args=args)
        target=Target()*