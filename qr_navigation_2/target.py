import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class Target(Node):
    def __init__(self):
        super().__init__('node_controller')

        # Publicador
        self.pub_target_type = self.create_publisher(Int8, 'target_type', 10)

        self.get_target_type()

    def get_target_type(self):
        while rclpy.ok():
            valor = int(input('Tipo de objetivo (-1 para salir): '))
            if valor == -1:
                break
            msg = Int8()
            msg.data = valor
            self.pub_target_type.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    target = Target()
    
    try:
        rclpy.spin(target)
    except KeyboardInterrupt:
        pass
    
    target.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
