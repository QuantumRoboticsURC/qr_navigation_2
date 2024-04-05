import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from custom_interfaces.msg import TargetCoordinates


class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.publisher_ = self.create_publisher(Int8, 'state', 1)
        self.coords = self.create_publisher(TargetCoordinates,'/target_coordinates',1)
        self.timer = self.create_timer(2.0, self.request_target_type)

    def request_target_type(self):
        try:
            target_type = int(input("Ingrese el tipo de objetivo (0-5): "))
            if 0 <= target_type <= 5:
                target_type_msg = Int8()
                target_type_msg.data = target_type
                self.publisher_.publish(target_type_msg)
                target = TargetCoordinates()
                target.lat = 0
            else:
                print("El número ingresado está fuera del rango permitido (0-3). Vuelva a intentarlo.")
        except ValueError:
            print("Por favor, ingrese un número entero.")

def main(args=None):
    rclpy.init(args=args)
    web_node = WebNode()
    rclpy.spin(web_node)
    web_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
        
    
