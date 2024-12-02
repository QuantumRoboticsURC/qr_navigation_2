import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class Web(Node):
    def __init__(self):
        super().__init__("Web")
        self.publish = self.create_publisher(Int8,"state",10)
        self.timer = self.create_timer(0.01,self.pub)
        
    def pub(self):
        state =Int8()
        state.data = int(input("Ingrese el valor: "))
        self.publish.publish(state)

def main(args=None):
	rclpy.init(args=args)
	gps = Web()
	rclpy.spin(gps)
	gps.destroy_node()
	rclpy.shutdown()

    
if __name__=="__main__":
    main()
         
   
        
        
    