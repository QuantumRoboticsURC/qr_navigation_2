import rclpy
from rclpy.node import Node
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from rclpy.qos import *

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        print("BBBBBBBBBBBBB")

        # Use the SensorData QoS preset profile for sensor data.
        #qos = rclpy.qos.QoSProfile.qos_profile_sensor_data 

        # Create a subscriber for the UBXNavHPPosLLH topic with the specified QoS.
        self.subscription = self.create_subscription(
            UBXNavHPPosLLH,
            '/gps_rover/ubx_nav_hp_pos_llh',  # Adjust the topic name if necessary
            self.callback,
            qos_profile_sensor_data)

    def callback(self, msg):
        print("AAAAAAAAAAAAA")

        self.get_logger().info(f"Received UBXNavHPPosLLH: Lat={msg.lat}, Lon={msg.lon}")

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()