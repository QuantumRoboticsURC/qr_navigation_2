import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FollowGPS

class NodeController(Node):
    def __init__(self):
        super().__init__('node_controller')
        self.client = self.create_client(FollowGPS, 'follow_gps')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FollowGPS.Request()

    def send_gps_coordinates(self, latitude, longitude):
        self.req.latitude = latitude
        self.req.longitude = longitude
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Response: %r' % (self.future.result().arrived))
        else:
            self.get_logger().error('Exception while calling service: %r' % (self.future.exception()))

def main(args=None):
    rclpy.init(args=args)
    node_controller = NodeController()
    # Definir las coordenadas 
    latitude = -34.603722
    longitude = -58.381592
    node_controller.send_gps_coordinates(latitude, longitude)
    node_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
