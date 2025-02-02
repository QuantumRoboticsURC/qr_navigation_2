import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Twist
from custom_interfaces.msg import CA

class CenterApproach(Node):
    def __init__(self):
        super().__init__("center_approach")

        # Subscriptions
        self.create_subscription(CA, "/center_approach", self.callback, 10)
        self.create_subscription(Bool, "/detected_aruco", self.aruco, 1)
        self.create_subscription(Bool, "/detected_orange", self.orange, 1)
        self.create_subscription(Bool, "/detected_bottle", self.bottle, 1)
        self.create_subscription(Int8, "/state", self.update_state, 1)

        # Publishers
        self.arrived_pub = self.create_publisher(Bool, "/arrived_ca", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_ca", 10)
        self.state_pub = self.create_publisher(Int8, "/state", 10)

        # Default state values
        self.state = -1
        self.Twist = Twist()

        # Velocity values
        self.vel_x = 0.16
        self.vel_theta = 0.06

        # CA topic variables
        self.distance = 0.0
        self.x = 0.0
        self.center = False

        # Flags
        self.found = False
        self.finish = False

        # Constants
        self.pixel_constant = 50
        self.center_distance_constant = 1200
        self.min_distance_constant = 1300
        self.relation = {2: "bottle", 3: "hammer", 4: "aruco"}

        # Timer
        self.timer = self.create_timer(0.1, self.center_and_approach)

    def update_state(self, msg):
        """Updates the state with data from the node controller."""
        self.state = msg.data

    def callback(self, msg):
        """Gets the data from the node detection CA topic and assigns it to variables."""
        self.distance = msg.distance
        self.x = msg.x
        self.center = msg.detected

    def aruco(self, msg):
        """Sets found to True if an aruco was detected."""
        self.found = msg.data

    def orange(self, msg):
        """Sets found to True if an orange object was detected."""
        self.found = msg.data

    def bottle(self, msg):
        """Sets found to True if a bottle was detected."""
        self.found = msg.data

    def approach(self):
        """Approaches the object while maintaining a minimum safe distance."""
        self.Twist.angular.z = 0.0
        arrived_msg = Bool()
        state_msg = Int8()

        if self.distance > self.min_distance_constant:
            self.Twist.linear.x = self.vel_x
        else:
            self.Twist.linear.x = 0.0
            self.found = False  # Consider an alternative approach instead of setting this to False
            self.finish = True

            arrived_msg.data = True
            state_msg.data = -1

            self.state_pub.publish(state_msg)
            self.arrived_pub.publish(arrived_msg)

        self.cmd_vel_pub.publish(self.Twist)

    def center_and_approach(self):
        """Centers and approaches detected objects based on state."""
        if self.state in self.relation:
            if self.found:
                self.get_logger().info(f"Entering center_and_approach for {self.relation[self.state]}")

                if not self.finish:
                    if self.center:
                        self.get_logger().info(f"{self.relation[self.state]} is centered")
                        self.approach()
                    elif self.x + self.pixel_constant * (self.center_distance_constant / (self.distance + 1e-6)) < 0:
                        self.Twist.linear.x = 0.0
                        self.Twist.angular.z = self.vel_theta
                        self.get_logger().info(f"{self.relation[self.state]} is on the left")
                    elif self.x - self.pixel_constant * (self.center_distance_constant / (self.distance + 1e-6)) > 0:
                        self.Twist.linear.x = 0.0
                        self.Twist.angular.z = -self.vel_theta
                        self.get_logger().info(f"{self.relation[self.state]} is on the right")

                    self.cmd_vel_pub.publish(self.Twist)

def main(args=None):
    rclpy.init(args=args)
    ca = CenterApproach()
    rclpy.spin(ca)
    ca.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
