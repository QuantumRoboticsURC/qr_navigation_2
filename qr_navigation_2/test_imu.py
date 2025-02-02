import rclpy 
import math
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import Imu 
from geometry_msgs.msg import Quaternion

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)  # Clamping to avoid out-of-range values
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z  # in radians

class ImuNode(Node): 
    def __init__(self):
        super().__init__("test_imu")
        
        # Initialize IMU angle
        self.angle = 0.0
        
        # Set up Matplotlib figure for live plotting
        self.fig, self.ax1 = plt.subplots(figsize=(6, 6))
        self.ax1.set_xlim(-1, 1)
        self.ax1.set_ylim(-1, 1)
        self.ax1.set_title("IMU Orientation (Compass View)")
        self.ax1.set_xlabel("X")
        self.ax1.set_ylabel("Y")
        
        # Create an arrow to represent the orientation
        self.arrow = self.ax1.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='g')

        # ROS2 Subscriber
        self.create_subscription(Imu, "/bno055/imu", self.callback, 10)    

        # ROS2 Timer for updating the visualization
        self.timer = self.create_timer(0.1, self.orientation)
        
        # Enable interactive mode for live updating
        plt.ion()
        plt.show()

    def callback(self, data):
        quat = data.orientation
        _, _, angle_z = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        
        # Normalize the angle to be between 0 and 2π
        self.angle = (angle_z + 2 * math.pi) % (2 * math.pi)
        print(f"Yaw Angle: {math.degrees(self.angle):.2f}°")

    def orientation(self):
        # Calculate X, Y direction from the angle
        x = math.cos(self.angle)
        y = math.sin(self.angle)
        
        # Update the arrow direction
        self.arrow.set_UVC(x, y)
        
        # Redraw the figure dynamically
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    rclpy.spin(imu_node)
    
    # Cleanup after shutdown
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
