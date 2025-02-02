import rclpy
import math
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from filterpy.kalman import KalmanFilter


def euler_from_quaternion(x, y, z, w):
    """Converts quaternion to Euler angles (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)  # Clamping
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class KalmanImu:
    """Implements a Kalman Filter for IMU data fusion."""

    def __init__(self):
        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        # State transition model
        self.kf.F = np.eye(6)
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],  # Roll
                              [0, 1, 0, 0, 0, 0],  # Pitch
                              [0, 0, 1, 0, 0, 0]])  # Yaw

        # Covariances
        self.kf.P *= 1000  # Initial uncertainty
        self.kf.R = np.eye(3) * 5  # Measurement noise
        self.kf.Q = np.eye(6) * 0.01  # Process noise
        self.kf.x = np.zeros(6)  # [Roll, Pitch, Yaw, dRoll, dPitch, dYaw]

    def predict(self, gyro_x, gyro_y, gyro_z, dt):
        """Performs Kalman prediction using gyroscope data."""
        if dt <= 0:  # Prevent division errors
            return 0, 1

        # Update state transition matrix with time step dt
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt

        # Angular velocity update
        self.kf.x[3] = gyro_x
        self.kf.x[4] = gyro_y
        self.kf.x[5] = gyro_z

        self.kf.predict()

        # Compute predicted yaw direction
        yaw = self.kf.x[2]
        return math.cos(yaw), math.sin(yaw)


class ImuNode(Node):
    """ROS 2 Node for IMU processing and visualization."""

    def __init__(self):
        super().__init__("imu_kalman")

        # Initialize variables
        self.angle = 0.0
        self.gyro_x = self.gyro_y = self.gyro_z = 0.0
        self.last_time = self.get_clock().now()
        self.predictor = KalmanImu()

        # Set up live plotting
        self.fig, self.ax1 = plt.subplots(figsize=(6, 6))
        self.ax1.set_xlim(-1, 1)
        self.ax1.set_ylim(-1, 1)
        self.ax1.set_title("IMU Orientation (Compass View)")
        self.ax1.set_xlabel("X")
        self.ax1.set_ylabel("Y")

        # Plot Arrows: Green = Actual, Red = Predicted, Blue = Filtered
        self.arrow = self.ax1.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='g')
        self.predicted_arrow = self.ax1.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='r')
        self.weighted_arrow = self.ax1.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='b')

        # ROS2 Subscription & Timer
        self.create_subscription(Imu, "/bno055/imu", self.callback, 10)
        self.timer = self.create_timer(0.1, self.orientation)

        plt.ion()
        plt.show()

    def callback(self, data):
        """Processes incoming IMU data."""
        self.gyro_x = data.angular_velocity.x
        self.gyro_y = data.angular_velocity.y
        self.gyro_z = data.angular_velocity.z

        # Convert quaternion to yaw
        quat = data.orientation
        _, _, angle_z = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        self.angle = (angle_z + 2 * math.pi) % (2 * math.pi)

        print(f"Yaw Angle: {math.degrees(self.angle):.2f}°")

    def orientation(self):
        """Updates visualization using Kalman Filter predictions."""
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) * 1e-9
        self.last_time = current_time

        pred_x, pred_y = self.predictor.predict(self.gyro_x, self.gyro_y, self.gyro_z, dt)
        x = math.cos(self.angle)
        y = math.sin(self.angle)

        alpha = 0.6  # Complementary filter parameter
        x_weighted = alpha * x + (1 - alpha) * pred_x
        y_weighted = alpha * y + (1 - alpha) * pred_y

        self.arrow.set_UVC(x, y)
        self.predicted_arrow.set_UVC(pred_x, pred_y)
        self.weighted_arrow.set_UVC(x_weighted, y_weighted)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    imu_node = ImuNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
