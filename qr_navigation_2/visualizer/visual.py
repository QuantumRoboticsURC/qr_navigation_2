import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Function to convert lat/lon to XY
def ll2xy(lat, lon, orglat, orglong):
    """ Convert GPS coordinates to X, Y in meters. """
    x = (lon - orglong) * 111320  
    y = (lat - orglat) * 110540  
    return x, y

# Simulated IMU function (yaw angle in radians)
def simulate_imu(yaw, target_angle, noise=0.05):
    """ Simulates IMU-based yaw updates with noise. """
    angle_error = target_angle - yaw
    yaw += np.clip(angle_error, -0.05, 0.05)  # Adjust heading slightly
    yaw += np.random.normal(0, noise)  # Add IMU noise
    return yaw

# Origin and target
origin_lat, origin_lon = 19.4326, -99.1332  
target_lat, target_lon = 19.4330, -99.1340

target_x, target_y = ll2xy(target_lat, target_lon, origin_lat, origin_lon)
origin_x, origin_y = ll2xy(origin_lat, origin_lon, origin_lat, origin_lon)

# Generate path points
num_points = 100
x_values = np.linspace(origin_x, target_x, num_points)
y_values = np.linspace(origin_y, target_y, num_points)

# Initialize plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 12))

# Main path visualization
ax1.set_xlim(min(x_values) - 10, max(x_values) + 10)
ax1.set_ylim(min(y_values) - 10, max(y_values) + 10)
ax1.set_title("FollowGPS8 Rover Simulation")
ax1.set_xlabel("X (meters)")
ax1.set_ylabel("Y (meters)")

# Path elements
path, = ax1.plot([], [], 'b-', label='Rover Path')  
rover_point, = ax1.plot([], [], 'ro', label='Rover')  
ax1.plot(target_x, target_y, 'go', markersize=10, label='Target')  
ax1.plot(x_values, y_values, 'k--', label='Planned Path')  

# Second plot for error tracking
ax2.set_xlim(0, num_points)
ax2.set_ylim(-5, 5)  # Adjust scale for error visualization
ax2.set_title("GPS Position & Angle Error Over Time")
ax2.set_xlabel("Frame")
ax2.set_ylabel("Error Magnitude")

# Error plot elements
error_line, = ax2.plot([], [], 'r-', label='Position Error (m)')
angle_error_line, = ax2.plot([], [], 'b-', label='Angle Error (radians)')

# Data storage
gps_data = []
error_per_point = []
angle_errors = []
frame_numbers = []

# Initialize Rover State
yaw_angle = 0.0  # Rover initial yaw
velocity = 0.5   # Movement speed

def update(frame):
    """ Updates rover position, IMU yaw, and error tracking. """
    global yaw_angle

    if frame < num_points:
        # Simulated GPS noise
        x = x_values[frame] + np.random.normal(0, 0.5)
        y = y_values[frame] + np.random.normal(0, 0.5)

        # Target angle calculation
        target_angle = np.arctan2(target_y - y, target_x - x)

        # IMU-based yaw update
        yaw_angle = simulate_imu(yaw_angle, target_angle)

        # Compute errors
        x_error = x - x_values[frame]
        y_error = y - y_values[frame]
        position_error = np.sqrt(x_error**2 + y_error**2)
        angle_error = yaw_angle - target_angle

        # Store data
        gps_data.append((x, y))
        error_per_point.append(position_error)
        angle_errors.append(angle_error)
        frame_numbers.append(frame)

        # Update plot data
        path.set_data([p[0] for p in gps_data], [p[1] for p in gps_data])
        rover_point.set_data([x], [y])
        error_line.set_data(frame_numbers, error_per_point)
        angle_error_line.set_data(frame_numbers, angle_errors)

    return path, rover_point, error_line, angle_error_line

# Start animation
ani = animation.FuncAnimation(fig, update, frames=num_points, interval=100, blit=False)
ax1.legend()
ax2.legend()
plt.show()
