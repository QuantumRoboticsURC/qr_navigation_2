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

# Origin and multiple targets
origin_lat, origin_lon = 19.5959507, -99.2258853
targets = [(19.5962508, -99.2256571), (19.5961391, -99.2255644), (19.5959643, -99.2256423),(19.5958306, -99.2258426)]

target_points = [ll2xy(lat, lon, origin_lat, origin_lon) for lat, lon in targets]
origin_x, origin_y = ll2xy(origin_lat, origin_lon, origin_lat, origin_lon)

# Initialize plot
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 16))
ax1.set_title("FollowGPS8 Rover Simulation")
ax1.set_xlabel("X (meters)")
ax1.set_ylabel("Y (meters)")

# Path elements
path, = ax1.plot([], [], 'b-', label='Rover Path')  
rover_point, = ax1.plot([], [], 'ro', label='Rover')  
ax1.plot(origin_x, origin_y, 'ks', markersize=8, label='Start')
for tx, ty in target_points:
    ax1.plot(tx, ty, 'go', markersize=10, label='Target')

# Second plot for error tracking
ax2.set_xlim(0, 300)
ax2.set_ylim(-5, 5)
ax2.set_title("GPS Position & Angle Error Over Time")
ax2.set_xlabel("Frame")
ax2.set_ylabel("Error Magnitude")

# Error plot elements
error_line, = ax2.plot([], [], 'r-', label='Position Error (m)')
angle_error_line, = ax2.plot([], [], 'b-', label='Angle Error (radians)')

# Third plot for IMU orientation
ax3.set_xlim(-1, 1)
ax3.set_ylim(-1, 1)
ax3.set_title("IMU Orientation (Compass View)")
ax3.set_xlabel("X")
ax3.set_ylabel("Y")
arrow = ax3.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='g')

# Data storage
gps_data = []
error_per_point = []
angle_errors = []
frame_numbers = []

yaw_angle = 0.0  # Rover initial yaw
velocity = 0.5   # Movement speed

# Path logic
current_target_idx = 0
num_points = 100
def update(frame):
    global yaw_angle, current_target_idx, origin_x, origin_y
    
    if current_target_idx >= len(target_points):
        return path, rover_point, error_line, angle_error_line, arrow
    
    target_x, target_y = target_points[current_target_idx]
    x_values = np.linspace(origin_x, target_x, num_points)
    y_values = np.linspace(origin_y, target_y, num_points)
    
    if frame < num_points:
        x = x_values[frame] + np.random.normal(0, 0.5)
        y = y_values[frame] + np.random.normal(0, 0.5)
        target_angle = np.arctan2(target_y - y, target_x - x)
        yaw_angle = simulate_imu(yaw_angle, target_angle)

        x_error = x - x_values[frame]
        y_error = y - y_values[frame]
        position_error = np.sqrt(x_error**2 + y_error**2)
        angle_error = yaw_angle - target_angle

        gps_data.append((x, y))
        error_per_point.append(position_error)
        angle_errors.append(angle_error)
        frame_numbers.append(frame)

        path.set_data([p[0] for p in gps_data], [p[1] for p in gps_data])
        rover_point.set_data([x], [y])
        error_line.set_data(frame_numbers, error_per_point)
        angle_error_line.set_data(frame_numbers, angle_errors)
        arrow.set_UVC(np.cos(yaw_angle), np.sin(yaw_angle))
    
    # Check if the rover reached the target
    if frame == num_points - 1:
        origin_x, origin_y = target_x, target_y
        current_target_idx += 1
    
    return path, rover_point, error_line, angle_error_line, arrow

ani = animation.FuncAnimation(fig, update, frames=num_points * len(targets), interval=100, blit=False)
ax1.legend()
ax2.legend()

plt.show()
