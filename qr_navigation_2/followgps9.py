import rclpy
from rclpy.node import Node
from rclpy.qos import * 
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Thread

from .submodules.alvinxy import *
from .submodules.constants import *

from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import NavSatFix,Imu,LaserScan
from std_msgs.msg import Int8,Bool,Int32
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from custom_interfaces.msg import TargetCoordinates
from std_msgs.msg import Float64,Float32MultiArray

import numpy
import math 
import time 


class Follow_GPS(Node):
    
    def __init__(self):
        super().__init__('gps9')

        #State publishers to give feedback to the node controller
        self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
        self.arrived_pub = self.create_publisher(Bool,'arrived_fg',1)
        self.state_pub = self.create_publisher(Int8,'state',1)
        #Subscribers to the node controller's publisher of the target coordinates
        self.target_coords = self.create_subscription(TargetCoordinates,"/target_coordinates",self.update_target,1)
        self.reset_coords = self.create_publisher(TargetCoordinates,"/target_coordinates",1)
        #Subscribers to the sensors' data
        #self.create_subscription(UBXNavHPPosLLH,'/gps_base/ubx_nav_hp_pos_llh',self.update_coords,qos_profile_sensor_data)
        self.create_subscription(Float64,'/latitude',self.update_coords_latitude,10)
        self.create_subscription(Float64,'/longitude',self.update_coords_longitude,10)
        self.create_subscription(Float32MultiArray, "/predicted_angle", self.update_angle, 10) 
        self.create_subscription(LaserScan,"/scan",self.lidar_callback,10)
        self.create_subscription(Int8,"/state",self.update_state,1)


        #Velocity data
        self.twist = Twist()
        self.linear_velocity = 0.25
        self.angular_velocity = 0.18
        self.obstacle_detected = False
        self.obstacle_routine = -1
        self.obstacle_min_distance = 4000
        
        #Coordinates and position on the plane
        self.gps_coordinates = [0.0,0.0]
        self.target_coordinates = [None,None]
        self.x_rover,self.y_rover,self.yaw_angle,self.pitch_angle,self.roll_angle = 0.0,0.0,0.0,0.0,0.0
        #Target x,y coordinates
        self.x_target = 0.0
        self.y_target = 0.0
        #Origin's latitude and logitude to map the plane using the alvinxy library
        self.orglong = None
        self.orglat = None
        #Flag for the initial coordinate registered
        self.HAS_STARTED = True
        #Default value of the state
        self.state = -1
        #Variable for range distance
        self.range_distance = 1.0
        self.just_started = [False,False,False]
        #Main
        self.timer = self.create_timer(0.01,self.followGPS)
        


    def update_target(self,msg):
        '''Sets the target coordinates to the given value'''
        self.target_coordinates = [msg.latitude,msg.longitude]

    def update_position(self):
        '''Updates the rover's position relative to the origin'''
        if self.orglat is not None and self.orglong is not None and self.gps_coordinates[0]!=0.0 and self.gps_coordinates[1]!=0.0:
            self.x_rover,self.y_rover = ll2xy(self.gps_coordinates[0] ,self.gps_coordinates[1] ,self.orglat,self.orglong)
    

    def update_coords_latitude(self,data):
        '''Updates the latitude of the rover's position'''
        if self.orglat is None:
            self.orglat = data.data/10000000.0
        self.gps_coordinates[0] = data.data/10000000.0
        self.update_position()

    def update_coords_longitude(self,data):
        '''Updates the longitude of the rover's position'''
        if self.orglong is None:
            self.orglong = data.data/10000000.0
        self.gps_coordinates[1] = data.data/10000000.0
        self.update_position()

    def update_state(self,msg):
        '''updates the state variable after receiving data from the controller'''
        self.state=msg.data
        
    def update_angle(self,msg):
        '''Updates the angle with the Imu's readings'''
        data = msg.data
        self.roll_angle = data[0]
        self.pitch_angle = data[1]
        self.yaw_angle = data[2]
        
    def lidar_callback(self, msg):
        '''Processes LiDAR scan data, only checking the front-facing portion'''
        self.lidar_ranges = msg.ranges  # Store full LiDAR scan for reference
        angle_min = msg.angle_min  # Min scan angle
        angle_increment = msg.angle_increment  # Angle step per reading
        total_ranges = len(self.lidar_ranges)

        # Define front sector (e.g., 60° in front)
        front_angle_range = math.radians(60)  # ±30° from the center
        front_indices = [
            i for i in range(total_ranges)
            if abs(angle_min + i * angle_increment) < front_angle_range / 2
        ]

        # Extract front-facing distances
        front_ranges = [self.lidar_ranges[i] for i in front_indices if not math.isinf(self.lidar_ranges[i])]

        # Ensure valid data exists
        if len(front_ranges) > 0:
            self.obstacle_min_distance = min(front_ranges)  # Closest object in front
        else:
            self.obstacle_min_distance = float('inf')  # No obstacles detected

        # Log LiDAR detection
        self.get_logger().info(f"Front obstacle distance: {self.obstacle_min_distance:.2f} m")


    def calc_angle(self):
        '''Calculates the target angle with the target position and the current position'''
        target_angle = math.atan2(self.y_target-self.y_rover,self.x_target-self.x_rover)
        return target_angle


    def direction_planner(self,target_angle):
        '''Decides the best direction to rotate towards the target angle'''
        ang_error = target_angle-self.yaw_angle
        ang_error_adj=math.atan2(math.sin(ang_error),math.cos(ang_error))
        return ang_error_adj/abs(ang_error_adj)

    
    def check_for_obstacles(self):
        '''Checks for obstacles and steep inclines, adjusting speed dynamically'''
        self.obstacle_detected = False

        if abs(self.pitch_angle) > 0.5:  # Steep incline detected
            self.obstacle_routine = 0  # Pitch-based obstacle
            self.obstacle_detected = True

        elif self.obstacle_min_distance < self.range_distance:  # LiDAR obstacle
            self.obstacle_routine = 1
            self.obstacle_detected = True
        
        if self.obstacle_detected:
            return  # Skip speed adjustment if stopping

        # Dynamic speed control for mild inclines
        incline_factor = max(0.2, 1.0 - abs(self.pitch_angle))  # Reduce speed on slopes
        self.linear_velocity = 0.2 * incline_factor

            
    def angle_correction(self,target_angle):
        '''Corrects the rover's angle based on its current position and target angle'''
        sign = self.direction_planner(target_angle)
        self.twist.angular.z = sign*self.angular_velocity
        self.twist.linear.x = 0.0

    def check_coord_precision(self):
        '''Boolean expression for the coordinate precision stoppage routine'''
        var = (
                (self.gps_coordinates[0]>(self.target_coordinates[0]-COORDINATE_ERROR) and self.gps_coordinates[0]<(self.target_coordinates[0]+COORDINATE_ERROR)) 
                and (self.gps_coordinates[1]>(self.target_coordinates[1]-COORDINATE_ERROR) and self.gps_coordinates[1]<(self.target_coordinates[1]+COORDINATE_ERROR))
        )		
        return var

    def check_distance_precision(self):
        '''Boolean expression for the distance precision stoppage routine'''
        var = ((self.x_rover>(self.x_target-DISTANCE_ERROR) and self.x_rover<(self.x_target+DISTANCE_ERROR))
             and (self.y_rover>(self.y_target-DISTANCE_ERROR) and self.y_rover<(self.y_target+DISTANCE_ERROR)))
        return var

    def check_angle_precision(self,target_angle):
        return not((self.yaw_angle>(target_angle-ANGLE_ERROR*2)) and (self.yaw_angle<(target_angle+ANGLE_ERROR*2)))

    def obstacle_evader(self):
        '''Handles obstacle evasion based on detected obstacle type'''
        self.twist.linear.x = 0.0  # Stop movement initially

        if self.obstacle_routine == 0:  # IMU-based obstacle (steep incline)
            self.get_logger().info("Steep incline detected, reversing and turning")
            self.twist.linear.x = -self.linear_velocity / 2  # Reverse slowly
            self.twist.angular.z = self.angular_velocity * 1.2  # Sharper turn to change path

        elif self.obstacle_routine == 1:  # LiDAR-based obstacle
            self.get_logger().info("Obstacle detected, finding clear path")
            
            # Scan left and right LiDAR ranges to decide the best turn
            left_clearance = min(self.lidar_ranges[len(self.lidar_ranges)//2:])  # Right side scan
            right_clearance = min(self.lidar_ranges[:len(self.lidar_ranges)//2])  # Left side scan

            if left_clearance > right_clearance:
                self.twist.angular.z = -self.angular_velocity  # Turn right
                self.get_logger().info("Turning right to avoid obstacle")
            else:
                self.twist.angular.z = self.angular_velocity  # Turn left
                self.get_logger().info("Turning left to avoid obstacle")

            self.twist.linear.x = 0.1  # Slight forward movement while turning

    def stop_movement(self):
        self.state = -1
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        self.target_coordinates=[None,None]
        self.HAS_STARTED=True
        self.orglat,self.orglong = None,None

        self.arrived_pub.publish(Bool(data=True))
        self.state_pub.publish(Int8(data=self.state))
        self.cmd_vel.publish(self.twist)
        
    def followGPSFunction(self,target_angle,distance):
        self.get_logger().info(f"Rover: {self.x_rover},{self.y_rover},a{self.yaw_angle}\n has a target angle of {target_angle}\ntarget: {self.x_target},{self.y_target}")
        
        if self.check_coord_precision() or self.check_distance_precision() or distance < 1:
            if self.check_coord_precision():
                self.get_logger().info("finished by coords")
            else:
                self.get_logger().info("finished by distance")
            
            self.stop_movement()
            return
        
        self.check_for_obstacles()
        
        if(self.obstacle_detected):
            self.get_logger().info(f"Obstacle")
            self.obstacle_evader()
        elif(self.check_angle_precision(target_angle)):
            self.get_logger().info(f"Rotate")
            self.angle_correction(target_angle)
        elif(distance > 2.5):
            self.get_logger().info("Correcting distance")
            self.get_logger().info(f"Go")
            self.twist.linear.x = self.linear_velocity
            self.twist.angular.z = 0.0
        self.cmd_vel.publish(self.twist)
    
    def followGPS(self):
        #print(self.state)
        if self.state==0: #Checks if the state is the one assigned to FGPS

            if not self.just_started[0]:
                self.get_logger().info("Entered Follow GPS v8.1")
                self.just_started[0]=True
    
            if(self.target_coordinates[0] is not None and self.target_coordinates[1] is not None): #Checks that the target coordinates are not null
                if not self.just_started[1]:
                    self.get_logger().info(f"The target coordinates are {self.target_coordinates}")
                    self.get_logger().info(f"The  coordinates are {self.gps_coordinates}")
                    self.just_started[1]=True

                if((self.gps_coordinates[0]!=0.0 and self.gps_coordinates[1]!=0.0 ) and
                   (self.orglat is not None and self.orglong is not None)): #Checks that the gps readings are valid
                    if not self.just_started[2]:
                        self.get_logger().info(f"The gps coordinates are valid, setting org to: {self.orglat},{self.orglong}")
                        self.get_logger().info(f"Routine")
                        self.just_started[2]=True
                        
                    #calculates the target x,y using the target coords and the origin
                    self.x_target,self.y_target = ll2xy(self.target_coordinates[0],self.target_coordinates[1],self.orglat,self.orglong)
                    #calculates the distance between the current position and the target position
                    distance = distanceBetweenCoords(self.gps_coordinates[0],self.gps_coordinates[1],self.target_coordinates[0],self.target_coordinates[1])
                    
                    target_angle = self.calc_angle()
                   
                    self.followGPSFunction(target_angle,distance)
                    
                    #self.get_logger().info(f"twist: {self.twist}")





def main(args=None):
    rclpy.init(args=args)
    gps = Follow_GPS()
    executor = MultiThreadedExecutor()
    executor.add_node(gps)
    executor.spin()
    gps.destroy_node()
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
