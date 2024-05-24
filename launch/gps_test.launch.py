from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
import launch


def generate_launch_description():

    return launch.LaunchDescription([
        #launch.actions.DeclareLaunchArgument(),
        launch_ros.actions.Node(
            package='qr_navigation_2',
            executable='followGPS',
            name='gps6',
            output='screen'
            
        ),
        launch_ros.actions.Node(
            package ='qr_navigation_2',
            executable='center_approach',
            name='center_approach',
            output='screen'
        ),
        launch_ros.actions.Node(
            
            package='qr_navigation_2',
            executable='controller',
            name='node_controller',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='qr_navigation_2',
            executable='auxiliary_search',
            name='auxiliary_search',
            output='screen'
        )
        
    ])
