from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
import launch


def generate_launch_description():
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(),
        Node(
            package='qr_navigation_2',
            executable='test_gps',
            name='test_gps'
        )
    ])