from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="uav_adaptive_control", executable="trajectory_publisher", name="trajectory_publisher"),
        Node(package="uav_adaptive_control", executable="oiac_controller", name="oiac_controller"),
        Node(package="uav_adaptive_control", executable="px4_bridge", name="px4_bridge"),
    ])
