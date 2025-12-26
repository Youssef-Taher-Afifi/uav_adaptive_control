import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get Package Paths
    pkg_uav_adaptive_control = get_package_share_directory('uav_adaptive_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. Define path to the specific world file
    # Matches path: uav_adaptive_control/gazebo/QuadcopterTeleop/world.sdf
    world_file_path = os.path.join(
        pkg_uav_adaptive_control,
        'gazebo',
        'QuadcopterTeleop',
        'world.sdf'
    )

    # 3. Launch Ignition Gazebo (ros_gz_sim)
    # The '-r' flag runs the simulation immediately upon loading
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    # 4. Bridge ROS 2 and Ignition Gazebo
    # Bridges the topics we set up in the SDF (Odometry and Command Velocity)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge Odometry: Ignition (Point/Vector3 data) -> ROS 2
            '/model/X3/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            
            # Bridge Control: ROS 2 (Twist) -> Ignition
            '/X3/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        ],
        output='screen'
    )

    # 5. Define Existing Nodes
    trajectory_publisher = Node(
        package="uav_adaptive_control", 
        executable="trajectory_publisher", 
        name="trajectory_publisher"
    )
    
    oiac_controller = Node(
        package="uav_adaptive_control", 
        executable="oiac_controller", 
        name="oiac_controller"
    )
    
    px4_bridge = Node(
        package="uav_adaptive_control", 
        executable="px4_bridge", 
        name="px4_bridge"
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        trajectory_publisher,
        oiac_controller,
        px4_bridge
    ])