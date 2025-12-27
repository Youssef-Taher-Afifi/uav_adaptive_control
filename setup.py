from setuptools import find_packages, setup

package_name = 'uav_adaptive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/uav_adaptive_control/launch', ['launch/oiac.launch.py']),
        ('share/uav_adaptive_control/launch', ['launch/pid.launch.py']),
        ('share/uav_adaptive_control/launch', ['launch/mrac.launch.py']),
        ('share/uav_adaptive_control/gazebo/QuadcopterTeleop', ['gazebo/QuadcopterTeleop/world.sdf']),
        ('share/uav_adaptive_control/rviz', ['rviz/rviz_config.rviz']),
        ('share/uav_adaptive_control/config', ['config/oiac_plot.perspective']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youssef',
    maintainer_email='youssef@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = uav_adaptive_control.trajectory_publisher:main',
            'oiac_controller = uav_adaptive_control.oiac_controller:main',
            'pid_controller = uav_adaptive_control.pid_controller:main',
            'mrac_controller = uav_adaptive_control.mrac_controller:main',
            'px4_bridge = uav_adaptive_control.px4_bridge:main',    
        ],
    },
)
