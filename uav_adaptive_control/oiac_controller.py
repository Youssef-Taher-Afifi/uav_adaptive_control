#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import SetParametersResult

class OIAC(Node):
    def __init__(self):
        super().__init__("oiac_controller")

        # tracking state
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.p_ref = np.zeros(3)
        self.v_ref = np.zeros(3)

        # 1. Declare Parameters with default values
        # Default values should be conservative to start
        self.declare_parameter('beta', 0.6)
        self.declare_parameter('a', 2.0)
        self.declare_parameter('b', 2.5)

        # 2. Read initial values into class variables
        self.a = self.get_parameter('a').value
        self.b = self.get_parameter('b').value
        self.beta = self.get_parameter('beta').value

        # 3. Add the callback for real-time updates
        self.add_on_set_parameters_callback(self.parameters_callback)

        # publishers / subscribers
        # self.cmd_pub = self.create_publisher(Twist, "/cmd_acc", 10)
        self.cmd_pub = self.create_publisher(Twist, "/X3/cmd_vel", 10)
        self.create_subscription(Odometry, "/model/X3/odometry", self.state_odometry, 10)
        self.create_subscription(Point, "/quad/pose", self.state_pose, 10)
        self.create_subscription(Vector3, "/quad/velocity", self.state_vel, 10)
        self.create_subscription(PoseStamped, "/reference_pose", self.ref_pose, 10)
        self.create_subscription(TwistStamped, "/reference_twist", self.ref_vel, 10)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)

    def parameters_callback(self, params):
        """
        Callback function triggered whenever a "ros2 param set" command is received.
        """
        for param in params:
            if param.name == 'a':
                if param.value > 0.0:
                    self.a = param.value
                    self.get_logger().info(f"Updated 'a' to: {self.a}")
                else:
                    return SetParametersResult(successful=False, reason="Parameter 'a' must be positive")
            
            elif param.name == 'b':
                if param.value >= 0.0:
                    self.b = param.value
                    self.get_logger().info(f"Updated 'b' to: {self.b}")
                else:
                    return SetParametersResult(successful=False, reason="Parameter 'b' must be non-negative")

            elif param.name == 'beta':
                if param.value > 0.0:
                    self.beta = param.value
                    self.get_logger().info(f"Updated 'beta' to: {self.beta}")
                else:
                    return SetParametersResult(successful=False, reason="Parameter 'beta' must be positive")

        return SetParametersResult(successful=True)

    def state_odometry(self, msg):
        self.p = np.array([msg.pose.pose.position.x,
                           msg.pose.pose.position.y,
                           msg.pose.pose.position.z])
        self.v = np.array([msg.twist.twist.linear.x,
                           msg.twist.twist.linear.y,
                           msg.twist.twist.linear.z])

    def state_pose(self, msg):
        self.p = np.array([msg.x, msg.y, msg.z])

    def state_vel(self, msg):
        self.v = np.array([msg.x, msg.y, msg.z])

    def ref_pose(self, msg):
        self.p_ref = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def ref_vel(self, msg):
        self.v_ref = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def update(self):
        e = self.p - self.p_ref
        edot = self.v - self.v_ref
        eps = e + self.beta * edot

        gamma = self.a / (1 + self.b * np.linalg.norm(eps)**2)
        F = eps / gamma

        # Impedance update law from paper
        u = -F - gamma * e - self.beta * gamma * edot

        msg = Twist()
        msg.linear.x = float(u[0])
        msg.linear.y = float(u[1])
        msg.linear.z = float(u[2])
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OIAC()
    rclpy.spin(node)
