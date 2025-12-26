#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
import numpy as np

class OIAC(Node):
    def __init__(self):
        super().__init__("oiac_controller")

        # tracking state
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.p_ref = np.zeros(3)
        self.v_ref = np.zeros(3)

        # paper parameters
        self.beta = 0.6
        self.a = 2.0
        self.b = 2.5

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
