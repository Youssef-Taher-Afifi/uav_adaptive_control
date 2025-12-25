#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        self.pose_pub = self.create_publisher(PoseStamped, "/reference_pose", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/reference_twist", 10)
        self.t = 0.0
        self.Tf = 20.0
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)

        # start/end points
        self.p0 = np.array([0., 0., 1.])
        self.pf = np.array([4., 4., 2.])

        # precompute 5th-order minimal-jerk coefficients
        self.a0 = self.p0
        self.a1 = np.zeros(3)
        self.a2 = np.zeros(3)
        self.a3 = 10*(self.pf - self.p0)/(self.Tf**3)
        self.a4 = -15*(self.pf - self.p0)/(self.Tf**4)
        self.a5 = 6*(self.pf - self.p0)/(self.Tf**5)

    def update(self):
        if self.t > self.Tf:
            self.t = 0.0

        tt = np.array([self.t**i for i in range(6)])
        pos = self.a0 + self.a3*tt[3] + self.a4*tt[4] + self.a5*tt[5]
        vel = 3*self.a3*tt[2] + 4*self.a4*tt[3] + 5*self.a5*tt[4]

        msg_pos = PoseStamped()
        msg_pos.header.stamp = self.get_clock().now().to_msg()
        msg_pos.pose.position.x = float(pos[0])
        msg_pos.pose.position.y = float(pos[1])
        msg_pos.pose.position.z = float(pos[2])

        msg_vel = TwistStamped()
        msg_vel.header = msg_pos.header
        msg_vel.twist.linear.x = float(vel[0])
        msg_vel.twist.linear.y = float(vel[1])
        msg_vel.twist.linear.z = float(vel[2])

        self.pose_pub.publish(msg_pos)
        self.twist_pub.publish(msg_vel)
        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
