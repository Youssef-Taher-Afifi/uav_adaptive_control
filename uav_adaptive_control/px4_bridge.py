#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint
import numpy as np

class PX4Bridge(Node):
    def __init__(self):
        super().__init__("oiac_px4_bridge")
        self.create_subscription(Twist, "/cmd_acc", self.cmd_cb, 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, "/fmu/trajectory_setpoint/in", 10)

    def cmd_cb(self, msg):
        tx, ty, tz = msg.linear.x, msg.linear.y, msg.linear.z
        sp = TrajectorySetpoint()
        sp.acceleration = [float(tx), float(ty), float(tz)]
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_traj.publish(sp)

def main(args=None):
    rclpy.init(args=args)
    node = PX4Bridge()
    rclpy.spin(node)
