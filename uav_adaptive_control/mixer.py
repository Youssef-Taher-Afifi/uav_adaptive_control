#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from actuator_msgs.msg import Actuators  # Standard ROS 2 message for Actuators
import numpy as np

class QuadMixer(Node):
    def __init__(self):
        super().__init__("quad_mixer")

        # Physical Parameters (X3)
        self.Ct = 8.54858e-06
        self.Cm = 0.016
        self.d = 0.26 * np.cos(np.radians(45))
        
        # Mixer Matrix (M)
        self.M = np.array([
            [self.Ct,          self.Ct,         self.Ct,         self.Ct       ], # Thrust
            [-self.d*self.Ct,  self.d*self.Ct,  self.d*self.Ct, -self.d*self.Ct], # Roll
            [-self.d*self.Ct,  self.d*self.Ct,  -self.d*self.Ct, self.d*self.Ct], # Pitch
            [-self.Cm,        -self.Cm,         self.Cm,         self.Cm       ]  # Yaw
])
        self.M_inv = np.linalg.pinv(self.M)

        # Subscribers
        self.sub_wrench = self.create_subscription(Wrench, "/cmd_force", self.wrench_cb, 10)
        
        # Publisher (Single topic for all actuators)
        self.pub_actuators = self.create_publisher(Actuators, "/X3/command/motor_speed", 10)

    def wrench_cb(self, msg):
        u = np.array([msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z])

        # Mixing
        w_sq = self.M_inv @ u
        w_sq = np.maximum(w_sq, 0)
        w = np.sqrt(w_sq)

        # Create Actuators Message
        act_msg = Actuators()
        # The X3 model expects motor velocities in the 'velocity' array
        # Order: [Motor 0, Motor 1, Motor 2, Motor 3]
        act_msg.velocity = [float(w[0]), float(w[1]), float(w[2]), float(w[3])]
        
        self.pub_actuators.publish(act_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QuadMixer()
    rclpy.spin(node)