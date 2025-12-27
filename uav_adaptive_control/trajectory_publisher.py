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
        self.Tf = 20.0  # Total time for one full loop
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)

        # Define the 4 corners of the square (Tilted in Z)
        # Order: Top-Right -> Top-Left -> Bottom-Left -> Bottom-Right
        # Z is calculated as z = 0.25*x + 0.25*y + 2.0 to ensure it is a flat tilted plane
        self.waypoints = [
            np.array([ 2.0,  2.0, 3.0]), # Corner 1
            np.array([-2.0,  2.0, 2.0]), # Corner 2
            np.array([-2.0, -2.0, 1.0]), # Corner 3
            np.array([ 2.0, -2.0, 2.0])  # Corner 4
        ]
        self.num_segments = len(self.waypoints)
        self.segment_time = self.Tf / self.num_segments

    def update(self):
        # 1. Handle time wrapping
        if self.t >= self.Tf:
            self.t -= self.Tf

        # 2. Determine which segment we are in (0 to 3)
        # current_idx is the start corner, next_idx is the destination
        current_idx = int(self.t // self.segment_time)
        next_idx = (current_idx + 1) % self.num_segments

        # 3. Calculate local time within the segment (0.0 to segment_time)
        t_local = self.t % self.segment_time
        
        # 4. Get Start and End points for this segment
        p_start = self.waypoints[current_idx]
        p_end   = self.waypoints[next_idx]

        # 5. Linear Interpolation (Position)
        # pos = start + (direction * percentage_complete)
        pos = p_start + (p_end - p_start) * (t_local / self.segment_time)

        # 6. Constant Velocity for this segment
        # vel = distance / time
        vel = (p_end - p_start) / self.segment_time

        # --- Construct Messages ---
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