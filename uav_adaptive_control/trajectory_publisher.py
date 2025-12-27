#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path, Odometry
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        
        # --- Control Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, "/reference_pose", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/reference_twist", 10)

        # --- Visualization Publishers ---
        self.viz_ref_path_pub = self.create_publisher(Path, "/viz/reference_path", 10)
        self.viz_actual_path_pub = self.create_publisher(Path, "/viz/actual_path", 10)

        # --- Visualization Subscribers ---
        self.actual_sub = self.create_subscription(Odometry, "/model/X3/odometry", self.actual_path_callback, 10)

        # --- Trajectory Settings ---
        self.t = 0.0
        self.Tf = 20.0  
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)

        # Define the 4 corners
        self.waypoints = [
            np.array([ 2.0,  2.0, 3.0]), 
            np.array([-2.0,  2.0, 2.0]), 
            np.array([-2.0, -2.0, 1.0]), 
            np.array([ 2.0, -2.0, 2.0])  
        ]
        self.num_segments = len(self.waypoints)
        self.segment_time = self.Tf / self.num_segments

        # --- Initialize Path Messages ---
        self.ref_path_msg = self.generate_static_reference_path()
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = "map"

    def generate_static_reference_path(self):
        """Generates the full square loop once for visualization."""
        path = Path()
        path.header.frame_id = "map"
        
        corners_to_plot = self.waypoints + [self.waypoints[0]]
        
        for point in corners_to_plot:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            
        return path

    def actual_path_callback(self, msg):
        """
        Receives Odometry, extracts the pose, and appends it to the Path history.
        """
        # Create a new PoseStamped to hold the odometry position
        pose_stamped = PoseStamped()
        
        # Copy the timestamp from the odom message
        pose_stamped.header.stamp = msg.header.stamp
        # Force the frame_id to 'map' so it displays correctly in RViz relative to the reference
        pose_stamped.header.frame_id = "map"
        
        # Extract the pose from the Odometry message
        # Odometry structure is msg.pose.pose (PoseWithCovariance -> Pose)
        pose_stamped.pose = msg.pose.pose

        # Append to history
        self.actual_path_msg.poses.append(pose_stamped)
        
        # Limit history length
        if len(self.actual_path_msg.poses) > 5000:
            self.actual_path_msg.poses.pop(0)

        self.viz_actual_path_pub.publish(self.actual_path_msg)

    def update(self):
        # 1. Handle time wrapping
        if self.t >= self.Tf:
            self.t -= self.Tf

        # 2. Determine segment
        current_idx = int(self.t // self.segment_time)
        next_idx = (current_idx + 1) % self.num_segments

        # 3. Local time
        t_local = self.t % self.segment_time
        
        # 4. Get points
        p_start = self.waypoints[current_idx]
        p_end   = self.waypoints[next_idx]

        # 5. Interpolate (Position)
        pos = p_start + (p_end - p_start) * (t_local / self.segment_time)

        # 6. Constant Velocity
        vel = (p_end - p_start) / self.segment_time

        # --- Construct Messages ---
        msg_pos = PoseStamped()
        msg_pos.header.stamp = self.get_clock().now().to_msg()
        msg_pos.header.frame_id = "map"
        msg_pos.pose.position.x = float(pos[0])
        msg_pos.pose.position.y = float(pos[1])
        msg_pos.pose.position.z = float(pos[2])
        msg_pos.pose.orientation.w = 1.0

        msg_vel = TwistStamped()
        msg_vel.header = msg_pos.header
        msg_vel.twist.linear.x = float(vel[0])
        msg_vel.twist.linear.y = float(vel[1])
        msg_vel.twist.linear.z = float(vel[2])

        self.pose_pub.publish(msg_pos)
        self.twist_pub.publish(msg_vel)
        
        # Publish Reference Path
        self.ref_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_ref_path_pub.publish(self.ref_path_msg)

        self.t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main