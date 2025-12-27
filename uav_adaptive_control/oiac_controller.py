#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench, Point, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

class OIAC(Node):
    def __init__(self):
        super().__init__("oiac_controller")

        # Physical Parameters
        self.mass = 2.2  # Approx mass of X3
        self.g = 9.81

        # Control Gains (Need Tuning)
        self.kp_att = np.array([8, 8, 8])
        self.kd_att = np.array([2.5, 2.5, 2.5])

        # Tracking state
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.v_angular = np.zeros(3)
        self.q = np.array([0., 0., 0., 1.]) # Quaternion (x,y,z,w)
        
        self.p_ref = np.zeros(3)
        self.v_ref = np.zeros(3)

        # Paper parameters (Adjusted for Rigid Body Dynamics)
        self.beta = 1.5 
        self.a = 1.0
        self.b = 0.2

        # Publishers
        # Publishes Wrench (Force + Torque) directly to Ignition Bridge
        self.wrench_pub = self.create_publisher(Wrench, "/cmd_force", 10)

        # Subscribers
        self.create_subscription(Odometry, "/model/X3/odometry", self.state_odometry, 10)
        self.create_subscription(PoseStamped, "/reference_pose", self.ref_pose, 10)
        self.create_subscription(TwistStamped, "/reference_twist", self.ref_vel, 10)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)

    def state_odometry(self, msg):
        self.p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.v_angular = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        # Capture orientation for World->Body transformation
        self.q = np.array([msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w])

    def ref_pose(self, msg):
        self.p_ref = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def ref_vel(self, msg):
        self.v_ref = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def get_allocation_from_force(self, force_world, yaw_target=0.0):
        """
        Converts a desired world force vector into Thrust + Attitude Setpoints.
        """
        # 1. Thrust
        thrust = np.linalg.norm(force_world)
    
        # 2. Desired Body Z axis (The direction we want to push)
        z_b = force_world / thrust
    
        # 3. Construct Rotation Matrix
        # Vector pointing East-ish (Yaw reference)
        y_ref = np.array([-np.sin(yaw_target), np.cos(yaw_target), 0])
    
        # Desired Body X (Forward)
        x_b = np.cross(y_ref, z_b)
        norm_x = np.linalg.norm(x_b)
        if norm_x < 1e-6:
            # Singularity (Force is straight up/down)
            # Keep current heading
            x_b = np.array() 
        else:
            x_b /= norm_x
        
        # Desired Body Y (Left)
        y_b = np.cross(z_b, x_b)
    
        # Rotation Matrix
        R_des = np.column_stack((x_b, y_b, z_b))
        return thrust, R_des

    def update(self):
        # 1. Error Calculation (World Frame)
        e = self.p - self.p_ref
        edot = self.v - self.v_ref
        eps = e + self.beta * edot

        # 2. OIAC Math Implementation
        # Eq 8: Adaptation scalar
        norm_eps_sq = np.dot(eps, eps)
        gamma = self.a / (1.0 + self.b * norm_eps_sq)

        # Eq 5: Adaptive Force
        F_adapt = eps / gamma

        # Eq 1 + 6 + 7: Control Law Substitution
        # K*e = F * (e.T * e) = F * ||e||^2
        # D*edot = F * (edot.T * edot) = F * ||edot||^2
        norm_e_sq = np.dot(e, e)
        norm_edot_sq = np.dot(edot, edot)
        
        # Total Control Force (World Frame)
        # u = -F - K*e - D*edot
        u_force_world = -F_adapt * (1.0 + norm_e_sq + norm_edot_sq)

        # 3. Physics Compensation
        # Add Feedforward Gravity Compensation (otherwise it falls)
        u_force_world[2] += self.mass * self.g

        # # 1. Desired Thrust Direction (Normalized Force Vector)
        # # We want the drone's Z-axis to align with u_force_world
        # total_thrust = np.linalg.norm(u_force_world)
        # z_b_des = u_force_world / total_thrust

        # # 2. Construct Desired Rotation Matrix (Simplified Calculation)
        # # We assume we want to keep the current yaw (or a reference yaw)
        # # Construct x_b_des and y_b_des using cross products
        # y_c = np.array([-np.sin(0), np.cos(0), 0]) # Assuming 0 yaw target for simplicity
        # x_b_des = np.cross(y_c, z_b_des)
        # x_b_des = x_b_des / np.linalg.norm(x_b_des)
        # y_b_des = np.cross(z_b_des, x_b_des)

        # # 4. Frame Transformation (World -> Body)
        # # Ignition ApplyLinkWrench applies forces in the Body Frame.
        # # We must rotate our World Frame force into the Body Frame.
        # rotation = R.from_quat(self.q)
        # # Invert rotation to get World->Body
        # u_force_body = rotation.inv().apply(u_force_world)

        # # 5. Publish Wrench
        # msg = Wrench()
        # msg.force.x = float(u_force_body[0])
        # msg.force.y = float(u_force_body[1])
        # msg.force.z = float(u_force_body[2])
        # # Torque is 0 because we are modeling a point-mass trajectory follower here
        # msg.torque.x = 0.0
        # msg.torque.y = 0.0
        # msg.torque.z = 0.0
        
        # self.wrench_pub.publish(msg)

        # CONVERT TO ATTITUDE
        target_thrust, R_des = self.get_allocation_from_force(u_force_world)
    
        # ATTITUDE CONTROLLER
        R_curr = R.from_quat(self.q).as_matrix()
    
        # Geometric Error (Lee et al.)
        # e_R = 0.5 * vee(R_d.T * R - R.T * R_d)
        error_mat = 0.5 * (R_des.T @ R_curr - R_curr.T @ R_des)

        # Extract unique elements from Skew-Symmetric Matrix
        # S = [[0, -ez, ey], [ez, 0, -ex], [-ey, ex, 0]]
        ex = error_mat[2, 1]  # Element at row 2, col 1 is 'x'
        ey = error_mat[0, 2]  # Element at row 0, col 2 is 'y'
        ez = error_mat[1, 0]  # Element at row 1, col 0 is 'z'
        e_R = np.array([ex, ey, ez]) # Vee map
    
        # Angular Velocity Error
        e_w = self.v_angular # Assuming target w=0 for stabilization
    
        # Compute Torque
        # Gains: Kp=10, Kd=2 (Tune these!)
        torque = -self.kp_att * e_R - self.kd_att * e_w

        self.get_logger().info('Torque: {}'.format(torque))
    
        # Publish
        msg = Wrench()
        msg.force.z = target_thrust
        msg.torque.x = torque[0]
        msg.torque.y = torque[1]
        msg.torque.z = torque[2]
        self.wrench_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OIAC()
    rclpy.spin(node)
