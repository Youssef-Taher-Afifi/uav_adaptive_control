#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64  # Added for metric publishing
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
        self.cmd_pub = self.create_publisher(Twist, "/X3/cmd_vel", 10)
        
        # --- Separate Publishers for Metrics ---
        self.rmse_pub = self.create_publisher(Float64, "/RMSE", 10)
        self.si_pub = self.create_publisher(Float64, "/SmoothnessIndex", 10)

        self.create_subscription(Odometry, "/model/X3/odometry", self.state_odometry, 10)
        self.create_subscription(PoseStamped, "/reference_pose", self.ref_pose, 10)
        self.create_subscription(TwistStamped, "/reference_twist", self.ref_vel, 10)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)

        # Metrics Initialization
        self.step_count = 0
        self.sum_sq_error = 0.0  # For RMSE
        
        # For Smoothness Index (SI)
        self.prev_u = np.zeros(3)
        self.sum_curv = 0.0
        self.sum_sq_acc = 0.0
        self.sum_sq_jerk = 0.0

    def parameters_callback(self, params):
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

        # --- Metrics Calculation ---
        self.step_count += 1
        
        # 1. RMSE Calculation
        self.sum_sq_error += np.sum(e**2)
        rmse_val = np.sqrt(self.sum_sq_error / self.step_count)

        # 2. Smoothness Index (SI)
        acc = u
        jerk = (acc - self.prev_u) / self.dt
        
        # Calculate Curvature K = |v x a| / |v|^3
        v_norm = np.linalg.norm(self.v)
        if v_norm > 1e-3:
            curvature = np.linalg.norm(np.cross(self.v, acc)) / (v_norm**3)
        else:
            curvature = 0.0
            
        self.sum_curv += curvature
        self.sum_sq_acc += np.sum(acc**2)
        self.sum_sq_jerk += np.sum(jerk**2)

        # Averages / RMS for SI formula
        mean_K = self.sum_curv / self.step_count
        rms_a = np.sqrt(self.sum_sq_acc / self.step_count)
        rms_j = np.sqrt(self.sum_sq_jerk / self.step_count)
        
        # Weights (assumed 1.0 as not specified in snippet)
        w_alpha, w_beta, w_gamma = 1.0, 1.0, 1.0
        
        denominator = (w_alpha * mean_K) + (w_beta * rms_a) + (w_gamma * rms_j)
        si_val = 1.0 / denominator if denominator > 1e-6 else 0.0

        # Store for next iteration
        self.prev_u = acc

        # --- Publish to Separate Topics ---
        rmse_msg = Float64()
        rmse_msg.data = float(rmse_val)
        self.rmse_pub.publish(rmse_msg)

        si_msg = Float64()
        si_msg.data = float(si_val)
        self.si_pub.publish(si_msg)
        # ----------------------------------

        msg = Twist()
        msg.linear.x = float(u[0])
        msg.linear.y = float(u[1])
        msg.linear.z = float(u[2])
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OIAC()
    rclpy.spin(node)