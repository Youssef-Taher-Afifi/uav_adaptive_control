#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
from rcl_interfaces.msg import SetParametersResult

class MRACController(Node):
    def __init__(self):
        super().__init__("mrac_controller")

        # Tracking state
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.p_ref = np.zeros(3)
        self.v_ref = np.zeros(3)
        
        # MRAC/PID state
        self.integral_error = np.zeros(3)
        self.prev_error = np.zeros(3)

        # 1. Declare Initial PID Gains (Table 5: Outer loop defaults approx P=0.1)
        self.declare_parameter('kp', 2.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 1.8)

        # 2. Declare Adaptation Learning Rates (Gamma)
        # Note: Default values set low to prevent instability during adaptation
        self.declare_parameter('gamma_p', 0.001)
        self.declare_parameter('gamma_i', 0.0001)
        self.declare_parameter('gamma_d', 0.001)

        # 3. Read initial values
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        self.gamma_p = self.get_parameter('gamma_p').value
        self.gamma_i = self.get_parameter('gamma_i').value
        self.gamma_d = self.get_parameter('gamma_d').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(Twist, "/X3/cmd_vel", 10)
        
        # --- Separate Publishers for Metrics (Identical to OIAC) ---
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
            if param.name == 'kp': self.kp = param.value
            elif param.name == 'ki': self.ki = param.value
            elif param.name == 'kd': self.kd = param.value
            elif param.name == 'gamma_p': self.gamma_p = param.value
            elif param.name == 'gamma_i': self.gamma_i = param.value
            elif param.name == 'gamma_d': self.gamma_d = param.value
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
        # 1. Calculate Errors
        # Paper defines e(t) = r(t) - y(t) [cite: 224]
        e = self.p_ref - self.p  
        edot = self.v_ref - self.v
        
        self.integral_error += e * self.dt

        # 2. MRAC Adaptation Law (MIT Rule) [cite: 229, 230, 231]
        # We use the inner product of vectors to drive the scalar gain adaptation
        # equations (15), (16), (17).
        
        # dKp = -gamma_p * e^2
        delta_kp = -self.gamma_p * np.dot(e, e) * self.dt
        
        # dKi = -gamma_i * e * integral(e)
        delta_ki = -self.gamma_i * np.dot(e, self.integral_error) * self.dt
        
        # dKd = -gamma_d * e * edot
        delta_kd = -self.gamma_d * np.dot(e, edot) * self.dt

        # Apply Adaptation
        self.kp += delta_kp
        self.ki += delta_ki
        self.kd += delta_kd

        # Optional: Safety clamping for gains to prevent instability/negative gains
        # (Though Eq 15 implies negative growth, practical MRAC often clamps)
        self.kp = max(0.0, self.kp)
        self.ki = max(0.0, self.ki)
        self.kd = max(0.0, self.kd)

        # 3. Calculate Control Output [cite: 221]
        # u = Kp*e + Ki*int(e) + Kd*edot
        u = (self.kp * e) + (self.ki * self.integral_error) + (self.kd * edot)

        # --- Metrics Calculation (Identical to OIAC) ---
        self.step_count += 1
        
        # 1. RMSE Calculation
        # sqrt( (1/T) * integral(e^2) )
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
        
        w_alpha, w_beta, w_gamma = 1.0, 1.0, 1.0
        
        denominator = (w_alpha * mean_K) + (w_beta * rms_a) + (w_gamma * rms_j)
        si_val = 1.0 / denominator if denominator > 1e-6 else 0.0

        self.prev_u = acc
        self.prev_error = e

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
    node = MRACController()
    rclpy.spin(node)