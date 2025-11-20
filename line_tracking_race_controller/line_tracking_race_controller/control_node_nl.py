#!/usr/bin/env python3

# Libraries imports
import os
import math
from datetime import datetime
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

MAX_THRUST = 0.5       
RAMP_UP = 0.1          
WHEEL_BASE = 1.4       
CONTROL_PERIOD = 0.05  

def sinc(x: float) -> float:
    return 1.0 if abs(x) < 1e-6 else math.sin(x)/x

class ControlNodeNl(Node):
    def __init__(self):
        super().__init__("control_node")

        # Useful parameters
        self.declare_parameter("duration", -1)
        self.declare_parameter("k_psi", 0.3)
        self.declare_parameter("k_dis", 0.15)
        self.declare_parameter("k_gamma", 0.45)
        self.declare_parameter("eps_div", 1e-3)

        self.max_duration = float(self.get_parameter("duration").value)
        self.k_psi = float(self.get_parameter("k_psi").value)
        self.k_dis = float(self.get_parameter("k_dis").value)
        self.k_gamma = float(self.get_parameter("k_gamma").value)
        self.eps_div = float(self.get_parameter("eps_div").value)

        # Logging
        date = datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        self.pkg_path = get_package_share_directory("line_tracking_race_controller")
        self._open_logs(date)

        # State variables
        self.time_start = None
        self.time_prev = None

        # Comando velocità
        self.v_ramp = 0.0
        self.v_ref = None

        # Feature NL
        self.d = 0.0
        self.psi = 0.0
        self.gamma = 0.0
        self.have_d = False
        self.have_psi = False
        self.have_gamma = False

        # Create the publisher for the wheels
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create the subscribers for d, psi and gamma
        self.sub_d = self.create_subscription(Float32, "/vision/d", self.cb_d, 10)
        self.sub_psi = self.create_subscription(Float32, "/vision/psi", self.cb_psi, 10)
        self.sub_gamma = self.create_subscription(Float32, "/vision/gamma", self.cb_gamma, 10)

        # Timer
        self.timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.get_logger().info("Control node initialized!")

    # Callback functions 
    def cb_d(self, msg: Float32):
        self.d = float(msg.data)
        self.have_d = True

    def cb_psi(self, msg: Float32):
        self.psi = float(msg.data)
        self.have_psi = True

    def cb_gamma(self, msg: Float32):
        self.gamma = float(msg.data)
        self.have_gamma = True

    # Function to control the robot
    def control_loop(self):
        # Calculate time to speed up the robot
        time_now = self.get_clock().now()
        if self.time_start is None:
            self.time_start = time_now
            self.time_prev = time_now
        elapsed = (time_now - self.time_start).nanoseconds / 1e9
        dt = (time_now - self.time_prev).nanoseconds / 1e9
        if dt <= 0.0:
            return
        if self.max_duration >= 0 and elapsed > self.max_duration:
            self.get_logger().warn("Max duration reached.")
            self.stop()
            return

        # Until it has not valid data, it proceeds straight
        if not (self.have_d and self.have_psi and self.have_gamma):
            self.v_ramp = min(MAX_THRUST, self.v_ramp + RAMP_UP * dt)
            self._publish_twist(0.25, 0.0)
            self.time_prev = time_now
            return

        # Obtain d, psi and gamma
        d = self.d
        psi = self.psi
        gamma = self.gamma

        # Set the velocity
        self.v_ramp = min(MAX_THRUST, self.v_ramp + RAMP_UP * dt)
        v = self.v_ramp

        # Denominator definition
        den = 1.0 - d * gamma
        if abs(den) < self.eps_div:
            den = self.eps_div if den >= 0 else -self.eps_div

        # Calculate the non-linear function 
        omega = (- self.k_psi * psi
                    - self.k_dis * d * v * sinc(psi)
                    + self.k_gamma * v * (math.cos(psi) * gamma) / den)

        # Publish the twist and log data
        self._publish_twist(v, omega)
        self._log_data(elapsed, dt, 0.0, omega, v, 0.0, 0.0, 0.0, 0.0)

        self.time_prev = time_now
        return

    # Function to publish the twist of the robot
    def _publish_twist(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_vel_pub.publish(msg)

    # Logging functions
    def _open_logs(self, date: str):
        logs_dir = os.path.join(self.pkg_path, "logs")
        eval_dir = os.path.join(self.pkg_path, "logs", "evaluations")
        os.makedirs(logs_dir, exist_ok=True)
        os.makedirs(eval_dir, exist_ok=True)

        tag = "nlpf"
        filepath = os.path.join(logs_dir, f"log_{date}_[{tag}].csv")
        self.logfile = open(filepath, "w+", newline="")
        self.log_writer = csv.writer(self.logfile)
        self.log_writer.writerow(["Time", "dt", "Error", "Omega", "V", "Dummy", "P", "I", "D"])

        evalpath = os.path.join(eval_dir, f"evaluation_{date}_[{tag}].csv")
        self.evaluation_file = open(evalpath, "w+", newline="")
        self.performance_index_writer = csv.writer(self.evaluation_file)
        self.performance_index_writer.writerow(["ISE"])

    def _log_data(self, elapsed, dt, error, omega, v, dummy, p_term, i_term, d_term):
        self.log_writer.writerow([elapsed, dt, error, omega, v, dummy, p_term, i_term, d_term])

    def _log_performance_indices(self):
        self.performance_index_writer.writerow([self.ISE])

    # Function to stop the robot 
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)

        self._log_performance_indices()
        self.logfile.close()
        self.evaluation_file.close()
        self.get_logger().info("Control node shutting down.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNodeNl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()

if __name__ == "__main__":
    main()
