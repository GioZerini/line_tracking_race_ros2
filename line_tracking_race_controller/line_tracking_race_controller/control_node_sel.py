#!/usr/bin/env python3

# Libraries imports
import os
from datetime import datetime
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32, Bool
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist


# Control constants
MAX_THRUST = 5     
RAMP_UP = 0.1     
TURNING_THRUST = 1  
WHEEL_BASE = 1.4 

# Control Node class
class ControlNode_sel(Node):
    def __init__(self):
        super().__init__("control_node")

        # Parameter declarations configurable via launch file
        self.declare_parameter("duration", -1)
        self.declare_parameter("k_p", 1.0)
        self.declare_parameter("k_i", 0.2)
        self.declare_parameter("k_d", 0.2)

        self.max_duration = self.get_parameter("duration").value
        self.k_p = self.get_parameter("k_p").value
        self.k_i = self.get_parameter("k_i").value
        self.k_d = self.get_parameter("k_d").value

        # Print PID parameters to the console
        self.get_logger().info(f"PID params: {self.k_p}, {self.k_i}, {self.k_d}")

        # Create log files for data logging and performance evaluation
        date = datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        self.pkg_path = get_package_share_directory("line_tracking_race_controller")
        self.open_logfile(date)
        self.open_performance_evaluation_file(date)

        # Initialize control variables
        self.setpoint = 0
        self.prev_error = 0
        self.accumulated_integral = 0
        self.thrust = 0
        self.started = False
        self.time_start = None
        self.time_prev = None

        # Create a publisher to control left and right wheels
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a subscription to the error topic to receive error measurements 
        self.create_subscription(Bool, "/planner/pid_reset", self._cb_pid_reset, 10)
        self.error_sub = self.create_subscription(Float32, "/planning/error", self.handle_error_callback, 10)

        self.get_logger().info("Control node initialized!")

    # Function called for each pid_reset message
    def _cb_pid_reset(self, msg):
        # Reset all the variables
        if msg.data:
            self.accumulated_integral = 0.0
            self.prev_error = 0.0
            self.get_logger().info("PID integrator reset by planner.")

    # Callback function to handle incoming error messages
    def handle_error_callback(self, msg):
        # Save the error measurement and the current time
        measurement = msg.data
        time_now = self.get_clock().now()

        # If the control node has not started yet, initialize the start time and set the started flag
        if self.started is False:
            self.time_start = time_now
            self.time_prev = time_now
            self.started = True
            return

        # Calculate elapsed time and time difference since the last measurement
        elapsed = (time_now - self.time_start).nanoseconds / 1e9
        dt = (time_now - self.time_prev).nanoseconds / 1e9

        # If the maximum duration is set and elapsed time exceeds it, stop the control node
        if self.max_duration >= 0 and elapsed > self.max_duration:
            self.get_logger().warn("Max duration reached.")
            self.stop()
            return

        error = measurement

        if dt <= 0:
            return

        # PID control calculations
        self.accumulated_integral += dt * (error + self.prev_error) / 2
        p_term = self.k_p * error
        i_term = self.k_i * self.accumulated_integral
        d_term = self.k_d * (error - self.prev_error) / dt
        control = p_term + i_term + d_term

        self.prev_error = error
        self.time_prev = time_now

        # Thrust control logic
        if self.thrust < MAX_THRUST:
            self.thrust += RAMP_UP

        # Calculate the left and right wheel velocities based on the control signal
        v_l = self.thrust + TURNING_THRUST * control
        v_r = self.thrust - TURNING_THRUST * control

        # Save the data and publish the wheel control commands
        self.log_data(elapsed, dt, error, control, v_l, v_r, p_term, i_term, d_term)
        self.publish_wheel_control(v_l, v_r)

    # Publish the calculated wheel velocities to the respective topics
    def publish_wheel_control(self, v_l, v_r):
        msg = Twist()
        # Calcola velocità lineare e angolare da v_l e v_r
        linear_vel = (v_l + v_r) / 2.0
        angular_vel = (v_r - v_l) / WHEEL_BASE  # WHEEL_BASE da definire in metri

        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)


    # Open a log file for data logging with the current PID parameters
    def open_logfile(self, date):
        pid_params = f"{self.k_p}-{self.k_i}-{self.k_d}".replace(".", ",")
        filepath = os.path.join(self.pkg_path, "logs", f"pid_log_{date}_[{pid_params}].csv")
        self.logfile = open(filepath, "w+")
        self.log_writer = csv.writer(self.logfile)
        self.log_writer.writerow(["Time", "dt", "Error", "CV", "LWheel", "RWheel", "P", "I", "D"])

    def open_performance_evaluation_file(self, date):
        pid_params = f"{self.k_p}-{self.k_i}-{self.k_d}".replace(".", ",")
        filepath = os.path.join(self.pkg_path, "logs", "evaluations", f"evaluation_{date}_[{pid_params}].csv")
        self.evaluation_file = open(filepath, "w+")
        self.performance_index_writer = csv.writer(self.evaluation_file)
        self.performance_index_writer.writerow(["ISE"])

    def log_data(self, elapsed, dt, error, control, v_l, v_r, p_term, i_term, d_term):
        self.log_writer.writerow([elapsed, dt, error, control, v_l, v_r, p_term, i_term, d_term])

    def log_performance_indices(self, ISE):
        self.performance_index_writer.writerow([ISE])

    # Stop the control node, log performance indices, and close the log files
    def stop(self):
        msg = Float64()
        msg.data = 0
        for _ in range(10):
            self.left_wheel_pub.publish(msg)
            self.right_wheel_pub.publish(msg)

        self.log_performance_indices(self.ISE)
        self.logfile.close()
        self.evaluation_file.close()
        self.get_logger().info("Control node shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode_sel()
    try:
        # Spin the node to keep it active and processing callbacks
        rclpy.spin(node)
    # Handle keyboard interrupts gracefully to stop the node
    except KeyboardInterrupt:
        node.stop()


if __name__ == "__main__":
    main()
