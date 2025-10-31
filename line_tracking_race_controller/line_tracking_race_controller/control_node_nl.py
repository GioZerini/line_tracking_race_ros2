#!/usr/bin/env python3

import os
from datetime import datetime
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist


# Control constants
MAX_THRUST = 5     # Maximum engine thrust limit 
RAMP_UP = 0.1       # Thrust increase per second
TURNING_THRUST = 1  # Thrust used for turning
WHEEL_BASE = 1.4 

# Control node class used to implement a PID controller for line following
class ControlNodeNL(Node):
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

        # --- nuovi parametri nel __init__ ---
        self.declare_parameter("controller_type", "atan")   # "pid" | "atan" | "tanh" | "smc"
        self.declare_parameter("k_nl", 1.0)                # guadagno non lineare (atan/tanh)
        self.declare_parameter("lambda_nl", 2.0)           # ripidità non linearità

        self.max_duration = self.get_parameter("duration").value
        self.controller_type = self.get_parameter("controller_type").value
        self.k_nl     = float(self.get_parameter("k_nl").value)
        self.lambda_nl= float(self.get_parameter("lambda_nl").value)


        # Create log files for data logging and performance evaluation
        date = datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        self.pkg_path = get_package_share_directory("line_tracking_race_controller")
        self.open_logfile(date)
        self.open_performance_evaluation_file(date)

        # Initialize control variables
        self.prev_error = 0
        self.thrust = 0
        self.started = False
        self.ISE = 0
        self.time_start = None
        self.time_prev = None

        # Create two publisher to control left and right wheels
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a subscription to the error topic to receive error measurements and call the callback function
        self.error_sub = self.create_subscription(Float32, "/planning/error", self.handle_error_callback, 10)

        self.get_logger().info("Control node initialized!")

        self.int_err = 0.0     # integrale per PID/SMC
        self.int_limit = 2.0   # anti-windup (clip)

    # --- funzione di supporto ---
    def sat(self, x):
        # boundary layer "sat", più morbida di sign(x)
        if x > 1.0: return 1.0
        if x < -1.0: return -1.0
        return x
    
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
        # Calculate the Integral of Squared Error (ISE) for performance evaluation
        self.ISE += dt * (error * error + self.prev_error * self.prev_error) / 2

        if dt <= 0:
            return

        # anti-windup: integra solo se non saturi troppo e dt>0
        self.int_err += dt * (error + self.prev_error) / 2.0
        #    clamp integrale
        if self.int_err > self.int_limit:  self.int_err = self.int_limit
        if self.int_err < -self.int_limit: self.int_err = -self.int_limit

        # scegli il controllore
        if self.controller_type == "pid":
            p_term = self.k_p * error
            i_term = self.k_i * self.int_err
            d_term = self.k_d * (error - self.prev_error) / dt
            control = p_term + i_term + d_term

        elif self.controller_type == "atan":
            # non linearità morbida e “limitante”
            control = self.k_nl * ( __import__("math").atan(self.lambda_nl * error) )
            p_term, i_term, d_term = control, 0.0, 0.0

        elif self.controller_type == "tanh":
            control = self.k_nl * ( __import__("math").tanh(self.lambda_nl * error) )
            p_term, i_term, d_term = control, 0.0, 0.0

        elif self.controller_type == "smc":
            # sliding con integrale nel manifold
            s = error + self.smc_c * self.int_err
            control = - self.smc_k1 * self.sat( s / max(self.smc_phi, 1e-6) ) - self.smc_k2 * error
            p_term, i_term, d_term = control, 0.0, 0.0

        else:
            # fallback PID
            p_term = self.k_p * error
            i_term = self.k_i * self.int_err
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
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)

        self.log_performance_indices(self.ISE)
        self.logfile.close()
        self.evaluation_file.close()
        self.get_logger().info("Control node shutting down.")
        rclpy.shutdown()


def main(args=None):
    # Initialize the ROS 2 Python client library and create the control node
    rclpy.init(args=args)
    node = ControlNodeNL()
    try:
        # Spin the node to keep it active and processing callbacks
        rclpy.spin(node)
    # Handle keyboard interrupts gracefully to stop the node
    except KeyboardInterrupt:
        node.stop()


if __name__ == "__main__":
    main()


# ======================== DIFFERENZE CON VERSIONE ROS1 =========================
# - La struttura e la logica sono rimaste pressoché invariate, ma con adattamenti a ROS 2:
#   - In ROS2 si usa `rclpy` invece di `rospy`, e la classe `Node` è usata per tutti i nodi.
#   - Parametri come k_p, k_i, k_d sono gestiti con `declare_parameter` e `get_parameter`.
#   - Il logging, la gestione del tempo e dei publisher/subscriber è ora asincrona e più modulare.
#   - La funzione `spin()` tiene il nodo attivo; lo shutdown si fa esplicitamente con `rclpy.shutdown()`.
# - Il comportamento PID, il calcolo dell'errore e la gestione del controllo motori restano identici.
# - L'output viene salvato su file CSV allo stesso modo per facilitare il confronto tra versioni.
# ================================================================================

