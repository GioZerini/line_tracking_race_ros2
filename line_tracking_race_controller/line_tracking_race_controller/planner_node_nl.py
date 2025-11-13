#!/usr/bin/env python3

# Libraries imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge

# Import delle strategie di pianificazione
from line_tracking_race_controller.error_type import ErrorType
from line_tracking_race_controller.nonlinear_strategy import NonLinearStrategy
from std_msgs.msg import Float32MultiArray

class PlannerNodeNl(Node):
    def __init__(self):
        super().__init__('planner')  

        # Parameters declarations
        self.declare_parameter("error_type", "offset")
        self.declare_parameter("viz", False)  

        error_type_arg = self.get_parameter("error_type").get_parameter_value().string_value
        viz = self.get_parameter("viz").get_parameter_value().bool_value

        # Error type
        if error_type_arg == "offset":
            error_type = ErrorType.OFFSET
        elif error_type_arg == "angle":
            error_type = ErrorType.ANGLE
        else:
            self.get_logger().error(f"Unknown error type {error_type_arg}. Exiting.")
            rclpy.shutdown()
            return

        # Selection of the strategy
        self.strategy = NonLinearStrategy(self, error_type, viz)

        # Creation of useful parameters
        self.bridge = CvBridge()
        self.error_pub = self.create_publisher(Float32, "/planning/error", 10)
        self.K = None
        self.robot_angle = 0.0

        # Creation of the publishers
        self.d_pub = self.create_publisher(Float32, "/vision/d", 10)
        self.psi_pub = self.create_publisher(Float32, "/vision/psi", 10)
        self.gamma_pub = self.create_publisher(Float32, "/vision/gamma", 10)

        # Creation of the subscribers
        self.camera_sub = self.create_subscription(
            Image, "/camera/image_raw", self.camera_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.get_logger().info("Planner node initialized.")

    # Every time we get a odom message
    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_angle = yaw

    # Extract the matrix to convert from pixel to meters
    def camera_info_callback(self, msg: CameraInfo):
        self.K = [
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]],
        ]

    # When an image arrives
    def camera_callback(self, msg: Image):
        if self.K is None:
            self.get_logger().warn("Camera matrix K not yet received — skipping frame.")
            return

        res = self.strategy.plan(msg, self.K, self.robot_angle)
        if res is None:
            err_msg = Float32()
            err_msg.data = 0.0
            self.error_pub.publish(err_msg)
            return

        # Check the nature of the res and manage it 
        if isinstance(res, dict):
            if "err" in res and res["err"] is not None:
                err_msg = Float32()
                err_msg.data = float(res["err"])
                self.error_pub.publish(err_msg)
                self.get_logger().info(f"Errore pianificato (compat): {res['err']:.3f}")

            if "d" in res:
                d_msg = Float32()
                d_msg.data = float(res["d"])
                self.d_pub.publish(d_msg)
            if "psi" in res:
                psi_msg = Float32()
                psi_msg.data = float(res["psi"])
                self.psi_pub.publish(psi_msg)
            if "gamma" in res:
                gamma_msg = Float32()
                gamma_msg.data = float(res["gamma"])
                self.gamma_pub.publish(gamma_msg)

            # Logging
            d_val = res.get("d", None)
            psi_val = res.get("psi", None)
            gamma_val = res.get("gamma", None)
            self.get_logger().info(
                f"NL features -> d:{d_val:.4f} m, psi:{psi_val:.4f} rad, gamma:{gamma_val:.4f} 1/m"
            )

        else:
            # Error messages
            try:
                err = float(res)
            except (TypeError, ValueError):
                self.get_logger().warn("Strategy returned an unexpected type; skipping publish.")
                return
            err_msg = Float32()
            err_msg.data = err
            self.error_pub.publish(err_msg)
            self.get_logger().info(f"Errore pianificato: {err:.3f}")

def main(args=None):
    # Keep the node alive until it receives CTRL+C
    rclpy.init(args=args)
    node = PlannerNodeNl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Planner node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
