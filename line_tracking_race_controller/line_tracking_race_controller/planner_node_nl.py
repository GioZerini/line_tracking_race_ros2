#!/usr/bin/env python3

# Libraries imports. rclpy is the corresponding of rospy in ROS1
import rclpy
from rclpy.node import Node

# Import of message types, such as image and float
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# To convert images to OpenCV and viceversa
from cv_bridge import CvBridge

# Import of planning strategies
from line_tracking_race_controller.centroid_strategy import CentroidStrategy
from line_tracking_race_controller.centerline_strategy import CenterlineStrategy
from line_tracking_race_controller.error_type import ErrorType
from line_tracking_race_controller.nonlinear_strategy import NonLinearStrategy


# Planner Node which extends the Node class
class PlannerNodeNl(Node):
    def __init__(self):
        super().__init__('planner')  # Node name

        # Parameters declaration and initialization (needed in ROS2 before reading them)
        self.declare_parameter("error_type", "offset")
        self.declare_parameter("viz", False)  # It doesn't show the image
        # accepted: "centroid" | "centerline" | "nonlinear"
        self.declare_parameter("strategy", "centroid")

        error_type_arg = self.get_parameter("error_type").get_parameter_value().string_value
        viz = self.get_parameter("viz").get_parameter_value().bool_value
        strategy_name = self.get_parameter("strategy").get_parameter_value().string_value

        # Error type: offset -> offset of the line with respect to the image center
        # angle -> angle between the robot direction and the line direction
        if error_type_arg == "offset":
            error_type = ErrorType.OFFSET
        elif error_type_arg == "angle":
            error_type = ErrorType.ANGLE
        else:
            self.get_logger().error(f"Unknown error type {error_type_arg}. Exiting.")
            rclpy.shutdown()
            return

        # Line following strategy:
        # - centroid   -> easy mode, calculates the line centroid
        # - centerline -> advanced mode, calculates the median
        # - nonlinear  -> publishes d, psi, gamma for non-linear controller
        if strategy_name == "centroid":
            self.strategy = CentroidStrategy(self, error_type, viz)
        elif strategy_name == "centerline":
            self.strategy = CenterlineStrategy(self, error_type, viz)
        elif strategy_name == "nonlinear":
            self.strategy = NonLinearStrategy(self, error_type, viz)
        else:
            self.get_logger().error(f"Unknown strategy {strategy_name}. Exiting.")
            rclpy.shutdown()
            return

        # Object to convert from ROS image to OpenCV image
        self.bridge = CvBridge()

        # Publisher for legacy scalar error (compatibility with existing controllers)
        self.error_pub = self.create_publisher(Float32, "/planning/error", 10)

        # Publishers for non-linear controller inputs
        self.d_pub = self.create_publisher(Float32, "/vision/d", 10)
        self.psi_pub = self.create_publisher(Float32, "/vision/psi", 10)
        self.gamma_pub = self.create_publisher(Float32, "/vision/gamma", 10)

        # Create a subscriber to the camera images
        self.camera_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_callback,
            10
        )

        self.get_logger().info("Planner node initialized.")

    # It is called every time a new image arrives
    # Receive an image, compute the error/features and publish messages
    def camera_callback(self, msg: Image):
        # The strategy may return:
        # - scalar (float) error      -> centroid/centerline (legacy)
        # - dict {"err","d","psi","gamma"} -> nonlinear
        res = self.strategy.plan(msg)

        if res is None:
            # Heartbeat per far girare il controller
            err_msg = Float32(); err_msg.data = 0.0
            self.error_pub.publish(err_msg)
            return

        # Handle nonlinear dict result
        if isinstance(res, dict):
            # Publish legacy /planning/error for compatibility (use the 'err' provided)
            if "err" in res and res["err"] is not None:
                err_msg = Float32()
                err_msg.data = float(res["err"])
                self.error_pub.publish(err_msg)
                self.get_logger().info(f"Errore pianificato (compat): {res['err']:.3f}")

            # Publish /vision/d, /vision/psi, /vision/gamma
            if "d" in res:
                d_msg = Float32(); d_msg.data = float(res["d"]); self.d_pub.publish(d_msg)
            if "psi" in res:
                psi_msg = Float32(); psi_msg.data = float(res["psi"]); self.psi_pub.publish(psi_msg)
            if "gamma" in res:
                gamma_msg = Float32(); gamma_msg.data = float(res["gamma"]); self.gamma_pub.publish(gamma_msg)

            # Optional log (compact)
            d_val = res.get("d", None); psi_val = res.get("psi", None); gamma_val = res.get("gamma", None)
            self.get_logger().info(
                f"NL features -> d:{d_val:.4f} m, psi:{psi_val:.4f} rad, gamma:{gamma_val:.4f} 1/m"
            )

        else:
            # Legacy scalar error (float)
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
    # Create the node and put it in a listening mode, allowing the exit with CTRL+C
    rclpy.init(args=args)
    node = PlannerNodeNl()
    try:
        # Put the node in a listening mode
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Planner node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()