#!/usr/bin/env python3

# Libraries imports
import rclpy
from rclpy.node import Node

# Import of message types
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# Library to convert images to OpenCV and viceversa
from cv_bridge import CvBridge

# Import of planning strategies
from line_tracking_race_controller.centroid_strategy import CentroidStrategy
from line_tracking_race_controller.centerline_strategy import CenterlineStrategy
from line_tracking_race_controller.error_type import ErrorType

# Planner Node which extends the Node class
class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner') 

        # Parameters declaration and initialization 
        self.declare_parameter("error_type", "offset")
        self.declare_parameter("viz", False) 
        self.declare_parameter("strategy", "centroid")
        error_type_arg = self.get_parameter("error_type").get_parameter_value().string_value
        viz = self.get_parameter("viz").get_parameter_value().bool_value
        strategy_name = self.get_parameter("strategy").get_parameter_value().string_value

        # Error type: 
        # offset -> offset of the line with respect to the image center
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
        # - centroid -> it calculates the line centroid
        # - centerline -> it calculates the median
        if strategy_name == "centroid":
            self.strategy = CentroidStrategy(self, error_type, viz)
        elif strategy_name == "centerline":
            self.strategy = CenterlineStrategy(self, error_type, viz)
        else:
            self.get_logger().error(f"Unknown strategy {strategy_name}. Exiting.")
            rclpy.shutdown()
            return

        # Object to convert from ROS image to OpenCV image
        self.bridge = CvBridge()

        # Create the publisher for the errors, in order to know of how much turn the wheels
        self.error_pub = self.create_publisher(Float32, "/planning/error", 10)
        
        # Create a subscriber to the camera images
        self.camera_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_callback,
            10
        )

        self.get_logger().info("Planner node initialized.")

    # It is called every time a new image arrives
    # Receive an image, compute the error and publish a message
    def camera_callback(self, msg):
        err = self.strategy.plan(msg)
        self.get_logger().info(f"Errore pianificato: {err}")

        if err is None:
            return
        err_msg = Float32()
        err_msg.data = err
        self.error_pub.publish(err_msg)


def main(args=None):
    # Create the node and put it in a listening mode, allowing the exit with CTRL+C 
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        # Put the node in a listening mode
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Planner node shutting down.")

    # Cleanup the resourses and close the ROS2 system
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
