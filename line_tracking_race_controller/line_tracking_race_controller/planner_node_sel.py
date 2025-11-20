#!/usr/bin/env python3

# Libraries imports
import math
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge

from line_tracking_race_controller.centroid_strategy import CentroidStrategy
from line_tracking_race_controller.centerline_strategy import CenterlineStrategy
from line_tracking_race_controller.nonlinear_strategy import NonLinearStrategy
from line_tracking_race_controller.error_type import ErrorType

# In OpenCV hue ranges from 0 to 179
MAX_HUE = 179

# HSV thresholds
LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)

# Planner node class
class PlannerNode_sel(Node):
    def __init__(self):
        super().__init__("planner")

        # Useful parameters
        self.declare_parameter("error_type", "offset")
        self.declare_parameter("viz", False)
        self.declare_parameter("strategy", "centroid")
        self.declare_parameter("turn_choice", "right")  # left|right|straight|alternate
        self.declare_parameter("turn_error_sign", -1.0) 
        self.declare_parameter("v_follow", 0.35)
        self.declare_parameter("v_enter",  0.12)
        self.declare_parameter("v_turn",   0.10)


        self.declare_parameter("roi_y_start_ratio", 0.50)  
        self.declare_parameter("roi_x_margin_px", 80)     
        self.declare_parameter("T_area_hi", 0.22)  
        self.declare_parameter("T_area_lo", 0.12)  
        self.declare_parameter("T_width_hi_ratio", 0.60) 
        self.declare_parameter("N_enter", 3)  
        self.declare_parameter("N_exit",  2)  
        self.declare_parameter("turn_timeout", 1.0)  
        self.declare_parameter("open_ksize", 3)
        self.declare_parameter("close_ksize", 5)
        self.declare_parameter("median_ksize", 5)
        
        self.turn_error_sign = float(self.get_parameter("turn_error_sign").value)
        error_type_arg = self.get_parameter("error_type").get_parameter_value().string_value
        viz = self.get_parameter("viz").get_parameter_value().bool_value
        strategy_name = self.get_parameter("strategy").get_parameter_value().string_value
        self.roi_y_start_ratio = float(self.get_parameter("roi_y_start_ratio").value)
        self.roi_x_margin_px   = int(self.get_parameter("roi_x_margin_px").value)
        self.T_area_hi         = float(self.get_parameter("T_area_hi").value)
        self.T_area_lo         = float(self.get_parameter("T_area_lo").value)
        self.T_width_hi_ratio  = float(self.get_parameter("T_width_hi_ratio").value)
        self.N_enter           = int(self.get_parameter("N_enter").value)
        self.N_exit            = int(self.get_parameter("N_exit").value)
        self.turn_choice       = str(self.get_parameter("turn_choice").value)
        self.turn_timeout      = float(self.get_parameter("turn_timeout").value)
        self.v_follow          = float(self.get_parameter("v_follow").value)
        self.v_enter           = float(self.get_parameter("v_enter").value)
        self.v_turn            = float(self.get_parameter("v_turn").value)
        self.open_ksize   = int(self.get_parameter("open_ksize").value)
        self.close_ksize  = int(self.get_parameter("close_ksize").value)
        self.median_ksize = int(self.get_parameter("median_ksize").value)

        # Error type
        if error_type_arg == "offset":
            self.error_type = ErrorType.OFFSET
        elif error_type_arg == "angle":
            self.error_type = ErrorType.ANGLE
        else:
            self.get_logger().error(f"Unknown error type {error_type_arg}. Exiting.")
            rclpy.shutdown(); return

        # Choice of the strategies
        if strategy_name == "centroid":
            self.strategy = CentroidStrategy(self, self.error_type, viz)
        elif strategy_name == "centerline":
            self.strategy = CenterlineStrategy(self, self.error_type, viz)
        elif strategy_name == "nonlinear":
            self.strategy = NonLinearStrategy(self, None, self.error_type, viz)
        else:
            self.get_logger().error(f"Unknown strategy {strategy_name}. Exiting.")
            rclpy.shutdown(); return

        # Create the needed publishers and subscribers
        self.bridge   = CvBridge()
        self.error_pub = self.create_publisher(Float32, "/planning/error", 10)
        self.pid_reset_pub = self.create_publisher(Bool, "/planner/pid_reset", 10)
        self.camera_sub = self.create_subscription(Image, "/camera/image_raw",
                                                   self.camera_callback, 10)

        # State machine variables to determine the robot state
        self.state = "FOLLOW"
        self.enter_cnt = 0
        self.exit_cnt  = 0
        self.turn_dir  = 0    # +1 left, -1 right, 0 straight
        self._last_dir = -1  
        self.turn_timer = None

        self.get_logger().info("Planner initialized.")

    # Camera callback function
    def camera_callback(self, msg: Image):
        # Convert the image 
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        H, W = bgr.shape[:2]

        # Get the lower part of the image
        y0 = int(self.roi_y_start_ratio * H)
        xL = self.roi_x_margin_px
        xR = W - self.roi_x_margin_px
        if xR <= xL:
            xL, xR = 0, W  
        roi = bgr[y0:, xL:xR]

        # Maschera gialla binaria 0/1
        mask01 = self.get_track(roi)  

        # Metrics to identify an intersection
        area_ratio = float(mask01.mean())  
        width_px   = self.width_estimate(mask01)  
        T_width_hi = self.T_width_hi_ratio * mask01.shape[1]
        is_broad   = (area_ratio > self.T_area_hi) or (width_px > T_width_hi)

        # Classic PID error
        err = self.line_error(msg) 

        # Finite State Machine
        if self.state == "FOLLOW":
            if is_broad:
                self.enter_cnt += 1
            else:
                self.enter_cnt = 0

            # If there is an intersection, we turn in the desidered direction
            if self.enter_cnt >= self.N_enter:
                self.state = "ENTER_X"
                self.set_turn_dir()
                self.enter_cnt = 0
                self.get_logger().info("FSM: FOLLOW -> ENTER_X")
            else:
                self.publish_error(err)

        elif self.state == "ENTER_X":
            self.publish_error(0.0)  
            self.exit_cnt += 1

            # If there is no more interesection, the robot has to turn
            if self.exit_cnt >= self.N_exit:
                self.state = "TURNING"
                self.exit_cnt = 0
                self.turn_timer = self.get_clock().now()
                self.get_logger().info("FSM: ENTER_X -> TURNING")
                self.pid_reset_pub.publish(Bool(data=True))

        elif self.state == "TURNING":
            # If the robot has to go straight, it doesn't turn
            if self.turn_dir == 0:
                err_follow = self.line_error(msg)
                err_turn = 0.0 
            else:
                # Else the robot turns in the desidered direction
                # turn_dir: +1 = right, -1 = left
                TURN_GAIN = 0.8
                err_turn = TURN_GAIN * self.turn_error_sign * (1.0 if self.turn_dir > 0 else -1.0)

            # Publish the PID error
            self.publish_error(err_turn)

            # Exit condition
            is_narrow = (area_ratio < self.T_area_lo) and (width_px < 0.5 * mask01.shape[1])

            timed_out = False
            if self.turn_timer is not None:
                dt_turn = (self.get_clock().now() - self.turn_timer).nanoseconds / 1e9
                timed_out = (dt_turn > self.turn_timeout)

            # If the line is back to normal dimension, the state comes back to "FOLLOW"
            if is_narrow or timed_out:
                self.state = "FOLLOW"
                self.straight_ok_cnt = 0
                self.get_logger().info("FSM: TURNING -> FOLLOW")

    def get_track(self, bgr_roi) -> np.ndarray:
        # Return the yellow mask 
        hsv = cv.cvtColor(bgr_roi, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array(LOWER_YELLOW, dtype=np.uint8),
                        np.array(UPPER_YELLOW, dtype=np.uint8))
        k_med = max(1, self.median_ksize | 1)  
        mask = cv.medianBlur(mask, k_med)

        if self.open_ksize > 0:
            k = cv.getStructuringElement(cv.MORPH_ELLIPSE, (self.open_ksize, self.open_ksize))
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, k)
        if self.close_ksize > 0:
            k = cv.getStructuringElement(cv.MORPH_ELLIPSE, (self.close_ksize, self.close_ksize))
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, k)

        # Normalize between 0 and 1
        mask01 = (mask > 0).astype(np.uint8)
        return mask01

    # Calculate the width of the yellow line
    def width_estimate(self, mask01: np.ndarray) -> float:
        h, w = mask01.shape[:2]
        N = min(12, max(4, h // 20))          
        rows = range(h - 1, h - 1 - N, -1)
        widths = []
        for r in rows:
            cols = np.flatnonzero(mask01[r, :])
            if cols.size >= 2:
                widths.append(float(cols[-1] - cols[0]))
        if len(widths) == 0:
            return 0.0
        return float(np.mean(widths))

    # Calculate the error by using the selected strategy
    def line_error(self, img_msg: Image) -> float:
        try:
            err = self.strategy.plan(img_msg)   
            return float(err) if err is not None else 0.0
        except Exception as e:
            self.get_logger().warn(f"Strategy error: {e}")
            return 0.0

    # Publish the calculated error
    def publish_error(self, e: float):
        e = max(-1.0, min(1.0, float(e)))
        self.error_pub.publish(Float32(data=e))

    # Set the direction of turning
    def set_turn_dir(self):
        ch = (self.turn_choice or "left").lower()
        if ch == "left":
            self.turn_dir = +1
        elif ch == "right":
            self.turn_dir = -1
        elif ch == "straight":
            self.turn_dir = 0
        elif ch == "alternate":
            self.turn_dir = +1 if self._last_dir < 0 else -1
            self._last_dir = self.turn_dir
        else:
            self.turn_dir = +1


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode_sel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Planner node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
