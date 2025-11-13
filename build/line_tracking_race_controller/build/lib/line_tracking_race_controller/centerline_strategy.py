# Libraries imports
import math
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

from line_tracking_race_controller.error_type import ErrorType
from line_tracking_race_controller.visualizer import Visualizer
import line_tracking_race_controller.colors as colors

# In OpenCV hue ranges from 0 to 179
MAX_HUE = 179

# HSV thresholds
LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)

class CenterlineStrategy:
    def __init__(self, error_type, should_visualize, node):
        self.error_type = error_type
        self.node = node  # ROS2 node per logging

        if should_visualize:
            self.viz = Visualizer()
        else:
            self.viz = None

        self.cv_bridge = CvBridge()
        self.prev_offset = 0
        self.prev_waypoint = (0, 0)

    def plan(self, img_msg):
        image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        height, width, _ = image.shape

        track_outline = self.get_track_outline(image)

        cropped_outline = track_outline[
            int(height / 2):(height - 10), 100:(width - 100)
        ]
        cr_height, cr_width = cropped_outline.shape

        left_limit, right_limit = self.extract_track_limits(cropped_outline)
        centerline = self.compute_centerline(left_limit, right_limit)

        crosshair = (math.floor(cr_width / 2), math.floor(cr_height / 2))
        position = (math.floor(cr_width / 2), cr_height - 1)

        if left_limit.size == 0 or right_limit.size == 0:
            self.node.get_logger().warn("Can't compute centerline, reusing previous waypoint.")
            waypoint = self.prev_waypoint
            waypoint_offset = self.prev_offset
        else:
            waypoint, waypoint_offset = self.get_next_waypoint(centerline, crosshair)
            self.prev_waypoint = waypoint
            self.prev_offset = waypoint_offset

        if self.error_type == ErrorType.OFFSET:
            err, offset = self.compute_offset_error(waypoint, crosshair, cr_width / 2)
        elif self.error_type == ErrorType.ANGLE:
            err, angle = self.compute_angle_error(waypoint, position)
        else:
            self.node.get_logger().error("Unknown error type. Shutting down node.")
            self.node.destroy_node()
            return None

        if self.viz is not None:
            self.viz.build_track_bg(cr_height, cr_width, left_limit, right_limit, centerline)
            if self.error_type == ErrorType.OFFSET:
                self.viz.build_offset_error_overlay(crosshair, waypoint)
            elif self.error_type == ErrorType.ANGLE:
                self.viz.build_angle_error_overlay(crosshair, waypoint, position, angle)
            else:
                self.node.get_logger().error("Unknown error type. Shutting down node.")
                self.node.destroy_node()
                return None
            self.viz.show()

        return err

    def get_track_outline(self, input):
        height, width, _ = input.shape
        track_outline = np.zeros((height, width), dtype=np.uint8)

        hsv = cv.cvtColor(input, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array(LOWER_YELLOW), np.array(UPPER_YELLOW))

        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(track_outline, contours, 0, colors.MAGENTA)

        return track_outline

    def extract_track_limits(self, track_outline):
        _, labels = cv.connectedComponents(track_outline)

        left_limit_cols, left_limit_rows = np.where(labels == 1)
        left_limit = np.column_stack((left_limit_rows, left_limit_cols))[::10]

        right_limit_cols, right_limit_rows = np.where(labels == 2)
        right_limit = np.column_stack((right_limit_rows, right_limit_cols))[::10]

        return left_limit, right_limit

    def compute_centerline(self, left, right):
        centerline = []
        for (x1, y1), (x2, y2) in zip(left, right):
            xc = math.floor((x1 + x2) / 2)
            yc = math.floor((y1 + y2) / 2)
            centerline.append((xc, yc))

        return np.array(centerline)

    def get_next_waypoint(self, trajectory, crosshair):
        if trajectory.size == 0:
            return crosshair, 0

        center_x, center_y = crosshair
        closest = 0
        closest_dist = float("inf")

        for i, (x, y) in enumerate(trajectory):
            if y > center_y - 30:
                continue

            dist = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
            if dist < closest_dist:
                closest_dist = dist
                closest = i

        return trajectory[closest], trajectory[closest][0] - center_x

    def compute_offset_error(self, waypoint, crosshair, max_offset):
        offset = waypoint[0] - crosshair[0]
        return (offset + max_offset) / max_offset - 1, offset

    def compute_angle_error(self, waypoint, position):
        dist = math.sqrt((waypoint[0] - position[0]) ** 2 + (waypoint[1] - position[1]) ** 2)
        angle = math.asin((waypoint[0] - position[0]) / dist)
        angle_deg = angle * 180 / math.pi
        return (angle_deg + 90) / 90 - 1, angle_deg
