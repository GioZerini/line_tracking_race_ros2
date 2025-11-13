import numpy as np
import math
import cv2 as cv

from line_tracking_race_controller.error_type import ErrorType
import line_tracking_race_controller.colors as colors

# Signals that the canvas was not initialized
class EmptyCanvasException(Exception):
    pass

# Utility class for visualization with OpenCV
class Visualizer:
    def __init__(self):
        self.canvas = None

    def build_track_bg(self, height, width, left_limit, right_limit, centerline):
        self.canvas = np.zeros((height, width, 3), dtype=np.uint8)

        for point in left_limit:
            cv.circle(self.canvas, point, 1, colors.LIGHT_BLUE, 1)
        for point in right_limit:
            cv.circle(self.canvas, point, 1, colors.YELLOW, 1)
        for point in centerline:
            cv.circle(self.canvas, point, 1, colors.MAGENTA, 1)

    def build_basic_bg(self, image):
        self.canvas = image

    def build_offset_error_overlay(self, crosshair, waypoint):
        if self.canvas is None:
            raise EmptyCanvasException()

        height, width, _ = self.canvas.shape
        crosshair = (math.floor(width / 2), math.floor(height / 2))

        cv.line(self.canvas, crosshair, waypoint, colors.WHITE, 2)
        cv.line(self.canvas, crosshair, (crosshair[0], waypoint[1]), colors.WHITE, 2)
        cv.line(self.canvas, (crosshair[0], waypoint[1]), waypoint, colors.RED, 2)
        cv.circle(self.canvas, crosshair, 5, colors.WHITE, 2)
        cv.circle(self.canvas, waypoint, 5, colors.WHITE, 2)

    def build_angle_error_overlay(self, crosshair, waypoint, position, angle):
        if self.canvas is None:
            raise EmptyCanvasException()

        height, width, _ = self.canvas.shape
        crosshair = (math.floor(width / 2), math.floor(height / 2))

        cv.line(self.canvas, position, waypoint, colors.WHITE, 2)
        cv.line(self.canvas, position, crosshair, colors.WHITE, 2)

        cv.ellipse(
            self.canvas,
            position,
            (60, 60),
            180,
            90,
            90 + angle,
            colors.RED,
            2,
        )

        cv.circle(self.canvas, crosshair, 5, colors.WHITE, 2)
        cv.circle(self.canvas, waypoint, 5, colors.WHITE, 2)

    def show(self):
        if self.canvas is None:
            raise EmptyCanvasException()
        cv.imshow("Visualization", self.canvas)
        cv.waitKey(1)

    def reset(self):
        self.canvas = None
