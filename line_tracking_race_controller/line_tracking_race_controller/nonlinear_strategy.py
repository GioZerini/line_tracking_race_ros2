#!/usr/bin/env python3

# Libraries imports 
import math
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

from line_tracking_race_controller.visualizer import Visualizer

LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)

class NonLinearStrategy:
    def __init__(self, node, error_type, should_visualize):
        # Strategy parameters
        self.error_type = error_type
        self.node = node
        self.cv_bridge = CvBridge()
        self.prev_waypoint = (0, 0)
        self.prev_offset_px = 0
        self.should_visualize = should_visualize
        self.viz = Visualizer() if should_visualize else None
        self.last_valid = None

        self.camera_height = 0.12   
        self.camera_pitch  = math.pi / 6 

    def plan(self, img_msg, camera_info_msg, robot_angle):
        # Convert the image
        image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        height, width, _ = image.shape

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array(LOWER_YELLOW), np.array(UPPER_YELLOW))

        # Compute centroid
        M = cv.moments(mask)
        if M["m00"] != 0:
            centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            self.prev_centroid = centroid
        else:
            if self.prev_centroid is None:
                self.node.get_logger().warn("No centroid found and no previous to reuse. Using center.")
                centroid = (width // 2, height // 2)
            else:
                self.node.get_logger().warn("No centroid found, reusing previous waypoint.")
                centroid = self.prev_centroid

        # Conversion matrix
        K = np.array(camera_info_msg)
        crosshair = (width // 2, height // 2)

        # Call the estimate_d_psi_gamma function
        d, psi, gamma = self.estimate_d_psi_gamma(
            mask=mask,
            crosshair_px=crosshair,
            robot_angle=robot_angle,
            K=K,
            image=image
        )

        # Visualization part
        if self.should_visualize:
            cv.circle(image, crosshair, 5, (0, 255, 0), 2)   
            cv.circle(image, centroid, 5, (0, 0, 255), 2)    
            cv.putText(image, f"d={d:.3f} m", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv.putText(image, f"psi={psi:.3f} rad", (10, 70), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv.putText(image, f"gamma={gamma:.3f} 1/m", (10, 110), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            cv.imshow("Line Following Visualization", image)
            cv.waitKey(1)

        return {"d": d, "psi": psi, "gamma": gamma}
    
    # Function to estimate d, psi and gamma
    def estimate_d_psi_gamma(self, mask, crosshair_px, robot_angle, K, image):
        # Useful parameters
        fx, fy = K[0, 0], K[1, 1]
        cx_px, cy_px = crosshair_px
        height, width = mask.shape
        Z = self.camera_height / np.cos(self.camera_pitch)

        # Find the contours of the line
        contours, _ = cv.findContours(mask.astype(np.uint8), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, 0.0, 0.0

        # Take the maximum contour
        cnt = max(contours, key=cv.contourArea)
        if cnt.shape[0] < 2:
            return 0.0, 0.0, 0.0

        mask_uint8 = (mask > 0).astype(np.uint8) * 255
        dist = cv.distanceTransform(mask_uint8, cv.DIST_L2, 5)

        # Select the middle points for each row in the image
        x_center_per_row, y_valid_center = [], []
        for y in range(height):
            xs = np.where(mask[y, :] > 0)[0]
            if xs.size == 0: continue
            best_idx = xs[np.argmax(dist[y, xs])]
            if dist[y, best_idx] >= 0.5:
                x_center_per_row.append(float(best_idx))
                y_valid_center.append(y)

        # Check if the points are sufficient and convert them into arrays
        if len(x_center_per_row) < 2:
            return 0.0, 0.0, 0.0
        x_center_per_row = np.array(x_center_per_row)
        y_valid_center = np.array(y_valid_center)

        # Smoothing to take only central points
        try:
            from scipy.signal import medfilt
            kernel = 5 if len(x_center_per_row) >= 5 else 3
            x_center_sm = medfilt(x_center_per_row, kernel_size=kernel)
        except:
            x_center_sm = x_center_per_row

        # We consider only the points near to the crosshair
        MAX_DIST_PX = 30
        valid = np.abs(y_valid_center - cy_px) < MAX_DIST_PX
        x_center_sm = x_center_sm[valid]
        y_valid_center = y_valid_center[valid]
        if len(x_center_sm) < 2:
            return 0.0, 0.0, 0.0

        # Central point near to the crosshair
        idx_y = np.argmin(np.abs(y_valid_center - cy_px))
        line_cx_px = x_center_sm[idx_y]
        line_cy_px = -y_valid_center[idx_y]

        # d = Lateral distance from the robot to the centerline
        d_m = (line_cx_px - cx_px) * Z / fx

        # Psi calculation
        psi = 0.0
        theta_line = 0.0

        # Local polinomial fit
        local_mask = np.abs(y_valid_center - cy_px) < 50 
        if np.sum(local_mask) >= 5:
            x_local = x_center_sm[local_mask]
            y_local = -y_valid_center[local_mask]
            coeffs = np.polyfit(x_local, y_local, 2) 
            a2, a1, a0 = coeffs

            # Calculate the first derivative to derive psi
            x0 = line_cx_px 
            slope = 2*a2*x0 + a1
            theta_line = math.atan(slope)  
            psi = self._wrap_pi(robot_angle - theta_line)
            xc = x_local.mean()
            dx = xc - cx_px
            psi = math.copysign(abs(psi), dx)

            # Calculate gamma via second derivative
            gamma_px = (2 * a2) / (1 + a1**2)**1.5
            gamma = gamma_px * (fx / Z)
        else:
            gamma = 0.0

        # Draw useful information on the viz window
        cv.circle(image, (int(cx_px), int(cy_px)), 5, (0, 255, 0), 2)
        cv.drawContours(image, [cnt.astype(np.int32)], -1, (0, 128, 255), 1)
        for xi, yi in zip(x_center_sm, y_valid_center):
            cv.circle(image, (int(xi), int(yi)), 2, (0, 255, 255), -1)
        line_len = 50
        dx = (line_len/2) * math.cos(theta_line)
        dy = (line_len/2) * math.sin(theta_line)
        xc_plot = np.mean(x_center_sm)
        yc_plot = np.mean(y_valid_center)
        pt1 = (int(xc_plot - dx), int(yc_plot - dy))
        pt2 = (int(xc_plot + dx), int(yc_plot + dy))
        cv.line(image, pt1, pt2, (255, 0, 0), 2)

        return float(d_m), float(psi), float(gamma)


    # Function to normalize an angle
    def _wrap_pi(self, a):
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

