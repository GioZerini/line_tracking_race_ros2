#!/usr/bin/env python3
import math
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge

# Importa le TUE strategie originali per calcolare l’errore PID
from line_tracking_race_controller.centroid_strategy import CentroidStrategy
from line_tracking_race_controller.centerline_strategy import CenterlineStrategy
from line_tracking_race_controller.error_type import ErrorType


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner")

        # --- Parametri esistenti (come nel tuo planner originale) ---
        self.declare_parameter("error_type", "offset")
        self.declare_parameter("viz", False)
        self.declare_parameter("strategy", "centroid")

        # --- Parametri aggiuntivi per incroci & tuning (tutti sovrascrivibili da launch/YAML) ---
        # ROI: parte bassa dell’immagine, con margini laterali
        self.declare_parameter("roi_y_start_ratio", 0.50)  # usa la metà inferiore
        self.declare_parameter("roi_x_margin_px", 80)      # pixel da tagliare a sx/dx

        # Soglie per rilevare incrocio
        self.declare_parameter("T_area_hi", 0.22)  # soglia entrata (area gialla / area ROI)
        self.declare_parameter("T_area_lo", 0.12)  # soglia uscita (isteresi)
        self.declare_parameter("T_width_hi_ratio", 0.60)  # % della larghezza ROI

        # Debounce
        self.declare_parameter("N_enter", 3)  # frame consecutivi per entrare in ENTER_X
        self.declare_parameter("N_exit",  2)  # frame in ENTER_X prima di passare a TURNING

        # Scelta svolta
        self.declare_parameter("turn_choice", "straight")  # left|right|straight|alternate
        self.declare_parameter("turn_timeout", 1.0)    # [s] timeout sicurezza in TURNING

        # Velocità consigliate (il control node PID può usare /planner/speed_cmd)
        self.declare_parameter("v_follow", 0.35)
        self.declare_parameter("v_enter",  0.15)
        self.declare_parameter("v_turn",   0.12)

        # HSV giallo (tarabili)
        self.declare_parameter("h_lo", 15)   # ~ giallo scuro
        self.declare_parameter("s_lo", 80)
        self.declare_parameter("v_lo", 80)
        self.declare_parameter("h_hi", 40)   # ~ giallo chiaro
        self.declare_parameter("s_hi", 255)
        self.declare_parameter("v_hi", 255)

        # Morfologia
        self.declare_parameter("open_ksize", 3)
        self.declare_parameter("close_ksize", 5)
        self.declare_parameter("median_ksize", 5)


        self.declare_parameter("turn_error_sign", -1.0)  # -1.0 = inverti, +1.0 = non invertire
        self.turn_error_sign = float(self.get_parameter("turn_error_sign").value)

        # === leggi parametri ===
        error_type_arg = self.get_parameter("error_type").get_parameter_value().string_value
        viz = self.get_parameter("viz").get_parameter_value().bool_value
        strategy_name = self.get_parameter("strategy").get_parameter_value().string_value

        # ROI & soglie
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

        # HSV
        self.h_lo = int(self.get_parameter("h_lo").value)
        self.s_lo = int(self.get_parameter("s_lo").value)
        self.v_lo = int(self.get_parameter("v_lo").value)
        self.h_hi = int(self.get_parameter("h_hi").value)
        self.s_hi = int(self.get_parameter("s_hi").value)
        self.v_hi = int(self.get_parameter("v_hi").value)

        # Morfologia
        self.open_ksize   = int(self.get_parameter("open_ksize").value)
        self.close_ksize  = int(self.get_parameter("close_ksize").value)
        self.median_ksize = int(self.get_parameter("median_ksize").value)

        # --- Error type ---
        if error_type_arg == "offset":
            self.error_type = ErrorType.OFFSET
        elif error_type_arg == "angle":
            self.error_type = ErrorType.ANGLE
        else:
            self.get_logger().error(f"Unknown error type {error_type_arg}. Exiting.")
            rclpy.shutdown(); return

        # --- Strategia per l’errore PID (riuso delle tue classi originali) ---
        if strategy_name == "centroid":
            self.strategy = CentroidStrategy(self, self.error_type, viz)
        elif strategy_name == "centerline":
            self.strategy = CenterlineStrategy(self, self.error_type, viz)
        else:
            self.get_logger().error(f"Unknown strategy {strategy_name}. Exiting.")
            rclpy.shutdown(); return

        # --- ROS pub/sub ---
        self.bridge   = CvBridge()
        self.error_pub = self.create_publisher(Float32, "/planning/error", 10)
        self.speed_pub = self.create_publisher(Float32, "/planner/speed_cmd", 10)
        self.state_pub = self.create_publisher(String,   "/planner/state", 10)

        self.camera_sub = self.create_subscription(Image, "/camera/image_raw",
                                                   self.camera_callback, 10)

        # --- Stato FSM ---
        self.state = "FOLLOW"
        self.enter_cnt = 0
        self.exit_cnt  = 0
        self.turn_dir  = 0    # +1 left, -1 right, 0 straight
        self._last_dir = -1    # per "alternate"
        self.turn_timer = None

        from std_msgs.msg import Bool
        self.pid_reset_pub = self.create_publisher(Bool, "/planner/pid_reset", 10)

        self.get_logger().info("Planner (PID + incroci) inizializzato.")

    # -------------------------------------------------------------------------
    # CALLBACK CAMERA
    # -------------------------------------------------------------------------
    def camera_callback(self, msg: Image):
        # Converti in OpenCV
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        H, W = bgr.shape[:2]

        # ROI: parte bassa, bordi tagliati
        y0 = int(self.roi_y_start_ratio * H)
        xL = self.roi_x_margin_px
        xR = W - self.roi_x_margin_px
        if xR <= xL:
            xL, xR = 0, W  # fallback se margin troppo grande
        roi = bgr[y0:, xL:xR]

        # Maschera gialla binaria 0/1
        mask01 = self._yellow_mask(roi)  # implementata sotto

        # Metriche per rilevare incrocio
        area_ratio = float(mask01.mean())  # 0..1
        width_px   = self._width_estimate(mask01)  # implementata sotto
        T_width_hi = self.T_width_hi_ratio * mask01.shape[1]
        is_broad   = (area_ratio > self.T_area_hi) or (width_px > T_width_hi)

        # Errore PID “classico” (riusa la tua strategia originale)
        err = self._line_error(msg)  # implementata sotto (strategy.plan(msg))

        # --------------- FSM ---------------
        if self.state == "FOLLOW":
            if is_broad:
                self.enter_cnt += 1
            else:
                self.enter_cnt = 0

            if self.enter_cnt >= self.N_enter:
                self.state = "ENTER_X"
                self._publish_speed(self.v_enter)
                self._set_turn_dir()
                self.enter_cnt = 0
                self.get_logger().info("FSM: FOLLOW -> ENTER_X")
            else:
                self._publish_speed(self.v_follow)
                self._publish_error(err)

        elif self.state == "ENTER_X":
            # rallenta e procedi dritto un attimo per "entrare" nell'area
            self._publish_speed(self.v_enter)
            self._publish_error(0.0)  # neutro: niente sterzate brusche
            self.exit_cnt += 1
            if self.exit_cnt >= self.N_exit:
                self.state = "TURNING"
                self.exit_cnt = 0
                self.turn_timer = self.get_clock().now()
                self.get_logger().info("FSM: ENTER_X -> TURNING")
                self.pid_reset_pub.publish(Bool(data=True))

        elif self.state == "TURNING":
            # velocità ridotta mentre attraversi/svolti
            self._publish_speed(self.v_turn)

            if self.turn_dir == 0:
                # --- STRAIGHT: segui comunque la corsia, ma "soft" ---
                err_follow = self._line_error(msg)
                # deadband per evitare zig-zag col rumore
                if abs(err_follow) < 0.08:
                    err_turn = 0.0
                else:
                    err_turn = 0.6 * err_follow  # gain ridotto in incrocio
            else:
                # --- LEFT/RIGHT: imponi un errore angolare fittizio col giusto segno ---
                # turn_dir: +1 = left, -1 = right
                TURN_GAIN = 0.8
                # turn_error_sign: +1 mantiene il segno, -1 lo inverte (tarabile da param)
                err_turn = TURN_GAIN * self.turn_error_sign * (1.0 if self.turn_dir > 0 else -1.0)

            # pubblica l'errore verso il PID
            self._publish_error(err_turn)

            # --- condizioni di uscita ---
            is_narrow = (area_ratio < self.T_area_lo) and (width_px < 0.5 * mask01.shape[1])

            timed_out = False
            if self.turn_timer is not None:
                dt_turn = (self.get_clock().now() - self.turn_timer).nanoseconds / 1e9
                timed_out = (dt_turn > self.turn_timeout)

            # opzionale: uscita anticipata se "straight" e l'errore resta piccolo per M frame
            if self.turn_dir == 0:
                if abs(err_turn) < 0.12:
                    self.straight_ok_cnt = getattr(self, "straight_ok_cnt", 0) + 1
                else:
                    self.straight_ok_cnt = 0
                straight_ok = self.straight_ok_cnt >= getattr(self, "N_straight_ok", 3)
            else:
                self.straight_ok_cnt = 0
                straight_ok = False

            if is_narrow or timed_out or straight_ok:
                self.state = "FOLLOW"
                self.straight_ok_cnt = 0
                self.get_logger().info("FSM: TURNING -> FOLLOW")

        # Pubblica stato (debug)
        self.state_pub.publish(String(data=self.state))

    # -------------------------------------------------------------------------
    # == TODO risolti: funzioni di supporto ==
    # -------------------------------------------------------------------------
    def _yellow_mask(self, bgr_roi) -> np.ndarray:
        """
        Ritorna una maschera binaria 0/1 (dtype=uint8) dei pixel gialli nel ROI,
        con un po' di morfologia per pulire rumore.
        Soglie HSV tarabili via parametri.
        """
        hsv = cv.cvtColor(bgr_roi, cv.COLOR_BGR2HSV)

        lower = np.array([self.h_lo, self.s_lo, self.v_lo], dtype=np.uint8)
        upper = np.array([self.h_hi, self.s_hi, self.v_hi], dtype=np.uint8)
        mask = cv.inRange(hsv, lower, upper)  # 0/255

        # riduci rumore
        k_med = max(1, self.median_ksize | 1)  # dispari
        mask = cv.medianBlur(mask, k_med)

        if self.open_ksize > 0:
            k = cv.getStructuringElement(cv.MORPH_ELLIPSE, (self.open_ksize, self.open_ksize))
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, k)
        if self.close_ksize > 0:
            k = cv.getStructuringElement(cv.MORPH_ELLIPSE, (self.close_ksize, self.close_ksize))
            mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, k)

        # normalizza a 0/1
        mask01 = (mask > 0).astype(np.uint8)
        return mask01

    def _width_estimate(self, mask01: np.ndarray) -> float:
        """
        Stima la larghezza 'utile' della pista nel bordo inferiore del ROI.
        Prende N righe vicino al fondo, misura (ultimo - primo) pixel attivo e fa la media.
        """
        h, w = mask01.shape[:2]
        N = min(12, max(4, h // 20))           # prendi alcune righe (4..12) vicino al fondo
        rows = range(h - 1, h - 1 - N, -1)

        widths = []
        for r in rows:
            cols = np.flatnonzero(mask01[r, :])
            if cols.size >= 2:
                widths.append(float(cols[-1] - cols[0]))
        if len(widths) == 0:
            return 0.0
        return float(np.mean(widths))

    def _line_error(self, img_msg: Image) -> float:
        """
        Calcola l'errore 'classico' (offset o angolo normalizzato) RIUSANDO le tue strategie originali,
        così resta coerente con il tuo controller PID attuale.
        """
        try:
            err = self.strategy.plan(img_msg)   # esattamente come nel tuo planner originale
            return float(err) if err is not None else 0.0
        except Exception as e:
            self.get_logger().warn(f"Strategy error: {e}")
            return 0.0

    # -------------------------------------------------------------------------
    # Utility pubblicazione
    # -------------------------------------------------------------------------
    def _publish_error(self, e: float):
        # saturazione in [-1, 1] per sicurezza
        e = max(-1.0, min(1.0, float(e)))
        self.error_pub.publish(Float32(data=e))

    def _publish_speed(self, v: float):
        self.speed_pub.publish(Float32(data=float(v)))

    def _set_turn_dir(self):
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
            # default di sicurezza
            self.turn_dir = +1


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Planner node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()