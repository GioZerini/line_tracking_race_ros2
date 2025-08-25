#!/usr/bin/env python3
import math
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

from line_tracking_race_controller.error_type import ErrorType
from line_tracking_race_controller.visualizer import Visualizer
import line_tracking_race_controller.colors as colors

LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)

# Determine the (d, psi, gamma) values for the non-linear strategy
class NonLinearStrategy:
    """
    Estende la pipeline della CenterlineStrategy per stimare:
      d   [m]  = offset laterale con segno (convenzione: >0 a destra del crosshair; inverti se vuoi)
      psi [rad]= errore d'orientamento (tangente linea vs asse robot)
      gamma [1/m] = curvatura locale della centerline
    Restituisce un dict: {"err": err_classico, "d": d, "psi": psi, "gamma": gamma}
    """
    def __init__(self, node, error_type, should_visualize):
        self.error_type = error_type      # usato solo per "err" di compatibilità
        self.node = node
        self.cv_bridge = CvBridge()
        self.prev_waypoint = (0, 0)
        self.prev_offset_px = 0

        self.viz = Visualizer() if should_visualize else None

        self.last_valid = None

        # ---- scala px↔m (tarare!) ----
        self.px_per_m_x = float(node.declare_parameter("px_per_m_x", 800.0).value)
        self.px_per_m_y = float(node.declare_parameter("px_per_m_y", 800.0).value)

    # ========== API principale ==========
    def plan(self, img_msg):
        image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        H, W, _ = image.shape

        # 1) Isola la linea gialla dal resto dell'immagine
        track_outline = self._get_track_outline(image)

        # 2) Taglia l'immagine per concentrarsi sulla parte inferiore (detta ROI) (vicina al robot) ed elaborare meno dati
        cropped = track_outline[int(H/2):(H-10), 100:(W-100)]
        cr_h, cr_w = cropped.shape

        # 3) Calcola limite dx, limite sx e centerline della linea gialla
        left_limit, right_limit = self._extract_track_limits(cropped)
        centerline = self._compute_centerline(left_limit, right_limit)

        # Calcola il punto di riferimento del ROI
        crosshair = (cr_w // 2, cr_h // 2)
        position  = (cr_w // 2, cr_h - 1)

        # se non ho limiti affidabili, riuso il punto di riferimento precedente
        if left_limit.size == 0 or right_limit.size == 0 or centerline.size == 0:
            self.node.get_logger().warn("NonLinearStrategy: centerline non disponibile, uso waypoint precedente.")
            waypoint = self.prev_waypoint
            offset_px = self.prev_offset_px
        else:
            waypoint, offset_px = self._get_next_waypoint(centerline, crosshair)
            self.prev_waypoint = waypoint
            self.prev_offset_px = offset_px

        # Calcolo dell'errore classico
        # POTREBBE NON SERVIRE
        if self.error_type == ErrorType.OFFSET:
            err_classic, _ = self._compute_offset_error(waypoint, crosshair, cr_w/2)
        elif self.error_type == ErrorType.ANGLE:
            err_classic, _ = self._compute_angle_error(waypoint, position)
        else:
            self.node.get_logger().error("Unknown error type.")
            return None
        
        if centerline.size == 0 or left_limit.size == 0 or right_limit.size == 0:
            self.node.get_logger().warn("Centerline non disponibile.")
            if self.last_valid is None:
                return None  # planner manderà heartbeat=0.0
            return {"err": err_classic, **self.last_valid}

        # -------- stima d, psi, gamma per il controller NL --------
        # Fit polinomiale y(x) sulla centerline in coordinate CROPPED (px)
        d_m, psi, gamma_m = self._estimate_d_psi_gamma(centerline, crosshair)

        # Visual debug (opzionale)
        if self.viz is not None:
            self.viz.build_track_bg(cr_h, cr_w, left_limit, right_limit, centerline)
            # overlay offset/angolo “classici”
            if self.error_type == ErrorType.OFFSET:
                self.viz.build_offset_error_overlay(crosshair, waypoint)
            else:
                _, angle_deg = self._compute_angle_error(waypoint, position)
                self.viz.build_angle_error_overlay(crosshair, waypoint, position, angle_deg)
            self.viz.show()

        if not hasattr(self, "_gamma_lp"):
            self._gamma_lp = float(gamma_m)
        BETA_G = 0.30
        self._gamma_lp = (1.0 - BETA_G) * self._gamma_lp + BETA_G * float(gamma_m)

        G_MAX = 1.5  # [1/m] curvatura massima da pubblicare
        gamma_m = max(-G_MAX, min(G_MAX, self._gamma_lp))

        if abs(d_m) > 0.4:
            self.node.get_logger().warn(f"d outlier {d_m:.2f} m -> scarto frame")
            # non aggiornare last_valid: riusa il vecchio se c'è
            if self.last_valid is not None:
                return {"err": err_classic, **self.last_valid}
            else:
                return {"err": err_classic}  # planner farà heartbeat e il controller userà l'idle-crawl

        self.last_valid = {"d": d_m, "psi": psi, "gamma": gamma_m}
        return {"err": err_classic, "d": d_m, "psi": psi, "gamma": gamma_m}


    def _get_track_outline(self, bgr):
        hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array(LOWER_YELLOW), np.array(UPPER_YELLOW))
        track_outline = np.zeros(bgr.shape[:2], dtype=np.uint8)
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if contours:
            cv.drawContours(track_outline, contours, 0, colors.MAGENTA)
        return track_outline

    def _extract_track_limits(self, track_outline):
        _, labels = cv.connectedComponents(track_outline)
        left_cols,  left_rows  = np.where(labels == 1)
        right_cols, right_rows = np.where(labels == 2)
        left  = np.column_stack((left_rows,  left_cols ))[::10]
        right = np.column_stack((right_rows, right_cols))[::10]
        return left, right

    def _compute_centerline(self, left, right):
        center = []
        for (x1,y1), (x2,y2) in zip(left, right):
            center.append(( (x1+x2)//2, (y1+y2)//2 ))
        return np.array(center)

    def _get_next_waypoint(self, trajectory, crosshair):
        if trajectory.size == 0:
            return crosshair, 0
        cx, cy = crosshair
        best_i = 0; best_d = float("inf")
        for i, (x,y) in enumerate(trajectory):
            if y > cy - 30:   # stessa euristica della CenterlineStrategy
                continue
            d = (x-cx)**2 + (y-cy)**2
            if d < best_d:
                best_d = d; best_i = i
        wp = tuple(trajectory[best_i])
        return wp, (wp[0] - cx)

    def _compute_offset_error(self, waypoint, crosshair, max_off):
        off = waypoint[0] - crosshair[0]
        return (off + max_off)/max_off - 1.0, off

    def _compute_angle_error(self, waypoint, position):
        dx = waypoint[0] - position[0]
        dy = waypoint[1] - position[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            angle = 0.0
        else:
            angle = math.asin(dx / dist)
        angle_deg = angle * 180.0 / math.pi
        return (angle_deg + 90)/90 - 1.0, angle_deg


    def _estimate_d_psi_gamma(self, centerline_px, crosshair_px):
        """
        Stima (d [m], psi [rad], gamma [1/m]) in modo robusto anche con centerline quasi verticale.
        centerline_px: np.array Nx2 in pixel (x,y) nel ROI
        crosshair_px : (cx,cy) in pixel nel ROI
        """
        import math
        import numpy as np

        # 1) serve almeno qualche punto
        if centerline_px.shape[0] < 5:
            # fallback: usa waypoint precedente, niente curvatura
            cx, cy = crosshair_px
            wx, wy = self.prev_waypoint
            # direzione (crosshair->wp)
            th = math.atan2((wy - cy), (wx - cx))
            psi = self._wrap_pi(th - self._robot_axis_angle())
            # d in metri (con segno dal lato x)
            d_m = self._signed_offset_m((cx, cy), (wx, wy))
            return d_m, psi, 0.0

        # 2) punto della centerline più vicino al crosshair (euclideo)
        cx, cy = crosshair_px
        diffs = centerline_px - np.array([[cx, cy]], dtype=np.float32)
        d2 = (diffs[:, 0]**2 + diffs[:, 1]**2)
        i0 = int(np.argmin(d2))

        # finestra locale [i0-2 .. i0+2] clampata nei bordi
        i1 = max(0, i0 - 2)
        i2 = min(centerline_px.shape[0] - 1, i0 + 2)
        window = centerline_px[i1:i2+1, :]
        if window.shape[0] < 3:
            # allarga se necessario
            i1 = max(0, i0 - 3)
            i2 = min(centerline_px.shape[0] - 1, i0 + 3)
            window = centerline_px[i1:i2+1, :]
            if window.shape[0] < 3:
                # ultima spiaggia: usa segmento locale per d e psi, gamma=0
                # vettore tangente approssimato
                j1 = max(0, i0 - 1); j2 = min(centerline_px.shape[0] - 1, i0 + 1)
                p1 = centerline_px[j1]; p2 = centerline_px[j2]
                tx, ty = (p2[0] - p1[0]), (p2[1] - p1[1])
                th_tan = math.atan2(ty, tx)
                psi = self._wrap_pi(th_tan - self._robot_axis_angle())
                d_m = self._signed_offset_m((cx, cy), (centerline_px[i0,0], centerline_px[i0,1]))
                return d_m, psi, 0.0

        # 3) derivate discrete (parametro = indice -> poi normalizziamo con arc-length)
        # punti in pixel
        xs = window[:, 0].astype(np.float32)
        ys = window[:, 1].astype(np.float32)

        # differenze centrali (x', y') e (x'', y'')
        # per semplicità lavoriamo sul punto centrale della finestra
        mid = window.shape[0] // 2
        if mid == 0 or mid == window.shape[0] - 1:
            # se la finestra è 3, mid=1 ok; se 2, gestito prima
            mid = min(1, window.shape[0] - 2)

        # derivate numeriche su indice i
        x_im1, x_i, x_ip1 = xs[mid-1], xs[mid], xs[mid+1]
        y_im1, y_i, y_ip1 = ys[mid-1], ys[mid], ys[mid+1]

        x1 = 0.5 * (x_ip1 - x_im1)     # ~ dx/di
        y1 = 0.5 * (y_ip1 - y_im1)     # ~ dy/di
        x2 = (x_ip1 - 2*x_i + x_im1)   # ~ d2x/di2
        y2 = (y_ip1 - 2*y_i + y_im1)   # ~ d2y/di2

        # normalizza rispetto alla lunghezza d'arco (per usare la formula della curvatura)
        # ds ≈ sqrt(x1^2 + y1^2) in pixel per passo
        ds = math.hypot(x1, y1)
        if ds < 1e-6:
            # segmento quasi puntuale: niente curvatura affidabile
            # usa tangente dal segmento più lungo in finestra
            p1 = window[0]; p2 = window[-1]
            tx, ty = (p2[0] - p1[0]), (p2[1] - p1[1])
            th_tan = math.atan2(ty, tx)
            psi = self._wrap_pi(th_tan - self._robot_axis_angle())
            d_m = self._signed_offset_m((cx, cy), (x_i, y_i))
            return d_m, psi, 0.0

        # derivate rispetto a s (lunghezza d'arco, in pixel)
        x_s  = x1 / ds
        y_s  = y1 / ds
        x_ss = x2 / (ds**2)
        y_ss = y2 / (ds**2)

        # 4) angolo della tangente (immagine: x orizzontale, y verso il basso)
        th_tan = math.atan2(y_s, x_s)    # [-pi, pi]
        psi = self._wrap_pi(th_tan - self._robot_axis_angle())

        # 5) curvatura in pixel^-1: kappa_px = |x' y'' - y' x''| / (x'^2 + y'^2)^(3/2)
        num = abs(x_s * y_ss - y_s * x_ss)
        den = (x_s**2 + y_s**2)**1.5
        kappa_px = num / den if den > 1e-9 else 0.0

        # conversione curvatura a 1/m (scala anisotropa px->m)
        # approssimazione: usa media delle scale per l'arco (meglio: omografia reale)
        # qui ds era in pixel; 1/px -> 1/m moltiplicando per (px_per_m) medio
        px_per_m_s = 0.5 * (self.px_per_m_x + self.px_per_m_y)
        gamma_m = kappa_px * px_per_m_s

        # 6) distanza laterale con segno dal crosshair alla retta tangente locale
        # retta tangente passante per (x_i, y_i) con versore t = (x_s, y_s)
        # distanza signed = cross( (P - Pi), t ) con convenzione immagine
        vx = (cx - x_i)
        vy = (cy - y_i)
        # prodotto vettoriale 2D (z): vx*ty - vy*tx
        cross_z = vx * y_s - vy * x_s
        # sign: positivo se la line è "a destra" del crosshair rispetto alla tangente
        # distanza in pixel = proiezione perpendicolare (dato t unitario)
        d_px_signed = cross_z  # perché |t| ~1 dopo normalizzazione su s
        # converti in metri: attenzione a scale diverse su x/y -> usa media per la norma
        d_m = (d_px_signed / px_per_m_s)

        return float(d_m), float(psi), float(gamma_m)

    # ---- utilità geometriche ----
    def _robot_axis_angle(self):
        # Asse robot = verticale verso il basso nell'immagine
        return math.pi/2

    def _wrap_pi(self, a):
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _signed_offset_m(self, p_crosshair, p_proj):
        # distanza con segno: + se la linea è a destra del crosshair (inverti segno se vuoi convenzione opposta)
        dx_px = (p_proj[0] - p_crosshair[0])
        dy_px = (p_proj[1] - p_crosshair[1])
        dx_m = dx_px / self.px_per_m_x
        dy_m = dy_px / self.px_per_m_y
        d = math.hypot(dx_m, dy_m)
        return math.copysign(d, dx_m)