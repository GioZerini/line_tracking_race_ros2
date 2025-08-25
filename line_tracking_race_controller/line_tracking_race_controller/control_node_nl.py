#!/usr/bin/env python3

import os
import math
from datetime import datetime
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

# ===================== Costanti controllo =====================
MAX_THRUST = 0.5       # [m/s] velocità lineare massima
RAMP_UP = 0.1          # [m/s^2] incremento per secondo se manca v_ref dal planner
WHEEL_BASE = 1.4       # [m] interasse
OMEGA_LIMIT = 0.8      # [rad/s] limite velocità angolare per sicurezza

def sinc(x: float) -> float:
    return 1.0 if abs(x) < 1e-6 else math.sin(x)/x


class ControlNodeNl(Node):
    def __init__(self):
        super().__init__("control_node")

        # ---------------- Parametri comuni ----------------
        self.declare_parameter("duration", -1)
        self.declare_parameter("controller_type", "pid")  # "pid" | "nlpf"

        # PID
        self.declare_parameter("k_p", 1.0)
        self.declare_parameter("k_i", 0.2)
        self.declare_parameter("k_d", 0.2)

        # Non lineare (README)
        self.declare_parameter("k_psi", 2.0)
        self.declare_parameter("eps_div", 1e-3)          # protezione (1 - d*gamma)
        self.declare_parameter("use_planner_speed", True) # usa /planner/speed_cmd se disponibile

        self.max_duration = float(self.get_parameter("duration").value)
        self.controller_type = self.get_parameter("controller_type").value

        self.k_p = float(self.get_parameter("k_p").value)
        self.k_i = float(self.get_parameter("k_i").value)
        self.k_d = float(self.get_parameter("k_d").value)

        self.k_psi = float(self.get_parameter("k_psi").value)
        self.eps_div = float(self.get_parameter("eps_div").value)
        self.use_planner_speed = bool(self.get_parameter("use_planner_speed").value)

        self.get_logger().info(
            f"Controller: {self.controller_type} | PID=({self.k_p},{self.k_i},{self.k_d}) | "
            f"NL(k_psi={self.k_psi})"
        )

        # ---------------- Log ----------------
        date = datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        self.pkg_path = get_package_share_directory("line_tracking_race_controller")
        self._open_logs(date)

        # ---------------- Stato ----------------
        self.started = False
        self.time_start = None
        self.time_prev = None
        self.ISE = 0.0

        # PID
        self.prev_error = 0.0
        self.accumulated_integral = 0.0
        self.int_limit = 2.0

        # Comando velocità
        self.v_ramp = 0.0         # rampa locale
        self.v_ref = None         # da /planner/speed_cmd

        # Feature NL frenet (da /vision/*)
        self.d = 0.0
        self.psi = 0.0
        self.gamma = 0.0
        self.have_d = False
        self.have_psi = False
        self.have_gamma = False

        # ---------------- Publisher ----------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------------- Subscribers ----------------
        # Trigger principale: resta /planning/error (compatibilità).
        # Se controller_type="nlpf", usiamo d/psi/gamma aggiornati in questo callback.
        self.error_sub = self.create_subscription(Float32, "/planning/error",
                                                  self.handle_error_callback, 10)
        # Non lineare: segnali dalla vision
        self.sub_d = self.create_subscription(Float32, "/vision/d", self.cb_d, 10)
        self.sub_psi = self.create_subscription(Float32, "/vision/psi", self.cb_psi, 10)
        self.sub_gamma = self.create_subscription(Float32, "/vision/gamma", self.cb_gamma, 10)
        # Velocità target dal planner (opzionale)
        self.sub_speed = self.create_subscription(Float32, "/planner/speed_cmd", self.cb_speed, 10)

        self.get_logger().info("Control node initialized!")

    
    def cb_d(self, msg: Float32):
        self.d = float(msg.data); self.have_d = True

    def cb_psi(self, msg: Float32):
        self.psi = float(msg.data); self.have_psi = True

    def cb_gamma(self, msg: Float32):
        self.gamma = float(msg.data); self.have_gamma = True

    def cb_speed(self, msg: Float32):
        self.v_ref = float(msg.data)

    # ==================== Callback principale ====================
    def handle_error_callback(self, msg: Float32):
        # timestamp
        time_now = self.get_clock().now()
        if not self.started:
            self.time_start = time_now
            self.time_prev = time_now
            self.started = True
            return

        elapsed = (time_now - self.time_start).nanoseconds / 1e9
        dt = (time_now - self.time_prev).nanoseconds / 1e9

        # durata max
        if self.max_duration >= 0 and elapsed > self.max_duration:
            self.get_logger().warn("Max duration reached.")
            self.stop()
            return

        if dt <= 0.0:
            return

        # ---------------- Selettore controllore ----------------
        if self.controller_type.lower() == "nlpf":
            # Se non ho ancora tutte le feature, procedo piano e dritto (idle crawl)
            if not (self.have_d and self.have_psi and self.have_gamma):
                # usa la rampa locale per far "agganciare" la linea
                self.v_ramp = min(MAX_THRUST, self.v_ramp + RAMP_UP * dt)
                v_idle = min(0.25, self.v_ramp)  # velocità molto bassa
                self._publish_twist(v_idle, 0.0)
                self.time_prev = time_now
                return

            # ---- costanti di sicurezza / tuning (locali) ----
            ALPHA       = 0.85   # filtro exp più deciso (su d, psi, gamma)
            D_MAX       = 0.30   # [m] offset plausibile
            PSI_MAX     = 0.90   # [rad]
            GAMMA_MAX   = 1.20   # [1/m] clamp curvatura
            DEN_EPS     = 0.12   # guardia su 1 - d*gamma
            PSI_DB      = 0.08   # [rad] dead-band su psi
            D_DB        = 0.02   # [m]   dead-band su d
            V_MAX_NL    = 0.28   # [m/s] cappello fisso
            A_Y_MAX     = 0.6    # [m/s^2] limite accelerazione laterale (rallenta in curva)
            k_dv_base   = 0.45   # guadagno su d*v*sinc
            k_g_base    = 0.00   # (riaccendilo più tardi: 0.20..0.35)

            # ---- filtro exp su d, psi, gamma ----
            if not hasattr(self, "_fd"):
                self._fd, self._fpsi, self._fg = self.d, self.psi, self.gamma
            self._fd   = (1.0 - ALPHA) * self._fd   + ALPHA * self.d
            self._fpsi = (1.0 - ALPHA) * self._fpsi + ALPHA * self.psi
            self._fg   = (1.0 - ALPHA) * self._fg   + ALPHA * self.gamma

            # ---- clamp ingressi ----
            d     = max(-D_MAX,     min(D_MAX,     self._fd))
            psi   = max(-PSI_MAX,   min(PSI_MAX,   self._fpsi))
            gamma = max(-GAMMA_MAX, min(GAMMA_MAX, self._fg))

            # ---- dead-band morbido: annulla micro-correzzioni che innescano zig-zag ----
            if abs(psi) < PSI_DB: psi = 0.0
            if abs(d)   < D_DB:   d   = 0.0

            # ---- velocità v (lenta, con cappello e rallentamento in curva) ----
            if self.use_planner_speed and self.v_ref is not None:
                v = float(self.v_ref)
            else:
                self.v_ramp = min(MAX_THRUST, self.v_ramp + RAMP_UP * dt)
                v = self.v_ramp
            # cappello di avvio
            if elapsed < 2.0:
                v = min(v, 0.22)
            # limite in funzione della curvatura: a_y = v^2 * |gamma| <= A_Y_MAX
            v_curve = (V_MAX_NL if abs(gamma) < 1e-6 else min(V_MAX_NL, (A_Y_MAX / abs(gamma)) ** 0.5))
            v = min(v, V_MAX_NL, v_curve)

            # ---- denominatore robusto ----
            den = 1.0 - d * gamma
            if abs(den) < DEN_EPS:
                den = DEN_EPS if den >= 0.0 else -DEN_EPS

            # ---- gain scheduling: quando |psi| è grande, attenua d e gamma ----
            psi_w = 1.0 / (1.0 + 2.0 * abs(psi))
            k_dv  = k_dv_base * psi_w
            k_g   = k_g_base  * psi_w

            # ---- legge non lineare ----
            omega_raw = (- self.k_psi * psi
                        - k_dv * d * v * sinc(psi)
                        + k_g  * v * (math.cos(psi) * gamma) / den)

            # ---- NaN guard ----
            if not math.isfinite(omega_raw):
                omega_raw = 0.0

            # ---- low-pass + rate limiter su omega ----
            BETA_OMEGA = 0.30      # low-pass del comando (0..1)
            OMEGA_SLEW = 1.0       # [rad/s^2] max variazione per secondo
            if not hasattr(self, "_omega_prev"):
                self._omega_prev = 0.0
            omega_lp = (1.0 - BETA_OMEGA) * self._omega_prev + BETA_OMEGA * omega_raw
            domega_max = OMEGA_SLEW * max(dt, 1e-3)
            omega = max(self._omega_prev - domega_max, min(self._omega_prev + domega_max, omega_lp))

            # ---- saturazione (mantieni eventuale inversione di verso se l’avevi messa) ----
            omega = max(-OMEGA_LIMIT, min(OMEGA_LIMIT, omega))
            omega = -omega  # SOLO se in sim il verso è invertito

            # ---- logging/ISE e publish ----
            error = float(d)
            self.ISE += dt * (error * error + self.prev_error * self.prev_error) / 2.0
            self.prev_error = error
            self.time_prev = time_now

            self._publish_twist(v, omega)
            self._omega_prev = omega
            self._log_data(elapsed, dt, error, omega, v, 0.0, 0.0, 0.0, 0.0)
            return
    
        # ---------------- PID (comportamento esistente) ----------------
        measurement = float(msg.data)
        error = measurement

        # ISE
        self.ISE += dt * (error*error + self.prev_error*self.prev_error) / 2.0

        # PID
        self.accumulated_integral += dt * (error + self.prev_error) / 2.0
        # anti-windup
        self.accumulated_integral = max(-self.int_limit, min(self.int_limit, self.accumulated_integral))

        p_term = self.k_p * error
        i_term = self.k_i * self.accumulated_integral
        d_term = self.k_d * (error - self.prev_error) / dt
        control = p_term + i_term + d_term

        self.prev_error = error
        self.time_prev = time_now

        # v (rampa) + mappa 'control' su omega
        v = self.v_ref if (self.use_planner_speed and self.v_ref is not None) else None
        if v is None:
            self.v_ramp = min(MAX_THRUST, self.v_ramp + RAMP_UP * dt)
            v = self.v_ramp

        omega = max(-OMEGA_LIMIT, min(OMEGA_LIMIT, control))

        self._publish_twist(v, omega)
        self._log_data(elapsed, dt, error, omega, v, 0.0, p_term, i_term, d_term)

    # ==================== Pubblicazione Twist ====================
    def _publish_twist(self, v: float, omega: float):
        # Pubblica cmd_vel direttamente (cinematica differenziale gestita a valle)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.cmd_vel_pub.publish(msg)

    # ==================== Log & Stop ====================
    def _open_logs(self, date: str):
        logs_dir = os.path.join(self.pkg_path, "logs")
        eval_dir = os.path.join(self.pkg_path, "logs", "evaluations")
        os.makedirs(logs_dir, exist_ok=True)
        os.makedirs(eval_dir, exist_ok=True)

        tag = self.controller_type
        filepath = os.path.join(logs_dir, f"log_{date}_[{tag}].csv")
        self.logfile = open(filepath, "w+", newline="")
        self.log_writer = csv.writer(self.logfile)
        self.log_writer.writerow(["Time", "dt", "Error", "Omega", "V", "Dummy", "P", "I", "D"])

        evalpath = os.path.join(eval_dir, f"evaluation_{date}_[{tag}].csv")
        self.evaluation_file = open(evalpath, "w+", newline="")
        self.performance_index_writer = csv.writer(self.evaluation_file)
        self.performance_index_writer.writerow(["ISE"])

    def _log_data(self, elapsed, dt, error, omega, v, dummy, p_term, i_term, d_term):
        self.log_writer.writerow([elapsed, dt, error, omega, v, dummy, p_term, i_term, d_term])

    def _log_performance_indices(self):
        self.performance_index_writer.writerow([self.ISE])

    def stop(self):
        # ferma il robot e chiudi i log
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)

        self._log_performance_indices()
        self.logfile.close()
        self.evaluation_file.close()
        self.get_logger().info("Control node shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNodeNl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()


if __name__ == "__main__":
    main()
