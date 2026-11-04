#!/usr/bin/env python3
"""Orientacja z IMU + regulator do wektora zadanego (kurs / poziom / glebokosc). Siedzi miedzy GUI a mikserem.

Wejscia:  /rov/imu (sensor_msgs/Imu, surowe zyro rad/s i akcel m/s2 w osiach CZUJNIKA)
          /rov/depth (Float32, m, + w dol)
          /rov/cmd_vel_pilot (Twist, -1..1 od pilota: GUI/pad)      <- GUI publikuje TU
          /rov/ctrl/mode String JSON {"yaw_hold": bool, "level": bool, "depth_hold": bool}
          /rov/ctrl/setpoint String JSON {"yaw_deg": .., "roll_deg": .., "pitch_deg": .., "depth_m": ..}  (dowolny podzbior)
          /rov/ctrl/gains String JSON {"yaw": [kp,ki,kd], "roll": [...], "pitch": [...], "depth": [...]}
Serwis:   /rov/imu/zero (Trigger) - aktualna orientacja = poziom (roll=pitch=0) i kurs 0
Wyjscia:  /rov/cmd_vel (Twist -> mikser) = pilot + korekty regulatora (tylko gdy pilot nadaje; deadman miksera zostaje)
          /rov/attitude (Vector3: roll, pitch, yaw [deg] wzgledem zera), /rov/ctrl/status (String JSON, 5 Hz)
          /rov/debug/ctrl (String JSON co tick, takze gdy pilot milczy - do analizy z bagow): pilot[6], out[6], rpy, gyro_dps,
             acc, depth, mode, sp, imu_age, susp, pid: {os: [e, p, i, d, u, sat]} (tylko osie liczone w tym ticku)

Osie: cialo FLU (x przod, y lewo, z gora). Mapowanie osi czujnika -> cialo parametrem imu_axes, np. "x,z,-y"
(cialo_x = sensor_x, cialo_y = sensor_z, cialo_z = -sensor_y). Semantyka komend jak w mikserze:
roll + = prawa burta w dol, pitch + = dziob w gore, yaw + = w lewo, heave + = do gory.
Filtr: komplementarny (Mahony-lite): kat z calkowania zyra, roll/pitch korygowane akcelerometrem, yaw tylko zyro (dryf -> zerowanie).
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


def axis_map(spec):
    """'x,z,-y' -> macierz 3x3 R taka, ze body = R @ sensor."""
    idx = {"x": 0, "y": 1, "z": 2}
    R = np.zeros((3, 3))
    for i, tok in enumerate(spec.replace(" ", "").split(",")):
        s = -1.0 if tok.startswith("-") else 1.0
        R[i, idx[tok.lstrip("+-")]] = s
    if abs(np.linalg.det(R) - 1.0) > 1e-6:
        raise ValueError(f"imu_axes '{spec}' nie jest prawoskretnym mapowaniem (det={np.linalg.det(R):.1f})")
    return R


class PID:
    """PID z anty-windupem: (1) calka ograniczona do +-i_lim, (2) calka NIE rosnie, gdy wyjscie jest nasycone
    w te sama strone co blad (warunkowe calkowanie), (3) reset przy wlaczaniu/zawieszaniu trybu."""

    def __init__(self, kp, ki, kd, out_lim=1.0, i_lim=0.3):
        self.kp, self.ki, self.kd, self.out_lim, self.i_lim = kp, ki, kd, out_lim, i_lim
        self.i = 0.0; self.prev_e = None; self.saturated = False
        self.last = None              # (e, p, i, d, u) z ostatniego step() - do /rov/debug/ctrl

    def reset(self):
        self.i = 0.0; self.prev_e = None; self.saturated = False

    def step(self, e, dt, rate=None):
        """rate = pochodna mierzona (np. predkosc katowa) - lepsza niz roznica bledu."""
        d = -rate if rate is not None else ((e - self.prev_e) / dt if self.prev_e is not None and dt > 0 else 0.0)
        self.prev_e = e
        u_unsat = self.kp * e + self.i + self.kd * d
        # calkuj tylko gdy wyjscie nie jest nasycone, albo gdy blad "odkreca" nasycenie
        if not (abs(u_unsat) >= self.out_lim and (u_unsat > 0) == (e > 0)):
            self.i = max(-self.i_lim, min(self.i_lim, self.i + self.ki * e * dt))
        u = self.kp * e + self.i + self.kd * d
        self.saturated = abs(u) >= self.out_lim
        u = max(-self.out_lim, min(self.out_lim, u))
        self.last = (e, self.kp * e, self.i, self.kd * d, u)
        return u


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class AttitudeController(Node):
    def __init__(self):
        super().__init__("attitude_controller")
        p = self.declare_parameter
        p("imu_axes", "x,z,-y"); p("acc_weight", 0.02); p("rate_hz", 20.0); p("pilot_timeout_s", 0.5)
        p("gains_yaw", [0.02, 0.002, 0.004]); p("gains_roll", [0.02, 0.002, 0.004]); p("gains_pitch", [0.02, 0.002, 0.004])
        p("gains_depth", [1.5, 0.2, 0.5])          # [1/m, 1/(m s), s/m]
        p("deadband", 0.08)                        # gdy |pilot| > deadband na osi -> regulator tej osi odpuszcza, setpoint = aktualny
        p("i_limit", 0.3)                          # maks. wklad calki w wyjscie (-1..1) - blokada windup
        p("out_limit", 0.6)                        # maks. korekta regulatora na os (reszta zostaje dla pilota)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.R = axis_map(g("imu_axes")); self.acc_w = float(g("acc_weight")); self.deadband = float(g("deadband"))
        self.pilot_timeout = float(g("pilot_timeout_s"))
        il, ol = float(g("i_limit")), float(g("out_limit"))
        self.pid = {k: PID(*g(f"gains_{k}"), out_lim=ol, i_lim=il) for k in ("yaw", "roll", "pitch", "depth")}

        # stan orientacji (kwaternion body->world, w x y z), zero = kwaternion odniesienia
        self.q = np.array([1.0, 0.0, 0.0, 0.0]); self.q_ref = np.array([1.0, 0.0, 0.0, 0.0]); self.yaw_ref = 0.0
        self.gyro_b = np.zeros(3); self.acc_b = np.zeros(3); self.last_imu = None; self.imu_t = 0.0; self.n_imu = 0
        self.rpy = [0.0, 0.0, 0.0]        # wzgledem zera [deg]
        self.depth = None; self.depth_t = 0.0
        self.mode = {"yaw_hold": False, "level": False, "depth_hold": False}
        self.sp = {"yaw_deg": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0, "depth_m": 0.0}
        self.pilot = [0.0] * 6; self.pilot_t = 0.0
        self.corr = {"yaw": 0.0, "roll": 0.0, "pitch": 0.0, "heave": 0.0}
        self.suspended = {"yaw": False, "level": False, "depth": False}

        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Imu, "/rov/imu", self.on_imu, be)
        self.create_subscription(Float32, "/rov/depth", self.on_depth, be)
        self.create_subscription(Twist, "/rov/cmd_vel_pilot", self.on_pilot, 10)
        self.create_subscription(String, "/rov/ctrl/mode", self.on_mode, 10)
        self.create_subscription(String, "/rov/ctrl/setpoint", self.on_setpoint, 10)
        self.create_subscription(String, "/rov/ctrl/gains", self.on_gains, 10)
        self.create_service(Trigger, "/rov/imu/zero", self.srv_zero)
        self.pub_cmd = self.create_publisher(Twist, "/rov/cmd_vel", 10)
        self.pub_att = self.create_publisher(Vector3, "/rov/attitude", 10)
        self.pub_st = self.create_publisher(String, "/rov/ctrl/status", 10)
        self.pub_dbg = self.create_publisher(String, "/rov/debug/ctrl", 10)
        self.create_timer(1.0 / float(g("rate_hz")), self.tick)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f"imu_axes={g('imu_axes')}  regulator: {list(self.pid)}")

    # ---- IMU / filtr ----
    def on_imu(self, m: Imu):
        now = time.monotonic()
        gs = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        as_ = np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        self.gyro_b = self.R @ gs; self.acc_b = self.R @ as_
        dt = min(max(now - self.imu_t, 1e-3), 0.1) if self.imu_t else 0.0
        self.imu_t = now; self.n_imu += 1
        if dt <= 0:
            return
        q = self.q
        # korekta akcelerometrem (grawitacja powinna wypadac na +z ciala w spoczynku)
        n = np.linalg.norm(self.acc_b)
        g = self.gyro_b.copy()
        if 0.5 * 9.81 < n < 1.5 * 9.81:
            a = self.acc_b / n
            # kierunek grawitacji wg kwaternionu: v = R(q)^T * [0,0,1]
            w, x, y, z = q
            v = np.array([2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z])
            e = np.cross(a, v)                # blad (a x v)
            g = g + self.acc_w / max(dt, 1e-3) * 0.5 * e   # proporcjonalna korekta (Mahony kp ~ acc_w/dt)
        # calkowanie kwaternionu
        half = 0.5 * dt * g
        dq = np.array([1.0, half[0], half[1], half[2]])
        q = quat_mul(q, dq); self.q = q / np.linalg.norm(q)
        self.rpy = self.relative_rpy()
        self.pub_att.publish(Vector3(x=self.rpy[0], y=self.rpy[1], z=self.rpy[2]))

    def relative_rpy(self):
        """roll/pitch/yaw [deg] orientacji wzgledem zera (q_rel = q_ref^-1 * q)."""
        qr = quat_mul(quat_conj(self.q_ref), self.q)
        w, x, y, z = qr
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        # konwencja komend: roll + = prawa burta w dol (obrot ujemny wokol x FLU), pitch + = dziob w gore (obrot ujemny wokol y FLU), yaw + = w lewo (+z)
        return [-roll, -pitch, yaw]

    def srv_zero(self, req, res):
        if not self.n_imu:
            res.success, res.message = False, "brak danych IMU"; return res
        self.q_ref = self.q.copy(); self.zeroed = True
        self.sp["yaw_deg"] = 0.0; self.sp["roll_deg"] = 0.0; self.sp["pitch_deg"] = 0.0
        for k in ("yaw", "roll", "pitch"):
            self.pid[k].reset()
        res.success, res.message = True, "IMU wyzerowane: aktualna orientacja = poziom, kurs 0"
        self.get_logger().info(res.message)
        return res

    def on_depth(self, m: Float32):
        self.depth = float(m.data); self.depth_t = time.monotonic()

    # ---- wejscia sterowania ----
    def on_pilot(self, m: Twist):
        self.pilot = [m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z]
        self.pilot_t = time.monotonic()

    def on_mode(self, m: String):
        try:
            d = json.loads(m.data)
            for k in self.mode:
                if k in d:
                    new = bool(d[k])
                    if new and not self.mode[k]:          # wlaczenie: setpoint = stan biezacy (bumpless)
                        if k == "yaw_hold":
                            self.sp["yaw_deg"] = self.rpy[2]; self.pid["yaw"].reset()
                        if k == "depth_hold" and self.depth is not None:
                            self.sp["depth_m"] = self.depth; self.pid["depth"].reset()
                        if k == "level":
                            self.pid["roll"].reset(); self.pid["pitch"].reset()
                    if not new and self.mode[k]:          # wylaczenie: wyczysc calki, zeby nie wrocily przy nastepnym wlaczeniu
                        for a in {"yaw_hold": ["yaw"], "level": ["roll", "pitch"], "depth_hold": ["depth"]}[k]:
                            self.pid[a].reset(); self.corr[a if a != "depth" else "heave"] = 0.0
                    self.mode[k] = new
            self.get_logger().info(f"tryb: {self.mode}")
        except Exception as e:
            self.get_logger().error(f"mode: {e}")

    def on_setpoint(self, m: String):
        try:
            d = json.loads(m.data)
            for k in self.sp:
                if k in d:
                    self.sp[k] = float(d[k])
            self.sp["yaw_deg"] = wrap(self.sp["yaw_deg"])
        except Exception as e:
            self.get_logger().error(f"setpoint: {e}")

    def on_gains(self, m: String):
        try:
            for k, v in json.loads(m.data).items():
                if k in self.pid and len(v) == 3:
                    self.pid[k].kp, self.pid[k].ki, self.pid[k].kd = map(float, v); self.pid[k].reset()
            self.get_logger().info("nowe nastawy: " + ", ".join(f"{k}={[p.kp, p.ki, p.kd]}" for k, p in self.pid.items()))
        except Exception as e:
            self.get_logger().error(f"gains: {e}")

    # ---- petla regulacji ----
    def tick(self):
        now = time.monotonic()
        for p in self.pid.values():
            p.last = None
        if not self.pilot_t or now - self.pilot_t > self.pilot_timeout:
            self.publish_debug(now, None)
            return                                 # pilot milczy -> nic nie publikujemy -> mikser zeruje (deadman)
        dt = 1.0 / 20.0
        out = list(self.pilot)
        imu_ok = self.imu_t and now - self.imu_t < 0.5
        gx, gy, gz = (math.degrees(v) for v in self.gyro_b)
        # yaw
        c = 0.0
        if self.mode["yaw_hold"] and imu_ok:
            if abs(self.pilot[5]) > self.deadband:
                self.suspended["yaw"] = True; self.pid["yaw"].reset()
            else:
                if self.suspended["yaw"]:
                    self.suspended["yaw"] = False; self.sp["yaw_deg"] = self.rpy[2]; self.pid["yaw"].reset()
                c = self.pid["yaw"].step(wrap(self.sp["yaw_deg"] - self.rpy[2]), dt, rate=gz)
        self.corr["yaw"] = c; out[5] = clamp(out[5] + c)
        # poziom (roll/pitch)
        cr = cp = 0.0
        if self.mode["level"] and imu_ok:
            if abs(self.pilot[3]) > self.deadband or abs(self.pilot[4]) > self.deadband:
                self.suspended["level"] = True; self.pid["roll"].reset(); self.pid["pitch"].reset()
            else:
                if self.suspended["level"]:
                    self.suspended["level"] = False; self.pid["roll"].reset(); self.pid["pitch"].reset()
                cr = self.pid["roll"].step(self.sp["roll_deg"] - self.rpy[0], dt, rate=-gx)
                cp = self.pid["pitch"].step(self.sp["pitch_deg"] - self.rpy[1], dt, rate=-gy)
        self.corr["roll"], self.corr["pitch"] = cr, cp
        out[3] = clamp(out[3] + cr); out[4] = clamp(out[4] + cp)
        # glebokosc (heave + = do gory, glebokosc + = w dol)
        ch = 0.0
        depth_ok = self.depth is not None and now - self.depth_t < 1.0
        if self.mode["depth_hold"] and depth_ok:
            if abs(self.pilot[2]) > self.deadband:
                self.suspended["depth"] = True; self.pid["depth"].reset()
            else:
                if self.suspended["depth"]:
                    self.suspended["depth"] = False; self.sp["depth_m"] = self.depth; self.pid["depth"].reset()
                ch = self.pid["depth"].step(self.depth - self.sp["depth_m"], dt)   # za gleboko (e>0) -> heave +
        self.corr["heave"] = ch; out[2] = clamp(out[2] + ch)
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = out[0], out[1], out[2]
        t.angular.x, t.angular.y, t.angular.z = out[3], out[4], out[5]
        self.pub_cmd.publish(t)
        self.publish_debug(now, out)

    def publish_debug(self, now, out):
        r = lambda v, n=4: round(float(v), n)  # noqa: E731
        d = {"active": out is not None, "pilot": [r(v) for v in self.pilot], "out": [r(v) for v in out] if out else None,
             "rpy": [r(v, 3) for v in self.rpy], "gyro_dps": [r(math.degrees(v), 3) for v in self.gyro_b],
             "acc": [r(v) for v in self.acc_b], "depth": self.depth, "mode": self.mode, "sp": self.sp,
             "imu_age": r(now - self.imu_t, 3) if self.imu_t else None, "susp": self.suspended,
             "pid": {k: [r(x, 5) for x in p.last] + [p.saturated] for k, p in self.pid.items() if p.last}}
        self.pub_dbg.publish(String(data=json.dumps(d, separators=(",", ":"))))

    def publish_status(self):
        now = time.monotonic()
        st = {"rpy": [round(v, 1) for v in self.rpy], "gyro_dps": [round(math.degrees(v), 1) for v in self.gyro_b],
              "acc": [round(float(v), 2) for v in self.acc_b], "imu_ok": bool(self.imu_t and now - self.imu_t < 0.5),
              "depth": self.depth, "mode": self.mode, "setpoint": self.sp, "corr": {k: round(v, 3) for k, v in self.corr.items()},
              "suspended": self.suspended, "pilot_active": bool(self.pilot_t and now - self.pilot_t < self.pilot_timeout),
              "gains": {k: [p.kp, p.ki, p.kd] for k, p in self.pid.items()},
              "integral": {k: round(p.i, 3) for k, p in self.pid.items()}, "saturated": [k for k, p in self.pid.items() if p.saturated],
              "zeroed": getattr(self, "zeroed", False)}
        self.pub_st.publish(String(data=json.dumps(st)))


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, float(v)))


def main():
    rclpy.init()
    n = AttitudeController()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
