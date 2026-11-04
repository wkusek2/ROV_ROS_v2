#!/usr/bin/env python3
"""Mikser pednikow ROV z alokacja sil (macierz B 6x8, pseudoodwrotnosc, saturacja) - patrz allocation.py.

Wejscia:  /rov/cmd_vel  geometry_msgs/Twist, zakres -1..1, semantyka jak w GUI:
             linear.x = surge (+ przod), linear.y = sway (+ w LEWO), linear.z = heave (+ do GORY),
             angular.x = roll (+ prawa burta w dol), angular.y = pitch (+ dziob w gore), angular.z = yaw (+ w lewo)
          Przeliczenie na tau FRD [X Y Z K M N]: X=surge, Y=-sway, Z=-heave, K=roll, M=pitch, N=-yaw
          (Fossen FRD: K+ = prawa burta w dol, M+ = dziob w GORE, N+ = dziob w prawo; patrz docs/stabilizacja/),
          kazda os skalowana przez tau_max tej osi (max osiagalne przy limitach pednikow) i przez gain.
          /rov/arm Bool, /rov/gain Float32
          /rov/mixer/config String JSON {"slots": {"FLU": {"motor": 5, "inv": false}, ...}}  (ktory silnik M1-8 siedzi w ktorej pozycji)
          /rov/thruster_test String JSON {"motor": 1, "value": 0.15, "duration": 2}  albo  {"dof": "heave", "value": 0.15, "duration": 2}
Wyjscie:  /rov/thrusters (20 Hz, zawsze), /rov/control/status String JSON (armed, gain, thrust, f_N, slots, alloc, test)
          /rov/debug/mixer String JSON co tick (do analizy z bagow): src (pilot/test/disarmed/timeout), armed, gain, cmd_age,
             cmd[6] (wejscie -1..1), tau[6] (zadane X Y Z K M N, N/Nm), f[8] (sily po saturacji, kolejnosc al.NAMES), sat, thr[8] (M1..M8)
Sloty: FLU FRU RLU RRU (gorne) FLD FRD RLD RRD (dolne) = pedniki PGL PGP TGL TGP PDL PDP TDL TDP z allocation.py.
"""
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rov_msgs.msg import ThrusterCommand
from std_msgs.msg import Bool, Float32, String

from rov_control import allocation as al

AXES = ("surge", "sway", "heave", "roll", "pitch", "yaw")
SLOTS = ("FLU", "FRU", "RLU", "RRU", "FLD", "FRD", "RLD", "RRD")
DEFAULT_SLOTS = {"FRD": 1, "FLD": 2, "RRD": 3, "RLD": 4, "FRU": 5, "FLU": 6, "RRU": 7, "RLU": 8}
CONFIG_FILE = os.path.expanduser("~/nowy_rov/config/thrusters.json")
TEST_MAX, TEST_MAX_S = 0.3, 3.0
DOF_OF_AXIS = {"surge": ("X", 1), "sway": ("Y", -1), "heave": ("Z", -1), "roll": ("K", 1), "pitch": ("M", 1), "yaw": ("N", -1)}


class ThrusterMixer(Node):
    def __init__(self):
        super().__init__("thruster_mixer")
        p = self.declare_parameter
        p("rate_hz", 20.0); p("cmd_timeout_s", 0.5); p("default_gain", 0.3)
        p("beta_deg", al.BETA_DEG)
        p("r_cg_mm", [0.0, 0.0, 0.0])          # przesuniecie srodka masy wzgledem srodka geometrycznego [mm]
        p("thrust_model", "t200")              # "t200" = krzywa T200 16 V z allocation.py, "power" = Fmax * cmd^exp
        p("thrust_scale", 1.0)                  # mnoznik krzywej T200 (podrobki, inne napiecie) - po pomiarze
        p("f_max_fwd", 40.0); p("f_max_rev", 30.0)   # [N] na pednik - tylko thrust_model "power"
        p("cmd_exponent", 1.0)                  # sila = Fmax * cmd^exp - tylko thrust_model "power"
        p("weights", [1.0] * 8)                 # wagi pednikow (wieksza = mniej uzywany)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.timeout = float(g("cmd_timeout_s")); self.gain = float(g("default_gain"))
        self.f_fwd, self.f_rev, self.expo = float(g("f_max_fwd")), float(g("f_max_rev")), float(g("cmd_exponent"))
        self.thrust_model = str(g("thrust_model"))
        self.tab_fwd = self.tab_rev = None
        if self.thrust_model == "t200":
            self.tab_fwd, self.tab_rev = al.thrust_tables(float(g("thrust_scale")))
            self.f_fwd, self.f_rev = self.tab_fwd[-1][1], self.tab_rev[-1][1]

        # ---- alokacja ----
        self.B = al.build_B(r_cg_mm=list(g("r_cg_mm")), beta_deg=float(g("beta_deg")))
        W = np.diag(list(g("weights"))) if any(abs(w - 1.0) > 1e-9 for w in g("weights")) else None
        self.Bp = al.pinv_weighted(self.B, W)
        self.info = al.analyze(self.B)
        self.tau_max = al.dof_limits(self.Bp, self.B, self.f_fwd, self.f_rev)
        self.pure = al.verify_pure_moves(self.Bp, self.B)
        self.get_logger().info(f"alokacja: rank={self.info['rank']} cond={self.info['cond']} tau_max={self.tau_max} "
                               f"ciag={self.thrust_model} +{self.f_fwd:.1f}/-{self.f_rev:.1f} N")
        if not self.info["controllable"]:
            self.get_logger().error("macierz B nie ma rzedu 6 - nie wszystkie DOF sterowalne!")

        self.armed = False; self.cmd = [0.0] * 6; self.cmd_time = 0.0
        self.thrust = [0.0] * 8; self.f_N = [0.0] * 8; self.sat_scale = 1.0; self.tau = [0.0] * 6
        self.test = None   # (thrust[8], until, opis)
        self.slots = {s: {"motor": DEFAULT_SLOTS.get(s, 0), "inv": False} for s in SLOTS}
        self.load_config(); self.build_map()

        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Twist, "/rov/cmd_vel", self.on_cmd, be)
        self.create_subscription(Twist, "/rov/cmd_vel_reliable", self.on_cmd, 10)
        self.create_subscription(Bool, "/rov/arm", self.on_arm, 10)
        self.create_subscription(Float32, "/rov/gain", self.on_gain, 10)
        self.create_subscription(String, "/rov/mixer/config", self.on_config, 10)
        self.create_subscription(String, "/rov/thruster_test", self.on_test, 10)
        self.pub_thr = self.create_publisher(ThrusterCommand, "/rov/thrusters", 10)
        self.pub_status = self.create_publisher(String, "/rov/control/status", 10)
        self.pub_dbg = self.create_publisher(String, "/rov/debug/mixer", 10)
        self.create_timer(1.0 / float(g("rate_hz")), self.tick)
        self.create_timer(0.2, self.publish_status)

    # ---- przypisanie pednik geometryczny -> silnik fizyczny ----
    def load_config(self):
        try:
            with open(CONFIG_FILE) as f:
                data = json.load(f)
            for s in SLOTS:
                if s in data.get("slots", {}):
                    v = data["slots"][s]; self.slots[s] = {"motor": int(v.get("motor", 0)), "inv": bool(v.get("inv", False))}
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().error(f"zla konfiguracja {CONFIG_FILE}: {e}")

    def save_config(self):
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, "w") as f:
            json.dump({"slots": self.slots, "legend": "FLU=przod lewy gora (PGL) ... RRD=tyl prawy dol (TDP); inv = odwrocony kierunek"}, f, indent=1)

    def build_map(self):
        """motor_of[i] = indeks silnika (0..7) dla pednika i w kolejnosci al.NAMES, sign_of[i] = +-1; -1 = nieprzypisany."""
        self.motor_of = [-1] * 8; self.sign_of = [1.0] * 8; used = {}
        for i, name in enumerate(al.NAMES):
            s = self.slots[al.SLOT_OF[name]]; m = s["motor"]
            if 1 <= m <= 8 and m not in used:
                used[m] = name; self.motor_of[i] = m - 1; self.sign_of[i] = -1.0 if s["inv"] else 1.0
            elif 1 <= m <= 8:
                self.get_logger().warning(f"silnik M{m} przypisany podwojnie ({used[m]} i {name}) - {name} pominiety")

    def to_motors(self, cmd8):
        out = [0.0] * 8
        for i in range(8):
            if self.motor_of[i] >= 0:
                out[self.motor_of[i]] = float(np.clip(cmd8[i] * self.sign_of[i], -1.0, 1.0))
        return out

    # ---- alokacja komendy ----
    def allocate_cmd(self, cmd, gain):
        """cmd[6] (-1..1, semantyka GUI) -> (sygnaly 8 pednikow geometrycznych, sily f [N], skala saturacji)."""
        tau = np.zeros(6)
        for k, ax in enumerate(AXES):
            dof, sgn = DOF_OF_AXIS[ax]
            tau[al.DOF.index(dof)] = sgn * cmd[k] * self.tau_max[dof] * gain
        self.tau = [round(float(v), 3) for v in tau]
        f = al.allocate(self.Bp, tau)
        f, scale, _ = al.saturate(f, self.f_fwd, self.f_rev)
        return al.force_to_cmd(f, self.f_fwd, self.f_rev, self.expo, self.tab_fwd, self.tab_rev), f, scale

    def on_config(self, m: String):
        try:
            for s, v in json.loads(m.data).get("slots", {}).items():
                if s in SLOTS:
                    self.slots[s] = {"motor": int(v.get("motor", 0)), "inv": bool(v.get("inv", False))}
            self.build_map(); self.save_config()
            self.get_logger().info(f"nowa konfiguracja pednikow: {self.slots}")
        except Exception as e:
            self.get_logger().error(f"config: {e}")

    def on_test(self, m: String):
        try:
            d = json.loads(m.data)
            value = float(np.clip(float(d.get("value", 0.0)), -TEST_MAX, TEST_MAX))
            dur = float(np.clip(float(d.get("duration", 2.0)), 0.0, TEST_MAX_S))
            if self.armed:
                self.get_logger().warning("test odrzucony: najpierw DISARM"); return
            if value == 0:
                self.test = None; return
            if "dof" in d:                                   # czysty ruch wg alokacji, skalowany tak, by max |sygnal| = value
                ax = d["dof"]
                if ax not in AXES:
                    return
                cmd = [0.0] * 6; cmd[AXES.index(ax)] = 1.0
                c8, _, _ = self.allocate_cmd(cmd, 1.0)
                peak = float(np.max(np.abs(c8))) or 1.0
                th = self.to_motors(c8 / peak * value); desc = f"{ax} {value:+.2f}"
            else:
                motor = int(d.get("motor", 0))
                if not 1 <= motor <= 8:
                    return
                th = [0.0] * 8; th[motor - 1] = value; desc = f"M{motor} {value:+.2f}"
            self.test = (th, time.monotonic() + dur, desc)
            self.get_logger().info(f"test {desc} przez {dur:.1f} s")
        except Exception as e:
            self.get_logger().error(f"test: {e}")

    def on_cmd(self, m: Twist):
        self.cmd = [clamp(m.linear.x), clamp(m.linear.y), clamp(m.linear.z), clamp(m.angular.x), clamp(m.angular.y), clamp(m.angular.z)]
        self.cmd_time = time.monotonic()

    def on_arm(self, m: Bool):
        if self.armed != m.data:
            self.get_logger().info("ARM" if m.data else "DISARM")
        self.armed = m.data
        if self.armed:
            self.test = None

    def on_gain(self, m: Float32):
        self.gain = clamp(m.data, 0.0, 1.0)

    def tick(self):
        now = time.monotonic()
        if self.test and now >= self.test[1]:
            self.test = None
        if self.test and not self.armed:
            self.thrust = list(self.test[0]); self.f_N = [0.0] * 8; self.sat_scale = 1.0; self.tau = [0.0] * 6; src = "test"
        elif self.armed and (now - self.cmd_time) < self.timeout:
            c8, f, scale = self.allocate_cmd(self.cmd, self.gain)
            self.thrust = self.to_motors(c8); self.f_N = [round(float(v), 2) for v in f]; self.sat_scale = scale; src = "pilot"
        else:
            self.thrust = [0.0] * 8; self.f_N = [0.0] * 8; self.sat_scale = 1.0; self.tau = [0.0] * 6
            src = "timeout" if self.armed else "disarmed"
        msg = ThrusterCommand(); msg.throttle = [float(v) for v in self.thrust]
        self.pub_thr.publish(msg)
        age = now - self.cmd_time if self.cmd_time else -1.0
        self.pub_dbg.publish(String(data=json.dumps({
            "src": src, "armed": self.armed, "gain": round(self.gain, 3), "cmd_age": round(age, 3),
            "cmd": [round(v, 3) for v in self.cmd], "tau": self.tau, "f": self.f_N, "sat": round(float(self.sat_scale), 3),
            "thr": [round(v, 4) for v in self.thrust], "test": self.test[2] if self.test else None}, separators=(",", ":"))))

    def publish_status(self):
        age = time.monotonic() - self.cmd_time if self.cmd_time else None
        st = {"armed": self.armed, "gain": round(self.gain, 3), "cmd_age_s": None if age is None else round(age, 2),
              "cmd": dict(zip(AXES, [round(v, 2) for v in self.cmd])),
              "thrust": [round(v, 3) for v in self.thrust], "f_N": self.f_N, "sat": round(self.sat_scale, 3),
              "slots": self.slots,
              "test": {"desc": self.test[2], "left_s": round(self.test[1] - time.monotonic(), 1)} if self.test else None,
              "alloc": {"rank": self.info["rank"], "cond": self.info["cond"], "sv": self.info["singular_values"],
                        "tau_max": self.tau_max, "f_max": [round(self.f_fwd, 1), round(self.f_rev, 1)], "thrust_model": self.thrust_model, "names": al.NAMES,
                        "pure": {k: {"f": v["f"], "err": v["err"], "coupling": v["coupling"]} for k, v in self.pure.items()},
                        "unassigned": [al.NAMES[i] for i in range(8) if self.motor_of[i] < 0]}}
        self.pub_status.publish(String(data=json.dumps(st)))


def clamp(v, lo=-1.0, hi=1.0):
    v = float(v)
    if v != v:
        return 0.0
    return max(lo, min(hi, v))


def main():
    rclpy.init()
    node = ThrusterMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = ThrusterCommand(); msg.throttle = [0.0] * 8
        for _ in range(3):
            node.pub_thr.publish(msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
