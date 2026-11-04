#!/usr/bin/env python3
"""Logger zdarzen ROV -> plik CSV (domyslnie ~/nowy_rov/logs/events.csv) + log ROS.

Zdarzenia: zalanie (start/koniec per czujnik, reset alarmu przez uzytkownika), zasilanie pojazdu (SSR), ladowanie,
BMS online/offline, bledy BMS, ARM/DISARM, zdalny restart zasilania. Kazda linia: czas ISO, zdarzenie, szczegoly,
oraz migawka: napiecie, prad, SOC, glebokosc.
"""
import csv
import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rov_msgs.msg import BmsStatus
from std_msgs.msg import Bool, Float32, String, UInt8

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


class EventLogger(Node):
    def __init__(self):
        super().__init__("event_logger")
        self.declare_parameter("file", os.path.expanduser("~/nowy_rov/logs/events.csv"))
        self.path = self.get_parameter("file").value
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        new = not os.path.exists(self.path)
        self.f = open(self.path, "a", newline="", buffering=1)
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(["czas", "zdarzenie", "szczegoly", "napiecie_v", "prad_a", "soc_pct", "glebokosc_m"])
        self.snap = {"v": None, "i": None, "soc": None, "depth": None}
        self.prev = {}
        self.create_subscription(UInt8, "/rov/leak", self.on_leak, 10)
        self.create_subscription(Bool, "/rov/power/state", lambda m: self.changed("zasilanie_pojazdu", "ON" if m.data else "OFF"), 10)
        self.create_subscription(Bool, "/rov/power", lambda m: self.event("zdalne_odciecie" if not m.data else "anuluj_odciecie", ""), 10)
        self.create_subscription(Bool, "/rov/arm", lambda m: self.changed("arm", "ARM" if m.data else "DISARM"), 10)
        self.create_subscription(BmsStatus, "/rov/bms/status", self.on_bms, 10)
        self.create_subscription(Float32, "/rov/depth", lambda m: self.snap.__setitem__("depth", round(m.data, 2)), BEST_EFFORT)
        self.create_subscription(String, "/rov/control/status", self.on_mixer, 10)
        # obecnosc zrodel: Pico ESC (telemetria), Pico BMS (status), kamera, GUI (klienci rosbridge)
        from rov_msgs.msg import EscTelemetry
        from std_msgs.msg import Int32
        self.seen = {}
        self.create_subscription(EscTelemetry, "/rov/esc/telemetry", lambda m: self.alive("pico_esc", m.failsafe), 10)
        self.create_subscription(String, "/rov/bms/info", lambda m: self.alive("pico_bms", None), 10)
        self.create_subscription(String, "/rov/camera/info", self.on_camera, 10)
        self.create_subscription(Int32, "/client_count", lambda m: self.changed("gui_klienci", str(m.data)), 10)
        self.create_subscription(String, "/rov/ctrl/status", self.on_ctrl, 10)
        self.create_subscription(UInt8, "/rov/leak", lambda m: None, 10)
        self.create_timer(1.0, self.check_alive)
        self.event("logger_start", self.path)
        self.get_logger().info(f"zdarzenia -> {self.path}")
        # detekcja skoku zegara (NTP po starcie): psuje DDS -> trzeba zrestartowac usluge rov
        self._wall, self._mono = time.time(), time.monotonic()
        self.create_timer(1.0, self.check_clock)

    def check_clock(self):
        w, m = time.time(), time.monotonic()
        jump = (w - self._wall) - (m - self._mono)
        self._wall, self._mono = w, m
        if abs(jump) > 10:
            self.event("SKOK_ZEGARA", f"{jump:+.0f} s - DDS moze nie dzialac, zalecany: sudo systemctl restart rov")
            self.get_logger().error(f"skok zegara {jump:+.0f} s po starcie - zrestartuj usluge rov")

    def event(self, name, detail):
        row = [datetime.now().isoformat(timespec="seconds"), name, detail,
               self.snap["v"], self.snap["i"], self.snap["soc"], self.snap["depth"]]
        self.w.writerow(row)
        self.get_logger().info(f"{name} {detail}")

    def changed(self, key, value):
        if self.prev.get(key) != value:
            if key in self.prev or key in ("arm",):
                self.event(key, value)
            self.prev[key] = value

    def on_leak(self, m):
        cur, lat = m.data & 0x0F, m.data >> 4
        pcur, plat = self.prev.get("leak_cur", 0), self.prev.get("leak_lat", 0)
        for bit, name in ((1, "czujnik_1"), (2, "czujnik_2")):
            if cur & bit and not pcur & bit:
                self.event("ZALANIE_START", name)
            if pcur & bit and not cur & bit:
                self.event("zalanie_koniec", name)
        if plat and not lat:
            self.event("zalanie_reset_uzytkownik", f"skasowano zatrzask (bylo: {plat})")
        self.prev["leak_cur"], self.prev["leak_lat"] = cur, lat

    def on_bms(self, b):
        self.snap.update(v=round(b.voltage_v, 2), i=round(b.current_a, 2), soc=round(b.soc_pct, 1))
        self.changed("bms_online", "online" if b.online else "OFFLINE")
        self.changed("ladowanie", "TAK" if b.state == 1 or b.charger_connected else "nie")
        faults = ",".join(f"b{by}.{bit}" for by in range(8) for bit in range(8) if (b.fault_bits[by] >> bit) & 1)
        self.changed("bledy_bms", faults or "brak")
        if b.online and b.soc_pct and b.soc_pct < 20:
            self.changed("soc_niski", f"{b.soc_pct:.0f}%")

    def alive(self, name, failsafe=None):
        self.seen[name] = time.monotonic()
        if failsafe is not None:
            self.changed("failsafe_pico", "TAK" if failsafe else "nie")
        if self.prev.get(f"alive_{name}") != "online":
            if f"alive_{name}" in self.prev:
                self.event(f"{name}_online", "")
            self.prev[f"alive_{name}"] = "online"

    def check_alive(self):
        now = time.monotonic()
        for name, t in list(self.seen.items()):
            if now - t > 3.0 and self.prev.get(f"alive_{name}") == "online":
                self.prev[f"alive_{name}"] = "offline"
                self.event(f"{name}_OFFLINE", f"brak danych {now - t:.0f} s")

    def on_camera(self, m):
        try:
            s = json.loads(m.data)
            self.changed("kamera", "klatki OK" if s.get("alive") else "BRAK KLATEK")
            self.changed("kamera_rec", "nagrywa" if s.get("recording") else "stop")
        except Exception:
            pass

    def on_ctrl(self, m):
        try:
            s = json.loads(m.data)
            self.changed("tryb_stabilizacji", ",".join(k for k, v in s.get("mode", {}).items() if v) or "brak")
            if s.get("saturated"):
                self.changed("pid_nasycenie", ",".join(s["saturated"]))
            else:
                self.changed("pid_nasycenie", "nie")
        except Exception:
            pass

    def on_mixer(self, m):
        try:
            self.changed("arm", "ARM" if json.loads(m.data).get("armed") else "DISARM")
        except Exception:
            pass


def main():
    rclpy.init()
    n = EventLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.event("logger_stop", "")
        n.f.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
