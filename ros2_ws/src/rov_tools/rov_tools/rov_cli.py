#!/usr/bin/env python3
"""CLI testowe ROV (ros2 run rov_tools rov <komenda>):

  rov status                   jednorazowy zrzut telemetrii ESC + IMU + cisnienie
  rov watch [hz]               ciagly podglad (domyslnie 2 Hz)
  rov thrust <m|all|e1|e2> <v> gaz -1.0..1.0, publikuje 20 Hz az do Ctrl+C (potem stop)
  rov stop                     stop wszystkich (CMD_STOP)
  rov cmd <m|all|e1|e2> <0..47> [n]   komenda DShot (silniki musza stac)
  rov 3d on|off                tryb 3D w ESC (cmd 10/9 + flaga firmware) - potem 'rov save'
  rov dir <m> n|r              kierunek (cmd 20/21) - potem 'rov save'
  rov save | beep [1-5] | edt
  rov poles <n> | rate 150|300|600|1200 | cz | cs <mV/A>
  rov bms [watch]              odczyt BMS Daly (ogniwa, temperatury, MOS, bledy); watch = odswiezanie 1 Hz
  rov power cycle|on           cycle = odetnij zasilanie pojazdu (SSR) na 5 s i wroc; on = anuluj odciecie
  rov led <r> <g> <b> [jasnosc] ring WS2812B, wartosci 0..1 (np. rov led 1 1 1 0.3); rov led off
  rov spin <m> <-0.3..0.3> [s] test pojedynczego silnika przez mikser (tylko DISARM, max 3 s), np. rov spin 3 0.15 2
  rov move <dof> <-0.3..0.3> [s] test czystego ruchu wg alokacji: surge|sway|heave|roll|pitch|yaw, np. rov move heave 0.15 2
  rov alloc                    tabela alokacji: rzad, uwarunkowanie, max na DOF, sily pednikow dla czystych ruchow
  rov zero                     zeruj IMU (aktualna orientacja = poziom, kurs 0)
  rov att                      orientacja (roll/pitch/yaw wzgledem zera), tryby, wektor zadany, korekty
  rov hold yaw|level|depth on|off   wlacz/wylacz trzymanie kursu / poziomu / glebokosci
  rov sp yaw=<deg> roll=<deg> pitch=<deg> depth=<m>   wektor zadany (dowolny podzbior)
  rov gains <yaw|roll|pitch|depth> <kp> <ki> <kd>     nastawy PID (do restartu; na stale w config/attitude.yaml)
  rov leak [reset]             stan czujnikow zalania (biezacy + zatrzask); reset = skasuj zatrzask
  rov events [n]               ostatnie n zdarzen z ~/nowy_rov/logs/events.csv (domyslnie 20)
  rov logs [n] [all]           ostatnie bledy/ostrzezenia stosu z journala (all = wszystkie linie)
  rov bags                     lista nagranych bagow (ros2 bag) z rozmiarem; odtwarzanie: ros2 bag play <katalog>
  rov diag                     paczka diagnostyczna ~/nowy_rov/logs/diag_<data>.tar.gz (journal, zdarzenia, status, konfiguracja)
  rov p0                       zeruj glebokosc (serwis /rov/depth/zero)
  rov imucal                   kalibracja zyro (serwis /rov/imu/calibrate)
"""
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rov_msgs.msg import BmsStatus, EscCommand, EscTelemetry, ThrusterCommand
from sensor_msgs.msg import FluidPressure, Imu, Temperature
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
NEVER = 0xFFFFFFFF


def parse_mask(s: str) -> int:
    s = s.lower()
    if s in ("all", "a"):
        return 0xFF
    if s == "e1":
        return 0x0F
    if s == "e2":
        return 0xF0
    m = int(s)
    if not 1 <= m <= 8:
        raise ValueError("silnik 1..8")
    return 1 << (m - 1)


class Cli(Node):
    def __init__(self):
        super().__init__("rov_cli")
        self.pub_thr = self.create_publisher(ThrusterCommand, "/rov/thrusters", 10)
        self.pub_cmd = self.create_publisher(EscCommand, "/rov/esc/command", 10)
        self.tlm = None
        self.imu = None
        self.press = {}
        self.temp = {}
        self.depth = None
        self.create_subscription(EscTelemetry, "/rov/esc/telemetry", self._on_tlm, 10)
        self.create_subscription(Imu, "/rov/imu", self._on_imu, BEST_EFFORT)
        self.create_subscription(Float32, "/rov/depth", self._on_depth, BEST_EFFORT)
        for n in ("ms5837", "ms5803"):
            self.create_subscription(FluidPressure, f"/rov/pressure/{n}",
                                     lambda m, n=n: self.press.__setitem__(n, m), BEST_EFFORT)
            self.create_subscription(Temperature, f"/rov/temperature/{n}",
                                     lambda m, n=n: self.temp.__setitem__(n, m), BEST_EFFORT)

    def _on_tlm(self, m):
        self.tlm = m

    def _on_imu(self, m):
        self.imu = m

    def _on_depth(self, m):
        self.depth = m.data

    # ---- wysylanie ----
    def wait_matched(self, pub, sec=2.0):
        # czekaj az publisher zostanie dopasowany z subskrypcja (agent/Pico) - inaczej pierwsze wiadomosci gina
        t0 = time.time()
        while time.time() - t0 < sec and pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.02)
        if pub.get_subscription_count() == 0:
            print("uwaga: brak subskrybenta na", pub.topic_name, "(agent/Pico nie dziala?)")

    def esc_cmd(self, mask, command, repeat=0):
        m = EscCommand(motor_mask=mask, command=command, repeat=repeat)
        self.wait_matched(self.pub_cmd)
        self.pub_cmd.publish(m)
        rclpy.spin_once(self, timeout_sec=0.1)
        print(f"EscCommand mask=0x{mask:02X} cmd={command} rep={repeat}")

    def thrust_loop(self, mask, value):
        msg = ThrusterCommand()
        msg.throttle = [value if mask & (1 << i) else 0.0 for i in range(8)]
        print(f"gaz {value:+.2f} na maske 0x{mask:02X}, 20 Hz, Ctrl+C = stop")
        self.wait_matched(self.pub_thr)
        self.tlm = None   # pokazuj tylko swieza telemetrie
        last = 0.0
        while rclpy.ok() and not STOP:
            self.pub_thr.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - last > 0.5 and self.tlm:
                last = time.time()
                t = self.tlm
                print("eRPM " + " ".join(f"{v:6d}" for v in t.erpm)
                      + f" | I {t.esc_current_a[0]:5.2f} {t.esc_current_a[1]:5.2f} A"
                      + (" FAILSAFE" if t.failsafe else ""))
        msg.throttle = [0.0] * 8
        for _ in range(5):
            self.pub_thr.publish(msg)
            time.sleep(0.02)
        print("stop")

    # ---- status ----
    def wait_data(self, sec=3.0):
        # czekaj az przyjdzie telemetria ESC, IMU, cisnienie i glebokosc (albo minie sec)
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tlm is not None and self.imu is not None and len(self.press) == 2 and self.depth is not None:
                break
        t1 = time.time()
        while time.time() - t1 < 0.3:
            rclpy.spin_once(self, timeout_sec=0.05)

    def print_status(self):
        t = self.tlm
        if t is None:
            print("brak /rov/esc/telemetry - agent dziala? Pico podlaczone?")
        else:
            print(f"--- DShot{t.dshot_rate_kbit} poles={t.poles} 3d={t.mode_3d} failsafe={t.failsafe} "
                  f"agent={t.agent_connected} loop={t.loop_count} cmd_age={t.cmd_age_ms} ms ---")
            for i in range(8):
                age = t.kiss_age_ms[i]
                if age != NEVER:
                    kiss = (f"KISS T={t.kiss_temp_c[i]:3d}C V={t.kiss_voltage_v[i]:5.2f} "
                            f"I={t.kiss_current_a[i]:5.2f}A mAh={t.kiss_mah[i]:4d} "
                            f"eRPM={t.kiss_erpm[i]:6d} ({age} ms)")
                else:
                    kiss = "KISS --"
                print(f"M{i+1} dshot={t.dshot_value[i]:4d} | bidir {'OK' if t.bidir_ok[i] else '--'} "
                      f"eRPM={t.erpm[i]:6d} RPM={t.rpm[i]:6d} ok={t.bidir_ok_count[i]} "
                      f"crc={t.bidir_crc_errors[i]} none={t.bidir_no_reply[i]} | {kiss}")
            print(f"CUR ESC1={t.esc_current_a[0]:.2f} A ESC2={t.esc_current_a[1]:.2f} A")
        if self.imu:
            q = self.imu.orientation
            roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            g, a = self.imu.angular_velocity, self.imu.linear_acceleration
            print(f"IMU roll={roll:6.1f} pitch={pitch:6.1f} yaw={yaw:6.1f} | gyro "
                  f"{math.degrees(g.x):6.1f} {math.degrees(g.y):6.1f} {math.degrees(g.z):6.1f} dps | "
                  f"acc {a.x:5.2f} {a.y:5.2f} {a.z:5.2f} m/s2")
        else:
            print("IMU: brak danych")
        for n in ("ms5837", "ms5803"):
            if n in self.press:
                tt = self.temp.get(n)
                print(f"{n}: P={self.press[n].fluid_pressure / 100:.2f} mbar"
                      + (f" T={tt.temperature:.2f} C" if tt else ""))
        if self.depth is not None:
            print(f"glebokosc: {self.depth:.3f} m")

    FAULTS = [
        ["V ogniwa wys. L1", "V ogniwa wys. L2", "V ogniwa nis. L1", "V ogniwa nis. L2", "V pakietu wys. L1", "V pakietu wys. L2", "V pakietu nis. L1", "V pakietu nis. L2"],
        ["T lad. wys. L1", "T lad. wys. L2", "T lad. nis. L1", "T lad. nis. L2", "T rozl. wys. L1", "T rozl. wys. L2", "T rozl. nis. L1", "T rozl. nis. L2"],
        ["nadprad lad. L1", "nadprad lad. L2", "nadprad rozl. L1", "nadprad rozl. L2", "SOC wys. L1", "SOC wys. L2", "SOC nis. L1", "SOC nis. L2"],
        ["roznica V L1", "roznica V L2", "roznica T L1", "roznica T L2", "", "", "", ""],
        ["T MOS lad.", "T MOS rozl.", "czujnik T MOS lad.", "czujnik T MOS rozl.", "MOS lad. sklejony", "MOS rozl. sklejony", "MOS lad. przerwa", "MOS rozl. przerwa"],
        ["AFE", "pomiar V odpadl", "czujnik T ogniw", "EEPROM", "RTC", "precharge", "komunikacja", "komunikacja wewn."],
        ["modul pradu", "pomiar V pakietu", "ZWARCIE", "zakaz lad. (nis. V)", "", "", "", ""],
        ["", "", "", "", "", "", "", ""]]

    def print_bms(self, watch=False):
        box = {}
        self.create_subscription(BmsStatus, "/rov/bms/status", lambda m: box.__setitem__("b", m), 10)
        while rclpy.ok() and not STOP:
            t0 = time.time()
            while time.time() - t0 < 3.0 and "b" not in box:
                rclpy.spin_once(self, timeout_sec=0.1)
            b = box.pop("b", None)
            if watch:
                print("\033[2J\033[H", end="")
            if b is None:
                print("brak /rov/bms/status - Pico BMS podlaczone? agent dziala?")
                if not watch:
                    return
                continue
            print(f"BMS {'ONLINE' if b.online else 'BRAK ODPOWIEDZI'}  (ostatnia ramka {b.age_ms} ms temu, CAN rx={b.can_rx} tx={b.can_tx} err={b.can_errors})")
            print(f"  {b.voltage_v:6.2f} V   {b.current_a:+6.1f} A   SOC {b.soc_pct:5.1f} %   pozostalo {b.remaining_ah:.2f} Ah   "
                  f"stan: {['spoczynek', 'ladowanie', 'rozladowanie'][b.state] if b.state < 3 else b.state}")
            print(f"  MOS lad.={'ON' if b.charge_mos else 'off'} rozl.={'ON' if b.discharge_mos else 'off'}   ladowarka={b.charger_connected} obciazenie={b.load_connected}   "
                  f"cykle={b.cycles}  bms_life={b.bms_life}  DI/DO=0x{b.dio_bits:02X}")
            print(f"  ogniwa {b.cell_count}S: max {b.max_cell_mv} mV (#{b.max_cell_no})  min {b.min_cell_mv} mV (#{b.min_cell_no})  dV {b.max_cell_mv - b.min_cell_mv} mV")
            for i in range(b.cell_count):
                bal = (b.balance_bits[i >> 3] >> (i & 7)) & 1
                mark = " <max" if i + 1 == b.max_cell_no else (" <min" if i + 1 == b.min_cell_no else "")
                print(f"    #{i+1:2d}  {b.cell_voltage_v[i]:.3f} V{'  BALANS' if bal else ''}{mark}")
            temps = ", ".join(f"#{i+1} {b.temperature_c[i]} C" for i in range(b.temp_count))
            print(f"  temperatury ({b.temp_count}): {temps}   max {b.max_temp_c} C (#{b.max_temp_no})  min {b.min_temp_c} C (#{b.min_temp_no})")
            faults = [self.FAULTS[by][bit] or f"bajt{by}.bit{bit}" for by in range(8) for bit in range(8) if (b.fault_bits[by] >> bit) & 1]
            print("  BLEDY: " + (", ".join(faults) if faults else "brak"))
            if not watch:
                return
            time.sleep(1.0)

    def call_trigger(self, name):
        cl = self.create_client(Trigger, name)
        if not cl.wait_for_service(timeout_sec=3.0):
            print(f"serwis {name} niedostepny")
            return
        fut = cl.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        print(fut.result().message if fut.result() else "brak odpowiedzi")


STOP = False


def _on_sigint(signum, frame):
    global STOP
    STOP = True


def main():
    global STOP
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help", "help"):
        print(__doc__)
        return
    # wlasna obsluga Ctrl+C: rclpy nie zamyka kontekstu, wiec po przerwaniu mozna jeszcze wyslac stop
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _on_sigint)
    signal.signal(signal.SIGTERM, _on_sigint)
    n = Cli()
    c = args[0].lower()
    try:
        if c == "status":
            n.wait_data()
            n.print_status()
        elif c == "watch":
            hz = float(args[1]) if len(args) > 1 else 2.0
            while rclpy.ok() and not STOP:
                n.wait_data(1.0 / hz)
                print("\033[2J\033[H", end="")
                n.print_status()
        elif c == "thrust":
            n.thrust_loop(parse_mask(args[1]), max(-1.0, min(1.0, float(args[2]))))
        elif c == "stop":
            n.esc_cmd(0xFF, EscCommand.CMD_STOP)
        elif c == "cmd":
            n.esc_cmd(parse_mask(args[1]), int(args[2]), int(args[3]) if len(args) > 3 else 0)
        elif c == "3d":
            on = args[1].lower() == "on"
            n.esc_cmd(0xFF, EscCommand.DSHOT_3D_ON if on else EscCommand.DSHOT_3D_OFF)
            time.sleep(0.3)
            n.esc_cmd(0xFF, EscCommand.CMD_SET_3D, 1 if on else 0)
            print("pamietaj: rov save")
        elif c == "dir":
            rev = args[2].lower().startswith("r")
            n.esc_cmd(parse_mask(args[1]), EscCommand.DSHOT_DIR_REVERSE if rev else EscCommand.DSHOT_DIR_NORMAL)
        elif c == "save":
            n.esc_cmd(0xFF, EscCommand.DSHOT_SAVE)
        elif c == "beep":
            b = int(args[1]) if len(args) > 1 else 1
            n.esc_cmd(0xFF, max(1, min(5, b)), 1)
        elif c == "edt":
            n.esc_cmd(0xFF, EscCommand.DSHOT_EDT_ON)
            time.sleep(0.3)
            n.esc_cmd(0xFF, EscCommand.CMD_SET_EDT, 1)
        elif c == "poles":
            n.esc_cmd(0xFF, EscCommand.CMD_SET_POLES, int(args[1]))
        elif c == "rate":
            n.esc_cmd(0xFF, EscCommand.CMD_SET_RATE, {150: 1, 300: 2, 600: 3, 1200: 4}[int(args[1])])
        elif c == "cz":
            n.esc_cmd(0xFF, EscCommand.CMD_CURRENT_ZERO)
        elif c == "cs":
            n.esc_cmd(0xFF, EscCommand.CMD_CURRENT_SCALE, int(float(args[1])))
        elif c == "bms":
            n.print_bms(watch=len(args) > 1 and args[1] == "watch")
        elif c == "led":
            from std_msgs.msg import ColorRGBA
            pub = n.create_publisher(ColorRGBA, "/rov/led", 10)
            n.wait_matched(pub)
            if args[1].lower() == "off":
                m = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
            else:
                m = ColorRGBA(r=float(args[1]), g=float(args[2]), b=float(args[3]), a=float(args[4]) if len(args) > 4 else 1.0)
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.2)
            print(f"led r={m.r} g={m.g} b={m.b} jasnosc={m.a}")
        elif c in ("spin", "move"):
            import json as _json
            from std_msgs.msg import String as _Str
            pub = n.create_publisher(_Str, "/rov/thruster_test", 10)
            n.wait_matched(pub)
            v = float(args[2]); d = float(args[3]) if len(args) > 3 else 2.0
            if c == "spin":
                m = int(args[1]); pub.publish(_Str(data=_json.dumps({"motor": m, "value": v, "duration": d}))); what = f"M{m}"
            else:
                pub.publish(_Str(data=_json.dumps({"dof": args[1], "value": v, "duration": d}))); what = args[1]
            rclpy.spin_once(n, timeout_sec=0.2)
            print(f"test {what} {v:+.2f} przez {d:.1f} s (mikser musi byc DISARM)")
        elif c == "logs":
            import subprocess
            k = int(args[1]) if len(args) > 1 and args[1].isdigit() else 40
            out = subprocess.run(["journalctl", "-u", "rov", "--no-pager", "-n", "4000", "-o", "short"], capture_output=True, text=True).stdout
            lines = out.splitlines()
            if "all" not in args:
                lines = [ln for ln in lines if any(w in ln for w in ("ERROR", "WARN", "error", "Error", "Traceback", "died", "respawn", "OFFLINE", "ZALANIE", "failsafe", "SKOK"))]
            print("\n".join(ln[:220] for ln in lines[-k:]) or "brak")
        elif c == "bags":
            import os, subprocess
            d = os.path.expanduser("~/nowy_rov/logs/bags")
            if not os.path.isdir(d):
                print("brak bagow"); return
            tot = 0
            for b in sorted(os.listdir(d)):
                p = os.path.join(d, b)
                sz = sum(os.path.getsize(os.path.join(p, f)) for f in os.listdir(p)) / 1e6 if os.path.isdir(p) else 0
                tot += sz
                print(f"{b}  {sz:8.1f} MB  {'(aktualny)' if b == sorted(os.listdir(d))[-1] else ''}")
            print(f"razem {tot / 1024:.2f} GB w {d}")
        elif c == "diag":
            import os, subprocess, tarfile, tempfile
            from datetime import datetime
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            out = os.path.expanduser(f"~/nowy_rov/logs/diag_{ts}.tar.gz")
            tmp = tempfile.mkdtemp()
            def sh(cmd, name):
                r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=60)
                open(os.path.join(tmp, name), "w").write(r.stdout + r.stderr)
            sh("journalctl -u rov --no-pager --since '-2 hours'", "journal_rov_2h.txt")
            sh("journalctl -b --no-pager -p warning", "journal_boot_warnings.txt")
            sh("dmesg 2>/dev/null | tail -300", "dmesg_tail.txt")
            sh("uptime; date -Is; timedatectl; df -h /; free -h; lsusb; ls -l /dev/serial/by-id/; ip -4 addr; systemctl status rov --no-pager", "system.txt")
            sh("source ~/nowy_rov/ROV_ROS2/scripts/env.sh; ros2 node list; ros2 topic list; for t in /rov/status /rov/bms/info /rov/control/status /rov/ctrl/status /rov/camera/info; do echo == $t; timeout 4 ros2 topic echo --once --truncate-length 2000 $t 2>&1 | head -40; done", "ros_snapshot.txt")
            sh("tail -500 ~/nowy_rov/logs/events.csv", "events_tail.csv")
            sh("cat ~/nowy_rov/config/thrusters.json; cat ~/nowy_rov/ROV_ROS2/ros2_ws/src/rov_control/config/*.yaml", "config.txt")
            sh("cd ~/nowy_rov/ROV_ROS2 && git log -1 2>/dev/null; md5sum pico_fw/build/rov_pico.uf2 pico_bms/build/rov_bms.uf2 2>/dev/null", "versions.txt")
            with tarfile.open(out, "w:gz") as tf:
                for f in os.listdir(tmp):
                    tf.add(os.path.join(tmp, f), arcname=f"diag_{ts}/{f}")
            print(f"paczka: {out}  ({os.path.getsize(out) / 1e6:.2f} MB)\nz laptopa: scp orin@192.168.1.108:{out} .")
        elif c == "zero":
            n.call_trigger("/rov/imu/zero")
        elif c in ("hold", "sp", "gains", "att"):
            import json as _json
            from std_msgs.msg import String as _Str
            if c == "att":
                box = {}
                n.create_subscription(_Str, "/rov/ctrl/status", lambda m: box.__setitem__("s", m.data), 10)
                t0 = time.time()
                while time.time() - t0 < 3 and "s" not in box:
                    rclpy.spin_once(n, timeout_sec=0.1)
                if "s" not in box:
                    print("brak /rov/ctrl/status (attitude_controller?)")
                else:
                    s = _json.loads(box["s"])
                    print(f"IMU {'OK' if s['imu_ok'] else 'BRAK'}{'' if s['zeroed'] else ' (NIE wyzerowane)'}  roll={s['rpy'][0]:+.1f} pitch={s['rpy'][1]:+.1f} yaw={s['rpy'][2]:+.1f} deg  "
                          f"gyro={s['gyro_dps']} dps  acc={s['acc']}  glebokosc={s['depth']}")
                    print(f"tryby: {s['mode']}  zawieszone: {s['suspended']}  pilot aktywny: {s['pilot_active']}")
                    print(f"wektor zadany: {s['setpoint']}  korekty: {s['corr']}")
                    print("nastawy: " + ", ".join(f"{k}={v}" for k, v in s["gains"].items()))
            else:
                topic = {"hold": "/rov/ctrl/mode", "sp": "/rov/ctrl/setpoint", "gains": "/rov/ctrl/gains"}[c]
                pub = n.create_publisher(_Str, topic, 10)
                n.wait_matched(pub)
                if c == "hold":
                    key = {"yaw": "yaw_hold", "level": "level", "depth": "depth_hold"}[args[1]]
                    payload = {key: args[2].lower() in ("on", "1", "true")}
                elif c == "sp":
                    km = {"yaw": "yaw_deg", "roll": "roll_deg", "pitch": "pitch_deg", "depth": "depth_m"}
                    payload = {km[k]: float(v) for k, v in (a.split("=") for a in args[1:])}
                else:
                    payload = {args[1]: [float(args[2]), float(args[3]), float(args[4])]}
                pub.publish(_Str(data=_json.dumps(payload)))
                rclpy.spin_once(n, timeout_sec=0.2)
                print(f"{topic}: {payload}")
        elif c == "alloc":
            import json as _json
            from std_msgs.msg import String as _Str
            box = {}
            n.create_subscription(_Str, "/rov/control/status", lambda m: box.__setitem__("s", m.data), 10)
            t0 = time.time()
            while time.time() - t0 < 3 and "s" not in box:
                rclpy.spin_once(n, timeout_sec=0.1)
            if "s" not in box:
                print("brak /rov/control/status (mikser?)")
            else:
                s = _json.loads(box["s"]); a = s["alloc"]
                print(f"rzad {a['rank']}/6  uwarunkowanie {a['cond']}  wart. osobliwe {a['sv']}  limity pednika +{a['f_max'][0]}/-{a['f_max'][1]} N")
                print("nieprzypisane: " + (" ".join(a["unassigned"]) or "brak"))
                print("DOF    max        blad     sprzez.  " + " ".join(f"{nm:>7}" for nm in a["names"]))
                unit = {"X": "N", "Y": "N", "Z": "N", "K": "Nm", "M": "Nm", "N": "Nm"}
                for dof, p in a["pure"].items():
                    print(f"{dof:3}  {a['tau_max'][dof]:7.2f} {unit[dof]:2} {p['err']:8.1e} {p['coupling']:8.1e}  " + " ".join(f"{v:+7.3f}" for v in p["f"]))
                print("sloty: " + " ".join(f"{k}={'-' if v['inv'] else ''}M{v['motor']}" for k, v in s["slots"].items()))
        elif c == "leak":
            from std_msgs.msg import Bool, UInt8
            if len(args) > 1 and args[1] == "reset":
                pub = n.create_publisher(Bool, "/rov/leak/reset", 10)
                n.wait_matched(pub)
                pub.publish(Bool(data=True))
                rclpy.spin_once(n, timeout_sec=0.2)
                print("zatrzask alarmu skasowany")
            else:
                box = {}
                n.create_subscription(UInt8, "/rov/leak", lambda m: box.__setitem__("d", m.data), 10)
                t0 = time.time()
                while time.time() - t0 < 3 and "d" not in box:
                    rclpy.spin_once(n, timeout_sec=0.1)
                if "d" not in box:
                    print("brak /rov/leak (Pico BMS?)")
                else:
                    d = box["d"]
                    print(f"teraz: czujnik1={'WODA' if d & 1 else 'sucho'} czujnik2={'WODA' if d & 2 else 'sucho'} | "
                          f"zatrzask: czujnik1={'TAK' if d & 16 else 'nie'} czujnik2={'TAK' if d & 32 else 'nie'}")
        elif c == "events":
            import os
            p = os.path.expanduser("~/nowy_rov/logs/events.csv")
            k = int(args[1]) if len(args) > 1 else 20
            lines = open(p, encoding="utf-8").read().splitlines() if os.path.exists(p) else []
            print("\n".join(lines if len(lines) <= k + 1 else lines[:1] + lines[-k:]) if lines else "brak pliku zdarzen")
        elif c == "power":
            from std_msgs.msg import Bool
            pub = n.create_publisher(Bool, "/rov/power", 10)
            n.wait_matched(pub)
            on = args[1].lower() == "on"
            pub.publish(Bool(data=on))
            rclpy.spin_once(n, timeout_sec=0.2)
            print("anulowano odciecie" if on else "SSR odciety na 5 s (restart elektroniki pojazdu)")
        elif c == "p0":
            n.call_trigger("/rov/depth/zero")
        elif c == "imucal":
            n.call_trigger("/rov/imu/calibrate")
        else:
            print(__doc__)
    except (IndexError, ValueError, KeyError) as e:
        print(f"zle argumenty ({e})\n{__doc__}")
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
