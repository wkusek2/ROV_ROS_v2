#!/usr/bin/env python3
"""Symulator obu Pico (do testow GUI/miksera/regulatora bez pojazdu, np. na laptopie).

Publikuje to, co normalnie daja Pico ESC i Pico BMS: /rov/esc/telemetry, /rov/imu, /rov/pressure/*, /rov/depth,
/rov/status, /rov/bms/status, /rov/bms/info, /rov/power/state, /rov/leak; slucha /rov/thrusters (eRPM z gazu),
/rov/esc/command, /rov/led, /rov/power, /rov/leak/reset; serwisy /rov/depth/zero, /rov/imu/calibrate.
Prosta dynamika: predkosci katowe = suma momentow z gazu pednikow (uklad slotow jak w mikserze), glebokosc z heave.
Uruchomienie: ros2 launch rov_bringup rov.launch.py sim:=true agents:=false camera:=false record:=false
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rov_msgs.msg import BmsStatus, EscCommand, EscTelemetry, ThrusterCommand
from sensor_msgs.msg import FluidPressure, Imu, Temperature
from std_msgs.msg import Bool, ColorRGBA, Float32, String, UInt8
from std_srvs.srv import Trigger

BE = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


class SimPico(Node):
    def __init__(self):
        super().__init__("sim_pico")
        self.declare_parameter("thrusters_json", "")   # opcjonalnie sciezka do thrusters.json (domyslny uklad gdy brak)
        self.thr = [0.0] * 8
        self.erpm = [0.0] * 8
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.rates = [0.0, 0.0, 0.0]     # roll, pitch, yaw [rad/s] w ciele FLU
        self.depth = 0.3
        self.vz = 0.0
        self.soc = 87.0
        self.leak = 0
        self.ssr = True
        self.led = (0.0, 0.0, 0.0, 0.0)
        self.poles, self.mode_3d, self.cmd_t = 14, True, 0.0
        self.t0 = time.time()
        # publishery
        self.p_tlm = self.create_publisher(EscTelemetry, "/rov/esc/telemetry", 10)
        self.p_imu = self.create_publisher(Imu, "/rov/imu", BE)
        self.p_imut = self.create_publisher(Temperature, "/rov/imu/temperature", BE)
        self.p_pr = [self.create_publisher(FluidPressure, f"/rov/pressure/{n}", BE) for n in ("ms5837", "ms5803")]
        self.p_te = [self.create_publisher(Temperature, f"/rov/temperature/{n}", BE) for n in ("ms5837", "ms5803")]
        self.p_depth = self.create_publisher(Float32, "/rov/depth", BE)
        self.p_status = self.create_publisher(String, "/rov/status", 10)
        self.p_bms = self.create_publisher(BmsStatus, "/rov/bms/status", 10)
        self.p_bmsi = self.create_publisher(String, "/rov/bms/info", 10)
        self.p_pwr = self.create_publisher(Bool, "/rov/power/state", 10)
        self.p_leak = self.create_publisher(UInt8, "/rov/leak", 10)
        # subskrypcje
        self.create_subscription(ThrusterCommand, "/rov/thrusters", self.on_thr, 10)
        self.create_subscription(EscCommand, "/rov/esc/command", self.on_cmd, 10)
        self.create_subscription(ColorRGBA, "/rov/led", lambda m: setattr(self, "led", (m.r, m.g, m.b, m.a)), 10)
        self.create_subscription(Bool, "/rov/power", lambda m: setattr(self, "ssr", bool(m.data)), 10)
        self.create_subscription(Bool, "/rov/leak/reset", lambda m: setattr(self, "leak", self.leak & 0x0F), 10)
        self.create_subscription(String, "/rov/sim/leak", lambda m: setattr(self, "leak", int(m.data) | (int(m.data) << 4)), 10)  # test: "1","2","3","0"
        self.create_service(Trigger, "/rov/depth/zero", lambda rq, rs: self._ok(rs, "sim: p0 ustawione"))
        self.create_service(Trigger, "/rov/imu/calibrate", lambda rq, rs: self._ok(rs, "sim: kalibracja zyro"))
        self.create_timer(0.02, self.step_imu)      # 50 Hz
        self.create_timer(0.05, self.pub_tlm)       # 20 Hz
        self.create_timer(0.1, self.pub_press)      # 10 Hz
        self.create_timer(0.5, self.pub_bms)        # 2 Hz
        self.create_timer(1.0, self.pub_status)     # 1 Hz
        self.get_logger().info("SYMULATOR Pico ESC + BMS aktywny (bez sprzetu)")

    def _ok(self, rs, msg):
        rs.success, rs.message = True, msg
        return rs

    def on_thr(self, m):
        self.thr = list(m.throttle); self.cmd_t = time.time()

    def on_cmd(self, m):
        if m.command == EscCommand.CMD_SET_POLES:
            self.poles = m.repeat
        if m.command == EscCommand.DSHOT_3D_ON or (m.command == EscCommand.CMD_SET_3D and m.repeat):
            self.mode_3d = True
        if m.command == EscCommand.DSHOT_3D_OFF or (m.command == EscCommand.CMD_SET_3D and not m.repeat):
            self.mode_3d = False

    # ---- dynamika (zgrubna): momenty z geometrii slotow domyslnych miksera ----
    def step_imu(self):
        dt = 0.02
        t = self.thr if time.time() - self.cmd_t < 0.5 else [0.0] * 8
        # domyslne sloty: FRD=1 FLD=2 RRD=3 RLD=4 FRU=5 FLU=6 RRU=7 RLU=8 (indeksy 0..7)
        frd, fld, rrd, rld, fru, flu, rru, rlu = t
        yaw_t = (fld + rld) - (frd + rrd)            # lewe poziome do przodu -> obrot w prawo? znak umowny: + = w lewo
        roll_t = (fru + rru) - (flu + rlu)           # prawe gorne w gore -> prawa burta w gore -> roll ujemny... umowny
        pitch_t = (fru + flu) - (rru + rlu)          # przednie w gore -> dziob w gore (+)
        heave = (fru + flu + rru + rlu) / 4.0
        k = 1.5
        self.rates[0] += (-k * roll_t - 1.0 * self.rates[0]) * dt
        self.rates[1] += (k * pitch_t - 1.0 * self.rates[1]) * dt
        self.rates[2] += (k * yaw_t - 0.5 * self.rates[2]) * dt
        self.vz += (0.8 * heave - 0.6 * self.vz) * dt
        self.depth = max(0.0, self.depth - self.vz * dt)
        # calkowanie kwaternionu (cialo FLU; roll + = prawa burta w dol = obrot ujemny wokol x)
        wx, wy, wz = -self.rates[0], -self.rates[1], self.rates[2]
        w, x, y, z = self.q
        dq = (1.0, 0.5 * dt * wx, 0.5 * dt * wy, 0.5 * dt * wz)
        q = [w * dq[0] - x * dq[1] - y * dq[2] - z * dq[3], w * dq[1] + x * dq[0] + y * dq[3] - z * dq[2],
             w * dq[2] - x * dq[3] + y * dq[0] + z * dq[1], w * dq[3] + x * dq[2] - y * dq[1] + z * dq[0]]
        n = math.sqrt(sum(v * v for v in q)); self.q = [v / n for v in q]
        # grawitacja w ciele: R^T * [0,0,g]
        w, x, y, z = self.q
        g = 9.81
        acc = [2 * (x * z - w * y) * g, 2 * (w * x + y * z) * g, (w * w - x * x - y * y + z * z) * g]
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "imu_link"
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = self.q
        # Pico wysyla osie CZUJNIKA; regulator mapuje "x,z,-y" -> wyslij tak, zeby po mapowaniu wyszlo cialo: sensor = R^T body
        # body = (sx, sz, -sy) => sx = bx, sz = by, sy = -bz
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = wx, -wz, wy
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = acc[0], -acc[2], acc[1]
        self.p_imu.publish(m)

    def pub_tlm(self):
        now = time.time()
        t = self.thr if now - self.cmd_t < 0.5 else [0.0] * 8
        m = EscTelemetry(); m.header.stamp = self.get_clock().now().to_msg()
        for i in range(8):
            target = abs(t[i]) * 14000.0
            self.erpm[i] += (target - self.erpm[i]) * 0.3
            v = t[i]
            m.dshot_value[i] = 0 if abs(v) < 0.005 else (int(1048 + v * 999) if v > 0 else int(48 - v * 999)) if self.mode_3d else int(48 + max(v, 0) * 1999)
            m.erpm[i] = int(self.erpm[i]); m.rpm[i] = int(self.erpm[i] / (self.poles / 2)); m.bidir_ok[i] = True
            m.bidir_ok_count[i] = int((now - self.t0) * 1000); m.kiss_temp_c[i] = 31; m.kiss_voltage_v[i] = 24.1
            m.kiss_current_a[i] = round(abs(v) * 6.0, 2); m.kiss_mah[i] = 120; m.kiss_erpm[i] = int(self.erpm[i]); m.kiss_age_ms[i] = 20
        m.esc_current_a[0] = sum(abs(x) for x in t[:4]) * 6.0; m.esc_current_a[1] = sum(abs(x) for x in t[4:]) * 6.0
        m.poles = self.poles; m.dshot_rate_kbit = 300; m.mode_3d = self.mode_3d
        m.failsafe = now - self.cmd_t > 0.5; m.agent_connected = True; m.loop_count = int((now - self.t0) * 1000)
        m.cmd_age_ms = int((now - self.cmd_t) * 1000) if self.cmd_t else 0xFFFFFFFF
        self.p_tlm.publish(m)
        self.p_leak.publish(UInt8(data=self.leak))
        self.p_pwr.publish(Bool(data=self.ssr))

    def pub_press(self):
        st = self.get_clock().now().to_msg()
        for i, p in enumerate((1013.25 + self.depth * 98.1, 1011.0)):
            fp = FluidPressure(); fp.header.stamp = st; fp.fluid_pressure = p * 100.0; self.p_pr[i].publish(fp)
            te = Temperature(); te.header.stamp = st; te.temperature = 18.5 + i; self.p_te[i].publish(te)
        self.p_depth.publish(Float32(data=self.depth))
        it = Temperature(); it.header.stamp = st; it.temperature = 29.0; self.p_imut.publish(it)

    def pub_bms(self):
        load = sum(abs(x) for x in self.thr) * 6.0
        self.soc = max(5.0, self.soc - load * 0.0005)
        b = BmsStatus(); b.header.stamp = self.get_clock().now().to_msg()
        b.online = True; b.voltage_v = 24.6 - (100 - self.soc) * 0.03; b.current_a = -load; b.soc_pct = self.soc
        b.remaining_ah = self.soc * 0.4; b.state = 2 if load > 0.5 else 0; b.charge_mos = b.discharge_mos = True
        b.cell_count = 6; b.temp_count = 2; b.cycles = 12; b.bms_life = 200
        for i in range(6):
            b.cell_voltage_v[i] = b.voltage_v / 6 + (i - 2.5) * 0.002
        b.temperature_c[0] = 22; b.temperature_c[1] = 23
        b.max_cell_mv = int(max(b.cell_voltage_v[:6]) * 1000); b.max_cell_no = 6; b.min_cell_mv = int(min(b.cell_voltage_v[:6]) * 1000); b.min_cell_no = 1
        b.max_temp_c = 23; b.max_temp_no = 2; b.min_temp_c = 22; b.min_temp_no = 1
        b.can_rx = int(time.time() - self.t0) * 9; b.can_tx = b.can_rx; b.age_ms = 30
        self.p_bms.publish(b)
        self.p_bmsi.publish(String(data=f"SIM up={int(time.time() - self.t0)}s ZALANIE={'TAK' if self.leak & 0xF else ('BYLO' if self.leak else 'nie')} SSR={'ON' if self.ssr else 'off'} rozl.MOS=ON ladowanie=nie | can ok | led={self.led}"))

    def pub_status(self):
        self.p_status.publish(String(data=f"SIM up={int(time.time() - self.t0)}s imu=OK(0x6A sim) ms5837=OK ms5803=OK | CUR1=12mV CUR2=9mV"))


def main():
    rclpy.init()
    n = SimPico()
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
