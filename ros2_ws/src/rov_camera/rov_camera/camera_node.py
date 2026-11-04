#!/usr/bin/env python3
"""Kamera IMX708 na Jetsonie (Argus przez nvarguscamerasrc, sprzetowy JPEG nvjpegenc).

- strumien MJPEG dla GUI:  http://<jetson>:8089/cam.mjpg   (snapshot: /cam.jpg, status: /status.json)
- ROS 2: /rov/camera/compressed (sensor_msgs/CompressedImage, throttle ros_fps), /rov/camera/info (String JSON, 1 Hz)
- nagrywanie: /rov/camera/record (std_srvs/SetBool) -> ~/nowy_rov/recordings/rov_YYYYmmdd_HHMMSS.mjpeg (+ .csv z czasami klatek)
- snapshot: /rov/camera/snapshot (std_srvs/Trigger) -> ~/nowy_rov/recordings/snap_*.jpg
- ostrosc: /rov/camera/focus (std_msgs/UInt16 0..1023, VCM DW9807 po I2C), /rov/camera/autofocus (Trigger, CDAF)

Orin Nano nie ma enkodera H.264, stad MJPEG. Argus bywa kruchy: po 3 nieudanych startach z rzedu proces konczy sie
kodem 1 i systemd/launch (respawn) go podnosi; jesli skonfigurowano sudoers, restartuje tez nvargus-daemon.
"""
import fcntl
import json
import os
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import String, UInt16  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402

try:
    import cv2
    import numpy as np
except Exception:  # autofocus wtedy niedostepny
    cv2 = None

I2C_SLAVE = 0x0706   # I2C_SLAVE_FORCE: adres 0x0C moze byc zajety przez sterownik kernela


class VCM:
    """Naped soczewki DW9807 (0x0C) modulu IMX708. Zasilany tylko gdy sensor streamuje."""

    def __init__(self, bus):
        self.dev = f"/dev/i2c-{bus}"
        self.pos = 500
        self.ok = os.path.exists(self.dev)

    def _write(self, reg, val):
        fd = os.open(self.dev, os.O_RDWR)
        try:
            fcntl.ioctl(fd, I2C_SLAVE, 0x0C)
            os.write(fd, bytes([reg, val & 0xFF]))
        finally:
            os.close(fd)

    def set(self, pos):
        pos = max(0, min(1023, int(pos)))
        try:
            self._write(0x02, 0x00)          # power on / normal
            self._write(0x03, pos >> 8)
            self._write(0x04, pos & 0xFF)
            self.pos = pos
            return True
        except OSError:
            return False


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")
        p = self.declare_parameter
        p("sensor_id", 0)            # Argus: 0 = CAM1 (10-001a) przy jednej kamerze
        p("sensor_mode", 1)          # IMX708 Arducam: 0=4608x2592@14, 1=2304x1296@55, 2=1536x864@90
        p("width", 1280); p("height", 720); p("fps", 30)
        p("quality", 80)
        p("http_port", 8089)
        p("ros_fps", 5.0)            # ile klatek/s idzie do ROS (CompressedImage); 0 = wcale
        p("vcm_bus", 10)             # i2c-10 = CAM1, i2c-9 = CAM0
        p("flip", 0)                 # nvvidconv flip-method: 0 brak, 2 = obrot 180
        p("rec_dir", os.path.expanduser("~/nowy_rov/recordings"))
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.cfg = {k: g(k) for k in ("sensor_id", "sensor_mode", "width", "height", "fps", "quality", "http_port", "ros_fps", "vcm_bus", "flip", "rec_dir")}
        os.makedirs(self.cfg["rec_dir"], exist_ok=True)

        self.frame = None            # ostatni JPEG (bytes)
        self.frame_t = 0.0
        self.frame_n = 0
        self.cond = threading.Condition()
        self.fps_meas = 0.0
        self.fail = 0
        self.rec = None              # (plik mjpeg, plik csv, t0, n)
        self.vcm = VCM(self.cfg["vcm_bus"])

        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pub_img = self.create_publisher(CompressedImage, "/rov/camera/compressed", be)
        self.pub_info = self.create_publisher(String, "/rov/camera/info", 10)
        self.create_subscription(UInt16, "/rov/camera/focus", lambda m: self.vcm.set(m.data), 10)
        self.create_service(SetBool, "/rov/camera/record", self.srv_record)
        self.create_service(Trigger, "/rov/camera/snapshot", self.srv_snapshot)
        self.create_service(Trigger, "/rov/camera/autofocus", self.srv_autofocus)
        self.create_timer(1.0, self.publish_info)
        if self.cfg["ros_fps"] > 0:
            self.create_timer(1.0 / self.cfg["ros_fps"], self.publish_frame)

        Gst.init(None)
        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()
        self.start_pipeline()
        self.http = ThreadingHTTPServer(("0.0.0.0", self.cfg["http_port"]), self.make_handler())
        threading.Thread(target=self.http.serve_forever, daemon=True).start()
        self.get_logger().info(f"MJPEG: http://0.0.0.0:{self.cfg['http_port']}/cam.mjpg  (sensor {self.cfg['sensor_id']}, {self.cfg['width']}x{self.cfg['height']}@{self.cfg['fps']})")

    # ---- GStreamer ----
    def start_pipeline(self):
        c = self.cfg
        desc = (f"nvarguscamerasrc sensor-id={c['sensor_id']} sensor-mode={c['sensor_mode']} name=src ! "
                f"video/x-raw(memory:NVMM),framerate={c['fps']}/1 ! "
                f"nvvidconv flip-method={c['flip']} ! video/x-raw(memory:NVMM),width={c['width']},height={c['height']} ! "
                f"nvjpegenc quality={c['quality']} ! "
                f"appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false")
        self.pipe = Gst.parse_launch(desc)
        sink = self.pipe.get_by_name("sink")
        sink.connect("new-sample", self.on_sample)
        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_gst_error)
        bus.connect("message::eos", lambda *_: self.on_gst_error(None, None))
        self.pipe.set_state(Gst.State.PLAYING)
        self.start_t = time.time()
        self.create_timer(5.0, self.check_alive) if not hasattr(self, "_alive_timer") else None
        self._alive_timer = True

    def on_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        now = time.time()
        # Argus nie zawsze honoruje framerate w caps (tryb 55 fps) - ograniczamy tutaj do cfg fps
        if self.frame_t and now - self.frame_t < 0.92 / max(self.cfg["fps"], 1):
            buf.unmap(mi)
            return Gst.FlowReturn.OK
        data = bytes(mi.data)
        buf.unmap(mi)
        with self.cond:
            if self.frame_t:
                self.fps_meas = 0.9 * self.fps_meas + 0.1 / max(now - self.frame_t, 1e-3)
            self.frame, self.frame_t, self.frame_n = data, now, self.frame_n + 1
            self.cond.notify_all()
        if self.rec:
            try:
                f, csvf, t0, n = self.rec
                f.write(data); csvf.write(f"{n},{now:.3f},{len(data)}\n")
                self.rec = (f, csvf, t0, n + 1)
            except Exception as e:
                self.get_logger().error(f"nagrywanie: {e}"); self.stop_record()
        return Gst.FlowReturn.OK

    def on_gst_error(self, bus, msg):
        err = msg.parse_error()[0].message if msg else "EOS"
        self.get_logger().error(f"GStreamer: {err}")
        self.restart_pipeline()

    def check_alive(self):
        if time.time() - max(self.frame_t, self.start_t) > 5.0:
            self.get_logger().warning("brak klatek > 5 s - restart pipeline")
            self.restart_pipeline()

    def restart_pipeline(self):
        self.fail += 1
        try:
            self.pipe.set_state(Gst.State.NULL)
        except Exception:
            pass
        if self.fail >= 3:
            self.get_logger().error("3 nieudane starty Argus (kamera odpieta?) - restart nvargus-daemon (jesli sudo), pauza 20 s i wyjscie (respawn)")
            subprocess.run(["sudo", "-n", "systemctl", "restart", "nvargus-daemon"], timeout=20, capture_output=True)
            time.sleep(20)       # nie spamuj logu i Argusa, gdy kamery fizycznie nie ma
            os._exit(1)
        time.sleep(1.0)
        self.start_pipeline()

    # ---- ROS ----
    def publish_frame(self):
        if self.frame is None:
            return
        m = CompressedImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "camera"
        m.format = "jpeg"
        m.data = self.frame
        self.pub_img.publish(m)

    def publish_info(self):
        if self.frame_t and time.time() - self.frame_t < 2:
            self.fail = 0
        st = {"fps": round(self.fps_meas, 1), "frames": self.frame_n, "alive": time.time() - self.frame_t < 2 if self.frame_t else False,
              "size": len(self.frame) if self.frame else 0, "recording": bool(self.rec), "rec_frames": self.rec[3] if self.rec else 0,
              "focus": self.vcm.pos, "vcm": self.vcm.ok, "w": self.cfg["width"], "h": self.cfg["height"], "port": self.cfg["http_port"]}
        self.pub_info.publish(String(data=json.dumps(st)))

    def srv_record(self, req, res):
        if req.data and not self.rec:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            base = os.path.join(self.cfg["rec_dir"], f"rov_{ts}")
            self.rec = (open(base + ".mjpeg", "wb"), open(base + ".csv", "w"), time.time(), 0)
            self.rec[1].write("frame,unix_time,bytes\n")
            res.success, res.message = True, f"nagrywam -> {base}.mjpeg"
        elif not req.data and self.rec:
            n = self.rec[3]; self.stop_record()
            res.success, res.message = True, f"zatrzymano, {n} klatek"
        else:
            res.success, res.message = True, "bez zmian"
        self.get_logger().info(res.message)
        return res

    def stop_record(self):
        if self.rec:
            f, csvf, _, _ = self.rec
            self.rec = None
            f.close(); csvf.close()

    def srv_snapshot(self, req, res):
        if self.frame is None:
            res.success, res.message = False, "brak klatki"; return res
        p = os.path.join(self.cfg["rec_dir"], f"snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
        open(p, "wb").write(self.frame)
        res.success, res.message = True, p
        return res

    # ---- autofocus (CDAF, Laplasjan na srodku kadru) ----
    def sharpness(self):
        with self.cond:
            self.cond.wait(timeout=0.5)
            data = self.frame
        if data is None or cv2 is None:
            return 0.0
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        h, w = img.shape
        roi = img[h // 3: 2 * h // 3, w // 3: 2 * w // 3]
        return float(cv2.Laplacian(roi, cv2.CV_64F).var())

    def srv_autofocus(self, req, res):
        if cv2 is None or not self.vcm.ok:
            res.success, res.message = False, "AF niedostepny (brak cv2 albo VCM)"; return res
        best_pos, best_s = self.vcm.pos, -1.0
        for step, rng in ((64, range(0, 1024, 64)), (8, None)):
            if rng is None:
                rng = range(max(0, best_pos - 64), min(1023, best_pos + 64) + 1, 8)
            for pos in rng:
                self.vcm.set(pos); time.sleep(0.12)
                s = self.sharpness()
                if s > best_s:
                    best_s, best_pos = s, pos
        self.vcm.set(best_pos)
        res.success, res.message = True, f"AF: pozycja {best_pos}, ostrosc {best_s:.0f}"
        self.get_logger().info(res.message)
        return res

    # ---- HTTP MJPEG ----
    def make_handler(node_self):
        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):  # cisza w logu
                pass

            def do_GET(self):
                if self.path.startswith("/cam.mjpg"):
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-cache"); self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    last = -1
                    try:
                        while True:
                            with node_self.cond:
                                node_self.cond.wait(timeout=2.0)
                                if node_self.frame_n == last:
                                    continue
                                data, last = node_self.frame, node_self.frame_n
                            if data is None:
                                continue
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(data))
                            self.wfile.write(data); self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                elif self.path.startswith("/cam.jpg"):
                    data = node_self.frame
                    if data is None:
                        self.send_response(503); self.end_headers(); return
                    self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.send_header("Content-Length", str(len(data)))
                    self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers(); self.wfile.write(data)
                elif self.path.startswith("/status.json"):
                    body = json.dumps({"fps": round(node_self.fps_meas, 1), "frames": node_self.frame_n, "focus": node_self.vcm.pos}).encode()
                    self.send_response(200); self.send_header("Content-Type", "application/json"); self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers(); self.wfile.write(body)
                else:
                    self.send_response(404); self.end_headers()
        return H


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_record()
        try:
            node.pipe.set_state(Gst.State.NULL)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
