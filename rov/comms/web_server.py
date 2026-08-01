"""
EGE ROV — Web Tabanli Yer Istasyonu (GCS) Sunucusu.

Jetson uzerinde calisir:
  1. HTTP Server (Port 8000): gcs/ statik web dosyalarini sunar.
  2. MJPEG Stream (/video_feed): Kamera goruntusunu canlı HTTP akisi olarak verir.
  3. Telemetri & Komut API (/api/telemetry, /api/command): 50Hz telemetri, PID ayari,
     gorev baslatma, abort ve teleop komutlarini yonetir.

Sifir harici kütüphane bağımlılığı (Python standart http.server + threading).
Tarayicida: http://192.168.1.10:8000/
"""
import json
import os
import socket
import struct
import threading
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

from config import (GCS_WEB_PORT, VIDEO_QUALITY, VIDEO_FPS,
                    MOTOR_CHANNELS, MOTOR_DIRECTION, THRUST_LIMIT)

# Statik web arayuzu klasoru
GCS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "gcs"))


class GCSContext:
    """Sistem bilesenlerinin GCS sunucusu ile paylasildigi ortak context."""
    def __init__(self):
        self.thrusters = None
        self.stabilizer = None
        self.camera = None
        self.estop = None
        self.active_mission_name = "IDLE"
        self.active_mission_obj = None
        self.teleop_axes = None
        self.last_telemetry = {}
        self.lock = threading.Lock()

    def update_telemetry(self, state_name, extra_data=None):
        """Ana donguden telemetri verisini guncelle."""
        with self.lock:
            data = {
                "timestamp": round(time.monotonic(), 3),
                "state": state_name,
                "armed": self.thrusters.armed if self.thrusters else False,
                "estop": self.estop.triggered.is_set() if self.estop else False,
            }
            if self.stabilizer:
                ori = self.stabilizer.ori
                depth = self.stabilizer.depth
                data.update({
                    "heading": round(ori.heading or 0.0, 1),
                    "target_heading": round(self.stabilizer.target_heading or 0.0, 1),
                    "pitch": round(ori.pitch, 1),
                    "roll": round(ori.roll, 1),
                    "yaw_rate": round(ori.yaw_rate, 2),
                    # SORUN 2/8: web thread'i ARTIK SENSORU OKUMUYOR.
                    # Eskiden buradaki read_depth_m() 40 ms I2C bloklamasi
                    # yapiyordu — ustelik kontrol dongusune paralel olarak.
                    # Simdi stabilizer'in onbellegindeki taze deger okunuyor.
                    "depth": round(getattr(self.stabilizer, "depth_m",
                                           0.0), 2),
                    "target_depth": round(self.stabilizer.target_depth or 0.0, 2),
                    "pressure_mbar": round(getattr(depth, "pressure_mbar", 1013.25), 1),
                    "temp_c": round(getattr(depth, "temp_c", 20.0), 1),
                })
                # PID kazançları
                data["pid_gains"] = {
                    "depth": self.stabilizer.pid_depth.get_params(),
                    "heading": self.stabilizer.pid_heading.get_params(),
                    "roll": self.stabilizer.pid_roll.get_params(),
                    "pitch": self.stabilizer.pid_pitch.get_params(),
                }

            if self.thrusters:
                data["thrusters"] = {
                    ch: round(self.thrusters._current.get(ch, 0.0), 2)
                    for ch in MOTOR_CHANNELS
                }

            if extra_data:
                data.update(extra_data)

            self.last_telemetry = data

    def get_telemetry_json(self):
        with self.lock:
            return json.dumps(self.last_telemetry)


g_ctx = GCSContext()


class GCSHTTPRequestHandler(SimpleHTTPRequestHandler):
    """GCS Web Arayuzu HTTP Istek Yoneticisi."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=GCS_DIR, **kwargs)

    def log_message(self, format, *args):
        """Terminali doldurmamak icin log seviyesini sessize al."""
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # ── 1. Telemetri API
        if path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(g_ctx.get_telemetry_json().encode())
            return

        # ── 2. Canli MJPEG Video Akisi
        if path == "/video_feed":
            self._stream_video()
            return

        # ── 3. Statik Web Dosyalari (index.html, css, js)
        return super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/command":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                msg = json.loads(body.decode())
                response = self._handle_command(msg)
            except Exception as e:
                response = {"ok": False, "error": str(e)}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())
            return

        self.send_error(404, "Endpoint Bulunamadi")

    def _handle_command(self, msg):
        cmd = msg.get("cmd", "")

        # ── ACİL DURDURMA / STOP
        if cmd == "abort":
            if g_ctx.thrusters:
                g_ctx.thrusters.stop()
            if g_ctx.active_mission_obj and hasattr(g_ctx.active_mission_obj, "abort"):
                g_ctx.active_mission_obj.abort()
            g_ctx.teleop_axes = None
            print("[GCS] Web üzerinden ACİL DURDURMA çağrıldı!")
            return {"ok": True, "message": "ACİL DURDURMA UYGULANDI"}

        # ── ARM / DISARM
        if cmd == "arm":
            if g_ctx.thrusters:
                g_ctx.thrusters.arm()
            return {"ok": True, "message": "Motorlar Arm Edildi"}

        if cmd == "disarm":
            if g_ctx.thrusters:
                g_ctx.thrusters.stop()
            return {"ok": True, "message": "Motorlar Disarm Edildi"}

        # ── TELEOP DRIVING
        if cmd == "teleop":
            g_ctx.teleop_axes = {
                "surge": float(msg.get("surge", 0.0)),
                "yaw": float(msg.get("yaw", 0.0)),
                "heave": float(msg.get("heave", 0.0)),
                "roll": float(msg.get("roll", 0.0)),
                "pitch": float(msg.get("pitch", 0.0)),
            }
            return {"ok": True}

        if cmd == "teleop_off":
            g_ctx.teleop_axes = None
            return {"ok": True}

        # ── PID KAZANÇ AYARI
        if cmd == "set_pid":
            target_pid = msg.get("pid_name")  # depth, heading, roll, pitch
            kp = msg.get("kp")
            ki = msg.get("ki")
            kd = msg.get("kd")
            if g_ctx.stabilizer:
                pid_obj = getattr(g_ctx.stabilizer, f"pid_{target_pid}", None)
                if pid_obj:
                    pid_obj.set_params(kp=kp, ki=ki, kd=kd)
                    print(f"[GCS] PID {target_pid} güncellendi: Kp={kp}, Ki={ki}, Kd={kd}")
                    return {"ok": True, "message": f"{target_pid} PID güncellendi"}
            return {"ok": False, "error": "Stabilizer bulunamadı"}

        # ── MINI ROV VİNÇ KONTROLÜ
        if cmd == "winch_deploy":
            if g_ctx.thrusters:
                from hal.winch import Winch
                w = Winch(g_ctx.thrusters.backend)
                threading.Thread(target=w.deploy, daemon=True).start()
                return {"ok": True, "message": "Vinç bırakma başladı"}
            return {"ok": False, "error": "Thrusters yok"}

        if cmd == "winch_retract":
            if g_ctx.thrusters:
                from hal.winch import Winch
                w = Winch(g_ctx.thrusters.backend)
                threading.Thread(target=w.retract, daemon=True).start()
                return {"ok": True, "message": "Vinç çekme başladı"}
            return {"ok": False, "error": "Thrusters yok"}

        # ── GÖREV1 MINI ROV GERİ GELDİ BİLDİRİMİ
        if cmd == "minrov_back":
            if g_ctx.active_mission_obj and hasattr(g_ctx.active_mission_obj, "signal_minrov_back"):
                g_ctx.active_mission_obj.signal_minrov_back()
                return {"ok": True, "message": "Mini ROV geri çekme tetiklendi"}

        return {"ok": False, "error": f"Bilinmeyen komut: {cmd}"}

    def _stream_video(self):
        """Kamera görüntüsünü MJPEG HTTP multipart stream olarak sunar."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, private")
        self.end_headers()

        interval = 1.0 / VIDEO_FPS
        while True:
            if not g_ctx.camera:
                time.sleep(0.5)
                continue
            frame = g_ctx.camera.read()
            if frame is None:
                time.sleep(interval)
                continue
            if _CV2_OK:
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, VIDEO_QUALITY])
                if not ok:
                    continue
                jpg_bytes = buf.tobytes()
            else:
                break

            try:
                self.wfile.write(b"--frame\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg_bytes)))
                self.end_headers()
                self.wfile.write(jpg_bytes)
                self.wfile.write(b"\r\n")
            except (ConnectionResetError, BrokenPipeError):
                break
            time.sleep(interval)


class WebGCS:
    """GCS Web Sunucu Yöneticisi."""

    def __init__(self, port=GCS_WEB_PORT):
        self.port = port
        self._server = None
        self._thread = None

    def start(self, thrusters=None, stabilizer=None, camera=None, estop=None):
        g_ctx.thrusters = thrusters
        g_ctx.stabilizer = stabilizer
        g_ctx.camera = camera
        g_ctx.estop = estop

        os.makedirs(GCS_DIR, exist_ok=True)

        try:
            self._server = ThreadingHTTPServer(("0.0.0.0", self.port), GCSHTTPRequestHandler)
        except OSError as e:
            print(f"[GCS] Port {self.port} açılamadı: {e}")
            return

        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        print(f"[GCS] Yer İstasyonu Web Arayüzü Hazır → http://localhost:{self.port}/")

    def stop(self):
        if self._server:
            self._server.shutdown()
            self._server.server_close()

    def update_telemetry(self, state_name, extra_data=None):
        g_ctx.update_telemetry(state_name, extra_data)

    def get_teleop(self):
        return g_ctx.teleop_axes

    def set_active_mission(self, name, mission_obj=None):
        g_ctx.active_mission_name = name
        g_ctx.active_mission_obj = mission_obj
