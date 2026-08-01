"""
EGE ROV — Web Tabanli Yer Istasyonu (GCS) Sunucusu.

Jetson uzerinde calisir:
  1. HTTP Server (Port 8000): gcs/ statik web dosyalarini sunar.
  2. MJPEG Stream (/video_feed): Kamera goruntusunu canlı HTTP akisi olarak verir.
  3. Telemetri & Komut API (/api/telemetry, /api/command, /api/pid): canli telemetri,
     PID ayari, gorev baslatma, abort ve teleop komutlarini yonetir.

Sifir harici kütüphane bağımlılığı (Python standart http.server + threading).
Tarayicida: http://192.168.1.10:8000/

DONANIMSIZ TEST:
    cd rov && python3 -m comms.web_server --demo
    -> http://localhost:8000/  (sahte telemetri uretir, PID paneli gercek calisir)
"""
import json
import math
import os
import random
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

from config import (GCS_WEB_PORT, VIDEO_QUALITY, VIDEO_FPS,
                    MOTOR_CHANNELS)

# Statik web arayuzu klasoru
GCS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "gcs"))

# Web arayuzunden ayarlanabilen PID'ler ve kabul edilen kazanc alanlari.
# HeadingController (kaskad) ic PID'i kp/ki/kd alir, ayrica kp_pos ve w_max
# dis katman parametreleridir.
_PID_NAMES = ("depth", "heading", "roll", "pitch")
_GAIN_FIELDS = ("kp", "ki", "kd", "ff", "out_limit", "i_limit",
                "d_tau", "deadzone", "kp_pos", "w_max")


def _safe_float(v):
    """None / NaN / sonsuz degerleri eler. Tarayicidan bos input gelirse
    parseFloat NaN uretir ve JSON'da null olur — o degeri yok sayariz."""
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return f


def _pid_snapshot(pid):
    """Bir denetleyicinin kazanclarini VE son adimdaki ic terimlerini dondurur.

    Web arayuzu bunlari canli cizer: hangi terim ne kadar katki veriyor,
    cikis doymus mu, hata ne kadar. Ayar yaparken kor kalmamak icin sart.
    """
    if pid is None:
        return None
    try:
        gains = pid.get_params()
    except Exception:
        gains = {}
    last = dict(getattr(pid, "last", {}) or {})

    # HeadingController kaskad: dis katman telemetrisi farkli alan adlari
    # kullaniyor (w_target/w_meas). Ic PID'in p/i/d terimlerini de ekle ki
    # arayuz tek bir sablonla cizebilsin.
    inner = getattr(pid, "rate", None)
    if inner is not None:
        inner_last = dict(getattr(inner, "last", {}) or {})
        for k in ("p", "i", "d", "ff", "sat", "dt"):
            last.setdefault(k, inner_last.get(k, 0.0))

    out = {
        "gains": {k: gains.get(k) for k in _GAIN_FIELDS if k in gains},
        "p": round(float(last.get("p", 0.0)), 4),
        "i": round(float(last.get("i", 0.0)), 4),
        "d": round(float(last.get("d", 0.0)), 4),
        "ff": round(float(last.get("ff", 0.0)), 4),
        "out": round(float(last.get("out", 0.0)), 4),
        "err": round(float(last.get("err", 0.0)), 4),
        "sat": int(last.get("sat", 0) or 0),
    }
    if "mode" in gains:
        out["mode"] = gains["mode"]
    if "w_target" in last:
        out["w_target"] = round(float(last["w_target"]), 3)
        out["w_meas"] = round(float(last.get("w_meas", 0.0)), 3)
    return out


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
        self.lock = threading.Lock()
        # Arayuz, gorev baslamadan once acilirsa bos ekran gormesin.
        self.last_telemetry = {
            "timestamp": 0.0, "state": "IDLE", "armed": False, "estop": False,
        }

    def _pid_objects(self):
        s = self.stabilizer
        if not s:
            return {}
        return {n: getattr(s, f"pid_{n}", None) for n in _PID_NAMES}

    def update_telemetry(self, state_name, extra_data=None):
        """Ana donguden telemetri verisini guncelle."""
        data = {
            "timestamp": round(time.monotonic(), 3),
            "state": state_name,
            "armed": self.thrusters.armed if self.thrusters else False,
            "estop": self.estop.triggered.is_set() if self.estop else False,
            "mission": self.active_mission_name,
        }

        if self.stabilizer:
            s = self.stabilizer
            ori = s.ori
            depth_sensor = s.depth
            data.update({
                "heading": round(getattr(s, "heading_deg", 0.0) or 0.0, 1),
                "target_heading": round(s.target_heading or 0.0, 1),
                "pitch": round(getattr(ori, "pitch", 0.0) or 0.0, 1),
                "roll": round(getattr(ori, "roll", 0.0) or 0.0, 1),
                "yaw_rate": round(getattr(ori, "yaw_rate", 0.0) or 0.0, 2),
                # SORUN 2/8: web thread'i sensoru OKUMUYOR. Eskiden buradaki
                # read_depth_m() 40 ms I2C bloklamasi yapiyordu — ustelik
                # kontrol dongusune paralel. Simdi stabilizer onbellegi okunur.
                "depth": round(getattr(s, "depth_m", 0.0) or 0.0, 2),
                "target_depth": round(s.target_depth or 0.0, 2),
                "pressure_mbar": round(getattr(depth_sensor, "pressure_mbar", 1013.25), 1),
                "temp_c": round(getattr(depth_sensor, "temp_c", 20.0), 1),
                "depth_error": round(
                    (s.target_depth or 0.0) - (getattr(s, "depth_m", 0.0) or 0.0), 3),
            })
            # Canli PID durumu: kazanclar + son adimdaki P/I/D terimleri.
            pids = {}
            for name, obj in self._pid_objects().items():
                snap = _pid_snapshot(obj)
                if snap is not None:
                    pids[name] = snap
            data["pid"] = pids

        if self.thrusters:
            data["thrusters"] = {
                ch: round(self.thrusters._current.get(ch, 0.0), 3)
                for ch in MOTOR_CHANNELS
            }

        if extra_data:
            data.update(extra_data)

        with self.lock:
            self.last_telemetry = data

    def get_telemetry_json(self):
        with self.lock:
            return json.dumps(self.last_telemetry)

    def get_pid_json(self):
        """Sayfa acilisinda kutulari GERCEK kazanclarla doldurmak icin."""
        pids = {}
        for name, obj in self._pid_objects().items():
            snap = _pid_snapshot(obj)
            if snap is not None:
                pids[name] = snap
        return json.dumps({"ok": True, "pid": pids})


g_ctx = GCSContext()


class GCSHTTPRequestHandler(SimpleHTTPRequestHandler):
    """GCS Web Arayuzu HTTP Istek Yoneticisi."""

    # HTTP/1.1 + Content-Length => kalici baglanti. 20 Hz yoklama artik
    # saniyede 20 yeni TCP baglantisi acmiyor.
    protocol_version = "HTTP/1.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=GCS_DIR, **kwargs)

    def log_message(self, format, *args):
        """Terminali doldurmamak icin log seviyesini sessize al."""
        pass

    # ------------------------------------------------------------- yardimci
    def _send_json(self, obj, status=200):
        payload = obj.encode() if isinstance(obj, str) else json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(payload)

    # ------------------------------------------------------------------ GET
    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/api/telemetry":
            self._send_json(g_ctx.get_telemetry_json())
            return

        if path == "/api/pid":
            self._send_json(g_ctx.get_pid_json())
            return

        if path == "/video_feed":
            self._stream_video()
            return

        return super().do_GET()

    # ----------------------------------------------------------------- POST
    def do_POST(self):
        path = urlparse(self.path).path

        if path == "/api/command":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length) if length else b"{}"
                msg = json.loads(body.decode() or "{}")
                response = self._handle_command(msg)
            except Exception as e:
                response = {"ok": False, "error": str(e)}
            self._send_json(response)
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
            axes = {}
            for k in ("surge", "yaw", "heave", "roll", "pitch"):
                axes[k] = _safe_float(msg.get(k)) or 0.0
            g_ctx.teleop_axes = axes
            return {"ok": True}

        if cmd == "teleop_off":
            g_ctx.teleop_axes = None
            return {"ok": True}

        # ── PID KAZANÇ AYARI
        if cmd == "set_pid":
            target = msg.get("pid_name")
            if target not in _PID_NAMES:
                return {"ok": False, "error": f"Bilinmeyen PID: {target}"}
            if not g_ctx.stabilizer:
                return {"ok": False, "error": "Stabilizer bulunamadı"}

            pid_obj = getattr(g_ctx.stabilizer, f"pid_{target}", None)
            if pid_obj is None:
                return {"ok": False, "error": f"pid_{target} yok"}

            # Sadece GERCEKTEN gonderilen sayisal alanlari uygula. Bos birakilan
            # kutu (NaN -> null) mevcut kazanci EZMEZ.
            kwargs = {}
            for field in _GAIN_FIELDS:
                val = _safe_float(msg.get(field))
                if val is not None:
                    kwargs[field] = val
            if not kwargs:
                return {"ok": False, "error": "Geçerli kazanç gönderilmedi"}

            # HeadingController kp_pos/w_max'i kendi alir, kalanini ic PID'e
            # gecirir. Duz PID'ler kp_pos/w_max'i tanimaz — ayikla.
            if not hasattr(pid_obj, "rate"):
                kwargs.pop("kp_pos", None)
                kwargs.pop("w_max", None)

            reset = bool(msg.get("reset", True))
            try:
                pid_obj.set_params(reset=reset, **kwargs)
            except TypeError as e:
                return {"ok": False, "error": f"Parametre reddedildi: {e}"}

            print(f"[GCS] PID {target} güncellendi: {kwargs}")
            return {"ok": True, "message": f"{target} PID güncellendi: "
                                           + ", ".join(f"{k}={v}" for k, v in kwargs.items()),
                    "gains": _pid_snapshot(pid_obj)["gains"]}

        # ── PID SIFIRLA (I birikimini temizle)
        if cmd == "reset_pid":
            target = msg.get("pid_name")
            pid_obj = getattr(g_ctx.stabilizer, f"pid_{target}", None) if g_ctx.stabilizer else None
            if pid_obj is None:
                return {"ok": False, "error": f"pid_{target} yok"}
            pid_obj.reset()
            return {"ok": True, "message": f"{target} PID sıfırlandı (I=0)"}

        # ── HEDEF AYARI (canli PID denemesi icin: basamak cevabi)
        if cmd == "set_target":
            if not g_ctx.stabilizer:
                return {"ok": False, "error": "Stabilizer bulunamadı"}
            d = _safe_float(msg.get("depth"))
            h = _safe_float(msg.get("heading"))
            if d is None and h is None:
                return {"ok": False, "error": "Hedef verilmedi"}
            g_ctx.stabilizer.set_targets(depth_m=d, heading_deg=h)
            return {"ok": True, "message": f"Hedef güncellendi (derinlik={d}, heading={h})"}

        # ── MINI ROV VİNÇ KONTROLÜ
        if cmd in ("winch_deploy", "winch_retract"):
            if not g_ctx.thrusters:
                return {"ok": False, "error": "Thrusters yok"}
            from hal.winch import Winch
            w = Winch(g_ctx.thrusters.backend)
            fn = w.deploy if cmd == "winch_deploy" else w.retract
            threading.Thread(target=fn, daemon=True).start()
            return {"ok": True, "message": "Vinç bırakma başladı" if cmd == "winch_deploy"
                                           else "Vinç çekme başladı"}

        # ── GÖREV1 MINI ROV GERİ GELDİ BİLDİRİMİ
        if cmd == "minrov_back":
            if g_ctx.active_mission_obj and hasattr(g_ctx.active_mission_obj, "signal_minrov_back"):
                g_ctx.active_mission_obj.signal_minrov_back()
                return {"ok": True, "message": "Mini ROV geri çekme tetiklendi"}
            return {"ok": False, "error": "Aktif görev bu komutu desteklemiyor"}

        return {"ok": False, "error": f"Bilinmeyen komut: {cmd}"}

    # ---------------------------------------------------------------- video
    def _stream_video(self):
        """Kamera görüntüsünü MJPEG HTTP multipart stream olarak sunar.

        NOT: Bu istek SONSUZA KADAR acik kalir. Sunucu ThreadingHTTPServer
        oldugu icin sadece BU thread'i mesgul eder; /api/telemetry yoklamalari
        etkilenmez. (Eski kod tek thread'li HTTPServer kullaniyordu ve video
        akisi baslar baslamaz tum arayuz donuyordu.)
        """
        if not _CV2_OK:
            self.send_error(503, "OpenCV yok - video akisi kapali")
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True

        interval = 1.0 / max(1, VIDEO_FPS)
        while True:
            if not g_ctx.camera:
                time.sleep(0.5)
                continue
            frame = g_ctx.camera.read()
            if frame is None:
                time.sleep(interval)
                continue

            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, VIDEO_QUALITY])
            if not ok:
                continue
            jpg = buf.tobytes()

            try:
                # Parca basliklarini ELLE yaz. send_header/end_headers burada
                # kullanilamaz: HTTP/1.1'de ana yanit basliklarina karisir.
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (ConnectionResetError, BrokenPipeError, OSError):
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
            # ThreadingHTTPServer SART: /video_feed sonsuz donguye girer.
            # Tek thread'li HTTPServer ile telemetri yoklamalari asla cevap almaz.
            self._server = ThreadingHTTPServer(("0.0.0.0", self.port),
                                               GCSHTTPRequestHandler)
            self._server.daemon_threads = True
            self._server.allow_reuse_address = True
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
            self._server = None

    def update_telemetry(self, state_name, extra_data=None):
        g_ctx.update_telemetry(state_name, extra_data)

    def get_teleop(self):
        return g_ctx.teleop_axes

    def set_active_mission(self, name, mission_obj=None):
        g_ctx.active_mission_name = name
        g_ctx.active_mission_obj = mission_obj


# ══════════════════════════════════════════════════════════════════════════
#  DONANIMSIZ DEMO MODU — arayuzu ve PID panelini masaustunde test etmek icin
# ══════════════════════════════════════════════════════════════════════════
def _run_demo(port=GCS_WEB_PORT):
    """Sahte bir ROV simule eder ve gercek Stabilizer/PID nesnelerini kullanir.

    PID paneli GERCEKTEN calisir: kazanci degistirdigin an simulasyondaki
    cevap degisir. Havuza girmeden ayar sezgisi kazanmak icin.
    """
    from control.stabilizer import Stabilizer
    from sensors.state import RovState

    class _FakeOri:
        """Stabilizer state=RovState ile calisiyor; bu nesne sadece
        telemetrideki roll/pitch/yaw_rate okumalari icin duruyor."""

        def __init__(self):
            self.heading = 0.0
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw_rate = 0.0
            self.gyro = (0.0, 0.0, 0.0)

        def update(self):
            pass

    class _FakeDepth:
        def __init__(self):
            self.pressure_mbar = 1013.25
            self.temp_c = 21.0
            self.surface_pressure_mbar = 1013.25
            self._d = 0.0

        def read_depth_m(self):
            return self._d

    class _FakeBackend:
        """Vinc komutlari demo modda da hatasiz calissin diye."""

        def set_us(self, channel, us):
            pass

    class _FakeThrusters:
        def __init__(self):
            self.armed = False
            self.backend = _FakeBackend()
            self._current = {ch: 0.0 for ch in MOTOR_CHANNELS}

        def arm(self):
            self.armed = True

        def stop(self):
            self.armed = False
            self._current = {ch: 0.0 for ch in MOTOR_CHANNELS}

        def command(self, mixed):
            if not self.armed:
                return
            for k, v in (mixed or {}).items():
                if k in self._current:
                    self._current[k] = max(-1.0, min(1.0, float(v)))

    ori, depth, thr = _FakeOri(), _FakeDepth(), _FakeThrusters()
    state = RovState()
    stab = Stabilizer(ori, depth, state=state)
    stab.set_targets(depth_m=1.5, heading_deg=90.0)

    gcs = WebGCS(port)
    gcs.start(thrusters=thr, stabilizer=stab)
    print("[DEMO] Sahte telemetri üretiliyor. Ctrl+C ile çık.")
    print("[DEMO] PID kutularını değiştirip 'GÜNCELLE'ye bas — cevap anında değişir.")

    # Basit 1. derece arac modeli: itki -> ivme -> hiz -> konum
    depth_pos, depth_v = 0.0, 0.0
    heading, yaw_v = 0.0, 0.0
    dt = 1.0 / 25.0
    t0 = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            t = now - t0

            roll = 3.0 * math.sin(t * 0.7) + random.gauss(0, 0.15)
            pitch = 2.0 * math.sin(t * 0.5) + random.gauss(0, 0.15)

            # Sensor katmanini besle (gercek sistemdeki SensorHub'in yerine)
            state.set_imu(heading, roll, pitch, yaw_v,
                          (roll * -0.5, pitch * -0.5, yaw_v), now, 100.0)
            state.set_depth(depth_pos, depth_v, now, 20.0)

            # Telemetride okunan ori/depth alanlarini da guncel tut
            ori.heading, ori.roll, ori.pitch = heading, roll, pitch
            ori.yaw_rate = yaw_v
            depth._d = depth_pos
            depth.pressure_mbar = 1013.25 + depth_pos * 98.1

            out = stab.compute(now=now)
            heave = out.get("heave", 0.0)
            yaw = out.get("yaw", 0.0)

            teleop = gcs.get_teleop()
            if teleop:
                heave = teleop.get("heave", 0.0)
                yaw = teleop.get("yaw", 0.0)

            # arac dinamigi (surtunmeli 1. derece + kucuk akinti bozucusu)
            drift = 0.05 * math.sin(t * 0.23)
            depth_v += (heave * 1.2 - depth_v * 1.8 + drift) * dt
            depth_pos = max(0.0, depth_pos + depth_v * dt)
            yaw_v += (yaw * 90.0 - yaw_v * 2.5) * dt
            heading = (heading + yaw_v * dt) % 360.0

            if thr.armed:
                for i, ch in enumerate(MOTOR_CHANNELS):
                    thr._current[ch] = round(
                        max(-1.0, min(1.0, 0.35 * math.sin(t * 1.3 + i) + heave * 0.5)), 3)

            gcs.update_telemetry("TELEOP" if teleop else
                                 ("ARMED" if thr.armed else "HOLD"))
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[DEMO] Kapatiliyor...")
    finally:
        gcs.stop()


if __name__ == "__main__":
    if "--demo" in sys.argv:
        p = GCS_WEB_PORT
        for i, a in enumerate(sys.argv):
            if a == "--port" and i + 1 < len(sys.argv):
                p = int(sys.argv[i + 1])
        _run_demo(p)
    else:
        print(__doc__)
        print("Kullanim: python3 -m comms.web_server --demo [--port 8000]")
