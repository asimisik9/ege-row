"""
TCP JSON komut sunucusu — Jetson uzerinde calisir.

Yer istasyonu (laptop) bu sunucuya baglanarak:
  - Gorev baslatabilir / iptal edebilir
  - Durum sorgulayabilir (heading, derinlik, state, batarya)
  - Teleoperation komutu gonderebilir (Manuel Mini ROV fazinda)

Kullanim:
    server = CommsServer(mission_handle, thrusters, stabilizer)
    server.start()    # arka planda thread
    ...
    server.stop()

Protokol (JSON satirlari, newline ayirici):
  Komut  -> { "cmd": "start",  "mission": "video" }
  Komut  -> { "cmd": "abort" }
  Komut  -> { "cmd": "teleop", "surge": 0.3, "yaw": -0.1, "heave": 0.0 }
  Komut  -> { "cmd": "state" }
  Yanit  <- { "ok": true, "state": "STRAIGHT1", "heading": 45.2,
              "depth": 0.62, "roll": 0.1, "pitch": -0.2 }
"""
import json
import socket
import threading
import time

from config import COMMS_PORT


class CommsServer:
    def __init__(self, get_state_fn, thrusters, stabilizer):
        """
        get_state_fn : () -> dict  — mevcut gorev durumu sozlugu
        thrusters    : Thrusters  — teleop komutlari icin
        stabilizer   : Stabilizer — heading/depth/roll/pitch okumak icin
        """
        self._get_state = get_state_fn
        self.thr  = thrusters
        self.stab = stabilizer
        self._teleop_axes  = None   # None = gorev kontrolu; dict = teleop aktif
        self._teleop_lock  = threading.Lock()
        self._thread = None
        self._running = False
        self._sock = None

    # ---------------------------------------------------------------- public
    def start(self):
        """Sunucu thread'ini baslat."""
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        print(f"[COMMS] TCP sunucu port {COMMS_PORT} dinleniyor.")

    def stop(self):
        """Sunucuyu kapat."""
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def get_teleop(self):
        """
        Teleop modundaysa son eksen komutlarini dondurur, degilse None.
        Ana gorev dongusu bu degerle mixer'i cagirir.
        """
        with self._teleop_lock:
            return self._teleop_axes

    def clear_teleop(self):
        """Teleop modundan cik (gorev kontrolune don)."""
        with self._teleop_lock:
            self._teleop_axes = None

    # ---------------------------------------------------------------- private
    def _serve(self):
        """Baglanti kabul dongusU (tek istemci destekler)."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", COMMS_PORT))
        self._sock.listen(1)
        self._sock.settimeout(1.0)

        while self._running:
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"[COMMS] Baglandi: {addr}")
            try:
                self._handle(conn)
            except Exception as e:
                print(f"[COMMS] Hata: {e}")
            finally:
                conn.close()
                print("[COMMS] Baglanti kapandi.")

    def _handle(self, conn):
        """Bir istemci baglantisindan satir bazli JSON okur."""
        buf = b""
        conn.settimeout(0.5)
        while self._running:
            try:
                data = conn.recv(1024)
            except socket.timeout:
                continue
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    msg = json.loads(line.decode())
                    reply = self._dispatch(msg)
                    conn.sendall((json.dumps(reply) + "\n").encode())
                except json.JSONDecodeError:
                    conn.sendall(b'{"ok":false,"error":"invalid json"}\n')

    def _dispatch(self, msg):
        """JSON mesajini isleme al, yanit sozlugu dondur."""
        cmd = msg.get("cmd", "")

        if cmd == "state":
            ori = self.stab.ori
            return {
                "ok": True,
                **self._get_state(),
                "heading": round(ori.heading or 0, 1),
                "depth":   round(self.stab.depth.read_depth_m(), 2),
                "roll":    round(ori.roll, 1),
                "pitch":   round(ori.pitch, 1),
            }

        if cmd == "abort":
            self.thr.stop()
            self.clear_teleop()
            return {"ok": True, "msg": "abort gonderildi"}

        if cmd == "teleop":
            axes = {
                "surge": float(msg.get("surge", 0)),
                "yaw":   float(msg.get("yaw",   0)),
                "heave": float(msg.get("heave", 0)),
                "roll":  float(msg.get("roll",  0)),
                "pitch": float(msg.get("pitch", 0)),
            }
            with self._teleop_lock:
                self._teleop_axes = axes
            return {"ok": True, "msg": "teleop alindi"}

        if cmd == "teleop_off":
            self.clear_teleop()
            return {"ok": True, "msg": "gorev kontrolune donuldu"}

        return {"ok": False, "error": f"bilinmeyen komut: {cmd}"}
