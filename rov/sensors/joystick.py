"""
PS3 / Fiziksel Joystick TCP İstemcisi.

joystick_server.py (port 12345) sunucusuna baglanip PS3 joystick verilerini okur.
Thread-guvenli: start() arka planda baglantiyi ve JSON satirlarini okur.

Kullanim:
    js = JoystickClient()
    js.start()
    axes = js.read_axes()   # {"surge": 0.5, "yaw": -0.2, "heave": 0.0} ya da None
    js.stop()
"""
import json
import socket
import threading
import time
from typing import Optional, Dict

from config import JOYSTICK_PORT, ROV_IP


class JoystickClient:
    def __init__(self, host: str = "127.0.0.1", port: int = JOYSTICK_PORT):
        self.host = host
        self.port = port
        self._axes: Optional[Dict[str, float]] = None
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._sock = None

    # ---------------------------------------------------------------- public
    def start(self):
        """Arka planda TCP dinleme thread'ini baslat."""
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[JOYSTICK] {self.host}:{self.port} baglantisi dinleniyor.")

    def stop(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def read_axes(self) -> Optional[Dict[str, float]]:
        """
        Son gecerli eksen komutlarini dondurur.
        {"surge": float, "yaw": float, "heave": 0.0, "roll": 0.0, "pitch": 0.0}
        """
        with self._lock:
            return self._axes.copy() if self._axes is not None else None

    # ---------------------------------------------------------------- private
    def _loop(self):
        while self._running:
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.settimeout(2.0)
                self._sock.connect((self.host, self.port))
                print(f"[JOYSTICK] Baglandi → {self.host}:{self.port}")
                buf = b""

                while self._running:
                    try:
                        data = self._sock.recv(1024)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            msg = json.loads(line.decode('utf-8'))
                            fb = float(msg.get("forward_back", 0.0))
                            lr = float(msg.get("left_right", 0.0))
                            with self._lock:
                                self._axes = {
                                    "surge": fb,
                                    "yaw": lr,
                                    "heave": 0.0,
                                    "roll": 0.0,
                                    "pitch": 0.0
                                }
                        except (json.JSONDecodeError, ValueError):
                            pass
            except (OSError, ConnectionRefusedError):
                time.sleep(1.0)  # baglanti kesilirse 1sn bekle tekrar dene
            finally:
                if self._sock:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                with self._lock:
                    self._axes = None
