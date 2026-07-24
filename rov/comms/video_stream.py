"""
UDP JPEG video akisi — Jetson (gonderen) <-> Laptop (alici).

Gonderen (Jetson):
    stream = VideoStreamer(camera)
    stream.start()    # arka plan thread

Alici (Laptop):
    python3 -m comms.video_stream --receive

Protokol: Her kare = 4 byte uzunluk (big-endian) + JPEG baytlari.
Kare boyutu ~20-60 KB @ 720p/q60 → UDP MTU'yu asan kareler
chunk'lara bolunup sirayla gonderilir (her chunk <= 60KB).
"""
import io
import socket
import struct
import threading
import time
import sys

try:
    import cv2
    import numpy as np
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

from config import GCS_IP, VIDEO_PORT, VIDEO_QUALITY, VIDEO_FPS, ROV_IP


# ─────────────────────────────────────── Gonderen (Jetson'da)
class VideoStreamer:
    CHUNK = 60_000   # bytes per UDP datagram

    def __init__(self, camera):
        """camera: sensors.camera.Camera nesnesi."""
        self.cam = camera
        self._thread = None
        self._running = False
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def start(self):
        """Arka planda video gondermeyi baslat."""
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[VIDEO] UDP akisi basliyor → {GCS_IP}:{VIDEO_PORT}")

    def stop(self):
        self._running = False

    def _loop(self):
        interval = 1.0 / VIDEO_FPS
        while self._running:
            t0 = time.monotonic()
            frame = self.cam.read()
            if frame is None:
                time.sleep(interval)
                continue
            ok, buf = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, VIDEO_QUALITY]
            )
            if not ok:
                continue
            data = buf.tobytes()
            # Header: toplam uzunluk
            header = struct.pack(">I", len(data))
            # Gonder (chunk'lara bol)
            payload = header + data
            for i in range(0, len(payload), self.CHUNK):
                self._sock.sendto(payload[i:i + self.CHUNK], (GCS_IP, VIDEO_PORT))
            elapsed = time.monotonic() - t0
            time.sleep(max(0, interval - elapsed))


# ─────────────────────────────────────── Alici (Laptopta)
class VideoReceiver:
    def __init__(self, port=VIDEO_PORT):
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(2.0)
        self._buf = b""
        self._expected = None

    def receive_frame(self):
        """Bir tam kare gelene kadar bekle, numpy array dondur. None = timeout."""
        while True:
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                return None

            self._buf += data

            # Header oku (ilk 4 byte)
            while len(self._buf) >= 4:
                if self._expected is None:
                    self._expected = struct.unpack(">I", self._buf[:4])[0]
                if len(self._buf) >= 4 + self._expected:
                    jpg = self._buf[4:4 + self._expected]
                    self._buf = self._buf[4 + self._expected:]
                    self._expected = None
                    arr = np.frombuffer(jpg, dtype=np.uint8)
                    return cv2.imdecode(arr, cv2.IMREAD_COLOR)
                else:
                    break
        return None

    def close(self):
        self._sock.close()


# ─────────────────────────────────────── komut satiri alici
def _receive_loop():
    """Laptopta: python3 -m comms.video_stream --receive"""
    if not _CV2_OK:
        print("OpenCV bulunamadi: pip3 install opencv-python")
        return
    print(f"[VIDEO] UDP port {VIDEO_PORT} dinleniyor... (q=cikis)")
    recv = VideoReceiver()
    try:
        while True:
            frame = recv.receive_frame()
            if frame is None:
                print("  (zaman asimi — paket bekleniyor)")
                continue
            cv2.imshow("EGE ROV - FPV", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        recv.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    if "--receive" in sys.argv:
        _receive_loop()
    else:
        print("Kullanim: python3 -m comms.video_stream --receive")
