"""
Thread-guvenli kamera yoneticisi.

CSI (GStreamer) veya USB kamera acilir; arka planda surekli kare okur.
read() her zaman en guncel kareyi dondurur (non-blocking).

Kullanim:
    cam = Camera()
    cam.start()
    frame = cam.read()   # BGR numpy array ya da None
    cam.stop()
"""
import threading
import time

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

from config import CSI_CAMERA, CAM_WIDTH, CAM_HEIGHT, CAM_FPS, CAM_SENSOR_ID


def _gstreamer_pipeline(sensor_id, width, height, fps):
    """Jetson Orin Nano / Xavier NX icin CSI GStreamer pipeline."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={fps}/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        f"video/x-raw, format=BGR ! "
        "appsink drop=1"
    )


class Camera:
    def __init__(self):
        self._cap = None
        self._frame = None
        self._lock = threading.Lock()
        self._thread = None
        self._running = False

    # ---------------------------------------------------------------- public
    def start(self):
        """Kamerayi ac ve okuma thread'ini baslat."""
        if not _CV2_OK:
            print("[KAMERA] OpenCV bulunamadi — kamera devre disi.")
            return

        if CSI_CAMERA:
            pipeline = _gstreamer_pipeline(CAM_SENSOR_ID, CAM_WIDTH, CAM_HEIGHT, CAM_FPS)
            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            self._cap = cv2.VideoCapture(0)   # USB /dev/video0
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            self._cap.set(cv2.CAP_PROP_FPS, CAM_FPS)

        if not self._cap.isOpened():
            print("[KAMERA] Kamera acilamadi! CSI_CAMERA ayarini kontrol et.")
            self._cap = None
            return

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        mode = "CSI" if CSI_CAMERA else "USB"
        print(f"[KAMERA] {mode} {CAM_WIDTH}x{CAM_HEIGHT}@{CAM_FPS}fps baslatildi.")

    def stop(self):
        """Okuma thread'ini durdur, kamerayi kapat."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()

    def read(self):
        """En guncel BGR kareyi dondurur. Kamera kapaliysa None."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    # ---------------------------------------------------------------- private
    def _loop(self):
        while self._running and self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.01)
