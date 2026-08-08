#!/usr/bin/env python3
"""
EGE ROV — CSI Kamera (Jetson Portu) ve USB Kamera Doğrulama ve Canlı Web Yayın Testi.

Jetson Orin Nano / Xavier NX üzerindeki CSI MIPI şerit kablo kamera portuna
bağlı kamerayı (nvarguscamerasrc) veya USB kamerayı test eder.

Kullanım (Jetson):
    python3 test_camera.py             # Canlı kare yakalama ve FPS ölçümü (Terminal)
    python3 test_camera.py --web       # PC'den CANLI İZLEMEK İÇİN http://192.168.1.10:8080/
    python3 test_camera.py --save      # Test fotoğrafı kaydet (camera_snapshot.jpg)
    python3 test_camera.py --samples=10 # 10 kare okuyup istatistik bas ve çık
"""
import sys
import time
import os
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

try:
    import cv2
except ImportError:
    print("[HATA] OpenCV (cv2) bulunamadı! Yüklemek için: sudo apt install python3-opencv")
    sys.exit(1)

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import CSI_CAMERA, CAM_WIDTH, CAM_HEIGHT, CAM_FPS, CAM_SENSOR_ID

# Global Son Kare (MJPEG Web Akışı İçin)
latest_frame_jpeg = None
frame_lock = threading.Lock()


def get_gstreamer_pipeline(sensor_id=0, width=1280, height=720, fps=30):
    """Jetson Orin Nano / Xavier NX CSI Kamera GStreamer Hattı."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={fps}/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=1"
    )


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """PC'den tarayıcı ile canlı görüntü izlemek için hafif HTTP MJPEG Sunucusu."""
    def log_message(self, format, *args):
        return  # Log kalabalığını engelle

    def do_GET(self):
        if self.path == '/' or self.path == '/video':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=jpgboundary')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        if latest_frame_jpeg is None:
                            time.sleep(0.05)
                            continue
                        jpeg_bytes = latest_frame_jpeg

                    self.wfile.write(b"--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(jpeg_bytes)))
                    self.end_headers()
                    self.wfile.write(jpeg_bytes)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.04)  # ~25 FPS web yayını
            except (ConnectionResetError, BrokenPipeError):
                pass
        else:
            self.send_error(404)


def start_web_stream_server(port=8080):
    server = HTTPServer(('0.0.0.0', port), MJPEGStreamHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"\n[CANLI YAYIN AKTİF] Bilgisayarınızdan izlemek için tarayıcıda açın:")
    print(f"👉  http://192.168.1.10:{port}/   veya   http://localhost:{port}/\n")


def test_camera():
    global latest_frame_jpeg

    print("=" * 70)
    print("        EGE ROV — CSI / USB KAMERA DOĞRULAMA VE PERFORMANS TESTİ")
    print("=" * 70)

    print(f"\n[KONFİGÜRASYON] CSI_CAMERA={CSI_CAMERA}, Çözünürlük={CAM_WIDTH}x{CAM_HEIGHT}, Hedef FPS={CAM_FPS}")

    cap = None
    cam_type = "Bilinmiyor"

    # 1. Deneme: CSI Kamera (Sensor ID 0 ve Sensor ID 1 deneniyor)
    if CSI_CAMERA:
        for sid in [CAM_SENSOR_ID, 1 if CAM_SENSOR_ID == 0 else 0]:
            print(f"\n---> [CSI KAMERA] Sensor ID {sid} (GStreamer nvarguscamerasrc) deneniyor...")
            pipeline = get_gstreamer_pipeline(sid, CAM_WIDTH, CAM_HEIGHT, CAM_FPS)
            c = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if c.isOpened():
                # Gerçek kare okumayı test et
                ret, test_f = c.read()
                if ret and test_f is not None:
                    cap = c
                    cam_type = f"CSI MIPI Port (Sensor ID {sid})"
                    print(f"[OK] CSI Kamera Sensor ID {sid} üzerinden görüntü alındı!")
                    break
                else:
                    c.release()

    # 2. Deneme: Düşme durumunda USB Kamera (/dev/video0)
    if cap is None:
        print("\n---> [USB KAMERA] /dev/video0 deneniyor...")
        c = cv2.VideoCapture(0)
        if c.isOpened():
            c.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            c.set(cv2.CAP_PROP_FPS, CAM_FPS)
            ret, test_f = c.read()
            if ret and test_f is not None:
                cap = c
                cam_type = "USB Kamera (/dev/video0)"
                print("[OK] USB Kamera (/dev/video0) üzerinden görüntü alındı!")

    if cap is None:
        print("\n[HATA] Hiçbir kameradan görüntü alınamadı!")
        print("-----------------------------------------------------------------")
        print("ÇÖZÜM ADIMLARI:")
        print(" 1. NVIDIA Argus servisini yeniden başlatın (En sık çözüm!):")
        print("    sudo systemctl restart nvargus-daemon")
        print(" 2. Şerit kablo yönü: Kablonun mavi kısmının konnektörün kilit")
        print("    mandalına doğru baktığından emin olun.")
        print(" 3. Şerit kablonun Jetson MIPI yuvasına tam oturduğunu ve siyah")
        print("    mandalın kilitlendiğini kontrol edin.")
        print(" 4. USB webcam kullanıyorsanız config.py içinde CSI_CAMERA = False yapın.")
        print("-----------------------------------------------------------------")
        sys.exit(1)

    # Argüman Kontrolleri
    save_snapshot = "--save" in sys.argv
    run_web = "--web" in sys.argv
    max_samples = None
    for arg in sys.argv[1:]:
        if arg.startswith("--samples="):
            max_samples = int(arg.split("=")[1])
        elif arg == "--once":
            max_samples = 1

    if run_web:
        start_web_stream_server(port=8080)

    print("\n-----------------------------------------------------------------")
    print("CANLI KARE AKIŞI VE HIZ ÖLÇÜMÜ (Çıkış için Ctrl+C'ye basın)...")
    print("-----------------------------------------------------------------")
    print(f"{'Kare #':<8} | {'Boyut':<12} | {'Ort. Parlaklık':<16} | {'Gerçek FPS':<12} | {'Durum'}")
    print("-" * 70)

    frame_count = 0
    t_start = time.monotonic()
    t_last_fps = t_start
    fps_counter = 0
    measured_fps = 0.0
    saved_flag = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print(f"[{frame_count}] [HATA] Kare okunamadı!")
                time.sleep(0.1)
                continue

            frame_count += 1
            fps_counter += 1
            now = time.monotonic()

            # FPS Hesapla (her 1 saniyede bir güncelle)
            if now - t_last_fps >= 1.0:
                measured_fps = fps_counter / (now - t_last_fps)
                fps_counter = 0
                t_last_fps = now

            # MJPEG Web Yayını İçin JPEG Sıkıştır
            if run_web or True:
                ret_jpg, jpeg_buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ret_jpg:
                    with frame_lock:
                        latest_frame_jpeg = jpeg_buf.tobytes()

            h, w, c = frame.shape
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            avg_brightness = float(gray.mean())

            # İsteğe bağlı ilk kare fotoğrafını kaydet
            if save_snapshot and not saved_flag:
                filename = "camera_snapshot.jpg"
                cv2.imwrite(filename, frame)
                saved_flag = True
                snapshot_msg = f" -> Fotoğraf kaydedildi: {filename}"
            else:
                snapshot_msg = ""

            end_char = "\n" if max_samples is not None else "\r"
            sys.stdout.write(
                f"  #{frame_count:<6} | {w}x{h:<7} | {avg_brightness:6.1f} / 255     | {measured_fps:5.1f} FPS    | OK{snapshot_msg}{end_char}"
            )
            sys.stdout.flush()

            if max_samples and frame_count >= max_samples:
                print("\n\n[OK] İstenen kare sayısına ulaşıldı.")
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nTest kullanıcı tarafından durduruldu.")
    finally:
        cap.release()
        total_time = time.monotonic() - t_start
        print("\n-----------------------------------------------------------------")
        print(f"ÖZET: Toplam {frame_count} kare okundu ({total_time:.1f} saniye).")
        print(f"Ortalama Performans: {frame_count / max(0.1, total_time):.1f} FPS")
        if saved_flag:
            print(f"Test Fotoğrafı: {os.path.abspath('camera_snapshot.jpg')}")
        print("==================================================================")


if __name__ == "__main__":
    test_camera()
