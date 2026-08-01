"""
EGE ROV — Sensörsüz Manuel Sürüş (Teleop) Scripti.

HİÇBİR SENSÖR (IMU, Derinlik, Kamera vb.) GEREKTİRMEZ!
Sadece PCA9685 motor sürücüsü ile çalışır.

Kullanım (Jetson):
    python3 manual_drive.py           # Gerçek donanımda manuel sürüş + Web GCS
    python3 manual_drive.py --sim     # Simülasyonda test

Kontrol Seçenekleri:
  1. Web GCS: http://192.168.1.10:8000/ (Tarayıcıdan WASD / IJKL tuşları veya butonlar)
  2. Fiziki PS3 Kolu: python3 joystick/joystick_server.py (Arka planda Otomatik bağlanır)
  3. Terminal Klavye:
       W / S : İleri / Geri (Surge)
       A / D : Sol / Sağ Dönüş (Yaw)
       I / K : Yüksel / Dal (Heave)
       SPACE : Tüm motorları durdur (Nötr)
       Q     : Çıkış
"""
import sys
import time
import select
import threading

import config
if "--sim" in sys.argv:
    config.SIM_MODE = True

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import LOOP_HZ, SIM_MODE, MOTOR_CHANNELS, PWM_NEUTRAL_US
from control.mixer import mix
from comms.web_server import WebGCS
from sensors.joystick import JoystickClient


def get_key_nonblocking():
    """Linux terminalinde basilan tusu okur (non-blocking)."""
    if sys.platform == "win32":
        import msvcrt
        if msvcrt.kbhit():
            try:
                return msvcrt.getch().decode('utf-8').lower()
            except UnicodeDecodeError:
                return ""
        return ""
    else:
        # Linux / Jetson
        import termios
        import tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
                key = sys.stdin.read(1).lower()
                return key
            return ""
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    print("=" * 60)
    print("   EGE ROV — SENSÖRSÜZ MANUEL SÜRÜŞ SİSTEMİ")
    print("=" * 60)

    # ── Donanim Kurulumu (SADECE MOTORLAR)
    if SIM_MODE:
        print("[BILGI] Simulasyon modu aktif (--sim).")
        from hal.thrusters import Thrusters, MockBackend
        backend = MockBackend()
    else:
        print("[DONANIM] PCA9685 motor surucusune baglaniliyor...")
        from hal.thrusters import Thrusters, PCA9685Backend
        try:
            backend = PCA9685Backend()
        except Exception as e:
            print(f"[HATA] PCA9685 baglanamadi: {e}")
            sys.exit(1)

    thr = Thrusters(backend)

    # ── Web GCS Sunucusu (Sensorsuz Mod)
    gcs = WebGCS()
    gcs.start(thrusters=thr)
    gcs.set_active_mission("MANUAL_TELEOP")

    # ── Fiziki PS3 Joystick Istemcisi
    js = JoystickClient()
    js.start()

    print(f"\n[OK] Motorlar Notrde ({PWM_NEUTRAL_US}us).")
    if not SIM_MODE:
        input("--> ESC'lere GUC VER, bip seslerinin bitmesini bekle, sonra ENTER'a bas...")
    
    print("\nMotorlar Arm ediliyor...")
    thr.arm()
    print("\n" + "=" * 60)
    print(" SÜRÜŞE HAZIR!")
    print(" Web GCS: http://192.168.1.10:8000/")
    print(" Tuşlar: W/S (İleri/Geri), A/D (Dönüş), I/K (Dikey), SPACE (Dur), Q (Çıkış)")
    print("=" * 60 + "\n")

    surge = 0.0
    yaw = 0.0
    heave = 0.0
    step = 0.15   # Her tus basiminda gaz artisi

    try:
        while True:
            # 1. PS3 Joystick kontrolu
            js_axes = js.read_axes()
            # 2. Web GCS kontrolu
            web_axes = gcs.get_teleop()
            # 3. Terminal Klavye kontrolu
            key = get_key_nonblocking()

            if key == 'q':
                print("\nCikis yapiliyor...")
                break
            elif key == 'w':
                surge = min(1.0, surge + step)
            elif key == 's':
                surge = max(-1.0, surge - step)
            elif key == 'a':
                yaw = max(-1.0, yaw - step)
            elif key == 'd':
                yaw = min(1.0, yaw + step)
            elif key == 'i':
                heave = max(-1.0, heave - step) # Yüksel (heave negatif = yukari)
            elif key == 'k':
                heave = min(1.0, heave + step) # Dal
            elif key == ' ':
                surge = 0.0; yaw = 0.0; heave = 0.0

            # Oncelik Sirasi: PS3 Kolu > Web GCS > Terminal Klavyesi
            # DIKKAT: kol sadece GERCEKTEN oynatildiginda oncelik alir. Aksi halde
            # bagli-ama-bosta duran bir kol, tum eksenleri 0 yazip Web GCS ve
            # klavyeyi (dolayisiyla dalis komutunu) tamamen susturuyordu.
            if js.is_active():
                axes_to_apply = js_axes
                source = "PS3 Kolu"
            elif web_axes:
                axes_to_apply = web_axes
                source = "Web GCS"
            else:
                axes_to_apply = {"surge": surge, "yaw": yaw, "heave": heave, "roll": 0.0, "pitch": 0.0}
                source = "Terminal Klavye"

            # Motor komutlarini uygula
            cmd_dict = mix(**axes_to_apply)
            thr.command(cmd_dict)

            # Telemetriyi guncelle
            gcs.update_telemetry("MANUAL_TELEOP", {
                "source": source,
                "surge": round(axes_to_apply.get("surge", 0.0), 2),
                "yaw": round(axes_to_apply.get("yaw", 0.0), 2),
                "heave": round(axes_to_apply.get("heave", 0.0), 2),
            })

            time.sleep(1.0 / LOOP_HZ)

    except KeyboardInterrupt:
        print("\nCtrl+C ile durduruldu.")
    finally:
        thr.stop()
        js.stop()
        gcs.stop()
        print("Tum motorlar notrde. Guvenli cikis yapildi.")


if __name__ == "__main__":
    main()
