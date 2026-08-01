"""
!!! BU SCRIPT ARTIK KULLANILMIYOR — YERINE:  python3 pid_tune.py  !!!

Sebep:
Sensoru kendi dongusunde bloklyarak okuyor (SORUN 2) ve katsayi
degistirirken I birikimini sifirlamiyor -> sonuclari yaniltici.

Dosya sadece referans icin duruyor. Havuzda pid_tune.py kullan.
"""

#!/usr/bin/env python3
"""
EGE ROV — Canlı Etkileşimli ve Adım Yanıtlı PID Kalibrasyon Scripti.

Bu script, su içinde veya test tankında ROV'un PID kontrolörlerini (Derinlik, Heading/Yön, Roll, Pitch)
canlı olarak ayarlamanızı, adım yanıtı (Step Response) testi yapmanızı ve ideal katsayıları
config.py dosyasına otomatik kaydetmenizi sağlar.

Kullanım (Jetson):
    python3 calibrate_pid.py

Komutlar (Canlı Konsol):
    kp <deger>    : Kp değerini ayarla (örn: kp 2.5)
    ki <deger>    : Ki değerini ayarla (örn: ki 0.1)
    kd <deger>    : Kd değerini ayarla (örn: kd 0.8)
    step <hedef>  : Adım testi başlat (örn: step 0.5 -> 0.5 metreye dal)
    stop          : Motorları durdur ve testi bitir
    save          : Yeni katsayıları config.py dosyasına otomatik kaydet
"""
import sys
import time
import re
import shutil
import threading
import statistics

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (
    LOOP_HZ, SIM_MODE,
    PID_DEPTH, PID_HEADING, PID_ROLL, PID_PITCH,
    MOTOR_CHANNELS
)
from control.pid import PID, angle_error_deg


def get_hardware_or_sim():
    """Donanım veya simülasyon sistemini başlatır."""
    if SIM_MODE:
        from sim.simulator import RovSimulator
        from hal.thrusters import Thrusters, MockBackend
        from sensors.imu import MockImu, Orientation
        from sensors.depth import MockDepth

        sim = RovSimulator()
        backend = MockBackend()
        thr = Thrusters(backend)
        ori = Orientation(MockImu(sim))
        depth = MockDepth(sim)

        def _sim_loop():
            dt = 1.0 / LOOP_HZ
            while True:
                sim.step(backend, dt)
                time.sleep(dt)

        t = threading.Thread(target=_sim_loop, daemon=True)
        t.start()
        print("[SİSTEM] Simülasyon modunda başlatıldı.")
        return thr, ori, depth
    else:
        from hal.thrusters import Thrusters, PCA9685Backend, MockBackend
        from sensors.imu import Mpu9250, MockImuStatic, Orientation
        from sensors.depth import Ms5837, MockDepthStatic

        try:
            backend = PCA9685Backend()
            print("[OK] PCA9685 Motor Sürücü bağlandı.")
        except Exception as e:
            print(f"[UYARI] PCA9685 bağlanamadı ({e}), MockBackend kullanılıyor.")
            backend = MockBackend()
        thr = Thrusters(backend)

        try:
            imu_sensor = Mpu9250()
            print("[OK] MPU-9250 IMU bağlandı.")
        except Exception as e:
            print(f"[UYARI] MPU-9250 bağlanamadı ({e}), MockImuStatic kullanılıyor.")
<<<<<<< Updated upstream
            imu_sensor = MockImuStatic()
=======
            imu_sensor = MockImuStatic()  # sim argümanı gerektirmeyen fallback
>>>>>>> Stashed changes
        ori = Orientation(imu_sensor)

        try:
            depth_sensor = Ms5837()
            print("[OK] MS5837 Derinlik Sensörü bağlandı.")
        except Exception as e:
            print(f"[UYARI] MS5837 bağlanamadı ({e}), MockDepthStatic kullanılıyor.")
            depth_sensor = MockDepthStatic()
        depth = depth_sensor

        return thr, ori, depth


<<<<<<< Updated upstream
def save_pid_to_config(axis_key, pid_dict, path="../config.py"):
    """Seçilen PID eksenini config.py dosyasına kaydeder."""
    CONFIG_PATH = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.py')
    )
    path = CONFIG_PATH
=======
_CONFIG_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.py')
)


def save_pid_to_config(axis_key, pid_dict, path=None):
    """Secilen PID eksenini config.py dosyasina kaydeder."""
    if path is None:
        path = _CONFIG_PATH
>>>>>>> Stashed changes
    shutil.copy(path, path + ".bak")
    with open(path) as f:
        text = f.read()

    line = (f"{axis_key:<11} = dict(kp={pid_dict['kp']:.3f}, ki={pid_dict['ki']:.3f}, "
            f"kd={pid_dict['kd']:.3f}, out_limit={pid_dict['out_limit']}, i_limit={pid_dict['i_limit']})")

    if re.search(fr"{axis_key}\s*=", text):
        text = re.sub(fr"{axis_key}\s*=.*", line, text, count=1)
    else:
        text += "\n" + line

    with open(path, "w") as f:
        f.write(text)
    print(f"\n[KAYIT OK] {axis_key} değerleri config.py dosyasına kaydedildi! (Yedek: config.py.bak)")


def print_pid_tuning_guide():
    print("""
======================================================================
               ROV PID KALİBRASYON VE AYAR REHBERİ
======================================================================
İdeal PID Ayarlama Adımları (Ziegler-Nichols Yöntemi):

1. ADIM (Sadece Kp):
   - Ki = 0.0 ve Kd = 0.0 yapın.
   - Kp değerini kademeli olarak artırın (örn: kp 0.5 -> kp 1.0 -> kp 2.0).
   - Sistem salınım (titreme/dalgalanma) yapmaya başladığı anı (Kritik Kp) bulun.

2. ADIM (Türev Kd Ekleyin - Frenleme):
   - Aşımı (Overshoot) ve salınımı sönümlemek için Kd ekleyin (örn: kd 0.2 -> kd 0.5).
   - Kd, ROV'un hedefe hızlı ama sarsıntısız oturmasını sağlar.

3. ADIM (İntegral Ki Ekleyin - Kalıcı Hata Giderimi):
   - Eğer araç hedefe tam oturmuyor (örn: 0.5m yerine 0.45m'de kalıyorsa) küçük bir Ki ekleyin (örn: ki 0.05).
   - Çok yüksek Ki salınıma ve kararsızlığa yol açar.
======================================================================
""")


def run_pid_calibration():
    print("=" * 70)
    print("        EGE ROV — ETKİLEŞİMLİ PID KALİBRASYON VE ADIM TESTİ")
    print("=" * 70)

    thr, ori, depth = get_hardware_or_sim()
    print_pid_tuning_guide()

    print("Kalibre edilecek ekseni seçin:")
    print("  1) Derinlik (DEPTH) PID")
    print("  2) Yön / Pusula (HEADING / YAW) PID")
    print("  3) Roll Stabilizasyon PID")
    print("  4) Pitch Stabilizasyon PID")

    choice = input("Seçim [1-4] (varsayılan 1): ").strip() or "1"
    
    axis_map = {
        "1": ("PID_DEPTH", PID_DEPTH, "Derinlik (m)", 0.5),
        "2": ("PID_HEADING", PID_HEADING, "Yön / Heading (derece)", 45.0),
        "3": ("PID_ROLL", PID_ROLL, "Roll Açısı (derece)", 0.0),
        "4": ("PID_PITCH", PID_PITCH, "Pitch Açısı (derece)", 0.0),
    }

    axis_key, initial_config, unit_label, default_step = axis_map.get(choice, axis_map["1"])
    print(f"\n[SEÇİLEN EKSEN] {axis_key} — Birim: {unit_label}")

    pid = PID(
        kp=initial_config['kp'],
        ki=initial_config['ki'],
        kd=initial_config['kd'],
        out_limit=initial_config['out_limit'],
        i_limit=initial_config['i_limit']
    )

    print(f"Başlangıç Değerleri: Kp={pid.kp}, Ki={pid.ki}, Kd={pid.kd}")
    print("\nMotorlar ARM ediliyor (ESC nötürleme)...")
    thr.arm()

    target_val = default_step
    testing = False
    test_thread = None
    stop_event = threading.Event()

    def pid_test_loop():
        nonlocal testing
        print(f"\n---> TEST BAŞLADI: Hedef {axis_key} = {target_val} | Kp={pid.kp}, Ki={pid.ki}, Kd={pid.kd}")
        print(f"{'Zaman (s)':<10} | {'Mevcut':<12} | {'Hedef':<12} | {'Hata':<10} | {'Çıktı (PWM)':<12}")
        print("-" * 65)

        t0 = time.monotonic()
        history_val = []
        history_err = []

        pid.reset()
        if not SIM_MODE:
            depth.zero_at_surface()

        while not stop_event.is_set():
            now = time.monotonic()
            elapsed = now - t0

            # Sensör okuması
            if not SIM_MODE:
                ori.update()

            if axis_key == "PID_DEPTH":
                current = depth.read_depth_m()
                err = target_val - current
            elif axis_key == "PID_HEADING":
                current = ori.heading if ori.heading is not None else 0.0
                err = angle_error_deg(target_val, current)
            elif axis_key == "PID_ROLL":
                current = ori.roll
                err = target_val - current
            else: # PID_PITCH
                current = ori.pitch
                err = target_val - current

            out = pid.update(err, now)

            # Motor komutu uygula
            if axis_key == "PID_DEPTH":
                # 4 dikey motora eşit dikey itki ver
                thr.command({"V_FL": out, "V_FR": out, "V_RL": out, "V_RR": out, "H_L": 0.0, "H_R": 0.0})
            elif axis_key == "PID_HEADING":
                # 2 yatay motora zıt dönü torku ver
                thr.command({"H_L": out, "H_R": -out, "V_FL": 0.0, "V_FR": 0.0, "V_RL": 0.0, "V_RR": 0.0})
            elif axis_key == "PID_ROLL":
                thr.command({"V_FL": out, "V_FR": -out, "V_RL": out, "V_RR": -out, "H_L": 0.0, "H_R": 0.0})
            else: # PITCH
                thr.command({"V_FL": out, "V_FR": out, "V_RL": -out, "V_RR": -out, "H_L": 0.0, "H_R": 0.0})

            history_val.append(current)
            history_err.append(err)

            if int(elapsed * 10) % 5 == 0:  # ~2Hz ekrana bas
                print(f" {elapsed:8.2f}s | {current:10.3f}   | {target_val:10.3f}   | {err:8.3f}   | {out:10.3f}")

            time.sleep(1.0 / LOOP_HZ)

        thr.stop()
        print("\n---> TEST DURDURULDU. Motorlar kapatıldı.")

        # Adım yanıtı analizi
        if history_val:
            peak_val = max(history_val) if target_val >= 0 else min(history_val)
            overshoot = abs((peak_val - target_val) / target_val * 100.0) if target_val != 0 else 0.0
            final_err = abs(history_err[-1])
            print("\n--- ADIM YANITI (STEP RESPONSE) ANALİZİ ---")
            print(f"  Maksimum Değer   : {peak_val:.3f}")
            print(f"  Aşım (Overshoot) : %{overshoot:.1f}")
            print(f"  Kalıcı Durum Hata: {final_err:.3f}")
            print("------------------------------------------")

    print("\n" + "=" * 70)
    print("ETKİLEŞİMLİ KONSOL AKTİF (Komut yazıp ENTER'a basın)")
    print("Örnekler: 'kp 2.5', 'ki 0.1', 'kd 0.8', 'step 0.6', 'stop', 'save', 'exit'")
    print("=" * 70 + "\n")

    try:
        while True:
            cmd_line = input("PID-Tuner> ").strip().lower()
            if not cmd_line:
                continue

            parts = cmd_line.split()
            cmd = parts[0]

            if cmd == "exit" or cmd == "quit":
                break
            elif cmd == "kp" and len(parts) > 1:
                pid.kp = float(parts[1])
                print(f"[OK] Kp = {pid.kp}")
            elif cmd == "ki" and len(parts) > 1:
                pid.ki = float(parts[1])
                print(f"[OK] Ki = {pid.ki}")
            elif cmd == "kd" and len(parts) > 1:
                pid.kd = float(parts[1])
                print(f"[OK] Kd = {pid.kd}")
            elif cmd == "step":
                if len(parts) > 1:
                    target_val = float(parts[1])
                if testing and test_thread and test_thread.is_alive():
                    stop_event.set()
                    test_thread.join()
                stop_event.clear()
                testing = True
                test_thread = threading.Thread(target=pid_test_loop, daemon=True)
                test_thread.start()
            elif cmd == "stop":
                if testing and test_thread and test_thread.is_alive():
                    stop_event.set()
                    test_thread.join()
                testing = False
                thr.stop()
                print("[OK] Test durduruldu.")
            elif cmd == "save":
                saved_params = {"kp": pid.kp, "ki": pid.ki, "kd": pid.kd,
                                "out_limit": pid.out_limit, "i_limit": pid.i_limit}
                save_pid_to_config(axis_key, saved_params)
            else:
                print(f"Bilinmeyen komut: {cmd_line}. Kullanılabilir: kp, ki, kd, step, stop, save, exit")

    except KeyboardInterrupt:
        print("\nÇıkış yapılıyor...")
    finally:
        if testing and test_thread and test_thread.is_alive():
            stop_event.set()
            test_thread.join()
        thr.stop()
        print("[TAMAMLANDI] Motorlar güvenle kapatıldı.")


if __name__ == "__main__":
    run_pid_calibration()
