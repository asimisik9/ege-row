#!/usr/bin/env python3
"""
EGE ROV — Otomatik PID Kalibrasyonu (Åström-Hägglund Röle Yöntemi).

Bu script, seçtiğiniz eksende (Derinlik veya Heading) ROV'a küçük darbe komutları (Relay pulses)
göndererek sistemin doğal salınım frekansını (Tu) ve genliğini (a) otomatik ölçer.
Ziegler-Nichols otomatik PID formülü ile ideal Kp, Ki, Kd değerlerini 30 saniyede otomatik hesaplar
ve config.py dosyasına kaydeder.

Kullanım (Jetson):
    python3 auto_tune_pid.py
"""
import sys
import time
import math
import shutil
import re
import threading

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (
    LOOP_HZ, SIM_MODE,
    PID_DEPTH, PID_HEADING, PID_ROLL, PID_PITCH
)
from control.pid import PID, angle_error_deg


def get_system():
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
        return thr, ori, depth
    else:
        from hal.thrusters import Thrusters, PCA9685Backend, MockBackend
        from sensors.imu import Mpu9250, MockImu, Orientation
        from sensors.depth import Ms5837, MockDepth

        try:
            backend = PCA9685Backend()
        except Exception:
            backend = MockBackend()
        thr = Thrusters(backend)

        try:
            imu_sensor = Mpu9250()
        except Exception:
            imu_sensor = MockImu()
        ori = Orientation(imu_sensor)

        try:
            depth_sensor = Ms5837()
        except Exception:
            depth_sensor = MockDepth()
        depth = depth_sensor

        return thr, ori, depth


def save_pid_to_config(axis_key, kp, ki, kd, out_limit=1.0, i_limit=0.5, path="config.py"):
    shutil.copy(path, path + ".bak")
    with open(path) as f:
        text = f.read()

    line = (f"{axis_key:<11} = dict(kp={kp:.3f}, ki={ki:.3f}, "
            f"kd={kd:.3f}, out_limit={out_limit}, i_limit={i_limit})")

    if re.search(fr"{axis_key}\s*=", text):
        text = re.sub(fr"{axis_key}\s*=.*", line, text, count=1)
    else:
        text += "\n" + line

    with open(path, "w") as f:
        f.write(text)
    print(f"\n[OTOMATİK KAYIT OK] {axis_key} -> Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f} config.py'ye yazıldı!")


def auto_tune():
    print("=" * 70)
    print("        EGE ROV — OTOMATİK PID OTOMATİK HESAPLAMA (AUTO-TUNER)")
    print("=" * 70)

    thr, ori, depth = get_system()

    print("\nHangi ekseni OTOMATİK ayarlamak istiyorsunuz?")
    print("  1) Derinlik (DEPTH) PID")
    print("  2) Yön / Pusula (HEADING) PID")
    choice = input("Seçim [1-2] (varsayılan 1): ").strip() or "1"

    if choice == "2":
        axis_key = "PID_HEADING"
        target_val = 90.0  # 90 derece hedef
        relay_amplitude = 0.35  # gaz genliği
        out_limit = 0.6
        i_limit = 0.2
    else:
        axis_key = "PID_DEPTH"
        target_val = 0.5  # 0.5m hedef
        relay_amplitude = 0.40
        out_limit = 1.0
        i_limit = 0.3

    print(f"\n[OTOMATİK TUNING] {axis_key} için 4 salınım periyodu ölçülüyor...")
    print("Motorlar ARM ediliyor...")
    thr.arm()

    if not SIM_MODE:
        depth.zero_at_surface()

    # Åström-Hägglund Röle Denetimi
    relay_state = True
    peaks = []
    zero_crossings = []
    
    t0 = time.monotonic()
    last_cross_t = t0

    try:
        while len(zero_crossings) < 8 and (time.monotonic() - t0) < 30.0:
            now = time.monotonic()
            if not SIM_MODE:
                ori.update()

            if axis_key == "PID_DEPTH":
                curr = depth.read_depth_m()
                err = target_val - curr
            else:
                curr = ori.heading if ori.heading is not None else 0.0
                err = angle_error_deg(target_val, curr)

            # Röle anahtarlama (Hata pozitifse pozitif gaz, negatifse negatif gaz)
            if err > 0 and not relay_state:
                relay_state = True
                zero_crossings.append(now)
            elif err < 0 and relay_state:
                relay_state = False
                zero_crossings.append(now)

            motor_out = relay_amplitude if relay_state else -relay_amplitude

            if axis_key == "PID_DEPTH":
                thr.command({"V_FL": motor_out, "V_FR": motor_out, "V_RL": motor_out, "V_RR": motor_out, "H_L": 0.0, "H_R": 0.0})
            else:
                thr.command({"H_L": motor_out, "H_R": -motor_out, "V_FL": 0.0, "V_FR": 0.0, "V_RL": 0.0, "V_RR": 0.0})

            peaks.append(abs(err))
            time.sleep(1.0 / LOOP_HZ)

    finally:
        thr.stop()

    if len(zero_crossings) >= 4:
        # Periyot T_u ve Genlik a hesapla
        periods = [zero_crossings[i] - zero_crossings[i-2] for i in range(2, len(zero_crossings), 2)]
        Tu = sum(periods) / len(periods)
        a = max(peaks) if peaks else 1.0

        # Kritik Kazanç Ku
        Ku = (4.0 * relay_amplitude) / (math.pi * max(0.01, a))

        # Ziegler-Nichols PID Formülleri
        kp = 0.6 * Ku
        ki = (2.0 * kp) / max(0.1, Tu)
        kd = (kp * Tu) / 8.0

        print("\n" + "=" * 70)
        print("        OTOMATİK PID HESAPLAMA SONUÇLARI (Ziegler-Nichols)")
        print("=" * 70)
        print(f"  Ölçülen Doğal Periyot (Tu) : {Tu:.2f} saniye")
        print(f"  Kritik Kazanç (Ku)        : {Ku:.3f}")
        print("-" * 50)
        print(f"  ÖNERİLEN Kp : {kp:.3f}")
        print(f"  ÖNERİLEN Ki : {ki:.3f}")
        print(f"  ÖNERİLEN Kd : {kd:.3f}")
        print("=" * 70)

        ans = input("\nBu ideal değerler config.py dosyasına kaydedilsin mi? [E/h]: ").strip().lower() or "e"
        if ans == "e":
            save_pid_to_config(axis_key, kp, ki, kd, out_limit, i_limit)
    else:
        print("\n[UYARI] Yeterli salınım ölçülemedi (zaman aşımı). Lütfen manuel testi deneyin.")


if __name__ == "__main__":
    auto_tune()
