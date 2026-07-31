"""
EGE ROV - Video gosterimi gorevi: SADE ana script (WebGCS'siz).

Yarisma gunu icin minimum bagimlilikli giris noktasi. Sadece:
sensorler + stabilizasyon + gorev durum makinesi + e-stop + CSV log.

Kullanim (Jetson, rov/ klasoru icinden):
  python3 video_main.py            # on kontrol + gorevi baslat
  python3 video_main.py --check    # SADECE on kontrol (motor donmez)
  python3 video_main.py --sim      # simulasyonda dene (donanim gerekmez)

Gorev dizisi (sartname 2.4.3.3):
  Geri sayim -> DAL -> DUZ1 -> DON1(+90) -> DUZ2 -> DAIRE(360)
             -> DUZ3 -> DON2(+90) -> DUZ4 -> DUR

Durdurma: Ctrl+C her an guvenli iptal (motorlar notre ceker).
"""
import sys
import time

import config

if "--sim" in sys.argv:
    config.SIM_MODE = True

from config import (LOOP_HZ, SIM_MODE, GYRO_BIAS, MAG_OFFSET, MAG_SCALE,
                    SURFACE_PRESSURE_MBAR, MISSION)


# ──────────────────────────────────────── on kontrol
def preflight_check():
    """Kalibrasyon ve yapilandirma kontrolu. Kritik eksikte False dondurur."""
    ok = True
    print("=== ON KONTROL ===")

    if SIM_MODE:
        print("[i] SIM_MODE acik - simulasyonda calisiyor (cihazda False olmali!)")

    if GYRO_BIAS == (0.0, 0.0, 0.0):
        print("[UYARI] GYRO_BIAS kalibre edilmemis -> python3 calibrate_imu.py")
        ok = False
    else:
        print(f"[OK] GYRO_BIAS = {GYRO_BIAS}")

    if MAG_OFFSET == (0.0, 0.0, 0.0) and MAG_SCALE == (1.0, 1.0, 1.0):
        print("[UYARI] Pusula kalibre edilmemis -> python3 calibrate_imu.py")
        ok = False
    else:
        print(f"[OK] MAG_OFFSET = {MAG_OFFSET}, MAG_SCALE = {MAG_SCALE}")

    if SURFACE_PRESSURE_MBAR is None:
        print("[i] Yuzey basinci config'de yok; gorev basinda otomatik "
              "sifirlanacak (istersen: python3 calibrate_depth.py)")
    else:
        print(f"[OK] SURFACE_PRESSURE_MBAR = {SURFACE_PRESSURE_MBAR}")

    m = MISSION
    print(f"[i] Gorev: derinlik {m['target_depth_m']} m, duz gidis "
          f"{m['straight_time_s']} sn @ gaz {m['cruise_throttle']}, "
          f"geri sayim {m['start_delay_s']} sn")
    return ok


def build_hardware():
    """Donanimi kurar. Gercek modda eksik donanim = HATA (gorev baslamaz)."""
    from hal.thrusters import Thrusters, MockBackend

    if SIM_MODE:
        import threading
        from sim.simulator import RovSimulator
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

        threading.Thread(target=_sim_loop, daemon=True).start()
        print("[SIM] Simulasyon aktif.")
        return thr, ori, depth

    # Gercek donanim: video gorevi ucu icin de sarttir, eksikse baslatma.
    from hal.thrusters import PCA9685Backend
    from sensors.imu import Mpu9250, Orientation
    from sensors.depth import Ms5837

    try:
        thr = Thrusters(PCA9685Backend())
        print("[OK] PCA9685 motor surucu baglandi.")
    except Exception as e:
        raise SystemExit(f"[HATA] Motor surucu (PCA9685) baglanamadi: {e}")

    try:
        ori = Orientation(Mpu9250())
        print("[OK] MPU-9250 IMU baglandi.")
    except Exception as e:
        raise SystemExit(f"[HATA] IMU (MPU-9250) baglanamadi: {e}")

    try:
        depth = Ms5837()
        print("[OK] MS5837 derinlik sensoru baglandi.")
    except Exception as e:
        raise SystemExit(f"[HATA] Derinlik sensoru (MS5837) baglanamadi: {e}")

    return thr, ori, depth


# ──────────────────────────────────────── gorev dongusu
def run(thr, ori, depth):
    """Gorevi kosar: durum makinesi LOOP_HZ'de adimlanir, 1 Hz durum basilir."""
    from control.stabilizer import Stabilizer
    from missions.video_demo import VideoDemoMission
    from utils.logger import MissionLogger
    from hal.estop import EStopMonitor

    stab = Stabilizer(ori, depth)
    log = MissionLogger("video_demo")
    estop = EStopMonitor(thr)
    mission = VideoDemoMission(stab, thr, logger=log)

    estop.start()
    print(f"\nGorev baslatiliyor - geri sayim {MISSION['start_delay_s']:.0f} sn. "
          "Iptal: Ctrl+C")
    mission.start()

    last_print = 0.0
    try:
        while True:
            if estop.triggered.is_set():
                print("E-STOP tetiklendi - gorev iptal!")
                break

            if mission.step():
                print("\n*** GOREV TAMAMLANDI ***")
                break

            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                h = stab.ori.heading
                print(f"[{mission.state:<10}] derinlik={depth.read_depth_m():5.2f} m  "
                      f"heading={h if h is not None else -1:6.1f} deg")

            time.sleep(1.0 / LOOP_HZ)
    except KeyboardInterrupt:
        print("\nCtrl+C - gorev iptal ediliyor...")
    finally:
        mission.abort()
        thr.stop()
        estop.stop()
        log.close()
        print(f"Motorlar durduruldu. Log: {log.path}")


# ──────────────────────────────────────── giris
if __name__ == "__main__":
    calib_ok = preflight_check()

    if "--check" in sys.argv:
        # Sadece kontrol: sensorlere baglanmayi da dene, motor komutu yok.
        thr, ori, depth = build_hardware()
        if not SIM_MODE:
            ori.update()
            print(f"[TEST] heading={ori.heading:.1f} deg, "
                  f"derinlik={depth.read_depth_m():.2f} m")
        thr.stop()
        print("On kontrol bitti." + ("" if calib_ok else " (KALIBRASYON EKSIK!)"))
        sys.exit(0 if calib_ok else 1)

    if not calib_ok and not SIM_MODE:
        ans = input("\nKalibrasyon eksik! Yine de devam? [e/H]: ").strip().lower()
        if ans != "e":
            print("Iptal. Once kalibrasyon: python3 calibrate_depth.py && "
                  "python3 calibrate_imu.py")
            sys.exit(1)

    thr, ori, depth = build_hardware()
    try:
        run(thr, ori, depth)
    finally:
        thr.stop()
