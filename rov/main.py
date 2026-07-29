"""
EGE ROV - ana giris noktasi (guncellenmis).

Kullanim (Jetson):
  python3 main.py                    # video demo gorevi + Web GCS (http://localhost:8000/)
  python3 main.py --mission video    # ayni
  python3 main.py --mission line     # hat takibi gorevi (Gorev 1) + Web GCS
  python3 main.py --mission nav      # navigasyon gorevi (Gorev 2) + Web GCS
  python3 main.py --test-motors      # tek tek motor testi
  python3 main.py --sim              # SIM_MODE zorla True (test icin)
  python3 main.py --estop-test       # e-stop GPIO tetikle (donanim testi)
"""
import sys
import time
import config

# --sim argumani: SIM_MODE'u override et
if "--sim" in sys.argv:
    config.SIM_MODE = True

from config import LOOP_HZ, SIM_MODE
from comms.web_server import WebGCS


# ──────────────────────────────────────── yardimcilar
def build_system():
    """Donanim baglantilarini kurar; donanim veya sensör yoksa Mock nesnelerine yumusak gecis yapar."""
    from hal.thrusters import Thrusters, PCA9685Backend, MockBackend
    from sensors.imu import Mpu9250, MockImu, Orientation
    from sensors.depth import Ms5837, MockDepth
    from sim.simulator import RovSimulator

    if SIM_MODE:
        import threading
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
        print("[SIM] 3-DOF ROV Fizik Simulasyonu ve Mock Sensörler Aktif!")
        return thr, ori, depth

    # Gerçek Donanım Bağlantı Denemeleri
    try:
        thr_backend = PCA9685Backend()
        print("[OK] PCA9685 Motor Sürücü Bağlandı.")
    except Exception as e:
        print(f"[UYARI] PCA9685 Sürücü Bağlanamadı ({e}), MockBackend Kullanılıyor.")
        thr_backend = MockBackend()

    thr = Thrusters(thr_backend)

    try:
        imu_sensor = Mpu9250()
        print("[OK] MPU-9250 IMU Sensörü Bağlandı.")
    except Exception as e:
        print(f"[UYARI] MPU-9250 IMU Bağlanamadı ({e}), MockIMU Kullanılıyor.")
        imu_sensor = MockImu()

    ori = Orientation(imu_sensor)

    try:
        depth_sensor = Ms5837()
        print("[OK] MS5837 Derinlik Sensörü Bağlandı.")
    except Exception as e:
        print(f"[UYARI] MS5837 Derinlik Sensörü Bağlanamadı ({e}), MockDepth Kullanılıyor.")
        depth_sensor = MockDepth()

    return thr, ori, depth


def test_motors(thr):
    """Her motoru sirayla 2sn dusuk gazda dondur (pervane yonu kontrolu)."""
    from config import MOTOR_CHANNELS
    print("Motor testi basliyor. Pervaneler takili degilken ya da su icindeyken yap!")
    thr.arm()
    for name in MOTOR_CHANNELS:
        print(f"  {name} ileri %10 ...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            thr.command({n: (0.10 if n == name else 0.0) for n in MOTOR_CHANNELS})
            time.sleep(1.0 / LOOP_HZ)
        thr.stop(); thr.arm()
        time.sleep(0.8)
    thr.stop()
    print("Motor testi bitti.")


# ──────────────────────────────────────── gorev kosturucular
def run_video_demo(thr, ori, depth):
    """Otonom video gosterimi gorevi."""
    from control.stabilizer import Stabilizer
    from missions.video_demo import VideoDemoMission
    from utils.logger import MissionLogger
    from hal.estop import EStopMonitor
    from control.mixer import mix

    stab = Stabilizer(ori, depth)
    log  = MissionLogger("video_demo")
    estop = EStopMonitor(thr)
    mission = VideoDemoMission(stab, thr, logger=log)

    gcs = WebGCS()
    gcs.start(thrusters=thr, stabilizer=stab, estop=estop)
    gcs.set_active_mission("VIDEO_DEMO", mission)

    estop.start()
    print("Video demo gorevi baslatiliyor (geri sayim: start_delay_s)...")
    mission.start()
    try:
        while True:
            # GCS teleop veya e-stop kontrolu
            teleop_axes = gcs.get_teleop()
            if teleop_axes:
                thr.command(mix(**teleop_axes))
            elif estop.triggered.is_set():
                print("E-STOP tetiklendi — gorev iptal.")
                break
            else:
                done = mission.step()
                if done:
                    print("GOREV TAMAMLANDI!")
                    break

            gcs.update_telemetry(mission.state)
            time.sleep(1.0 / LOOP_HZ)
    except KeyboardInterrupt:
        print("Ctrl+C — iptal.")
    finally:
        mission.abort()
        thr.stop()
        estop.stop()
        gcs.stop()
        log.close()
        print(f"Log: {log.path}")


def run_line_follow(thr, ori, depth):
    """Hat takibi gorevi (Gorev 1)."""
    from control.stabilizer import Stabilizer
    from sensors.camera import Camera
    from missions.line_follow import LineFollowMission
    from utils.logger import MissionLogger
    from hal.estop import EStopMonitor
    from control.mixer import mix

    stab   = Stabilizer(ori, depth)
    cam    = Camera()
    log    = MissionLogger("line_follow")
    estop  = EStopMonitor(thr)
    mission = LineFollowMission(stab, thr, cam, logger=log)

    cam.start()
    estop.start()

    gcs = WebGCS()
    gcs.start(thrusters=thr, stabilizer=stab, camera=cam, estop=estop)
    gcs.set_active_mission("LINE_FOLLOW", mission)

    print("Hat takibi gorevi baslatiliyor...")
    mission.start()
    try:
        while True:
            teleop_axes = gcs.get_teleop()
            if teleop_axes:
                thr.command(mix(**teleop_axes))
            elif estop.triggered.is_set():
                print("E-STOP tetiklendi — gorev iptal.")
                break
            else:
                done = mission.step()
                if done:
                    print("GOREV 1 TAMAMLANDI!")
                    break

            gcs.update_telemetry(mission.state)
            time.sleep(1.0 / LOOP_HZ)
    except KeyboardInterrupt:
        print("Ctrl+C — iptal.")
    finally:
        mission.abort()
        thr.stop()
        cam.stop()
        estop.stop()
        gcs.stop()
        log.close()
        print(f"Log: {log.path}")


def run_nav_mission(thr, ori, depth):
    """Otonom navigasyon gorevi (Gorev 2)."""
    from control.stabilizer import Stabilizer
    from sensors.camera import Camera
    from sensors.gps import GPS
    from sensors.ping_sonar import PingSonar
    from missions.nav_mission import NavMission
    from utils.logger import MissionLogger
    from hal.estop import EStopMonitor
    from control.mixer import mix

    stab   = Stabilizer(ori, depth)
    cam    = Camera()
    gps    = GPS()
    sonar  = PingSonar()
    log    = MissionLogger("nav_mission")
    estop  = EStopMonitor(thr)
    mission = NavMission(stab, thr, cam, gps, sonar, logger=log)

    cam.start()
    gps.start()
    sonar.start()
    estop.start()

    gcs = WebGCS()
    gcs.start(thrusters=thr, stabilizer=stab, camera=cam, estop=estop)
    gcs.set_active_mission("NAV_MISSION", mission)

    print("Navigasyon gorevi baslatiliyor...")
    mission.start()
    try:
        while True:
            teleop_axes = gcs.get_teleop()
            if teleop_axes:
                thr.command(mix(**teleop_axes))
            elif estop.triggered.is_set():
                print("E-STOP tetiklendi — gorev iptal.")
                break
            else:
                done = mission.step()
                if done:
                    print("GOREV 2 TAMAMLANDI!")
                    break

            extra = {}
            if sonar.measurement:
                extra["sonar_dist_mm"] = sonar.measurement[0]
            gcs.update_telemetry(mission.state, extra)
            time.sleep(1.0 / LOOP_HZ)
    except KeyboardInterrupt:
        print("Ctrl+C — iptal.")
    finally:
        mission.abort()
        thr.stop()
        cam.stop()
        gps.stop()
        sonar.stop()
        estop.stop()
        gcs.stop()
        log.close()
        print(f"Log: {log.path}")


# ──────────────────────────────────────── giris noktasi
if __name__ == "__main__":
    args = sys.argv[1:]

    if "--test-motors" in args:
        thr, _, _ = build_system()
        try:
            test_motors(thr)
        finally:
            thr.stop()
        sys.exit(0)

    if "--estop-test" in args:
        thr, _, _ = build_system()
        from hal.estop import EStopMonitor
        estop = EStopMonitor(thr)
        estop.start()
        print("E-stop GPIO aktif. 'e' + Enter ile yazilimsal tetikleme yap...")
        thr.arm()
        try:
            while True:
                cmd = input()
                if cmd.strip().lower() == "e":
                    estop.simulate_trigger()
                    print("Motorlar durdu mu? Evet ise e-stop calisiyor!")
                    break
        finally:
            thr.stop()
            estop.stop()
    if "--manual" in args or "--no-sensors" in args:
        from manual_drive import main as run_manual
        run_manual()
        sys.exit(0)

    # Gorev secimi
    mission_arg = "video"
    for a in args:
        if a.startswith("--mission="):
            mission_arg = a.split("=")[1]
        elif a == "--mission" and args.index(a) + 1 < len(args):
            mission_arg = args[args.index(a) + 1]

    thr, ori, depth = build_system()
    try:
        if mission_arg == "line":
            run_line_follow(thr, ori, depth)
        elif mission_arg == "nav":
            run_nav_mission(thr, ori, depth)
        else:
            run_video_demo(thr, ori, depth)
    finally:
        thr.stop()
