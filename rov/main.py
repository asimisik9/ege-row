"""
EGE ROV - cihaz uzerinde ana giris noktasi.
Kullanim (Jetson):  python3 main.py            # gorevi baslatir (10 sn geri sayim)
                    python3 main.py --test-motors  # tek tek motor testi (dusuk gaz)
"""
import sys
import time
from config import LOOP_HZ, SIM_MODE
from hal.thrusters import Thrusters, MockBackend
from control.stabilizer import Stabilizer
from missions.video_demo import VideoDemoMission
from utils.logger import MissionLogger


def build_system():
    """Gercek donanim baglantilarini kurar: motorlari (PCA9685), IMU'yu ve
    derinlik sensorunu olusturup Thrusters + Stabilizer nesnelerini dondurur.
    SIM_MODE acikken (varsayilan) cihazda calismayi engeller."""
    if SIM_MODE:
        raise SystemExit("config.SIM_MODE=True. Cihazda calistirmak icin False yap. "
                         "Simulasyon icin: python3 run_sim.py")
    from hal.thrusters import PCA9685Backend
    from sensors.imu import Mpu9250, Orientation
    from sensors.depth import Ms5837
    thr = Thrusters(PCA9685Backend())
    ori = Orientation(Mpu9250())
    depth = Ms5837()
    return thr, Stabilizer(ori, depth)


def test_motors(thr):
    """Her motoru sirayla 2 sn dusuk gazda dondur (pervane yonu kontrolu)."""
    from config import MOTOR_CHANNELS
    thr.arm()
    for name in MOTOR_CHANNELS:
        print(f"motor {name} ...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            # sadece test edilen motora dusuk gaz (0.15), digerleri notr
            thr.command({n: (0.15 if n == name else 0.0) for n in MOTOR_CHANNELS})
            time.sleep(1.0 / LOOP_HZ)
        thr.stop()      # motorlari notre cek
        thr.arm()        # bir sonraki motor icin tekrar arm et
        time.sleep(1.0)  # motorlar arasi bekleme
    thr.stop()
    print("bitti.")


def run_mission():
    """Video gosterimi gorevini gercek donanimda baslatir ve gorev bitene
    (ya da Ctrl+C ile iptal edilene) kadar LOOP_HZ frekansinda step() cagirir.
    Her durumda (hata/iptal dahil) motorlari durdurup logu kapatir."""
    thr, stab = build_system()
    log = MissionLogger("video_demo")
    mission = VideoDemoMission(stab, thr, logger=log)
    print("gorev baslatiliyor (geri sayim config.MISSION['start_delay_s'])...")
    mission.start()
    try:
        while True:
            done = mission.step()          # bir kontrol dongusu ilerlet
            if done:
                print("GOREV TAMAMLANDI")
                break
            time.sleep(1.0 / LOOP_HZ)       # dongu frekansini sabit tut
    except KeyboardInterrupt:
        print("iptal - motorlar durduruluyor")
    finally:
        mission.abort()   # motorlari notre cek, durumu ABORT yap
        thr.stop()        # guvenlik icin ayrica dogrudan da durdur
        log.close()
        print(f"log: {log.path}")


if __name__ == "__main__":
    if "--test-motors" in sys.argv:
        thr, _ = build_system()
        try:
            test_motors(thr)
        finally:
            thr.stop()
    else:
        run_mission()
