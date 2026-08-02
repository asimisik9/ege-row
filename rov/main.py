"""
EGE ROV - ana giris noktasi.

Kullanim (Jetson):
  python3 main.py                    # video demo gorevi + Web GCS (http://localhost:8000/)
  python3 main.py --mission video    # ayni
  python3 main.py --mission line     # hat takibi gorevi (Gorev 1)
  python3 main.py --mission nav      # navigasyon gorevi (Gorev 2)
  python3 main.py --test-motors      # tek tek motor testi
  python3 main.py --check-loop       # SORUN 2 dogrulamasi: dongu kac Hz donuyor?
  python3 main.py --sim              # SIM_MODE zorla True
  python3 main.py --estop-test       # e-stop GPIO tetikle

==============================================================================
BU DOSYADAKI DEGISIKLIKLER
==============================================================================
SORUN 2 (dongu 7.3 Hz'de doniyordu):
  1. Sensorler artik SensorHub icinde ayri thread'lerde okunuyor.
     Kontrol dongusu sensore hic dokunmuyor.
  2. `time.sleep(1/LOOP_HZ)` yerine DEADLINE tabanli LoopTimer.
     Eski yontem hesaplama suresini hesaba katmiyordu.
  3. `--check-loop` ile gercek frekans olculebiliyor (kabul kriteri H1).

GUVENLIK (yeni):
  WATCHDOG — bir sensor thread'i takilirsa kontrol dongusu ESKI veriyle
  ucmaya devam ederdi; bu tehlikeli. Artik SENSOR_STALE_S kadar taze veri
  gelmezse motorlar notre cekilir ve gorev iptal edilir.
"""
import sys
import time
import config

# --sim argumani: SIM_MODE'u override et
if "--sim" in sys.argv:
    config.SIM_MODE = True

from config import (LOOP_HZ, SIM_MODE, LOOP_WARN_HZ,
                    IMU_THREAD_HZ, DEPTH_THREAD_HZ, DEPTH_RATE_TAU,
                    SENSOR_STALE_S)
from comms.web_server import WebGCS
from utils.looptimer import LoopTimer


# ──────────────────────────────────────── yardimcilar
def build_system():
    """Donanim baglantilarini kurar ve sensor thread'lerini baslatir.

    Donus: (thrusters, orientation, depth_sensor, sensor_hub)
    sensor_hub.state -> Stabilizer'a verilecek paylasilan durum.
    """
    from hal.thrusters import Thrusters, PCA9685Backend, MockBackend
    from sensors.imu import Mpu9250, MockImu, Orientation
    from sensors.depth import Ms5837, MockDepth
    from sensors.state import SensorHub
    from sensors.camera import Camera
    from vision.grid_tracker import GridTracker

    if SIM_MODE:
        import threading
        from sim.simulator import RovSimulator
        sim = RovSimulator()
        backend = MockBackend()
        thr = Thrusters(backend)
        ori = Orientation(MockImu(sim))
        depth = MockDepth(sim)

        def _sim_loop():
            # DIKKAT: fizigi SABIT dt ile ilerletmek YANLIS olur.
            # time.sleep(dt) + islem yuku yuzunden gercekte dt'den fazla
            # zaman gecer; sabit adim kullanilirsa simulasyon saati gercek
            # zamandan YAVAS akar (olculen: ~0.8x) ve tum sure/hiz/cap
            # olcumleri sessizce yanilir. Gercek gecen sureyi kullaniyoruz.
            adim = 1.0 / (LOOP_HZ * 2)
            onceki = time.monotonic()
            while True:
                simdi = time.monotonic()
                gecen = min(0.05, simdi - onceki)   # donma sonrasi sicramayi kirp
                onceki = simdi
                if gecen > 0:
                    sim.step(backend, gecen)
                time.sleep(adim)

        threading.Thread(target=_sim_loop, daemon=True).start()
        print("[SIM] 3-DOF ROV fizik simulasyonu + mock sensorler aktif.")
    else:
        try:
            thr_backend = PCA9685Backend()
            print("[OK] PCA9685 motor surucu baglandi.")
        except Exception as e:
            print(f"[UYARI] PCA9685 baglanamadi ({e}), MockBackend kullaniliyor.")
            thr_backend = MockBackend()
        thr = Thrusters(thr_backend)

        try:
            imu_sensor = Mpu9250()
            print("[OK] MPU-9250 IMU baglandi.")
        except Exception as e:
            print(f"[UYARI] MPU-9250 baglanamadi ({e}), MockImu kullaniliyor.")
            imu_sensor = MockImu()
        ori = Orientation(imu_sensor)

        try:
            depth_sensor = Ms5837()
            print("[OK] MS5837 derinlik sensoru baglandi.")
        except Exception as e:
            print(f"[UYARI] MS5837 baglanamadi ({e}), MockDepth kullaniliyor.")
            depth_sensor = MockDepth()
            
        try:
            if getattr(config, "USE_VISION", False):
                cam = Camera()
                cam.start()
                grid_trk = GridTracker()
                print("[OK] Vision Grid Tracker aktif.")
            else:
                cam = None
                grid_trk = None
                print("[VISION] Kamera kapali (config.USE_VISION = False).")
        except Exception as e:
            print(f"[UYARI] Kamera veya GridTracker baslatilamadi: {e}")
            cam = None
            grid_trk = None
        depth = depth_sensor

    # SORUN 2: sensorler kendi thread'lerinde, kontrol dongusu bloklanmaz
    hub = SensorHub(ori, depth,
                    imu_hz=IMU_THREAD_HZ, depth_hz=DEPTH_THREAD_HZ,
                    depth_rate_tau=DEPTH_RATE_TAU, stale_s=SENSOR_STALE_S)
    
    if not SIM_MODE and cam and grid_trk:
        hub.enable_vision(cam, grid_trk)
        
    hub.start()
    return thr, ori, depth, hub


def test_motors(thr):
    """Her motoru sirayla 2 sn dusuk gazda dondur (pervane yonu kontrolu)."""
    from config import MOTOR_CHANNELS
    print("Motor testi basliyor. Pervaneler takili degilken ya da su icindeyken yap!")
    print("NOT: yon dogrulamasi icin daha iyi arac -> python3 verify_directions.py")
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


def check_loop(hub, seconds=10.0):
    """SORUN 2 DOGRULAMASI — kontrol dongusu gercekte kac Hz donuyor?

    Motorlara hic komut gondermez; sadece stabilizer'in yaptigi is kadar
    is yapip frekansi olcer. Kabul kriteri H1: >= 30 Hz.
    """
    from control.stabilizer import Stabilizer
    stab = Stabilizer(hub.ori, hub.depth, state=hub.state)
    stab.set_targets(depth_m=0.5, heading_deg=0.0)
    lt = LoopTimer(LOOP_HZ, warn_hz=None, name="check")
    print(f"\n{seconds:.0f} saniye boyunca dongu frekansi olculuyor...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        lt.tick()
        stab.compute(surge=0.0)          # gercek gorevdeki ile ayni is
        lt.sleep()
    s = hub.state.snapshot()
    print("\n" + "=" * 66)
    print(f"  {lt.report()}")
    print(f"  IMU thread   : {s.imu_hz:.1f} Hz   (hata sayisi {hub.imu_errors})")
    print(f"  Derinlik thr.: {s.depth_hz:.1f} Hz   (hata sayisi {hub.depth_errors})")
    print("-" * 66)
    if lt.hz >= 30.0:
        print(f"  H1 KABUL KRITERI: GECTI  ({lt.hz:.1f} Hz >= 30 Hz)")
    else:
        print(f"  H1 KABUL KRITERI: KALDI  ({lt.hz:.1f} Hz < 30 Hz)")
        print("  -> config.DEPTH_OSR dusur, LOG_EVERY_N arttir, kamera/GCS kapat.")
    print("=" * 66)


# ──────────────────────────────────────── ortak gorev kosturucu
def _run_mission_loop(mission, thr, stab, estop, gcs, hub, log, ad,
                      mission_factory=None):
    """Tum gorevler icin ortak ana dongu: deadline zamanlayici + watchdog.

    OPERATOR MODU (yeni — bkz. control/operator.py)
    ------------------------------------------------
    Eskiden bu dongu KOSULSUZ `mission.step()` cagiriyordu. Gorevlerin step()
    metodu ise her cagrilista `stab.set_targets(...)` yaziyor:

        missions/video_demo.py:118   set_targets(depth_m=M["target_depth_m"])
        missions/line_follow.py:108  set_targets(depth_m=LINE_TARGET_DEPTH)
        missions/nav_mission.py:119  set_targets(depth_m=NAV_TARGET_DEPTH)

    Bu dongu 50 Hz dondugu icin yer istasyonundan verilen hedef 20 MILISANIYE
    icinde siliniyordu. "Hedef belirleme calismiyor" sikayetinin sebebi buydu.

    Artik hedefin sahibi ACIK: mission.step() SADECE AUTO modunda calisir.
    HOLD / HOVER / RATE / TELEOP modlarinda hedefleri operator yonetir ve
    hicbir sey ustune yazmaz.
    """
    from control.mixer import mix
    lt = LoopTimer(LOOP_HZ, warn_hz=LOOP_WARN_HZ, name="control")
    op = gcs.operator if gcs else None

    # Dongu istatistiklerini, sensor sagligini ve gorev listesini arayuze bagla
    if gcs:
        from comms.web_server import g_ctx
        g_ctx.loop_timer = lt
        if hub is not None:
            g_ctx.hub = hub
        if mission_factory:
            g_ctx.mission_factory = mission_factory

    print(f"{ad} baslatiliyor...")
    mission.start()
    last_print = 0.0
    try:
        while True:
            now = lt.tick()

            # --- WATCHDOG: sensor verisi bayatladiysa ucmaya devam etme ---
            if hub is not None and not hub.healthy():
                print("[WATCHDOG] Sensor verisi bayat! Motorlar durduruluyor.")
                if log:
                    log.event("WATCHDOG: sensor verisi bayat -> ABORT")
                mission.abort()
                thr.stop()
                break

            if estop and estop.triggered.is_set():
                print("E-STOP tetiklendi — gorev iptal.")
                if log:
                    log.event("E-STOP")
                mission.abort()
                thr.stop()
                break

            # --- Web'den gorev degistirme istegi ---
            if gcs and mission_factory:
                istek = gcs.take_mission_request()
                if istek and istek in mission_factory:
                    mission.abort()
                    mission = mission_factory[istek]()
                    mission.start()
                    gcs.set_active_mission(istek.upper(), mission)
                    print(f"[GCS] Web uzerinden yeni gorev: {istek}")
                    if log:
                        log.event(f"WEB: gorev baslatildi -> {istek}")

            # Sensorleri dongu basina TEK KEZ orneklle (SORUN 8).
            snap = stab.sample(now)

            if op is not None:
                axes, done, durum = op.step(stab, mission, thr, mix, now=now)
                # Adim cevabi kaydi (aktif degilse hicbir sey yapmaz)
                op.recorder.sample(snap, stab, op.rate_target, now=now)
            else:
                done = bool(mission.step())
                axes = getattr(mission, "last_axes", None)
                durum = mission.state

            if done:
                print("GOREV TAMAMLANDI!")
                break

            # 1 Hz terminal logu
            if now - last_print >= 1.0:
                last_print = now
                d = getattr(snap, "depth_m", 0.0)
                h = getattr(snap, "heading", 0.0)
                mod = op.get()["mode"] if op else "-"
                print(f"[{durum:<10}|{mod:<6}] derinlik={d:5.2f} m  heading={h:6.1f} deg")

            if gcs:
                gcs.update_telemetry(durum)
            lt.sleep()
    except KeyboardInterrupt:
        print("Ctrl+C — iptal.")
    finally:
        print(lt.report())
        if log:
            log.event(lt.report())
        mission.abort()
        thr.stop()


def run_video_demo(thr, ori, depth, hub):
    from comms.web_server import WebGCS
    from control.stabilizer import Stabilizer
    from utils.logger import MissionLogger
    from missions.video_demo import VideoDemoMission
    from missions.line_follow import LineFollowMission
    from missions.nav_mission import NavMission
    from missions.simple_mission import SimpleMission
    from hal.estop import EStopMonitor

    stab = Stabilizer(ori, depth, state=hub.state if hub else None)
    log = MissionLogger("video_demo")
    estop = EStopMonitor(thr)
    mission = VideoDemoMission(stab, thr, logger=log)

    # Web'den baslatilabilecek gorevler (kamera gerektirmeyenler)
    factory = {
        "video": lambda: VideoDemoMission(stab, thr, logger=log),
        "line": lambda: LineFollowMission(stab, thr, logger=log),
        "nav": lambda: NavMission(stab, thr, logger=log),
        "simple": lambda: SimpleMission(stab, thr, logger=log),
    }

    gcs = WebGCS()
    gcs.start(thrusters=thr, stabilizer=stab, estop=estop, hub=hub,
              mission_factory=factory)
    gcs.set_active_mission("VIDEO_DEMO", mission)
    estop.start()

    try:
        _run_mission_loop(mission, thr, stab, estop, gcs, hub, log,
                          "Video demo gorevi", mission_factory=factory)
    finally:
        if mission.surface_violations:
            print(f"[UYARI] Yuzey ihlali ornegi: {mission.surface_violations} "
                  f"(H4 kabul kriteri: 0 olmali)")
        estop.stop()
        gcs.stop()
        log.close()
        print(f"Log: {log.path}")
        print(f"Analiz: python3 tools/analyze_log.py {log.path}")


def run_line_follow(thr, ori, depth, hub):
    """Hat takibi gorevi (Gorev 1)."""
    from control.stabilizer import Stabilizer
    from sensors.camera import Camera
    from missions.line_follow import LineFollowMission
    from utils.logger import MissionLogger
    from hal.estop import EStopMonitor

    stab = Stabilizer(ori, depth, state=hub.state if hub else None)
    cam = hub.camera if hub else None
    log = MissionLogger("line_follow")
    estop = EStopMonitor(thr)
    mission = LineFollowMission(stab, thr, cam, logger=log)

    estop.start()
    factory = {
        "line": lambda: LineFollowMission(stab, thr, cam, logger=log),
        "video": lambda: __import__("missions.video_demo", fromlist=["x"])
                         .VideoDemoMission(stab, thr, logger=log),
    }
    gcs = WebGCS()
    gcs.start(thrusters=thr, stabilizer=stab, camera=cam, estop=estop, hub=hub,
              mission_factory=factory)
    gcs.set_active_mission("LINE_FOLLOW", mission)

    try:
        _run_mission_loop(mission, thr, stab, estop, gcs, hub, log,
                          "Hat takibi gorevi", mission_factory=factory)
    finally:
        cam.stop()
        estop.stop()
        gcs.stop()
        log.close()
        print(f"Log: {log.path}")


def run_nav_mission(thr, ori, depth, hub):
    """Otonom navigasyon gorevi (Gorev 2)."""
    from control.stabilizer import Stabilizer
    from sensors.camera import Camera
    from sensors.gps import GPS
    from sensors.ping_sonar import PingSonar
    from missions.nav_mission import NavMission
    from utils.logger import MissionLogger
    from hal.estop import EStopMonitor

    stab = Stabilizer(ori, depth, state=hub.state if hub else None)
    cam = hub.camera if hub else None
    gps, sonar = GPS(), PingSonar()
    log = MissionLogger("nav_mission")
    estop = EStopMonitor(thr)
    mission = NavMission(stab, thr, cam, gps, sonar, logger=log)

    gps.start(); sonar.start(); estop.start()
    factory = {"nav": lambda: NavMission(stab, thr, cam, gps, sonar, logger=log)}
    gcs = WebGCS()
    # gps/sonar da verilir: konum ve mesafe artik arayuzde canli gorunur
    gcs.start(thrusters=thr, stabilizer=stab, camera=cam, estop=estop, hub=hub,
              gps=gps, sonar=sonar, mission_factory=factory)
    gcs.set_active_mission("NAV_MISSION", mission)

    try:
        _run_mission_loop(mission, thr, stab, estop, gcs, hub, log,
                          "Navigasyon gorevi", mission_factory=factory)
    finally:
        cam.stop(); gps.stop(); sonar.stop()
        estop.stop()
        gcs.stop()
        log.close()
        print(f"Log: {log.path}")


# ──────────────────────────────────────── giris noktasi
if __name__ == "__main__":
    args = sys.argv[1:]

    if "--manual" in args or "--no-sensors" in args:
        from manual_drive import main as run_manual
        run_manual()
        sys.exit(0)

    if "--test-motors" in args:
        thr, _, _, hub = build_system()
        try:
            test_motors(thr)
        finally:
            thr.stop(); hub.stop()
        sys.exit(0)

    if "--check-loop" in args:
        thr, _, _, hub = build_system()
        try:
            check_loop(hub)
        finally:
            thr.stop(); hub.stop()
        sys.exit(0)

    if "--estop-test" in args:
        thr, _, _, hub = build_system()
        from hal.estop import EStopMonitor
        estop = EStopMonitor(thr)
        estop.start()
        print("E-stop GPIO aktif. 'e' + Enter ile yazilimsal tetikleme yap...")
        thr.arm()
        try:
            while True:
                if input().strip().lower() == "e":
                    estop.simulate_trigger()
                    print("Motorlar durdu mu? Evet ise e-stop calisiyor!")
                    break
        finally:
            thr.stop(); estop.stop(); hub.stop()
        sys.exit(0)

    # Gorev secimi
    mission_arg = "video"
    for i, a in enumerate(args):
        if a.startswith("--mission="):
            mission_arg = a.split("=", 1)[1]
        elif a == "--mission" and i + 1 < len(args):
            mission_arg = args[i + 1]

    thr, ori, depth, hub = build_system()
    try:
        if mission_arg == "line":
            run_line_follow(thr, ori, depth, hub)
        elif mission_arg == "nav":
            run_nav_mission(thr, ori, depth, hub)
        elif mission_arg == "simple":
            run_video_demo(thr, ori, depth, hub) # run_video_demo contains the factory and UI logic
        else:
            run_video_demo(thr, ori, depth, hub)
    finally:
        thr.stop()
        hub.stop()
