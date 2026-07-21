"""
Gorev simulasyonu: python3 run_sim.py
Video gosterimi gorevini sanal ROV'da kosar, rota izini ve sonucu yazar.
Gercek zaman beklemeden (sanal saat) calisir.
"""
import math
import time as _time

# ---- sanal saat: time.monotonic'i yamala (kod degismeden hizli kosum) ----
class FakeClock:
    """time.monotonic() yerine gecen sahte saat. Gercek zamanda beklemek
    yerine 't' degeri elle (adim adim) ilerletilerek simulasyon gercek
    zamandan cok daha hizli kosturulabilir."""
    def __init__(self):
        self.t = 0.0  # sanal zaman (saniye)

    def monotonic(self):
        """time.monotonic() ile ayni imzada - sanal zamani dondurur."""
        return self.t

clock = FakeClock()
_time.monotonic = clock.monotonic

from config import LOOP_HZ, MISSION
from hal.thrusters import Thrusters, MockBackend
from sensors.imu import MockImu, Orientation
from sensors.depth import MockDepth
from control.stabilizer import Stabilizer
from missions.video_demo import VideoDemoMission
from sim.simulator import RovSimulator
from utils.logger import MissionLogger

# time.sleep'i sanal saate bagla (arm() icindeki bekleme icin)
_time.sleep = lambda s: setattr(clock, "t", clock.t + s)


def run(current=(0.0, 0.0), label=""):
    """Bir senaryoyu bastan sona kosar: sanal ROV + gercek kontrol yiginini
    (Stabilizer/VideoDemoMission/Thrusters, MockBackend uzerinden) kurar,
    gorev bitene ya da max_t suresine kadar dongusunu ilerletir, sonra
    sonucu (basarili/basarisiz, bitis konumu) ekrana yazip rota izini cizer.
    current: (vx, vy) su akintisi m/s - bozucu etki testi icin."""
    sim = RovSimulator(current=current)
    backend = MockBackend()
    thr = Thrusters(backend)
    ori = Orientation(MockImu(sim))
    stab = Stabilizer(ori, MockDepth(sim))
    log = MissionLogger("sim")
    mission = VideoDemoMission(stab, thr, logger=log)

    dt = 1.0 / LOOP_HZ
    mission.start()
    max_t = 300.0
    while clock.t < max_t:
        clock.t += dt
        done = mission.step()
        sim.step(backend, dt)
        if done:
            break

    # ---- sonuc ----
    dist_to_start = math.hypot(sim.x, sim.y)
    print(f"--- {label or 'senaryo'} ---")
    print(f"durum        : {mission.state}")
    print(f"gorev suresi : {clock.t:.1f} sn")
    print(f"bitis konumu : x={sim.x:+.2f} m, y={sim.y:+.2f} m "
          f"(baslangica uzaklik {dist_to_start:.2f} m)")
    print(f"bitis derinlik: {sim.depth_m:.2f} m, heading: {sim.heading_deg:.1f}")
    ok = mission.state == "DONE" and dist_to_start < 0.5
    print("SONUC        :", "BASARILI - baslangic alaninda" if ok
          else "DIKKAT - alan disi ya da tamamlanamadi")
    print(f"log          : {log.path}")
    log.close()

    # rota izini ASCII ciz
    _plot(sim.trail)
    return ok


def _plot(trail, w=61, h=25):
    """ROV'un (x, y) izini w x h boyutunda basit bir ASCII harita olarak
    terminale basar. S=baslangic, B=bitis noktasi."""
    xs = [p[0] for p in trail]; ys = [p[1] for p in trail]
    x0, x1 = min(xs) - 0.5, max(xs) + 0.5
    y0, y1 = min(ys) - 0.5, max(ys) + 0.5
    grid = [[" "] * w for _ in range(h)]
    for x, y in trail:
        c = int((y - y0) / (y1 - y0) * (w - 1))
        r = int((x - x0) / (x1 - x0) * (h - 1))
        grid[h - 1 - r][c] = "."
    # baslangic
    c = int((0 - y0) / (y1 - y0) * (w - 1)); r = int((0 - x0) / (x1 - x0) * (h - 1))
    grid[h - 1 - r][c] = "S"
    c = int((trail[-1][1] - y0) / (y1 - y0) * (w - 1))
    r = int((trail[-1][0] - x0) / (x1 - x0) * (h - 1))
    grid[h - 1 - r][c] = "B"
    print("\nrota (S=start, B=bitis):")
    for row in grid:
        print("".join(row))


if __name__ == "__main__":
    run(label="akintisiz")
    print()
    run(current=(0.02, 0.03), label="hafif akintili (2-3 cm/s)")
