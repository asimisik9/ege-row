"""
EGE ROV - Sadece dalis testi: ileri gitmeden 1 metre bat ve derinligi tut.

Kullanim (Jetson):
  python3 dive_1m.py            # 1 m'ye dal, hold_time_s kadar tut, cik
  python3 dive_1m.py --sim      # simulasyonla test

Tum parametreler config.py'den okunur (PID_DEPTH, MISSION toleranslari vb).
Surge/yaw komutu HIC verilmez -> arac oldugu yerde sadece dikey hareket eder.
"""
import sys
import time

import config
if "--sim" in sys.argv:
    config.SIM_MODE = True

from config import LOOP_HZ, MISSION
from main import build_system
from control.stabilizer import Stabilizer
from control.mixer import mix

# ---- bu testin parametreleri (config degerlerinden turetilir) ----
TARGET_DEPTH_M = 1.5                      # hedef: 1 metre
DEPTH_TOL_M    = MISSION["depth_tol_m"]    # derinlik "tamam" toleransi
DIVE_TIMEOUT_S = 20.0 # dalis icin max sure
START_DELAY_S  = MISSION["start_delay_s"]  # baslamadan once geri sayim
HOLD_TIME_S    = 30.0                      # hedefte derinlik tutma suresi


def main():
    thr, ori, depth = build_system()
    stab = Stabilizer(ori, depth)

    try:
        # 1) yuzey referansi: mevcut basinc = 0 m
        depth.zero_at_surface()
        print(f"Yuzey referansi alindi. Hedef: {TARGET_DEPTH_M} m")

        # 2) geri sayim
        for i in range(int(START_DELAY_S), 0, -1):
            print(f"  baslamaya {i}...")
            time.sleep(1.0)

        thr.arm()
        stab.set_targets(depth_m=TARGET_DEPTH_M)

        # 3) DAL: sadece heave (dikey) — surge=0, yaw PID hedefsiz oldugu icin 0
        t0 = time.monotonic()
        reached_t = None
        dt = 1.0 / LOOP_HZ

        while True:
            axes = stab.compute(surge=0.0)   # ileri komut YOK
            thr.command(mix(**axes))

            d = depth.read_depth_m()
            err = stab.depth_error()

            if abs(err) < DEPTH_TOL_M:
                if reached_t is None:
                    reached_t = time.monotonic()
                    print(f"Hedef derinlige ulasildi: {d:.2f} m — {HOLD_TIME_S:.0f} sn tutuluyor")
                elif time.monotonic() - reached_t >= HOLD_TIME_S:
                    print("Tutma suresi tamamlandi.")
                    break
            else:
                if reached_t is None and time.monotonic() - t0 > DIVE_TIMEOUT_S:
                    print(f"ZAMAN ASIMI ({DIVE_TIMEOUT_S:.0f} sn): derinlik {d:.2f} m, hata {err:+.2f} m")
                    break

            # saniyede ~2 kez durum yazdir
            if int((time.monotonic() - t0) * 2) != int((time.monotonic() - t0 - dt) * 2):
                print(f"  derinlik={d:5.2f} m  hata={err:+5.2f} m  heave={axes['heave']:+.2f}")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("Ctrl+C — iptal.")
    finally:
        thr.stop()
        print("Motorlar notrde. Test bitti.")


if __name__ == "__main__":
    main()