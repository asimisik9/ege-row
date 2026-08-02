#!/usr/bin/env python3
"""
EGE ROV — DUZ GIT (acik dongu, PID YOK, sensor YOK).

Geri sayimdan sonra araci belirtilen sure boyunca (varsayilan 15 sn)
sabit gucle DUZ ILERI surer. Baska hicbir sey yapmaz.

Kullanim (Jetson, rov/ klasorunde):
    python3 missions/duz_git.py                     # 5 sn geri sayim + 15 sn ileri
    python3 missions/duz_git.py --sure 20           # 20 sn ileri
    python3 missions/duz_git.py --geri-sayim 10     # geri sayimi 10 sn yap
    python3 missions/duz_git.py --guc 0.40          # itki gucu %40
    python3 missions/duz_git.py --dal 0.25          # sabit dalis itkisi tut
    python3 missions/duz_git.py --ters              # ileri geri cikiyorsa
    python3 missions/duz_git.py --sim               # donanimsiz kuru test

NOT: PID / IMU / pusula KULLANILMAZ. Arac motor dengesizligi nedeniyle
     hafifce suruklenebilir; bu normaldir.

GUVENLIK: Ctrl+C ve script sonu her kosulda motorlari notre ceker.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (LOOP_HZ, PWM_NEUTRAL_US, PWM_RANGE_US, PWM_DEADBAND_US,
                    THRUST_LIMIT, ESC_ABS_MIN_US, ESC_ABS_MAX_US)
from control.mixer import mix


def parse_args():
    p = argparse.ArgumentParser(
        description="ROV'u geri sayimdan sonra duz ileri surer (PID yok).")
    p.add_argument("--sure", type=float, default=15.0,
                   help="ileri gitme suresi, saniye (varsayilan: 15)")
    p.add_argument("--geri-sayim", type=float, default=5.0, dest="geri_sayim",
                   help="baslamadan onceki geri sayim, saniye (varsayilan: 5)")
    p.add_argument("--guc", type=float, default=0.30,
                   help="ileri itki orani 0..1 (varsayilan: 0.30)")
    p.add_argument("--dal", type=float, default=0.0,
                   help="hareket boyunca sabit dalis itkisi 0..1 (varsayilan: 0)")
    p.add_argument("--ters", action="store_true",
                   help="ileri yerine geri gidiyorsa surge yonunu cevir")
    p.add_argument("--sim", action="store_true",
                   help="donanimsiz test (motorlara sinyal gitmez)")
    p.add_argument("--onay-yok", action="store_true",
                   help="ENTER onayini atla (otomatik calistirma icin)")
    args = p.parse_args()

    if not 0.0 < args.guc <= 1.0:
        p.error("--guc 0 ile 1 arasinda olmali")
    if not 0.0 <= args.dal <= 1.0:
        p.error("--dal 0 ile 1 arasinda olmali")
    if args.sure <= 0:
        p.error("--sure sifirdan buyuk olmali")
    if args.geri_sayim < 0:
        p.error("--geri-sayim negatif olamaz")
    return args


def olu_bant_kontrol(ad, guc):
    """Komutun olu bandin ustunde kalip kalmadigini dogrular."""
    sapma_us = guc * THRUST_LIMIT * PWM_RANGE_US
    if sapma_us <= PWM_DEADBAND_US:
        gereken = PWM_DEADBAND_US / (THRUST_LIMIT * PWM_RANGE_US)
        print(f"[HATA] {ad} {guc} -> notrden sadece {sapma_us:.0f}us sapma.")
        print(f"       Olu bant {PWM_DEADBAND_US}us, yani motorlar HIC donmez.")
        print(f"       En az {gereken:.2f} kullanin.")
        sys.exit(1)
    return sapma_us


def geri_say(saniye):
    """Ekranda geri sayim gosterir. Ctrl+C ile iptal edilebilir."""
    if saniye <= 0:
        return
    print(f"\n[GERI SAYIM] {saniye:g} sn — iptal icin Ctrl+C")
    t0 = time.monotonic()
    while True:
        kalan = saniye - (time.monotonic() - t0)
        if kalan <= 0:
            break
        sys.stdout.write(f"\r  {kalan:5.1f} sn ...   ")
        sys.stdout.flush()
        time.sleep(0.1)
    print("\r  BASLIYOR!        ")


def notre_don(thr, heave, sure, dt):
    """Yatay ekseni sifirlar; dalis itkisi verilmisse korur."""
    komut = mix(surge=0.0, yaw=0.0, heave=heave)
    t0 = time.monotonic()
    while time.monotonic() - t0 < sure:
        thr.command(komut)
        time.sleep(dt)


def main():
    args = parse_args()
    olu_bant_kontrol("--guc", args.guc)
    if args.dal > 0:
        olu_bant_kontrol("--dal", args.dal)

    surge = args.guc * (-1.0 if args.ters else 1.0)
    heave = args.dal

    print("=" * 62)
    print("   EGE ROV — DUZ GIT (acik dongu, PID yok)")
    print("=" * 62)

    if args.sim:
        print("[MOD] SIMULASYON — motorlara gercek sinyal GITMEZ.")
        from hal.thrusters import Thrusters, MockBackend
        backend = MockBackend()
    else:
        print("[MOD] GERCEK DONANIM")
        from hal.thrusters import Thrusters, PCA9685Backend
        try:
            backend = PCA9685Backend()
        except Exception as e:
            print(f"[HATA] PCA9685'e baglanilamadi: {e}")
            sys.exit(1)

    thr = Thrusters(backend)
    dt = 1.0 / LOOP_HZ

    print(f"\n  Yon          : {'GERI' if args.ters else 'ILERI'} (duz)")
    print(f"  Guc          : %{args.guc * 100:.0f}  (efektif "
          f"%{args.guc * THRUST_LIMIT * 100:.0f})")
    print(f"  Sure         : {args.sure:g} sn")
    print(f"  Geri sayim   : {args.geri_sayim:g} sn")
    print(f"  Notr         : {PWM_NEUTRAL_US} us   "
          f"(ESC siniri {ESC_ABS_MIN_US}..{ESC_ABS_MAX_US} us)")
    if args.dal > 0:
        print(f"  Dalis itkisi : %{args.dal * 100:.0f} (hareket boyunca sabit)")
    else:
        print("  Dalis itkisi : YOK — arac yuzeye cikabilir (--dal ile ekleyin)")

    if not args.onay_yok:
        input("\n--> ESC'lere GUC VER, bip sesleri bitsin, sonra ENTER...")

    print("\n[ARM] ESC'ler arm ediliyor (2 sn notr)...")
    thr.arm()

    komut = mix(surge=surge, yaw=0.0, heave=heave)
    us = {m: type(thr)._to_us(v) for m, v in komut.items()}
    print(f"  H_L={us['H_L']}us  H_R={us['H_R']}us  |  "
          f"V_*={us['V_FL']},{us['V_FR']},{us['V_RL']},{us['V_RR']} us")

    try:
        geri_say(args.geri_sayim)

        print(f"\n[DUZ GIT] surge={surge:+.2f} heave={heave:+.2f} "
              f"({args.sure:g} sn) — durdurmak icin Ctrl+C")
        t0 = time.monotonic()
        while True:
            gecen = time.monotonic() - t0
            if gecen >= args.sure:
                break
            thr.command(komut)
            sys.stdout.write(f"\r  gecen: {gecen:5.1f} / {args.sure:.1f} sn")
            sys.stdout.flush()
            time.sleep(dt)
        print()

        print("[NOTR] Motorlar yumusak sekilde notre cekiliyor...")
        notre_don(thr, 0.0, 1.0, dt)

    except KeyboardInterrupt:
        print("\n[IPTAL] Ctrl+C algilandi.")
    finally:
        thr.stop()
        print(f"[GUVENLI] Tum motorlar notrde ({PWM_NEUTRAL_US} us).")


if __name__ == "__main__":
    main()
