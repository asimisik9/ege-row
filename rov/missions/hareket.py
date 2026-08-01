#!/usr/bin/env python3
"""
EGE ROV — MANUEL HAREKET SCRIPTI (sensorsuz, acik dongu).

IMU / pusula KULLANMAZ. Yatay iticileri belirli bir gucte surerek
ileri / geri / sag / sol hareketleri sirayla uygular.

Kullanim (Jetson, rov/ klasorunde):
    python3 hareket.py                        # tam dizi: ileri, geri, sag, sol
    python3 hareket.py --hareket ileri        # tek hareket
    python3 hareket.py --guc 0.35 --sure 4    # guc ve sure ayari
    python3 hareket.py --dal 0.25             # hareket boyunca dalis itkisi tut
    python3 hareket.py --ters                 # ileri/geri ters ciktiysa
    python3 hareket.py --ters-donus           # sag/sol ters ciktiysa
    python3 hareket.py --sim                  # donanimsiz kuru test

NOT: --dal olmadan arac pozitif yuzerlik nedeniyle hareket sirasinda
     yuzeye cikabilir. Once dal.py ile daldirip hemen bu scripti
     calistirin ya da --dal ile sabit dalis itkisi verin.

GUVENLIK: Ctrl+C ve script sonu her kosulda motorlari notre ceker.
"""
import argparse
import sys
import time

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (LOOP_HZ, PWM_NEUTRAL_US, PWM_RANGE_US, PWM_DEADBAND_US,
                    THRUST_LIMIT, ESC_ABS_MIN_US, ESC_ABS_MAX_US)
from control.mixer import mix

# hareket adi -> (surge_isareti, yaw_isareti)
HAREKETLER = {
    "ileri": (+1.0, 0.0),
    "geri":  (-1.0, 0.0),
    "sag":   (0.0, +1.0),
    "sol":   (0.0, -1.0),
}
VARSAYILAN_DIZI = ("ileri", "geri", "sag", "sol")


def parse_args():
    p = argparse.ArgumentParser(
        description="ROV'u acik dongu ile ileri/geri/sag/sol hareket ettirir.")
    p.add_argument("--hareket", choices=sorted(HAREKETLER), default=None,
                   help="tek bir hareket calistir (varsayilan: hepsi sirayla)")
    p.add_argument("--guc", type=float, default=0.30,
                   help="yatay itki orani 0..1 (varsayilan: 0.30)")
    p.add_argument("--sure", type=float, default=3.0,
                   help="her hareketin suresi, saniye (varsayilan: 3)")
    p.add_argument("--ara", type=float, default=2.0,
                   help="hareketler arasi notr bekleme, saniye (varsayilan: 2)")
    p.add_argument("--dal", type=float, default=0.0,
                   help="hareket boyunca sabit dalis itkisi 0..1 (varsayilan: 0)")
    p.add_argument("--ters", action="store_true",
                   help="ileri/geri ters ciktiysa surge yonunu cevir")
    p.add_argument("--ters-donus", action="store_true",
                   help="sag/sol ters ciktiysa yaw yonunu cevir")
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
    if args.ara < 0:
        p.error("--ara negatif olamaz")
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


def hareketi_uygula(thr, ad, surge, yaw, heave, sure, dt):
    """Tek bir hareketi verilen sure boyunca uygular."""
    komut = mix(surge=surge, yaw=yaw, heave=heave)
    us = {m: type(thr)._to_us(v) for m, v in komut.items()}
    print(f"\n[{ad.upper()}] surge={surge:+.2f} yaw={yaw:+.2f} heave={heave:+.2f} "
          f"({sure:g} sn)")
    print(f"  H_L={us['H_L']}us  H_R={us['H_R']}us  |  "
          f"V_*={us['V_FL']},{us['V_FR']},{us['V_RL']},{us['V_RR']} us")
    t0 = time.monotonic()
    while True:
        gecen = time.monotonic() - t0
        if gecen >= sure:
            break
        thr.command(komut)
        sys.stdout.write(f"\r  gecen: {gecen:4.1f} / {sure:.1f} sn")
        sys.stdout.flush()
        time.sleep(dt)
    print()


def notre_don(thr, heave, sure, dt):
    """Yatay eksenleri sifirlar; dalis itkisi varsa korur."""
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

    dizi = (args.hareket,) if args.hareket else VARSAYILAN_DIZI
    surge_isaret = -1.0 if args.ters else 1.0
    yaw_isaret = -1.0 if args.ters_donus else 1.0
    heave = args.dal  # +1 = dal

    print("=" * 62)
    print("   EGE ROV — MANUEL HAREKET (acik dongu, sensorsuz)")
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

    print(f"\n  Dizi         : {' -> '.join(dizi)}")
    print(f"  Guc          : %{args.guc * 100:.0f}  (efektif "
          f"%{args.guc * THRUST_LIMIT * 100:.0f})")
    print(f"  Her hareket  : {args.sure:g} sn,  arada {args.ara:g} sn notr")
    print(f"  Notr         : {PWM_NEUTRAL_US} us   "
          f"(ESC siniri {ESC_ABS_MIN_US}..{ESC_ABS_MAX_US} us)")
    if args.dal > 0:
        print(f"  Dalis itkisi : %{args.dal * 100:.0f} (hareket boyunca sabit)")
    else:
        print("  Dalis itkisi : YOK — arac yuzeye cikabilir (--dal ile ekleyin)")
    if args.ters:
        print("  --ters       : ileri/geri cevrildi")
    if args.ters_donus:
        print("  --ters-donus : sag/sol cevrildi")

    if not args.onay_yok:
        input("\n--> ESC'lere GUC VER, bip sesleri bitsin, sonra ENTER...")

    print("\n[ARM] ESC'ler arm ediliyor (2 sn notr)...")
    thr.arm()

    print("Baslamaya 3 sn. Acil durdurma: Ctrl+C")
    for kalan in (3, 2, 1):
        print(f"  {kalan}...")
        time.sleep(1.0)

    try:
        for i, ad in enumerate(dizi):
            s_bir, y_bir = HAREKETLER[ad]
            hareketi_uygula(
                thr, ad,
                surge=s_bir * args.guc * surge_isaret,
                yaw=y_bir * args.guc * yaw_isaret,
                heave=heave, sure=args.sure, dt=dt,
            )
            if i < len(dizi) - 1 and args.ara > 0:
                print(f"  ...{args.ara:g} sn notr...")
                notre_don(thr, heave, args.ara, dt)

        print("\n[NOTR] Motorlar yumusak sekilde notre cekiliyor...")
        notre_don(thr, 0.0, 1.0, dt)

    except KeyboardInterrupt:
        print("\n[IPTAL] Ctrl+C algilandi.")
    finally:
        thr.stop()
        print(f"[GUVENLI] Tum motorlar notrde ({PWM_NEUTRAL_US} us).")

    print("\n" + "-" * 62)
    print(" SONUCU DEGERLENDIR:")
    print("   'ileri' geri gittiyse VE sag/sol da ters ise")
    print("     -> config.py MOTOR_DIRECTION'da H_L ve H_R isaretlerini cevirin.")
    print("   'ileri' dogru ama sag/sol yer degistirmisse")
    print("     -> MOTOR_CHANNELS'ta H_L ve H_R kanal numaralarini takas edin.")
    print("   Arac ileri giderken suruklenip donuyorsa")
    print("     -> iki yatay motorun itkisi esit degil, ESC kalibrasyonu gerekir")
    print("        (calibrate_escs.py).")
    print("-" * 62)


if __name__ == "__main__":
    main()
