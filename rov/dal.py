#!/usr/bin/env python3
"""
EGE ROV — MANUEL DALIS SCRIPTI (sensorsuz, acik dongu).

Derinlik sensoru / IMU KULLANMAZ. Sadece dikey iticileri belirli bir gucte
belirli bir sure calistirip araci daldirir. Kalibrasyon gerektirmez.

Kullanim (Jetson, rov/ klasorunde):
    python3 dal.py                      # %30 guc, 5 sn dalis
    python3 dal.py --guc 0.4 --sure 8   # %40 guc, 8 sn
    python3 dal.py --ters               # arac YUKARI ciktiysa yonu cevir
    python3 dal.py --tut                # sure boyunca dal, sonra notre don (varsayilan)
    python3 dal.py --sim                # donanimsiz kuru test

GUVENLIK:
  - Ctrl+C her an motorlari notre ceker.
  - Script bitiminde motorlar her kosulda notre doner.
  - Ilk denemede --guc degerini dusuk tutun (0.25-0.30).
"""
import argparse
import sys
import time

from config import (LOOP_HZ, PWM_NEUTRAL_US, PWM_RANGE_US, PWM_DEADBAND_US,
                    THRUST_LIMIT, MOTOR_DIRECTION, ESC_ABS_MIN_US, ESC_ABS_MAX_US)
from control.mixer import mix

DIKEY_MOTORLAR = ("V_FL", "V_FR", "V_RL", "V_RR")


def parse_args():
    p = argparse.ArgumentParser(
        description="ROV'u acik dongu ile daldirir (sensorsuz).")
    p.add_argument("--guc", type=float, default=0.30,
                   help="dikey itki orani 0..1 (varsayilan: 0.30)")
    p.add_argument("--sure", type=float, default=5.0,
                   help="dalis suresi, saniye (varsayilan: 5)")
    p.add_argument("--ters", action="store_true",
                   help="arac dalmak yerine YUKARI ciktiysa bunu kullan")
    p.add_argument("--sim", action="store_true",
                   help="donanimsiz test (motorlara sinyal gitmez)")
    p.add_argument("--onay-yok", action="store_true",
                   help="ENTER onayini atla (otomatik calistirma icin)")
    args = p.parse_args()

    if not 0.0 < args.guc <= 1.0:
        p.error("--guc 0 ile 1 arasinda olmali")
    if args.sure <= 0:
        p.error("--sure sifirdan buyuk olmali")
    return args


def olu_bant_kontrol(guc):
    """Komutun olu bandin ustunde kalip kalmadigini dogrular."""
    sapma_us = guc * THRUST_LIMIT * PWM_RANGE_US
    if sapma_us <= PWM_DEADBAND_US:
        gereken = (PWM_DEADBAND_US / (THRUST_LIMIT * PWM_RANGE_US))
        print(f"[HATA] --guc {guc} -> notrden sadece {sapma_us:.0f}us sapma.")
        print(f"       Olu bant {PWM_DEADBAND_US}us, yani motorlar HIC donmez.")
        print(f"       En az --guc {gereken:.2f} kullanin.")
        sys.exit(1)
    return sapma_us


def main():
    args = parse_args()
    sapma_us = olu_bant_kontrol(args.guc)

    # heave sozlesmesi: +1 = DAL, -1 = YUKSEL
    heave = -args.guc if args.ters else args.guc
    yon_etiket = "YUKARI (--ters aktif)" if args.ters else "ASAGI"

    print("=" * 62)
    print("   EGE ROV — MANUEL DALIS (acik dongu, sensorsuz)")
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

    # Onizleme: tahmin degil, thrusters.py'nin gercekte yazacagi degerler.
    komut = mix(surge=0.0, yaw=0.0, heave=heave)
    hedef = {ad: Thrusters._to_us(deger) for ad, deger in komut.items()}

    print(f"\n  Itki yonu    : {yon_etiket}")
    print(f"  Guc          : %{args.guc * 100:.0f}  (THRUST_LIMIT={THRUST_LIMIT} ile "
          f"efektif %{args.guc * THRUST_LIMIT * 100:.0f})")
    print(f"  Sure         : {args.sure:g} sn")
    print(f"  Notr         : {PWM_NEUTRAL_US} us   "
          f"(ESC siniri {ESC_ABS_MIN_US}..{ESC_ABS_MAX_US} us)")
    print("  Gonderilecek PWM:")
    for ad in DIKEY_MOTORLAR:
        fark = hedef[ad] - PWM_NEUTRAL_US
        print(f"    {ad:5} = {hedef[ad]:4d} us ({fark:+4d})")
    for ad in ("H_L", "H_R"):
        print(f"    {ad:5} = {hedef[ad]:4d} us (notr, yatay hareket yok)")

    if all(us == PWM_NEUTRAL_US for ad, us in hedef.items() if ad in DIKEY_MOTORLAR):
        print("\n[HATA] Dikey motorlarin hepsi notrde kaliyor - hicbir sey olmaz.")
        print("       --guc degerini artirin.")
        sys.exit(1)

    if not args.onay_yok:
        input("\n--> ESC'lere GUC VER, bip sesleri bitsin, sonra ENTER...")

    print("\n[ARM] ESC'ler arm ediliyor (2 sn notr)...")
    thr.arm()

    print("Baslamaya 3 sn. Acil durdurma: Ctrl+C")
    for kalan in (3, 2, 1):
        print(f"  {kalan}...")
        time.sleep(1.0)

    dt = 1.0 / LOOP_HZ

    try:
        print(f"\n[DALIS] {args.sure:g} sn boyunca itki uygulaniyor...")
        t0 = time.monotonic()
        while True:
            gecen = time.monotonic() - t0
            if gecen >= args.sure:
                break
            thr.command(komut)
            sys.stdout.write(f"\r  gecen: {gecen:4.1f} / {args.sure:.1f} sn")
            sys.stdout.flush()
            time.sleep(dt)
        print("\n[DALIS] Sure doldu.")

        # Yumusak notre donus (slew limiti zaten sinirliyor, 1 sn yeterli)
        print("[NOTR] Motorlar yumusak sekilde notre cekiliyor...")
        sifir = mix(surge=0.0, yaw=0.0, heave=0.0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            thr.command(sifir)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[IPTAL] Ctrl+C algilandi.")
    finally:
        thr.stop()
        print(f"[GUVENLI] Tum motorlar notrde ({PWM_NEUTRAL_US} us).")

    print("\n" + "-" * 62)
    print(" SONUCU DEGERLENDIR:")
    if args.ters:
        print("   Arac ASAGI gittiyse -> yon dogru ama config TERS.")
        print("   config.py'de V_FL/V_FR/V_RL/V_RR isaretlerini cevirin,")
        print("   sonra --ters OLMADAN calistirin.")
    else:
        print("   Arac ASAGI gittiyse -> MOTOR_DIRECTION dikey kismi DOGRU.")
        print("   Arac YUKARI ciktiysa -> 'python3 dal.py --ters' ile dogrulayin.")
    print("   Arac egilerek/donerek daldiysa -> dikey motorlarin biri ters,")
    print("   her birini test_motors.py ile tek tek kontrol edin.")
    print("-" * 62)


if __name__ == "__main__":
    main()
