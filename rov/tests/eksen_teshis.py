#!/usr/bin/env python3
"""
EGE ROV — JIROSKOP EKSEN TESHISI (motorlara DOKUNMAZ, kuru masada calisir).

NE ICIN VAR
===========
Olculen belirti: arac elle TAM TUR (360 derece) cevrildiginde
sensors/imu.py'nin heading'i sadece ~58 derece degisti. Havuzda 180 derece
cevrildiginde ~17 derece degismisti. Yani gz ekseni gercek donusun ancak
altida-onda birini goruyor.

Olcek sabiti DOGRU oldugu register okumasiyla teyit edildi
(GYRO_CONFIG=0x00 -> +-250 dps -> bolen 131). O halde aci baska yerde
kayboluyor. Geriye kalan makul aciklama:

    DONUS gz EKSENINDE DEGIL.

IMU sasiye yatik ya da cevrilmis monteliyse aracin yaw donusu jiroskopun
x ya da y eksenine duser; gz'ye sadece projeksiyon kadari yansir.

Ayrica sensors/imu.py MOUNT_ROLL_DEG / MOUNT_PITCH_DEG'i YALNIZCA
ivmeolcerden gelen roll/pitch acisindan cikariyor; jiroskop ucluvsune
hicbir donusum uygulamiyor. Yani montaj acisi bilinse bile
yaw_rate = gz sensor cercevesinde kaliyor.

Bu script uc ekseni AYNI ANDA entegre eder ve donusu hangisinin
tasidigini gosterir.

KULLANIM
========
    python3 rov/tests/eksen_teshis.py

    Araci DUZ zemine koyun (normal calisma durusunda, suda nasil
    duruyorsa oyle). ENTER'a basin, sonra araci kendi DIKEY ekseni
    etrafinda -- yani suda donerken dondugu eksende -- ELLE TAM 360
    DERECE cevirin. Yavas ve duzgun cevirin (10-20 saniye).
    Tam tur bitince Ctrl+C.

NASIL YORUMLANIR
================
    Cikista uc eksenin entegrali yan yana verilir. TAM TUR attirdiysaniz
    birinin +-360'a yakin olmasi gerekir.

      gz ~ +-360  -> eksen DOGRU. Sorun baska yerde (o zaman haber verin).
      gx ~ +-360  -> IMU 90 derece yatik montajli. yaw_rate gx olmali.
      gy ~ +-360  -> IMU 90 derece yatik montajli. yaw_rate gy olmali.
      hicbiri 360 degil ama toplam buyukluk 360 -> IMU ARA bir acida;
         montaj matrisi gerekir (asagida hesaplanan aci ile).
      hicbiri ve toplam da degil -> okuma bozuk; eksen sorunu degil.

    ISARET: saat yonunde (saga) cevirdiyseniz, yaw'i tasiyan eksenin
    entegrali POZITIF olmali. Negatifse o eksen kaynagında negatiflenmeli.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from config import GYRO_BIAS
from sensors.imu import Mpu9250


def main():
    print()
    print("=" * 74)
    print(" EGE ROV — JIROSKOP EKSEN TESHISI")
    print(" (motorlara komut GONDERMEZ)")
    print("=" * 74)
    print()
    print("  HAZIRLIK:")
    print("    1. Araci DUZ zemine, normal calisma durusunda koyun.")
    print("    2. ENTER'a basin.")
    print("    3. Araci kendi DIKEY ekseni etrafinda ELLE TAM 360 DERECE")
    print("       cevirin. Saat yonunde (SAGA), yavas ve duzgun, 10-20 sn.")
    print("    4. Tam tur bitince Ctrl+C.")
    print()
    print("  NOT: cevirirken araci YATIRMAYIN, sadece dondurun.")
    print()

    try:
        imu = Mpu9250()
    except Exception as e:
        print(f"[HATA] MPU-9250'ye baglanilamadi: {e}")
        sys.exit(1)

    input("  Hazir olunca ENTER...")
    print()

    # Uc eksenin BAGIMSIZ entegrali. Bias cikarilmis ve cikarilmamis ayri
    # tutulur; bias yanlissa hangisinin ne kadar etkilendigi gorulsun.
    top = [0.0, 0.0, 0.0]        # bias duzeltilmis entegral
    top_ham = [0.0, 0.0, 0.0]    # bias duzeltmesiz entegral
    tepe = [0.0, 0.0, 0.0]       # her eksende gorulen en buyuk |hiz|
    n = 0

    onceki = time.monotonic()
    baslangic = onceki
    son_yazim = -9.0

    print(f"  {'t':>5} | {'gx':>7} {'gy':>7} {'gz':>7} | "
          f"{'∫gx':>8} {'∫gy':>8} {'∫gz':>8}")
    print("  " + "-" * 62)

    try:
        while True:
            simdi = time.monotonic()
            dt = simdi - onceki
            onceki = simdi

            g = imu.read_gyro_dps_raw()
            n += 1
            for i in range(3):
                duz = g[i] - GYRO_BIAS[i]
                top[i] += duz * dt
                top_ham[i] += g[i] * dt
                if abs(g[i]) > tepe[i]:
                    tepe[i] = abs(g[i])

            t = simdi - baslangic
            if t - son_yazim >= 0.25:
                son_yazim = t
                print(f"  {t:5.1f} | {g[0]:7.2f} {g[1]:7.2f} {g[2]:7.2f} | "
                      f"{top[0]:8.1f} {top[1]:8.1f} {top[2]:8.1f}")
            time.sleep(0.008)
    except KeyboardInterrupt:
        pass

    sure = time.monotonic() - baslangic
    print()
    print("=" * 74)
    print(" SONUC")
    print("=" * 74)
    print(f"  Sure: {sure:.1f} sn   Ornek: {n}   "
          f"Gercek okuma hizi: {n / max(sure, 1e-6):.0f} Hz")
    print()
    print(f"  {'eksen':<6} {'entegral':>10} {'bias-siz':>10} {'tepe hiz':>10}")
    print("  " + "-" * 40)
    for i, ad in enumerate(("gx", "gy", "gz")):
        print(f"  {ad:<6} {top[i]:>9.1f}° {top_ham[i]:>9.1f}° "
              f"{tepe[i]:>9.1f}")
    print()

    buyukluk = math.sqrt(sum(v * v for v in top))
    print(f"  Vektor buyuklugu (3 eksen birlikte): {buyukluk:.1f}°")
    print()

    # Hangi eksen baskin?
    mutlak = [abs(v) for v in top]
    baskin = mutlak.index(max(mutlak))
    ad = ("gx", "gy", "gz")[baskin]
    print("-" * 74)
    print(" DEGERLENDIRME (tam tur = 360 derece attirdiginizi varsayarak)")
    print("-" * 74)

    if max(mutlak) < 60:
        print("  Hicbir eksen donusu gormedi. Bu bir EKSEN sorunu degil;")
        print("  okuma yolu bozuk demektir (I2C, kablo, guc, ya da sensor).")
    elif abs(mutlak[2] - 360) < 60:
        print("  gz ~360 -> EKSEN DOGRU. yaw_rate = gz kalabilir.")
        print("  Sorun bu katmanda degil; sonucu paylasin.")
    elif abs(max(mutlak) - 360) < 90:
        isaret = "+" if top[baskin] > 0 else "-"
        print(f"  Donusu {ad} tasiyor ({top[baskin]:+.1f}°), gz degil "
              f"({top[2]:+.1f}°).")
        print(f"  -> IMU yatik montajli. sensors/imu.py'de yaw_rate {ad}")
        print(f"     ekseninden alinmali.")
        if isaret == "-":
            print(f"  -> Saga cevirdiyseniz {ad} NEGATIF cikmis: kaynagında")
            print("     negatiflenmeli (yaw_rate, daire sayaci ve kaskad ic")
            print("     dongusu boylece birlikte duzelir).")
    elif abs(buyukluk - 360) < 90:
        # Tek eksen degil, ara bir aci. Montaj acisini kestir.
        aci = math.degrees(math.acos(min(1.0, abs(top[2]) / max(buyukluk, 1e-6))))
        print(f"  Tek eksen degil: donus eksenler arasina dagilmis.")
        print(f"  Toplam buyukluk {buyukluk:.0f}° ~ 360°, yani jiroskop")
        print(f"  donusun TAMAMINI goruyor — ama yanlis eksenlerde.")
        print(f"  Kestirilen montaj egimi: ~{aci:.0f}° (gz ekseni ile arac")
        print(f"  dikey ekseni arasindaki aci).")
        print("  -> Jiroskop ucluvsune montaj donusum matrisi uygulanmali;")
        print("     tek bir eksen secmek yetmez.")
    else:
        print(f"  Belirsiz: baskin eksen {ad} = {top[baskin]:+.1f}°, "
              f"toplam buyukluk {buyukluk:.1f}°.")
        print("  Tam tur attirdigimizdan emin misiniz? Tekrarlayin.")

    print()
    print("  NOT: config.GYRO_BIAS =", GYRO_BIAS)
    print("  'entegral' ile 'bias-siz' arasindaki fark bias'in katkisidir.")
    print("  Fark buyukse bias yanlis olcumus demektir.")
    print("=" * 74)


if __name__ == "__main__":
    main()
