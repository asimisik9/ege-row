#!/usr/bin/env python3
"""
EGE ROV — IMU OKUMA YOLU TESHISI (motorlara DOKUNMAZ, kuru masada calisir).

NE ICIN VAR
===========
Havuzda olculen belirti: arac elle 180 derece cevrildiginde heading sadece
~17 derece degisti; durdurulunca da azalmaya devam etti.

Bunun iki olasi sebebi var ve ikisi de ayni belirtiyi verir:

  (A) JIROSKOP OLCEGI YANLIS
      sensors/imu.py ham degeri /131.0 ile boluyor. Bu bolen SADECE cip
      +-250 dps araligindayken dogru. Kod GYRO_CONFIG (0x1B) registerine
      HIC yazmiyor, cipi resetlemiyor (PWR_MGMT_1'e 0x00 yaziyor, reset
      icin 0x80 gerekir). Onceden baska bir sey araligi degistirdiyse
      okuma 2x / 4x / 8x kucuk cikar ve kimse fark etmez.

  (B) PUSULA HEADING'I GERI CEKIYOR
      config.USE_MAGNETOMETER = False yaziyor ama sensors/imu.py bu bayragi
      okumuyor; AK8963 hala heading'e karisiyor. Kalibrasyonsuz pusula
      (MAG_OFFSET=0, MAG_SCALE=1) sabit bir yone kilitlenince fuzyon
      heading'i ~2 saniyelik zaman sabitiyle o degere geri ceker.

Bu script ikisini AYIRIR.

KULLANIM
========
    python3 rov/tests/imu_teshis.py

    1) Once REGISTER RAPORU basilir — (A) aninda gorunur.
    2) Sonra canli tablo baslar. Araci masada elle TAM 90 (ya da 180)
       derece saat yonunde cevirin, birakin, birkac saniye bekleyin.

NASIL YORUMLANIR
================
    saf_gyro  : SADECE jiroskop entegrali. Fuzyon yok, pusula yok.
                90 derece cevirdiyseniz ~90 yazmali.
                ~45 -> olcek 2x yanlis      ~22 -> 4x yanlis
                ~11 -> 8x yanlis (FS_SEL=3, +-2000 dps)
    fuzyon    : imu.py'nin urettigi heading (Orientation.heading).
                saf_gyro dogru ama fuzyon geride kaliyorsa -> sebep (B).
    mag_h     : ham pusula acisi. Araci cevirirken bu HIC degismiyorsa
                pusula hard-iron'a kilitlenmis demektir.

    Ayrica ISARET kontrolu: saga (saat yonunde) cevirince saf_gyro
    ARTMALI. Azaliyorsa gz kaynagında negatiflenmeli.
"""
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config
from config import IMU_ADDR, GYRO_BIAS
from sensors.imu import Mpu9250, Orientation
from hal.i2c_lock import I2C_LOCK

# MPU-9250 yapilandirma registerleri (imu.py bunlarin HICBIRINE yazmiyor)
REG = {
    0x19: "SMPLRT_DIV",
    0x1A: "CONFIG (DLPF)",
    0x1B: "GYRO_CONFIG",
    0x1C: "ACCEL_CONFIG",
    0x6B: "PWR_MGMT_1",
}

# FS_SEL (GYRO_CONFIG bit 4:3) -> (aralik dps, dogru bolen)
GYRO_FS = {0: (250, 131.0), 1: (500, 65.5), 2: (1000, 32.8), 3: (2000, 16.4)}
# AFS_SEL (ACCEL_CONFIG bit 4:3) -> (aralik g, dogru bolen)
ACCEL_FS = {0: (2, 16384.0), 1: (4, 8192.0), 2: (8, 4096.0), 3: (16, 2048.0)}

KOD_GYRO_BOLEN = 131.0     # sensors/imu.py:181
KOD_ACCEL_BOLEN = 16384.0  # sensors/imu.py:173


def register_raporu(imu):
    print("=" * 74)
    print(" 1) REGISTER RAPORU — cip gercekte hangi aralikta?")
    print("=" * 74)
    degerler = {}
    for reg, ad in REG.items():
        try:
            with I2C_LOCK:
                v = imu.bus.read_byte_data(IMU_ADDR, reg)
            degerler[reg] = v
            print(f"    0x{reg:02X} {ad:<16} = 0x{v:02X}  (0b{v:08b})")
        except OSError as e:
            print(f"    0x{reg:02X} {ad:<16} = OKUNAMADI ({e})")

    print("-" * 74)
    sorun = False

    fs = (degerler.get(0x1B, 0) >> 3) & 0x03
    aralik, dogru = GYRO_FS[fs]
    oran = dogru and (KOD_GYRO_BOLEN / dogru)
    print(f"    JIROSKOP : FS_SEL={fs} -> +-{aralik} dps, dogru bolen {dogru}")
    print(f"               imu.py {KOD_GYRO_BOLEN} kullaniyor", end="")
    if abs(oran - 1.0) < 0.01:
        print("  -> DOGRU")
    else:
        sorun = True
        print(f"  -> YANLIS: okumalar {oran:.1f}x KUCUK cikiyor!")
        print(f"               Duzeltme: imu.py'de bolen {dogru} olmali,")
        print("               ya da init'te GYRO_CONFIG'e 0x00 yazilmali.")

    afs = (degerler.get(0x1C, 0) >> 3) & 0x03
    a_aralik, a_dogru = ACCEL_FS[afs]
    a_oran = KOD_ACCEL_BOLEN / a_dogru
    print(f"    IVMEOLCER: AFS_SEL={afs} -> +-{a_aralik} g, dogru bolen {a_dogru}")
    print(f"               imu.py {KOD_ACCEL_BOLEN} kullaniyor", end="")
    if abs(a_oran - 1.0) < 0.01:
        print("  -> DOGRU")
    else:
        sorun = True
        print(f"  -> YANLIS: okumalar {a_oran:.1f}x KUCUK cikiyor!")

    dlpf = degerler.get(0x1A, 0) & 0x07
    if dlpf == 0:
        print("    DLPF     : 0 (filtre KAPALI) -> 100 Hz orneklemede aliasing")
        print("               beklenir; roll/pitch gurultulu olur.")

    print("-" * 74)
    print(f"    MANYETOMETRE (AK8963) aktif mi : {imu.has_mag}")
    print(f"    config.USE_MAGNETOMETER        : "
          f"{getattr(config, 'USE_MAGNETOMETER', 'tanimsiz')}")
    if imu.has_mag and not getattr(config, "USE_MAGNETOMETER", True):
        sorun = True
        print("    -> UYUMSUZ! config 'kullanma' diyor ama imu.py bayragi hic")
        print("       okumuyor; pusula heading'e KARISIYOR. Sebep (B) gecerli.")
    print(f"    config.GYRO_BIAS               : {GYRO_BIAS}")
    print("=" * 74)
    return sorun


def canli_tablo(imu, saniye=90.0):
    print()
    print("=" * 74)
    print(" 2) CANLI OLCUM")
    print("=" * 74)
    print("    Araci masada elle TAM 90 (ya da 180) derece SAAT YONUNDE")
    print("    cevirin, birakin ve birkac saniye bekleyin. Ctrl+C ile cikin.")
    print()
    print("    saf_gyro : sadece jiroskop entegrali (fuzyon yok)")
    print("    fuzyon   : imu.py'nin urettigi heading")
    print("    mag_h    : ham pusula acisi (cevirirken degismiyorsa hard-iron)")
    print()
    input("    Hazir olunca ENTER...")
    print()

    ori = Orientation(imu)
    saf = 0.0            # sadece jiroskop entegrali (bias duzeltilmis)
    saf_ham = 0.0        # bias HIC cikarilmadan entegral (bias teshisi icin)
    onceki = time.monotonic()
    baslangic = onceki
    son_yazim = 0.0

    print(f"    {'t':>5} {'gz_ham':>8} {'gz_duz':>8} {'saf_gyro':>9} "
          f"{'saf_ham':>8} {'fuzyon':>8} {'mag_h':>8}")
    print("    " + "-" * 62)
    try:
        while time.monotonic() - baslangic < saniye:
            simdi = time.monotonic()
            dt = simdi - onceki
            onceki = simdi

            gz_ham = imu.read_gyro_dps_raw()[2]
            gz_duz = gz_ham - GYRO_BIAS[2]
            saf += gz_duz * dt
            saf_ham += gz_ham * dt

            ori.update()   # imu.py'nin gercek fuzyon yolu

            t = simdi - baslangic
            if t - son_yazim >= 0.2:
                son_yazim = t
                mag_h = ori.mag_heading
                mag_s = f"{mag_h:8.1f}" if mag_h is not None else "     yok"
                print(f"    {t:5.1f} {gz_ham:8.2f} {gz_duz:8.2f} {saf:9.1f} "
                      f"{saf_ham:8.1f} {ori.heading:8.1f} {mag_s}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n    (Ctrl+C)")

    print()
    print("=" * 74)
    print(" DEGERLENDIRME")
    print("=" * 74)
    print(f"    Toplam saf jiroskop entegrali : {saf:+.1f} derece")
    print(f"    (bias duzeltmesi olmadan)     : {saf_ham:+.1f} derece")
    print(f"    imu.py fuzyon heading'i       : {ori.heading:.1f} derece")
    print(f"    Pusula duzeltmesi atlanan adim: {ori.mag_rejected}")
    print("-" * 74)
    print("    90 derece cevirdiyseniz saf_gyro ~90 olmali:")
    print("      ~90 -> olcek DOGRU. Fuzyon geride kaldiysa sebep (B): pusula.")
    print("      ~45 -> olcek 2x yanlis   |   ~22 -> 4x yanlis")
    print("      ~11 -> olcek 8x yanlis (FS_SEL=3). Sebep (A).")
    print()
    print("    ISARET: saga (saat yonunde) cevirdiyseniz saf_gyro ARTMALI.")
    print("      Azaldiysa gz kaynaginda negatiflenmeli (yaw_rate ve daire")
    print("      sayaci da o zaman kendiliginden duzelir).")
    print()
    print("    BIAS: arac HAREKETSIZKEN saf_gyro surekli kayiyorsa")
    print("      GYRO_BIAS yanlis -> calibrate_imu.py'yi arac tamamen")
    print("      hareketsizken yeniden calistirin.")
    print("=" * 74)


def main():
    print()
    print("EGE ROV — IMU OKUMA YOLU TESHISI")
    print("(motorlara komut GONDERMEZ, kuru masada guvenle calisir)")
    print()
    try:
        imu = Mpu9250()
    except Exception as e:
        print(f"[HATA] MPU-9250'ye baglanilamadi: {e}")
        print("       Kontrol: python3 rov/tests/i2c_tara.py")
        sys.exit(1)

    register_raporu(imu)
    canli_tablo(imu)


if __name__ == "__main__":
    main()
