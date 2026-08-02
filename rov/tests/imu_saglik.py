#!/usr/bin/env python3
"""
EGE ROV — IMU SAGLIK TESTI (motorlara DOKUNMAZ, kuru masada calisir).

NE ICIN VAR
===========
Eksen teshisinde arac TAM TUR (360 derece) cevrildi, jiroskop uc eksenin
toplaminda sadece ~42 derece gordu. Bu bir eksen sorunu degil — eksen
sorunu olsaydi donus BASKA bir eksende 360 olarak gorunurdu.

Geriye iki ihtimal kaldi:
    1) Donus fiilen yapilmadi / cok kucuktu  -> sensor saglam, test tekrarlanmali
    2) Jiroskop donuse yanit VERMIYOR        -> sensor/baglanti arizali

Bu script ikisini ayirir ve sensorun temel sagligini olcer:

    A) WHO_AM_I  : cip gercekten MPU-9250 mi? (AK8963'un yoklugunu tek
                   basina aciklayabilir — MPU-6500/klon cipte manyetometre
                   YOKTUR)
    B) SICAKLIK  : cip canli mi, I2C tutarli mi?
    C) HAREKETSIZ: sifir-hiz ofsetleri ve gurultu, veri sayfasi sinirlariyla
    D) IVMEOLCER : araci yatirinca 1g dogru eksende goruluyor mu?
    E) JIROSKOP  : HIZLI cevirince tepe hiz kac dps okunuyor?

KULLANIM
========
    python3 rov/tests/imu_saglik.py

    Yonergeleri takip edin. Toplam ~2 dakika.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sensors.imu import Mpu9250
from hal.i2c_lock import I2C_LOCK
from config import IMU_ADDR

WHO_AM_I = 0x75
TEMP_OUT_H = 0x41

# WHO_AM_I -> cip. MPU-9250 disindaki cipte AK8963 (manyetometre) YOKTUR.
CIPLER = {
    0x71: ("MPU-9250", True),
    0x73: ("MPU-9255", True),
    0x70: ("MPU-6500", False),
    0x68: ("MPU-6050 / MPU-9250 klonu", False),
    0x69: ("MPU-6500 varyanti / klon", False),
}

# Veri sayfasi sinirlari
GYRO_OFFSET_TIPIK = 5.0     # dps, tipik sifir-hiz cikisi
GYRO_OFFSET_MAX = 20.0      # dps, mutlak sinir
GYRO_GURULTU_MAX = 1.0      # dps RMS, DLPF ile beklenen ust sinir


def satir(baslik):
    print()
    print("=" * 74)
    print(f" {baslik}")
    print("=" * 74)


def oku_word(imu, reg):
    with I2C_LOCK:
        hi = imu.bus.read_byte_data(IMU_ADDR, reg)
        lo = imu.bus.read_byte_data(IMU_ADDR, reg + 1)
    raw = (hi << 8) | lo
    return raw - 65536 if raw > 32767 else raw


# ----------------------------------------------------------------- A + B
def kimlik(imu):
    satir("A) CIP KIMLIGI")
    try:
        with I2C_LOCK:
            wai = imu.bus.read_byte_data(IMU_ADDR, WHO_AM_I)
    except OSError as e:
        print(f"  WHO_AM_I OKUNAMADI ({e}) -> I2C baglantisi saglam degil.")
        return False

    ad, mag_var = CIPLER.get(wai, (None, None))
    print(f"  WHO_AM_I (0x75) = 0x{wai:02X}")
    if ad is None:
        print("  -> TANINMAYAN CIP. Beklenen 0x71 (MPU-9250).")
        print("     Bu adreste MPU olmayan bir sey olabilir.")
        return False

    print(f"  -> {ad}")
    if not mag_var:
        print()
        print("  !! BU CIPTE MANYETOMETRE YOKTUR.")
        print("     AK8963'un Errno 121 vermesi ARIZA DEGIL — o cip orada yok.")
        print("     config.USE_MAGNETOMETER = False zaten dogru karar.")
    elif not imu.has_mag:
        print()
        print("  !! Cip MPU-9250 ama AK8963'e erisilemiyor.")
        print("     Paket icindeki manyetometre olu ya da bypass calismiyor.")

    satir("B) CIP CANLI MI? (sicaklik)")
    try:
        ham = oku_word(imu, TEMP_OUT_H)
        c = ham / 333.87 + 21.0
        print(f"  TEMP_OUT = {ham}  ->  {c:.1f} °C")
        if 5.0 < c < 70.0:
            print("  -> Makul. Cip calisiyor, I2C tutarli veri veriyor.")
        else:
            print("  -> ANORMAL. I2C'den bozuk veri geliyor olabilir.")
    except OSError as e:
        print(f"  Sicaklik okunamadi: {e}")
    return True


# --------------------------------------------------------------------- C
def hareketsiz(imu, sure=15.0):
    satir("C) HAREKETSIZ TEST — sifir-hiz ofseti ve gurultu")
    print("  Araci masaya KOYUN ve ELLEMEYIN.")
    input("  Hazir olunca ENTER...")
    print(f"  {sure:.0f} saniye olculuyor, dokunmayin...")

    ornekler = [[], [], []]
    a_ornek = [[], [], []]
    t0 = time.monotonic()
    while time.monotonic() - t0 < sure:
        g = imu.read_gyro_dps_raw()
        a = imu.read_accel_g_raw()
        for i in range(3):
            ornekler[i].append(g[i])
            a_ornek[i].append(a[i])
        time.sleep(0.01)

    print()
    print(f"  {'eksen':<6} {'ortalama':>10} {'gurultu(RMS)':>14} {'durum':>10}")
    print("  " + "-" * 46)
    arizali = False
    for i, ad in enumerate(("gx", "gy", "gz")):
        v = ornekler[i]
        ort = sum(v) / len(v)
        rms = math.sqrt(sum((x - ort) ** 2 for x in v) / len(v))
        if abs(ort) > GYRO_OFFSET_MAX:
            durum = "ARIZALI"
            arizali = True
        elif abs(ort) > GYRO_OFFSET_TIPIK:
            durum = "SINIRDA"
        else:
            durum = "iyi"
        print(f"  {ad:<6} {ort:>9.2f}° {rms:>13.2f} {durum:>10}")

    print()
    print(f"  Veri sayfasi: sifir-hiz cikisi tipik +-{GYRO_OFFSET_TIPIK:.0f} dps,")
    print(f"                mutlak sinir +-{GYRO_OFFSET_MAX:.0f} dps.")
    if arizali:
        print("  !! Bir ya da daha fazla eksen MUTLAK SINIRIN disinda.")
        print("     Bias ile duzeltilse bile artik hata ve sicaklik kaymasi")
        print("     buyuk olur; bu cip ile jiroskop entegrasyonuyla yon")
        print("     tutmak pratikte mumkun degildir.")

    print()
    print("  Ivmeolcer (duz duruyorsa z ~ +1.0 g, x ve y ~ 0):")
    for i, ad in enumerate(("ax", "ay", "az")):
        ort = sum(a_ornek[i]) / len(a_ornek[i])
        print(f"    {ad} = {ort:+.3f} g")
    toplam = math.sqrt(sum((sum(a_ornek[i]) / len(a_ornek[i])) ** 2
                           for i in range(3)))
    print(f"    |a| = {toplam:.3f} g   (saglam sensorde 1.00 +- 0.05 olmali)")
    if abs(toplam - 1.0) > 0.1:
        print("    !! Buyukluk 1 g degil -> ivmeolcer de guvenilmez.")
    return arizali


# --------------------------------------------------------------------- D
def ivme_testi(imu):
    satir("D) IVMEOLCER YANIT TESTI")
    print("  Araci sirayla uc konuma getirin. Her konumda yercekimi")
    print("  FARKLI bir eksende +-1 g olarak gorunmeli.")
    duruslar = [
        ("DUZ (normal calisma durusu)", "bir eksen ~ +1.0 g"),
        ("SAG YANINA yatirin", "baska bir eksen ~ +-1.0 g"),
        ("ON tarafi yukari kaldirin", "ucuncu eksen ~ +-1.0 g"),
    ]
    gorulen = []
    for ad, beklenen in duruslar:
        print()
        print(f"  --> {ad}")
        print(f"      Beklenen: {beklenen}")
        input("      Konuma getirip ENTER...")
        birikim = [0.0, 0.0, 0.0]
        for _ in range(50):
            a = imu.read_accel_g_raw()
            for i in range(3):
                birikim[i] += a[i] / 50.0
            time.sleep(0.01)
        b = max(range(3), key=lambda i: abs(birikim[i]))
        ad_eks = ("ax", "ay", "az")[b]
        gorulen.append(ad_eks)
        print(f"      Olculen: ax={birikim[0]:+.2f} ay={birikim[1]:+.2f} "
              f"az={birikim[2]:+.2f} g   -> baskin: {ad_eks}")

    print()
    if len(set(gorulen)) == 3:
        print("  -> GECTI: uc konumda uc FARKLI eksen baskin cikti.")
        print("     Ivmeolcer harekete dogru yanit veriyor.")
    else:
        print(f"  -> KALDI: baskin eksenler {gorulen} — uc farkli olmaliydi.")
        print("     Ivmeolcer harekete dogru yanit VERMIYOR.")


# --------------------------------------------------------------------- E
def jiro_testi(imu, sure=15.0):
    satir("E) JIROSKOP YANIT TESTI — asil soru")
    print("  Bu testin amaci: jiroskop harekete YANIT VERIYOR MU?")
    print("  Aci dogrulugu degil, sadece yanit araniyor.")
    print()
    print("  ENTER'a basinca araci elinizle HIZLICA saga-sola cevirin.")
    print("  Yavas degil — belirgin, hizli hareketler yapin. Her uc")
    print(f"  ekseni de deneyin. {sure:.0f} saniye surecek.")
    input("  Hazir olunca ENTER...")

    # once hareketsiz taban (ofseti cikarmak icin)
    print("  Once 2 sn hareketsiz taban aliniyor, DOKUNMAYIN...")
    taban = [0.0, 0.0, 0.0]
    n = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        g = imu.read_gyro_dps_raw()
        for i in range(3):
            taban[i] += g[i]
        n += 1
        time.sleep(0.01)
    taban = [v / n for v in taban]
    print(f"  Taban (ofset): gx={taban[0]:+.1f} gy={taban[1]:+.1f} "
          f"gz={taban[2]:+.1f} dps")
    print()
    print("  SIMDI CEVIRIN! " + "-" * 40)

    tepe = [0.0, 0.0, 0.0]
    t0 = time.monotonic()
    son = -9.0
    while True:
        t = time.monotonic() - t0
        if t >= sure:
            break
        g = imu.read_gyro_dps_raw()
        for i in range(3):
            fark = abs(g[i] - taban[i])
            if fark > tepe[i]:
                tepe[i] = fark
        if t - son >= 1.0:
            son = t
            print(f"    {sure - t:4.0f} sn kaldi | tepe: "
                  f"gx={tepe[0]:6.1f} gy={tepe[1]:6.1f} gz={tepe[2]:6.1f} dps")
        time.sleep(0.005)

    satir("SONUC")
    print(f"  Ofsetten SAPMA olarak gorulen tepe hizlar:")
    for i, ad in enumerate(("gx", "gy", "gz")):
        print(f"    {ad} = {tepe[i]:7.1f} dps")
    print()
    enb = max(tepe)
    if enb > 100:
        print("  -> JIROSKOP YANIT VERIYOR (tepe > 100 dps).")
        print("     Sensor harekete tepki gosteriyor. O halde 360 derecelik")
        print("     donusun gorulmemesi TEST YONTEMINDEN kaynaklaniyor:")
        print("     donus ya yapilmadi ya da cok kucuktu. eksen_teshis.py'yi")
        print("     tekrarlayin, bu sefer donusu belirgin ve kesintisiz yapin.")
    elif enb > 30:
        print("  -> ZAYIF YANIT (tepe 30-100 dps).")
        print("     Elle hizli cevirmede 150+ dps beklenir. Sensor tepki")
        print("     veriyor ama olcek supheli.")
    else:
        print("  -> JIROSKOP YANIT VERMIYOR (tepe < 30 dps).")
        print("     Elinizle hizlica cevirdiginiz halde sensor bunu")
        print("     gormedi. Cip ya da baglantisi ARIZALI.")
        print("     Yazilim tarafinda yapilacak bir sey yok; sensor")
        print("     degistirilmeli ya da kablolamasi/beslemesi kontrol")
        print("     edilmeli (3.3V parca — 5V verilmis olabilir mi?).")
    print("=" * 74)


def main():
    print()
    print("EGE ROV — IMU SAGLIK TESTI  (motorlara komut GONDERMEZ)")
    try:
        imu = Mpu9250()
    except Exception as e:
        print(f"[HATA] MPU-9250'ye baglanilamadi: {e}")
        sys.exit(1)

    if not kimlik(imu):
        print("\n  Kimlik dogrulanamadi — diger testler anlamsiz olur.")
        sys.exit(1)
    hareketsiz(imu)
    ivme_testi(imu)
    jiro_testi(imu)


if __name__ == "__main__":
    main()
