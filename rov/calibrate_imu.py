"""
IMU (pusula) kalibrasyon scripti.

Kullanim (cihazda, rov/ klasoru icinden):
    python3 calibrate_imu.py

Ne yapar:
  1) Adim: Arac SABIT dururken 5 sn olcum yapar -> GYRO_BIAS hesaplar.
  2) Adim: Sen araci elinle her yone cevirirken 15 sn olcum yapar
     -> MAG_OFFSET / MAG_SCALE hesaplar.
  3) Sonuclari ekrana basar ve config.py dosyasini OTOMATIK gunceller
     (eski hali config.py.bak olarak yedeklenir).

Not: Bu script gercek IMU donanimi bagliyken calisir (SIM_MODE ile ilgisi yok,
     dogrudan sensoru okur). Simulasyonda / donanimsiz calistirilamaz.
"""
import re
import shutil
import time

from sensors.imu import Mpu9250


def _avg(vals):
    """Bir listenin aritmetik ortalamasini dondurur."""
    return sum(vals) / len(vals)


def calibrate_gyro(imu, duration_s=5.0):
    """Arac hareketsizken jiroskopu duration_s saniye orneklyip ortalama
    sapmayi (GYRO_BIAS) hesaplar. HAM okuma kullanilir ki onceki
    kalibrasyon degerleri ust uste binmesin. Ideal jiroskop durgun halde
    0 okumali; olculen ortalama gercek okumadan cikarilarak sapma giderilir."""
    print(f"\nOlculuyor ({duration_s:.0f} sn) - ARACA DOKUNMA...")
    xs, ys, zs = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        gx, gy, gz = imu.read_gyro_dps_raw()
        xs.append(gx)
        ys.append(gy)
        zs.append(gz)
        time.sleep(0.02)
    bias = (round(_avg(xs), 3), round(_avg(ys), 3), round(_avg(zs), 3))
    print(f"GYRO_BIAS olculen deger: {bias}")

    # kalite kontrolu: olcum sirasinda hareket var miydi?
    spread = max(max(v) - min(v) for v in (xs, ys, zs))
    if spread > 3.0:
        print(f"[UYARI] Olcum sirasinda hareket algilandi (yayilim {spread:.1f} "
              "dps)! Araci sabit tutup tekrar calistir.")
    if max(abs(b) for b in bias) > 10.0:
        print("[UYARI] Bias cok buyuk (>10 dps) - sensor arizali olabilir "
              "veya olcum sirasinda arac donduruldu.")
    return bias


def calibrate_mag(imu, duration_s=15.0):
    """Arac her yone cevrilirken manyetometreyi duration_s saniye orneklyip
    hard-iron ofsetini (MAG_OFFSET) ve soft-iron olcegini (MAG_SCALE)
    hesaplar. Mantik: her eksendeki min/max degerlerin ortasi ofset,
    yaricaplarin esitlenmesi de eksenler arasi olcek farkini duzeltir."""
    print(f"\n{duration_s:.0f} saniye boyunca ARACI YAVASCA HER YONE CEVIR "
          "(saga-sola, yukari-asagi, tam tur) ...")
    xs, ys, zs = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        mx, my, mz = imu.read_mag_ut_raw()   # HAM okuma (onceki kalibrasyon binmesin)
        xs.append(mx)
        ys.append(my)
        zs.append(mz)
        time.sleep(0.02)
    print("Tamam, aracı birakabilirsin.")

    def offset_radius(vals):
        """Bir eksendeki degerler icin (ofset, yaricap) dondurur:
        ofset = (max+min)/2 (hard-iron kaymasi), yaricap = (max-min)/2."""
        return (max(vals) + min(vals)) / 2.0, (max(vals) - min(vals)) / 2.0

    ox, rx = offset_radius(xs)
    oy, ry = offset_radius(ys)
    oz, rz = offset_radius(zs)
    avg_r = (rx + ry + rz) / 3.0

    # kalite kontrolu: her eksende yeterli donus yapildi mi?
    # Yeryuzu manyetik alani ~25-65 uT; yaricap cok kucukse o eksen az cevrilmis.
    for name, r in (("X", rx), ("Y", ry), ("Z", rz)):
        if r < 10.0:
            print(f"[UYARI] {name} ekseni yaricapi kucuk ({r:.1f} uT) - arac o "
                  "eksende yeterince cevrilmemis. Kalibrasyonu tekrarla!")
    if avg_r > 100.0:
        print(f"[UYARI] Ortalama yaricap cok buyuk ({avg_r:.0f} uT) - yakinlarda "
              "miknatis/motor/metal olabilir, araci ortamdan uzaklastir.")

    offset = (round(ox, 2), round(oy, 2), round(oz, 2))
    scale = (
        round(avg_r / rx, 3) if rx else 1.0,
        round(avg_r / ry, 3) if ry else 1.0,
        round(avg_r / rz, 3) if rz else 1.0,
    )
    print(f"MAG_OFFSET olculen deger: {offset}")
    print(f"MAG_SCALE  olculen deger: {scale}")
    return offset, scale


def write_config(gyro_bias, mag_offset, mag_scale, path="config.py"):
    """config.py icindeki GYRO_BIAS / MAG_OFFSET / MAG_SCALE satirlarini
    regex ile yeni olculen degerlerle degistirir. Once dosyayi .bak olarak
    yedekler ki yanlis kalibrasyonda eski hale donulebilsin."""
    shutil.copy(path, path + ".bak")
    with open(path) as f:
        text = f.read()

    text = re.sub(r"GYRO_BIAS\s*=.*",
                  f"GYRO_BIAS  = {gyro_bias}  # kalibrasyon ile olculdu", text)
    text = re.sub(r"MAG_OFFSET\s*=.*",
                  f"MAG_OFFSET = {mag_offset}  # kalibrasyon ile olculdu", text)
    text = re.sub(r"MAG_SCALE\s*=.*",
                  f"MAG_SCALE  = {mag_scale}  # kalibrasyon ile olculdu", text)

    with open(path, "w") as f:
        f.write(text)
    print(f"\nconfig.py guncellendi. Eski hali: {path}.bak")


def main():
    """Kalibrasyon akisini yonetir: once jiroskop (arac sabit), sonra
    manyetometre (arac elle cevrilerek) olculur, sonuclar ozetlenip
    config.py dosyasina yazilir."""
    print("=== IMU Kalibrasyonu ===")
    input("\n1) ARACI SABIT TUT (hic kipirdatma). Hazir olunca ENTER'a bas...")
    imu = Mpu9250()

    gyro_bias = calibrate_gyro(imu)

    input("\n2) Simdi araci elinle cevirmeye hazirlan. "
          "Hazir olunca ENTER'a bas, sonra hemen cevirmeye basla...")
    mag_offset, mag_scale = calibrate_mag(imu)

    print("\n--- OZET ---")
    print(f"GYRO_BIAS  = {gyro_bias}")
    print(f"MAG_OFFSET = {mag_offset}")
    print(f"MAG_SCALE  = {mag_scale}")

    write_config(gyro_bias, mag_offset, mag_scale)
    print("\nBitti. Simdi gorevi baslatabilirsin: 'python3 video_main.py' "
          "(ya da tam sistem icin 'python3 main.py').")


if __name__ == "__main__":
    main()
