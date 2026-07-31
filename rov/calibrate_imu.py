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

    # Kalite kontrolü: Gerçek hareket var mı? (Standart sapma ile kontrol)
    import statistics
    std_x = statistics.pstdev(xs)
    std_y = statistics.pstdev(ys)
    std_z = statistics.pstdev(zs)
    max_std = max(std_x, std_y, std_z)

    if max_std > 1.5:
        print(f"[UYARI] Ölçüm sırasında belirgin hareket algılandı (Standart Sapma: {max_std:.2f} dps)! "
              "Aracı tamamen sabit tutup tekrar çalıştırın.")
    if max(abs(b) for b in bias) > 15.0:
        print("[UYARI] Bias çok büyük (>15 dps) - sensör arızalı olabilir "
              "veya ölçüm sırasında araç döndürüldü.")
    return bias


def calibrate_accel(imu, duration_s=5.0):
    """Arac düz bir zeminde hareketsizken ivmeolceri ornekler ve
    ivmeolcer sapmasini (ACCEL_BIAS) hesaplar. Arac düzken
    beklenen ivme (0, 0, 1) olmalidir."""
    print(f"\nOlculuyor ({duration_s:.0f} sn) - İvmeölçer için... ARACA DOKUNMA...")
    xs, ys, zs = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        ax, ay, az = imu.read_accel_g_raw()
        xs.append(ax)
        ys.append(ay)
        zs.append(az)
        time.sleep(0.02)
    # Z ekseninden yerçekimini çıkar (1g varsayarak)
    bias = (round(_avg(xs), 3), round(_avg(ys), 3), round(_avg(zs) - 1.0, 3))
    print(f"ACCEL_BIAS olculen deger: {bias}")
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


def write_config(gyro_bias, accel_bias, mag_offset, mag_scale, path="config.py"):
    """config.py icindeki GYRO_BIAS / MAG_OFFSET / MAG_SCALE satirlarini
    regex ile yeni olculen degerlerle degistirir. Once dosyayi .bak olarak
    yedekler ki yanlis kalibrasyonda eski hale donulebilsin."""
    shutil.copy(path, path + ".bak")
    with open(path) as f:
        text = f.read()

    text = re.sub(r"GYRO_BIAS\s*=.*",
                  f"GYRO_BIAS  = {gyro_bias}  # kalibrasyon ile olculdu", text)
    if "ACCEL_BIAS" in text:
        text = re.sub(r"ACCEL_BIAS\s*=.*",
                      f"ACCEL_BIAS = {accel_bias}  # kalibrasyon ile olculdu", text)
    else:
        text = text.replace(f"GYRO_BIAS  = {gyro_bias}  # kalibrasyon ile olculdu",
                            f"GYRO_BIAS  = {gyro_bias}  # kalibrasyon ile olculdu\n"
                            f"ACCEL_BIAS = {accel_bias}  # kalibrasyon ile olculdu")
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
    accel_bias = calibrate_accel(imu)

    input("\n2) Simdi araci elinle cevirmeye hazirlan. "
          "Hazir olunca ENTER'a bas, sonra hemen cevirmeye basla...")
    mag_offset, mag_scale = calibrate_mag(imu)

    print("\n--- OZET ---")
    print(f"GYRO_BIAS  = {gyro_bias}")
    print(f"ACCEL_BIAS = {accel_bias}")
    print(f"MAG_OFFSET = {mag_offset}")
    print(f"MAG_SCALE  = {mag_scale}")

    write_config(gyro_bias, accel_bias, mag_offset, mag_scale)
    print("\nBitti. Simdi gorevi baslatabilirsin: 'python3 video_main.py' "
          "(ya da tam sistem icin 'python3 main.py').")


if __name__ == "__main__":
    main()
