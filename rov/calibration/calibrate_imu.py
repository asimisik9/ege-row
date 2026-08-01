"""
EGE ROV — IMU Kalibrasyon Scripti (Eksen Bağımsız, 9-DoF)

IMU (MPU-9250) fiziksel olarak ROV'a hangi açıda monte edilmiş olursa
olsun, ROV'un kendi doğal duruşunu referans alarak tüm kalibrasyon
parametrelerini otomatik hesaplar ve config.py'ye yazar.

Önceki calibrate_imu.py'den farkı:
  - ROV'u yatay tutmak GEREKMEZ
  - İvmeölçer Z ekseninde 1g varsaymaz → her açıda doğru çalışır
  - MOUNT_PITCH_DEG / MOUNT_ROLL_DEG: ROV'un doğal duruşundan sapma
    Orientation sınıfı bu değerleri çıkararak her zaman doğal duruşu
    roll=0°, pitch=0° olarak kabul eder

Adımlar:
  1) Gyro Bias    : ROV sabit 5 sn → GYRO_BIAS
  2) Accel+Montaj : ROV doğal duruşunda 5 sn → ACCEL_BIAS, MOUNT_PITCH_DEG, MOUNT_ROLL_DEG
  3) Pusula (Mag) : Her yöne çevir 15 sn → MAG_OFFSET, MAG_SCALE
  4) config.py'ye yaz (yedek .bak olarak alınır)

Kullanım (cihazda, rov/ klasöründen):
    python3 calibration/calibrate_imu.py
"""
import math
import re
import shutil
import statistics
import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

CONFIG_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.py')
)

from sensors.imu import Mpu9250


# ─────────────────────────────────────────────── yardımcılar

def _avg(vals):
    return sum(vals) / len(vals)


def _sample(imu, read_fn, duration_s, label, print_live=False):
    """İMU'dan duration_s saniye boyunca (gx,gy,gz) / (ax,ay,az) vb. örnekler.
    (xs, ys, zs) listelerini döner."""
    xs, ys, zs = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        v = read_fn()
        xs.append(v[0]); ys.append(v[1]); zs.append(v[2])
        if print_live:
            elapsed = time.monotonic() - t0
            remaining = duration_s - elapsed
            print(f"    {remaining:4.1f}s kaldı  "
                  f"X:{v[0]:7.2f}  Y:{v[1]:7.2f}  Z:{v[2]:7.2f}", end="\r")
        time.sleep(0.02)
    if print_live:
        print()
    return xs, ys, zs


# ─────────────────────────────────────────────── adım 1: gyro

def calibrate_gyro(imu, duration_s=5.0):
    """Gyro bias: ROV hareketsizken ortalama sapma değeri.
    İdeal jiroskop durgun halde 0 dps okur; ölçülen ortalama = bias."""
    print(f"\n[1/3] GYRO — ROV'u SABIT TUT ({duration_s:.0f} sn, hareketsiz)...")
    xs, ys, zs = _sample(imu, imu.read_gyro_dps_raw, duration_s, "gyro", print_live=True)

    bias = (round(_avg(xs), 4), round(_avg(ys), 4), round(_avg(zs), 4))
    std_max = max(statistics.pstdev(xs), statistics.pstdev(ys), statistics.pstdev(zs))

    print(f"    GYRO_BIAS  = {bias}")
    if std_max > 1.5:
        print(f"    [UYARI] Gürültü yüksek (std={std_max:.2f} dps) — ölçüm sırasında hareket var mıydı?")
    if max(abs(b) for b in bias) > 15.0:
        print("    [UYARI] Bias çok büyük (>15 dps) — sensör arızalı olabilir.")
    return bias


# ─────────────────────────────────────────────── adım 2: accel + montaj açısı

def calibrate_accel_and_mount(imu, duration_s=5.0):
    """İvmeölçer bias'ı ve IMU montaj açısını ölçer.

    Yöntem (eksen bağımsız):
      ROV'un doğal duruşunda yerçekimi vektörü IMU çerçevesinde ölçülür:
        G = (ax_avg, ay_avg, az_avg)   |G| ≈ 1g

      Bu vektörden montaj açıları:
        MOUNT_ROLL  = atan2(ay, az)              ← IMU Y/Z düzlemindeki roll
        MOUNT_PITCH = atan2(-ax, √(ay²+az²))     ← IMU X eksenindeki pitch

      Orientation sınıfı bu değerleri çıkararak ROV'un doğal duruşunu
      her zaman roll=0°, pitch=0° olarak alır. IMU hangi açıda olursa olsun.

      ACCEL_BIAS: Yerçekimi büyüklüğü normalleşmesinden kalan kazanç hatası.
    """
    print(f"\n[2/3] İVMEÖLÇER + MONTAJ AÇISI — ROV DOĞAL DURUMDA ({duration_s:.0f} sn)...")
    print("      ROV nasıl duruyor ise öyle bırakın (yatay olması GEREKMEZ)")
    xs, ys, zs = _sample(imu, imu.read_accel_g_raw, duration_s, "accel", print_live=True)

    ax_avg = _avg(xs)
    ay_avg = _avg(ys)
    az_avg = _avg(zs)
    magnitude = math.sqrt(ax_avg**2 + ay_avg**2 + az_avg**2)

    print(f"    Yerçekimi vektörü (IMU çerçeve): ({ax_avg:.4f}, {ay_avg:.4f}, {az_avg:.4f}) g")
    print(f"    Büyüklük: {magnitude:.4f}g  (ideal: 1.000g)")

    if abs(magnitude - 1.0) > 0.15:
        print(f"    [UYARI] Büyüklük 1g'den belirgin sapıyor ({magnitude:.3f}g). "
              "Sensör veya hareket sorunu olabilir. Tekrar deneyin.")

    # Montaj açıları: ROV doğal durumunda IMU'nun gördüğü roll/pitch
    mount_roll  = math.degrees(math.atan2(ay_avg, az_avg))
    mount_pitch = math.degrees(math.atan2(-ax_avg, math.hypot(ay_avg, az_avg)))

    # ACCEL_BIAS: normalize birim vektörden sapma (gain hatası, genellikle küçük)
    if magnitude > 1e-6:
        # Birim yönde beklenen değerler
        exp_ax = ax_avg / magnitude
        exp_ay = ay_avg / magnitude
        exp_az = az_avg / magnitude
        accel_bias = (
            round(ax_avg - exp_ax, 4),
            round(ay_avg - exp_ay, 4),
            round(az_avg - exp_az, 4),
        )
    else:
        accel_bias = (0.0, 0.0, 0.0)

    mount_pitch_r = round(mount_pitch, 2)
    mount_roll_r  = round(mount_roll, 2)

    print(f"    MOUNT_ROLL_DEG  = {mount_roll_r}°")
    print(f"    MOUNT_PITCH_DEG = {mount_pitch_r}°")
    print(f"    ACCEL_BIAS      = {accel_bias}")

    if abs(mount_pitch) > 30 or abs(mount_roll) > 30:
        print("    [BİLGİ] Montaj açısı >30° — bu normaldir; kalibrasyon bunu telafi eder.")
        print("            Tilt-compensation doğruluğu büyük açılarda azalabilir.")

    return mount_roll_r, mount_pitch_r, accel_bias


# ─────────────────────────────────────────────── adım 3: manyetometre

def calibrate_mag(imu, duration_s=15.0):
    """Hard-iron ofseti (MAG_OFFSET) ve soft-iron ölçeği (MAG_SCALE).
    Yöntem: min/max örnekleme → merkez = hard-iron, yarıçap oranı = soft-iron."""
    print(f"\n[3/3] MANYETOMETRe — {duration_s:.0f} sn boyunca ROV'u HER YÖNE ÇEVİR...")
    print("      Sağa-sola, yukarı-aşağı, tam tur — yavaşça ve sürekli")

    # Eger mag arizasi nedeniyle has_mag=False olduysa bunu iptal et
    if not getattr(imu, "has_mag", True):
        print("    [UYARI] Manyetometre erisilemedigi icin bu adim ATLANDI.")
        return (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)

    xs, ys, zs = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        try:
            m = imu.read_mag_ut_raw()
            if m:
                xs.append(m[0]); ys.append(m[1]); zs.append(m[2])
                elapsed = time.monotonic() - t0
                remaining = duration_s - elapsed
                print(f"    {remaining:4.1f}s kaldı  "
                      f"X:[{min(xs):5.0f},{max(xs):5.0f}]  "
                      f"Y:[{min(ys):5.0f},{max(ys):5.0f}]  "
                      f"Z:[{min(zs):5.0f},{max(zs):5.0f}] µT", end="\r")
        except OSError:
            pass
        time.sleep(0.05)
    print()
    print("    Tamam.")

    if not xs:
        print("    [UYARI] Manyetometreden hic veri alinamadi.")
        return (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)

    def offset_radius(vals):
        return (max(vals) + min(vals)) / 2.0, (max(vals) - min(vals)) / 2.0

    ox, rx = offset_radius(xs)
    oy, ry = offset_radius(ys)
    oz, rz = offset_radius(zs)
    avg_r = (rx + ry + rz) / 3.0

    for name, r in (("X", rx), ("Y", ry), ("Z", rz)):
        if r < 10.0:
            print(f"    [UYARI] {name} ekseni yarıçapı küçük ({r:.1f} µT) — "
                  "o yönde yeterince çevrilmedi. Tekrar dene!")
    if avg_r > 100.0:
        print(f"    [UYARI] Büyük manyetik bozucu ({avg_r:.0f} µT) — "
              "motor/metal/kablo yakında mı? Uzaklaştır.")

    offset = (round(ox, 2), round(oy, 2), round(oz, 2))
    scale  = (
        round(avg_r / rx, 3) if rx > 1e-6 else 1.0,
        round(avg_r / ry, 3) if ry > 1e-6 else 1.0,
        round(avg_r / rz, 3) if rz > 1e-6 else 1.0,
    )
    print(f"    MAG_OFFSET = {offset}")
    print(f"    MAG_SCALE  = {scale}")
    return offset, scale


# ─────────────────────────────────────────────── config.py yazar

def write_config(gyro_bias, accel_bias, mag_offset, mag_scale,
                 mount_roll, mount_pitch, path=None):
    """Tüm kalibrasyon parametrelerini config.py'ye yazar.
    Önce config.py.bak olarak yedek alır.
    Parametre zaten varsa → regex ile günceller.
    Yoksa → IMU kalibrasyon bloğuna ekler."""
    if path is None:
        path = CONFIG_PATH

    shutil.copy(path, path + ".bak")
    with open(path, encoding="utf-8") as f:
        text = f.read()

    def _set(text, key, value, comment):
        line = f"{key} = {value}  # {comment}"
        pattern = fr"^{re.escape(key.strip())}\s*=.*"
        if re.search(pattern, text, re.MULTILINE):
            return re.sub(pattern, line, text, flags=re.MULTILINE, count=1)
        else:
            # Yoksa dosyanin sonuna ekle
            return text.rstrip() + f"\n{line}\n"

    c = "calibrate_imu.py ile ölçüldü"
    text = _set(text, "GYRO_BIAS      ", str(gyro_bias),    c)
    text = _set(text, "ACCEL_BIAS     ", str(accel_bias),   c)
    text = _set(text, "MAG_OFFSET     ", str(mag_offset),   c)
    text = _set(text, "MAG_SCALE      ", str(mag_scale),    c)
    text = _set(text, "MOUNT_ROLL_DEG ", str(mount_roll),
                "IMU montaj roll ofseti — calibrate_imu.py")
    text = _set(text, "MOUNT_PITCH_DEG", str(mount_pitch),
                "IMU montaj pitch ofseti — calibrate_imu.py")

    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"\n    [OK] config.py güncellendi: {path}")
    print(f"    [OK] Yedek: {path}.bak")


# ─────────────────────────────────────────────── ana akış

def main():
    print("=" * 65)
    print("  EGE ROV — IMU KALİBRASYONU  (Eksen Bağımsız, 9-DoF)")
    print("=" * 65)
    print("""
Bu script IMU montaj açısından bağımsız çalışır.
ROV'u yatay tutmak GEREKMEZ — kendi doğal duruşunu referans alır.
Adımlar:
  1) Gyro    — ROV SABİT (5 sn)
  2) Accel   — ROV DOĞAL DURUMDA (5 sn)  ← montaj açısı da ölçülür
  3) Pusula  — ROV HER YÖNE ÇEVİR (15 sn)

Sonuçlar:
  GYRO_BIAS, ACCEL_BIAS, MAG_OFFSET, MAG_SCALE,
  MOUNT_PITCH_DEG, MOUNT_ROLL_DEG  → config.py'ye yazılır
""")
    input("Hazır olunca ENTER...")

    print("\nMPU-9250 bağlanıyor...")
    try:
        imu = Mpu9250()
        print("[OK] IMU bağlandı.")
    except Exception as e:
        print(f"\n[HATA] IMU baglanamadi: {e}")
        return

    # ── 1) Gyro
    gyro_bias = calibrate_gyro(imu, duration_s=5.0)

    # ── 2) Accel + Montaj açısı
    print()
    input("ROV'u doğal duruşuna getir (nasıl monte edilmişse öyle). ENTER...")
    mount_roll, mount_pitch, accel_bias = calibrate_accel_and_mount(imu, duration_s=5.0)

    # ── 3) Manyetometre
    from config import USE_MAGNETOMETER
    if USE_MAGNETOMETER:
        print()
        input("Pusula kalibrasyonu için ENTER (ardından 15 sn boyunca çevir)...")
        mag_offset, mag_scale = calibrate_mag(imu, duration_s=15.0)
    else:
        print("\n[BİLGİ] config.py'de USE_MAGNETOMETER = False. Manyetometre kalibrasyonu ATLANACAK.")
        mag_offset, mag_scale = (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)

    # ── Özet
    print("\n" + "=" * 65)
    print("ÖZET — config.py'ye yazılacak:")
    print(f"  GYRO_BIAS       = {gyro_bias}")
    print(f"  ACCEL_BIAS      = {accel_bias}")
    print(f"  MAG_OFFSET      = {mag_offset}")
    print(f"  MAG_SCALE       = {mag_scale}")
    print(f"  MOUNT_ROLL_DEG  = {mount_roll}°")
    print(f"  MOUNT_PITCH_DEG = {mount_pitch}°")
    print("=" * 65)

    write_config(gyro_bias, accel_bias, mag_offset, mag_scale, mount_roll, mount_pitch)

    print("""
Doğrulama:
  python3 tests/test_imu.py
  → ROV doğal durumundayken:
      Roll  ≈ 0°   (±2° normal)
      Pitch ≈ 0°   (±2° normal)
      Heading  kararlı, sürükleme yok

Görev başlatmak için:
  python3 video_main.py
""")


if __name__ == "__main__":
    main()
