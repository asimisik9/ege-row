"""
EGE ROV — ArduPilot Stili Kusursuz IMU Kalibrasyon Scripti (6-Point Accel + Strict Gyro)

Pixhawk ve ArduPilot mimarisine uygun şekilde:
1) Gyro Bias: Araç TİTREŞİMSİZ durumda iken ölçülür. Titreşim varsa kabul edilmez.
2) Accel Bias ve Scale: Araç 6 farklı eksende (Alt, Üst, Sağ, Sol, Ön, Arka) 
   yere dik olarak konumlandırılarak X, Y, Z için offset ve kazanç (scale) hataları hesaplanır.
3) Mag Bias ve Scale: Küresel çevirme ile hesaplanır (Manyetometre aktifse).

Kullanım:
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

try:
    from sensors.imu import Mpu9250
except ImportError as e:
    print(f"IMU modülü yüklenemedi: {e}")
    sys.exit(1)

def _avg(vals):
    return sum(vals) / len(vals)

def _sample(imu, read_fn, duration_s, label, print_live=False):
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

def calibrate_gyro(imu, max_attempts=5):
    print("\n=======================================================")
    print("[1/3] KATI GYRO KALİBRASYONU (ARACI ASLA HAREKET ETTİRMEYİN)")
    print("=======================================================")
    
    for attempt in range(max_attempts):
        print(f"\nDeneme {attempt+1}/{max_attempts} - Lütfen masaya veya araca DOKUNMAYIN (5 saniye)...")
        time.sleep(1.0)
        xs, ys, zs = _sample(imu, imu.read_gyro_dps_raw, 5.0, "gyro", print_live=True)
        
        std_x = statistics.pstdev(xs)
        std_y = statistics.pstdev(ys)
        std_z = statistics.pstdev(zs)
        max_std = max(std_x, std_y, std_z)
        
        bias = (round(_avg(xs), 4), round(_avg(ys), 4), round(_avg(zs), 4))
        print(f"    Gürültü (StdDev): X:{std_x:.3f}, Y:{std_y:.3f}, Z:{std_z:.3f} dps")
        
        if max_std > 0.5:
            print("    [HATA] Gürültü çok yüksek (Hareket Algılandı!). Araç titriyor veya masaya dokundunuz.")
            if attempt < max_attempts - 1:
                print("    Tekrar deneniyor...")
                continue
            else:
                print("    [KRİTİK] Gyro kalibrasyonu BAŞARISIZ oldu! Sensör arızalı olabilir.")
                sys.exit(1)
                
        print(f"    [BAŞARILI] Gyro Bias bulundu: {bias}")
        return bias
        
    return (0.0, 0.0, 0.0)

def wait_for_user(prompt):
    print(f"\n---> {prompt}")
    input("Hazır olduğunuzda ENTER'a basın (ölçüm hemen başlayacak)...")
    time.sleep(1.0) # ENTER'a basarken oluşan sarsıntıyı sönümle

def calibrate_accel_6point(imu):
    print("\n=======================================================")
    print("[2/3] 6-NOKTALI İVMEÖLÇER KALİBRASYONU (PIXHAWK YÖNTEMİ)")
    print("=======================================================")
    print("Aracı sırasıyla 6 farklı eksende DÜZ BİR ZEMİNE yerleştirmeniz istenecek.")
    print("Her ölçüm 3 saniye sürecektir. Ölçüm sırasında aracı OYNATMAYIN.")
    
    positions = [
        "NORMAL (Alt Yüzey Yere Bakıyor)",
        "TERS (Üst Yüzey Yere Bakıyor)",
        "SAĞ YAN (Sağ Yüzey Yere Bakıyor)",
        "SOL YAN (Sol Yüzey Yere Bakıyor)",
        "BURUN AŞAĞI (Ön Yüzey Yere Bakıyor)",
        "BURUN YUKARI (Arka Yüzey Yere Bakıyor)"
    ]
    
    measurements = []
    
    for pos in positions:
        wait_for_user(pos)
        print(f"Ölçülüyor ({pos})... Lütfen hareketsiz tutun!")
        xs, ys, zs = _sample(imu, imu.read_accel_g_raw, 3.0, "accel", print_live=True)
        ax, ay, az = _avg(xs), _avg(ys), _avg(zs)
        measurements.append((ax, ay, az))
        print(f"    Okunan Ortalama: ({ax:.3f}g, {ay:.3f}g, {az:.3f}g)")

    # 6 ölçümden X, Y, Z eksenleri için Min ve Max değerlerini bul
    x_vals = [m[0] for m in measurements]
    y_vals = [m[1] for m in measurements]
    z_vals = [m[2] for m in measurements]
    
    bias_x = (max(x_vals) + min(x_vals)) / 2.0
    bias_y = (max(y_vals) + min(y_vals)) / 2.0
    bias_z = (max(z_vals) + min(z_vals)) / 2.0
    
    scale_x = 2.0 / (max(x_vals) - min(x_vals)) if (max(x_vals) - min(x_vals)) > 0 else 1.0
    scale_y = 2.0 / (max(y_vals) - min(y_vals)) if (max(y_vals) - min(y_vals)) > 0 else 1.0
    scale_z = 2.0 / (max(z_vals) - min(z_vals)) if (max(z_vals) - min(z_vals)) > 0 else 1.0
    
    accel_bias = (round(bias_x, 4), round(bias_y, 4), round(bias_z, 4))
    accel_scale = (round(scale_x, 4), round(scale_y, 4), round(scale_z, 4))
    
    print("\n    [BAŞARILI] İvmeölçer Kalibrasyonu Tamamlandı!")
    print(f"    ACCEL_BIAS  = {accel_bias}")
    print(f"    ACCEL_SCALE = {accel_scale}")
    
    # Montaj açısı: 0, 0 çünkü yazılım 6-noktalı kalibrasyon ile doğal ekseni oturtuyor
    # Eger kullanici araci kasten egik monte ettiyse ekstra mount pitch/roll eklenebilir 
    # ama o zaman normal ekseni (1. olcum) referans alinmalidir. Biz simdilik 0 verelim.
    # Pixhawk mantiginda Board Orientation ayri ayarlanir.
    return accel_bias, accel_scale

def calibrate_mag(imu, duration_s=20.0):
    print("\n=======================================================")
    print("[3/3] MANYETOMETRE KALİBRASYONU (KÜRESEL)")
    print("=======================================================")
    if not getattr(imu, "has_mag", True):
        print("    [UYARI] Manyetometre donanımda bulunamadı, bu adım ATLANDI.")
        return (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)
        
    print(f"    Önümüzdeki {duration_s:.0f} saniye boyunca aracı HAVA DA, HER EKSENDE ÇEVİRİN (Sekiz çizin).")
    input("Hazır olduğunuzda ENTER'a basın...")
    
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
    
    if not xs:
        print("    [UYARI] Manyetometreden hiç veri alınamadı.")
        return (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)
        
    def offset_radius(vals):
        return (max(vals) + min(vals)) / 2.0, (max(vals) - min(vals)) / 2.0

    ox, rx = offset_radius(xs)
    oy, ry = offset_radius(ys)
    oz, rz = offset_radius(zs)
    
    avg_r = (rx + ry + rz) / 3.0
    scale_x = avg_r / rx if rx > 0 else 1.0
    scale_y = avg_r / ry if ry > 0 else 1.0
    scale_z = avg_r / rz if rz > 0 else 1.0
    
    mag_offset = (round(ox, 2), round(oy, 2), round(oz, 2))
    mag_scale = (round(scale_x, 3), round(scale_y, 3), round(scale_z, 3))
    
    print("    [BAŞARILI] Manyetometre Kalibrasyonu Tamamlandı!")
    print(f"    MAG_OFFSET = {mag_offset}")
    print(f"    MAG_SCALE  = {mag_scale}")
    
    return mag_offset, mag_scale

def update_config(gyro_bias, accel_bias, accel_scale, mag_offset, mag_scale):
    print("\n=======================================================")
    print("CONFIG.PY GÜNCELLENİYOR...")
    print("=======================================================")
    
    shutil.copyfile(CONFIG_PATH, CONFIG_PATH + ".bak")
    print(f"    Yedek alındı: {CONFIG_PATH}.bak")

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        text = f.read()

    def replace_tuple(var_name, new_val, content):
        pattern = r"^(" + var_name + r"\s*=\s*)\(?[-\d\.,\s]+\)?"
        replacement = f"\\g<1>{new_val}"
        return re.sub(pattern, replacement, content, flags=re.MULTILINE)
        
    def replace_float(var_name, new_val, content):
        pattern = r"^(" + var_name + r"\s*=\s*)[-\d\.]+"
        replacement = f"\\g<1>{new_val}"
        return re.sub(pattern, replacement, content, flags=re.MULTILINE)

    text = replace_tuple("GYRO_BIAS", gyro_bias, text)
    text = replace_tuple("ACCEL_BIAS", accel_bias, text)
    text = replace_tuple("ACCEL_SCALE", accel_scale, text)
    text = replace_tuple("MAG_OFFSET", mag_offset, text)
    text = replace_tuple("MAG_SCALE", mag_scale, text)
    
    # 6-nokta yapıldıysa mount açıları sıfırlanmalıdır çünkü artık sapmalar bias/scale ile düzeltildi
    text = replace_float("MOUNT_ROLL_DEG", "0.0", text)
    text = replace_float("MOUNT_PITCH_DEG", "0.0", text)

    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        f.write(text)

    print("    config.py başarıyla güncellendi!")
    print("=======================================================\n")

def main():
    print("EGE ROV Kapsayıcı IMU Kalibrasyon Aracı Başlıyor...\n")
    try:
        imu = Mpu9250()
    except Exception as e:
        print(f"IMU başlatılamadı: {e}")
        return

    gb = calibrate_gyro(imu)
    ab, asc = calibrate_accel_6point(imu)
    mo, msc = calibrate_mag(imu)

    update_config(gb, ab, asc, mo, msc)
    print("Tüm kalibrasyon adımları tamamlandı! 'python3 main.py' ile sistemi başlatabilirsiniz.")

if __name__ == "__main__":
    main()
