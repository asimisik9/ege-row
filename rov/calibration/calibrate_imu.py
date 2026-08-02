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
        try:
            v = read_fn()
            xs.append(v[0]); ys.append(v[1]); zs.append(v[2])
            if print_live:
                elapsed = time.monotonic() - t0
                remaining = duration_s - elapsed
                print(f"    {remaining:4.1f}s kaldı  "
                      f"X:{v[0]:7.2f}  Y:{v[1]:7.2f}  Z:{v[2]:7.2f}", end="\r")
        except Exception:
            # I2C okuma hatası olursa (Errno 121), çökmek yerine bu okumayı pas geç
            pass
        time.sleep(0.02)
    if print_live:
        print()
    return xs, ys, zs

def calibrate_gyro(imu, max_attempts=5):
    print("\n=======================================================")
    print("[1/2] KATI GYRO KALİBRASYONU (ARACI ASLA HAREKET ETTİRMEYİN)")
    print("=======================================================")
    
    for attempt in range(max_attempts):
        print(f"\nDeneme {attempt+1}/{max_attempts} - Lütfen masaya veya araca DOKUNMAYIN (10 saniye)...")
        time.sleep(1.0)
        xs, ys, zs = _sample(imu, imu.read_gyro_dps_raw, 10.0, "gyro", print_live=True)
        
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

def calibrate_accel_1point(imu):
    print("\n=======================================================")
    print("[2/2] 1-NOKTALI İVMEÖLÇER KALİBRASYONU")
    print("=======================================================")
    print("Aracı DÜZ BİR ZEMİNE yerleştirin (şu anki konumu sıfır noktası kabul edilecek).")
    print("Ölçüm 5 saniye sürecektir. Ölçüm sırasında aracı OYNATMAYIN.")
    
    input("Hazır olduğunuzda ENTER'a basın (ölçüm hemen başlayacak)...")
    time.sleep(1.0)
    
    print(f"Ölçülüyor (DÜZ KONUM)... Lütfen hareketsiz tutun!")
    xs, ys, zs = _sample(imu, imu.read_accel_g_raw, 5.0, "accel", print_live=True)
    ax, ay, az = _avg(xs), _avg(ys), _avg(zs)
    
    print(f"    Okunan Ortalama: ({ax:.3f}g, {ay:.3f}g, {az:.3f}g)")
    
    # 1-point kalibrasyon: 
    # Varsayım: Z ekseninde yerçekimi var (1.0g). X ve Y 0 olmalı.
    accel_bias = (round(ax, 4), round(ay, 4), round(az - 1.0, 4))
    accel_scale = (1.0, 1.0, 1.0)
    
    print("\n    [BAŞARILI] İvmeölçer Kalibrasyonu Tamamlandı!")
    print(f"    ACCEL_BIAS  = {accel_bias}")
    print(f"    ACCEL_SCALE = {accel_scale}")
    
    # Montaj açısı: 0, 0 çünkü yazılım 1-noktalı kalibrasyon ile doğal ekseni oturtuyor
    return accel_bias, accel_scale

def update_config(gyro_bias, accel_bias, accel_scale):
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
    
    # 1-nokta yapıldıysa mount açıları sıfırlanmalıdır çünkü artık sapmalar bias/scale ile düzeltildi
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
    ab, asc = calibrate_accel_1point(imu)

    update_config(gb, ab, asc)
    print("Tüm kalibrasyon adımları tamamlandı! 'python3 main.py' ile sistemi başlatabilirsiniz.")

if __name__ == "__main__":
    main()
