#!/usr/bin/env python3
"""
EGE ROV — MPU-9250 IMU ve Pusula Canlı Veri Test Scripti.

'calibrate_imu.py' çalıştırıldıktan SONRA kullanılır.
config.py dosyasındaki kalibrasyon değerlerini (GYRO_BIAS, MAG_OFFSET, MAG_SCALE)
kullanarak anlık Heading (Pusula Açı), Pitch (Yunuslama), Roll (Yatış) ve
Yaw Rate (Açısal Hız) verilerini canlı terminalde gösterir.

Kullanım (Jetson):
    python3 test_imu.py
"""
import sys
import time

from config import GYRO_BIAS, MAG_OFFSET, MAG_SCALE, I2C_BUS, IMU_ADDR
from sensors.imu import Mpu9250, Orientation


def main():
    print("=" * 70)
    print("        EGE ROV — MPU-9250 IMU VE PUSULA ANLIK VERİ TESTİ")
    print("=" * 70)

    print("\nYüklenen Kalibrasyon Parametreleri (config.py):")
    print(f"  GYRO_BIAS  = {GYRO_BIAS}")
    print(f"  MAG_OFFSET = {MAG_OFFSET}")
    print(f"  MAG_SCALE  = {MAG_SCALE}")
    print(f"  I2C Bus    = {I2C_BUS} (Adres: 0x{IMU_ADDR:02X})")
    print("-----------------------------------------------------------------")

    print("\nMPU-9250 IMU sensörüne bağlanılıyor...")
    mpu = None
    for b in [I2C_BUS, 8, 7, 1, 0, 2, 3]:
        try:
            mpu = Mpu9250(bus_num=b)
            ori = Orientation(mpu)
            print(f"[OK] MPU-9250 ve AK8963 Manyetometre I2C Bus {b} (0x68) üzerinde BAŞARIYLA BAĞLANDI!")
            break
        except Exception:
            continue

    if mpu is None:
        print(f"[HATA] MPU-9250 IMU Sensörüne hiçbir I2C busunda (8, 7, 1, 0) bağlanılamadı!")
        print("Kontrol edin:")
        print("  1. IMU kabloları HANGİ pinlere takılı? (Pin 3/5 mi yoksa Pin 27/28 mi?)")
        print("  2. 'i2cdetect -y -r 8' ve 'i2cdetect -y -r 1' komutlarında 0x68 adresi görünüyor mu?")
        print("  3. IMU üzerindeki VCC (3.3V) ve GND ışığı yanıyor mu?")
        sys.exit(1)

    print("\nFiltre ısınması için 1 saniye bekleniyor...")
    for _ in range(20):
        ori.update()
        time.sleep(0.05)

    print("\n-----------------------------------------------------------------")
    print("CANLI IMU YÖNELİM VERİLERİ OKUNUYOR (Çıkış için Ctrl+C'ye basın)...")
    print("-----------------------------------------------------------------")
    print(f"{'Pusula (Heading)':<18} | {'Yunuslama (Pitch)':<19} | {'Yatış (Roll)':<15} | {'Dönüş Hızı (YawRate)':<20}")
    print("-" * 75)

    max_samples = None
    for arg in sys.argv[1:]:
        if arg.startswith("--samples="):
            max_samples = int(arg.split("=")[1])
        elif arg == "--once":
            max_samples = 1

    sample_count = 0
    try:
        while True:
            hdg = ori.update()
            pitch = ori.pitch
            roll = ori.roll
            yaw_rate = ori.yaw_rate

            end_char = "\n" if max_samples is not None else "\r"
            sys.stdout.write(
                f"  {hdg:6.1f}° (Heading)    | "
                f"  {pitch:6.1f}° (Pitch)     | "
                f"  {roll:6.1f}° (Roll)   | "
                f"  {yaw_rate:6.2f} °/s{end_char}"
            )
            sys.stdout.flush()

            sample_count += 1
            if max_samples and sample_count >= max_samples:
                print("\n[OK] İstenen sayıda okuma alındı.")
                break

            time.sleep(0.05)  # 20 Hz canlı yayın

    except KeyboardInterrupt:
        print("\n\nTest durduruldu. IMU ve Pusula Veri Akışı BAŞARILI!")


if __name__ == "__main__":
    main()
