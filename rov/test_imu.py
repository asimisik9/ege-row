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
    try:
        mpu = Mpu9250()
        ori = Orientation(mpu)
        print("[OK] MPU-9250 ve AK8963 Manyetometre BAŞARIYLA BAĞLANDI!")
    except Exception as e:
        print(f"[HATA] IMU Sensörüne bağlanılamadı: {e}")
        print("Kontrol edin:")
        print(f"  1. 'i2cdetect -y {I2C_BUS}' çıktısında 0x68 (MPU) görünüyor mu?")
        print("  2. SDA / SCL ve 3.3V / GND kabloları bağlı mı?")
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

    try:
        while True:
            hdg = ori.update()
            pitch = ori.pitch
            roll = ori.roll
            yaw_rate = ori.yaw_rate

            # Terminal satırını dinamik güncelle (\r)
            sys.stdout.write(
                f"\r  {hdg:6.1f}° (Pusula)     | "
                f"  {pitch:6.1f}° (Pitch)     | "
                f"  {roll:6.1f}° (Roll)   | "
                f"  {yaw_rate:6.2f} °/s        "
            )
            sys.stdout.flush()
            time.sleep(0.05)  # 20 Hz canlı yayın

    except KeyboardInterrupt:
        print("\n\nTest durduruldu. IMU ve Pusula Veri Akışı BAŞARILI!")


if __name__ == "__main__":
    main()
