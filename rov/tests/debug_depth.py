#!/usr/bin/env python3
"""
EGE ROV — MS5837 Derinlik Sensörü Hata Teşhis Scripti.
Ms5837 sınıfının ham ADC okumalarını, PROM katsayılarını ve hesaplama adımlarını ekrana basar.
"""
import sys
import time

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import I2C_BUS, DEPTH_ADDR
from sensors.depth import Ms5837

def main():
    print("=" * 60)
    print("   MS5837 DERİNLİK SENSÖRÜ DETAYLI TEŞHİS TESTİ")
    print("=" * 60)
    
    print(f"Config I2C_BUS: {I2C_BUS}, DEPTH_ADDR: 0x{DEPTH_ADDR:02X}")
    
    try:
        sensor = Ms5837()
        print(f"[OK] Sensör nesnesi oluşturuldu. Bağlı I2C Bus: {sensor.bus.bus if hasattr(sensor.bus, 'bus') else 'Bilinmiyor'}")
    except Exception as e:
        print(f"[HATA] Sensör başlatılamadı: {e}")
        sys.exit(1)

    print("\nPROM Katsayıları (C0..C6):")
    for i, val in enumerate(sensor.C):
        print(f"  C{i} = {val}")

    print("\nAdım Adım Hesaplama Tespiti (5 örnek):")
    print("-" * 60)
    
    for k in range(1, 6):
        try:
            D1 = sensor._convert(sensor.CMD_CONV_D1)
            D2 = sensor._convert(sensor.CMD_CONV_D2)
            C = sensor.C
            
            dT = D2 - C[5] * 256
            SENS = C[1] * 32768 + (C[3] * dT) / 256
            OFF = C[2] * 65536 + (C[4] * dT) / 128
            P = (D1 * SENS / 2097152 - OFF) / 8192
            p_mbar = P / 10.0
            
            print(f"Örnek #{k}:")
            print(f"  Ham D1 (Basınç ADC)  : {D1}")
            print(f"  Ham D2 (Sıcaklık ADC): {D2}")
            print(f"  dT = {dT}")
            print(f"  SENS = {SENS}")
            print(f"  OFF = {OFF}")
            print(f"  P (ham) = {P}")
            print(f"  Hesaplanan Basınç  : {p_mbar:.2f} mbar")
            print(f"  read_pressure_mbar(): {sensor.read_pressure_mbar():.2f} mbar")
        except Exception as e:
            print(f"Örnek #{k} HATA: {e}")
        time.sleep(0.5)

    print("=" * 60)

if __name__ == "__main__":
    main()
