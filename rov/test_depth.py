#!/usr/bin/env python3
"""
EGE ROV — MS5837-30BA Derinlik ve Basınç Sensörü Doğrulama Testi.

Karada (su dışında) çalıştırılır. 
Sensörün I2C bağlantısını, PROM katsayılarını, ortam basıncını (mbar) ve sıcaklığını (°C) test eder.

Kullanım (Jetson):
    python3 test_depth.py

İpuçları:
  - Karadayken basınç ~1013 mbar civarında okunmalıdır.
  - Sensör deliğine hafifçe üflediğinizde veya parmağınızla hafif yaklaştığınızda
    basınç ve derinlik değerinin yükseldiğini görebilirsiniz.
"""
import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    print("[HATA] 'smbus2' kütüphanesi bulunamadı! Yüklemek için: pip3 install smbus2")
    sys.exit(1)

# I2C Ayarları
I2C_BUSES_TO_TRY = [7, 8, 1, 0]  # Jetson Orin Nano pin 3/5 genelde Bus 7 veya Bus 8
DEPTH_ADDR = 0x76                # MS5837-30BA varsayılan I2C adresi

# MS5837 Komutları
CMD_RESET = 0x1E
CMD_PROM_READ = 0xA0
CMD_CONV_D1 = 0x4A  # Basınç ADC dönüşümü (OSR=8192)
CMD_CONV_D2 = 0x5A  # Sıcaklık ADC dönüşümü (OSR=8192)
CMD_ADC_READ = 0x00


class MS5837Tester:
    def __init__(self):
        self.bus = None
        self.bus_num = None
        self.C = []
        self.surface_pressure_mbar = None

    def find_and_connect(self):
        """Mevcut I2C buslarını tarar ve MS5837 sensörüne bağlanır."""
        print("I2C Busları taranıyor...")
        for b_num in I2C_BUSES_TO_TRY:
            try:
                bus = SMBus(b_num)
                # Reset komutu göndererek sensörü test et
                bus.write_byte(DEPTH_ADDR, CMD_RESET)
                time.sleep(0.05)
                self.bus = bus
                self.bus_num = b_num
                print(f"[OK] MS5837 sensörü I2C Bus {b_num} (Adres 0x76) üzerinde BULUNDU!")
                return True
            except OSError:
                continue
            except Exception as e:
                continue

        print("[HATA] MS5837-30BA sensörü hiçbir I2C busunda (7, 8, 1, 0) bulunamadı!")
        print("Kontrol edin:")
        print("  1. VCC (3.3V) ve GND kabloları bağlı mı?")
        print("  2. SDA -> Jetson Pin 3, SCL -> Jetson Pin 5 bağlı mı?")
        print("  3. Terminalde 'i2cdetect -y 7' çıktısında 0x76 görünüyor mu?")
        return False

    def read_prom(self):
        """Sensörün fabrika kalibrasyon katsayılarını (PROM C1..C6) okur."""
        print("\nFabrika Kalibrasyon Katsayıları (PROM) okunuyor:")
        self.C = []
        for i in range(7):
            try:
                d = self.bus.read_i2c_block_data(DEPTH_ADDR, CMD_PROM_READ + 2 * i, 2)
                val = d[0] << 8 | d[1]
                self.C.append(val)
                if i > 0:
                    print(f"  C{i} = {val}")
            except Exception as e:
                print(f"[HATA] PROM C{i} okunamadı: {e}")
                return False
        return True

    def _convert(self, cmd):
        """ADC dönüşüm komutunu başlatır ve 24-bit sonucu okur."""
        self.bus.write_byte(DEPTH_ADDR, cmd)
        time.sleep(0.02)  # OSR=8192 için dönüşüm süresi ~18ms
        d = self.bus.read_i2c_block_data(DEPTH_ADDR, CMD_ADC_READ, 3)
        return d[0] << 16 | d[1] << 8 | d[2]

    def read_sensor(self):
        """
        Ham ADC verilerini okur ve datasheet 1. derece kompanzasyon formülüyle
        basınç (mbar) ve sıcaklık (°C) değerlerini hesaplar.
        """
        D1 = self._convert(CMD_CONV_D1)  # Ham Basınç
        D2 = self._convert(CMD_CONV_D2)  # Ham Sıcaklık

        C = self.C
        dT = D2 - C[5] * 256
        TEMP = 2000 + (dT * C[6]) / 8388608  # 1/100 °C

        SENS = C[1] * 32768 + (C[3] * dT) / 256
        OFF = C[2] * 65536 + (C[4] * dT) / 128
        P = (D1 * SENS / 2097152 - OFF) / 8192  # 1/10 mbar

        pressure_mbar = P / 10.0
        temp_c = TEMP / 100.0

        return pressure_mbar, temp_c

    def calibrate_surface(self):
        """Karadaki mevcut atmosferik basıncı yüzey referansı (0.00 m) olarak kaydeder."""
        pressures = []
        print("\nYüzey basıncı kalibre ediliyor (10 örnek alınıyor)...")
        for _ in range(10):
            p, _ = self.read_sensor()
            pressures.append(p)
            time.sleep(0.05)
        self.surface_pressure_mbar = sum(pressures) / len(pressures)
        print(f"[REFERANS] Karadaki Sıfır Derinlik Basıncı: {self.surface_pressure_mbar:.2f} mbar")


def main():
    print("=" * 65)
    print("      EGE ROV — MS5837-30BA DERİNLİK VE BASINÇ SENSÖRÜ TESTİ")
    print("=" * 65)

    tester = MS5837Tester()

    if not tester.find_and_connect():
        sys.exit(1)

    if not tester.read_prom():
        sys.exit(1)

    tester.calibrate_surface()

    print("\n-----------------------------------------------------------------")
    print("CANLI SENSÖR VERİLERİ OKUNUYOR (Çıkış için Ctrl+C'ye basın)...")
    print("-----------------------------------------------------------------")
    print(f"{'Sıcaklık (°C)':<15} | {'Basınç (mbar)':<16} | {'Hesaplanan Derinlik (m)':<25}")
    print("-" * 65)

    try:
        while True:
            p_mbar, temp_c = tester.read_sensor()
            
            # Derinlik hesabı: 1 mbar ≈ 0.010197 m tatlı su (997 kg/m³)
            # Karadayken p_mbar - surface_pressure ~ 0 olur
            p_diff = p_mbar - tester.surface_pressure_mbar
            depth_m = max(0.0, (p_diff * 100.0) / (997.0 * 9.81))

            print(f" {temp_c:6.2f} °C        | {p_mbar:8.2f} mbar       | {depth_m:6.3f} m", end="\r")
            time.sleep(0.2)  # 5Hz canlı yayın

    except KeyboardInterrupt:
        print("\n\nTest durduruldu. MS5837 sensör bağlantısı BAŞARILI!")


if __name__ == "__main__":
    main()
