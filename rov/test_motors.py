#!/usr/bin/env python3
"""
EGE ROV — PCA9685 Motor Sürücü Tam Teşhis ve PWM Test Scripti

Tüm I2C bus kombinasyonlarını dener, PCA9685'i bulur ve gerçek PWM sinyali gönderir.

Kullanım:
    python3 rov/test_motors.py          # Sadece teşhis
    python3 rov/test_motors.py --pwm    # Teşhis + aktif PWM testi (motor döner!)
"""
import sys
import time

print("=" * 70)
print("   EGE ROV — PCA9685 MOTOR SÜRÜCÜ TEŞHİS VE PWM SİNYAL TESTİ")
print("=" * 70)

# ---------------------------------------------------------------- ADIM 1: smbus2 ile ham I2C tarama
print("\n[ADIM 1] smbus2 ile tüm I2C busları taranıyor...")
try:
    from smbus2 import SMBus
    found_on_buses = {}
    for bus_num in [8, 7, 1, 0, 2, 3, 4, 5, 6]:
        try:
            bus = SMBus(bus_num)
            found_addrs = []
            for addr in range(0x03, 0x78):
                try:
                    bus.read_byte(addr)
                    found_addrs.append(hex(addr))
                except Exception:
                    pass
            bus.close()
            if found_addrs:
                found_on_buses[bus_num] = found_addrs
                print(f"  Bus {bus_num}: {found_addrs}")
        except Exception as e:
            pass
    if not found_on_buses:
        print("  [HATA] Hiçbir I2C busunda cihaz bulunamadı!")
    else:
        print(f"\n  [OK] PCA9685 (0x40) şu buslarda: {[b for b, a in found_on_buses.items() if '0x40' in a]}")
except ImportError:
    print("  [HATA] smbus2 bulunamadı: pip3 install smbus2")

# ---------------------------------------------------------------- ADIM 2: board pin haritası
print("\n[ADIM 2] Adafruit Blinka board pin haritası kontrol ediliyor...")
try:
    import board
    pins = dir(board)
    scl_pins = [p for p in pins if 'SCL' in p]
    sda_pins = [p for p in pins if 'SDA' in p]
    print(f"  Mevcut SCL pinleri: {scl_pins}")
    print(f"  Mevcut SDA pinleri: {sda_pins}")
    for scl_name, sda_name in [('SCL_1', 'SDA_1'), ('SCL', 'SDA')]:
        if hasattr(board, scl_name) and hasattr(board, sda_name):
            scl = getattr(board, scl_name)
            sda = getattr(board, sda_name)
            print(f"  board.{scl_name} = {scl}  |  board.{sda_name} = {sda}")
except ImportError:
    print("  [HATA] board/blinka bulunamadı: pip3 install adafruit-blinka")

# ---------------------------------------------------------------- ADIM 3: busio.I2C ile bağlantı dene
print("\n[ADIM 3] busio.I2C kombinasyonları deneniyor (PCA9685 0x40 için)...")
working_i2c = None
working_combo = None
try:
    import busio
    combos = []
    for scl_name, sda_name in [('SCL_1', 'SDA_1'), ('SCL', 'SDA'), ('SCL_2', 'SDA_2')]:
        if hasattr(board, scl_name) and hasattr(board, sda_name):
            combos.append((scl_name, sda_name, getattr(board, scl_name), getattr(board, sda_name)))
    
    for scl_name, sda_name, scl_pin, sda_pin in combos:
        try:
            i2c = busio.I2C(scl_pin, sda_pin)
            # I2C scan
            while not i2c.try_lock():
                pass
            addrs = [hex(a) for a in i2c.scan()]
            i2c.unlock()
            print(f"  board.{scl_name}/{sda_name}: cihazlar = {addrs}")
            if '0x40' in addrs:
                print(f"  [OK] PCA9685 (0x40) board.{scl_name}/{sda_name} üzerinde BULUNDU!")
                working_i2c = i2c
                working_combo = (scl_name, sda_name)
        except Exception as e:
            print(f"  board.{scl_name}/{sda_name}: HATA - {e}")
except Exception as e:
    print(f"  [HATA] busio import edilemedi: {e}")

# ---------------------------------------------------------------- ADIM 4: PCA9685 başlat
print("\n[ADIM 4] PCA9685 başlatılıyor...")
pca = None
if working_i2c is not None:
    try:
        from adafruit_pca9685 import PCA9685
        pca = PCA9685(working_i2c, address=0x40)
        pca.frequency = 50
        print(f"  [OK] PCA9685 başlatıldı! Frekans = {pca.frequency} Hz")
        print(f"  [OK] Bağlantı: board.{working_combo[0]}/{working_combo[1]}")
    except Exception as e:
        print(f"  [HATA] PCA9685 başlatılamadı: {e}")
else:
    print("  [ATLA] Çalışan I2C bağlantısı bulunamadığı için PCA9685 atlandı.")

# ---------------------------------------------------------------- ADIM 5: PWM sinyali testi
if "--pwm" in sys.argv and pca is not None:
    print("\n[ADIM 5] PWM SİNYAL TESTİ (UYARI: Motorlar dönecek!)")
    print("  ESC'lere güç verildiğinden emin olun. 3 saniye bekleniyor...")
    time.sleep(3)

    def set_us(ch, us):
        duty = int(max(1100, min(1900, us)) / 20000.0 * 0xFFFF)
        pca.channels[ch].duty_cycle = duty

    # Kanalları 1500us nötr'e al
    print("  Tüm kanallar 1500us NÖTR...")
    for ch in range(8):
        set_us(ch, 1500)
    time.sleep(3)

    # Test sinyali gönder
    print("  Kanal 0 -> 1600us (hafif ileri)...")
    set_us(0, 1600)
    time.sleep(2)

    print("  Kanal 0 -> 1500us (nötr)...")
    set_us(0, 1500)
    time.sleep(1)

    print("  [OK] PWM testi tamamlandı!")
elif "--pwm" in sys.argv and pca is None:
    print("\n[ADIM 5] PCA9685 bulunamadı, PWM testi yapılamadı.")
else:
    print("\n[ADIM 5] PWM testi atlandı. Motor testi için: python3 rov/test_motors.py --pwm")

# ---------------------------------------------------------------- ÖZET
print("\n" + "=" * 70)
print("ÖZET:")
if pca is not None:
    print(f"  ✓ PCA9685 bağlı: board.{working_combo[0]}/{working_combo[1]}")
    print(f"  ✓ config.py güncelleme önerisi:")
    print(f"    simple_wasd.py ve hal/thrusters.py içindeki i2c satırını şu şekilde ayarlayın:")
    if working_combo:
        print(f"    i2c = busio.I2C(board.{working_combo[0]}, board.{working_combo[1]})")
else:
    print("  ✗ PCA9685 hiçbir kombinasyonda bulunamadı.")
    print("    - 'i2cdetect -y -r 8' çıktısında 0x40 görünüyor mu?")
    print("    - adafruit-blinka ve adafruit-circuitpython-pca9685 kurulu mu?")
print("=" * 70)
