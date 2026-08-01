"""
PCA9685 kanal testi - birden fazla kanali sirayla test eder.

Kullanim (Jetson):
    sudo python3 kanal_test.py

Prosedur:
  1. ESC guc kablosu TAKILI DEGILKEN sinyal kablolarini kanallara tak.
  2. Scripti calistir, kanal numaralarini virgullu gir (orn: 0,1,2).
  3. Tum kanallara notr gonderilir -> ESC'lere guc ver, bipler bitince Enter.
  4. Her kanal sirayla 2 sn ileri, 2 sn geri doner. Aralarda Enter beklenir.
  5. Kanal + motor + donus yonunu not al, config.py'ye isle.

Guvenlik: pervanesiz test et!
"""
import sys
import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import PWM_NEUTRAL_US, FREQ_HZ

from hal.i2c import pca9685_ac


NEUTRAL_US = PWM_NEUTRAL_US
FWD_US = PWM_NEUTRAL_US + 150
REV_US = PWM_NEUTRAL_US - 150
SPIN_S = 2.0

# Baglanti bus NUMARASI ile kurulur (board.SCL/SDA yanlis busu seciyordu).
# PCA9685_REF_CLOCK_HZ sart: olculmemis 25MHz ile kart 58.1Hz'de calisir ve tum
# darbeler %14 kisalir (1500us -> 1291us), yani her motor her testte geri doner.
try:
    p = pca9685_ac(freq_hz=FREQ_HZ)
except Exception as e:
    print(f"\n[HATA] PCA9685 baglanamadi:\n{e}")
    print("\n  Teshis icin: python3 i2c_tara.py")
    sys.exit(1)


def us(ch, u):
    """Kanala mikrosaniye cinsinden darbe genisligi yaz."""
    p.set_us(ch, u)


while True:
    raw = input("Kanal numaralari (virgullu, orn 0,1,2): ")
    if not raw.strip():
        continue
    try:
        channels = [int(x) for x in raw.replace(" ", "").split(",")]
        break
    except ValueError:
        print("Hatali giris, lutfen sadece virgulle ayrilmis sayilar girin.")

for ch in channels:
    us(ch, NEUTRAL_US)
print(f"Kanallar {channels} notrde ({NEUTRAL_US}us).")
input("ESC'lere guc ver, bip sesleri bitince Enter...")

try:
    for ch in channels:
        input(f"\n--- Kanal {ch} testi icin Enter (q ile atlanmaz, Ctrl+C cikis) ---")
        print(f"kanal {ch} ileri ({FWD_US}us)...")
        us(ch, FWD_US)
        time.sleep(SPIN_S)
        us(ch, NEUTRAL_US)
        time.sleep(0.5)
        print(f"kanal {ch} geri ({REV_US}us)...")
        us(ch, REV_US)
        time.sleep(SPIN_S)
        us(ch, NEUTRAL_US)
        print(f"kanal {ch} tamam. Hangi motor dondu, yonu neydi - not al!")
finally:
    for ch in channels:
        us(ch, NEUTRAL_US)
    print("\nTum kanallar notrde. Bitti.")
