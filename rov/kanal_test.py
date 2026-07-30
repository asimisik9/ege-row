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
import time
from config import PWM_NEUTRAL_US, PWM_MIN_US, PWM_MAX_US

import board
import busio
from adafruit_pca9685 import PCA9685


NEUTRAL_US = PWM_NEUTRAL_US
FWD_US = PWM_NEUTRAL_US + 150
REV_US = PWM_NEUTRAL_US - 150
SPIN_S = 2.0

i2c = busio.I2C(board.SCL, board.SDA)
p = PCA9685(i2c, address=0x40)
p.frequency = 50



def us(ch, u):
    """Mikrosaniye -> 16-bit duty cycle (50Hz = 20000us periyot)."""
    p.channels[ch].duty_cycle = int(u / 20000 * 0xFFFF)


raw = input("Kanal numaralari (virgullu, orn 0,1,2): ")
channels = [int(x) for x in raw.replace(" ", "").split(",")]

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
