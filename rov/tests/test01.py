import time, board, busio
from adafruit_pca9685 import PCA9685

from config import PWM_NEUTRAL_US, PCA9685_REF_CLOCK_HZ, FREQ_HZ

i2c = busio.I2C(board.SCL, board.SDA)
# reference_clock_speed olmadan kart 58.1Hz'e kacar, darbeler %14 kisalir.
p = PCA9685(i2c, address=0x40, reference_clock_speed=PCA9685_REF_CLOCK_HZ)
p.frequency = FREQ_HZ

PERIOD_US = 1_000_000 / FREQ_HZ

def us(ch, u):
    p.channels[ch].duty_cycle = int(u / PERIOD_US * 0xFFFF)

ch = int(input("Kanal no (0-5): "))
us(ch, PWM_NEUTRAL_US)
input("ESC'e guc ver, bip bitince Enter...")
print("ileri dusuk guc...")
us(ch, PWM_NEUTRAL_US + 100); time.sleep(2)
print("geri dusuk guc...")
us(ch, PWM_NEUTRAL_US - 100); time.sleep(2)
us(ch, PWM_NEUTRAL_US)
print(f"notr ({PWM_NEUTRAL_US}us). bitti.")