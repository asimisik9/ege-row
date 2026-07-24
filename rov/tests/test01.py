import time, board, busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
p = PCA9685(i2c, address=0x40)
p.frequency = 50

def us(ch, u):
    p.channels[ch].duty_cycle = int(u / 20000 * 0xFFFF)

from config import PWM_NEUTRAL_US

ch = int(input("Kanal no (0-5): "))
us(ch, PWM_NEUTRAL_US)
input("ESC'e guc ver, bip bitince Enter...")
print("ileri dusuk guc...")
us(ch, PWM_NEUTRAL_US + 100); time.sleep(2)
print("geri dusuk guc...")
us(ch, PWM_NEUTRAL_US - 100); time.sleep(2)
us(ch, PWM_NEUTRAL_US)
print(f"notr ({PWM_NEUTRAL_US}us). bitti.")