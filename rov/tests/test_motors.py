"""
Motor/ESC test scripti - PCA9685 uzerinden tek tek motor testi.

KULLANIM (Jetson uzerinde, PERVANELER TAKILI DEGILKEN ya da su icinde sabitlenmis halde):
    python3 test_motors.py            # interaktif menu
    python3 test_motors.py --all      # tum motorlari sirayla %10 dondur

GUVENLIK:
  - Ilk testte pervaneleri SOKMEN onerilir.
  - ESC guc verilmeden once script calisir olmali (arm sirasi asagida).
  - Ctrl+C her zaman motorlari notre ceker.
"""
import sys
import time

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from hal.thrusters import Thrusters, PCA9685Backend
from config import MOTOR_CHANNELS, PWM_NEUTRAL_US, PWM_DEADBAND_US, PWM_RANGE_US

# Olu bant esigi: bunun altindaki komutlar _to_us() icinde notre yuvarlanir,
# yani motor HIC donmez. Eski %10 degeri (40us < 45us olu bant) tam bu tuzaga
# dusuyordu ve "motorlar calismiyor" gibi gorunuyordu.
_MIN_EFFECTIVE = PWM_DEADBAND_US / PWM_RANGE_US        # ~0.1125
TEST_POWER = round(_MIN_EFFECTIVE + 0.08, 2)           # olu bandin guvenli ustu (~%19)
TEST_TIME_S = 2.0


def main():
    print("PCA9685'e baglaniliyor...")
    backend = PCA9685Backend()
    thr = Thrusters(backend)
    print(f"Baglandi. Notr sinyal gonderiliyor ({PWM_NEUTRAL_US}us).")

    input("ESC'lere GUC VER, bip seslerini bekle, sonra Enter'a bas...")
    thr.arm()
    print("Arm edildi.\n")

    try:
        if "--all" in sys.argv:
            for name in MOTOR_CHANNELS:
                run_one(thr, name)
        else:
            while True:
                print("Motorlar:", ", ".join(MOTOR_CHANNELS))
                sel = input("Motor adi (q=cikis): ").strip()
                if sel.lower() == "q":
                    break
                if sel not in MOTOR_CHANNELS:
                    print("Gecersiz isim.\n")
                    continue
                run_one(thr, sel)
    finally:
        thr.stop()
        print("Tum motorlar notrde. Cikis.")


def run_one(thr, name):
    print(f"{name} ileri %{int(TEST_POWER*100)} ({TEST_TIME_S}s)...")
    spin(thr, name, +TEST_POWER)
    print(f"{name} geri  %{int(TEST_POWER*100)} ({TEST_TIME_S}s)...")
    spin(thr, name, -TEST_POWER)
    print(f"{name} tamam. Donus yonunu not al!\n")


def spin(thr, name, power):
    t0 = time.monotonic()
    while time.monotonic() - t0 < TEST_TIME_S:
        thr.command({name: power})
        time.sleep(0.02)  # 50Hz
    # notre don
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:
        thr.command({name: 0.0})
        time.sleep(0.02)


if __name__ == "__main__":
    main()
