"""
PCA9685 CH0..CH5 uzerindeki 6 motoru ayni PWM degerinde birlikte test eder.

Ornekler:
    python3 tum_motorlari_pwm_test.py 1850
    python3 tum_motorlari_pwm_test.py 1650
    python3 tum_motorlari_pwm_test.py 1850 --sure 4

Her testten once ve test bittiginde motorlar 1750 us notre alinir.
"""
import argparse
import time

from config import (
    MOTOR_CHANNELS,
    PWM_MAX_US,
    PWM_MIN_US,
    PWM_NEUTRAL_US,
)
from hal.thrusters import PCA9685Backend, Thrusters


def parse_args():
    parser = argparse.ArgumentParser(
        description="6 motoru ayni anda verilen PWM degerinde calistir."
    )
    parser.add_argument(
        "pwm",
        type=int,
        help=f"PWM darbe genisligi, {PWM_MIN_US}..{PWM_MAX_US} us",
    )
    parser.add_argument(
        "--sure",
        type=float,
        default=2.0,
        help="Motorlarin calisma suresi, saniye (varsayilan: 2)",
    )
    args = parser.parse_args()

    if not PWM_MIN_US <= args.pwm <= PWM_MAX_US:
        parser.error(f"pwm {PWM_MIN_US} ile {PWM_MAX_US} arasinda olmali")
    if args.sure <= 0:
        parser.error("--sure sifirdan buyuk olmali")
    return args


def set_all_motors_us(backend, pulse_us):
    """Yapilandirmadaki alti motor kanalina ayni PWM darbesini gonder."""
    for channel in MOTOR_CHANNELS.values():
        backend.set_us(channel, pulse_us)


def main():
    args = parse_args()
    thrusters = None

    try:
        print("PCA9685'e baglaniliyor...")
        backend = PCA9685Backend()
        thrusters = Thrusters(backend)  # CH0..CH5 hemen 1750 us notr olur.
        print(f"Tum motorlar notrde: {PWM_NEUTRAL_US} us")

        print("\nMotor-kanal sirasi:")
        for name, channel in MOTOR_CHANNELS.items():
            print(f"  CH{channel}: {name}")

        input(
            f"\nDIKKAT: 6 motor ayni anda {args.pwm} us degerinde "
            f"{args.sure:g} saniye calisacak. ROV su icinde ve sabitse "
            "ESC'lere guc verip bipleri bekleyin, sonra Enter'a basin..."
        )

        # ESC'lerin 1750 us notr sinyalini tanimasi icin iki saniye bekler.
        thrusters.arm()
        print("3 saniye sonra basliyor. Acil durdurma: Ctrl+C")
        for remaining in (3, 2, 1):
            print(remaining)
            time.sleep(1.0)

        print(f"6 motor calisiyor: {args.pwm} us")
        set_all_motors_us(backend, args.pwm)
        time.sleep(args.sure)
        print("Test tamamlandi.")

    except KeyboardInterrupt:
        print("\nCtrl+C algilandi.")
    finally:
        if thrusters is not None:
            thrusters.stop()
            print(f"Tum motorlar notre alindi: {PWM_NEUTRAL_US} us")


if __name__ == "__main__":
    main()
