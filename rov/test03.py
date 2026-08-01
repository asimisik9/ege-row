"""
EGE ROV — TAM GUC DALIS TESTI (bagimsiz script).

1.5 metreye dalar, toplam 60 saniye calisir, sure dolunca motorlari notre ceker.
Dalis fazinda dikey motorlara PWM tavanina kadar (tam guc, ILERI yon) sinyal verir;
hedefe ulasinca basit P kontrolu ile derinligi tutar.

BAGIMLILIK: SADECE config.py (+ donanim kutuphaneleri: smbus2, adafruit_pca9685).
Projedeki baska hicbir dosyayi import ETMEZ.

Kullanim (Jetson): python3 dalis_1_5m.py
Ctrl+C her an guvenli cikis yapar (motorlar notre cekilir).
"""
import time

from config import (
    LOOP_HZ,
    MOTOR_CHANNELS, MOTOR_DIRECTION,
    PWM_NEUTRAL_US, PWM_MIN_US, PWM_MAX_US, PWM_RANGE_US,
    PCA9685_REF_CLOCK_HZ,
    I2C_BUS, DEPTH_ADDR, FLUID_DENSITY, DIVE_MAX
)

VERTICAL = ("V_FL", "V_FR", "V_RL", "V_RR")   # dikey motorlar (config yerlesimi)


# ------------------------------------------------------------ PCA9685 (motor PWM)
class Pwm:
    def __init__(self):
        # Blinka (board/busio) yanlis busu secebiliyor; basinc sensoru smbus2
        # ile bus 8'e baglanirken motorlar baglanamiyordu. Artik ayni yol:
        # hal/i2c.py bus NUMARASI ile baglanir (config.I2C_BUS -> 8).
        from hal.i2c import pca9685_ac
        self.dev = pca9685_ac(freq_hz=50)

    def set_us(self, channel, us):
        us = max(PWM_MIN_US, min(PWM_MAX_US, us))
        self.dev.set_us(channel, us)

    def motor(self, name, cmd):
        """cmd: -1..+1 (+1 = o motorun ILERI yonu, MOTOR_DIRECTION uygulanir)."""
        cmd = max(-1.0, min(1.0, cmd)) * MOTOR_DIRECTION[name]
        self.set_us(MOTOR_CHANNELS[name], PWM_NEUTRAL_US + cmd * PWM_RANGE_US)

    def all_neutral(self):
        for ch in MOTOR_CHANNELS.values():
            self.set_us(ch, PWM_NEUTRAL_US)


# ------------------------------------------------------------ MS5837 (derinlik)
class Depth:
    CMD_RESET, CMD_PROM, CMD_D1, CMD_D2, CMD_READ = 0x1E, 0xA0, 0x4A, 0x5A, 0x00

    def __init__(self):
        from smbus2 import SMBus
        self.bus = SMBus(I2C_BUS)
        self.bus.write_byte(DEPTH_ADDR, self.CMD_RESET)
        time.sleep(0.05)
        self.C = []
        for i in range(7):
            d = self.bus.read_i2c_block_data(DEPTH_ADDR, self.CMD_PROM + 2 * i, 2)
            self.C.append(d[0] << 8 | d[1])
        self.surface_mbar = None

    def _convert(self, cmd):
        self.bus.write_byte(DEPTH_ADDR, cmd)
        time.sleep(0.02)
        d = self.bus.read_i2c_block_data(DEPTH_ADDR, self.CMD_READ, 3)
        return d[0] << 16 | d[1] << 8 | d[2]

    def pressure_mbar(self):
        D1, D2 = self._convert(self.CMD_D1), self._convert(self.CMD_D2)
        C = self.C
        dT = D2 - C[5] * 256
        SENS = C[1] * 32768 + (C[3] * dT) / 256
        OFF = C[2] * 65536 + (C[4] * dT) / 128
        return (D1 * SENS / 2097152 - OFF) / 8192 / 10.0

    def zero_at_surface(self):
        self.surface_mbar = self.pressure_mbar()

    def read_m(self):
        p = self.pressure_mbar()
        return max(0.0, (p - self.surface_mbar) * 100.0 / (FLUID_DENSITY * 9.81))


# ------------------------------------------------------------ ana akis
def main():
    D = DIVE_MAX
    pwm = Pwm()
    depth = Depth()
    print(f"[OK] PCA9685 + MS5837 hazir. Hedef: {D['target_depth_m']} m, "
          f"sure: {D['duration_s']:.0f} sn, guc: %{D['dive_power']*100:.0f}")

    try:
        # 1) yuzey referansi (arac su yuzeyindeyken)
        depth.zero_at_surface()
        print("Yuzey referansi alindi.")

        # 2) ESC arm: notr gonder
        pwm.all_neutral()
        time.sleep(2.0)

        # 3) geri sayim
        for i in range(int(D["start_delay_s"]), 0, -1):
            print(f"  baslamaya {i}...")
            time.sleep(1.0)

        # 4) dalis dongusu
        t0 = time.monotonic()
        dt = 1.0 / LOOP_HZ
        reached = False
        last_print = 0.0

        while True:
            t = time.monotonic() - t0
            if t >= D["duration_s"]:
                print("SURE DOLDU (60 sn) — motorlar notre cekiliyor.")
                break

            d = depth.read_m()
            err = D["target_depth_m"] - d          # + : daha derine inmeli

            if not reached and d >= D["target_depth_m"]:
                reached = True
                print(f"HEDEFE ULASILDI: {d:.2f} m ({t:.1f} sn) — derinlik tutuluyor.")

            if not reached:
                cmd = D["dive_power"]              # TAM GUC ileri (PWM tavani)
            else:
                cmd = max(-1.0, min(1.0, D["hold_kp"] * err))  # P ile tut

            for name in VERTICAL:
                pwm.motor(name, cmd)
            # yatay motorlar notrde kalir (ileri gitme yok)

            if t - last_print >= 0.5:
                last_print = t
                us = PWM_NEUTRAL_US + cmd * PWM_RANGE_US
                print(f"  t={t:5.1f}s derinlik={d:5.2f}m hata={err:+5.2f}m "
                      f"guc={cmd:+.2f} (~{us:.0f}us)")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nCtrl+C — iptal.")
    finally:
        pwm.all_neutral()
        print("Motorlar notrde. Test bitti.")


if __name__ == "__main__":
    main()