"""
ESC/motor HAL - Degz Mitras M1 + standart PWM ESC (1100-1900us, 1500 notr).

Backend degistirilebilir:
  - PCA9685Backend : I2C 16 kanal PWM karti (onerilen, Jetson Xavier NX ile)
  - MockBackend    : donanimsiz test/simulasyon

Guvenlik: arm edilmeden komut gonderilmez; stop() her kosulda notre ceker.
"""
import time
from config import (MOTOR_CHANNELS, PWM_NEUTRAL_US, PWM_MIN_US, PWM_MAX_US,
                    PWM_DEADBAND_US, SLEW_RATE, PCA9685_REF_CLOCK_HZ,
                    ESC_ABS_MIN_US, ESC_ABS_MAX_US)

# config.py'de yanlis bir PWM_RANGE_US yazilirsa (orn. notrden sapma yerine
# toplam genislik girilirse) PWM_MIN/MAX ESC'nin anlamadigi degerlere kayar.
# Import aninda yakala - suya girdikten sonra degil.
if not (ESC_ABS_MIN_US <= PWM_MIN_US < PWM_NEUTRAL_US < PWM_MAX_US <= ESC_ABS_MAX_US):
    raise ValueError(
        f"config.py PWM ayarlari ESC araliginin disinda!\n"
        f"  hesaplanan: {PWM_MIN_US}..{PWM_NEUTRAL_US}..{PWM_MAX_US} us\n"
        f"  ESC siniri: {ESC_ABS_MIN_US}..{ESC_ABS_MAX_US} us\n"
        f"  PWM_RANGE_US notrden TEK YONDEKI sapmadir, toplam genislik degil.\n"
        f"  Notr {PWM_NEUTRAL_US} icin en fazla "
        f"{min(PWM_NEUTRAL_US - ESC_ABS_MIN_US, ESC_ABS_MAX_US - PWM_NEUTRAL_US)} olabilir."
    )


# ------------------------------------------------------------ backendler
class MockBackend:
    """Donanim yokken PWM degerlerini sadece saklar."""
    def __init__(self):
        self.last_us = {}

    def set_us(self, channel, us):
        """Gercek donanima yazmak yerine son gonderilen PWM degerini (us)
        kanal bazinda saklar - simulator.py bu degerleri okuyup fizigi ilerletir."""
        self.last_us[channel] = us


class PCA9685Backend:
    """Adafruit PCA9685 (I2C). Kurulum (Jetson):
        pip3 install adafruit-circuitpython-pca9685 adafruit-blinka
    I2C baglantisi: PCA9685 -> Jetson pin 3 (SDA), pin 5 (SCL), 3.3V, GND
    """
    def __init__(self, freq_hz=50, address=0x40):
        """I2C uzerinden PCA9685 karti ile baglanti kurar ve PWM frekansini
        (standart ESC icin 50Hz) ayarlar."""
        import board, busio
        from adafruit_pca9685 import PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.dev = PCA9685(i2c, address=address, reference_clock_speed=PCA9685_REF_CLOCK_HZ)
        self.dev.frequency = freq_hz
        self.period_us = 1_000_000 / freq_hz

    def set_us(self, channel, us):
        """Mikrosaniye cinsinden PWM darbe genisligini PCA9685'in 16-bit
        duty-cycle degerine cevirip ilgili kanala yazar."""
        duty = int(us / self.period_us * 0xFFFF)
        self.dev.channels[channel].duty_cycle = duty


# ------------------------------------------------------------ ana sinif
class Thrusters:
    def __init__(self, backend):
        """backend: MockBackend ya da PCA9685Backend. Baslangicta guvenlik
        icin stop() cagrilarak tum motorlar notre cekilir."""
        self.backend = backend
        self.armed = False
        self._current = {name: 0.0 for name in MOTOR_CHANNELS}  # slew icin
        self._last_t = time.monotonic()
        self.stop()

    # ---- guvenlik ----
    def arm(self):
        """ESC'lere notr gonderip hazirlar. ESC bip sesi bekle."""
        for name in MOTOR_CHANNELS:
            self._write_us(name, PWM_NEUTRAL_US)
        time.sleep(2.0)  # ESC'lerin notru tanimasi icin
        self.armed = True

    def stop(self):
        """ACIL: tum motorlar notr. Her durumda cagrilabilir."""
        for name in MOTOR_CHANNELS:
            self._write_us(name, PWM_NEUTRAL_US)
        self._current = {name: 0.0 for name in MOTOR_CHANNELS}
        self.armed = False

    # ---- komut ----
    def command(self, motor_dict):
        """{motor_adi: -1..+1} komutlarini slew-limitli PWM'e cevirir."""
        if not self.armed:
            return
        now = time.monotonic()
        dt = min(0.1, now - self._last_t)
        self._last_t = now
        max_step = SLEW_RATE * dt

        for name, target in motor_dict.items():
            target = max(-1.0, min(1.0, target))
            cur = self._current[name]
            # ani degisimi sinirla (ESC ve guc hattini korur)
            step = max(-max_step, min(max_step, target - cur))
            cur += step
            self._current[name] = cur
            self._write_us(name, self._to_us(cur))

    # ---- yardimcilar ----
    @staticmethod
    def _to_us(value):
        """-1..+1 -> PWM us. config.py'deki PWM_NEUTRAL_US etrafında simetrik ölçekler."""
        us = PWM_NEUTRAL_US + value * (PWM_MAX_US - PWM_NEUTRAL_US)
        if abs(us - PWM_NEUTRAL_US) < PWM_DEADBAND_US:
            us = PWM_NEUTRAL_US
        us = max(PWM_MIN_US, min(PWM_MAX_US, us))
        # son emniyet: hicbir kosulda ESC'nin fiziksel sinirlarini asma
        return int(max(ESC_ABS_MIN_US, min(ESC_ABS_MAX_US, us)))

    def _write_us(self, name, us):
        self.backend.set_us(MOTOR_CHANNELS[name], us)
