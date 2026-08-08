"""
PCA9685 baglantisi — I2C bus NUMARASI ile (Blinka pin tahminine bagimli degil).

NEDEN BOYLE:
  Eski yol `busio.I2C(board.SCL, board.SDA)` idi. Bu, Blinka'nin karti tanimasina
  ve SECTIGI VARSAYILAN I2C busuna bagli. Jetson'da 40-pin header (pin 3/5)
  JetPack surumune gore /dev/i2c-7 ya da /dev/i2c-8 olarak gorunur; Blinka
  yanlis busu secince ya "board modulunde SCL yok" (pin goremiyor) ya da
  "cihaz yok" hatasi alinir.

  sensors/imu.py ve sensors/depth.py zaten smbus2 ile bus numarasi deneyerek
  calisiyordu — bu yuzden onlar baglaniyor, motor surucusu baglanmiyordu.
  Artik motor surucusu de ayni yolu kullaniyor.

Teshis icin: python3 i2c_tara.py
"""
import time

from config import I2C_BUS, FREQ_HZ, PCA9685_REF_CLOCK_HZ
from hal.i2c_lock import I2C_LOCK

PCA_ADDR = 0x40

# PCA9685 register haritasi (datasheet)
_MODE1     = 0x00
_MODE2     = 0x01
_PRESCALE  = 0xFE
_LED0_ON_L = 0x06

_MODE1_RESTART = 0x80
_MODE1_SLEEP   = 0x10
_MODE1_AI      = 0x20   # auto increment
_MODE2_OUTDRV  = 0x04   # totem pole cikis (ESC sinyali icin sart)


def bus_adaylari(tercih=None):
    """Denenecek I2C bus numaralari: once verilen/config, sonra Jetson'da sik gorulenler."""
    aday = [tercih, I2C_BUS, 8, 7, 1, 0]
    return list(dict.fromkeys(b for b in aday if b is not None))


def _bulunamadi_mesaji(address, hatalar):
    satirlar = [
        f"PCA9685 (0x{address:02X}) hicbir I2C busunda cevap vermedi.",
        "",
        "  Sirasiyla kontrol edin:",
        "   1. PCA9685 VCC pini Jetson 3.3V (pin 1) / 5V (pin 2)'ye bagli mi?",
        "      Kart lojik beslemesi olmadan I2C'de GORUNMEZ. ESC batarya gucu",
        "      (V+ klemensi) bunun yerine gecmez — o sadece motor rayini besler.",
        "   2. GND ortak mi? (Jetson pin 6 <-> PCA9685 GND)",
        "   3. SDA -> Jetson pin 3, SCL -> pin 5 (ters takilmis olabilir)",
        "   4. 'python3 i2c_tara.py' calistirip 0x40'in hangi busta gorundugune bakin.",
        "",
        "  Denenen buslar:",
    ]
    satirlar += [f"    {h}" for h in hatalar]
    return "\n".join(satirlar)


class Pca9685:
    """smbus2 uzerinden PCA9685 surucusu (board/busio/Blinka GEREKTIRMEZ)."""

    def __init__(self, bus_num=None, address=PCA_ADDR, freq_hz=FREQ_HZ,
                 ref_clock_hz=PCA9685_REF_CLOCK_HZ, sessiz=False):
        from smbus2 import SMBus
        self.addr = address
        self.freq_hz = freq_hz
        self.period_us = 1_000_000.0 / freq_hz
        self.bus = None
        self.bus_num = None

        hatalar = []
        for b in bus_adaylari(bus_num):
            try:
                bus = SMBus(b)
                bus.read_byte_data(address, _MODE1)   # cihaz gercekten orada mi
                self.bus, self.bus_num = bus, b
                break
            except Exception as e:
                hatalar.append(f"bus {b}: {e}")
        if self.bus is None:
            raise RuntimeError(_bulunamadi_mesaji(address, hatalar))

        self._baslat(ref_clock_hz)
        if not sessiz:
            print(f"[PCA9685] I2C bus {self.bus_num} (0x{address:02X}) baglandi — "
                  f"prescale={self.prescale}, gercek frekans "
                  f"{self.gercek_freq_hz:.2f} Hz")

    def _baslat(self, ref_clock_hz):
        """Cipi uyandirir, totem-pole cikisa alir ve PWM frekansini yazar."""
        self.bus.write_byte_data(self.addr, _MODE1, _MODE1_AI)
        time.sleep(0.005)
        self.bus.write_byte_data(self.addr, _MODE2, _MODE2_OUTDRV)

        # prescale = round(osilator / (4096 * frekans)) - 1
        # ref_clock_hz config'te OLCULEREK kalibre edildi (bkz. PCA9685_REF_CLOCK_HZ)
        prescale = int(round(ref_clock_hz / (4096.0 * self.freq_hz))) - 1
        prescale = max(3, min(255, prescale))

        eski = self.bus.read_byte_data(self.addr, _MODE1)
        # prescale SADECE sleep modunda yazilabilir
        self.bus.write_byte_data(self.addr, _MODE1,
                                 (eski & ~_MODE1_RESTART) | _MODE1_SLEEP)
        self.bus.write_byte_data(self.addr, _PRESCALE, prescale)
        self.bus.write_byte_data(self.addr, _MODE1, eski)
        time.sleep(0.005)
        self.bus.write_byte_data(self.addr, _MODE1, eski | _MODE1_RESTART)

        self.prescale = prescale
        self.gercek_freq_hz = ref_clock_hz / (4096.0 * (prescale + 1))

    def set_us(self, channel, us):
        """Kanala mikrosaniye cinsinden darbe genisligi yazar.

        I2C_LOCK: motor yazimi kontrol dongusunden, sensor okumalari ayri
        thread'lerden geliyor. Ayni veri yolunda ust uste binmemeleri icin
        (SORUN 2 - thread mimarisi) tek ortak kilit kullaniliyor.
        """
        sayac = int(round(us / self.period_us * 4096.0))
        sayac = max(0, min(4095, sayac))
        reg = _LED0_ON_L + 4 * channel
        try:
            with I2C_LOCK:
                self.bus.write_i2c_block_data(self.addr, reg,
                                              [0x00, 0x00, sayac & 0xFF, sayac >> 8])
        except OSError:
            # Motor anlik yuklenmesinde olusan elektriksel gurultu I2C'yi saniyede 1-2 kez dusurebilir (Errno 121 / 110).
            # 50Hz'de gonderdigimiz icin 1 paketin kaybolmasi sorun degil. Gormezden gel.
            pass


class _BlinkaPca9685:
    """Yedek yol: smbus2 yoksa eski board/busio yontemi."""

    def __init__(self, address=PCA_ADDR, freq_hz=FREQ_HZ, sessiz=False):
        import board, busio
        from adafruit_pca9685 import PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.dev = PCA9685(i2c, address=address,
                           reference_clock_speed=PCA9685_REF_CLOCK_HZ)
        self.dev.frequency = freq_hz
        self.period_us = 1_000_000.0 / freq_hz
        self.bus_num = None
        if not sessiz:
            print("[PCA9685] Blinka (board/busio) uzerinden baglandi — "
                  "varsayilan I2C busu.")

    def set_us(self, channel, us):
        self.dev.channels[channel].duty_cycle = int(us / self.period_us * 0xFFFF)


def pca9685_ac(bus_num=None, address=PCA_ADDR, freq_hz=FREQ_HZ, sessiz=False):
    """PCA9685'e baglanir ve set_us(kanal, us) arayuzu olan nesne dondurur.

    Once smbus2 + bus numarasi denenir (Jetson'da guvenilir yol), smbus2 kurulu
    degilse Blinka'ya duser.
    """
    try:
        import smbus2  # noqa: F401
    except ImportError:
        return _BlinkaPca9685(address, freq_hz, sessiz)
    return Pca9685(bus_num, address, freq_hz, sessiz=sessiz)
