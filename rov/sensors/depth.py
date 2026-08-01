"""
MS5837-30BA basinc/derinlik sensoru (I2C).

==============================================================================
BURADA COZULEN IKI SORUN
==============================================================================
SORUN 2 (dongu 7.3 Hz'e dusmustu) — SEBEBIN YARISI BU DOSYAYDI
  Eski kod her okumada OSR=8192 istiyordu. O hassasiyette sensorun
  "dusunme" suresi ~17 ms, kod ise emniyetli olsun diye 20 ms bekliyordu.
  Iki donusum (basinc + sicaklik) = 40 ms BLOKLAMA. Ve bu okuma dongu
  basina 3 ayri yerden yapiliyordu (stabilizer + gorev + logger) = 120 ms.

  COZUM 1: OSR config'ten ayarlanabilir, varsayilan 1024.
           Bekleme 20 ms -> 3 ms. Cozunurluk ~0.2 mbar = 2 MILIMETRE derinlik.
           Bizim toleransimiz 5 SANTIMETRE; 2 mm fazlasiyla yeter.
  COZUM 2: Okuma artik sensors/state.py icindeki ayri bir thread'de yapiliyor;
           kontrol dongusu son degeri hafizadan aliyor, HIC beklemiyor.
  COZUM 3: I2C_LOCK sadece veri yolu islemlerini sariyor, BEKLEMEYI SARMIYOR
           (yoksa IMU thread'i de bloke olurdu).

SORUN 4a (yukari cikisi goremiyordu)
  Eski kod:  return max(0.0, ...)
  Arac referans yuzeyin ustune ciktiginda derinlik negatif olur; bu satir
  onu 0 yapiyordu. Yani PID "0 metredeyim" deyip donuyor, NE KADAR yukari
  firladigini goremiyordu. Yuzeye cikmak bizde ELEME sebebi — bu korluk
  kabul edilemez.
  COZUM: read_depth_m() artik ISARETLI (negatif olabilen) deger dondurur.
         Ekranda gostermek icin read_depth_m_display() kirpilmis deger verir.

NOT (bilincli olarak DEGISTIRILMEDI):
  Basinc hesabi datasheet'in 1. derece kompanzasyonunu kullaniyor.
  2. derece (sicaklik) duzeltmesi havuz sicakliginda (~25 C) ihmal
  edilebilir seviyede ve suya girmeden dogrulayamayiz. Calisan ama
  dogrulanamayacak bir seyi degistirmemek icin oldugu gibi birakildi.
"""
import time

from config import I2C_BUS, DEPTH_ADDR, FLUID_DENSITY
from hal.i2c_lock import I2C_LOCK

try:
    from config import SURFACE_PRESSURE_MBAR
except ImportError:
    SURFACE_PRESSURE_MBAR = None

try:
    from config import DEPTH_OSR
except ImportError:
    DEPTH_OSR = 1024


# OSR -> (D1 komutu, D2 komutu, gereken bekleme sn)
# Datasheet donusum sureleri + emniyet payi.
_OSR_TABLE = {
    256:  (0x40, 0x50, 0.0010),
    512:  (0x42, 0x52, 0.0015),
    1024: (0x44, 0x54, 0.0030),   # <- varsayilan: ~2 mm cozunurluk, 3 ms
    2048: (0x46, 0x56, 0.0050),
    4096: (0x48, 0x58, 0.0095),
    8192: (0x4A, 0x5A, 0.0200),   # <- eskiden bu kullaniliyordu (40 ms/okuma)
}


class Ms5837:
    """Gercek donanim surucusu (datasheet 1. derece kompanzasyon)."""
    CMD_RESET = 0x1E
    CMD_PROM = 0xA0
    CMD_READ = 0x00

    def __init__(self, bus_num=None, osr=None):
        """I2C baglantisini acar, sensoru resetler ve fabrika kalibrasyon
        katsayilarini okur. osr verilmezse config.DEPTH_OSR kullanilir."""
        from smbus2 import SMBus

        osr = DEPTH_OSR if osr is None else osr
        if osr not in _OSR_TABLE:
            raise ValueError(f"DEPTH_OSR gecersiz: {osr}. Secenekler: {sorted(_OSR_TABLE)}")
        self.CMD_CONV_D1, self.CMD_CONV_D2, self.conv_wait_s = _OSR_TABLE[osr]
        self.osr = osr

        buses_to_try = [bus_num] if bus_num is not None else [I2C_BUS, 1, 0, 8, 7]
        last_err = None
        self.bus = None
        for b in buses_to_try:
            try:
                bus = SMBus(b)
                with I2C_LOCK:
                    bus.write_byte(DEPTH_ADDR, self.CMD_RESET)
                time.sleep(0.05)
                self.bus = bus
                break
            except Exception as e:
                last_err = e
        if self.bus is None:
            raise RuntimeError(
                f"MS5837 sensorune baglanilamadi (Denenen Buslar: {buses_to_try}): {last_err}")

        self.C = []
        for i in range(7):
            with I2C_LOCK:
                d = self.bus.read_i2c_block_data(DEPTH_ADDR, self.CMD_PROM + 2 * i, 2)
            self.C.append(d[0] << 8 | d[1])

        # Telemetri icin son okunan ham degerler (web arayuzu okuyor)
        self.pressure_mbar = 1013.25
        self.temp_c = 20.0

        # Kalibrasyonla olculmus yuzey referansi varsa fallback olarak kullan;
        # gorev basinda zero_at_surface() cagrilirsa taze deger ile ezilir.
        self.surface_pressure_mbar = SURFACE_PRESSURE_MBAR
        print(f"[DEPTH] MS5837 hazir — OSR={osr}, donusum beklemesi "
              f"{self.conv_wait_s*1000:.1f} ms x2")
        if SURFACE_PRESSURE_MBAR is not None:
            print(f"[DEPTH] Yuzey referansi config'den yuklendi: "
                  f"{SURFACE_PRESSURE_MBAR} mbar")

    # ------------------------------------------------------------ ham okuma
    def _convert(self, cmd):
        """ADC donusumunu tetikler ve 24 bitlik ham sonucu dondurur.

        DIKKAT: I2C_LOCK sadece iki veri yolu islemini sarar; ARADAKI
        BEKLEME KILIT DISINDADIR. Aksi halde bu bekleme suresince IMU
        thread'i de duraklar ve SORUN 2'yi geri getiririz.
        """
        with I2C_LOCK:
            self.bus.write_byte(DEPTH_ADDR, cmd)
        time.sleep(self.conv_wait_s)          # <-- kilit DISINDA
        with I2C_LOCK:
            d = self.bus.read_i2c_block_data(DEPTH_ADDR, self.CMD_READ, 3)
        return d[0] << 16 | d[1] << 8 | d[2]

    def read_pressure_mbar(self):
        """Ham basinc (D1) ve sicaklik (D2) okumalarini PROM katsayilariyla
        birlestirip kalibre edilmis basinci mbar cinsinden dondurur.
        Yan etki: self.pressure_mbar ve self.temp_c guncellenir."""
        D1 = self._convert(self.CMD_CONV_D1)
        D2 = self._convert(self.CMD_CONV_D2)
        C = self.C
        dT = D2 - C[5] * 256
        SENS = C[1] * 32768 + (C[3] * dT) / 256
        OFF = C[2] * 65536 + (C[4] * dT) / 128
        P = (D1 * SENS / 2097152 - OFF) / 8192
        self.pressure_mbar = P / 10.0
        self.temp_c = (2000 + dT * C[6] / 8388608) / 100.0
        return self.pressure_mbar

    # -------------------------------------------------------------- derinlik
    def zero_at_surface(self):
        """Arac su yuzeyindeyken cagir: derinlik referansini sifirlar.

        Tek okuma yerine 8 okumanin uc degerleri atilmis ortalamasi alinir —
        tek okuma dalga/gurultu yuzunden birkac cm kayabilir ve o kayma TUM
        gorev boyunca tasinir."""
        vals = sorted(self.read_pressure_mbar() for _ in range(8))
        kirpik = vals[2:-2]
        self.surface_pressure_mbar = sum(kirpik) / len(kirpik)
        print(f"[DEPTH] Yuzey referansi olculdu: {self.surface_pressure_mbar:.2f} mbar")
        return self.surface_pressure_mbar

    def read_depth_m(self):
        """Derinlik (metre), ISARETLI.

        SORUN 4a: negatif deger KIRPILMAZ. Arac referansin ustune cikarsa
        derinlik negatif doner ve PID ne kadar yukari firladigini gorur.
        """
        if self.surface_pressure_mbar is None:
            self.zero_at_surface()
        p = self.read_pressure_mbar()
        return (p - self.surface_pressure_mbar) * 100.0 / (FLUID_DENSITY * 9.81)

    def read_depth_m_display(self):
        """Ekranda/telemetride gostermek icin kirpilmis derinlik (>= 0)."""
        return max(0.0, self.read_depth_m())


class MockDepth:
    """Simulasyon: gercek sensor yerine RovSimulator'in derinlik degerini dondurur."""

    def __init__(self, sim=None):
        self.sim = sim
        self.pressure_mbar = 1013.25
        self.temp_c = 20.0
        self.surface_pressure_mbar = 1013.25
        self.osr = 0
        self.conv_wait_s = 0.0

    def zero_at_surface(self):
        """Simulasyonda yuzey referansi gerekmez, no-op."""
        return self.surface_pressure_mbar

    def read_pressure_mbar(self):
        return self.pressure_mbar

    def read_depth_m(self):
<<<<<<< Updated upstream
        """Simulatorun o anki derinligini dondurur (sim yoksa 0)."""
        return 0.0 if self.sim is None else self.sim.depth_m

    def read_depth_m_display(self):
        return max(0.0, self.read_depth_m())
=======
        """Simulatorun o anki derinligini dondurur."""
        return self.sim.depth_m


class MockDepthStatic:
    """Donanim bagli degil ve simulatorsuz calismak gerektiginde kullanilir.
    Her zaman 0.0 m dondurur. PID test gibi basit senaryolarda yeterli."""
    def __init__(self):
        self._surface = 0.0

    def zero_at_surface(self):
        pass  # no-op

    def read_depth_m(self):
        return 0.0

    def read_pressure_mbar(self):
        return 1013.25
>>>>>>> Stashed changes
