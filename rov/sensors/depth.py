"""MS5837-30BA basinc/derinlik sensoru (I2C)."""
import time
from config import I2C_BUS, DEPTH_ADDR, FLUID_DENSITY


class Ms5837:
    """Gercek donanim surucusu (datasheet'e gore 1. derece kompanzasyon)."""
    CMD_RESET = 0x1E
    CMD_PROM = 0xA0
    CMD_CONV_D1 = 0x4A  # basinc, OSR=8192
    CMD_CONV_D2 = 0x5A  # sicaklik, OSR=8192
    CMD_READ = 0x00

    def __init__(self):
        """I2C baglantisini acar, sensoru resetler ve fabrika kalibrasyon
        katsayilarini (PROM, C[0..6]) okuyup saklar - basinc hesabinda
        (read_pressure_mbar) bu katsayilar kullanilir."""
        from smbus2 import SMBus
        self.bus = SMBus(I2C_BUS)
        self.bus.write_byte(DEPTH_ADDR, self.CMD_RESET)
        time.sleep(0.05)
        self.C = []
        for i in range(7):
            d = self.bus.read_i2c_block_data(DEPTH_ADDR, self.CMD_PROM + 2 * i, 2)
            self.C.append(d[0] << 8 | d[1])
        self.surface_pressure_mbar = None

    def _convert(self, cmd):
        """Verilen ADC donusum komutunu (D1=basinc ya da D2=sicaklik)
        tetikler ve 24 bitlik ham sonucu dondurur."""
        self.bus.write_byte(DEPTH_ADDR, cmd)
        time.sleep(0.02)
        d = self.bus.read_i2c_block_data(DEPTH_ADDR, self.CMD_READ, 3)
        return d[0] << 16 | d[1] << 8 | d[2]

    def read_pressure_mbar(self):
        """Ham basinc (D1) ve sicaklik (D2) okumalarini PROM katsayilariyla
        (datasheet 1. derece kompanzasyon formulu) birlestirip kalibre
        edilmis basinci mbar cinsinden dondurur."""
        D1 = self._convert(self.CMD_CONV_D1)
        D2 = self._convert(self.CMD_CONV_D2)
        C = self.C
        dT = D2 - C[5] * 256
        SENS = C[1] * 32768 + (C[3] * dT) / 256
        OFF = C[2] * 65536 + (C[4] * dT) / 128
        P = (D1 * SENS / 2097152 - OFF) / 8192
        return P / 10.0  # mbar

    def zero_at_surface(self):
        """Arac su yuzeyindeyken cagir: derinlik referansini sifirlar."""
        self.surface_pressure_mbar = self.read_pressure_mbar()

    def read_depth_m(self):
        """Yuzey referansina gore derinligi metre cinsinden dondurur:
        basinc farki (mbar->Pa) / (yogunluk * g) hidrostatik formulu.
        Henuz sifirlanmadiysa once zero_at_surface() cagrilir."""
        if self.surface_pressure_mbar is None:
            self.zero_at_surface()
        p = self.read_pressure_mbar()
        return max(0.0, (p - self.surface_pressure_mbar) * 100.0 / (FLUID_DENSITY * 9.81))


class MockDepth:
    """Simulasyon: gercek sensor yerine RovSimulator'in derinlik degerini dondurur."""
    def __init__(self, sim):
        self.sim = sim

    def zero_at_surface(self):
        """Simulasyonda yuzey referansi gerekmez, no-op."""
        pass

    def read_depth_m(self):
        """Simulatorun o anki derinligini dondurur."""
        return self.sim.depth_m
