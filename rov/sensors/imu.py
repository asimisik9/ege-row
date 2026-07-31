"""
MPU-9250 IMU surucusu + yonelim kestirimi (Jetson Orin Nano, I2C).

Cikti:
  heading_deg : 0..360 pusula yonu (jiroskop + manyetometre tamamlayici filtre)
  roll_deg, pitch_deg : ivmeolcer + jiroskop tamamlayici filtre
  yaw_rate_dps : z ekseni acisal hiz (daire gorevi icin)

Kalibrasyon (cihazda yapilacak, config'e yazilacak):
  GYRO_BIAS  : arac sabitken 5 sn ortalama jiroskop degeri
  MAG_OFFSET/MAG_SCALE : araci her eksende cevirerek min/max kaydi (hard/soft iron)
"""
import math
import time
from config import (I2C_BUS, IMU_ADDR, MAG_ADDR, HEADING_FILTER_ALPHA,
                    MAG_OFFSET, MAG_SCALE, GYRO_BIAS)

try:
    from config import ACCEL_BIAS
except ImportError:
    ACCEL_BIAS = (0.0, 0.0, 0.0)


class Mpu9250:
    """Gercek donanim. Kurulum: pip3 install smbus2"""

    # register adresleri
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    INT_PIN_CFG = 0x37
    MAG_CNTL1 = 0x0A
    MAG_HXL = 0x03
    MAG_ST2 = 0x09

    def __init__(self, bus_num=None):
        """MPU-9250'yi uyandirir. Sensörün takılı olduğu I2C busunu (8, 7, 1, 0) otomatik bulur.
        Dahili AK8963 manyetometreye I2C bypass acar ve 16-bit/100Hz moduna alir."""
        from smbus2 import SMBus
        buses_to_try = [bus_num] if bus_num is not None else [I2C_BUS, 8, 7, 1, 0]
        last_err = None
        self.bus = None
        for b in buses_to_try:
            try:
                bus = SMBus(b)
                bus.write_byte_data(IMU_ADDR, self.PWR_MGMT_1, 0x00)   # uyandir
                time.sleep(0.05)
                self.bus = bus
                print(f"[IMU] MPU-9250 I2C Bus {b} (0x68) üzerinde bağlandı.")
                break
            except Exception as e:
                last_err = e
        if self.bus is None:
            raise RuntimeError(f"MPU-9250 IMU sensörüne bağlanılamadı (Denenen Buslar: {buses_to_try}): {last_err}")

        self.bus.write_byte_data(IMU_ADDR, self.INT_PIN_CFG, 0x02)  # mag'a bypass
        time.sleep(0.05)
        self.bus.write_byte_data(MAG_ADDR, self.MAG_CNTL1, 0x16)    # 16bit, 100Hz surekli
        time.sleep(0.05)

    def _read_i16(self, addr, reg, little=False):
        """Verilen I2C adresinden 2 byte okuyup isaretli 16-bit tam sayiya
        cevirir (buyuk-endian varsayilan, little=True ile kucuk-endian)."""
        hi = self.bus.read_byte_data(addr, reg)
        lo = self.bus.read_byte_data(addr, reg + 1)
        raw = (lo << 8 | hi) if little else (hi << 8 | lo)
        return raw - 65536 if raw > 32767 else raw

    def read_accel_g_raw(self):
        """Uc eksen ivmeyi KALIBRASYONSUZ g biriminde (x, y, z) tuple olarak dondurur."""
        d = self.bus.read_i2c_block_data(IMU_ADDR, self.ACCEL_XOUT_H, 6)
        vals = []
        for i in range(3):
            raw = (d[2*i] << 8) | d[2*i + 1]
            if raw > 32767:
                raw -= 65536
            vals.append(raw / 16384.0)
        return tuple(vals)

    def read_accel_g(self):
        vals = self.read_accel_g_raw()
        return tuple(vals[i] - ACCEL_BIAS[i] for i in range(3))

    def read_gyro_dps_raw(self):
        """Uc eksen acisal hizi KALIBRASYONSUZ (ham) derece/sn dondurur."""
        d = self.bus.read_i2c_block_data(IMU_ADDR, self.GYRO_XOUT_H, 6)
        vals = []
        for i in range(3):
            raw = (d[2*i] << 8) | d[2*i + 1]
            if raw > 32767:
                raw -= 65536
            vals.append(raw / 131.0)
        return tuple(vals)

    def read_gyro_dps(self):
        """Uc eksen acisal hizi derece/sn cinsinden dondurur; kalibrasyonla
        bulunan GYRO_BIAS her eksenden cikarilarak sapma giderilir."""
        g = self.read_gyro_dps_raw()
        return tuple(g[i] - GYRO_BIAS[i] for i in range(3))

    def read_mag_ut_raw(self):
        """Uc eksen manyetik alani KALIBRASYONSUZ (ham) mikroTesla dondurur.
        Kalibrasyon scripti bunu kullanir (ofset/olcek ust uste binmesin diye)."""
        # AK8963 little-endian; ST2 okunmadan yeni veri gelmez
        m = tuple(self._read_i16(MAG_ADDR, self.MAG_HXL + 2 * i, little=True) * 0.15
                  for i in range(3))
        self.bus.read_byte_data(MAG_ADDR, self.MAG_ST2)
        return m

    def read_mag_ut(self):
        """Uc eksen manyetik alani mikroTesla cinsinden dondurur; hard/soft
        iron kalibrasyonu (MAG_OFFSET/MAG_SCALE) uygulanmis olarak."""
        m = self.read_mag_ut_raw()
        return tuple((m[i] - MAG_OFFSET[i]) * MAG_SCALE[i] for i in range(3))


class MockImu:
    """Simulasyon: simulator.py durumu buraya yazar."""
    def __init__(self, sim):
        self.sim = sim

    def read_accel_g(self):
        """Duz durus varsayilir: sadece z ekseninde 1g (yercekimi)."""
        return (0.0, 0.0, 1.0)

    def read_gyro_dps(self):
        """Sadece z ekseninde (yaw) simulatorden gelen donus hizini dondurur."""
        return (0.0, 0.0, self.sim.yaw_rate_dps)

    def read_mag_ut(self):
        """Simulatorun heading degerinden sahte bir manyetometre okumasi uretir."""
        h = math.radians(self.sim.heading_deg)
        return (math.cos(h) * 30.0, -math.sin(h) * 30.0, 0.0)


class Orientation:
    """Tamamlayici filtre ile heading/roll/pitch kestirimi."""

    def __init__(self, driver):
        self.drv = driver
        self.heading = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self._prev_t = None

    @staticmethod
    def _mag_heading(mag, roll_r, pitch_r):
        """Egim-telafili (tilt-compensated) pusula hesabi: roll/pitch acilari
        kullanilarak manyetometre okumasi yatay duzleme izdusurulur, sonra
        atan2 ile 0-360 derece heading elde edilir."""
        mx, my, mz = mag
        # egim telafisi
        xh = mx * math.cos(pitch_r) + mz * math.sin(pitch_r)
        yh = (mx * math.sin(roll_r) * math.sin(pitch_r) + my * math.cos(roll_r)
              - mz * math.sin(roll_r) * math.cos(pitch_r))
        return (math.degrees(math.atan2(-yh, xh)) + 360.0) % 360.0

    def update(self):
        """Sensorleri bir kez okuyup heading/roll/pitch/yaw_rate degerlerini
        tamamlayici filtre (complementary filter) ile gunceller ve guncel
        heading degerini dondurur. LOOP_HZ frekansinda cagrilmasi beklenir."""
        now = time.monotonic()
        dt = 0.0 if self._prev_t is None else max(1e-4, now - self._prev_t)
        self._prev_t = now

        ax, ay, az = self.drv.read_accel_g()
        gx, gy, gz = self.drv.read_gyro_dps()
        mag = self.drv.read_mag_ut()
        self.yaw_rate = gz

        # roll/pitch: ivmeolcer referans + jiroskop kisa vade
        acc_roll = math.degrees(math.atan2(ay, az))
        acc_pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        a = 0.98
        self.roll = a * (self.roll + gx * dt) + (1 - a) * acc_roll
        self.pitch = a * (self.pitch + gy * dt) + (1 - a) * acc_pitch

        # heading: jiroskop entegrasyonu + manyetometre duzeltmesi
        mag_h = self._mag_heading(mag, math.radians(self.roll), math.radians(self.pitch))
        if self.heading is None:
            self.heading = mag_h
        else:
            gyro_h = (self.heading + gz * dt) % 360.0
            # acisal sarmali dogru harmanla
            diff = (mag_h - gyro_h + 180.0) % 360.0 - 180.0
            self.heading = (gyro_h + (1 - HEADING_FILTER_ALPHA) * diff) % 360.0
        return self.heading
