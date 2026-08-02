"""
MPU-9250 IMU surucusu + yonelim kestirimi (Jetson Orin Nano, I2C).

Cikti:
  heading   : 0..360 pusula yonu (jiroskop + manyetometre tamamlayici filtre)
  roll, pitch : ivmeolcer + jiroskop tamamlayici filtre (derece)
  yaw_rate  : z ekseni acisal hiz (derece/sn) — DAIRE ve kaskad kontrol icin
  gyro      : (gx, gy, gz) ham eksen hizlari — roll/pitch PID'inin D terimi icin

DEGISIKLIKLER (bkz. PID_BASIT_ANLATIM.md):
  SORUN 1 : ACCEL_BIAS gecersizdi. Surucude duzeltme yok (kalibrasyon isi),
            ama artik calibrate_imu.py bozuk degeri config'e YAZDIRMIYOR.
            Burada ek olarak: bias'lar okunurken fiziksel siniri asiyorsa
            baslangicta EKRANA BUYUK UYARI basiyoruz.
  SORUN 2 : Tum I2C erisimleri I2C_LOCK ile sarildi (artik ayri thread'den
            okunuyor, ayni hatti PCA9685 ve derinlik sensoru de kullaniyor).
  YENI    : Orientation.gyro (3 eksen) disariya aciliyor — roll/pitch PID'i
            artik turev almak yerine dogrudan jiroskop hizini kullaniyor.
  YENI    : Manyetometre okumasi bozuksa (alan siddeti sacma) o adimda
            pusula duzeltmesi ATLANIR; jiroskopla devam edilir. Havuz
            kenarindaki demir donati sicrama yaptirmasin diye.
"""
import math
import time

from config import (I2C_BUS, IMU_ADDR, MAG_ADDR, HEADING_FILTER_ALPHA,
                    MAG_OFFSET, MAG_SCALE, GYRO_BIAS, USE_MAGNETOMETER)
from hal.i2c_lock import I2C_LOCK
from sensors.ekf import EKF

try:
    from config import ACCEL_BIAS
except ImportError:
    ACCEL_BIAS = (0.0, 0.0, 0.0)

try:
    from config import ROLL_PITCH_FILTER_ALPHA
except ImportError:
    ROLL_PITCH_FILTER_ALPHA = 0.98

try:
    from config import MOUNT_PITCH_DEG, MOUNT_ROLL_DEG
except ImportError:
    # calibrate_imu.py henuz calistirilmadiysa 0 kullan (kalibrasyonsuz calisir)
    MOUNT_PITCH_DEG = 0.0
    MOUNT_ROLL_DEG  = 0.0

# Manyetometre gecerlilik araligi (mikroTesla). Dunya alani ~25-65 uT.
MAG_MIN_UT = 5.0
MAG_MAX_UT = 200.0


def _bias_sagligi_uyar():
    """Import aninda config'teki kalibrasyon degerlerini fiziksel sinirlarla
    karsilastirir. SORUN 1'in bir daha sessizce olmasini engeller."""
    uyari = []
    if max(abs(b) for b in ACCEL_BIAS) > 0.3:
        uyari.append(
            f"ACCEL_BIAS={ACCEL_BIAS} FIZIKSEL OLARAK GECERSIZ.\n"
            "    Ivmeolcer +-2g araliginda calisiyor; 0.3g ustu bir 'sapma'\n"
            "    olamaz. Bu deger arac SALLANIRKEN kalibre edilmis demektir.\n"
            "    SONUCU: arac duz dururken program 'yan yatmisim' saniyor,\n"
            "    roll/pitch PID'leri bosuna calisiyor VE pusula da bozuluyor\n"
            "    (pusula egim telafisi icin roll/pitch kullanir).\n"
            "    YAP: python3 calibrate_imu.py  (arac DUZ ve HAREKETSIZ)")
    if max(abs(b) for b in GYRO_BIAS) > 5.0:
        uyari.append(
            f"GYRO_BIAS={GYRO_BIAS} cok buyuk (>5 dps).\n"
            "    Muhtemelen olcum sirasinda arac hareket ediyordu.\n"
            "    YAP: python3 calibrate_imu.py  (arac HAREKETSIZ)")
    if uyari:
        print("\n" + "!" * 74)
        for u in uyari:
            print("[IMU KALIBRASYON UYARISI] " + u)
        print("!" * 74 + "\n")


_bias_sagligi_uyar()


class Mpu9250:
    """Gercek donanim. Kurulum: pip3 install smbus2"""

    PWR_MGMT_1   = 0x6B
    USER_CTRL    = 0x6A   # I2C master modu
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H  = 0x43
    INT_PIN_CFG  = 0x37
    MAG_CNTL1    = 0x0A
    MAG_HXL      = 0x03
    MAG_ST2      = 0x09

    def __init__(self, bus_num=None):
        """MPU-9250'yi uyandirir, dahili AK8963 manyetometreye I2C bypass
        acar ve 16-bit/100Hz surekli moda alir.

        Magnetometre ZORUNLU degil: Errno 121 (bypass calismadi, donanim
        sorunu) durumunda has_mag=False ile devam eder. Bu durumda
        heading = sadece jiroskop entegrasyonu ile hesaplanir (suruklenme olabilir).
        """
        from smbus2 import SMBus
        buses_to_try = [bus_num] if bus_num is not None else [I2C_BUS, 8, 7, 1, 0]
        last_err = None
        self.bus = None
        for b in buses_to_try:
            try:
                bus = SMBus(b)
                with I2C_LOCK:
                    bus.write_byte_data(IMU_ADDR, self.PWR_MGMT_1, 0x00)   # uyandir
                time.sleep(0.10)  # guc stabilizasyonu icin bekleme
                self.bus = bus
                self._bus_num = b
                print(f"[IMU] MPU-9250 I2C Bus {b} (0x68) uzerinde baglandi.")
                break
            except Exception as e:
                last_err = e
        if self.bus is None:
            raise RuntimeError(
                f"MPU-9250 IMU sensorune baglanilamadi (Denenen Buslar: {buses_to_try}): {last_err}")

        # AK8963 bypass aktivasyonu:
        # 1) USER_CTRL: I2C master modu KAPAT (0x00) — acik kalirsa bypass calismaz
        # 2) INT_PIN_CFG: BYPASS_EN (bit1) = 0x02 — AK8963'u direkt I2C'ye bag
        # 3) Uzun bekleme — chip bypass modunu tanimak icin zaman gerekiyor
        with I2C_LOCK:
            self.bus.write_byte_data(IMU_ADDR, self.USER_CTRL, 0x00)
        time.sleep(0.05)
        with I2C_LOCK:
            self.bus.write_byte_data(IMU_ADDR, self.INT_PIN_CFG, 0x02)
        time.sleep(0.15)  # bypass aktiflesme icin yeterli bekleme

        # AK8963 magnetometre init — non-fatal (3 deneme)
        self.has_mag = False
        if USE_MAGNETOMETER:
            for attempt in range(3):
                try:
                    with I2C_LOCK:
                        self.bus.write_byte_data(MAG_ADDR, self.MAG_CNTL1, 0x16)  # 16bit, 100Hz
                    time.sleep(0.05)
                    self.has_mag = True
                    print(f"[IMU] AK8963 manyetometre aktif (Bus {self._bus_num}).")
                    break
                except OSError as e:
                    if attempt < 2:
                        print(f"[IMU] AK8963 deneme {attempt+1}/3 basarisiz (Errno {e.errno}), bekleniyor...")
                        time.sleep(0.20)
                    else:
                        print(
                            f"[IMU UYARI] AK8963 manyetometre erisilemedi (Errno {e.errno}).\n"
                            "            Olasilik: bypass registeri yazilmadi, kablo sorunu,\n"
                            "            veya IC donanim hatasi. Gyroskop+ivmeolcer CALISMAYI SURDURUYOR.\n"
                            "            Heading = sadece jiroskop entegrasyonu (zamanla suruklenebilir).")
        else:
            print("[IMU] config.py'de USE_MAGNETOMETER = False. Manyetometre bypass edildi (Sadece Gyro-Heading).")

    # ------------------------------------------------------------- ham okuma
    def _read_i16(self, addr, reg, little=False):
        with I2C_LOCK:
            hi = self.bus.read_byte_data(addr, reg)
            lo = self.bus.read_byte_data(addr, reg + 1)
        raw = (lo << 8 | hi) if little else (hi << 8 | lo)
        return raw - 65536 if raw > 32767 else raw

    def _read_block3(self, reg, scale):
        with I2C_LOCK:
            d = self.bus.read_i2c_block_data(IMU_ADDR, reg, 6)
        vals = []
        for i in range(3):
            raw = (d[2 * i] << 8) | d[2 * i + 1]
            if raw > 32767:
                raw -= 65536
            vals.append(raw / scale)
        return tuple(vals)

    def read_accel_g_raw(self):
        """Uc eksen ivme, KALIBRASYONSUZ, g biriminde."""
        return self._read_block3(self.ACCEL_XOUT_H, 16384.0)

    def read_accel_g(self):
        v = self.read_accel_g_raw()
        return tuple(v[i] - ACCEL_BIAS[i] for i in range(3))

    def read_gyro_dps_raw(self):
        """Uc eksen acisal hiz, KALIBRASYONSUZ, derece/sn."""
        return self._read_block3(self.GYRO_XOUT_H, 131.0)

    def read_gyro_dps(self):
        g = self.read_gyro_dps_raw()
        return tuple(g[i] - GYRO_BIAS[i] for i in range(3))

    def read_mag_ut_raw(self):
        """Uc eksen manyetik alan, KALIBRASYONSUZ, mikroTesla.
        AK8963 little-endian; ST2 okunmadan yeni veri gelmez.
        Manyetometre yoksa (has_mag=False) None dondurur."""
        if not self.has_mag:
            return None
        try:
            m = tuple(self._read_i16(MAG_ADDR, self.MAG_HXL + 2 * i, little=True) * 0.15
                      for i in range(3))
            with I2C_LOCK:
                self.bus.read_byte_data(MAG_ADDR, self.MAG_ST2)
            return m
        except OSError:
            return None

    def read_mag_ut(self):
        """Hard/soft iron kalibrasyonu uygulanmis manyetik alan.
        Manyetometre yoksa None dondurur."""
        m = self.read_mag_ut_raw()
        if m is None:
            return None
        return tuple((m[i] - MAG_OFFSET[i]) * MAG_SCALE[i] for i in range(3))


class MockImu:
    """Simulasyon: simulator.py durumu buraya yazar."""

    def __init__(self, sim=None):
        self.sim = sim

    def read_accel_g(self):
        """Duz durus varsayilir: sadece z ekseninde 1g (yercekimi)."""
        return (0.0, 0.0, 1.0)

    def read_gyro_dps(self):
        """Sadece z ekseninde (yaw) simulatorden gelen donus hizi."""
        return (0.0, 0.0, 0.0 if self.sim is None else self.sim.yaw_rate_dps)

    def read_mag_ut(self):
        """Simulatorun heading degerinden sahte manyetometre okumasi."""
        h = math.radians(0.0 if self.sim is None else self.sim.heading_deg)
        return (math.cos(h) * 30.0, -math.sin(h) * 30.0, 0.0)


class MockImuStatic:
    """Donanim bagli degil ve simulatorsuz calismak gerektiginde kullanilir.
    Sabit/sifir degerler dondurur. PID test gibi basit senaryolarda yeterli."""
    def read_accel_g(self):
        return (0.0, 0.0, 1.0)  # duz durus

    def read_gyro_dps(self):
        return (0.0, 0.0, 0.0)  # hareketsiz

    def read_mag_ut(self):
        return (30.0, 0.0, 0.0)  # kuzey yonu


class Orientation:
    """EKF (Extended Kalman Filter) ile heading/roll/pitch kestirimi.

    Jiroskop hizli ama SURUKLENIR.
    Ivmeolcer roll/pitch duzeltir.
    Pusula (veya gorsel isleme) heading duzeltir.
    EKF, bunlari istatistiksel en iyi sekilde harmanlar ve gyro bias'i da tahmin eder.
    """

    def __init__(self, driver):
        self.drv = driver
        self.ekf = EKF(dt=0.01)
        self.heading = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.gyro = (0.0, 0.0, 0.0)
        self.mag_heading = None
        self.mag_rejected = 0
        self._prev_t = None
        
        # Sadece ilk deger atamasi icin
        self._initialized = False

    @staticmethod
    def _mag_heading(mag, roll_r, pitch_r):
        """Egim-telafili (tilt-compensated) pusula hesabi."""
        mx, my, mz = mag
        xh = mx * math.cos(pitch_r) + mz * math.sin(pitch_r)
        yh = (mx * math.sin(roll_r) * math.sin(pitch_r) + my * math.cos(roll_r)
              - mz * math.sin(roll_r) * math.cos(pitch_r))
        return (math.degrees(math.atan2(-yh, xh)) + 360.0) % 360.0

    def update(self, external_yaw_rad=None):
        """Sensorleri bir kez okuyup EKF'yi isletir.
        external_yaw_rad: Eger disaridan mutlak bir yaw (Vision Grid gibi) verilirse, EKF bunu kullanir.
        """
        now = time.monotonic()
        dt = 0.01 if self._prev_t is None else max(1e-4, now - self._prev_t)
        self._prev_t = now

        ax, ay, az = self.drv.read_accel_g()
        gx, gy, gz = self.drv.read_gyro_dps()
        mag = self.drv.read_mag_ut()
        
        # Rad/s'ye cevir
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)

        # 1. EKF Predict (Jiroskop)
        self.ekf.predict([gx_rad, gy_rad, gz_rad], dt)

        # 2. EKF Update (Ivmeolcer)
        # MOUNT_ROLL_DEG / MOUNT_PITCH_DEG telafisini ivme vektorunu dondurerek yapmak EKF'de zordur,
        # ancak EKF Euler sonuclarindan cikarabiliriz.
        self.ekf.update_accel([ax, ay, az])

        # 3. EKF Update (Pusula veya Dis Yaw)
        mag_h = None
        if mag is not None:
            mag_norm = math.sqrt(sum(v * v for v in mag))
            mag_gecerli = MAG_MIN_UT < mag_norm < MAG_MAX_UT
            if mag_gecerli:
                # EKF Euler
                r, p, y = self.ekf.get_euler()
                mag_h = self._mag_heading(mag, r, p)
                self.mag_heading = mag_h
            else:
                self.mag_rejected += 1

        if external_yaw_rad is not None:
            self.ekf.update_yaw(external_yaw_rad)
        elif mag_h is not None:
            self.ekf.update_yaw(math.radians(mag_h))

        # Euler acilarini al
        r_rad, p_rad, y_rad = self.ekf.get_euler()
        
        # Montaj acilarini cikar (basit lineer cıkarma, Euler varsayımı ile)
        self.roll = math.degrees(r_rad) - MOUNT_ROLL_DEG
        self.pitch = math.degrees(p_rad) - MOUNT_PITCH_DEG
        
        # Heading 0-360 araligina getir
        raw_heading = (math.degrees(y_rad) + 360.0) % 360.0
        
        if not self._initialized:
            if mag_h is not None:
                self.heading = mag_h
                # EKF'nin baslangic yaw'ini ayarla
                dq = np.array([math.cos(math.radians(mag_h)/2), 0, 0, math.sin(math.radians(mag_h)/2)])
                self.ekf.q = dq
                raw_heading = mag_h
            else:
                self.heading = 0.0
            self._initialized = True
        else:
            self.heading = raw_heading

        # EKF'nin bias-kompanze edilmis jiroskop verisini disari veriyoruz (PID D terimi icin cok temiz)
        # EKF'nin bg'si rad/s cinsinden
        comp_gx = gx - math.degrees(self.ekf.bg[0])
        comp_gy = gy - math.degrees(self.ekf.bg[1])
        comp_gz = gz - math.degrees(self.ekf.bg[2])
        
        self.yaw_rate = comp_gz
        self.gyro = (comp_gx, comp_gy, comp_gz)

        return self.heading
