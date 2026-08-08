"""
Paylasilan arac durumu + sensor okuma thread'leri — YENI DOSYA.

==============================================================================
SORUN 2 ve SORUN 8'IN ASIL COZUMU BURASI
==============================================================================
ESKI DURUM:
  Kontrol dongusu sensoru kendisi okuyordu. Derinlik sensoru bir okumada
  40 ms boyunca programi DURDURUYORDU ve dongu basina 3 ayri yerden
  okunuyordu (stabilizer + gorev kodu + logger). Olculen sonuc:
      hedef 50 Hz  ->  gercek 7.3 Hz
  Yani araba kullanirken yola 7 saniyede bir bakmak gibi.

  Bunun PID'e etkisi:
    - D terimi anlamsizlasiyor (degisim hizini olcemiyor)
    - 136 ms gecikme: her gecikme kontrol sisteminde SALINIM uretir.
      Ayni Kp degeri hizli dongude stabilken yavas donguda salinir.
    - Duzensiz araliklar (88-138 ms): PID matematigi duzenli aralik varsayar.

YENI DURUM (bu dosya):
  Benzetme: eski halde asci hem yemek yapiyor hem surekli firina bakmaya
  gidiyordu, o sirada tezgah duruyordu. Simdi bir kisi surekli firina bakip
  TAHTAYA son sicakligi yaziyor; asci sadece tahtaya bakiyor ve hic beklemiyor.

    IMU thread'i      (100 Hz)  ->  RovState'e yazar
    Derinlik thread'i ( 20 Hz)  ->  RovState'e yazar
    Kontrol dongusu   ( 50 Hz)  ->  RovState'ten OKUR, asla beklemez

  Ayrica SORUN 8: sensor artik dongu basina 1 kez okunuyor. Gorev kodu ve
  logger ayni anlik goruntuyu (snapshot) kullanir, tekrar okuma yok.

EK: DERINLIK HIZI
  PID'in D terimi icin "saniyede kac metre iniyorum/cikiyorum" lazim.
  Bunu burada bir kez hesaplayip filtreliyoruz; PID sayisal turev almiyor.
  (bkz. PID.update(..., meas_rate=...))

EK: WATCHDOG
  Bir sensor thread'i olur da takilirsa, kontrol dongusu ESKI veriyle
  ucmaya devam eder — bu tehlikelidir. `healthy()` son verinin yasini
  kontrol eder; main.py bu bozulursa motorlari notre ceker.
"""
import threading
import time


class Snapshot:
    """Kontrol dongusunun tek adimda kullandigi TUTARLI durum kopyasi.

    Neden kopya: thread'ler surekli yaziyor. Dongu ortasinda heading
    guncellenirse, ayni adimda yarisi eski yarisi yeni veriyle hesap
    yapmis oluruz. Kopya alarak bunu engelliyoruz.
    """
    __slots__ = ("t", "heading", "roll", "pitch", "yaw_rate",
                 "gyro_x", "gyro_y", "gyro_z",
                 "depth_m", "depth_rate_mps",
                 "imu_age", "depth_age", "imu_hz", "depth_hz")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k, 0.0))


class RovState:
    """Sensor thread'lerinin yazdigi, kontrol dongusunun okudugu ortak hafiza."""

    def __init__(self):
        self._lock = threading.Lock()
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.gyro = (0.0, 0.0, 0.0)
        self.depth_m = 0.0
        self.depth_rate_mps = 0.0
        self._imu_t = 0.0
        self._depth_t = 0.0
        self.imu_hz = 0.0
        self.depth_hz = 0.0
        self.imu_ready = False
        self.depth_ready = False

    # ---------------------------------------------------------- yazma (thread)
    def set_imu(self, heading, roll, pitch, yaw_rate, gyro, now, hz):
        with self._lock:
            self.heading = heading
            self.roll = roll
            self.pitch = pitch
            self.yaw_rate = yaw_rate
            self.gyro = gyro
            self._imu_t = now
            self.imu_hz = hz
            self.imu_ready = True

    def set_depth(self, depth_m, depth_rate_mps, now, hz):
        with self._lock:
            self.depth_m = depth_m
            self.depth_rate_mps = depth_rate_mps
            self._depth_t = now
            self.depth_hz = hz
            self.depth_ready = True

    # ------------------------------------------------------ okuma (ana dongu)
    def snapshot(self, now=None):
        """Tutarli bir durum kopyasi dondurur. HIC BLOKLAMAZ."""
        now = time.monotonic() if now is None else now
        with self._lock:
            return Snapshot(
                t=now,
                heading=self.heading, roll=self.roll, pitch=self.pitch,
                yaw_rate=self.yaw_rate,
                gyro_x=self.gyro[0], gyro_y=self.gyro[1], gyro_z=self.gyro[2],
                depth_m=self.depth_m, depth_rate_mps=self.depth_rate_mps,
                imu_age=now - self._imu_t if self.imu_ready else 999.0,
                depth_age=now - self._depth_t if self.depth_ready else 999.0,
                imu_hz=self.imu_hz, depth_hz=self.depth_hz)


class SensorHub:
    """IMU ve derinlik sensorunu kendi thread'lerinde okur, RovState'i besler."""

    def __init__(self, orientation, depth_sensor,
                 imu_hz=100.0, depth_hz=20.0, depth_rate_tau=0.30,
                 stale_s=0.5):
        """
        orientation    : sensors.imu.Orientation (heading/roll/pitch kaynagi)
        depth_sensor   : Ms5837 ya da MockDepth
        imu_hz         : IMU okuma hedef frekansi
        depth_hz       : derinlik okuma hedef frekansi
                         (OSR=1024 ile bir okuma ~7 ms surer, 20 Hz rahat)
        depth_rate_tau : derinlik hizi alcak geciren filtre zaman sabiti (sn).
                         Buyuk = daha yumusak ama daha gec. 0.3 s iyi baslangic.
        stale_s        : bu suredir veri gelmiyorsa "saglıksız" say (watchdog)
        """
        self.ori = orientation
        self.depth = depth_sensor
        self.state = RovState()
        self.imu_dt = 1.0 / imu_hz
        self.depth_dt = 1.0 / depth_hz
        self.depth_rate_tau = depth_rate_tau
        self.stale_s = stale_s

        self.camera = None
        self.grid_tracker = None
        self.vision_yaw_deg = None
        self.use_vision_yaw = False
        self._vision_t = 0.0
        self.vision_hz = 0.0

        self._stop = threading.Event()
        self._threads = []
        self.imu_errors = 0
        self.depth_errors = 0
        self.vision_errors = 0
        
        self._last_err_print_t = 0.0

    def enable_vision(self, camera, grid_tracker):
        self.camera = camera
        self.grid_tracker = grid_tracker

    # ------------------------------------------------------------- yasam dongusu
    def start(self):
        """Sensor thread'lerini baslatir ve ilk verinin gelmesini bekler."""
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._imu_loop, name="imu", daemon=True),
            threading.Thread(target=self._depth_loop, name="depth", daemon=True),
        ]
        if self.camera and self.grid_tracker:
            self._threads.append(threading.Thread(target=self._vision_loop, name="vision", daemon=True))

        for t in self._threads:
            t.start()

        # Ilk veriyi bekle (en fazla 3 sn) — kontrol dongusu sifir heading ile
        # baslamasin, yoksa ilk adimda sacma bir hata gorur.
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            s = self.state.snapshot()
            if s.imu_age < 1.0 and s.depth_age < 1.0:
                break
            time.sleep(0.02)
        s = self.state.snapshot()
        print(f"[HUB] sensor thread'leri hazir — imu_age={s.imu_age:.2f}s "
              f"depth_age={s.depth_age:.2f}s")
        return self

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)

    def zero_at_surface(self):
        """Gorev basinda derinlik referansini sifirlar (thread calisiyorken guvenli)."""
        return self.depth.zero_at_surface()

    def healthy(self):
        """Watchdog: her iki sensorden de taze veri geliyor mu?"""
        s = self.state.snapshot()
        return s.imu_age < self.stale_s and s.depth_age < self.stale_s

    # -------------------------------------------------------------- thread'ler
    def _imu_loop(self):
        """IMU'yu sabit frekansta okur. Orientation.update() zaten tamamlayici
        filtreyi calistirip heading/roll/pitch/yaw_rate uretir."""
        next_t = time.monotonic()
        prev = next_t
        hz = 0.0
        while not self._stop.is_set():
            try:
                ext_yaw = None
                now = time.monotonic()
                if self.use_vision_yaw and self.vision_yaw_deg is not None and (now - self._vision_t) < 1.0:
                    import math
                    ext_yaw = math.radians(self.vision_yaw_deg)
                    
                self.ori.update(external_yaw_rad=ext_yaw)
                now = time.monotonic()
                inst = 1.0 / max(1e-3, now - prev)
                prev = now
                hz += 0.05 * (inst - hz)          # yumusatilmis frekans olcumu
                gx, gy, gz = getattr(self.ori, "gyro", (0.0, 0.0, self.ori.yaw_rate))
                self.state.set_imu(
                    heading=self.ori.heading if self.ori.heading is not None else 0.0,
                    roll=self.ori.roll, pitch=self.ori.pitch,
                    yaw_rate=self.ori.yaw_rate, gyro=(gx, gy, gz),
                    now=now, hz=hz)
            except Exception as e:
                self.imu_errors += 1
                if time.monotonic() - self._last_err_print_t > 2.0:
                    print(f"[HUB UYARI] IMU okuma hatasi: {e}")
                    self._last_err_print_t = time.monotonic()
            next_t += self.imu_dt
            time.sleep(max(0.0, next_t - time.monotonic()))
            if next_t < time.monotonic() - 0.5:   # cok geri kaldiysak saati sifirla
                next_t = time.monotonic()

    def _depth_loop(self):
        """Derinligi sabit frekansta okur ve DERINLIK HIZINI hesaplar.

        Derinlik hizi PID'in D terimi icin kullanilir (meas_rate). Boylece
        PID sayisal turev almak zorunda kalmaz — gurultu cok daha az olur.
        """
        next_t = time.monotonic()
        prev_t = None
        prev_d = None
        rate = 0.0
        hz = 0.0
        prev_hz_t = time.monotonic()
        while not self._stop.is_set():
            try:
                d = self.depth.read_depth_m()
                now = time.monotonic()

                inst = 1.0 / max(1e-3, now - prev_hz_t)
                prev_hz_t = now
                hz += 0.05 * (inst - hz)

                if prev_t is not None and now > prev_t:
                    dt = now - prev_t
                    raw_rate = (d - prev_d) / dt
                    a = dt / (self.depth_rate_tau + dt)   # zaman sabitli filtre
                    rate += a * (raw_rate - rate)
                prev_t, prev_d = now, d
                self.state.set_depth(d, rate, now, hz)
            except Exception as e:
                self.depth_errors += 1
                if time.monotonic() - self._last_err_print_t > 2.0:
                    print(f"[HUB UYARI] Derinlik okuma hatasi: {e}")
                    self._last_err_print_t = time.monotonic()
            next_t += self.depth_dt
            time.sleep(max(0.0, next_t - time.monotonic()))
            if next_t < time.monotonic() - 0.5:
                next_t = time.monotonic()

    def _vision_loop(self):
        """Kamerayi 30Hz'de okuyup Grid Tracker'i calistirir."""
        next_t = time.monotonic()
        prev_t = time.monotonic()
        hz = 0.0
        while not self._stop.is_set():
            try:
                frame = self.camera.read()
                if frame is not None:
                    yaw_err, _ = self.grid_tracker.process(frame)
                    now = time.monotonic()
                    inst = 1.0 / max(1e-3, now - prev_t)
                    prev_t = now
                    hz += 0.05 * (inst - hz)
                    
                    if yaw_err is not None:
                        self.vision_yaw_deg = yaw_err
                        self._vision_t = now
                        self.vision_hz = hz
            except Exception as e:
                self.vision_errors += 1
                if time.monotonic() - self._last_err_print_t > 2.0:
                    print(f"[HUB UYARI] Vision okuma hatasi: {e}")
                    self._last_err_print_t = time.monotonic()
                
            next_t += 1.0 / 30.0 # max 30 hz
            time.sleep(max(0.0, next_t - time.monotonic()))
            if next_t < time.monotonic() - 0.5:
                next_t = time.monotonic()
