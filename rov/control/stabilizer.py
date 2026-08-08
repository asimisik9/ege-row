"""
Stabilizasyon katmani — derinlik + yon + roll/pitch kontrolunu birlestirir.

Bu dosya asagidaki sorunlarin bulustugu yer:

  SORUN 2/8 : Artik sensoru KENDISI OKUMUYOR. SensorHub'in doldurdugu
              RovState'ten tek bir anlik goruntu (snapshot) aliyor.
              Boylece dongu basina 1 okuma, hic bloklama yok.
              (Eski halde compute() her cagrildiginda 40 ms bekliyordu.)

  SORUN 4b  : Derinlik PID'ine ILERI BESLEME (FF_HOVER) eklendi.
              Arac pozitif kaldirma kuvvetli oldugu icin 0.6 m'de DURMAK
              bile surekli asagi itki ister. Eskiden bunu tek basina I
              terimi tasiyordu (yavas + windup). Artik bilinen kismi
              FF veriyor, PID sadece artigi duzeltiyor.

  SORUN 4a  : Derinlik isaretli okunuyor; arac yuzeye firlarsa PID goruyor.

  SORUN 7a  : Derinlik D terimi artik OLCULEN DIKEY HIZ (m/s) uzerinden.
              Roll/pitch D terimi ise dogrudan JIROSKOP hizindan.
              Hicbir yerde hata turevi alinmiyor -> hedef degisince sicrama yok.

  §5.2      : Yon kontrolu tek PID degil KASKAD (control/cascade.py).
              Duz seyirde 'cruise' modu (dusuk yetki, dusuk donus hizi),
              yerinde donuste 'turn' modu (tam yetki).

GERIYE UYUMLULUK:
  Eski API korundu — .ori, .depth, .set_targets(), .compute(surge, yaw_override),
  .depth_error(), .heading_error(), .pid_depth/.pid_heading/.pid_roll/.pid_pitch
  hepsi calisiyor (missions/line_follow.py, missions/nav_mission.py ve
  comms/web_server.py bunlari kullaniyor).
"""
from config import (PID_DEPTH, PID_ROLL, PID_PITCH,
                    HEADING_POS, HEADING_RATE, HEADING_MODES)
from control.pid import PID, angle_error_deg
from control.cascade import HeadingController


class Stabilizer:
    def __init__(self, orientation, depth_sensor, state=None):
        """
        orientation  : sensors.imu.Orientation (geriye uyumluluk + sim)
        depth_sensor : Ms5837 / MockDepth
        state        : sensors.state.RovState. VERILIRSE sensorler buradan
                       okunur (bloklamayan, hizli yol). Verilmezse eski
                       dogrudan-okuma yolu kullanilir (yedek).
        """
        self.ori = orientation
        self.depth = depth_sensor
        self.state = state

        self.pid_depth = PID(**PID_DEPTH, name="depth")
        self.pid_heading = HeadingController(HEADING_POS, HEADING_RATE, HEADING_MODES)
        self.pid_roll = PID(**PID_ROLL, name="roll")
        self.pid_pitch = PID(**PID_PITCH, name="pitch")

        self.target_depth = None
        self.target_heading = None
        self.target_roll = None
        self.target_pitch = None

        # Son anlik goruntu — gorev kodu ve logger BUNU okur, sensoru tekrar
        # okumaz (SORUN 8). compute() her cagrildiginda tazelenir.
        self.snap = None
        self.depth_m = 0.0
        self.heading_deg = 0.0

    # ------------------------------------------------------------- hedefler
    def set_targets(self, depth_m=None, heading_deg=None, roll_deg=None, pitch_deg=None):
        """Derinlik, yon, roll ve pitch hedeflerini gunceller.

        Hedef GERCEKTEN degistiyse ilgili denetleyici reset edilir; aksi
        halde eski I birikimi yeni hedefe sicrar. None gecilen eksen
        dokunulmadan kalir."""
        if depth_m is not None and depth_m != self.target_depth:
            self.target_depth = depth_m
            self.pid_depth.reset()
        if heading_deg is not None:
            hd = heading_deg % 360.0
            if hd != self.target_heading:
                self.target_heading = hd
                self.pid_heading.reset()
        if roll_deg is not None and roll_deg != self.target_roll:
            self.target_roll = roll_deg
            self.pid_roll.reset()
        if pitch_deg is not None and pitch_deg != self.target_pitch:
            self.target_pitch = pitch_deg
            self.pid_pitch.reset()

    def set_heading_mode(self, mode):
        """'cruise' (duz seyir) / 'turn' (yerinde donus) — bkz. cascade.py"""
        self.pid_heading.set_mode(mode)

    def set_depth_ff(self, ff):
        """Havuzda olculen 'asili kalma gucu'nu canli olarak uygular."""
        self.pid_depth.set_params(ff=ff, reset=False)

    # -------------------------------------------------------- sensor okuma
    def sample(self, now=None):
        """Sensorlerin anlik goruntusunu alir ve saklar. Dongu basina 1 KEZ."""
        if self.state is not None:
            s = self.state.snapshot(now)
            self.snap = s
            self.depth_m = s.depth_m
            self.heading_deg = s.heading
        else:
            # yedek yol: thread yoksa dogrudan oku (yavas — sadece test icin)
            self.ori.update()
            d = self.depth.read_depth_m()

            class _S:
                pass
            s = _S()
            s.heading = self.ori.heading or 0.0
            s.roll = self.ori.roll
            s.pitch = self.ori.pitch
            s.yaw_rate = self.ori.yaw_rate
            g = getattr(self.ori, "gyro", (0.0, 0.0, self.ori.yaw_rate))
            s.gyro_x, s.gyro_y, s.gyro_z = g
            s.depth_m = d
            s.depth_rate_mps = 0.0
            s.imu_age = s.depth_age = 0.0
            s.imu_hz = s.depth_hz = 0.0
            self.snap = s
            self.depth_m = d
            self.heading_deg = s.heading
        return self.snap

    # ---------------------------------------------------------- durum sorgu
    def depth_error(self):
        """Hedef derinlik - olculen derinlik (metre). SENSORU TEKRAR OKUMAZ:
        son anlik goruntuyu kullanir (SORUN 8)."""
        if self.snap is None:
            self.sample()
        return (self.target_depth or 0.0) - self.depth_m

    def heading_error(self):
        """Hedef yon ile olculen yon arasindaki en kisa acisal fark (derece)."""
        if self.snap is None:
            self.sample()
        return angle_error_deg(self.target_heading or 0.0, self.heading_deg)

    # ------------------------------------------------------------ ana hesap
    def compute(self, surge=0.0, yaw_override=None, yaw_rate_target=None,
                now=None, resample=True):
        """Bir kontrol adimi: sensorleri orneklyip eksen komutlarini dondurur.

        surge           : ileri gaz (gorev tarafindan verilir, acik cevrim)
        yaw_override    : dogrudan yaw komutu (vizyon gorevleri icin — heading
                          PID'i devre disi kalir)
        yaw_rate_target : DONUS HIZI hedefi (derece/sn). DAIRE gorevi bunu
                          kullanir; kaskadin ic dongusu devreye girer.
        resample        : False verilirse mevcut anlik goruntu tekrar kullanilir
        """
        s = self.sample(now) if resample else (self.snap or self.sample(now))

        # ---- DERINLIK -----------------------------------------------------
        # D terimi: olculen dikey hiz (m/s). Pozitif = derinlesiyor.
        # hata = hedef - olcum oldugu icin PID icinde isaret ters cevriliyor.
        heave = 0.0
        if self.target_depth is not None:
            heave = self.pid_depth.update(
                self.target_depth - s.depth_m,
                meas_rate=s.depth_rate_mps,
                now=now)

        # ---- YON ----------------------------------------------------------
        if yaw_override is not None:
            yaw = yaw_override
        elif yaw_rate_target is not None:
            yaw = self.pid_heading.update_rate(yaw_rate_target, s.yaw_rate, now=now)
        elif self.target_heading is not None:
            yaw = self.pid_heading.update_heading(
                self.target_heading, s.heading, s.yaw_rate, now=now)
        else:
            yaw = 0.0

        # ---- ROLL / PITCH -------------------------------------------------
        # Hedef atanmissa kullanir, yoksa 0 derece (duz durus) hedefler.
        # D terimi jiroskoptan. deadzone sayesinde kucuk acilarda hic karismaz.
        t_roll = self.target_roll or 0.0
        roll = self.pid_roll.update(t_roll - s.roll, meas_rate=s.gyro_x, now=now)
        
        t_pitch = self.target_pitch or 0.0
        pitch = self.pid_pitch.update(t_pitch - s.pitch, meas_rate=s.gyro_y, now=now)

        return dict(surge=surge, yaw=yaw, heave=heave, roll=roll, pitch=pitch)
