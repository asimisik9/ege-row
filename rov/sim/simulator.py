"""
Basit 3-DOF ROV simulasyonu (x, y, heading + derinlik).
Amac: algoritma dogrulama — fizik hassasiyeti degil, MANTIK dogrulugu.

==============================================================================
NEDEN GUNCELLENDI
==============================================================================
Eski simulasyon NOTR YUZERLIKTE bir arac modelliyordu: motorlar kapaliyken
arac oldugu derinlikte kaliyordu. Bu, gercege UYMUYOR ve daha kotusu,
yeni tasarimin en onemli parcasini TEST EDILEMEZ hale getiriyordu:

  - ILERI BESLEME (FF_HOVER) sadece pozitif yuzerlikte anlam kazanir.
    Notr yuzerlikte ff=0 zaten dogru cevap olur, hicbir sey ogrenemeyiz.
  - I terimi ve windup davranisi da ancak surekli bir yuke karsi test edilir.

Eklenenler:
  BUOYANCY_MS  : motorlar kapaliyken aracin yukselme hizi (m/s).
                 Gercek arac ARIZA HALINDE YUZMELI (guvenlik), yani pozitif
                 kaldirma kuvvetli olmali. Bu yuzden sabit bir yukari kayma var.
                 Beklenen FF_HOVER = BUOYANCY_MS / MAX_VZ
  YAW_BIAS_DPS : motor/pervane asimetrisi — heading Ki'sini test etmek icin.
  current      : su akintisi (zaten vardi) — rota kapanmasini test etmek icin.

Not: mixer'in olu bant telafisi ve Thrusters'in olu bandi PWM uzerinden
zaten simulasyona yansiyor (backend'e yazilan gercek PWM okunuyor), yani
SORUN 3'un etkisi burada da gorulebilir.
"""
import math

from config import MOTOR_CHANNELS, PWM_NEUTRAL_US, PWM_MAX_US, MOTOR_DIRECTION


class RovSimulator:
    # kaba model katsayilari (Degz M1 ~ birkac kg itki; kutle ~5.7 kg KTR'den)
    MAX_SPEED = 0.8        # m/s tam gazda
    MAX_YAW_RATE = 60.0    # derece/sn tam diferansiyelde
    MAX_VZ = 0.5           # m/s dikeyde
    TAU_LIN = 0.8          # hiz zaman sabiti (su direnci)
    TAU_YAW = 0.5

    # --- bozucular (disturbance) ---
    BUOYANCY_MS = 0.06     # motorlar kapaliyken yukselme hizi (m/s)
                           # -> beklenen FF_HOVER = 0.06/0.5 = 0.12
    YAW_BIAS_DPS = 1.5     # motor asimetrisi: sabit sola/saga kayma (derece/sn)

    def __init__(self, current=(0.0, 0.0), buoyancy_ms=None, yaw_bias_dps=None):
        """current: (vx, vy) sabit su akintisi m/s — kontrolun akintiya karsi
        dayanikliligini test etmek icin bozucu olarak eklenir."""
        self.x = self.y = 0.0
        self.depth_m = 0.0
        self.heading_deg = 0.0
        self.yaw_rate_dps = 0.0
        self.speed = 0.0
        self.vz = 0.0
        self.current = current
        self.buoyancy_ms = self.BUOYANCY_MS if buoyancy_ms is None else buoyancy_ms
        self.yaw_bias_dps = self.YAW_BIAS_DPS if yaw_bias_dps is None else yaw_bias_dps
        self.trail = []             # (x, y) izi

    def step(self, backend, dt):
        """Simulasyonu dt saniye ileri goturur: MockBackend'e yazilmis son
        PWM degerlerini okur, eksen komutlarina cevirir, birinci derece
        dinamikle hiz/donus/dikey hizi gunceller, sonra konumu entegre eder."""
        us = {name: backend.last_us.get(ch, PWM_NEUTRAL_US)
              for name, ch in MOTOR_CHANNELS.items()}
        # PWM -> -1..1 (yon duzeltmesini geri uygula — config.py ile tam uyumlu)
        span = max(1.0, float(PWM_MAX_US - PWM_NEUTRAL_US))
        val = {n: (us[n] - PWM_NEUTRAL_US) / span * MOTOR_DIRECTION[n] for n in us}

        # motor PWM degerlerinden eksen komutlarini geri cikar (mixer'in tersi)
        surge_cmd = (val["H_L"] + val["H_R"]) / 2.0
        yaw_cmd = (val["H_R"] - val["H_L"]) / 2.0
        heave_cmd = (val["V_FL"] + val["V_FR"] + val["V_RL"] + val["V_RR"]) / 4.0

        # birinci derece dinamik (su direnci: hiz aninda degil, zaman sabitiyle)
        self.speed += (surge_cmd * self.MAX_SPEED - self.speed) * dt / self.TAU_LIN
        self.yaw_rate_dps += (yaw_cmd * self.MAX_YAW_RATE + self.yaw_bias_dps
                              - self.yaw_rate_dps) * dt / self.TAU_YAW
        # POZITIF YUZERLIK: heave=0 iken arac yukselir (vz negatif)
        vz_hedef = heave_cmd * self.MAX_VZ - self.buoyancy_ms
        self.vz += (vz_hedef - self.vz) * dt / self.TAU_LIN

        # konum/yonelim/derinligi entegre et (Euler)
        self.heading_deg = (self.heading_deg + self.yaw_rate_dps * dt) % 360.0
        h = math.radians(self.heading_deg)
        self.x += (self.speed * math.cos(h) + self.current[0]) * dt
        self.y += (self.speed * math.sin(h) + self.current[1]) * dt
        # su yuzeyinin ustune cikamaz (fiziksel sinir)
        self.depth_m = max(0.0, self.depth_m + self.vz * dt)
        self.trail.append((self.x, self.y))
