"""
Basit 3-DOF ROV simulasyonu (x, y, heading + derinlik).
Amac: algoritma dogrulama - fizik hassasiyeti degil, mantik dogrulugu.
Gercek zamandan hizli kosabilmesi icin monotonic saat yamalanir (run_sim.py).
"""
import math
from config import MOTOR_CHANNELS, PWM_NEUTRAL_US, MOTOR_DIRECTION


class RovSimulator:
    # kaba model katsayilari (Degz M1 ~ birkac kg itki; kutle ~5.7 kg KTR'den)
    MAX_SPEED = 0.8        # m/s tam gazda
    MAX_YAW_RATE = 60.0    # derece/sn tam diferansiyelde
    MAX_VZ = 0.5           # m/s dikeyde
    TAU_LIN = 0.8          # hiz zaman sabiti (su direnci)
    TAU_YAW = 0.5

    def __init__(self, current=(0.0, 0.0)):
        """current: (vx, vy) sabit su akintisi m/s - kontrolun akintiya karsi
        dayanikliligini test etmek icin bozucu (disturbance) olarak eklenir."""
        self.x = self.y = 0.0
        self.depth_m = 0.0
        self.heading_deg = 0.0
        self.yaw_rate_dps = 0.0
        self.speed = 0.0
        self.vz = 0.0
        self.current = current      # akinti (m/s) - bozucu test icin
        self.trail = []             # (x, y) izi

    # Thrusters MockBackend'inden PWM'leri okuyup fizigi ilerlet
    def step(self, backend, dt):
        """Simulasyonu dt saniye ileri goturur: MockBackend'e yazilmis son
        PWM degerlerini okur, bunlari surge/yaw/heave komutlarina cevirir,
        birinci derece (gecikmeli) dinamikle hiz/donus/dikey hizi gunceller,
        sonra konum/derinlik/heading'i entegre eder ve rota izine ekler."""
        us = {name: backend.last_us.get(ch, PWM_NEUTRAL_US)
              for name, ch in MOTOR_CHANNELS.items()}
        # PWM -> -1..1 (yon duzeltmesini geri uygula)
        val = {n: (us[n] - PWM_NEUTRAL_US) / 400.0 * MOTOR_DIRECTION[n] for n in us}

        # motor PWM degerlerinden eksen komutlarini geri cikar (mixer'in tersi)
        surge_cmd = (val["H_L"] + val["H_R"]) / 2.0
        yaw_cmd = (val["H_R"] - val["H_L"]) / 2.0
        heave_cmd = (val["V_FL"] + val["V_FR"] + val["V_RL"] + val["V_RR"]) / 4.0

        # birinci derece dinamik (su direnci nedeniyle hiz aninda degil,
        # TAU_LIN/TAU_YAW zaman sabitiyle hedefe yaklasir)
        self.speed += (surge_cmd * self.MAX_SPEED - self.speed) * dt / self.TAU_LIN
        self.yaw_rate_dps += (yaw_cmd * self.MAX_YAW_RATE - self.yaw_rate_dps) * dt / self.TAU_YAW
        self.vz += (heave_cmd * self.MAX_VZ - self.vz) * dt / self.TAU_LIN

        # konum/yonelim/derinligi entegre et (Euler entegrasyonu)
        self.heading_deg = (self.heading_deg + self.yaw_rate_dps * dt) % 360.0
        h = math.radians(self.heading_deg)
        self.x += (self.speed * math.cos(h) + self.current[0]) * dt
        self.y += (self.speed * math.sin(h) + self.current[1]) * dt
        self.depth_m = max(0.0, self.depth_m + self.vz * dt)
        self.trail.append((self.x, self.y))
