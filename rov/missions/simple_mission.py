"""
Basit açık çevrim (open-loop) otonom görev.
Derinlik veya Yön PID'lerini kullanmaz (hedefleri kapatır). 
Motorlara doğrudan zaman ayarlı komut (surge, yaw, heave) verir.
Roll ve Pitch PID'leri aracı düz tutmak için arka planda hafifçe çalışmaya devam eder.
"""
import time
from control.mixer import mix

class SimpleMission:
    def __init__(self, stabilizer, thrusters, logger=None):
        self.stab = stabilizer
        self.thr = thrusters
        self.log = logger
        self.state = "IDLE"
        self._t0 = None
        self.last_axes = dict(surge=0.0, yaw=0.0, heave=0.0, roll=0.0, pitch=0.0)

    def start(self):
        self._t0 = time.monotonic()
        self.state = "DIVE"
        
        # Derinlik ve Heading hedeflerini serbest bırak (PID kapatılır)
        self.stab.set_targets(depth_m=None, heading_deg=None)
        
        if self.log:
            self.log.event("SIMPLE MISSION START")

    def abort(self):
        self.state = "ABORT"

    def step(self):
        if self.state in ("IDLE", "ABORT", "DONE"):
            return True

        t = time.monotonic() - self._t0
        s = self.stab.sample()

        # Stabilizer'ı hesapla (heave ve yaw = 0 döner çünkü hedefleri kapattık.
        # Ama roll ve pitch için düzeltme dönmeye devam eder).
        axes = self.stab.compute() 

        surge = 0.0
        yaw = 0.0
        heave = 0.0

        # Zaman ayarlı hareketler (Açık çevrim)
        if t < 4.0:
            self.state = "DIVE"
            heave = 0.5  # Pozitif = Dalış
        elif t < 8.0:
            self.state = "FORWARD 1"
            heave = 0.1  # Asılı kalma gücü (FF)
            surge = 0.5  # İleri git
        elif t < 12.0:
            self.state = "TURN RIGHT"
            heave = 0.1
            yaw = 0.4    # Sağa dön
        elif t < 16.0:
            self.state = "FORWARD 2"
            heave = 0.1
            surge = 0.5  # Tekrar ileri
        elif t < 20.0:
            self.state = "SURFACE"
            heave = -0.5 # Negatif = Çıkış
        else:
            self.state = "DONE"
            return True

        # Ekseni Stabilizer'ın üzerine yazıyoruz
        axes["surge"] = surge
        axes["yaw"] = yaw
        axes["heave"] = heave
        
        self.last_axes = axes
        pwms = mix(**axes)
        self.thr.set_pwm(pwms)

        if self.log:
            self.log.step(self.state, axes, s, pwms)

        return False
