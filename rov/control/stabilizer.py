"""
Stabilizasyon katmani: derinlik + heading + roll/pitch PID'lerini birlestirir.

Kullanim:
    stab = Stabilizer(orientation, depth_sensor)
    stab.set_targets(depth_m=0.6, heading_deg=90)
    axes = stab.compute(surge=0.35)   # -> mixer'a verilecek eksen komutlari
"""
from config import PID_DEPTH, PID_HEADING, PID_ROLL, PID_PITCH
from control.pid import PID, angle_error_deg


class Stabilizer:
    def __init__(self, orientation, depth_sensor):
        """orientation: Orientation nesnesi (heading/roll/pitch kaynagi).
        depth_sensor: Ms5837 ya da MockDepth. Her eksen icin ayri bir PID
        denetleyici olusturulur; hedefler baslangicta bos (None) birakilir."""
        self.ori = orientation
        self.depth = depth_sensor
        self.pid_depth = PID(**PID_DEPTH)
        self.pid_heading = PID(**PID_HEADING)
        self.pid_roll = PID(**PID_ROLL)
        self.pid_pitch = PID(**PID_PITCH)
        self.target_depth = None
        self.target_heading = None

    def set_targets(self, depth_m=None, heading_deg=None):
        """Derinlik ve/veya heading hedefini gunceller. Hedef gercekten
        degistiyse ilgili PID'i reset() ederek eski I/D birikimini temizler
        (aksi halde yeni hedefe gecerken sicrama olur). None gecilen eksen
        dokunulmadan kalir (var olan hedef korunur)."""
        if depth_m is not None and depth_m != self.target_depth:
            self.target_depth = depth_m
            self.pid_depth.reset()
        if heading_deg is not None:
            hd = heading_deg % 360.0
            if hd != self.target_heading:
                self.target_heading = hd
                self.pid_heading.reset()

    # ---- durum sorgulari ----
    def depth_error(self):
        """Hedef derinlik ile olculen derinlik arasindaki fark (metre)."""
        return self.target_depth - self.depth.read_depth_m()

    def heading_error(self):
        """Hedef heading ile olculen heading arasindaki en kisa acisal fark (derece)."""
        return angle_error_deg(self.target_heading, self.ori.heading)

    # ---- ana hesap ----
    def compute(self, surge=0.0, yaw_override=None):
        """Sensorleri okur, PID'leri kosar, eksen komutlarini dondurur.
        yaw_override: daire gorevi gibi sabit donus istenen durumlarda
                      heading PID yerine dogrudan yaw komutu."""
        self.ori.update()

        heave = 0.0
        if self.target_depth is not None:
            heave = self.pid_depth.update(self.depth_error())

        if yaw_override is not None:
            yaw = yaw_override
        elif self.target_heading is not None:
            yaw = self.pid_heading.update(self.heading_error())
        else:
            yaw = 0.0

        roll = self.pid_roll.update(-self.ori.roll)
        pitch = self.pid_pitch.update(-self.ori.pitch)

        return dict(surge=surge, yaw=yaw, heave=heave, roll=roll, pitch=pitch)
