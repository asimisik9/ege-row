"""
Olu hesap (Dead Reckoning) navigasyon.

IMU heading + tahmin edilen ilerleme hizini entegre ederek XY konumunu hesaplar.
GPS fix geldiginde pozisyonu sifirlar/gunceller.

Kullanim:
    dr = DeadReckoning()
    dr.reset()                     # baslangic noktasi
    dr.update(heading_deg, speed)  # her kontrol dongusunde (50Hz)
    x, y = dr.position            # metre, baslangica gore

    # GPS fix geldiginde
    dr.set_gps(fix)               # (opsiyonel) referans sifirla
"""
import math
import time
from typing import Tuple, Optional


class DeadReckoning:
    """
    Basit Euler entegrasyonu.

    heading_deg : 0 = kuzey, 90 = dogu (pusula yonu)
    speed_ms    : ileri hiz (m/s) — kalibre edilmis throttle -> hiz tablosundan
    """

    # Konservatif hiz modeli: throttle 0.35 ~ 0.3 m/s (havuz testinden ayarlanacak)
    # config.py'de CRUISE_SPEED/THR ile eslestir
    THROTTLE_TO_MS = 0.8    # m/s per unit throttle (1.0 = tam gaz)

    def __init__(self):
        self.x: float = 0.0       # metre, baslangica gore dogu+
        self.y: float = 0.0       # metre, baslangica gore kuzey+
        self._prev_t: Optional[float] = None

    # ---------------------------------------------------------------- public
    def reset(self, x: float = 0.0, y: float = 0.0):
        """Konumu sifirla (gorev basinda cagir)."""
        self.x = x
        self.y = y
        self._prev_t = None
        print(f"[DR] Konum sifirlandi: ({x:.2f}, {y:.2f})")

    def update(self, heading_deg: float, throttle: float):
        """
        Mevcut heading ve ileri gazla pozisyonu guncelle.
        50Hz dongu icin tasarlandi (dt ~ 0.02s).

        heading_deg : Orientation.heading (0-360, kuzey=0, dogu=90)
        throttle    : surge komutu (-1..+1)
        """
        now = time.monotonic()
        dt = 0.0 if self._prev_t is None else min(0.1, now - self._prev_t)
        self._prev_t = now

        if dt == 0.0:
            return

        speed = throttle * self.THROTTLE_TO_MS
        h_rad = math.radians(heading_deg)
        # Kuzey = 0° → X = dogu, Y = kuzey
        self.x += speed * math.sin(h_rad) * dt
        self.y += speed * math.cos(h_rad) * dt

    def set_gps(self, lat: float, lon: float,
                ref_lat: float, ref_lon: float):
        """
        GPS koordinatlarini metre cinsinden konuma cevir ve pozisyonu guncelle.
        ref_lat/lon: baslangic referans noktasi (0,0 kabul edilen).
        """
        # Basit duzlemsel yaklasim (kisa mesafeler icin yeterli)
        EARTH_R = 6_371_000.0
        dlat = math.radians(lat - ref_lat)
        dlon = math.radians(lon - ref_lon)
        dy = dlat * EARTH_R
        dx = dlon * EARTH_R * math.cos(math.radians(ref_lat))
        self.x = dx
        self.y = dy

    @property
    def position(self) -> Tuple[float, float]:
        """(x_dogu, y_kuzey) metre."""
        return (self.x, self.y)

    def distance_to(self, tx: float, ty: float) -> float:
        """Hedef noktaya uzaklik (metre)."""
        return math.hypot(tx - self.x, ty - self.y)

    def bearing_to(self, tx: float, ty: float) -> float:
        """Hedef noktaya yon (0-360 derece, kuzey=0)."""
        dx, dy = tx - self.x, ty - self.y
        angle = math.degrees(math.atan2(dx, dy))
        return angle % 360.0
