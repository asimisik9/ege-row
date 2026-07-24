"""
Quectel L80-R GNSS/GPS modulu surucu.

UART uzerinden NMEA 0183 satirlari okur; $GPRMC ve $GPGGA ayristirir.
Thread-guvenli: start() arka planda okur, fix property'si son konumu verir.

Kullanim:
    gps = GPS()
    gps.start()
    fix = gps.fix       # GpsFix | None
    gps.stop()

NMEA: $GPRMC,HHMMSS,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,...
"""
import re
import serial
import threading
import time
from dataclasses import dataclass
from typing import Optional

from config import GPS_SERIAL, GPS_BAUD, GPS_TIMEOUT_S


@dataclass
class GpsFix:
    lat: float       # ondalik derece, + = kuzey
    lon: float       # ondalik derece, + = dogu
    speed_ms: float  # m/s (GPRMC'den, deniz mili -> m/s)
    heading: float   # derece (GPRMC hareket yonu — compass degil)
    timestamp: float # time.monotonic()


class GPS:
    def __init__(self):
        self._fix: Optional[GpsFix] = None
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._port = None

    # ---------------------------------------------------------------- public
    @property
    def fix(self) -> Optional[GpsFix]:
        """Son gecerli GPS konumunu dondurur, yoksa None."""
        with self._lock:
            return self._fix

    def start(self):
        """UART okuma thread'ini baslat."""
        try:
            self._port = serial.Serial(GPS_SERIAL, GPS_BAUD, timeout=1.0)
        except serial.SerialException as e:
            print(f"[GPS] Port acilamadi ({GPS_SERIAL}): {e}")
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[GPS] {GPS_SERIAL}@{GPS_BAUD} baud dinleniyor.")
        # Ilk fix uyarisi
        t0 = time.monotonic()
        while self._fix is None and time.monotonic() - t0 < GPS_TIMEOUT_S:
            time.sleep(0.5)
        if self._fix is None:
            print(f"[GPS] {GPS_TIMEOUT_S:.0f}sn icerisinde fix alinamadi — "
                  "anteni kontrol et / gok goruntu saglandi mi?")

    def stop(self):
        self._running = False
        if self._port:
            self._port.close()

    def wait_fix(self, timeout_s: float = GPS_TIMEOUT_S) -> Optional[GpsFix]:
        """Fix gelene kadar bloke et. Yoksa None dondur."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            f = self.fix
            if f is not None:
                return f
            time.sleep(0.2)
        return None

    # ---------------------------------------------------------------- private
    def _loop(self):
        while self._running:
            try:
                line = self._port.readline().decode("ascii", errors="ignore").strip()
            except Exception:
                time.sleep(0.1)
                continue
            if line.startswith("$GPRMC") or line.startswith("$GNRMC"):
                fix = self._parse_rmc(line)
                if fix:
                    with self._lock:
                        self._fix = fix
            elif line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                fix = self._parse_gga(line)
                if fix:
                    with self._lock:
                        self._fix = fix

    @staticmethod
    def _nmea_to_decimal(value: str, direction: str) -> float:
        """DDMM.MMMM + N/S/E/W -> ondalik derece."""
        if not value:
            return 0.0
        dot = value.index(".")
        deg = float(value[:dot - 2])
        minutes = float(value[dot - 2:])
        decimal = deg + minutes / 60.0
        if direction in ("S", "W"):
            decimal = -decimal
        return decimal

    @classmethod
    def _parse_rmc(cls, line: str) -> Optional[GpsFix]:
        """$GPRMC satirini ayristir."""
        parts = line.split(",")
        if len(parts) < 10:
            return None
        status = parts[2]
        if status != "A":
            return None   # fix yok
        try:
            lat = cls._nmea_to_decimal(parts[3], parts[4])
            lon = cls._nmea_to_decimal(parts[5], parts[6])
            speed_kn = float(parts[7]) if parts[7] else 0.0
            hdg = float(parts[8]) if parts[8] else 0.0
            return GpsFix(lat=lat, lon=lon,
                          speed_ms=speed_kn * 0.5144,
                          heading=hdg,
                          timestamp=time.monotonic())
        except (ValueError, IndexError):
            return None

    @classmethod
    def _parse_gga(cls, line: str) -> Optional[GpsFix]:
        """$GPGGA satirini ayristir (GPRMC fix yoksa yedek)."""
        parts = line.split(",")
        if len(parts) < 7:
            return None
        fix_q = parts[6]
        if fix_q == "0":
            return None
        try:
            lat = cls._nmea_to_decimal(parts[2], parts[3])
            lon = cls._nmea_to_decimal(parts[4], parts[5])
            return GpsFix(lat=lat, lon=lon, speed_ms=0.0,
                          heading=0.0, timestamp=time.monotonic())
        except (ValueError, IndexError):
            return None
