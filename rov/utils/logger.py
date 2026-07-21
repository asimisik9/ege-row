"""CSV loglama - PID optimizasyonu icin her test kaydedilir."""
import csv
import os
import time
from config import LOG_DIR, LOG_EVERY_N


class MissionLogger:
    def __init__(self, name="mission"):
        """LOG_DIR altinda '{name}_{tarih_saat}.csv' adinda yeni bir log
        dosyasi acar ve basligi yazar."""
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(LOG_DIR, f"{name}_{stamp}.csv")
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "type", "state", "heading", "target_heading",
                          "depth", "target_depth", "roll", "pitch", "yaw_rate",
                          "surge", "yaw", "heave", "note"])
        self._t0 = time.monotonic()
        self._n = 0

    def _t(self):
        """Log baslangicindan bu yana gecen sure (saniye, 3 basamak)."""
        return round(time.monotonic() - self._t0, 3)

    def event(self, note):
        """Onemli bir olayi (orn. durum degisikligi) tek satir olarak loglar."""
        self._w.writerow([self._t(), "EVENT", "", "", "", "", "", "", "", "", "", "", "", note])
        self._f.flush()

    def sample(self, state, stab, axes):
        """Anlik telemetri (heading, derinlik, roll/pitch, eksen komutlari)
        yazar; LOG_EVERY_N cagridan birinde kaydeder (dosya boyutunu ve
        yazma yukunu azaltmak icin)."""
        self._n += 1
        if self._n % LOG_EVERY_N:
            return
        self._w.writerow([
            self._t(), "DATA", state,
            round(stab.ori.heading or 0, 2), round(stab.target_heading or 0, 2),
            round(stab.depth.read_depth_m(), 3), stab.target_depth,
            round(stab.ori.roll, 2), round(stab.ori.pitch, 2),
            round(stab.ori.yaw_rate, 2),
            round(axes["surge"], 3), round(axes["yaw"], 3), round(axes["heave"], 3), ""])

    def close(self):
        """Log dosyasini kapatir."""
        self._f.close()
