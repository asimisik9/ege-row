"""
CSV loglama — PID optimizasyonunun TAMAMI bu dosyadan yapilacak.

==============================================================================
NEDEN GENISLETILDI (SORUN 7f + SORUN 2)
==============================================================================
ESKI LOG sadece sunlari yaziyordu:
    heading, depth, roll, pitch, yaw_rate, surge, yaw, heave
Bunlarla havuzda su soruyu CEVAPLAYAMIYORDUK:
    "Motor neden %70 gaz verdi? P'den mi geldi, I birikmis mi, D mi fren yapti?"
Yani KOR AYAR yapiyorduk. 3 saatlik havuz suresinde kor deneme = kayip deneme.

YENI LOG her PID'in IC TERIMLERINI ayri ayri yaziyor:
    depth_p, depth_i, depth_d, depth_ff, depth_sat
    yaw_p,   yaw_i,   yaw_sat, yaw_rate_target
Artik log dosyasina bakarak:
    - I terimi tavana dayanmissa   -> windup var, Ki'yi dusur
    - sat=1 uzun sure kalmissa     -> yetki yetmiyor / FF eksik
    - D cok buyuk ve titriyorsa    -> Kd'yi dusur ya da d_tau'yu buyut

AYRICA (SORUN 2):
  Eski logger her satirda `stab.depth.read_depth_m()` cagiriyordu — yani
  40 ms daha bloklama, dongu basina UCUNCU derinlik okumasi. Artik
  stabilizer'in ONBELLEGINDEKI degeri okuyor, sensore HIC dokunmuyor.

  Yeni `dt` ve `hz` sutunlari: dongunun gercekten kac Hz'de dondugunu
  logdan dogrulayabiliyoruz (kabul kriteri H1: >= 30 Hz).
"""
import csv
import os
import time

from config import LOG_DIR, LOG_EVERY_N, MOTOR_CHANNELS

HEADER = [
    "t", "dt", "hz", "state",
    # --- derinlik ekseni
    "depth", "depth_target", "depth_rate",
    "depth_p", "depth_i", "depth_d", "depth_ff", "depth_sat",
    # --- yon ekseni (kaskad)
    "heading", "heading_target", "heading_err",
    "yaw_rate", "yaw_rate_target", "yaw_p", "yaw_i", "yaw_sat",
    # --- roll/pitch
    "roll", "pitch", "roll_cmd", "pitch_cmd",
    # --- eksen komutlari
    "surge", "yaw", "heave",
    # --- motorlara giden nihai komutlar (olu bant telafisinden SONRA)
    "m_H_L", "m_H_R", "m_V_FL", "m_V_FR", "m_V_RL", "m_V_RR",
    # --- sensor sagligi
    "imu_hz", "depth_hz",
    "note",
]


def _r(v, n=3):
    """None-guvenli yuvarlama."""
    try:
        return round(float(v), n)
    except (TypeError, ValueError):
        return ""


class MissionLogger:
    def __init__(self, name="mission"):
        """LOG_DIR altinda '{name}_{tarih_saat}.csv' acar ve basligi yazar."""
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(LOG_DIR, f"{name}_{stamp}.csv")
        self._f = open(self.path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(HEADER)
        self._t0 = time.monotonic()
        self._n = 0
        self._prev_t = None

    def _t(self):
        return round(time.monotonic() - self._t0, 3)

    # ---------------------------------------------------------------- olaylar
    def event(self, note):
        """Durum degisikligi gibi onemli olaylari tek satir olarak yazar."""
        row = [""] * len(HEADER)
        row[0] = self._t()
        row[3] = "EVENT"
        row[-1] = note
        self._w.writerow(row)
        self._f.flush()

    def note(self, text):
        """Serbest not (ornegin havuzda hangi katsayiyla test edildigi)."""
        self.event(text)

    # ------------------------------------------------------------- telemetri
    def sample(self, state, stab, axes, thrusters=None):
        """Anlik telemetri yazar. SENSORU OKUMAZ — stabilizer'in onbellegini
        ve PID'lerin `.last` telemetrisini kullanir."""
        self._n += 1
        if self._n % LOG_EVERY_N:
            return

        now = time.monotonic()
        dt = 0.0 if self._prev_t is None else now - self._prev_t
        self._prev_t = now
        hz = (1.0 / dt) if dt > 1e-6 else 0.0

        snap = getattr(stab, "snap", None)
        dp = getattr(stab.pid_depth, "last", {})
        hc = stab.pid_heading                      # HeadingController
        hl = getattr(hc, "last", {})
        rp = getattr(hc.rate, "last", {}) if hasattr(hc, "rate") else {}
        rl = getattr(stab.pid_roll, "last", {})
        pl = getattr(stab.pid_pitch, "last", {})

        motors = {}
        if thrusters is not None:
            motors = getattr(thrusters, "_current", {})

        # dt ve hz LOG_EVERY_N ornek arasi oldugu icin gercek dongu frekansi
        # hz * LOG_EVERY_N olur; karisiklik olmasin diye ikisini de duzelttik.
        self._w.writerow([
            self._t(), _r(dt / max(1, LOG_EVERY_N), 4),
            _r(hz * LOG_EVERY_N, 1), state,
            # derinlik
            _r(stab.depth_m), _r(stab.target_depth),
            _r(getattr(snap, "depth_rate_mps", 0.0)),
            _r(dp.get("p")), _r(dp.get("i")), _r(dp.get("d")),
            _r(dp.get("ff")), dp.get("sat", ""),
            # yon
            _r(stab.heading_deg, 2), _r(stab.target_heading, 2),
            _r(hl.get("err"), 2),
            _r(getattr(snap, "yaw_rate", 0.0), 2), _r(hl.get("w_target"), 2),
            _r(rp.get("p")), _r(rp.get("i")), rp.get("sat", ""),
            # roll/pitch
            _r(getattr(snap, "roll", 0.0), 2), _r(getattr(snap, "pitch", 0.0), 2),
            _r(rl.get("out")), _r(pl.get("out")),
            # eksenler
            _r(axes.get("surge")), _r(axes.get("yaw")), _r(axes.get("heave")),
            # motorlar
            *[_r(motors.get(k, 0.0)) for k in
              ("H_L", "H_R", "V_FL", "V_FR", "V_RL", "V_RR")],
            # sensor sagligi
            _r(getattr(snap, "imu_hz", 0.0), 1),
            _r(getattr(snap, "depth_hz", 0.0), 1),
            "",
        ])

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass
