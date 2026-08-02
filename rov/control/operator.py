"""
Operator (yer istasyonu) kontrol katmani — YENI DOSYA.

NEDEN VAR
=========
Web arayuzunden "hedef derinlik 1.2 m" dedigimizde hicbir sey olmuyordu.
Sebep sinsi: gorevlerin `step()` metodu KENDI hedefini her cagrilista
yeniden yaziyor —

    missions/video_demo.py:118   self.stab.set_targets(depth_m=M["target_depth_m"])
    missions/line_follow.py:108  self.stab.set_targets(depth_m=LINE_TARGET_DEPTH)
    missions/nav_mission.py:119  self.stab.set_targets(depth_m=NAV_TARGET_DEPTH)

`step()` ana dongude 50 Hz cagriliyor. Yani operatorun web'den yazdigi hedef
20 MILISANIYE sonra gorev tarafindan siliniyordu. Arayuzde deger degisiyor
gibi gorunuyordu cunku telemetri bir sonraki karede gorevin hedefini geri
okuyordu.

COZUM
=====
Hedefi kimin sahiplendigini ACIKCA belirleyen bir mod anahtari:

    AUTO   : hedefleri GOREV yonetir (eski davranis, yarisma modu)
    HOLD   : hedefleri OPERATOR yonetir; gorev duraklatilir, PID tutmaya
             devam eder. "Su derinlikte, su yonde dur."
    HOVER  : derinlik PID'i KAPALI, sabit dikey gaz. FF_HOVER olcumu icin
             (bkz. config.FF_HOVER — arac ne cikip ne iniyorsa o degerdir).
    RATE   : sabit DONUS HIZI hedefi (daire gorevi ayari; 'circle' modu).
    TELEOP : operatorun klavye/joystick eksenleri dogrudan mixer'a gider.

Ana dongu (main.py::_run_mission_loop) her adimda `Operator.step()` cagirir;
mission.step() SADECE AUTO modunda calisir. Boylece operator hedefi hicbir
yerde ezilmez.

AYRICA: adim cevabi kaydedici/analizci burada. pid_tune.py'daki konsol
analizinin (asim, yerlesme, kalici hata, RMS + oneri) ayni matematigi web
arayuzunden de kullanilabilsin diye ortak hale getirildi.
"""
import statistics
import threading
import time

from control.pid import angle_error_deg

MODES = ("AUTO", "HOLD", "HOVER", "RATE", "TELEOP")

# Adim testi en fazla bu kadar saniye kaydedilir (bellek sinirlamasi)
STEP_MAX_S = 45.0


class StepRecorder:
    """Bir hedef degisiminden sonraki cevabi kaydeder ve analiz eder.

    Kaydedilen: (t, olculen, hedef). Analiz pid_tune.py ile AYNI matematigi
    kullanir — havuzda konsoldan ve web'den ayni sonucu gormek onemli.
    """

    UNITS = {"depth": "m", "heading": "°", "rate": "°/s", "roll": "°", "pitch": "°"}

    def __init__(self):
        self.kind = None          # 'depth' | 'heading' | 'rate' | 'roll' | 'pitch'
        self.t0 = None
        self.rows = []            # (t, deger, hedef)
        self._lock = threading.Lock()

    def start(self, kind):
        if kind not in ("depth", "heading", "rate", "roll", "pitch"):
            raise ValueError(f"Invalid kind: {kind}")
        with self._lock:
            self.kind = kind
            self.rows = []
            self.t0 = time.monotonic()

    def stop(self):
        with self._lock:
            self.t0 = None

    def active(self):
        return self.t0 is not None

    def sample(self, snap, stab, rate_target=0.0, now=None):
        """Ana dongudan her adimda cagrilir. Kayit aktif degilse hic is yapmaz."""
        if self.t0 is None:
            return
        now = time.monotonic() if now is None else now
        t = now - self.t0
        if t > STEP_MAX_S:
            return
        if self.kind == "depth":
            row = (t, getattr(snap, "depth_m", 0.0), stab.target_depth or 0.0)
        elif self.kind == "heading":
            row = (t, getattr(snap, "heading", 0.0), stab.target_heading or 0.0)
        elif self.kind == "rate":
            row = (t, getattr(snap, "yaw_rate", 0.0), rate_target)
        elif self.kind == "roll":
            row = (t, getattr(snap, "roll", 0.0), 0.0)
        elif self.kind == "pitch":
            row = (t, getattr(snap, "pitch", 0.0), 0.0)
        else:
            return
        with self._lock:
            self.rows.append(row)

    def series(self, max_points=240):
        """Grafik icin seyreltilmis (t, olculen, hedef) dizisi."""
        with self._lock:
            rows = list(self.rows)
        if not rows:
            return []
        step = max(1, len(rows) // max_points)
        return [[round(r[0], 2), round(r[1], 4), round(r[2], 4)]
                for r in rows[::step]]

    # ------------------------------------------------------------- analiz
    def analyze(self):
        """Asim / yerlesme / kalici hata / RMS + 'ne yapmali' onerisi."""
        with self._lock:
            rows = list(self.rows)
        kind = self.kind
        if len(rows) < 5:
            return {"ok": False, "error": "Yeterli veri yok — önce bir adım "
                                          "testi başlat ve birkaç saniye bekle."}

        t = [r[0] for r in rows]
        v = [r[1] for r in rows]
        hedef = rows[-1][2]
        v0 = v[0]

        if kind == "heading":
            # Aci sarmalini duzelt (359 -> 1 gecisi 358 derecelik sicrama degildir)
            err = [angle_error_deg(hedef, x) for x in v]
            v = [hedef - e for e in err]
            v0 = v[0]

        genlik = hedef - v0
        if abs(genlik) < 1e-6:
            return {"ok": False, "error": "Adım genliği sıfır — anlamlı analiz yok. "
                                          "Mevcut değerden FARKLI bir hedef ver."}

        # asim (overshoot)
        uc = max(v) if genlik > 0 else min(v)
        asim_pct = max(0.0, (uc - hedef) / genlik * 100.0)
        asim_abs = abs(uc - hedef)

        # yerlesme suresi: %5 bandina girip bir daha cikmadigi ilk an (O(N) algoritma)
        band = abs(genlik) * 0.05
        yerlesme = None
        for i in range(len(v) - 1, -1, -1):
            if abs(v[i] - hedef) > band:
                if i < len(v) - 1:
                    yerlesme = t[i+1]
                break
        else:
            if len(v) > 0:
                yerlesme = t[0]

        # kalici hata + gurultu (son 5 sn)
        son = [x for tt, x in zip(t, v) if tt >= t[-1] - 5.0]
        kalici = (sum(son) / len(son)) - hedef if son else 0.0
        rms = statistics.pstdev(son) if len(son) > 1 else 0.0

        return {
            "ok": True,
            "kind": kind,
            "unit": self.UNITS.get(kind, ""),
            "baslangic": round(v0, 3),
            "hedef": round(hedef, 3),
            "sure_s": round(t[-1], 1),
            "asim_pct": round(asim_pct, 1),
            "asim_abs": round(asim_abs, 3),
            "yerlesme_s": None if yerlesme is None else round(yerlesme, 2),
            "kalici": round(kalici, 4),
            "rms": round(rms, 4),
            "oneri": self._advice(kind, asim_abs, yerlesme, kalici, rms),
        }

    @staticmethod
    def _advice(kind, asim, yerlesme, kalici, rms):
        """Havuz kenarinda dusunmeye vakit yok — ne yapilacagini soyle."""
        o = []
        if kind == "depth":
            if asim > 0.15:
                o.append("Aşım büyük (>15 cm): Kd'yi artır, sonra Kp'yi biraz düşür.")
            if yerlesme is None or yerlesme > 6.0:
                o.append("Yavaş oturuyor: Kp'yi artır — ya da FF (asılı kalma gücü) eksik.")
            if abs(kalici) > 0.03:
                o.append("Kalıcı hata var (>3 cm): Ki'yi artır ya da FF'i düzelt.")
            if rms > 0.05:
                o.append("Titreşim var: Kd'yi düşür ya da d_tau'yu büyüt.")
        elif kind == "heading":
            if asim > 8.0:
                o.append("Aşım büyük (>8°): dış katman kp_pos'u düşür ya da w_max'ı kıs.")
            if yerlesme is None or yerlesme > 8.0:
                o.append("Yavaş dönüyor: kp_pos'u ya da iç döngü Kp'sini artır.")
            if abs(kalici) > 3.0:
                o.append("Kalıcı sapma (>3°): iç döngü Ki'sini artır (akıntı/motor asimetrisi).")
            if rms > 2.0:
                o.append("Yön oynuyor: iç döngü Kd'sini düşür, d_tau'yu büyüt.")
        elif kind == "rate":
            if abs(kalici) > 3.0:
                o.append("Hedef dönüş hızına ulaşılamıyor: i_limit'i out_limit'e yaklaştır.")
            if rms > 3.0:
                o.append("Dönüş hızı gürültülü: Kd'yi düşür.")
            o.append("Log'da yaw_sat sütunu daire boyunca 0 kalmalı — doyuyorsa "
                     "'circle' modunun out_limit'ini artır.")
        elif kind in ("roll", "pitch"):
            if asim > 5.0:
                o.append(f"Aşım büyük (>5°): {kind} Kd'sini artır, Kp'yi düşür.")
            if abs(kalici) > 2.0:
                o.append(f"Kalıcı hata var (>2°): ağırlık merkezini (mekanik trim) kontrol et, sonra Ki eklenebilir.")
            if rms > 1.5:
                o.append("Çok fazla gürültü/titreme var: Kd'yi düşür, config.py'de ROLL_PITCH_FILTER_ALPHA'yı artır (ör: 0.99 veya 0.995).")
        if not o:
            o.append("Sonuçlar makul görünüyor. Değerleri config.py'a işlemeyi unutma.")
        return o


class Operator:
    """Yer istasyonunun kontrol otoritesi. Thread-safe."""

    def __init__(self):
        self._lock = threading.Lock()
        self.mode = "AUTO"
        self.hover_cmd = 0.0      # HOVER modunda sabit dikey gaz (-1..1)
        self.surge = 0.0          # HOLD/RATE modunda ileri gaz (0..1)
        self.rate_target = 0.0    # RATE modunda donus hizi hedefi (derece/sn)
        self.teleop_axes = None
        self.recorder = StepRecorder()
        self.last_axes = dict(surge=0.0, yaw=0.0, heave=0.0, roll=0.0, pitch=0.0)
        self.note = ""            # arayuzde gosterilecek son islem aciklamasi

    # --------------------------------------------------------------- mod
    def set_mode(self, mode, stab=None):
        """Mod degistirir.

        AUTO disindaki bir moda gecerken, hedef verilmemisse ARACIN SU ANKI
        derinligi/yonu hedef yapilir. Boylece 'HOLD'a basinca arac oldugu
        yerde kalir — eski gorev hedefine dogru firlamaz.
        """
        mode = (mode or "").upper()
        if mode not in MODES:
            raise ValueError(f"Bilinmeyen mod: {mode}")
        with self._lock:
            onceki = self.mode
            self.mode = mode

        if mode in ("HOLD", "RATE") and stab is not None and onceki == "AUTO":
            # "Burada kal": mevcut durumu hedef yap
            stab.set_targets(depth_m=round(getattr(stab, "depth_m", 0.0), 2),
                             heading_deg=round(getattr(stab, "heading_deg", 0.0), 1))
        if mode == "HOVER":
            with self._lock:
                self.hover_cmd = 0.0
        if mode != onceki:
            self.note = f"{onceki} → {mode}"
        return mode

    def get(self):
        with self._lock:
            return dict(mode=self.mode, hover_cmd=self.hover_cmd,
                        surge=self.surge, rate_target=self.rate_target,
                        teleop=self.teleop_axes, note=self.note)

    def set(self, **kw):
        with self._lock:
            for k in ("hover_cmd", "surge", "rate_target"):
                if k in kw and kw[k] is not None:
                    setattr(self, k, float(kw[k]))
            if "teleop_axes" in kw:
                self.teleop_axes = kw["teleop_axes"]

    # ------------------------------------------------------------- adim
    def step(self, stab, mission, thr, mix, now=None):
        """Ana dongunun bir adimi.

        Donus: (axes, mission_done, durum_adi)
          axes         : mixer'a verilen eksen sozlugu
          mission_done : sadece AUTO modunda gorev bitti bilgisi
          durum_adi    : telemetride gosterilecek durum
        """
        with self._lock:
            mode = self.mode
            hover = self.hover_cmd
            surge = self.surge
            rate_t = self.rate_target
            teleop = self.teleop_axes

        done = False

        if mode == "TELEOP" or (teleop and mode != "AUTO"):
            axes = dict(surge=0.0, yaw=0.0, heave=0.0, roll=0.0, pitch=0.0)
            if teleop:
                axes.update({k: float(v) for k, v in teleop.items() if k in axes})
            thr.command(mix(**axes))
            durum = "TELEOP"

        elif mode == "HOVER":
            # Derinlik PID'i devre disi — SABIT dikey gaz. FF_HOVER olcumu:
            # arac ne cikiyor ne iniyorsa o deger FF_HOVER'dir.
            axes = dict(surge=surge, yaw=0.0, heave=hover, roll=0.0, pitch=0.0)
            thr.command(mix(**axes))
            durum = "HOVER"

        elif mode == "RATE":
            axes = stab.compute(surge=surge, yaw_rate_target=rate_t, resample=False)
            thr.command(mix(**axes))
            durum = "RATE"

        elif mode == "HOLD":
            # Gorev DURDURULDU; hedefleri operator veriyor, PID tutuyor.
            axes = stab.compute(surge=surge, resample=False)
            thr.command(mix(**axes))
            durum = "HOLD"

        else:  # AUTO — gorev hedefleri yonetir (eski davranis)
            axes = None
            if mission is not None:
                done = bool(mission.step())
                axes = getattr(mission, "last_axes", None)
            durum = getattr(mission, "state", "AUTO") if mission else "IDLE"

        if axes:
            self.last_axes = dict(axes)
        return self.last_axes, done, durum
