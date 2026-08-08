#!/usr/bin/env python3
"""
HAVUZ ICIN CANLI PID AYAR KONSOLU  (YENI DOSYA — eski pid_test_cal.py'nin yerine)

==============================================================================
NEDEN VAR
==============================================================================
Havuzda 3 saatimiz var. Her katsayi denemesi icin kodu durdurup dosyayi
duzenleyip yeniden baslatmak = deneme basina 2-3 dakika kayip = gunun yarisi.
Bu konsol calisirken katsayilari DEGISTIRIYOR, adim testi yapiyor ve
sonucu (asim, yerlesme suresi, kalici hata) OTOMATIK olcuyor.

Ayrica eski pid_test_cal.py'nin iki sorunu vardi:
  - Sensoru kendi dongusunde bloklyarak okuyordu (SORUN 2)
  - Ki'yi dogrudan atiyordu, I birikimini sifirlamiyordu -> sonuc yaniltici

==============================================================================
KOMUTLAR
==============================================================================
  zero                 derinlik referansini yuzeye gore sifirla (SUYA GIRINCE ILK IS)
  arm / disarm         motorlari hazirla / notre cek
  hover <0..1>         PID KAPALI, sabit dikey gaz. FF_HOVER olcumu icin (Adim 2).
                       Arac ne cikiyor ne iniyorsa o deger FF_HOVER'dir.
  depth <m>            derinlik hedefi ver (PID acik)          -> adim testi baslar
  head  <derece>       yon hedefi ver (mutlak)                 -> adim testi baslar
  head  +90 / -90      mevcut yone gore goreli hedef
  rate  <dps>          sabit donus hizi hedefi (DAIRE ayari icin)
  surge <0..1>         ileri gaz (duz segment / daire testi)
  mode cruise|turn     yon kontrolu modu
  axis depth|rate|pos|roll|pitch     hangi eksenin katsayilari degisecek
  kp/ki/kd/ff <deger>  secili eksenin katsayisini degistir (I birikimi sifirlanir)
  show                 mevcut katsayilar
  analyze              son adim testinin sonucu (asim / yerlesme / kalici hata)
  sweep kp 0.5 3.0 5   kp'yi 5 kademede tarar, her kademede adim testi yapar
  stop                 motorlari notre cek (hedefleri korur)
  save                 katsayilari config.py'ye yaz (once .bak alinir)
  quit

TIPIK HAVUZ AKISI (protokol §7):
  zero -> arm -> hover 0.15 / 0.20 / 0.25 ...  (FF_HOVER'i bul)
  -> ff <bulunan>  -> axis depth -> kp 1.0 -> depth 0.6 -> analyze
  -> kp 1.5 -> depth 0.6 -> analyze ... (salinim basladigi Kp'nin %60'i)
  -> kd ... -> ki ...  -> save
  -> axis rate -> rate 20 -> analyze ...   (yon ic dongusu)
  -> axis pos  -> head +90 -> analyze ...  (yon dis dongusu)
"""
import re
import shutil
import statistics
import sys
import threading
import time

import config
from config import LOOP_HZ, MISSION
from control.mixer import mix
from control.pid import angle_error_deg
from utils.logger import MissionLogger
from utils.looptimer import LoopTimer


# ---------------------------------------------------------------- config yazma
def _dict_blogunu_degistir(text, isim, yeni_govde):
    """`ISIM = dict( ... )` blogunu bulur ve icerigini degistirir."""
    m = re.search(rf"^{isim}\s*=\s*dict\(", text, re.M)
    if not m:
        return None
    i = text.index("(", m.start())
    derinlik = 0
    for j in range(i, len(text)):
        if text[j] == "(":
            derinlik += 1
        elif text[j] == ")":
            derinlik -= 1
            if derinlik == 0:
                return text[:m.start()] + f"{isim} = dict({yeni_govde})" + text[j + 1:]
    return None


def config_kaydet(stab, path="config.py"):
    """Mevcut katsayilari config.py'ye yazar. Once .bak yedegi alinir."""
    shutil.copy(path, path + ".bak")
    with open(path, encoding="utf-8") as f:
        text = f.read()

    d = stab.pid_depth
    govde = (f"\n    kp={d.kp:.4g}, ki={d.ki:.4g}, kd={d.kd:.4g},"
             f"\n    out_limit={d.out_limit:.4g}, i_limit={d.i_limit:.4g},"
             f"\n    d_tau={d.d_tau:.4g}, deadzone={d.deadzone:.4g}, ff={d.ff:.4g},\n")
    yeni = _dict_blogunu_degistir(text, "PID_DEPTH", govde)
    if yeni:
        text = yeni

    text = re.sub(r"^FF_HOVER\s*=.*$",
                  f"FF_HOVER = {d.ff:.4g}               # pid_tune.py ile OLCULDU",
                  text, count=1, flags=re.M)

    hc = stab.pid_heading
    yeni = _dict_blogunu_degistir(text, "HEADING_POS", f"\n    kp={hc.kp_pos:.4g},\n")
    if yeni:
        text = yeni
    r = hc.rate
    govde = (f"\n    kp={r.kp:.4g}, ki={r.ki:.4g}, kd={r.kd:.4g},"
             f"\n    i_limit={r.i_limit:.4g}, d_tau={r.d_tau:.4g},\n")
    yeni = _dict_blogunu_degistir(text, "HEADING_RATE", govde)
    if yeni:
        text = yeni

    for isim, p in (("PID_ROLL", stab.pid_roll), ("PID_PITCH", stab.pid_pitch)):
        govde = (f"kp={p.kp:.4g}, ki={p.ki:.4g}, kd={p.kd:.4g}, "
                 f"out_limit={p.out_limit:.4g},\n                 "
                 f"i_limit={p.i_limit:.4g}, d_tau={p.d_tau:.4g}, "
                 f"deadzone={p.deadzone:.4g}, ff={p.ff:.4g}")
        yeni = _dict_blogunu_degistir(text, isim, govde)
        if yeni:
            text = yeni

    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"[OK] config.py guncellendi (yedek: {path}.bak)")


# ---------------------------------------------------------------- ayar konsolu
class Tuner:
    def __init__(self):
        from main import build_system
        from control.stabilizer import Stabilizer

        self.thr, self.ori, self.depth, self.hub = build_system()
        self.stab = Stabilizer(self.ori, self.depth, state=self.hub.state)
        self.log = MissionLogger("pid_tune")

        self.mode = "idle"          # idle | hover | pid | rate
        self.hover_cmd = 0.0
        self.surge = 0.0
        self.rate_target = 0.0
        self.axis = "depth"
        self.running = threading.Event()
        self.running.set()

        # adim testi kaydi
        self.rec_t0 = None
        self.rec = []               # (t, deger, hedef)
        self.rec_kind = None        # 'depth' | 'heading' | 'rate'

        self.lt = LoopTimer(LOOP_HZ, warn_hz=None, name="tune")
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ kontrol dongusu
    def _loop(self):
        while self.running.is_set():
            self.lt.tick()
            try:
                s = self.stab.sample()

                if self.mode == "idle":
                    self.thr.command(mix(0.0, 0.0, 0.0))
                    axes = dict(surge=0.0, yaw=0.0, heave=0.0, roll=0.0, pitch=0.0)

                elif self.mode == "hover":
                    # PID KAPALI — sadece sabit dikey gaz. FF_HOVER olcumu.
                    axes = dict(surge=self.surge, yaw=0.0, heave=self.hover_cmd,
                                roll=0.0, pitch=0.0)
                    self.thr.command(mix(**axes))

                elif self.mode == "rate":
                    axes = self.stab.compute(surge=self.surge,
                                             yaw_rate_target=self.rate_target,
                                             resample=False)
                    self.thr.command(mix(**axes))

                else:   # pid
                    axes = self.stab.compute(surge=self.surge, resample=False)
                    self.thr.command(mix(**axes))

                self._kaydet(s)
                self.log.sample(self.mode.upper(), self.stab, axes, self.thr)
            except Exception as e:
                print(f"\n[DONGU HATASI] {e}")
            self.lt.sleep()

    def _kaydet(self, s):
        if self.rec_t0 is None:
            return
        t = time.monotonic() - self.rec_t0
        if t > 40.0:
            return
        if self.rec_kind == "depth":
            self.rec.append((t, s.depth_m, self.stab.target_depth or 0.0))
        elif self.rec_kind == "heading":
            self.rec.append((t, s.heading, self.stab.target_heading or 0.0))
        elif self.rec_kind == "rate":
            self.rec.append((t, s.yaw_rate, self.rate_target))

    def _test_basla(self, kind):
        self.rec_kind = kind
        self.rec = []
        self.rec_t0 = time.monotonic()

    # ------------------------------------------------------------ adim analizi
    def analyze(self, sessiz=False):
        if not self.rec:
            print("Once bir adim testi baslat (depth / head / rate).")
            return None
        kind = self.rec_kind
        t = [r[0] for r in self.rec]
        v = [r[1] for r in self.rec]
        hedef = self.rec[-1][2]
        v0 = v[0]

        if kind == "heading":
            # aci sarmalini duzelt: hedefe gore isaretli hata dizisi
            err = [angle_error_deg(hedef, x) for x in v]
            v = [hedef - e for e in err]
            v0 = v[0]

        genlik = hedef - v0
        if abs(genlik) < 1e-6:
            print("Adim genligi sifir — anlamli analiz yok.")
            return None

        # asim (overshoot)
        uc = max(v) if genlik > 0 else min(v)
        asim = (uc - hedef) / genlik * 100.0
        asim = max(0.0, asim)
        asim_mutlak = abs(uc - hedef)

        # yerlesme suresi: son %5 bandina girip cikmadigi ilk an
        band = abs(genlik) * 0.05
        yerlesme = None
        for i in range(len(v)):
            if all(abs(x - hedef) <= band for x in v[i:]):
                yerlesme = t[i]
                break

        # kalici hata + RMS (son 5 sn)
        son = [x for tt, x in zip(t, v) if tt >= t[-1] - 5.0]
        kalici = (sum(son) / len(son)) - hedef if son else float("nan")
        rms = (statistics.pstdev(son) if len(son) > 1 else 0.0)

        birim = {"depth": "m", "heading": "deg", "rate": "dps"}[kind]
        if not sessiz:
            print("\n" + "-" * 62)
            print(f"  ADIM YANITI ANALIZI  ({kind}, {v0:.3f} -> {hedef:.3f} {birim})")
            print("-" * 62)
            print(f"  Asim (overshoot) : {asim:5.1f} %   ({asim_mutlak:.3f} {birim})")
            if yerlesme is None:
                print(f"  Yerlesme (%5)    : OLUSMADI  "
                      f"(banda hic girmedi ya da kalici hata bandi asiyor: "
                      f"band +-{abs(genlik)*0.05:.3f} {birim})")
            else:
                print(f"  Yerlesme (%5)    : {yerlesme:5.2f} s")
            print(f"  Kalici hata      : {kalici:+.3f} {birim}")
            print(f"  Son 5 sn RMS     : {rms:.3f} {birim}")
            print(f"  Dongu            : {self.lt.hz:.1f} Hz")
            self._yorum(kind, asim_mutlak, yerlesme, kalici, rms)
            print("-" * 62)
        return dict(asim=asim, asim_mutlak=asim_mutlak, yerlesme=yerlesme,
                    kalici=kalici, rms=rms)

    @staticmethod
    def _yorum(kind, asim, yerlesme, kalici, rms):
        """Ne yapmali? — havuzda dusunmeye vakit olmadigi icin."""
        oneri = []
        if kind == "depth":
            if asim > 0.15:
                oneri.append("Asim buyuk (>15 cm): Kd'yi artir, sonra Kp'yi biraz dusur.")
            if yerlesme is None or (yerlesme and yerlesme > 6.0):
                oneri.append("Yavas oturuyor: Kp'yi artir (ya da FF eksik olabilir).")
            if abs(kalici) > 0.03:
                oneri.append("Kalici hata var (>3 cm): Ki'yi artir ya da FF'i duzelt.")
            if rms > 0.05:
                oneri.append("Salinim/gurultu (>5 cm RMS): Kp ya da Kd fazla; "
                             "d_tau'yu buyutmeyi dene.")
            if not oneri:
                oneri.append("H2/H3 kabul kriterlerini sagliyor gorunuyor. SAVE et.")
        elif kind == "heading":
            if asim > 8.0:
                oneri.append("Asim buyuk (>8 deg): HEADING_POS kp'yi dusur ya da "
                             "w_max'i dusur (mode turn).")
            if abs(kalici) > 2.0:
                oneri.append("Kalici sapma (>2 deg): ic donguye Ki ekle/artir.")
            if rms > 3.0:
                oneri.append("Salinim (>3 deg RMS): ic dongu kp'sini dusur.")
            if not oneri:
                oneri.append("H6 kabul kriterini sagliyor gorunuyor.")
        else:
            if abs(kalici) > 3.0:
                oneri.append("Donus hizi hedefi tutmuyor (>3 dps): ic dongu Ki'yi artir.")
            if rms > 3.0:
                oneri.append("Titriyor: ic dongu kp'yi dusur ya da d_tau'yu buyut.")
            if not oneri:
                oneri.append("Ic dongu saglam. Dis donguye gec (axis pos).")
        for o in oneri:
            print("  ONERI: " + o)

    # ------------------------------------------------------------- yardimcilar
    def secili(self):
        return {"depth": self.stab.pid_depth,
                "rate": self.stab.pid_heading.rate,
                "roll": self.stab.pid_roll,
                "pitch": self.stab.pid_pitch}.get(self.axis)

    def show(self):
        s = self.stab
        print("\n" + "=" * 62)
        print(f"  SECILI EKSEN: {self.axis}      MOD: {self.mode}      "
              f"dongu {self.lt.hz:.1f} Hz")
        print("-" * 62)
        d = s.pid_depth
        print(f"  DEPTH  kp={d.kp:<7.4g} ki={d.ki:<7.4g} kd={d.kd:<7.4g} "
              f"ff={d.ff:<6.4g} i_lim={d.i_limit}")
        print(f"  POS    kp={s.pid_heading.kp_pos:<7.4g} "
              f"w_max={s.pid_heading.w_max:.4g} dps  mod={s.pid_heading.mode}")
        r = s.pid_heading.rate
        print(f"  RATE   kp={r.kp:<7.4g} ki={r.ki:<7.4g} kd={r.kd:<7.4g} "
              f"out_lim={r.out_limit}")
        print(f"  ROLL   kp={s.pid_roll.kp:<7.4g} kd={s.pid_roll.kd:<7.4g} "
              f"deadzone={s.pid_roll.deadzone}")
        print(f"  PITCH  kp={s.pid_pitch.kp:<7.4g} kd={s.pid_pitch.kd:<7.4g} "
              f"deadzone={s.pid_pitch.deadzone}")
        snap = s.snap
        if snap:
            print("-" * 62)
            print(f"  derinlik {s.depth_m:+.3f} m (hedef {s.target_depth})   "
                  f"dikey hiz {snap.depth_rate_mps:+.3f} m/s")
            print(f"  yon      {s.heading_deg:6.1f} deg (hedef {s.target_heading})  "
                  f"donus hizi {snap.yaw_rate:+.1f} dps")
            print(f"  roll {snap.roll:+.1f}  pitch {snap.pitch:+.1f}  "
                  f"| imu {snap.imu_hz:.0f} Hz  derinlik {snap.depth_hz:.0f} Hz")
        print("=" * 62)

    def kapat(self):
        self.running.clear()
        self.thread.join(timeout=2.0)
        self.thr.stop()
        self.hub.stop()
        self.log.close()
        print(f"\nLog: {self.log.path}")
        print(f"Analiz: python3 tools/analyze_log.py {self.log.path}")


# ---------------------------------------------------------------------- konsol
def main():
    print("=" * 74)
    print("        EGE ROV — CANLI PID AYAR KONSOLU")
    print("=" * 74)
    print(__doc__.split("KOMUTLAR")[1].split("TIPIK")[0])

    t = Tuner()
    t.show()
    try:
        while True:
            try:
                satir = input("tune> ").strip()
            except EOFError:
                break
            if not satir:
                continue
            p = satir.split()
            k = p[0].lower()
            arg = p[1] if len(p) > 1 else None

            try:
                if k in ("quit", "exit", "q"):
                    break
                elif k == "zero":
                    t.mode = "idle"
                    t.stab.depth.zero_at_surface()
                elif k == "arm":
                    t.thr.arm()
                    print("[OK] motorlar armed")
                elif k == "disarm" or k == "stop":
                    t.mode = "idle"
                    t.surge = 0.0
                    t.thr.stop()
                    print("[OK] motorlar notr")
                elif k == "hover":
                    t.hover_cmd = float(arg)
                    t.mode = "hover"
                    t.rec_t0 = None
                    print(f"[HOVER] PID KAPALI, sabit dikey gaz = {t.hover_cmd:.3f}")
                    print("  Arac ne cikiyor ne iniyorsa bu deger FF_HOVER'dir.")
                    print("  Izle: 'show'.  Bulunca: 'axis depth' + 'ff <deger>'")
                elif k == "depth":
                    t.stab.set_targets(depth_m=float(arg))
                    t.mode = "pid"
                    t._test_basla("depth")
                    print(f"[ADIM] derinlik hedefi {arg} m — 'analyze' ile sonucu gor")
                elif k == "head":
                    if arg.startswith(("+", "-")):
                        hedef = (t.stab.heading_deg + float(arg)) % 360.0
                    else:
                        hedef = float(arg) % 360.0
                    t.stab.set_targets(heading_deg=hedef)
                    t.mode = "pid"
                    t._test_basla("heading")
                    print(f"[ADIM] yon hedefi {hedef:.1f} deg")
                elif k == "rate":
                    t.rate_target = float(arg)
                    # Daire yetkisine gec: 'cruise' modunun out_limit=0.35
                    # yetkisi ile yuksek donus hizi hedefleri ULASILAMAZ ve
                    # ic dongu doyar (simulasyonda bu bulundu). Daire ayari
                    # yaparken gercekci yetkiyle olcmek sart.
                    t.stab.set_heading_mode("circle")
                    t.mode = "rate"
                    t._test_basla("rate")
                    print(f"[ADIM] donus hizi hedefi {t.rate_target} dps")
                    if t.surge > 0:
                        cap = 2.0 * (t.surge * 0.8) / max(1e-3, abs(t.rate_target)) * 57.3
                        print(f"  (kaba tahmin: surge={t.surge} ile cap ~{cap:.2f} m)")
                elif k == "surge":
                    t.surge = float(arg)
                    print(f"[OK] ileri gaz = {t.surge}")
                elif k == "mode":
                    t.stab.set_heading_mode(arg)
                    print(f"[OK] yon modu = {arg}")
                elif k == "axis":
                    if arg not in ("depth", "rate", "pos", "roll", "pitch"):
                        print("axis depth|rate|pos|roll|pitch")
                    else:
                        t.axis = arg
                        print(f"[OK] secili eksen = {arg}")
                elif k in ("kp", "ki", "kd", "ff"):
                    v = float(arg)
                    if t.axis == "pos":
                        if k != "kp":
                            print("dis dongude sadece kp var.")
                        else:
                            t.stab.pid_heading.set_params(kp_pos=v)
                            print(f"[OK] HEADING_POS kp = {v}")
                    else:
                        t.secili().set_params(**{k: v})   # reset=True: I sifirlanir
                        print(f"[OK] {t.axis}.{k} = {v}   (I birikimi sifirlandi)")
                elif k == "show":
                    t.show()
                elif k == "analyze":
                    t.analyze()
                elif k == "sweep":
                    ad, bas, bit, n = p[1], float(p[2]), float(p[3]), int(p[4])
                    hedef = float(p[5]) if len(p) > 5 else 0.6
                    for i in range(n):
                        v = bas + (bit - bas) * i / max(1, n - 1)
                        t.secili().set_params(**{ad: v})
                        print(f"\n### {t.axis}.{ad} = {v:.4g}")
                        t.stab.set_targets(depth_m=0.0 if t.axis == "depth" else None)
                        time.sleep(4.0)
                        t.stab.set_targets(depth_m=hedef)
                        t.mode = "pid"
                        t._test_basla("depth")
                        time.sleep(12.0)
                        t.analyze()
                elif k == "save":
                    config_kaydet(t.stab)
                else:
                    print("Bilinmeyen komut. Yardim icin dosyanin basindaki listeye bak.")
            except (TypeError, ValueError, IndexError) as e:
                print(f"[ARGUMAN HATASI] {e}")
            except Exception as e:
                print(f"[HATA] {e}")
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        t.kapat()


if __name__ == "__main__":
    sys.exit(main())
