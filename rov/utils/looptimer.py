"""
Sabit frekansli dongu zamanlayici — YENI DOSYA.

NEDEN VAR (SORUN 2'nin ucuncu parcasi):
  Eski kod her dongu sonunda soyle yapiyordu:
      time.sleep(1.0 / LOOP_HZ)
  Bu YANLIS. Cunku hesaplama suresini HESABA KATMIYOR:
      gercek periyot = hesaplama_suresi + 1/LOOP_HZ
  Hesaplama 100 ms surduyse dongu 20 ms degil 120 ms olur ve kimse fark etmez.

  Dogrusu "son teslim tarihi" (deadline) mantigidir:
      bir sonraki adim = onceki adim + periyot
      uyu(bir sonraki adim - simdi)
  Boylece hesaplama ne kadar surerse sursun ORTALAMA frekans korunur.

AYRICA:
  - `hz` ile gercek frekansi olcer (kabul kriteri H1: >= 30 Hz)
  - Hedefin altina duserse UYARI basar; sessizce yavaslamayi engeller
  - Cok geri kalinirsa saati sifirlar (birikmis gecikmeyi kovalayip
    art arda sifir-uykulu dongulere girmesin)
"""
import time


class LoopTimer:
    def __init__(self, hz, warn_hz=None, name="loop", warn_period_s=5.0):
        self.period = 1.0 / float(hz)
        self.target_hz = float(hz)
        self.warn_hz = warn_hz
        self.name = name
        self.warn_period_s = warn_period_s
        self._next = None
        self._prev = None
        self.dt = 0.0
        self.hz = 0.0          # yumusatilmis olculen frekans
        self.worst_dt = 0.0
        self.stalls = 0        # 1 sn'yi asan tek seferlik duraklamalar
        self.count = 0
        self._last_warn = 0.0

    def tick(self):
        """Dongu BASINDA cagrilir: gecen sureyi olcer, self.dt/self.hz gunceller."""
        now = time.monotonic()
        if self._prev is not None:
            self.dt = now - self._prev
            # 1 sn'yi asan adimlar dongu performansi degildir; bunlar
            # ESC arm (time.sleep(2.0)) gibi tek seferlik bloklamalardir.
            # Ayri sayilir ki "en kotu adim" istatistigini bozmasin.
            if self.dt > 1.0:
                self.stalls += 1
            else:
                self.worst_dt = max(self.worst_dt, self.dt)
            inst = 1.0 / max(1e-6, self.dt)
            self.hz = inst if self.hz == 0.0 else self.hz + 0.05 * (inst - self.hz)
        self._prev = now
        self.count += 1
        return now

    def sleep(self):
        """Dongu SONUNDA cagrilir: bir sonraki adima kadar uyur."""
        now = time.monotonic()
        if self._next is None:
            self._next = now
        self._next += self.period
        gecikme = self._next - now
        if gecikme > 0:
            time.sleep(gecikme)
        elif gecikme < -0.5:
            # Cok geri kaldik: birikmis gecikmeyi kovalamak yerine saati sifirla.
            self._next = time.monotonic()

        if self.warn_hz and self.hz and self.hz < self.warn_hz:
            t = time.monotonic()
            if t - self._last_warn > self.warn_period_s:
                self._last_warn = t
                print(f"[{self.name}] UYARI: dongu {self.hz:.1f} Hz "
                      f"(hedef {self.target_hz:.0f} Hz, esik {self.warn_hz:.0f} Hz). "
                      f"En kotu adim {self.worst_dt*1000:.0f} ms.")

    def report(self):
        ek = f", {self.stalls} tek-seferlik duraklama (ESC arm vb.)" if self.stalls else ""
        return (f"{self.name}: {self.count} adim, olculen {self.hz:.1f} Hz "
                f"(hedef {self.target_hz:.0f} Hz), en kotu adim "
                f"{self.worst_dt*1000:.0f} ms{ek}")
