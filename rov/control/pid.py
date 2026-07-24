"""Genel amacli PID denetleyici (anti-windup + turev filtreli)."""
import time


class PID:
    def __init__(self, kp, ki, kd, out_limit=1.0, i_limit=0.5, d_filter=0.2):
        """kp/ki/kd: PID katsayilari. out_limit: cikti sinirlari (+-).
        i_limit: I terimi icin anti-windup siniri. d_filter: D teriminin
        alcak geciren filtre katsayisi (gurultuyu azaltir)."""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.d_filter = d_filter  # 0..1, dusuk deger = daha cok filtre
        self.reset()

    def reset(self):
        """Ic durumu (I terimi birikimi, onceki hata/zaman) sifirlar.
        Hedef degistiginde cagirilmali (aksi halde eski I/D birikimi
        yeni hedefe sicrar)."""
        self._i = 0.0
        self._prev_err = None
        self._d = 0.0
        self._prev_t = None

    def get_params(self):
        """Mevcut PID katsayilarini dict olarak dondurur."""
        return {"kp": self.kp, "ki": self.ki, "kd": self.kd}

    def set_params(self, kp=None, ki=None, kd=None):
        """PID katsayilarini canli (online) gunceller ve I birikimini sifirlar."""
        if kp is not None: self.kp = float(kp)
        if ki is not None: self.ki = float(ki)
        if kd is not None: self.kd = float(kd)
        self.reset()

    def update(self, error, now=None):
        """Hata girisine gore -out_limit..+out_limit arasi cikti dondurur."""
        now = time.monotonic() if now is None else now
        if self._prev_t is None:
            dt = 0.0
        else:
            dt = max(1e-4, now - self._prev_t)
        self._prev_t = now

        # I terimi (anti-windup: sinirli)
        self._i += error * self.ki * dt
        self._i = max(-self.i_limit, min(self.i_limit, self._i))

        # D terimi (birinci derece filtreli)
        if self._prev_err is not None and dt > 0:
            raw_d = (error - self._prev_err) / dt
            self._d += self.d_filter * (raw_d - self._d)
        self._prev_err = error

        out = self.kp * error + self._i + self.kd * self._d
        return max(-self.out_limit, min(self.out_limit, out))


def angle_error_deg(target, current):
    """Iki heading arasi en kisa acisal fark (-180..+180 derece).
    Pozitif = saga (saat yonu) donulmeli."""
    err = (target - current + 180.0) % 360.0 - 180.0
    return err
