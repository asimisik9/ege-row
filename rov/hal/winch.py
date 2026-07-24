"""
CLS3860MED vinc servo surucu (Mini ROV konuslandirma).

PCA9685 CH6 uzerinden calisan 60kg servo.
PWM degerleri config.py'de tanimli.

Kullanim:
    winch = Winch(backend)   # PCA9685Backend veya MockBackend
    winch.hold()             # nötr konumda beklet
    winch.deploy()           # Mini ROV birak (WINCH_DEPLOY_S sn)
    winch.retract()          # Mini ROV cek (WINCH_RETRACT_S sn)
"""
import time

from config import (WINCH_CHANNEL, WINCH_NEUTRAL_US,
                    WINCH_DEPLOY_US, WINCH_RETRACT_US,
                    WINCH_DEPLOY_S, WINCH_RETRACT_S)


class Winch:
    def __init__(self, backend):
        """backend: PCA9685Backend veya MockBackend (hal/thrusters.py'den)."""
        self._backend = backend
        self._set(WINCH_NEUTRAL_US)

    # ---------------------------------------------------------------- public
    def hold(self):
        """Servo nötr konumda tut."""
        self._set(WINCH_NEUTRAL_US)

    def deploy(self):
        """Mini ROV bırak — WINCH_DEPLOY_S saniye boyunca servo sür, sonra nötr."""
        print(f"[VINC] Mini ROV bırakılıyor ({WINCH_DEPLOY_S:.1f}s)...")
        self._set(WINCH_DEPLOY_US)
        time.sleep(WINCH_DEPLOY_S)
        self._set(WINCH_NEUTRAL_US)
        print("[VINC] Bırakma tamamlandı.")

    def retract(self):
        """Mini ROV çek — WINCH_RETRACT_S saniye, sonra nötr."""
        print(f"[VINC] Mini ROV çekiliyor ({WINCH_RETRACT_S:.1f}s)...")
        self._set(WINCH_RETRACT_US)
        time.sleep(WINCH_RETRACT_S)
        self._set(WINCH_NEUTRAL_US)
        print("[VINC] Çekme tamamlandı.")

    def set_raw_us(self, us: int):
        """Doğrudan µs değeri gönder (kalibrasyon için)."""
        self._set(int(us))

    # ---------------------------------------------------------------- private
    def _set(self, us: int):
        self._backend.set_us(WINCH_CHANNEL, us)
