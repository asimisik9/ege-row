"""
E-stop GPIO izleyicisi — manyetik acil durdurma.

Jetson 40-pin header Pin 13 (BCM 27) uzerindeki Hall sensoru #2'yi izler.
Miknatisla tetiklendiginde (pin HIGH olur) Thrusters.stop() aninda cagirilir
ve threading.Event ile gorev dongusune sinyal gonderilir.

Jetson.GPIO kutuphanesi yoksa (gelistirme PC) sessizce devre disi birakilir.
"""
import threading
import time

try:
    import Jetson.GPIO as GPIO
    _GPIO_OK = True
except ImportError:
    try:
        import RPi.GPIO as GPIO   # Raspberry Pi yedek
        _GPIO_OK = True
    except ImportError:
        GPIO = None
        _GPIO_OK = False

from config import ESOP_GPIO_BCM, ESTOP_BOUNCE_MS


class EStopMonitor:
    """
    Arka planda calisan Hall sensoru izleyicisi.

    Kullanim:
        estop = EStopMonitor(thrusters)
        estop.start()
        ...
        if estop.triggered.is_set():
            raise SystemExit("E-STOP devrede!")
        ...
        estop.stop()
    """

    def __init__(self, thrusters):
        """thrusters: Thrusters nesnesi — tetiklendiginde stop() cagirilir."""
        self.thr = thrusters
        self.triggered = threading.Event()
        self._active = False

    # ---------------------------------------------------------------- public
    def start(self):
        """GPIO izlemeyi baslat. Donanim yoksa uyari yaz, devam et."""
        if not _GPIO_OK:
            print("[E-STOP] Jetson.GPIO yok — yazilim e-stop devre disi "
                  "(simulasyon / gelistirme modunda normal).")
            return

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ESOP_GPIO_BCM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # Kesme tabanli algilama — dongulere gerek yok
        GPIO.add_event_detect(
            ESOP_GPIO_BCM,
            GPIO.RISING,
            callback=self._on_trigger,
            bouncetime=ESTOP_BOUNCE_MS,
        )
        self._active = True
        print(f"[E-STOP] BCM {ESOP_GPIO_BCM} izleniyor. "
              "Miknatisi yaklastir → tum motorlar durur.")

    def stop(self):
        """GPIO temizligi. Program kapatilirken cagir."""
        if _GPIO_OK and self._active:
            GPIO.cleanup(ESOP_GPIO_BCM)
            self._active = False

    def simulate_trigger(self):
        """Test icin yazilimsal e-stop tetikleme (donanim gerektirmez)."""
        self._on_trigger(ESOP_GPIO_BCM)

    # ---------------------------------------------------------------- private
    def _on_trigger(self, channel):
        """GPIO kesme geri cagrimi (veya yazilimsal tetik)."""
        print("\n[E-STOP] *** ACIL DURDURMA AKTİF *** — tum motorlar notrde!")
        self.thr.stop()          # derhal motorlari durdur
        self.triggered.set()     # ana dongune bildir
