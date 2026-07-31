#!/usr/bin/env python3
"""
EGE ROV — 1 Dosyada Sensörsüz WASD Manuel Sürüş Scripti.

HİÇBİR SENSÖR / KÜTÜPHANE BAĞIMLILIĞI GEREKTİRMEZ!
Sadece Adafruit PCA9685 + Python standart kütüphaneleri ile çalışır.

Kullanım (Jetson):
    python3 simple_wasd.py

Tuşlar:
    W / S   : İleri / Geri (Surge)
    A / D   : Sol / Sağ Dönüş (Yaw)
    I / K   : Yüksel / Dal (Heave)
    J / L   : Sol / Sağ Yanaşma (Sway)
    SPACE   : Tüm motorları NÖTR yap
    + / -   : Gaz gücünü artır / azalt (%10 adımlarla)
    Q       : Tüm motorları durdur ve çık
"""

import sys
import time
import select

# PCA9685 Donanım Bağlantısı
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    _HW_OK = True
except ImportError:
    _HW_OK = False
    print("[UYARI] PCA9685 kütüphanesi bulunamadı! Simülasyon modunda çalışıyor.")

# ── PWM ve Kanal Ayarları (tek kaynak: config.py)
from config import (PWM_NEUTRAL_US, PWM_RANGE_US, PWM_MIN_US, PWM_MAX_US,
                    FREQ_HZ, PCA9685_REF_CLOCK_HZ, MOTOR_CHANNELS, MOTOR_DIRECTION)

NEUTRAL_US = PWM_NEUTRAL_US
MAX_SPAN_US = PWM_RANGE_US
CHANNELS = MOTOR_CHANNELS
PERIOD_US = 1_000_000 / FREQ_HZ


class ThrusterDriver:
    def __init__(self):
        self.dev = None
        if _HW_OK:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                # reference_clock_speed olmadan kart 58.1Hz'e kacar (darbeler %14 kisa).
                self.dev = PCA9685(i2c, address=0x40,
                                   reference_clock_speed=PCA9685_REF_CLOCK_HZ)
                self.dev.frequency = FREQ_HZ
                print(f"[OK] PCA9685 bağlandı (0x40, {FREQ_HZ}Hz).")
            except Exception as e:
                print(f"[HATA] PCA9685 bağlanamadı: {e}")
                self.dev = None

        self.stop_all()

    def set_us(self, ch, us_val):
        """Mikrosaniye (us) değerini PCA9685 kanallarına yazar."""
        us_val = int(max(PWM_MIN_US, min(PWM_MAX_US, us_val)))
        if self.dev:
            duty = int(us_val / PERIOD_US * 0xFFFF)
            self.dev.channels[ch].duty_cycle = duty

    def stop_all(self):
        """Tüm kanalları nötr sinyaline çeker."""
        for ch in CHANNELS.values():
            self.set_us(ch, NEUTRAL_US)

    def apply_mix(self, surge, yaw, heave, sway):
        """
        -1.0 ile +1.0 arasındaki eksen komutlarını motorlara dağıtır.
        """
        # Yatay motor mikseri (Surge + Yaw + Sway)
        h_l = surge + yaw + sway
        h_r = surge - yaw - sway

        # Dikey motor mikseri (Heave)
        v_fl = heave
        v_fr = heave
        v_rl = heave
        v_rr = heave

        # Değerleri -1..+1 ile sınırla
        cmds = {
            "H_L":  max(-1.0, min(1.0, h_l)),
            "H_R":  max(-1.0, min(1.0, h_r)),
            "V_FL": max(-1.0, min(1.0, v_fl)),
            "V_FR": max(-1.0, min(1.0, v_fr)),
            "V_RL": max(-1.0, min(1.0, v_rl)),
            "V_RR": max(-1.0, min(1.0, v_rr)),
        }

        # PWM hesapla ve kanallara yaz (config.py MOTOR_DIRECTION yon duzeltmesiyle)
        for name, ch in CHANNELS.items():
            val = cmds[name] * MOTOR_DIRECTION.get(name, 1)
            us_val = NEUTRAL_US + (val * MAX_SPAN_US)
            self.set_us(ch, us_val)


def get_key_nonblocking():
    """Non-blocking klavye tuşu okur (Linux & Windows uyumlu)."""
    if sys.platform == "win32":
        import msvcrt
        if msvcrt.kbhit():
            try:
                return msvcrt.getch().decode('utf-8').lower()
            except UnicodeDecodeError:
                return ""
        return ""
    else:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist:
                return sys.stdin.read(1).lower()
            return ""
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    print("=" * 60)
    print("      EGE ROV — SENSÖRSÜZ TEK DOSYA WASD SÜRÜŞÜ")
    print("=" * 60)

    drv = ThrusterDriver()

    print(f"\nTüm kanallar NÖTR sinyalinde ({NEUTRAL_US}us).")
    input("--> ESC'lere Güç Ver, Bip Sesleri Bitince ENTER'a Bas...")
    
    print("\n[ARM] ESC'ler arm ediliyor (2 sn nötr bekleniyor)...")
    drv.stop_all()
    time.sleep(2.0)
    print("[ARM] TAMAMLANDI! Sürüşe başlayabilirsiniz.\n")

    print("-" * 60)
    print(" Tuşlar:")
    print("   W / S : İleri / Geri")
    print("   A / D : Sol / Sağ Dönüş")
    print("   I / K : Yüksel / Dal")
    print("   J / L : Yana Kayma (Sway)")
    print("   SPACE : Motorları Durdur (Nötr)")
    print("   + / - : Maksimum Gaz Limiti Artır/Azalt")
    print("   Q     : Çıkış")
    print("-" * 60)

    surge = 0.0
    yaw = 0.0
    heave = 0.0
    sway = 0.0

    power_limit = 0.30  # Başlangıç gaz limiti (%30)
    step = 0.10         # Tuş başına ivmelenme

    try:
        while True:
            key = get_key_nonblocking()

            if key == 'q':
                print("\nÇıkış komutu alındı.")
                break
            elif key == 'w':
                surge = min(power_limit, surge + step)
            elif key == 's':
                surge = max(-power_limit, surge - step)
            elif key == 'a':
                yaw = max(-power_limit, yaw - step)
            elif key == 'd':
                yaw = min(power_limit, yaw + step)
            elif key == 'i':
                heave = max(-power_limit, heave - step)  # Yüksel
            elif key == 'k':
                heave = min(power_limit, heave + step)  # Dal
            elif key == 'j':
                sway = max(-power_limit, sway - step)
            elif key == 'l':
                sway = min(power_limit, sway + step)
            elif key == '+':
                power_limit = min(1.0, power_limit + 0.05)
                print(f"\n[GÜÇ] Limiti Artırıldı: %{int(power_limit * 100)}")
            elif key == '-':
                power_limit = max(0.05, power_limit - 0.05)
                print(f"\n[GÜÇ] Limiti Düşürüldü: %{int(power_limit * 100)}")
            elif key == ' ':
                surge = 0.0; yaw = 0.0; heave = 0.0; sway = 0.0
                print("\n[NÖTR] Motorlar Sıfırlandı.")

            # Motor komutlarını PCA9685'e sür
            drv.apply_mix(surge, yaw, heave, sway)

            # Ekran durum çıktısı (tek satır)
            sys.stdout.write(
                f"\r[GÜÇ %{int(power_limit*100)}] "
                f"Surge: {surge:+.2f} | Yaw: {yaw:+.2f} | Heave: {heave:+.2f} | Sway: {sway:+.2f}    "
            )
            sys.stdout.flush()

            time.sleep(0.02)  # 50Hz döngü

    except KeyboardInterrupt:
        print("\nCtrl+C ile durduruldu.")
    finally:
        drv.stop_all()
        print(f"\n[GUVENLI CIKIS] Tüm motorlar {NEUTRAL_US}us Nötr konumuna çekildi.")


if __name__ == "__main__":
    main()
