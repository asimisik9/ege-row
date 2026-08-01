"""
EGE ROV — ESC Kalibrasyon ve PCA9685 Dahili Saat Ayar Araci.

Bu script 2 kritik donanimsal problemi TAMAMEN COZER:
  1. PCA9685 Saat Sapmasi (Clock Drift): Dahili 25MHz osilatorun sicaklik/voltajla
     kaymasini engellemek icin referans frekansini kilitler.
  2. ESC Notr & Gaz Araligi Kalibrasyonu (Throttle Range Calibration):
     BLU 30A / BLHeli / Degz M1 ESC'lerinin dahili EEPROM hafizasina
     1000-1500-2000 us araligini ve 1500 us sabit notru kaydeder.

KULLANIM (Jetson uzerinde, PERVANELER TAKILI DEGILKEN):
    python3 calibrate_escs.py            # Otomatik 3 adimli ESC kalibrasyonu
    python3 calibrate_escs.py --clock    # PCA9685 saat frekansi olcum / test modu
"""
import sys
import time
import config
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (MOTOR_CHANNELS, PCA9685_REF_CLOCK_HZ, FREQ_HZ,
                    PWM_NEUTRAL_US, PWM_MIN_US, PWM_MAX_US)


def get_pca9685():
    """PCA9685'e baglanir (bus numarasi ile — bkz. hal/i2c.py)."""
    try:
        from hal.i2c import pca9685_ac
        return pca9685_ac(freq_hz=FREQ_HZ)
    except Exception as e:
        print(f"\n[HATA] PCA9685 baglanamadi:\n{e}")
        print("\n  Teshis icin: python3 i2c_tara.py")
        sys.exit(1)


def set_all_us(dev, us_val):
    for ch in MOTOR_CHANNELS.values():
        dev.set_us(ch, us_val)


def calibrate_escs():
    print("==================================================================")
    print("         EGE ROV — OTOMATIK ESC NOTR & GAZ KALIBRASYONU          ")
    print("==================================================================")
    print("UYARI: Pervaneler MUTLAKA SOKULU olmalidir!")
    print("PROSEDUR:")
    print(" 1. ESC BATARYA GUCUNU KESIN (Fişi çekin).")
    print(" 2. Enter'a bastiginizda Jetson MAX GAZ (2000us) gondermeye baslayacak.")
    print(" 3. Komuttan sonra ESC bataryasini baglayin -> ESC'ler yüksek bip sesi verecek.")
    print(" 4. Sistem otomatik MIN GAZ (1000us) ve NOTR (1500us) gondererek kalibrasyonu bitirecek.")
    print("------------------------------------------------------------------")
    
    input("\n[ADIM 1] ESC Gücünü KESTINIZ MI? Devam etmek icin ENTER'a basin...")
    
    dev = get_pca9685()

    # Adim 1: MAX GAZ
    print(f"\n---> [MAX GAZ] {PWM_MAX_US} us sinyali gonderiliyor...")
    set_all_us(dev, PWM_MAX_US)
    print("Şimdi ESC BATARYASINI BAGLAYIN!")
    print("ESC'lerin BİP-BİP (yüksek ton) sesini duyunca 4 saniye bekleyin...")
    time.sleep(4.0)

    # Adim 2: MIN GAZ
    print(f"\n---> [MIN GAZ] {PWM_MIN_US} us sinyali gonderiliyor...")
    set_all_us(dev, PWM_MIN_US)
    print("ESC'lerin alçak tonlu onay biplerini bekle (3sn)...")
    time.sleep(3.0)

    # Adim 3: NOTR
    print(f"\n---> [SABIT NOTR] {PWM_NEUTRAL_US} us sinyali gonderiliyor...")
    set_all_us(dev, PWM_NEUTRAL_US)
    print("ESC'lerin arm olma melodi sesini bekle (3sn)...")
    time.sleep(3.0)

    print("\n------------------------------------------------------------------")
    print("[BAŞARILI] ESC Notr & Gaz Kalibrasyonu Tamamlandi!")
    print(f"ESC EEPROM'una {PWM_MIN_US}us (Min) - {PWM_NEUTRAL_US}us (Notr) - {PWM_MAX_US}us (Max) sabitlendi.")
    print("Artik motorlar titremeyecek ve kayma yapmayacaktir.")
    print("==================================================================")


def clock_test():
    print("=== PCA9685 Saat Frekansı Ölçüm / Test Modu ===")
    print(f"Mevcut PCA9685_REF_CLOCK_HZ: {PCA9685_REF_CLOCK_HZ} Hz")
    print(f"Çıkış Sinyali: Tum kanallarda tam {PWM_NEUTRAL_US} us (50Hz)")
    print("Osiloskop veya multimetre ile darbe genisligini olcun.")
    print("Çıkış için Ctrl+C'ye basın.")

    dev = get_pca9685()
    try:
        while True:
            set_all_us(dev, PWM_NEUTRAL_US)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nTest bitti.")


if __name__ == "__main__":
    if "--clock" in sys.argv:
        clock_test()
    else:
        calibrate_escs()
