#!/usr/bin/env python3
"""
EGE ROV — I2C TARAMA / TESHIS ARACI.

"PCA9685 baglanamadi" / "pin yok" hatasinin sebebini bulur.
HICBIR MOTORU DONDURMEZ, hicbir yere yazmaz — sadece okur ve rapor eder.

Ne kontrol eder:
  1. /dev/i2c-* buslarindan hangileri var, hangilerini acabiliyoruz (izin)
  2. Her busta hangi adresler cevap veriyor (0x40 motor surucu, 0x68 IMU, ...)
  3. Blinka (board/busio) calisiyor mu, hangi busu seciyor
  4. adafruit-extended-bus kurulu mu

Kullanim (Jetson):
    python3 i2c_tara.py
    sudo python3 i2c_tara.py      # "Permission denied" aliyorsan
"""
import glob
import os
import sys

from config import I2C_BUS, IMU_ADDR, MAG_ADDR, DEPTH_ADDR

PCA_ADDR = 0x40

BILINEN = {
    PCA_ADDR:   "PCA9685 — MOTOR SURUCU (bize lazim olan)",
    IMU_ADDR:   "MPU-9250 IMU",
    MAG_ADDR:   "AK8963 pusula (IMU icinde)",
    DEPTH_ADDR: "MS5837 derinlik sensoru",
}


def buslari_bul():
    """Sistemde gorunen /dev/i2c-N bus numaralari."""
    nums = []
    for yol in glob.glob("/dev/i2c-*"):
        try:
            nums.append(int(yol.rsplit("-", 1)[1]))
        except ValueError:
            pass
    return sorted(nums)


def bus_tara(num):
    """Tek bir busu tarar. -> (adres_listesi, hata). Sadece OKUMA yapar."""
    try:
        from smbus2 import SMBus
    except ImportError:
        return None, "smbus2 kurulu degil (pip3 install smbus2)"
    try:
        bus = SMBus(num)
    except Exception as e:
        return None, str(e)

    bulunan = []
    for addr in range(0x03, 0x78):
        # i2cdetect mantigi: EEPROM bolgesi (0x50-0x5F) ve 0x30-0x37 READ ile,
        # geri kalani QUICK WRITE ile yoklanir. Sadece read_byte kullanmak
        # MS5837 gibi "komutsuz okumaya ACK vermeyen" cihazlari KACIRIR —
        # sensor calisir ama taramada bus 'bos' gorunur.
        try:
            if 0x30 <= addr <= 0x37 or 0x50 <= addr <= 0x5F:
                bus.read_byte(addr)
            else:
                bus.write_quick(addr)
            bulunan.append(addr)
            continue
        except Exception:
            pass
        try:  # diger yontemi de dene (bazi cihazlar tersine cevap verir)
            bus.read_byte(addr)
            bulunan.append(addr)
        except Exception:
            pass
    try:
        bus.close()
    except Exception:
        pass
    return bulunan, None


def blinka_kontrol():
    """board/busio ile varsayilan I2C busunu acmayi dener. -> (durum, detay)."""
    try:
        import board
    except Exception as e:
        return "YOK", (f"'import board' basarisiz: {e}\n"
                       "       adafruit-blinka kurulu degil ya da bozuk.")

    board_id = getattr(board, "board_id", None)
    if not hasattr(board, "SCL"):
        return "PIN YOK", (
            f"board modulu yuklendi ama SCL/SDA pini TANIMLI DEGIL "
            f"(board_id={board_id}).\n"
            "       Blinka karti tanimamis — 'pini goremiyor' hatasi tam olarak budur.")

    try:
        import busio
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        return "ACILAMADI", (f"busio.I2C(board.SCL, board.SDA) hata verdi: {e}\n"
                             f"       (board_id={board_id})")

    detay = f"board_id={board_id}"
    yol = _blinka_bus_adi(i2c)
    if yol:
        detay += f", kullandigi cihaz: {yol}"
    try:
        if i2c.try_lock():
            adresler = i2c.scan()
            i2c.unlock()
            detay += (", bu busta gordugu adresler: "
                      + (", ".join(f"0x{a:02X}" for a in adresler) or "YOK"))
            if PCA_ADDR in adresler:
                return "OK", detay
            return "YANLIS BUS", detay
    except Exception as e:
        detay += f", tarama hatasi: {e}"
    return "OK", detay


def _blinka_bus_adi(i2c):
    """Blinka'nin hangi /dev/i2c-N dosyasini actigini bulmaya calisir."""
    for yol in (("_i2c", "_i2c_bus", "_device", "name"),
                ("_i2c", "_device", "name"),
                ("_i2c", "_i2c_bus", "fd")):
        o = i2c
        try:
            for ad in yol:
                o = getattr(o, ad)
            return str(o)
        except Exception:
            continue
    return None


def main():
    print("=" * 66)
    print("   EGE ROV — I2C TARAMA / TESHIS  (hicbir motor donmez)")
    print("=" * 66)
    if hasattr(os, "geteuid"):
        print(f"  Kullanici     : uid={os.geteuid()} "
              f"({'root' if os.geteuid() == 0 else 'root DEGIL'})")
    print(f"  config I2C_BUS: {I2C_BUS}")

    buslar = buslari_bul()
    if not buslar:
        print("\n[HATA] /dev/i2c-* hic bulunamadi — I2C surucusu yuklu degil.")
        print("       Jetson'da: sudo modprobe i2c-dev  (ve 40-pin I2C aktif mi bak)")
        sys.exit(1)
    print(f"  Bulunan buslar: {', '.join(str(b) for b in buslar)}")

    print("\n--- BUS TARAMASI " + "-" * 49)
    pca_buslari = []
    izin_hatasi = False
    for b in buslar:
        adresler, hata = bus_tara(b)
        if hata:
            print(f"  /dev/i2c-{b:<3} acilamadi: {hata}")
            if "ermission" in hata:
                izin_hatasi = True
            continue
        if not adresler:
            print(f"  /dev/i2c-{b:<3} bos")
            continue
        etiketli = []
        for a in adresler:
            ad = BILINEN.get(a)
            etiketli.append(f"0x{a:02X}" + (f" <{ad}>" if ad else ""))
        print(f"  /dev/i2c-{b:<3} {', '.join(etiketli)}")
        if PCA_ADDR in adresler:
            pca_buslari.append(b)

    print("\n--- BLINKA (board / busio) " + "-" * 39)
    durum, detay = blinka_kontrol()
    print(f"  Durum: {durum}")
    print(f"       {detay}")

    try:
        import adafruit_extended_bus  # noqa: F401
        print("  adafruit-extended-bus: kurulu")
    except ImportError:
        print("  adafruit-extended-bus: kurulu DEGIL "
              "(gerekmez — yeni surucu smbus2 kullaniyor)")

    print("\n" + "=" * 66)
    print("   SONUC")
    print("=" * 66)

    if not pca_buslari:
        print("  PCA9685 (0x40) HICBIR BUSTA CEVAP VERMIYOR.")
        print("  Bu bir YAZILIM degil, BESLEME/KABLO problemidir:")
        print("   1. PCA9685'in VCC pini Jetson 3.3V (pin 1) ya da 5V (pin 2/4)'e")
        print("      BAGLI MI? Kart lojik beslemesi olmadan I2C'de GORUNMEZ.")
        print("      ESC batarya gucu (V+ / yesil klemens) BUNUN YERINE GECMEZ —")
        print("      o sadece motor rayini besler, cipi degil.")
        print("   2. GND ortak mi? (Jetson pin 6 <-> PCA9685 GND)")
        print("   3. SDA -> Jetson pin 3, SCL -> pin 5 (ters takilmis olabilir)")
        print("   4. Kartin uzerindeki guc LED'i yaniyor mu?")
        if izin_hatasi:
            print("   5. Bazi buslar izin hatasi verdi — 'sudo python3 i2c_tara.py'")
            print("      ile tekrar calistirip listenin degisip degismedigine bak.")
    else:
        print(f"  PCA9685 (0x40) BULUNDU -> /dev/i2c-{pca_buslari[0]}")
        print("  Yani kablolama ve besleme SAGLAM, sorun yazilimin yanlis busa")
        print("  bakmasiydi.")
        if durum != "OK":
            print(f"  Blinka durumu '{durum}' — eski scriptlerin 'pin goremiyor'")
            print("  hatasinin sebebi bu.")
        if I2C_BUS not in pca_buslari:
            print(f"  DIKKAT: config.py I2C_BUS={I2C_BUS}, ama 0x40 bus "
                  f"{pca_buslari[0]}'de. config.py'yi guncelleyin.")
        print("\n  Su komutla dogrulayin (motor DONMEZ, sadece notr sinyal):")
        print("      python3 calibrate_escs.py --clock")
    print("=" * 66)


if __name__ == "__main__":
    main()
