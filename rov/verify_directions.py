#!/usr/bin/env python3
"""
MOTOR YON DOGRULAMA — SORUN 5'in cozumu.  (YENI DOSYA)

==============================================================================
NEDEN BU KADAR KRITIK?
==============================================================================
config.py'de her motor icin +1 / -1 yazan bir tablo var (MOTOR_DIRECTION):
"bu motor ters bagli, komutu ters cevir".

Su an HEPSI -1. Ve config.py'nin kendi yorumu bunu itiraf ediyor: bu tablo,
AYRI bir PWM frekans hatasi (58.1 Hz yerine 50 Hz olmasi gerekiyordu) varken
olculmus. O sirada notr/ileri/geri darbelerinin UCU DE 1500 us altina
dusuyordu, yani her motor her testte geri donuyordu -> hepsine -1 yazilmis.
Frekans hatasi duzeltildi, TABLO DUZELTILMEDI.

PID acisindan anlami:
  PID'in tum mantigi su varsayima dayanir:
      "cok yukaridayim -> asagi it -> asagi inerim -> hata azalir"
  Isaret tersse:
      "cok yukaridayim -> 'asagi it' diyorum ama motor YUKARI itiyor ->
       daha da yukari cikiyorum -> hata buyuyor -> daha sert itiyorum -> ..."
  Buna POZITIF GERI BESLEME denir; sistem saniyeler icinde patlar.
  HICBIR Kp/Ki/Kd degeri bunu duzeltemez.

Bu yuzden havuza girmeden ONCE, kuru ortamda calistirilmasi ZORUNLUDUR.

==============================================================================
NASIL CALISIR
==============================================================================
ASAMA A — motorlar tek tek:
  Her motora HAM +%25 komut gonderilir (MOTOR_DIRECTION uygulanmadan).
  Sana beklenen davranis sorulur:
     yatay motorlar (H_L/H_R): "+" komut araci ILERI itmeli
     dikey motorlar (V_*)    : "+" komut araci ASAGI itmeli
  Cevabina gore o motorun isareti belirlenir.

ASAMA B — eksen dogrulamasi:
  Bulunan tabloyla mixer calistirilir ve butun eksenler test edilir:
  ileri / saga donus / dalis. Bu, tablonun dogru oldugunu TEYIT eder.

ASAMA C — config.py'ye yazma (once .bak yedegi alinir).

GUVENLIK: Ctrl+C her an motorlari notre ceker.
"""
import re
import shutil
import sys
import time

from config import MOTOR_CHANNELS, MOTOR_DIRECTION, LOOP_HZ

TEST_GUC = 0.25      # %25 — dondugunu gorecek kadar, tehlikeli olmayacak kadar
TEST_SURE = 2.5      # saniye

BEKLENEN = {
    "H_L":  ("YATAY (sol)",  "araci ILERI itmeli (su GERIYE atilmali)"),
    "H_R":  ("YATAY (sag)",  "araci ILERI itmeli (su GERIYE atilmali)"),
    "V_FL": ("DIKEY (on-sol)",   "araci ASAGI itmeli (su YUKARI atilmali)"),
    "V_FR": ("DIKEY (on-sag)",   "araci ASAGI itmeli (su YUKARI atilmali)"),
    "V_RL": ("DIKEY (arka-sol)", "araci ASAGI itmeli (su YUKARI atilmali)"),
    "V_RR": ("DIKEY (arka-sag)", "araci ASAGI itmeli (su YUKARI atilmali)"),
}


def evet_mi(soru):
    while True:
        c = input(soru + " [e/h/t=tekrar]: ").strip().lower()
        if c in ("e", "evet", "y", "yes"):
            return True
        if c in ("h", "hayir", "n", "no"):
            return False
        if c in ("t", "tekrar", "r"):
            return None
        print("  'e' (evet), 'h' (hayir) ya da 't' (tekrar) yaz.")


def motor_sur(thr, isim, guc, sure):
    """Tek motoru HAM komutla surer (MOTOR_DIRECTION UYGULANMADAN)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < sure:
        thr.command({n: (guc if n == isim else 0.0) for n in MOTOR_CHANNELS})
        time.sleep(1.0 / LOOP_HZ)
    thr.command({n: 0.0 for n in MOTOR_CHANNELS})
    time.sleep(0.4)


def eksen_sur(thr, yeni_yonler, surge=0.0, yaw=0.0, heave=0.0, sure=3.0):
    """Bulunan yon tablosuyla mixer'i calistirir (eksen dogrulamasi)."""
    import control.mixer as mixer_mod
    eski = dict(mixer_mod.MOTOR_DIRECTION)
    mixer_mod.MOTOR_DIRECTION.update(yeni_yonler)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < sure:
            thr.command(mixer_mod.mix(surge, yaw, heave))
            time.sleep(1.0 / LOOP_HZ)
        thr.command(mixer_mod.mix(0, 0, 0))
        time.sleep(0.4)
    finally:
        mixer_mod.MOTOR_DIRECTION.clear()
        mixer_mod.MOTOR_DIRECTION.update(eski)


def config_yaz(yonler, path="config.py"):
    shutil.copy(path, path + ".bak")
    with open(path, encoding="utf-8") as f:
        text = f.read()

    yeni = ("MOTOR_DIRECTION = {\n"
            f'    "V_FL": {yonler["V_FL"]:>2}, "V_FR": {yonler["V_FR"]:>2}, '
            f'"V_RL": {yonler["V_RL"]:>2}, "V_RR": {yonler["V_RR"]:>2},\n'
            f'    "H_L": {yonler["H_L"]:>2}, "H_R": {yonler["H_R"]:>2},\n'
            "}  # verify_directions.py ile OLCULDU")

    desen = re.compile(r"MOTOR_DIRECTION\s*=\s*\{[^}]*\}[^\n]*")
    if not desen.search(text):
        print("[HATA] config.py icinde MOTOR_DIRECTION blogu bulunamadi.")
        return False
    text = desen.sub(yeni, text, count=1)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"\n[OK] config.py guncellendi. Eski hali: {path}.bak")
    return True


def main():
    print("=" * 74)
    print("        MOTOR YON DOGRULAMA  (SORUN 5)")
    print("=" * 74)
    print("""
GUVENLIK KONTROL LISTESI — devam etmeden once:
  1. Pervaneler SOKULU olsun ya da arac SUYUN ICINDE olsun.
     (Havada pervaneli calistirmak hem tehlikeli hem yanlis sonuc verir.)
  2. ESC batarya baglantisi takili ve PCA9685 lojik beslemesi var.
  3. Elini pervanelerden uzak tut. Ctrl+C her an motorlari durdurur.

Mevcut tablo (dogrulanacak):""")
    for k, v in MOTOR_DIRECTION.items():
        print(f"    {k:<5} : {v:+d}")
    if input("\nDevam edilsin mi? [e/H]: ").strip().lower() not in ("e", "evet"):
        print("Iptal.")
        return

    from main import build_system
    thr, ori, depth, hub = build_system()
    yonler = {}

    try:
        thr.arm()
        print("\n" + "=" * 74)
        print("ASAMA A — MOTORLAR TEK TEK")
        print("=" * 74)
        for isim in ("H_L", "H_R", "V_FL", "V_FR", "V_RL", "V_RR"):
            rol, beklenti = BEKLENEN[isim]
            while True:
                print(f"\n--- {isim}  [{rol}] ---")
                print(f"    Beklenen: bu motor {beklenti}")
                input("    ENTER'a bas, motor 2.5 sn calisacak...")
                motor_sur(thr, isim, TEST_GUC, TEST_SURE)
                c = evet_mi(f"    {isim} beklenen yonde itti mi?")
                if c is None:
                    continue
                yonler[isim] = 1 if c else -1
                print(f"    -> {isim} = {yonler[isim]:+d}")
                break

        print("\n" + "=" * 74)
        print("BULUNAN TABLO")
        print("=" * 74)
        for k in ("V_FL", "V_FR", "V_RL", "V_RR", "H_L", "H_R"):
            degisti = " (DEGISTI)" if yonler[k] != MOTOR_DIRECTION[k] else ""
            print(f"    {k:<5} : {yonler[k]:+d}{degisti}")

        print("\n" + "=" * 74)
        print("ASAMA B — EKSEN DOGRULAMASI (mixer ile)")
        print("=" * 74)
        print("Bu asama tablonun DOGRU oldugunu teyit eder. Yanlissa A'ya don.")

        testler = [
            ("ILERI  (surge=+0.3)", dict(surge=0.30),
             "iki YATAY motor calismali, arac ILERI gitmeli"),
            ("SAGA DONUS (yaw=+0.3)", dict(yaw=0.30),
             "sol yatay ILERI, sag yatay GERI -> arac SAGA donmeli"),
            ("DALIS  (heave=+0.3)", dict(heave=0.30),
             "dort DIKEY motor da araci ASAGI itmeli"),
        ]
        hatali = False
        for ad, kw, aciklama in testler:
            print(f"\n--- {ad} ---")
            print(f"    Beklenen: {aciklama}")
            input("    ENTER'a bas (3 sn)...")
            eksen_sur(thr, yonler, sure=3.0, **kw)
            if evet_mi("    Beklendigi gibi miydi?") is not True:
                hatali = True
                print("    [!] Bu eksende sorun var — asagida ozetlenecek.")

        print("\n" + "=" * 74)
        if hatali:
            print("SONUC: EKSEN TESTI BASARISIZ.")
            print("  Muhtemel sebepler:")
            print("   - Motor KANALLARI karisik (config.MOTOR_CHANNELS) —")
            print("     yani 'H_L' dedigin motor aslinda baska bir kanalda.")
            print("     Kontrol: python3 kanal_test.py")
            print("   - Bir motorun mekanik yerlesimi (pervane hatvesi) ters.")
            print("  config.py YAZILMADI.")
        else:
            print("SONUC: TUM EKSENLER DOGRU.")
            if input("Tablo config.py'ye yazilsin mi? [E/h]: ").strip().lower() in ("", "e", "evet"):
                config_yaz(yonler)
                print("\nSonraki adim: python3 main.py --check-loop  (SORUN 2 dogrulamasi)")
            else:
                print("Yazilmadi. Degerler:", yonler)
        print("=" * 74)

    except KeyboardInterrupt:
        print("\nCtrl+C — motorlar durduruluyor.")
    finally:
        thr.stop()
        try:
            hub.stop()
        except Exception:
            pass
        print("[TAMAM] Motorlar notr.")


if __name__ == "__main__":
    sys.exit(main())
