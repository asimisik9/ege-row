"""
Basinc (derinlik) sensoru kalibrasyon scripti - MS5837-30BA.

Kullanim (cihazda, rov/ klasoru icinden):
    python3 calibrate_depth.py

Ne yapar:
  1) Su tipini sorar (havuz/deniz) -> FLUID_DENSITY secilir.
  2) Sensor SU YUZEYINDEYKEN (veya havada, yuzeyle ayni seviyede) 10 sn
     olcum yapar -> ortalama yuzey basinci SURFACE_PRESSURE_MBAR hesaplanir.
  3) Olcum gurultusunu (std sapma) kontrol eder, cok gurultuluyse uyarir.
  4) Sonuclari config.py dosyasina OTOMATIK yazar
     (eski hali config.py.bak olarak yedeklenir).
  5) Dogrulama: kalibrasyon sonrasi derinlik ~0 m okunmali.

Not: Gorev basinda mission.start() yine zero_at_surface() cagirir; buradaki
     deger guc kesintisi / yeniden baslatma durumunda guvenli fallback olur.
"""
import re
import shutil
import statistics
import time
import os

from sensors.depth import Ms5837


# config.py her zaman bu scriptin yanindadir (calistirma dizininden bagimsiz)
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.py")

SAMPLE_S = 10.0          # olcum suresi (sn)
MAX_STD_MBAR = 2.0       # kabul edilebilir gurultu (std sapma, mbar)
                         # 1 mbar ~ 1 cm su derinligi


def choose_density():
    """Kullaniciya su tipini sorar, kg/m3 yogunluk dondurur."""
    print("\nSu tipi sec:")
    print("  1) Havuz / tatli su (997 kg/m3)  <- TEKNOFEST havuzu icin bu")
    print("  2) Deniz suyu (1025 kg/m3)")
    while True:
        c = input("Secim [1/2] (varsayilan 1): ").strip() or "1"
        if c in ("1", "2"):
            return 997 if c == "1" else 1025
        print("Gecersiz secim, 1 veya 2 yaz.")


def measure_surface_pressure(sensor, duration_s=SAMPLE_S):
    """Sensor yuzeydeyken duration_s saniye basinc orneklyip
    (ortalama, std sapma) dondurur."""
    print(f"\nOlculuyor ({duration_s:.0f} sn) - SENSORU OYNATMA, "
          "su yuzeyi seviyesinde sabit tut...")
    vals = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        vals.append(sensor.read_pressure_mbar())
        time.sleep(0.05)
    avg = statistics.mean(vals)
    std = statistics.pstdev(vals)
    print(f"Ornek sayisi : {len(vals)}")
    print(f"Ortalama     : {avg:.2f} mbar")
    print(f"Std sapma    : {std:.3f} mbar")
    if std > MAX_STD_MBAR:
        print(f"[UYARI] Gurultu yuksek (>{MAX_STD_MBAR} mbar)! Sensor sabit mi? "
              "Dalga var mi? Gerekirse tekrar calistir.")
    if not (300.0 < avg < 1300.0):
        print("[UYARI] Basinc degeri beklenen atmosfer araliginin disinda! "
              "Sensor baglantisi / PROM okumasi hatali olabilir.")
    return round(avg, 2), std



def write_config(surface_mbar, density,
                  path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.py")):    
    """config.py icindeki SURFACE_PRESSURE_MBAR ve FLUID_DENSITY satirlarini
    yeni degerlerle degistirir. Once .bak yedegi alinir."""
    shutil.copy(path, path + ".bak")
    with open(path) as f:
        text = f.read()

    text = re.sub(r"FLUID_DENSITY\s*=\s*\S+",
                  f"FLUID_DENSITY = {density}", text, count=1)

    line = (f"SURFACE_PRESSURE_MBAR = {surface_mbar}"
            "  # kalibrasyon ile olculdu (calibrate_depth.py)")
    if re.search(r"SURFACE_PRESSURE_MBAR\s*=", text):
        text = re.sub(r"SURFACE_PRESSURE_MBAR\s*=.*", line, text, count=1)
    else:
        # FLUID_DENSITY satirinin altina ekle
        text = re.sub(r"(FLUID_DENSITY\s*=.*)", r"\1\n" + line, text, count=1)

    with open(path, "w") as f:
        f.write(text)
    print(f"\nconfig.py guncellendi. Eski hali: {path}.bak")


def verify(sensor, surface_mbar):
    """Kalibrasyon sonrasi hizli dogrulama: derinlik ~0 m okunmali."""
    sensor.surface_pressure_mbar = surface_mbar
    d = sensor.read_depth_m()
    print(f"Dogrulama: su anki derinlik = {d:.3f} m (0'a yakin olmali)")
    if d > 0.10:
        print("[UYARI] Derinlik 0'dan belirgin sapiyor - sensor yuzeyde mi?")


def main():
    print("=== Basinc Sensoru (MS5837) Kalibrasyonu ===")
    density = choose_density()

    input("\nSensoru SU YUZEYI seviyesinde sabit tut. Hazir olunca ENTER...")
    sensor = Ms5837()

    surface_mbar, _ = measure_surface_pressure(sensor)

    print("\n--- OZET ---")
    print(f"SURFACE_PRESSURE_MBAR = {surface_mbar}")
    print(f"FLUID_DENSITY         = {density}")

    write_config(surface_mbar, density)
    verify(sensor, surface_mbar)
    print("\nBitti. Sirada IMU: 'python3 calibrate_imu.py'")


if __name__ == "__main__":
    main()
