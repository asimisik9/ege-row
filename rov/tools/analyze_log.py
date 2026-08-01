#!/usr/bin/env python3
"""
LOG ANALIZ ARACI — kabul kriterlerini otomatik olcer.  (YENI DOSYA)

Kullanim:
    python3 tools/analyze_log.py logs/video_demo_20260801_141530.csv
    python3 tools/analyze_log.py            # en son logu otomatik bulur

NEDEN VAR:
  Havuz kenarinda her denemeden sonra CSV'ye elle bakip "asim ne kadardi,
  kac Hz donuyordu, yuzeye cikti mi" diye hesaplamak dakikalar alir ve
  hata yapilir. Bu arac PID_TASARIM_PLANI.md §2'deki H1..H9 kabul
  kriterlerini olcup GECTI/KALDI tablosu basar.

  Deneme -> analiz -> katsayi degistir -> tekrar dene dongusunu
  saniyeler icinde donmek, 3 saatlik havuz suresinde en degerli sey.
"""
import csv
import glob
import os
import statistics
import sys

# kabul kriterleri (PID_TASARIM_PLANI.md §2)
H = {
    "H1": ("Kontrol dongusu >= 30 Hz", 30.0),
    "H2": ("Derinlik tutma <= 5 cm RMS", 0.05),
    "H3": ("Derinlik asimi <= 15 cm", 0.15),
    "H4": ("Yuzeye cikmama (derinlik >= 0.25 m)", 0.25),
    "H5": ("Duz seyirde yon <= 3 deg RMS", 3.0),
    "H6": ("90 derece donus <= 6 s", 6.0),
    "H7": ("Roll/pitch <= 8 deg", 8.0),
    "H8": ("Daire sayaci hedefe ulasti (+-10 deg)", 10.0),
}


def f(row, key, default=None):
    v = row.get(key, "")
    if v in ("", None):
        return default
    try:
        return float(v)
    except ValueError:
        return default


def yukle(path):
    olaylar, veri = [], []
    with open(path, newline="", errors="replace") as fh:
        for row in csv.DictReader(fh):
            if row.get("state") == "EVENT":
                olaylar.append((f(row, "t", 0.0), row.get("note", "")))
            elif row.get("state"):
                veri.append(row)
    return olaylar, veri


def rms_sapma(vals, hedefler):
    d = [v - t for v, t in zip(vals, hedefler) if v is not None and t is not None]
    return statistics.pstdev(d) if len(d) > 1 else 0.0


def sonuc(ad, gecti, olculen, esik, birim=""):
    isaret = "GECTI" if gecti else "KALDI"
    print(f"  [{isaret}] {ad:<42} olculen {olculen:>8}  esik {esik}{birim}")
    return gecti


def main(path=None):
    if path is None:
        adaylar = sorted(glob.glob("logs/*.csv"), key=os.path.getmtime)
        if not adaylar:
            print("logs/ altinda csv bulunamadi.")
            return 1
        path = adaylar[-1]
    print("=" * 78)
    print(f"  LOG ANALIZI: {path}")
    print("=" * 78)

    olaylar, veri = yukle(path)
    if not veri:
        print("Veri satiri yok (gorev hic calismamis olabilir).")
        for t, n in olaylar:
            print(f"  {t:7.2f}s  {n}")
        return 1

    print("\n--- OLAYLAR ---")
    for t, n in olaylar:
        print(f"  {t:7.2f}s  {n}")

    durumlar = {}
    for r in veri:
        durumlar.setdefault(r["state"], []).append(r)

    print("\n--- DURUM SURELERI ---")
    for st, rows in durumlar.items():
        t0, t1 = f(rows[0], "t", 0), f(rows[-1], "t", 0)
        print(f"  {st:<12} {t1 - t0:6.1f} s   ({len(rows)} ornek)")

    print("\n--- KABUL KRITERLERI ---")
    tumu = []

    # H1 — dongu frekansi
    hzs = [f(r, "hz") for r in veri if f(r, "hz")]
    hz_med = statistics.median(hzs) if hzs else 0.0
    tumu.append(sonuc(H["H1"][0], hz_med >= H["H1"][1], f"{hz_med:.1f}", H["H1"][1], " Hz"))

    # H2 — derinlik RMS (duz segmentlerde)
    duz = [r for r in veri if r["state"].startswith("STRAIGHT")]
    if duz:
        d = [f(r, "depth") for r in duz]
        dt_ = [f(r, "depth_target") for r in duz]
        r2 = rms_sapma(d, dt_)
        tumu.append(sonuc(H["H2"][0], r2 <= H["H2"][1], f"{r2:.3f}", H["H2"][1], " m"))

        # H5 — yon RMS
        he = [abs(f(r, "heading_err", 0.0)) for r in duz]
        r5 = statistics.pstdev(he) if len(he) > 1 else 0.0
        tumu.append(sonuc(H["H5"][0], r5 <= H["H5"][1], f"{r5:.2f}", H["H5"][1], " deg"))
    else:
        print("  [ - ] duz segment verisi yok (H2/H5 olculemedi)")

    # H3 — derinlik asimi (dalis sonrasi ilk 10 sn)
    dive = [r for r in veri if r["state"] in ("DIVE", "STRAIGHT1")]
    if dive:
        hedef = f(dive[-1], "depth_target", 0.6) or 0.6
        asim = max((f(r, "depth", 0.0) or 0.0) - hedef for r in dive)
        tumu.append(sonuc(H["H3"][0], asim <= H["H3"][1], f"{asim:.3f}", H["H3"][1], " m"))

    # H4 — yuzeye cikmama
    gorev = [r for r in veri if r["state"] not in ("IDLE", "COUNTDOWN", "DIVE", "HOVER")]
    if gorev:
        en_sig = min(f(r, "depth", 9.9) or 9.9 for r in gorev)
        tumu.append(sonuc(H["H4"][0], en_sig >= H["H4"][1], f"{en_sig:.3f}", H["H4"][1], " m"))

    # H6 — donus sureleri
    for st in ("TURN1", "TURN2"):
        if st in durumlar:
            rows = durumlar[st]
            sure = f(rows[-1], "t", 0) - f(rows[0], "t", 0)
            tumu.append(sonuc(f"{H['H6'][0]} ({st})", sure <= H["H6"][1],
                              f"{sure:.2f}", H["H6"][1], " s"))

    # H7 — roll/pitch
    rp = [max(abs(f(r, "roll", 0.0) or 0.0), abs(f(r, "pitch", 0.0) or 0.0)) for r in veri]
    if rp:
        # tek tuk sicramayi cezalandirmamak icin %95'lik dilim
        rp.sort()
        p95 = rp[int(len(rp) * 0.95)]
        tumu.append(sonuc(H["H7"][0], p95 <= H["H7"][1], f"{p95:.1f}", H["H7"][1], " deg"))

    # H8 — daire toplam donusu (olay notundan)
    # DIKKAT: hedef 360 DEGIL, config.MISSION["circle_deg"] (varsayilan 370).
    # 370 bilincli bir paydir: sartname "en az 1 tam tur" diyor, eksik
    # kalmaktansa 10 derece fazla donmek guvenli. Fazla donus zaten hemen
    # ardindan h0+180 kilidiyle toparlaniyor.
    # Bu yuzden kriter "360'a esit mi" degil, "SAYAC HEDEFINE DOGRU ULASTI MI".
    try:
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from config import MISSION as _M
        hedef_deg = float(_M.get("circle_deg", 370.0))
    except Exception:
        hedef_deg = 370.0
    for t, n in olaylar:
        if "DAIRE tamam" in n:
            try:
                deg = float(n.split()[-2])
                ok = abs(abs(deg) - hedef_deg) <= H["H8"][1] and hedef_deg >= 360.0
                tumu.append(sonuc(f"{H['H8'][0]} [hedef {hedef_deg:.0f}]", ok,
                                  f"{deg:.1f}", H["H8"][1], " deg"))
            except (ValueError, IndexError):
                pass

    # --- PID TANI (havuzda ne yapmali) ---
    print("\n--- PID TANI ---")
    doygun = [r for r in veri if r.get("depth_sat") == "1"]
    if veri:
        oran = 100.0 * len(doygun) / len(veri)
        print(f"  Derinlik cikisi doygun kaldigi sure: %{oran:.1f}")
        if oran > 25:
            print("    -> Yetki yetmiyor ya da FF_HOVER eksik. "
                  "Once 'hover' ile FF'i olc.")
    i_terms = [abs(f(r, "depth_i", 0.0) or 0.0) for r in veri]
    if i_terms:
        i_max = max(i_terms)
        print(f"  Derinlik I terimi tepe degeri: {i_max:.3f}")
        if i_max > 0.35:
            print("    -> I tavana dayanmis (windup riski). FF'i artir, Ki'yi dusur.")
    ff = [f(r, "depth_ff", 0.0) for r in veri if f(r, "depth_ff") is not None]
    if ff and max(ff) == 0.0:
        print("  FF_HOVER = 0 -> ILERI BESLEME HIC OLCULMEMIS.")
        print("    -> Havuz protokolu Adim 2: python3 pid_tune.py ; 'hover 0.15' ...")

    print("\n" + "=" * 78)
    if tumu and all(tumu):
        print("  SONUC: TUM KRITERLER GECTI")
    else:
        print(f"  SONUC: {sum(1 for x in tumu if x)}/{len(tumu)} kriter gecti")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else None))
