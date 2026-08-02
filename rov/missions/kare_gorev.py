#!/usr/bin/env python3
"""
EGE ROV — KARE GOREV (acik dongu, PID YOK, sensor YOK).

Varsayilan gorev (baslangic alanina geri doner):

    1) dal        10 sn   TAM GUC dalis
    2) duz        15 sn   ileri
    3) sag         2.5 sn 90 derece saga donus (yerinde)
    4) duz        15 sn   ileri
    5) cember_sag  6 sn   genis cember cizerek saga donus
    6) duz        15 sn   ileri
    7) sag         2.5 sn 90 derece saga donus (yerinde)
    8) duz        15 sn   ileri   -> baslangic alani
    9) cik        10 sn   TAM GUC yuzeye cikis

    4 duz kenar + 3 saga donus = kapali kare, baslangica donus.

PID / IMU / pusula KULLANILMAZ. Donus acisi SURE ile belirlenir, yani
--donus-sure degerini havuzda bir kez kalibre etmeniz gerekir:

    python3 missions/kare_gorev.py --gorev "sag:2.5"      # sadece donusu dene
    ... 90 dereceden az dondu  -> --donus-sure buyut
    ... 90 dereceden cok dondu -> --donus-sure kucult

Kullanim (Jetson, rov/ klasorunde):
    python3 missions/kare_gorev.py --sim                       # kuru test
    python3 missions/kare_gorev.py                             # varsayilan gorev
    python3 missions/kare_gorev.py --duz-sure 20 --donus-sure 3
    python3 missions/kare_gorev.py --guc 0.35 --donus-guc 0.45
    python3 missions/kare_gorev.py --cember-sure 8 --cember-guc 0.30 --cember-yaw 0.25
    python3 missions/kare_gorev.py --dalis-sure 12 --dalis-guc 0.80   # dalis/cikis
    python3 missions/kare_gorev.py --dal 0.25                  # sabit dalis itkisi
    python3 missions/kare_gorev.py --gorev "dal:10,duz:15,sag,duz:15,cik:10"

--gorev ile tamamen kendi diziniz:
    adim adlari : duz, geri, sag, sol, cember_sag, cember_sol, dal, cik, bekle
    "ad"        -> o adimin varsayilan suresi
    "ad:SANIYE" -> sureyi bu adim icin ez

GUVENLIK: Ctrl+C ve script sonu her kosulda motorlari notre ceker.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (LOOP_HZ, PWM_NEUTRAL_US, PWM_RANGE_US, PWM_DEADBAND_US,
                    THRUST_LIMIT, ESC_ABS_MIN_US, ESC_ABS_MAX_US)
from control.mixer import mix

# ---------------------------------------------------------------- gorev
# Varsayilan dizi. --gorev ile komut satirindan da verilebilir.
VARSAYILAN_GOREV = [
    "dal",          # tam guc dalis
    "duz",          # 1. kenar
    "sag",          # 90 derece saga
    "duz",          # 2. kenar
    "cember_sag",   # genis cember cizerek saga
    "duz",          # 3. kenar
    "sag",          # 90 derece saga
    "duz",          # 4. kenar -> baslangic alani
    "cik",          # tam guc yuzeye cikis
]

# adim adi -> (surge_carpani, yaw_carpani, heave_carpani, sure_tipi)
#   heave: +1 = dal, -1 = cik
#   sure_tipi: hangi CLI suresinin kullanilacagi
ADIMLAR = {
    "duz":        (+1.0,  0.0,  0.0, "duz"),
    "geri":       (-1.0,  0.0,  0.0, "duz"),
    "sag":        ( 0.0, +1.0,  0.0, "donus"),
    "sol":        ( 0.0, -1.0,  0.0, "donus"),
    "cember_sag": (+1.0, +1.0,  0.0, "cember"),
    "cember_sol": (+1.0, -1.0,  0.0, "cember"),
    "dal":        ( 0.0,  0.0, +1.0, "dalis"),
    "cik":        ( 0.0,  0.0, -1.0, "dalis"),
    "bekle":      ( 0.0,  0.0,  0.0, "bekle"),
}


def parse_args():
    p = argparse.ArgumentParser(
        description="ROV kare gorev: duz-donus-duz... acik dongu, PID yok.",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    g = p.add_argument_group("sureler (saniye)")
    g.add_argument("--duz-sure", type=float, default=15.0, dest="duz_sure",
                   help="her duz kenarin suresi (varsayilan: 15)")
    g.add_argument("--donus-sure", type=float, default=2.5, dest="donus_sure",
                   help="yerinde 90 derece donus suresi — KALIBRE EDIN (varsayilan: 2.5)")
    g.add_argument("--cember-sure", type=float, default=6.0, dest="cember_sure",
                   help="genis cember donusunun suresi (varsayilan: 6)")
    g.add_argument("--dalis-sure", type=float, default=10.0, dest="dalis_sure",
                   help="'dal' ve 'cik' adimlarinin suresi (varsayilan: 10)")
    g.add_argument("--ara", type=float, default=1.0,
                   help="adimlar arasi notr bekleme (varsayilan: 1)")
    g.add_argument("--bekle-sure", type=float, default=3.0, dest="bekle_sure",
                   help="'bekle' adiminin suresi (varsayilan: 3)")
    g.add_argument("--geri-sayim", type=float, default=5.0, dest="geri_sayim",
                   help="baslamadan onceki geri sayim (varsayilan: 5)")

    g = p.add_argument_group("gucler (0..1)")
    g.add_argument("--guc", type=float, default=0.30,
                   help="duz kenar itki gucu (varsayilan: 0.30)")
    g.add_argument("--donus-guc", type=float, default=0.40, dest="donus_guc",
                   help="yerinde donus yaw gucu (varsayilan: 0.40)")
    g.add_argument("--cember-guc", type=float, default=0.30, dest="cember_guc",
                   help="cember sirasinda ileri itki (varsayilan: 0.30)")
    g.add_argument("--cember-yaw", type=float, default=0.18, dest="cember_yaw",
                   help="cember sirasinda donus itkisi — kucuk = genis cember "
                        "(varsayilan: 0.18)")
    g.add_argument("--dalis-guc", type=float, default=1.0, dest="dalis_guc",
                   help="'dal'/'cik' adimlarinin dikey itki gucu, "
                        "1.0 = TAM GUC (varsayilan: 1.0)")
    g.add_argument("--dal", type=float, default=0.0,
                   help="yatay adimlar boyunca sabit dalis itkisi, "
                        "derinligi korur (varsayilan: 0)")

    g = p.add_argument_group("gorev / yon")
    g.add_argument("--gorev", type=str, default=None,
                   help='ozel dizi, orn: "duz:15,sag,duz:20,cember_sag:8,duz"')
    g.add_argument("--tur", type=int, default=1,
                   help="gorevi kac kez tekrarla (varsayilan: 1)")
    g.add_argument("--ters", action="store_true",
                   help="ileri/geri ters ciktiysa surge yonunu cevir")
    g.add_argument("--ters-donus", action="store_true", dest="ters_donus",
                   help="sag/sol ters ciktiysa yaw yonunu cevir")

    g = p.add_argument_group("calistirma")
    g.add_argument("--sim", action="store_true",
                   help="donanimsiz test (motorlara sinyal gitmez)")
    g.add_argument("--onay-yok", action="store_true", dest="onay_yok",
                   help="ENTER onayini atla")
    g.add_argument("--kuru", action="store_true",
                   help="sadece plani yazdir, motorlari hic calistirma")

    args = p.parse_args()

    for ad, v in (("--guc", args.guc), ("--donus-guc", args.donus_guc),
                  ("--cember-guc", args.cember_guc), ("--cember-yaw", args.cember_yaw),
                  ("--dalis-guc", args.dalis_guc)):
        if not 0.0 < v <= 1.0:
            p.error(f"{ad} 0 ile 1 arasinda olmali")
    if not 0.0 <= args.dal <= 1.0:
        p.error("--dal 0 ile 1 arasinda olmali")
    for ad, v in (("--duz-sure", args.duz_sure), ("--donus-sure", args.donus_sure),
                  ("--cember-sure", args.cember_sure), ("--dalis-sure", args.dalis_sure)):
        if v <= 0:
            p.error(f"{ad} sifirdan buyuk olmali")
    if args.ara < 0 or args.geri_sayim < 0:
        p.error("--ara ve --geri-sayim negatif olamaz")
    if args.tur < 1:
        p.error("--tur en az 1 olmali")
    return args, p


def gorev_coz(args, p):
    """Adim listesini (ad, sure) ciftlerine cevirir."""
    ham = args.gorev.split(",") if args.gorev else list(VARSAYILAN_GOREV)
    sure_tablo = {
        "duz": args.duz_sure,
        "donus": args.donus_sure,
        "cember": args.cember_sure,
        "dalis": args.dalis_sure,
        "bekle": args.bekle_sure,
    }
    plan = []
    for parca in ham:
        parca = parca.strip()
        if not parca:
            continue
        if ":" in parca:
            ad, _, s = parca.partition(":")
            ad = ad.strip()
            try:
                sure = float(s)
            except ValueError:
                p.error(f"'{parca}' icindeki sure sayi degil")
            if sure <= 0:
                p.error(f"'{parca}' suresi sifirdan buyuk olmali")
        else:
            ad, sure = parca, None
        if ad not in ADIMLAR:
            p.error(f"bilinmeyen adim '{ad}'. Gecerli: {', '.join(ADIMLAR)}")
        if sure is None:
            sure = sure_tablo[ADIMLAR[ad][3]]
        plan.append((ad, sure))
    if not plan:
        p.error("--gorev bos")
    return plan * args.tur


def adim_komutu(ad, args):
    """Adim adindan (surge, yaw, heave) komut degerlerini uretir.

    heave: dal/cik adimlarinda o adimin kendi itkisi; diger adimlarda
    derinligi korumak icin sabit --dal degeri kullanilir.
    """
    s_c, y_c, h_c, tip = ADIMLAR[ad]
    s_isaret = -1.0 if args.ters else 1.0
    y_isaret = -1.0 if args.ters_donus else 1.0
    heave = args.dal
    if tip == "duz":
        surge, yaw = s_c * args.guc, 0.0
    elif tip == "donus":
        surge, yaw = 0.0, y_c * args.donus_guc
    elif tip == "cember":
        surge, yaw = s_c * args.cember_guc, y_c * args.cember_yaw
    elif tip == "dalis":
        surge, yaw = 0.0, 0.0
        heave = h_c * args.dalis_guc      # --dal'i ez: burada aktif dikey hareket var
    else:  # bekle
        surge, yaw = 0.0, 0.0
    return surge * s_isaret, yaw * y_isaret, heave


def olu_bant_kontrol(ad, guc, oldurucu=True):
    """Komutun olu bandin ustunde kalip kalmadigini dogrular."""
    sapma_us = guc * THRUST_LIMIT * PWM_RANGE_US
    if sapma_us <= PWM_DEADBAND_US:
        gereken = PWM_DEADBAND_US / (THRUST_LIMIT * PWM_RANGE_US)
        print(f"[HATA] {ad} {guc} -> notrden sadece {sapma_us:.0f}us sapma.")
        print(f"       Olu bant {PWM_DEADBAND_US}us, yani motorlar HIC donmez.")
        print(f"       En az {gereken:.2f} kullanin.")
        if oldurucu:
            sys.exit(1)
        return False
    return True


def geri_say(saniye):
    """Ekranda geri sayim gosterir. Ctrl+C ile iptal edilebilir."""
    if saniye <= 0:
        return
    print(f"\n[GERI SAYIM] {saniye:g} sn — iptal icin Ctrl+C")
    t0 = time.monotonic()
    while True:
        kalan = saniye - (time.monotonic() - t0)
        if kalan <= 0:
            break
        sys.stdout.write(f"\r  {kalan:5.1f} sn ...   ")
        sys.stdout.flush()
        time.sleep(0.1)
    print("\r  BASLIYOR!        ")


def surekli_uygula(thr, komut, sure, dt, etiket=None, toplam=None, gecmis=0.0):
    """Verilen motor komutunu 'sure' boyunca dongude tekrar gonderir."""
    t0 = time.monotonic()
    while True:
        gecen = time.monotonic() - t0
        if gecen >= sure:
            break
        thr.command(komut)
        if etiket:
            sys.stdout.write(
                f"\r  {etiket}: {gecen:5.1f}/{sure:.1f} sn"
                f"   | gorev {gecmis + gecen:6.1f}/{toplam:.1f} sn")
            sys.stdout.flush()
        time.sleep(dt)
    if etiket:
        print()


def main():
    args, p = parse_args()
    plan = gorev_coz(args, p)

    # kullanilan gucleri olu bant acisindan dogrula
    kullanilan = {ADIMLAR[ad][3] for ad, _ in plan}
    if "duz" in kullanilan:
        olu_bant_kontrol("--guc", args.guc)
    if "donus" in kullanilan:
        olu_bant_kontrol("--donus-guc", args.donus_guc)
    if "cember" in kullanilan:
        olu_bant_kontrol("--cember-guc", args.cember_guc)
        if not olu_bant_kontrol("--cember-yaw", args.cember_yaw, oldurucu=False):
            print("       [UYARI] Cember donusu gerceklesmeyecek, duz gidecek.")
    if "dalis" in kullanilan:
        olu_bant_kontrol("--dalis-guc", args.dalis_guc)
    if args.dal > 0:
        olu_bant_kontrol("--dal", args.dal)

    dt = 1.0 / LOOP_HZ
    toplam = sum(s for _, s in plan)
    toplam_ara = args.ara * max(0, len(plan) - 1)

    print("=" * 66)
    print("   EGE ROV — KARE GOREV (acik dongu, PID yok, sensor yok)")
    print("=" * 66)
    print(f"\n  Adim sayisi  : {len(plan)}   ({args.tur} tur)")
    print(f"  Hareket sure : {toplam:.1f} sn  + {toplam_ara:.1f} sn notr ara"
          f"  = {toplam + toplam_ara:.1f} sn")
    print(f"  Guc          : duz %{args.guc*100:.0f}  donus %{args.donus_guc*100:.0f}"
          f"  cember %{args.cember_guc*100:.0f}/yaw %{args.cember_yaw*100:.0f}"
          f"  dal/cik %{args.dalis_guc*100:.0f}")
    print(f"  Notr         : {PWM_NEUTRAL_US} us  "
          f"(ESC siniri {ESC_ABS_MIN_US}..{ESC_ABS_MAX_US} us)")
    if "dalis" in kullanilan:
        print(f"  dal/cik      : {args.dalis_sure:g} sn  @ %{args.dalis_guc*100:.0f}"
              f"{'  <-- TAM GUC' if args.dalis_guc >= 0.99 else ''}")
    if args.dal > 0:
        print(f"  Ara dalis    : %{args.dal*100:.0f} (yatay adimlarda derinlik korumasi)")
    else:
        print("  Ara dalis    : YOK — arac yatay adimlarda yuzeye cikabilir "
              "(--dal ile ekleyin)")
    if args.ters:
        print("  --ters       : ileri/geri cevrildi")
    if args.ters_donus:
        print("  --ters-donus : sag/sol cevrildi")

    print("\n  PLAN")
    print("  " + "-" * 62)
    print(f"  {'#':>2}  {'adim':<12} {'sure':>7}  {'surge':>6} {'yaw':>6} {'heave':>6}")
    for i, (ad, sure) in enumerate(plan, 1):
        surge, yaw, heave = adim_komutu(ad, args)
        print(f"  {i:>2}  {ad:<12} {sure:>6.1f}s  {surge:>+6.2f} {yaw:>+6.2f} "
              f"{heave:>+6.2f}")
    print("  " + "-" * 62)
    print("  heave: + dalis / - cikis")
    print("  NOT: donus acisi ve derinlik SURE ile belirlenir (pusula/basinc yok).")
    print("       --donus-sure ve --dalis-sure degerlerini havuzda kalibre edin.")

    if args.kuru:
        print("\n[KURU] --kuru verildi, motorlar calistirilmadi.")
        return

    if args.sim:
        print("\n[MOD] SIMULASYON — motorlara gercek sinyal GITMEZ.")
        from hal.thrusters import Thrusters, MockBackend
        backend = MockBackend()
    else:
        print("\n[MOD] GERCEK DONANIM")
        from hal.thrusters import Thrusters, PCA9685Backend
        try:
            backend = PCA9685Backend()
        except Exception as e:
            print(f"[HATA] PCA9685'e baglanilamadi: {e}")
            sys.exit(1)

    thr = Thrusters(backend)

    if not args.onay_yok:
        input("\n--> ESC'lere GUC VER, bip sesleri bitsin, sonra ENTER...")

    print("\n[ARM] ESC'ler arm ediliyor (2 sn notr)...")
    thr.arm()

    gecmis = 0.0

    try:
        geri_say(args.geri_sayim)
        print("\nDurdurmak icin Ctrl+C\n")

        for i, (ad, sure) in enumerate(plan, 1):
            surge, yaw, heave = adim_komutu(ad, args)
            komut = mix(surge=surge, yaw=yaw, heave=heave)
            us = {m: type(thr)._to_us(v) for m, v in komut.items()}
            print(f"[{i}/{len(plan)}] {ad.upper()}  surge={surge:+.2f} "
                  f"yaw={yaw:+.2f} heave={heave:+.2f}  ({sure:g} sn)")
            print(f"        H_L={us['H_L']}us H_R={us['H_R']}us  "
                  f"V_*={us['V_FL']},{us['V_FR']},{us['V_RL']},{us['V_RR']} us")
            surekli_uygula(thr, komut, sure, dt, etiket=ad,
                           toplam=toplam, gecmis=gecmis)
            gecmis += sure
            if i < len(plan) and args.ara > 0:
                # 'cik' sonrasi araci tekrar asagi itme; diger aralarda
                # derinligi korumak icin --dal itkisini surdur.
                ara_heave = 0.0 if ad == "cik" else args.dal
                print(f"        ...{args.ara:g} sn notr...")
                surekli_uygula(thr, mix(surge=0.0, yaw=0.0, heave=ara_heave),
                               args.ara, dt)

        print("\n[NOTR] Motorlar yumusak sekilde notre cekiliyor...")
        surekli_uygula(thr, mix(surge=0.0, yaw=0.0, heave=0.0), 1.0, dt)
        print("[BITTI] Gorev tamamlandi — arac baslangic alaninda olmali.")

    except KeyboardInterrupt:
        print("\n[IPTAL] Ctrl+C algilandi.")
    finally:
        thr.stop()
        print(f"[GUVENLI] Tum motorlar notrde ({PWM_NEUTRAL_US} us).")

    print("\n" + "-" * 66)
    print(" KALIBRASYON IPUCLARI")
    print("   Donus 90 dereceden AZ  -> --donus-sure buyut (ya da --donus-guc)")
    print("   Donus 90 dereceden COK -> --donus-sure kucult")
    print("   Cember cok dar         -> --cember-yaw kucult / --cember-guc buyut")
    print("   Cember cok genis       -> --cember-yaw buyut")
    print("   Kenarlar kisa/uzun     -> --duz-sure ayarla")
    print("   Yeterince dalmadi      -> --dalis-sure buyut")
    print("   Dibe carpti            -> --dalis-sure kucult / --dalis-guc dusur")
    print("   Yatay adimlarda yukseldi-> --dal 0.15..0.30 ver")
    print("   Duz giderken suruklenme-> ESC kalibrasyonu (calibrate_escs.py)")
    print("-" * 66)


if __name__ == "__main__":
    main()
