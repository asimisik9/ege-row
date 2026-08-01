#!/usr/bin/env python3
"""
EGE ROV — ROTA GOREVI (duz / donus / cember, yuzeyde ya da su altinda).

Rota:
    DUZ1 (15 sn) -> DONUS1 (90 sag) -> DUZ2 (15 sn) -> CEMBER (360)
    -> DONUS2 (90 sag) -> DUZ3 (15 sn) -> DONUS3 (90 sag) -> DUZ4 (15 sn) -> DUR

4 esit bacak + 3 kose donusu = kapali kare, yani arac baslangic alanina doner.
CEMBER 2. kose donusunden hemen once cizilir: kendi etrafinda donme DEGIL,
iki yatay itici de ILERI calisir (H_R > H_L), boylece arac yay cizerek
ilerler ve tur sonunda ayni noktaya doner.

Bu script sensor ZORUNLU tutmaz:
    - varsayilan: tamamen acik dongu (sure ile donus, sure ile cember)
    - --imu      : MPU-9250 jiroskopu ile aci sayarak donus/cember (daha isabetli)
    - --dal 0    : yuzeyde surus (varsayilan)
    - --dal 0.3  : rota boyunca sabit dalis itkisi (su altinda surus)

PID + derinlik sensoru ile kapali dongu isteyen surumu: main.py --mission video
(missions/video_demo.py) — o surum once dalmak zorundadir.

Kullanim (Jetson, rov/ klasorunde):
    python3 rota.py --plan                  # sadece rota ozetini ve PWM'leri yazdir
    python3 rota.py                         # yuzeyde, acik dongu
    python3 rota.py --imu                   # jiroskopla aci sayarak
    python3 rota.py --dal 0.30              # rota boyunca dalis itkisi
    python3 rota.py --donus-sure 4.5        # 90 derece donus suresini kalibre et
    python3 rota.py --yon sol               # donusler sola
    python3 rota.py --sim                   # donanimsiz kuru test

GUVENLIK: Ctrl+C ve script sonu her kosulda motorlari notre ceker.
"""
import argparse
import math
import sys
import time

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (LOOP_HZ, PWM_NEUTRAL_US, PWM_RANGE_US, PWM_DEADBAND_US,
                    THRUST_LIMIT, ESC_ABS_MIN_US, ESC_ABS_MAX_US, MISSION)
from control.mixer import mix

# Jiroskop ile heading tutma (sadece --imu ile aktif)
KP_HEADING     = 0.010   # derece sapma basina yaw komutu
YAW_DUZELT_MAX = 0.20    # duz giderken izin verilen max duzeltme komutu

# Rota: (ad, tip). Kose sayisi 3 — kapali kare icin yeterli.
ROTA = (
    ("DUZ1",   "duz"),
    ("DONUS1", "don"),
    ("DUZ2",   "duz"),
    ("CEMBER", "cember"),
    ("DONUS2", "don"),
    ("DUZ3",   "duz"),
    ("DONUS3", "don"),
    ("DUZ4",   "duz"),
)


# ----------------------------------------------------------------- argumanlar
def parse_args():
    p = argparse.ArgumentParser(
        description="ROV'u duz-donus-cember rotasinda gezdirir (yuzey ya da su alti).")
    p.add_argument("--sure", type=float, default=15.0,
                   help="her duz bacagin suresi, saniye (varsayilan: 15)")
    p.add_argument("--guc", type=float, default=MISSION["cruise_throttle"],
                   help=f"duz gidis itkisi 0..1 (varsayilan: {MISSION['cruise_throttle']})")
    p.add_argument("--donus-guc", type=float, default=0.45,
                   help="yerinde donus yaw itkisi 0..1 (varsayilan: 0.45)")
    p.add_argument("--donus-sure", type=float, default=3.0,
                   help="acik donguda 90 derece donus suresi, sn (varsayilan: 3)")
    p.add_argument("--donus-aci", type=float, default=90.0,
                   help="--imu ile: kose donus acisi, derece (varsayilan: 90)")
    p.add_argument("--cember-guc", type=float, default=0.35,
                   help="cemberde ileri itki 0..1 (varsayilan: 0.35)")
    p.add_argument("--cember-yaw", type=float, default=0.12,
                   help="cemberde yaw itkisi 0..1 — kucultursen cember buyur "
                        "(varsayilan: 0.12)")
    p.add_argument("--cember-sure", type=float, default=25.0,
                   help="acik donguda tam tur suresi, sn (varsayilan: 25)")
    p.add_argument("--cember-aci", type=float, default=370.0,
                   help="--imu ile: cember acisi, derece (varsayilan: 370)")
    p.add_argument("--ara", type=float, default=1.0,
                   help="fazlar arasi notr bekleme, sn (varsayilan: 1)")
    p.add_argument("--dal", type=float, default=0.0,
                   help="rota boyunca sabit dalis itkisi 0..1 "
                        "(varsayilan: 0 = yuzeyde)")
    p.add_argument("--yon", choices=("sag", "sol"), default="sag",
                   help="donus ve cember yonu (varsayilan: sag)")
    p.add_argument("--imu", action="store_true",
                   help="MPU-9250 jiroskopu ile aci sayarak don (sure yerine)")
    p.add_argument("--gyro-isaret", choices=("oto", "+1", "-1"), default="oto",
                   help="jiroskop z ekseni isareti; oto = ilk donuste olculur")
    p.add_argument("--donus-timeout", type=float, default=20.0,
                   help="--imu ile: donus icin max sure, sn (varsayilan: 20)")
    p.add_argument("--ters", action="store_true",
                   help="ileri/geri ters ciktiysa surge yonunu cevir")
    p.add_argument("--ters-donus", action="store_true",
                   help="sag/sol ters ciktiysa yaw yonunu cevir")
    p.add_argument("--ters-dal", action="store_true",
                   help="--dal araci daldirmak yerine yukari ittiyse cevir")
    p.add_argument("--iz-genisligi", type=float, default=0.30,
                   help="iki yatay itici arasi mesafe, m — sadece cember "
                        "yaricap tahmini icin (varsayilan: 0.30)")
    p.add_argument("--plan", action="store_true",
                   help="sadece rota ozetini ve PWM degerlerini yazdir, calistirma")
    p.add_argument("--sim", action="store_true",
                   help="donanimsiz test (motorlara sinyal gitmez)")
    p.add_argument("--onay-yok", action="store_true",
                   help="ENTER onayini atla (otomatik calistirma icin)")
    args = p.parse_args()

    for ad, deger in (("--guc", args.guc), ("--donus-guc", args.donus_guc),
                      ("--cember-guc", args.cember_guc),
                      ("--cember-yaw", args.cember_yaw)):
        if not 0.0 < deger <= 1.0:
            p.error(f"{ad} 0 ile 1 arasinda olmali")
    if not 0.0 <= args.dal <= 1.0:
        p.error("--dal 0 ile 1 arasinda olmali")
    for ad, deger in (("--sure", args.sure), ("--donus-sure", args.donus_sure),
                      ("--cember-sure", args.cember_sure)):
        if deger <= 0:
            p.error(f"{ad} sifirdan buyuk olmali")
    if args.ara < 0:
        p.error("--ara negatif olamaz")
    if args.cember_yaw >= args.cember_guc:
        p.error("--cember-yaw, --cember-guc degerinden KUCUK olmali; aksi halde "
                "arac cember cizmez, kendi etrafinda doner.")
    return args


def olu_bant_kontrol(ad, guc):
    """Komutun olu bandin ustunde kalip kalmadigini dogrular."""
    sapma_us = guc * THRUST_LIMIT * PWM_RANGE_US
    if sapma_us <= PWM_DEADBAND_US:
        gereken = PWM_DEADBAND_US / (THRUST_LIMIT * PWM_RANGE_US)
        print(f"[HATA] {ad} {guc} -> notrden sadece {sapma_us:.0f}us sapma.")
        print(f"       Olu bant {PWM_DEADBAND_US}us, yani motorlar HIC donmez.")
        print(f"       En az {gereken:.2f} kullanin.")
        sys.exit(1)
    return sapma_us


# ----------------------------------------------------------------- yardimcilar
class GyroTakip:
    """MPU-9250 z ekseni acisal hizini entegre eder (manyetometre KULLANMAZ).

    Pusula yerine jiroskop kullanilir: motor manyetik gurultusu heading'i
    bozar, kisa donuslerde entegrasyon daha guvenilirdir.
    """

    def __init__(self, imu, isaret=None):
        self.imu = imu
        self.isaret = isaret   # None: ilk donuste olculecek
        self.ham = 0.0         # ham entegre aci (derece)
        self._t = None

    def sifirla(self):
        self.ham = 0.0
        self._t = None

    def guncelle(self):
        """Bir okuma alip aciyi ilerletir; duzeltilmis aciyi dondurur."""
        now = time.monotonic()
        dt = 0.0 if self._t is None else min(0.2, now - self._t)
        self._t = now
        try:
            gz = self.imu.read_gyro_dps()[2]
        except Exception:
            return self.aci          # okuma hatasi: son aci ile devam
        self.ham += gz * dt
        return self.aci

    @property
    def aci(self):
        """Sag donus pozitif olacak sekilde duzeltilmis aci."""
        return (self.isaret if self.isaret is not None else 1.0) * self.ham


class Surucu:
    """Eksen komutlarini isaret duzeltmeleriyle motorlara gonderir."""

    def __init__(self, thr, args):
        self.thr = thr
        self.s_isaret = -1.0 if args.ters else 1.0
        self.y_isaret = -1.0 if args.ters_donus else 1.0
        self.heave = -args.dal if args.ters_dal else args.dal
        self.dt = 1.0 / LOOP_HZ

    def _komut(self, surge, yaw):
        return mix(surge=surge * self.s_isaret,
                   yaw=yaw * self.y_isaret,
                   heave=self.heave)

    def gonder(self, surge=0.0, yaw=0.0):
        self.thr.command(self._komut(surge, yaw))

    def pwm(self, surge=0.0, yaw=0.0):
        """Bu eksen komutu icin thrusters.py'nin gercekte yazacagi us degerleri."""
        return {ad: type(self.thr)._to_us(v)
                for ad, v in self._komut(surge, yaw).items()}

    def notr(self, sure):
        """Yatay eksenleri sifirlar; dalis itkisi varsa korunur."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < sure:
            self.gonder(0.0, 0.0)
            time.sleep(self.dt)


def pwm_yaz(us, girinti="    "):
    print(f"{girinti}H_L={us['H_L']:4d}  H_R={us['H_R']:4d} us   "
          f"V={us['V_FL']},{us['V_FR']},{us['V_RL']},{us['V_RR']} us")


def cember_yaricap_tahmini(surge, yaw, iz_genisligi):
    """Diferansiyel surusten kaba yaricap tahmini (metre).

    Itki ~ hiz^2 varsayimiyla hizlar sqrt ile olceklenir. Sadece buyukluk
    mertebesi verir — gercek yaricap suda olculmelidir.
    """
    sag, sol = surge + yaw, surge - yaw
    if sol <= 0:
        return None                      # ic itici geri/durur -> yay cok dar
    v_sag, v_sol = math.sqrt(sag), math.sqrt(sol)
    if v_sag - v_sol < 1e-6:
        return None
    return (iz_genisligi / 2.0) * (v_sag + v_sol) / (v_sag - v_sol)


# ----------------------------------------------------------------- faz kosucular
def faz_duz(sur, gyro, ad, surge, sure):
    """Duz bacak. Jiroskop varsa bacak basindaki yonu tutmaya calisir."""
    print(f"\n[{ad}] duz gidis  surge={surge:+.2f}  ({sure:g} sn)")
    pwm_yaz(sur.pwm(surge=surge))
    if gyro:
        gyro.sifirla()
    t0 = time.monotonic()
    while True:
        gecen = time.monotonic() - t0
        if gecen >= sure:
            break
        yaw = 0.0
        sapma = 0.0
        if gyro:
            sapma = gyro.guncelle()
            if gyro.isaret is not None:
                yaw = max(-YAW_DUZELT_MAX,
                          min(YAW_DUZELT_MAX, -KP_HEADING * sapma))
        sur.gonder(surge=surge, yaw=yaw)
        ek = f"  sapma: {sapma:+6.1f} deg" if gyro else ""
        sys.stdout.write(f"\r  gecen: {gecen:4.1f} / {sure:.1f} sn{ek}   ")
        sys.stdout.flush()
        time.sleep(sur.dt)
    print()


def faz_don(sur, gyro, ad, yaw, hedef_aci, sure, timeout):
    """Yerinde donus. Jiroskop varsa aciyla, yoksa sureyle biter."""
    if gyro:
        print(f"\n[{ad}] yerinde donus  yaw={yaw:+.2f}  hedef={hedef_aci:.0f} deg "
              f"(timeout {timeout:g} sn)")
    else:
        print(f"\n[{ad}] yerinde donus  yaw={yaw:+.2f}  ({sure:g} sn, acik dongu)")
    pwm_yaz(sur.pwm(yaw=yaw))
    if gyro:
        gyro.sifirla()
    t0 = time.monotonic()
    gecen = 0.0
    while True:
        gecen = time.monotonic() - t0
        if gyro:
            aci = gyro.guncelle()
            if abs(aci) >= hedef_aci:
                break
            if gecen >= timeout:
                print(f"\n  [UYARI] timeout — sadece {abs(aci):.0f} deg donuldu.")
                break
            sys.stdout.write(f"\r  aci: {abs(aci):5.1f} / {hedef_aci:.0f} deg   "
                             f"({gecen:4.1f} sn)   ")
        else:
            if gecen >= sure:
                break
            sys.stdout.write(f"\r  gecen: {gecen:4.1f} / {sure:.1f} sn   ")
        sys.stdout.flush()
        sur.gonder(yaw=yaw)
        time.sleep(sur.dt)
    print()
    return gecen


def faz_cember(sur, gyro, ad, surge, yaw, hedef_aci, sure, timeout):
    """Tam tur cember: iki yatay itici de ileri, biri daha guclu."""
    if gyro:
        print(f"\n[{ad}] cember  surge={surge:+.2f} yaw={yaw:+.2f}  "
              f"hedef={hedef_aci:.0f} deg (timeout {timeout:g} sn)")
    else:
        print(f"\n[{ad}] cember  surge={surge:+.2f} yaw={yaw:+.2f}  "
              f"({sure:g} sn, acik dongu)")
    pwm_yaz(sur.pwm(surge=surge, yaw=yaw))
    if gyro:
        gyro.sifirla()
    t0 = time.monotonic()
    while True:
        gecen = time.monotonic() - t0
        if gyro:
            aci = gyro.guncelle()
            if abs(aci) >= hedef_aci:
                break
            if gecen >= timeout:
                print(f"\n  [UYARI] timeout — sadece {abs(aci):.0f} deg donuldu.")
                break
            sys.stdout.write(f"\r  aci: {abs(aci):5.1f} / {hedef_aci:.0f} deg   "
                             f"({gecen:4.1f} sn)   ")
        else:
            if gecen >= sure:
                break
            sys.stdout.write(f"\r  gecen: {gecen:4.1f} / {sure:.1f} sn   ")
        sys.stdout.flush()
        sur.gonder(surge=surge, yaw=yaw)
        time.sleep(sur.dt)
    print()


# ----------------------------------------------------------------- ozet / plan
def plani_yaz(args, sur, yaw_yon):
    duz_us = sur.pwm(surge=args.guc)
    don_us = sur.pwm(yaw=args.donus_guc * yaw_yon)
    cem_us = sur.pwm(surge=args.cember_guc, yaw=args.cember_yaw * yaw_yon)

    print("\n  ROTA:")
    toplam = 0.0
    for i, (ad, tip) in enumerate(ROTA):
        if tip == "duz":
            print(f"    {i+1}. {ad:7} duz {args.sure:g} sn   surge %{args.guc*100:.0f}")
            toplam += args.sure
        elif tip == "don":
            if args.imu:
                print(f"    {i+1}. {ad:7} {args.donus_aci:.0f} deg {args.yon} "
                      f"(jiroskop)")
                toplam += min(args.donus_timeout, 6.0)
            else:
                print(f"    {i+1}. {ad:7} {args.donus_sure:g} sn {args.yon} donus "
                      f"(acik dongu)")
                toplam += args.donus_sure
        else:
            if args.imu:
                print(f"    {i+1}. {ad:7} {args.cember_aci:.0f} deg cember "
                      f"(jiroskop)")
                toplam += args.cember_sure
            else:
                print(f"    {i+1}. {ad:7} {args.cember_sure:g} sn cember "
                      f"(acik dongu)")
                toplam += args.cember_sure
        toplam += args.ara
    print(f"    9. DUR")
    print(f"\n  Tahmini toplam sure : ~{toplam:.0f} sn")

    print("\n  GONDERILECEK PWM (notr {}us, ESC siniri {}..{}us):".format(
        PWM_NEUTRAL_US, ESC_ABS_MIN_US, ESC_ABS_MAX_US))
    print("    DUZ    ->"); pwm_yaz(duz_us, "      ")
    print("    DONUS  ->"); pwm_yaz(don_us, "      ")
    print("    CEMBER ->"); pwm_yaz(cem_us, "      ")

    if duz_us["H_L"] == PWM_NEUTRAL_US and duz_us["H_R"] == PWM_NEUTRAL_US:
        print("\n[HATA] Duz gidiste iki yatay motor da notrde — arac hic gitmez.")
        print("       --guc degerini artirin.")
        sys.exit(1)

    r = cember_yaricap_tahmini(args.cember_guc, args.cember_yaw, args.iz_genisligi)
    print("\n  CEMBER kontrolu:")
    if cem_us["H_L"] == PWM_NEUTRAL_US:
        print("    [UYARI] ic itici (H_L) olu bantta kaliyor — tek motorla donus.")
        print("            Cember cizer ama dar olur; --cember-yaw dusurun.")
    elif (cem_us["H_L"] - PWM_NEUTRAL_US) * (cem_us["H_R"] - PWM_NEUTRAL_US) < 0:
        print("    [UYARI] iki itici ZIT yonde — bu cember degil, yerinde donus!")
        print("            --cember-yaw dusurun ya da --cember-guc artirin.")
    else:
        print("    iki itici de ileri — yay cizerek ilerler (dogru).")
    if r:
        print(f"    Kaba yaricap tahmini: ~{r:.1f} m  (cap ~{2*r:.1f} m) — "
              f"iz genisligi {args.iz_genisligi:g} m varsayimiyla.")
        print("    Gercek yaricap suda olculmeli; kucukse --cember-yaw dusurun.")


# ----------------------------------------------------------------- ana akis
def main():
    args = parse_args()
    olu_bant_kontrol("--guc", args.guc)
    olu_bant_kontrol("--donus-guc", args.donus_guc)
    olu_bant_kontrol("--cember-guc", args.cember_guc)
    if args.dal > 0:
        olu_bant_kontrol("--dal", args.dal)

    yaw_yon = 1.0 if args.yon == "sag" else -1.0

    print("=" * 62)
    print("   EGE ROV — ROTA GOREVI (duz / donus / cember)")
    print("=" * 62)

    if args.sim:
        print("[MOD] SIMULASYON — motorlara gercek sinyal GITMEZ.")
        from hal.thrusters import Thrusters, MockBackend
        backend = MockBackend()
    else:
        print("[MOD] GERCEK DONANIM")
        from hal.thrusters import Thrusters, PCA9685Backend
        try:
            backend = PCA9685Backend()
        except Exception as e:
            print(f"[HATA] PCA9685'e baglanilamadi: {e}")
            sys.exit(1)

    thr = Thrusters(backend)
    sur = Surucu(thr, args)

    # Jiroskop (opsiyonel)
    gyro = None
    if args.imu:
        try:
            from sensors.imu import Mpu9250
            isaret = None if args.gyro_isaret == "oto" else float(args.gyro_isaret)
            gyro = GyroTakip(Mpu9250(), isaret)
            print("[OK] MPU-9250 jiroskop aktif — donusler aci ile sayilacak.")
        except Exception as e:
            print(f"[HATA] IMU'ya baglanilamadi: {e}")
            print("       --imu olmadan (acik dongu) calistirin.")
            sys.exit(1)

    print(f"\n  Derinlik     : ", end="")
    if args.dal > 0:
        print(f"su alti — rota boyunca %{args.dal*100:.0f} dalis itkisi")
    else:
        print("YUZEY — dikey iticiler notrde (--dal ile daldirabilirsiniz)")
    print(f"  Donus yonu   : {args.yon}")
    print(f"  Aci olcumu   : {'jiroskop (--imu)' if gyro else 'YOK — sure ile (acik dongu)'}")
    if args.ters:
        print("  --ters       : ileri/geri cevrildi")
    if args.ters_donus:
        print("  --ters-donus : sag/sol cevrildi")
    if args.ters_dal:
        print("  --ters-dal   : dalis yonu cevrildi")

    plani_yaz(args, sur, yaw_yon)

    if args.plan:
        print("\n[PLAN] --plan verildi, motorlar calistirilmadi.")
        return

    if not args.onay_yok:
        input("\n--> ESC'lere GUC VER, bip sesleri bitsin, sonra ENTER...")

    print("\n[ARM] ESC'ler arm ediliyor (2 sn notr)...")
    thr.arm()

    print("Baslamaya 3 sn. Acil durdurma: Ctrl+C")
    for kalan in (3, 2, 1):
        print(f"  {kalan}...")
        time.sleep(1.0)

    donus_sureleri = []
    try:
        for i, (ad, tip) in enumerate(ROTA):
            if tip == "duz":
                faz_duz(sur, gyro, ad, args.guc, args.sure)
            elif tip == "don":
                gecen = faz_don(sur, gyro, ad, args.donus_guc * yaw_yon,
                                args.donus_aci, args.donus_sure,
                                args.donus_timeout)
                donus_sureleri.append(gecen)
                # Ilk donuste jiroskop isaretini olc: sag donus pozitif olmali
                if gyro and gyro.isaret is None:
                    if abs(gyro.ham) > 10.0:
                        gyro.isaret = yaw_yon * math.copysign(1.0, gyro.ham)
                        print(f"  [GYRO] z ekseni isareti olculdu: "
                              f"{gyro.isaret:+.0f} — duz bacaklarda yon tutulacak.")
                    else:
                        print("  [GYRO] aci okunamadi (arac donmedi mi?) — "
                              "yon tutma devre disi.")
            else:
                faz_cember(sur, gyro, ad, args.cember_guc,
                           args.cember_yaw * yaw_yon, args.cember_aci,
                           args.cember_sure, args.cember_sure * 2.0)

            if i < len(ROTA) - 1 and args.ara > 0:
                print(f"  ...{args.ara:g} sn notr...")
                sur.notr(args.ara)

        print("\n[DUR] Rota tamamlandi — motorlar notre cekiliyor...")
        sur.heave = 0.0
        sur.notr(1.0)

    except KeyboardInterrupt:
        print("\n[IPTAL] Ctrl+C algilandi.")
    finally:
        thr.stop()
        print(f"[GUVENLI] Tum motorlar notrde ({PWM_NEUTRAL_US} us).")

    print("\n" + "-" * 62)
    print(" SONUCU DEGERLENDIR:")
    if gyro and donus_sureleri:
        ort = sum(donus_sureleri) / len(donus_sureleri)
        print(f"   Olculen 90 deg donus suresi ort. {ort:.1f} sn.")
        print(f"   IMU olmadan calistirmak icin: --donus-sure {ort:.1f}")
    if not gyro:
        print("   Donusler 90 dereceden AZ ise --donus-sure artirin, COK ise azaltin.")
    print("   Arac baslangic noktasina donmediyse: bacak sureleri esit ama")
    print("     donusler esit degil demektir — once donusu kalibre edin.")
    print("   Cember cok darsa --cember-yaw dusurun, cok genisse artirin.")
    print("   Duz bacaklarda kayma varsa iki yatay motorun itkisi esit degil")
    print("     (calibrate_escs.py) ya da --imu ile yon tutmayi acin.")
    print("-" * 62)


if __name__ == "__main__":
    main()
