"""
Eksen komutlarini 6 motora dagitan mixer + OLU BANT TELAFISI.

Eksenler (hepsi -1..+1):
  surge : ileri(+) / geri(-)
  yaw   : saga donus(+) / sola donus(-)
  heave : dalis(+) / cikis(-)      (pervane yonune gore isaret config'ten duzeltilir)
  roll  : sancak yatis duzeltmesi
  pitch : bas asagi/yukari duzeltmesi

Yerlesim:
  V_FL --- V_FR        on
   |         |
  V_RL --- V_RR        arka        H_L / H_R : yatay iticiler

==============================================================================
SORUN 3 — OLU BANT (deadband) ve TELAFISI
==============================================================================
Motorlarda kasitli bir "bosluk" var: config.PWM_DEADBAND_US = 30 us.
Amaci iyi — notre cok yakin komutlarda ESC titrer ve bip sesi cikarir,
o yuzden "cok kucuk komutlari sifir say" deniyor.

Ama hesabi yapilmamis:
    olu bant / komut araligi = 30 / 470 = 0.0638
yani komutun %6.4'unden kucuk HER SEY motora sifir olarak gidiyordu.

PID katsayilariyla birlestirince:
    heading kp=0.02 -> itki uretmek icin en az 3.2 derece hata gerekiyordu
    roll    kp=0.01 -> en az 6.4 derece
=> Roll/pitch kontrolu PRATIKTE HIC CALISMIYORDU. 6.4 derecenin altinda
   hicbir duzeltme yok, ustunde ise aniden itki basliyor (limit-cycle).

TELAFI (asagidaki `deadband_compensate`):
    komut = 0.00   -> motora 0.000  (gercekten dur, titreme yok)
    komut = 0.01   -> motora 0.074  (olu bandi atla, hafif it)
    komut = 0.50   -> motora 0.532
    komut = 1.00   -> motora 1.000
Boylece komut ile uretilen itki arasindaki iliski BASTAN SONA DOGRUSAL olur.
PID'in matematigi zaten bunu varsayar.

Not: telafi motor bazinda uygulandigi icin diferansiyel (H_L vs H_R) farki
cok az kuculur (orn. 0.100 -> 0.094). Bu, ucurumdan (0'a dusme) cok daha iyi
bir takas. Havuzda sorun cikarirsa config.DEADBAND_COMPENSATION = False ile
tek satirda kapatilabilir.
"""
import config
from config import (MOTOR_DIRECTION, THRUST_LIMIT,
                    PWM_DEADBAND_US, PWM_RANGE_US)

# NOT: THRUST_LIMIT asagida `config.THRUST_LIMIT` olarak OKUNUR, yukaridaki
# import edilmis kopya uzerinden degil. Sebep: yer istasyonundan guc sinirini
# calisirken kisabilmek (ilk havuz testlerinde sart). Modulden import edilen
# deger sabit kalir, config modulundeki nitelik ise canli degisir.
# Yukaridaki THRUST_LIMIT importu eski kodla uyum icin duruyor.

try:
    from config import DEADBAND_COMPENSATION
except ImportError:          # eski config ile de calissin
    DEADBAND_COMPENSATION = True

try:
    from config import DEADBAND_EPS
except ImportError:
    DEADBAND_EPS = 0.01

# Olu bandin normalize (0..1) karsiligi. PWM_RANGE_US notrden TEK YONDEKI paydir.
DEADBAND_N = float(PWM_DEADBAND_US) / float(PWM_RANGE_US)


def deadband_compensate(u, eps=DEADBAND_EPS):
    """Motor komutunu olu bandin ustune tasir.

    u   : -1..+1 arasi istenen komut
    eps : bu esigin altindaki komutlar GERCEKTEN sifir sayilir
          (yoksa PID gurultusu motorlari surekli titretir)

    Cikis da -1..+1 arasindadir:
        |cikis| = DEADBAND_N + (1 - DEADBAND_N) * |u|
    |u| = 1 iken cikis tam olarak 1 olur; tasma yok.
    """
    au = abs(u)
    if au < eps:
        return 0.0
    if au > 1.0:
        au = 1.0
    mag = DEADBAND_N + (1.0 - DEADBAND_N) * au
    return mag if u > 0 else -mag


def mix(surge, yaw, heave, roll=0.0, pitch=0.0):
    """Eksen komutlari -> {motor_adi: -1..+1} sozlugu."""
    m = {
        # yatay: diferansiyel surus
        "H_L": surge - yaw,
        "H_R": surge + yaw,
        # dikey: heave ortak, roll sol/sag zit, pitch on/arka zit
        "V_FL": heave + roll + pitch,
        "V_FR": heave - roll + pitch,
        "V_RL": heave + roll - pitch,
        "V_RR": heave - roll - pitch,
    }

    # doygunluk yonetimi: en buyuk |deger| 1'i asarsa hepsini oranla kucult.
    # Yatay ve dikey gruplar AYRI normalize edilir ki derinlik kontrolu
    # donus komutundan etkilenmesin (dalarken donmek derinligi bozmasin).
    for group in (("H_L", "H_R"), ("V_FL", "V_FR", "V_RL", "V_RR")):
        peak = max(abs(m[k]) for k in group)
        if peak > 1.0:
            for k in group:
                m[k] /= peak

    # SORUN 3: olu bant telafisi — normalizasyondan SONRA, yon duzeltmesinden ONCE.
    # (Once normalize et ki motorlar arasi oran korunsun; sonra her motoru
    #  olu bandin ustune tasi.)
    if DEADBAND_COMPENSATION:
        for k in m:
            m[k] = deadband_compensate(m[k])

    # yon duzeltme + genel guc siniri (canli: config.THRUST_LIMIT)
    limit = getattr(config, "THRUST_LIMIT", THRUST_LIMIT)
    for k in m:
        m[k] = m[k] * MOTOR_DIRECTION[k] * limit
    return m
