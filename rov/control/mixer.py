"""
Eksen komutlarini 6 motora dagitan mixer.

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
"""
from config import MOTOR_DIRECTION, THRUST_LIMIT


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

    # doygunluk yonetimi: en buyuk |deger| 1'i asarsa hepsini oranla kucult
    # (yatay ve dikey gruplari ayri normalize edilir ki derinlik kontrolu
    #  donus komutundan etkilenmesin)
    for group in (("H_L", "H_R"), ("V_FL", "V_FR", "V_RL", "V_RR")):
        peak = max(abs(m[k]) for k in group)
        if peak > 1.0:
            for k in group:
                m[k] /= peak

    # yon duzeltme + genel guc siniri
    for k in m:
        m[k] = m[k] * MOTOR_DIRECTION[k] * THRUST_LIMIT
    return m
