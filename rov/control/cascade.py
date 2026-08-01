"""
Kaskad (ic ice) yon kontrolu — YENI DOSYA.

NEDEN VAR (PID_BASIT_ANLATIM.md §5.2):

  ESKI YAPI (tek PID):
      "90 derece hata var  ->  su kadar gaz ver"
  Sorun: "su kadar gaz" aracin saniyede kac derece dondugunu belirlemiyor.
  Batarya doluyken hizli, boskken yavas doner. Motor sicakligi, su akintisi,
  pervane kirliligi... hepsi degistiriyor. Havuzda bulunan katsayi
  yarismada ayni davranmiyor.

  YENI YAPI (iki katman):
      DIS KATMAN (yavas, "NEREYE"):
          hata 90 derece  ->  "saniyede 30 derece hizla don"
          ^ buraya UST SINIR koyabiliyoruz: w_max
      IC KATMAN (hizli, "NE KADAR"):
          hedef 30 dps, jiroskop 22 dps diyor  ->  "gazi biraz artir"

  KAZANCLAR:
    1. Donus hizi dogrudan sinirlanir  -> hedefi asma (overshoot) buyuk
       olcude biter. Bu, TURN1/TURN2'de kritik.
    2. Ic katman jiroskopu OLCUM olarak kullanir. Donus hizini zaten
       olcuyoruz; turev almaya, gurultuyle bogusmaya gerek yok.
    3. Batarya/motor/akinti farklarini ic katman emer; dis katman hep
       ayni davranir. Yani havuzda buldugumuz ayar yarismada da gecerli.
    4. DAIRE gorevi zaten "sabit donus hizi" istiyor -> ayni ic katman
       `update_rate()` ile tekrar kullaniliyor, ayri kod yok.
       (bkz. missions/video_demo.py CIRCLE durumu)

Benzetme: arabada "su kavsakta don" (dis katman) ile "hiz sabitleyici 50"
(ic katman). Hiz sabitleyici yokusu ve ruzgari kendi halleder; sen sadece
nereye gidecegini soylersin.
"""
from control.pid import PID, angle_error_deg, clamp


class HeadingController:
    """Dis P (aci -> istenen donus hizi) + ic PI (donus hizi -> motor komutu)."""

    def __init__(self, pos_cfg, rate_cfg, modes):
        """
        pos_cfg  : dict(kp=...)                      dis katman
        rate_cfg : dict(kp=, ki=, kd=, i_limit=, d_tau=)  ic katman
        modes    : dict(cruise=dict(w_max_dps=, out_limit=),
                        turn  =dict(w_max_dps=, out_limit=))
        """
        self.kp_pos = float(pos_cfg["kp"])
        self.modes = modes
        self.rate = PID(
            kp=rate_cfg["kp"], ki=rate_cfg["ki"], kd=rate_cfg.get("kd", 0.0),
            out_limit=modes["turn"]["out_limit"],
            i_limit=rate_cfg.get("i_limit", 0.25),
            d_tau=rate_cfg.get("d_tau", 0.10),
            name="yaw_rate")
        self.mode = "turn"
        self.set_mode("cruise")
        # telemetri (logger okuyor)
        self.last = dict(err=0.0, w_target=0.0, w_meas=0.0, out=0.0)

    # ------------------------------------------------------------------ modlar
    def set_mode(self, mode):
        """'cruise' (duz seyir) veya 'turn' (yerinde donus).

        Neden iki mod:
          - Duz seyirde araci saniyede 30 derece dondurmek istemeyiz; rota
            bozulur. Hiz sinirini 15 dps'e cekip yetkiyi 0.35'e dusururuz,
            boylece heading duzeltmesi ileri gidisi bozmaz.
          - Yerinde donuste ise hizli ve tam yetkiyle donmesini isteriz.
        """
        if mode == self.mode:
            return
        if mode not in self.modes:
            raise ValueError(f"bilinmeyen mod: {mode}")
        self.mode = mode
        cfg = self.modes[mode]
        self.w_max = float(cfg["w_max_dps"])
        # out_limit degisirken I birikimini SIFIRLAMA (reset=False):
        # duz segment icinde mod degismiyor; degisirse de birikim anlamli kalir.
        self.rate.set_params(out_limit=float(cfg["out_limit"]), reset=False)

    def reset(self):
        """Yeni bir hedefe gecerken cagrilir — ic PID'in I birikimini temizler."""
        self.rate.reset()

    # ------------------------------------------------------------ ana kullanim
    def update_heading(self, target_deg, heading_deg, yaw_rate_dps, now=None):
        """ACI hedefi ile calisir (duz segmentler ve 90 derece donusler).

        target_deg   : hedef pusula acisi
        heading_deg  : olculen pusula acisi
        yaw_rate_dps : jiroskoptan olculen donus hizi (derece/sn)
        """
        err = angle_error_deg(target_deg, heading_deg)

        # DIS KATMAN: aciyi istenen donus hizina cevir, sinirla.
        w_target = clamp(self.kp_pos * err, -self.w_max, self.w_max)

        # IC KATMAN: istenen donus hizini yakala.
        out = self.rate.update(w_target - yaw_rate_dps, meas_rate=yaw_rate_dps, now=now)

        self.last = dict(err=err, w_target=w_target,
                         w_meas=yaw_rate_dps, out=out)
        return out

    def update_rate(self, w_target_dps, yaw_rate_dps, now=None):
        """DOGRUDAN donus hizi hedefi ile calisir (DAIRE gorevi).

        Eski kodda daireye sabit bir yaw KOMUTU veriliyordu; capi bataryaya
        ve suruklenmeye gore degisiyordu. Burada kapali cevrim oldugu icin
        cap tekrarlanabilir olur:   cap = 2 * ileri_hiz / donus_hizi
        """
        out = self.rate.update(w_target_dps - yaw_rate_dps,
                               meas_rate=yaw_rate_dps, now=now)
        self.last = dict(err=0.0, w_target=w_target_dps,
                         w_meas=yaw_rate_dps, out=out)
        return out

    # ------------------------------------------- web arayuzu / pid_tune uyumu
    def get_params(self):
        p = self.rate.get_params()
        p["kp_pos"] = self.kp_pos
        p["w_max"] = self.w_max
        p["mode"] = self.mode
        return p

    def set_params(self, kp_pos=None, w_max=None, reset=True, **rate_kwargs):
        if kp_pos is not None:
            self.kp_pos = float(kp_pos)
        if w_max is not None:
            self.w_max = float(w_max)
        if rate_kwargs:
            self.rate.set_params(reset=reset, **rate_kwargs)
        elif reset:
            self.rate.reset()
