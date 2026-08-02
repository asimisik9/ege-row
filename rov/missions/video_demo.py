"""
Otonom video gosterimi gorevi (sartname 2.4.3.3, Ileri Kategori).

Dizi (tamami su altinda, otonom):
  DAL -> DUZ1(16sn) -> DON1(+90) -> DUZ2(16sn) -> DAIRE(360, >=1.2m cap)
      -> DUZ3(16sn) -> DON2(+90) -> DUZ4(16sn) -> DUR (baslangic alaninda)

==============================================================================
BU DOSYADA COZULEN SORUNLAR
==============================================================================
SORUN 6 — DAIRE SAYACI GURULTUYU TUR SANIYORDU
  Eski kod:  self._circle_acc += abs(yaw_rate) * dt
  abs() eksi isaretini atiyordu. Jiroskop hic donmeyen bir aracta bile
  bir an +0.5 bir an -0.5 okur; normalde bunlar birbirini goturur.
  abs() yuzunden GURULTU BIRIKIYORDU: ~0.5 dps gurultu x 40 sn = ~20 derece
  sahte donus. Ustelik arac TERS yone sapsa bile sayac ileri gidiyordu.
  COZUM: isaretli toplama + GYRO_NOISE_DPS altini yoksayma + duruma
         girerken sayaci sifirlama.

DAIRE CAPI ARTIK HESAPLANIYOR (plan §4.4)
  Eski: sabit yaw KOMUTU -> cap bataryaya/suruklenmeye gore degisiyordu.
  Yeni: sabit DONUS HIZI hedefi (kapali cevrim, cascade ic dongusu).
        cap = 2 * ileri_hiz / donus_hizi
  Havuzda ileri hizi bir kez olculur (Adim 8), config'teki
  MISSION["circle_yaw_rate_dps"] ona gore secilir.

SORUN 8 — AYNI SENSOR DONGUDE 3 KEZ OKUNUYORDU
  stabilizer.compute() artik dongu basina 1 kez ornekliyor; buradaki
  depth_error() ve logger o anlik goruntuyu kullaniyor, tekrar okuma yok.

YON MODU (plan §4.2)
  Duz segmentlerde 'cruise' (dusuk donus hizi + dusuk yetki: rotayi bozmaz),
  yerinde donuslerde 'turn' (tam yetki). Mod degisimi durum makinesinde.

GUVENLIK — YUZEYE CIKMA (kabul kriteri H4)
  Yuzeye cikmak ELEME sebebi. Derinlik SURFACE_GUARD_M'in ustune cikarsa
  olay loglanir ve sayac artar; prova sonrasi analiz araci bunu raporlar.
"""
import math
import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import MISSION, GYRO_NOISE_DPS
from control.mixer import mix

try:
    from config import SURFACE_GUARD_M
except ImportError:
    SURFACE_GUARD_M = 0.25


class VideoDemoMission:
    def __init__(self, stabilizer, thrusters, logger=None):
        """stabilizer: Stabilizer. thrusters: Thrusters. logger: MissionLogger.
        Gorev bir durum makinesidir; 'IDLE' ile baslar, start() COUNTDOWN'a gecirir."""
        self.stab = stabilizer
        self.thr = thrusters
        self.log = logger
        self.state = "IDLE"
        self._t0 = None           # durum baslangic zamani
        self._h0 = None           # gorev baslangic heading'i (referans)
        self._circle_acc = 0.0    # dairede biriken ISARETLI aci
        self._circle_prev_t = None
        self._circle_done = False # tur tamamlandi mi (sayac dondurulur)
        self._settle_t = None
        self.surface_violations = 0   # H4: yuzeye cikma ihlali sayaci
        self.last_axes = dict(surge=0.0, yaw=0.0, heave=0.0, roll=0.0, pitch=0.0)

    # ------------------------------------------------ durum yonetimi
    def start(self):
        """Gorevi baslatir: derinlik referansini yuzeye gore sifirlar ve
        COUNTDOWN'a gecer (geri sayim sonunda DIVE baslar)."""
        self.stab.depth.zero_at_surface()
        self._enter("COUNTDOWN")

    def abort(self):
        """Acil durdurma: motorlar notr, durum ABORT."""
        self.thr.stop()
        self.state = "ABORT"

    def _enter(self, state):
        """Yeni duruma gecer: zamanlayicilari sifirlar, olayi loglar."""
        self.state = state
        self._t0 = time.monotonic()
        self._settle_t = None
        if state == "CIRCLE":
            # SORUN 6: daireye her giriste sayaci ve saati SIFIRLA.
            # Eski kodda _circle_acc sadece __init__'te sifirlaniyordu.
            self._circle_acc = 0.0
            self._circle_prev_t = None
            self._circle_done = False
        if self.log:
            self.log.event(f"STATE -> {state}")

    def _elapsed(self):
        return time.monotonic() - (self._t0 or time.monotonic())

    def get_step_info(self):
        """Web GCS ve terminal logu icin anlik gorev durumu ve sure bilgisi."""
        M = MISSION
        durations = {
            "COUNTDOWN": M.get("start_delay_s", 10.0),
            "DIVE": M.get("dive_timeout_s", 30.0),
            "STRAIGHT1": M.get("straight_time_s", 17.0),
            "STRAIGHT2": M.get("straight_time_s", 17.0),
            "STRAIGHT3": M.get("straight_time_s", 17.0),
            "STRAIGHT4": M.get("straight_time_s", 17.0),
            "TURN1": M.get("turn_timeout_s", 20.0),
            "TURN2": M.get("turn_timeout_s", 20.0),
            "CIRCLE": 30.0,
            "FINISH": 3.0,
        }
        elapsed = self._elapsed()
        target_dur = 1.0 if self.state.startswith("PAUSE_") else durations.get(self.state, 0.0)
        progress = min(100.0, (elapsed / target_dur * 100.0)) if target_dur > 0 else 0.0
        return {
            "step": self.state,
            "elapsed_s": round(elapsed, 1),
            "duration_s": round(target_dur, 1),
            "progress_pct": round(progress, 1),
        }

    # ------------------------------------------------ ana dongu adimi
    def step(self):
        """Kontrol dongusunden LOOP_HZ frekansinda cagrilir.
        Gorev bittiyse True dondurur."""
        M = MISSION
        s = self.state

        if s in ("IDLE", "ABORT", "DONE"):
            return s == "DONE"

        if s == "COUNTDOWN":
            if self._elapsed() >= M["start_delay_s"]:
                self.thr.arm()
                self._enter("DIVE")
            return False

        # ---- DALIS -------------------------------------------------------
        if s == "DIVE":
            self.stab.set_targets(depth_m=M["target_depth_m"])
            depth_err = self.stab.depth_error()

            # ---- Iki fazli dalis ----
            # Faz 1 (Guc): Hedef derinligin 2x tolerans disindayken tam guc.
            #   Roll/Pitch PID DEVRE DISI - bu kompanzasyon dikey motor gucunu
            #   normalize ederek azaltiyor; dalis sirasinda gereksiz.
            # Faz 2 (PID): Hedefe yakinken ince ayar (PID devreye girer).
            if abs(depth_err) > M["depth_tol_m"] * 2:
                # Tam guc dalis: hata isareti hangi yondeyse o yonde 1.0
                dive_pwr = math.copysign(M.get("dive_power", 1.0), depth_err)
                self.thr.command(mix(surge=0.0, yaw=0.0, heave=dive_pwr))
                if self.log:
                    self.log.event(
                        f"DIVE_POWER heave={dive_pwr:.2f} err={depth_err:.3f}m"
                    )
            else:
                # Hedefe yakin: PID ince kontrolu (roll/pitch dahil)
                axes = self.stab.compute(surge=0.0)
                self._apply(axes)

            ok = abs(depth_err) < M["depth_tol_m"]
            if ok or self._elapsed() > M["dive_timeout_s"]:
                if self._elapsed() > M["dive_timeout_s"] and not ok:
                    print(f"[UYARI] Dalis timeout ({M['dive_timeout_s']:.0f}s) - "
                          f"hedef derinlige ulassamadi! (err={depth_err:.2f}m) "
                          "MOTOR_DIRECTION veya itki gucunu kontrol et.")
                self._h0 = self.stab.ori.heading or 0.0  # referans heading

                self.stab.set_targets(heading_deg=self._h0)
                self.stab.set_heading_mode("cruise")
                if self.log:
                    self.log.event(f"REFERANS HEADING h0 = {self._h0:.1f} deg")
                self._enter("STRAIGHT1")
            return False

        # ---- PAUSE (HER ADIM SONRASI 1.0s BEKLEME) ------------------------
        if s.startswith("PAUSE_"):
            next_st = s.replace("PAUSE_", "")
            self.stab.set_targets(depth_m=M["target_depth_m"])
            axes = self.stab.compute(surge=0.0)  # dur, yerinde kal
            self._apply(axes)
            if self._elapsed() >= 1.0:  # 1.0 saniye durakla
                self._enter(next_st)
            return False

        # ---- DUZ GIDISLER -------------------------------------------------
        # Sadece STRAIGHT1 (15sn) ve STRAIGHT2 (15sn) aktif; digerleri yorum satirinda.
        for st, next_st, hdg_off in (("STRAIGHT1", "TURN1", 0),
                                     ("STRAIGHT2", "FINISH", -90)):
                                     # ("STRAIGHT3", "TURN3", -180),
                                     # ("STRAIGHT4", "FINISH", -270)):
            if s == st:
                self.stab.set_heading_mode("cruise")
                self.stab.set_targets(depth_m=M["target_depth_m"],
                                      heading_deg=self._h0 + hdg_off)
                axes = self.stab.compute(surge=M["cruise_throttle"])
                self._apply(axes)
                if self._elapsed() >= M["straight_time_s"]:
                    self._enter(f"PAUSE_{next_st}")
                return False

        # ---- 90 DERECE DONUSLER (yerinde) ---------------------------------
        # Sadece TURN1 (SAGA 90) aktif; digerleri yorum satirinda.
        for st, next_st, target_off in (("TURN1", "STRAIGHT2", -90),):
                                        # ("TURN2", "STRAIGHT3", -180),
                                        # ("TURN3", "STRAIGHT4", -270)):
            if s == st:
                self.stab.set_heading_mode("turn")
                self.stab.set_targets(depth_m=M["target_depth_m"],
                                      heading_deg=self._h0 + target_off)
                axes = self.stab.compute(surge=0.0)
                self._apply(axes)
                if self._turn_done(M):
                    self._enter(f"PAUSE_{next_st}")
                elif self._elapsed() > M["turn_timeout_s"]:
                    if self.log:
                        self.log.event(f"UYARI: {st} zaman asimi "
                                       f"(hata {self.stab.heading_error():.1f} deg)")
                    self._enter(f"PAUSE_{next_st}")
                return False

        # ---- DAIRE (GECICI OLARAK YORUM SATIRINDA / DEVRE DISI) -----------
        """
        if s == "CIRCLE":
            now = time.monotonic()
            dt = 0.0 if self._circle_prev_t is None else (now - self._circle_prev_t)
            self._circle_prev_t = now

            w = self.stab.snap.yaw_rate if self.stab.snap else 0.0
            if not self._circle_done and abs(w) > GYRO_NOISE_DPS:
                self._circle_acc += w * dt
                if abs(self._circle_acc) >= M["circle_deg"]:
                    self._circle_done = True
                    if self.log:
                        self.log.event(f"DAIRE 360 tamamlandi: {self._circle_acc:.1f} deg")

            if not self._circle_done:
                self.stab.set_heading_mode("circle")
                self.stab.set_targets(depth_m=M["target_depth_m"])
                axes = self.stab.compute(
                    surge=M["circle_throttle"],
                    yaw_rate_target=M["circle_yaw_rate_dps"])
                self._apply(axes)
            else:
                self.stab.set_heading_mode("turn")
                self.stab.set_targets(depth_m=M["target_depth_m"],
                                      heading_deg=self._h0 + 180)
                axes = self.stab.compute(surge=0.0)
                self._apply(axes)
                if self._turn_done(M) or self._elapsed() > (M["turn_timeout_s"] + 60.0):
                    self._circle_prev_t = None
                    self._enter("STRAIGHT3")
            return False
        """

        # ---- BITIS --------------------------------------------------------
        if s == "FINISH":
            # Ileri gaz yok; derinlik ve yon tutulmaya devam eder.
            # (Yuzeye cikmak yasak — su altinda duruyoruz.)
            self.stab.set_heading_mode("turn")
            self.stab.set_targets(depth_m=M["target_depth_m"],
                                  heading_deg=self._h0 - 90)
            axes = self.stab.compute(surge=0.0)
            self._apply(axes)
            if self._elapsed() > 3.0:
                self.thr.command(mix(0.0, 0.0, 0.0))
                self._enter("DONE")
            return False

        return False

    # ------------------------------------------------ yardimcilar
    def _apply(self, axes):
        """Eksen komutlarini mixer ile motor komutlarina cevirip gonderir,
        loga bir veri satiri yazar ve yuzeye cikma ihlalini kontrol eder."""
        self.last_axes = axes
        self.thr.command(mix(**axes))

        # H4 kabul kriteri: yuzeye cikmak ELEME sebebi.
        if self.state not in ("IDLE", "COUNTDOWN", "DIVE") and \
                self.stab.depth_m < SURFACE_GUARD_M:
            self.surface_violations += 1
            if self.log and self.surface_violations % 20 == 1:
                self.log.event(f"!!! YUZEY IHLALI: derinlik "
                               f"{self.stab.depth_m:.2f} m < {SURFACE_GUARD_M} m")

        if self.log:
            self.log.sample(self.state, self.stab, axes, self.thr)

    def _turn_done(self, M):
        """Yon hedefe ulastigi an BEKLEMEDEN (aninda) tamamla — fazladan donusu onler."""
        return abs(self.stab.heading_error()) < M.get("turn_tol_deg", 5.0)
