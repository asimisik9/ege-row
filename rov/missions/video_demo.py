"""
Otonom video gosterimi gorevi (sartname 2.4.3.3, Ileri Kategori).

Dizi (tamami su altinda, otonom):
  DAL -> DUZ1(15sn) -> DON1(+90) -> DUZ2(15sn) -> DAIRE(360, >=1m cap)
      -> DUZ3(15sn) -> DON2(+90) -> DUZ4(15sn) -> DUR (baslangic alaninda)

Rota kapali dikdortgen: DUZ1..DUZ4 esit sure/hiz -> araca basa dondurur.
Daire kosede cizilir, cikista heading daire oncesinden +90 olmalidir;
bunu jiroskop toplami (360 sayma) + son heading kilidi ile sagliyoruz.
"""
import time
from config import MISSION
from control.mixer import mix
from control.pid import angle_error_deg


class VideoDemoMission:
    def __init__(self, stabilizer, thrusters, logger=None):
        """stabilizer: Stabilizer (PID hesaplarini yapar). thrusters: Thrusters
        (motor komutlarini gonderir). logger: opsiyonel MissionLogger.
        Gorev bir durum makinesi (state machine) olarak isler; state="IDLE"
        ile baslar, start() COUNTDOWN'a gecirir."""
        self.stab = stabilizer
        self.thr = thrusters
        self.log = logger
        self.state = "IDLE"
        self._t0 = None          # durum baslangic zamani
        self._h0 = None          # gorev baslangic heading'i
        self._circle_acc = 0.0   # dairede biriken aci
        self._prev_t = None
        self._settle_t = None

    # ------------------------------------------------ durum yonetimi
    def start(self):
        """Gorevi baslatir: derinlik sensorunu su yuzeyine gore sifirlar
        ve durumu COUNTDOWN'a geciler (geri sayim sonunda DIVE baslar)."""
        self.stab.depth.zero_at_surface()
        self._enter("COUNTDOWN")

    def abort(self):
        """Gorevi acil durdurur: motorlari notre ceker, durumu ABORT yapar."""
        self.thr.stop()
        self.state = "ABORT"

    def _enter(self, state):
        """Durum makinesinde yeni bir duruma gecer: durum zamanlayicisini
        (_t0) ve settle zamanlayicisini sifirlar, olayi loglar."""
        self.state = state
        self._t0 = time.monotonic()
        self._settle_t = None
        if self.log:
            self.log.event(f"STATE -> {state}")

    def _elapsed(self):
        """Mevcut duruma girildiginden bu yana gecen sure (saniye)."""
        return time.monotonic() - self._t0

    # ------------------------------------------------ ana dongu adimi
    def step(self):
        """Kontrol dongusunden LOOP_HZ frekansinda cagrilir.
        Gorev bitti ise True dondurur."""
        M = MISSION
        s = self.state

        if s in ("IDLE", "ABORT", "DONE"):
            return s == "DONE"

        if s == "COUNTDOWN":
            if self._elapsed() >= M["start_delay_s"]:
                self.thr.arm()
                self._enter("DIVE")
            return False

        if s == "DIVE":
            self.stab.set_targets(depth_m=M["target_depth_m"])
            axes = self.stab.compute(surge=0.0)
            self._apply(axes)
            ok = abs(self.stab.depth_error()) < M["depth_tol_m"]
            if ok or self._elapsed() > M["dive_timeout_s"]:
                self._h0 = self.stab.ori.heading  # referans heading
                self.stab.set_targets(heading_deg=self._h0)
                self._enter("STRAIGHT1")
            return False

        # ---- duz gidisler ----
        for i, (st, next_st, hdg_off) in enumerate((
                ("STRAIGHT1", "TURN1", 0),
                ("STRAIGHT2", "CIRCLE", 90),
                ("STRAIGHT3", "TURN2", 180),
                ("STRAIGHT4", "FINISH", 270))):
            if s == st:
                self.stab.set_targets(depth_m=M["target_depth_m"],
                                      heading_deg=self._h0 + hdg_off)
                axes = self.stab.compute(surge=M["cruise_throttle"])
                self._apply(axes)
                if self._elapsed() >= M["straight_time_s"]:
                    self._enter(next_st)
                return False

        # ---- 90 derece donusler (yerinde) ----
        for st, next_st, target_off in (("TURN1", "STRAIGHT2", 90),
                                        ("TURN2", "STRAIGHT4", 270)):
            if s == st:
                self.stab.set_targets(depth_m=M["target_depth_m"],
                                      heading_deg=self._h0 + target_off)
                axes = self.stab.compute(surge=0.0)
                self._apply(axes)
                if self._turn_done(M):
                    self._enter(next_st)
                elif self._elapsed() > M["turn_timeout_s"]:
                    self._enter(next_st)  # takilmamak icin devam (log'a duser)
                return False

        # ---- daire ----
        if s == "CIRCLE":
            now = time.monotonic()
            dt = 0.0 if self._prev_t is None else now - self._prev_t
            self._prev_t = now
            self._circle_acc += abs(self.stab.ori.yaw_rate) * dt

            if self._circle_acc < M["circle_deg"]:
                # sabit donus + ileri: heading PID devre disi (yaw_override)
                self.stab.set_targets(depth_m=M["target_depth_m"])
                axes = self.stab.compute(surge=M["circle_throttle"],
                                         yaw_override=M["circle_yaw_rate"])
                self._apply(axes)
            else:
                # tur tamam: daire cikis heading'ine kilitlen (h0+180)
                self.stab.set_targets(depth_m=M["target_depth_m"],
                                      heading_deg=self._h0 + 180)
                axes = self.stab.compute(surge=0.0)
                self._apply(axes)
                if self._turn_done(M):
                    self._prev_t = None
                    self._enter("STRAIGHT3")
            return False

        if s == "FINISH":
            # motorlari notre cek, su altinda bekle (yuzeye cikmak yasak)
            self.thr.command(mix(0, 0, 0))
            axes = self.stab.compute(surge=0.0)  # derinligi tutmaya devam
            self._apply(axes)
            if self._elapsed() > 3.0:
                self._enter("DONE")
            return False

        return False

    # ------------------------------------------------ yardimcilar
    def _apply(self, axes):
        """Stabilizer'dan gelen eksen komutlarini (surge/yaw/heave/roll/pitch)
        mixer ile motor komutlarina cevirip gonderir ve varsa loga bir
        veri satiri yazar."""
        self.thr.command(mix(**axes))
        if self.log:
            self.log.sample(self.state, self.stab, axes)

    def _turn_done(self, M):
        """Heading hedefe geldi ve settle suresi kadar orada kaldi mi?"""
        if abs(self.stab.heading_error()) < M["turn_tol_deg"]:
            if self._settle_t is None:
                self._settle_t = time.monotonic()
            elif time.monotonic() - self._settle_t >= M["turn_settle_s"]:
                return True
        else:
            self._settle_t = None
        return False
