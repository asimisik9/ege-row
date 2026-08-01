"""
Gorev 1 — Hat Takibi + Mini ROV Konuslandirma durum makinesi.

Durum akisi:
  IDLE
  → DESCEND      : hedef derinlige in
  → FOLLOW_LINE  : hat takibi (line_tracker → yaw PID → surge)
  → APPROACH_PIPE: hatta son, yavasla, boru arayi
  → ALIGN_PIPE   : surj sifir, boru merkezle (yaw + heave)
  → DEPLOY       : vinc servo → Mini ROV birak
  → WAIT_MINROV  : operatör FPV ile Mini ROV yonetiyor; 'geri' komutu bekle
  → RETRACT      : vinc servo → Mini ROV cek
  → ASCEND       : giris derinligine geri cik
  → DONE

Gorev 1 degerlendirme (sartnameden):
  - Hat takibi otonom olmali
  - Boru girisine hizalanma
  - Mini ROV konuslandirma (operatör manuel)
  - Mini ROV geri cekme
"""
import time
import threading

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config import (LOOP_HZ, MISSION, LINE_TARGET_DEPTH, LINE_PIPE_DEPTH,
                    LINE_CRUISE_SURGE, LINE_APPROACH_SURGE,
                    VISION_KP, VISION_MAX_YAW, PIPE_ALIGN_TOL)
from control.mixer import mix
from control.pid import PID


class LineFollowMission:
    """
    Non-blocking step() tabanli durum makinesi.
    Her LOOP_HZ cagrisiyla bir adim ilerler.
    """

    # Hangi durumda hat yoksa kac saniye tolerans
    _LINE_LOST_TIMEOUT = 3.0
    # Boru hizalama max sure
    _PIPE_ALIGN_TIMEOUT = 20.0
    # Mini ROV geri cekme bekleme max suresi (operatör bitirdi sinyali)
    _WAIT_MINROV_TIMEOUT = 300.0  # 5 dk

    def __init__(self, stabilizer, thrusters, camera, logger=None):
        """
        stabilizer : Stabilizer
        thrusters  : Thrusters
        camera     : sensors.camera.Camera
        logger     : utils.logger.MissionLogger (opsiyonel)
        """
        from vision.line_tracker import LineTracker
        from vision.pipe_aligner import PipeAligner
        from hal.winch import Winch

        self.stab    = stabilizer
        self.thr     = thrusters
        self.cam     = camera
        self.log     = logger
        self.state   = "IDLE"

        self._tracker = LineTracker()
        self._aligner = PipeAligner()
        self._winch   = Winch(thrusters.backend)   # PCA9685Backend paylas

        # Gorev kontrolune cevrilmis "mini rov geri don" eventi
        self._minrov_back = threading.Event()

        # Gorsel yaw PID (piksel hatasi -> yaw komutu)
        self._vis_pid = PID(kp=VISION_KP, ki=0.0, kd=0.0002,
                            out_limit=VISION_MAX_YAW)

        self._t0   = None   # mevcut durum baslangic zamani
        self._line_lost_t = None

    # ---------------------------------------------------------------- public
    def start(self):
        """Gorevi baslatir."""
        self.stab.depth.zero_at_surface()
        self._enter("DESCEND")

    def abort(self):
        """Acil durdurma."""
        self.thr.stop()
        self.state = "ABORT"

    def signal_minrov_back(self):
        """
        Operatör / comms sunucusu Mini ROV'un geri geldigini bildirdigi anda
        bu metodu cagirir → WAIT_MINROV durumundan cikilir.
        """
        self._minrov_back.set()

    def step(self):
        """Kontrol dongusu adimi. True: gorev bitti."""
        s = self.state
        M = MISSION

        if s in ("IDLE", "ABORT", "DONE"):
            return s == "DONE"

        # ── DAL
        if s == "DESCEND":
            self.stab.set_targets(depth_m=LINE_TARGET_DEPTH)
            axes = self.stab.compute(surge=0.0)
            self._apply(axes)
            ok = abs(self.stab.depth_error()) < M["depth_tol_m"]
            if ok or self._elapsed() > M["dive_timeout_s"]:
                self._enter("FOLLOW_LINE")
            return False

        # ── HAT TAKİBİ
        if s == "FOLLOW_LINE":
            frame = self.cam.read()
            result = self._tracker.process(frame)

            if result.found:
                self._line_lost_t = None
                yaw = self._vis_pid.update(result.error_px)
                surge = LINE_CRUISE_SURGE
                # Hat guven dusukse yavasla
                if result.confidence < 0.3:
                    surge *= 0.5
            else:
                # Hat kayip: yav yavaşla, saymaya basla
                yaw = 0.0
                surge = 0.0
                if self._line_lost_t is None:
                    self._line_lost_t = time.monotonic()
                elif time.monotonic() - self._line_lost_t > self._LINE_LOST_TIMEOUT:
                    # Hat cok uzun sure kayip → boru aramaya gec
                    print("[GOREV1] Hat kaybedildi — boru arama moduna geciliyor.")
                    self._enter("APPROACH_PIPE")
                    return False

            self.stab.set_targets(depth_m=LINE_TARGET_DEPTH,
                                  heading_deg=self.stab.ori.heading)  # heading'i tut
            axes = self.stab.compute(surge=surge, yaw_override=yaw)
            self._apply(axes)
            return False

        # ── BORUYA YAKLAS
        if s == "APPROACH_PIPE":
            # Yavas ilerle ve boru ara
            frame = self.cam.read()
            pipe = self._aligner.process(frame)
            if pipe.found and pipe.radius_px > 60:
                # Boru yeterince buyuk: hizalamaya gec
                self._enter("ALIGN_PIPE")
                return False
            # Yaklasma: yavaş surge
            self.stab.set_targets(depth_m=LINE_PIPE_DEPTH)
            axes = self.stab.compute(surge=LINE_APPROACH_SURGE)
            self._apply(axes)
            if self._elapsed() > self._PIPE_ALIGN_TIMEOUT:
                # Boru hic bulunamadi — dogrudan deploy
                self._enter("DEPLOY")
            return False

        # ── BORU HİZALA
        if s == "ALIGN_PIPE":
            frame = self.cam.read()
            pipe = self._aligner.process(frame)
            if pipe.found:
                # X hatasi → yaw  |  Y hatasi → heave
                yaw_cmd   = max(-VISION_MAX_YAW,
                                min(VISION_MAX_YAW, pipe.x_error * VISION_KP * 2))
                heave_cmd = max(-0.3, min(0.3, pipe.y_error * 0.002))
                self.stab.set_targets(depth_m=LINE_PIPE_DEPTH)
                axes = self.stab.compute(surge=0.0, yaw_override=yaw_cmd)
                axes["heave"] = heave_cmd   # derinlik PID'i override et
                self._apply(axes)
                if pipe.aligned or self._elapsed() > self._PIPE_ALIGN_TIMEOUT:
                    self._enter("DEPLOY")
            else:
                # Boru gorunmuyor: dur ve bekle
                self.stab.set_targets(depth_m=LINE_PIPE_DEPTH)
                axes = self.stab.compute(surge=0.0)
                self._apply(axes)
                if self._elapsed() > self._PIPE_ALIGN_TIMEOUT:
                    self._enter("DEPLOY")
            return False

        # ── MINI ROV BIRAK
        if s == "DEPLOY":
            self.thr.stop()   # motorlari durdur (vinc calisirken titresim azalt)
            # Vinc blocking fonksiyon — gorev dongusu burada bekler
            self._winch.deploy()
            self._enter("WAIT_MINROV")
            return False

        # ── MINI ROV BEKLE (operatör)
        if s == "WAIT_MINROV":
            # Mini ROV manuel moddayken operatör comms.client ile kontrol eder.
            # signal_minrov_back() cagirilinca ya da timeout dolunca devam et.
            if self._minrov_back.is_set():
                self._enter("RETRACT")
            elif self._elapsed() > self._WAIT_MINROV_TIMEOUT:
                print("[GOREV1] Mini ROV bekleme zaman asimi — geri cekiliyor.")
                self._enter("RETRACT")
            return False

        # ── MINI ROV CEK
        if s == "RETRACT":
            self._winch.retract()
            self._enter("ASCEND")
            return False

        # ── YOL NOKTASINA CIK
        if s == "ASCEND":
            # Baslangic derinligine yuksel (0.3m)
            self.stab.set_targets(depth_m=0.3)
            axes = self.stab.compute(surge=0.0)
            self._apply(axes)
            if abs(self.stab.depth_error()) < M["depth_tol_m"] \
                    or self._elapsed() > 15.0:
                self._enter("DONE")
            return False

        return False  # bilinmeyen durum

    # ---------------------------------------------------------------- helpers
    def _enter(self, state):
        self.state = state
        self._t0 = time.monotonic()
        self._line_lost_t = None
        print(f"[GOREV1] → {state}")
        if self.log:
            self.log.event(f"STATE -> {state}")

    def _elapsed(self):
        return time.monotonic() - self._t0

    def _apply(self, axes):
        """Eksen komutlarini mixer'dan gec, motora gonder, logla."""
        # Eksik anahtarlari 0.0 ile tamamla (guvenli)
        full = {"surge": 0.0, "yaw": 0.0, "heave": 0.0, "roll": 0.0, "pitch": 0.0}
        full.update(axes)
        self.thr.command(mix(**full))
        if self.log:
            self.log.sample(self.state, self.stab, full)
