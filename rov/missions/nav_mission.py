"""
Gorev 2 — Otonom Navigasyon durum makinesi.

KTR ve sartname tabanli akis:
  IDLE
  → SURFACE_GPS_FIX   : yuzey GPS referansini al
  → DESCEND           : gorev derinligine in
  → NAVIGATE_TO_ZONE  : olu hesap / heading ile hedef bolgede ilerle
  → SONAR_SCAN        : yerinde don, Ping Sonar ile samandira ara
  → APPROACH_BUOY     : kamera + heading ile samandiraya yaklas (~1m)
  → ORBIT_BUOY        : 360°+ cember — min 1m cap
  → EXIT_ZONE         : cikis koordinatina ilerle
  → ASCEND            : yuzey koridor icinde yuksel
  → DONE

Sartnameden:
  - GPS + IMU fuzyon navigasyon
  - Ping Sonar samandira algilama
  - Kamera merkezleme (gorunce)
  - 1m yaricap cember (= 2m cap)
  - Samandira etrafinda 360°+
  - Kose samandiralarini gorünce yuzey cikis merkezi hesapla
"""
import math
import time

from config import (LOOP_HZ, MISSION, NAV_TARGET_DEPTH, NAV_CRUISE_SURGE,
                    BUOY_ORBIT_RADIUS_M, BUOY_ORBIT_SPEED, BUOY_ORBIT_YAW,
                    BUOY_ORBIT_DEG, GPS_TIMEOUT_S, VISION_KP, VISION_MAX_YAW)
from control.mixer import mix
from control.dead_reckoning import DeadReckoning


class NavMission:
    """
    Non-blocking step() tabanli navigasyon gorevi.

    stabilizer : Stabilizer
    thrusters  : Thrusters
    camera     : sensors.camera.Camera
    gps        : sensors.gps.GPS
    sonar      : sensors.ping_sonar.PingSonar
    logger     : utils.logger.MissionLogger (opsiyonel)
    """

    # Samandiraya yaklasma mesafesi (kameradan: gorece buyukluk taban)
    _BUOY_APPROACH_DIST_MM = 1000   # sonar ile ~1m = 1000mm
    # Samandira kayip sayac (kac adim kayipsa vazgec)
    _SONAR_SCAN_STEP_DEG   = 5.0    # tarama adimi
    _SONAR_SCAN_MAX_DEG    = 360.0  # tam tur tarama

    def __init__(self, stabilizer, thrusters, camera, gps, sonar, logger=None):
        from vision.pipe_aligner import PipeAligner  # buoy merkezlemede kullanilir
        self.stab  = stabilizer
        self.thr   = thrusters
        self.cam   = camera
        self.gps   = gps
        self.sonar = sonar
        self.log   = logger
        self.state = "IDLE"

        self._dr   = DeadReckoning()
        self._t0   = None

        # GPS referans
        self._ref_lat = None
        self._ref_lon = None

        # Samandira tarama
        self._scan_start_hdg = None
        self._scan_total_deg = 0.0

        # Orbit
        self._orbit_deg = 0.0
        self._orbit_start_hdg = None

        # Hedef konum (basit: ileri git ~ N metre)
        # Gercek parkur koordinatlari saha testinde girilir
        self._target_dist_m = 20.0  # tahmini alan mesafesi (m)

    # ---------------------------------------------------------------- public
    def start(self):
        self.stab.depth.zero_at_surface()
        self._enter("SURFACE_GPS_FIX")

    def abort(self):
        self.thr.stop()
        self.state = "ABORT"

    def step(self) -> bool:
        """Kontrol dongusunde cagirilir. True: gorev bitti."""
        s = self.state
        M = MISSION

        if s in ("IDLE", "ABORT", "DONE"):
            return s == "DONE"

        # ── GPS FIX AL (yuzey)
        if s == "SURFACE_GPS_FIX":
            fix = self.gps.fix
            if fix is not None:
                self._ref_lat = fix.lat
                self._ref_lon = fix.lon
                self._dr.reset()
                print(f"[GOREV2] GPS fix: {fix.lat:.6f}, {fix.lon:.6f}")
                self._enter("DESCEND")
            elif self._elapsed() > GPS_TIMEOUT_S:
                print("[GOREV2] GPS fix alinamadi — olu hesapla devam ediliyor.")
                self._dr.reset()
                self._enter("DESCEND")
            return False

        # ── DERINLIGE IN
        if s == "DESCEND":
            self.stab.set_targets(depth_m=NAV_TARGET_DEPTH)
            axes = self.stab.compute(surge=0.0)
            self._apply(axes, 0.0)
            ok = abs(self.stab.depth_error()) < M["depth_tol_m"]
            if ok or self._elapsed() > M["dive_timeout_s"]:
                self._enter("NAVIGATE_TO_ZONE")
            return False

        # ── HEDEF BOLGEYE ILERLE
        if s == "NAVIGATE_TO_ZONE":
            surge = NAV_CRUISE_SURGE
            heading = self.stab.ori.heading or 0.0
            self.stab.set_targets(depth_m=NAV_TARGET_DEPTH,
                                  heading_deg=heading)  # heading'i tut
            axes = self.stab.compute(surge=surge)
            dist = self._dr.distance_to(0.0, self._target_dist_m)
            self._apply(axes, surge)
            if dist < 2.0 or self._elapsed() > 120.0:
                # Hedef bolgeye yeterince yaklastik
                self._enter("SONAR_SCAN")
            return False

        # ── SONAR TARAMA
        if s == "SONAR_SCAN":
            if self._scan_start_hdg is None:
                self._scan_start_hdg = self.stab.ori.heading or 0.0
                self._scan_total_deg = 0.0

            # Samandirayi ara
            dist = self.sonar.distance_mm()
            if dist is not None:
                print(f"[GOREV2] Samandira bulundu! {dist}mm")
                self._enter("APPROACH_BUOY")
                return False

            # Donarak tara
            if self._scan_total_deg < self._SONAR_SCAN_MAX_DEG:
                tgt_hdg = (self._scan_start_hdg + self._scan_total_deg) % 360.0
                self.stab.set_targets(depth_m=NAV_TARGET_DEPTH, heading_deg=tgt_hdg)
                axes = self.stab.compute(surge=0.0)
                self._apply(axes, 0.0)
                hdg_err = abs(self._angle_diff(self.stab.ori.heading, tgt_hdg))
                if hdg_err < 5.0:
                    self._scan_total_deg += self._SONAR_SCAN_STEP_DEG
            else:
                # Tam tur tarandi, bulunamadi — devam et
                print("[GOREV2] Samandira bulunamadi — genis alanda devam.")
                self._enter("EXIT_ZONE")
            return False

        # ── SAMANDIRAYA YAKLAS
        if s == "APPROACH_BUOY":
            dist = self.sonar.distance_mm()
            if dist is None:
                # Sonar kayip: dur ve bekle
                axes = self.stab.compute(surge=0.0)
                self._apply(axes, 0.0)
                if self._elapsed() > 10.0:
                    self._enter("ORBIT_BUOY")  # yakinda oldugunu varsay
                return False
            if dist < self._BUOY_APPROACH_DIST_MM:
                # Yeterince yakin
                self._enter("ORBIT_BUOY")
            else:
                # Yaklas
                heading = self.stab.ori.heading or 0.0
                self.stab.set_targets(depth_m=NAV_TARGET_DEPTH,
                                      heading_deg=heading)
                axes = self.stab.compute(surge=NAV_CRUISE_SURGE * 0.5)
                self._apply(axes, NAV_CRUISE_SURGE * 0.5)
            return False

        # ── SAMANDIRA ORBITI
        if s == "ORBIT_BUOY":
            if self._orbit_start_hdg is None:
                self._orbit_start_hdg = self.stab.ori.heading or 0.0
                self._orbit_deg = 0.0
                print(f"[GOREV2] Orbit basliyor. Baslangic heading: {self._orbit_start_hdg:.1f}°")

            # Sabit ileri + sabit sag yaw = yaricaply daire
            self.stab.set_targets(depth_m=NAV_TARGET_DEPTH)
            axes = self.stab.compute(surge=BUOY_ORBIT_SPEED,
                                     yaw_override=BUOY_ORBIT_YAW)
            self._apply(axes, BUOY_ORBIT_SPEED)

            # Kumulatif aci takibi
            cur_hdg = self.stab.ori.heading or 0.0
            delta = abs(self._angle_diff(
                cur_hdg,
                (self._orbit_start_hdg + self._orbit_deg) % 360.0
            ))
            self._orbit_deg += self._SONAR_SCAN_STEP_DEG  # adim adim say
            if self._orbit_deg >= BUOY_ORBIT_DEG:
                print(f"[GOREV2] Orbit tamamlandi ({self._orbit_deg:.1f}°).")
                self._enter("EXIT_ZONE")
            return False

        # ── CIKIS BOLGESINE GIT
        if s == "EXIT_ZONE":
            # Kose samandiralarindan cikis merkezi hesaplanir (basit: geri don)
            # Baslangic heading'e gore 180° don ve ayni mesafe ilerle
            tgt_hdg = (self.stab.ori.heading + 180.0) % 360.0
            self.stab.set_targets(depth_m=NAV_TARGET_DEPTH, heading_deg=tgt_hdg)
            axes = self.stab.compute(surge=NAV_CRUISE_SURGE)
            self._apply(axes, NAV_CRUISE_SURGE)
            dist_home = self._dr.distance_to(0.0, 0.0)
            if dist_home < 2.0 or self._elapsed() > 120.0:
                self._enter("ASCEND")
            return False

        # ── YUZEY CIK
        if s == "ASCEND":
            self.stab.set_targets(depth_m=0.2)
            axes = self.stab.compute(surge=0.0)
            self._apply(axes, 0.0)
            if abs(self.stab.depth_error()) < M["depth_tol_m"] \
                    or self._elapsed() > 20.0:
                self.thr.stop()
                self._enter("DONE")
            return False

        return False

    # ---------------------------------------------------------------- helpers
    def _enter(self, state):
        self.state = state
        self._t0 = time.monotonic()
        # Orbit ve tarama durumu sifirla
        if state == "SONAR_SCAN":
            self._scan_start_hdg = None
        if state == "ORBIT_BUOY":
            self._orbit_start_hdg = None
        print(f"[GOREV2] → {state}")
        if self.log:
            self.log.event(f"STATE -> {state}")

    def _elapsed(self):
        return time.monotonic() - self._t0

    def _apply(self, axes, surge):
        self._dr.update(self.stab.ori.heading or 0.0, surge)
        self.thr.command(mix(**axes))
        if self.log:
            self.log.sample(self.state, self.stab, axes)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """a - b, [-180, 180] araliginda normalize edilmis."""
        d = (a - b) % 360.0
        return d if d <= 180.0 else d - 360.0
