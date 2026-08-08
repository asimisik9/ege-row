#!/usr/bin/env python3
"""
SIMULASYON UCTAN UCA TESTI — havuza girmeden once TUM gorevi dogrular.

Kullanim (rov/ klasoru icinden):
    python3 tools/sim_test.py

Ne dogrular:
  - Tam gorev dizisi hatasiz basliyor ve DONE'a ulasiyor mu
  - Kontrol dongusu hedef frekansta mi (SORUN 2)
  - Daire sayaci hedefine dogru ulasiyor mu, gurultu biriktirmiyor mu (SORUN 6)
  - Yuzey ihlali var mi (kabul kriteri H4)

Not: sureler simulasyonda KISALTILIR (havuz suresi degil, mantik test edilir).
Gercek gorev sureleri config.py'deki MISSION sozlugundedir.

Sonrasinda:  python3 tools/analyze_log.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
sys.argv = ["x", "--sim"]
import config
config.SIM_MODE = True
config.MISSION["start_delay_s"] = 0.5
config.MISSION["straight_time_s"] = 1.5
config.MISSION["turn_settle_s"] = 0.4
config.MISSION["circle_yaw_rate_dps"] = 45.0
config.MISSION["dive_timeout_s"] = 6.0
config.LOG_EVERY_N = 2

import main as M
thr, ori, depth, hub = M.build_system()
from control.stabilizer import Stabilizer
from missions.video_demo import VideoDemoMission
from utils.logger import MissionLogger
from utils.looptimer import LoopTimer
from control.mixer import mix
from config import LOOP_HZ

stab = Stabilizer(ori, depth, state=hub.state)
log = MissionLogger("sim_video_demo")
m = VideoDemoMission(stab, thr, logger=log)
lt = LoopTimer(LOOP_HZ, warn_hz=None, name="sim")
m.start()
t0 = time.time(); son = None
while time.time() - t0 < 40:
    lt.tick()
    if m.step():
        print("GOREV TAMAM", flush=True); break
    if m.state != son:
        print(f"  t={time.time()-t0:6.1f}  {m.state:<10} "
              f"derinlik={stab.depth_m:5.2f} yon={stab.heading_deg:6.1f} "
              f"hedef={stab.target_heading}", flush=True)
        son = m.state
    lt.sleep()
print(lt.report())
print("yuzey ihlali:", m.surface_violations)
print("daire toplam donus:", round(m._circle_acc, 1), "derece")
log.close(); hub.stop(); thr.stop()
print("LOG:", log.path)
