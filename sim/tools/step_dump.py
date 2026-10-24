# 한 월드를 돌리며 매 스텝 센서·오차·조향·곡선 평균·위치 기록 → 실패 직전 구간 출력
import contextlib, io, json, math, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import numpy as np, core, eval_lf3 as EV
src_path, k, t_from, t_to = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
over = json.loads(sys.argv[5]) if len(sys.argv) > 5 else {}
track = sys.argv[6] if len(sys.argv) > 6 else "photo"
ANCHOR = "        steering_dc(steer, power)\n\n\n"
s = open(src_path if src_path.startswith("/") else os.path.join(ROOT, src_path), encoding="utf-8").read()
s = s.replace("AUTO_STOP = True", "AUTO_STOP = False")
for kk, v in dict({"RAW_MODE": False, "VALUE": [34, 6, 22, 3, 21, 3], "AUTO_CAL": False}, **over).items():
    s = EV.setc(s, kk, v)
s = s.replace(ANCHOR, "        _S = __import__('core').SIM\n"
              "        REC.append((now, nL, nF, nR, error, steer, c_ema // 1000, 1 if was_off else 0, bank_dir, _S.x, _S.y, _S.h, _S._pi, int(_S._imu_rate_now())))\n" + ANCHOR)
core.Sim.LOST_MM = 450.0
a = dict(noise=[0.3, 2.0], lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=[1.4, 1.6], wob=[0.005, 0.035])
base = dict(EV.base_profile("spike_3s_geo2", 0), gear_ratio=3.3333)
p, off = EV.world(base, k, a)
tr = core.make_track(track, base["line_width_mm"])
sim = core.Sim(p, tr, start_lateral_mm=off, time_limit_s=25.0, seed=k)
core.SIM = sim
g = {"__name__": "__main__", "REC": []}
with contextlib.redirect_stdout(io.StringIO()):
    try:
        exec(compile(s, "x", "exec"), g)
    except (core.SimDone, SystemExit):
        pass
print("결과:", sim.result)
fwd = 170.0
print("  t(ms)   nL  nF  nR  error steer c_ema yaw  off bank | 센서 위치(x,y) 진행s")
for r in g["REC"]:
    t = r[0]
    if t_from * 1000 <= t <= t_to * 1000:
        sx = r[9] + fwd * math.cos(r[11]); sy = r[10] + fwd * math.sin(r[11])
        print("%6d  %3d %3d %3d  %5d %5d %5d %4d  %d  %2d | (%5.0f,%5.0f) %5.0f" % (t, r[1], r[2], r[3], r[4], r[5], r[6], r[13], r[7], r[8], sx, sy, tr.s[r[12]]))
