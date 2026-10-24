# =====================================================================
#  core(+실기 파일 spike_lf3_rule.py) vs fastsim(+RuleLF3) 일치 확인 + 속도 측정
#
#  fastsim은 시간을 센서 프레임 단위로 끊는 등 단순화가 있어 궤적이 비트 단위로
#  같을 수는 없다. 그래서 같은 월드(모터·배터리·노이즈·지연·장착오차·시작오프셋)에서
#   ① 노이즈 0 결정론 월드: 랩타임·횡편차가 가까운지
#   ② 랜덤 월드 여러 개: 완주율·랩타임·도달거리 통계가 가까운지
#  를 본다.
#
#    python3 -m rl.parity_test                 # (sim/ 폴더에서)
#    python3 -m rl.parity_test --worlds 4 --bench
# =====================================================================
import argparse
import contextlib
import io
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

from . import _SIM_DIR
import core
from .fastsim import FastSim, TrackPool
from .rule_lf3 import RuleLF3, RULE_FILE
from . import tracks as gen

ROOT = os.path.dirname(_SIM_DIR)
LIMIT = {"mrc": 50.0, "photo": 30.0}
STUCK_S = 5.0
STARTUP_S = 0.2           # 실기 파일의 find_yaw 대기 (정지)
P_TORN = 2.0 / 3.0        # core와 같은 '찢어진 읽기' 확률 (센서 3개 차례 읽기)


def world(base, k):
    """sweep_lf3.make_world와 같은 분포·같은 난수열 (센서 4개분 오차를 뽑고 앞 N개 사용)."""
    rng = np.random.default_rng(7000 + k)
    w = dict(tau_up=base["motor_tau_s"] * rng.uniform(0.85, 1.15),
             battery=rng.uniform(0.85, 1.0), noise=rng.uniform(0.5, 2.0),
             lag_ms=rng.uniform(10, 25))
    pert = np.asarray([(rng.uniform(-3, 3), rng.uniform(-1.5, 1.5)) for _ in range(4)])
    w["pos_err"] = pert[:len(base["sensor_ports"])]
    w["off"] = float(rng.uniform(-8, 8))
    return w


def _track(name):
    if name.startswith("gen"):
        return gen.make_track(gen.generate(int(name[3:]), "medium"))
    return core.make_track(name, 20.0)


# ------------------------------------------------------------ core 쪽
class _StuckSim(core.Sim):
    def _update_sensors(self):
        super()._update_sensors()
        if self.max_progress_mm > getattr(self, "_mark", 0.0) + 50.0:
            self._mark, self._t_mark = self.max_progress_mm, self.t
        elif self.t - getattr(self, "_t_mark", 0.0) > STUCK_S:
            self._finish("stuck")


def core_run(task):
    tname, k, noiseless = task
    base = core.load_profile("spike_3s")
    tr = _track(tname)
    p = dict(base)
    if noiseless:
        p["sensor_noise"] = 0.0
        off = 0.0
    else:
        w = world(base, k)
        p.update(motor_tau_s=w["tau_up"], battery_scale=w["battery"],
                 sensor_noise=w["noise"], sensor_lag_ms=w["lag_ms"],
                 sensor_pos_mm=(np.asarray(base["sensor_pos_mm"]) + w["pos_err"]).tolist())
        off = w["off"]
    sim = _StuckSim(p, tr, start_lateral_mm=off, time_limit_s=LIMIT.get(tname, 40.0) + STARTUP_S,
                    seed=k)
    core.SIM = sim
    src = open(RULE_FILE, encoding="utf-8").read()
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, "lf3", "exec"), {"__name__": "__main__"})
        except core.SimDone:
            pass
    r = sim.result
    return dict(track=tname, k=k, reason=r["reason"], time_s=round(r["time_s"] - STARTUP_S, 3),
                frac=round(sim.max_progress_mm / tr.length, 4), lat=r["mean_lat_mm"])


# ------------------------------------------------------------ fastsim 쪽
def fast_run(tnames, ks, noiseless, seed=0):
    base = core.load_profile("spike_3s")
    pool = TrackPool([_track(t) for t in tnames], tnames)
    combos = [(ti, k) for ti in range(len(tnames)) for k in ks]
    n = len(combos)
    sim = FastSim(n, pool, base, seed=seed)
    idx = np.arange(n)
    if noiseless:
        sim.set_world(idx, dict(noise=0.0))
        off = np.zeros(n)
    else:
        ws = [world(base, k) for _, k in combos]
        sim.set_world(idx, dict(tau_up=np.array([w["tau_up"] for w in ws]),
                                battery=np.array([w["battery"] for w in ws]),
                                noise=np.array([w["noise"] for w in ws]),
                                lag_ms=np.array([w["lag_ms"] for w in ws]),
                                pos_err=np.stack([w["pos_err"] for w in ws])))
        off = np.array([w["off"] for w in ws])
    sim.set_tracks(idx, np.array([ti for ti, _ in combos]))
    sim.reset_pose(idx, np.zeros(n, np.int64), off, np.zeros(n))
    rule = RuleLF3(n)
    wv = np.asarray(base["sensor_white"][:3])
    bv = np.asarray(base["sensor_black"][:3])
    rule.set_value(idx, np.broadcast_to(wv, (n, 3)), np.broadcast_to(bv, (n, 3)))

    z = np.zeros(n)
    for _ in range(int(round(STARTUP_S / sim.dt))):        # 정지 대기 (지연 버퍼 채움)
        sim.tick(z, z)
    rule.reset(idx, sim.raw)
    t_loop = 0.0
    done = np.zeros(n, bool)
    res = [None] * n
    lat_sum, lat_n = np.zeros(n), np.zeros(n)
    mark, t_mark = np.zeros(n), np.zeros(n)
    limits = np.array([LIMIT.get(tnames[ti], 40.0) for ti, _ in combos])
    lengths = pool.length[np.array([ti for ti, _ in combos])]
    K = 4                                                     # 진행도 계산 주기 (틱)
    tick = 0
    while not done.all():
        now = np.full(n, int(t_loop * 1000), np.int64)
        gyro = np.trunc(sim.yaw_dps()).astype(np.int64)
        l, r = rule.step_frame(sim, now, gyro, p_torn=P_TORN)
        sim.tick(np.where(done, 0.0, l), np.where(done, 0.0, r))
        t_loop += sim.dt
        tick += 1
        if tick % K:
            continue
        s, lat, _ = sim.progress()
        live = ~done
        lat_sum[live] += np.abs(lat[live])
        lat_n[live] += 1
        up = sim.max_s > mark + 50.0
        mark = np.where(up, sim.max_s, mark)
        t_mark = np.where(up, t_loop, t_mark)
        reason = np.where(np.abs(lat) > core.Sim.LOST_MM, 1,
                          np.where(s > lengths - core.Sim.FINISH_MM, 2,
                                   np.where(t_loop - t_mark > STUCK_S, 3,
                                            np.where(t_loop > limits, 4, 0))))
        for i in np.nonzero(live & (reason > 0))[0]:
            done[i] = True
            res[i] = dict(track=tnames[combos[i][0]], k=combos[i][1],
                          reason=["", "lost", "finish", "stuck", "timeout"][reason[i]],
                          time_s=round(t_loop, 3), frac=round(sim.max_s[i] / lengths[i], 4),
                          lat=round(lat_sum[i] / max(1, lat_n[i]), 2))
    return res


# ------------------------------------------------------------ 비교·벤치
def summarize(rows, label):
    out = {}
    for t in sorted(set(r["track"] for r in rows)):
        sub = [r for r in rows if r["track"] == t]
        fin = [r for r in sub if r["reason"] == "finish"]
        out[t] = dict(fin="%d/%d" % (len(fin), len(sub)),
                      time=round(float(np.mean([r["time_s"] for r in fin])), 2) if fin else None,
                      frac=round(float(np.mean([r["frac"] for r in sub])), 3),
                      lat=round(float(np.mean([r["lat"] for r in sub])), 1))
    print("%-8s %s" % (label, out))
    return out


def bench(ns=(64, 256, 1024), sim_s=2.0):
    base = core.load_profile("spike_3s")
    pool = TrackPool([core.make_track("mrc", 20.0), core.make_track("photo", 20.0)], ["mrc", "photo"])
    for n in ns:
        sim = FastSim(n, pool, base, seed=1)
        idx = np.arange(n)
        sim.set_world(idx, {})
        sim.set_tracks(idx, idx % 2)
        sim.reset_pose(idx, np.zeros(n, np.int64), np.zeros(n), np.zeros(n))
        rule = RuleLF3(n)
        rule.set_value(idx, np.broadcast_to(base["sensor_white"][:3], (n, 3)),
                       np.broadcast_to(base["sensor_black"][:3], (n, 3)))
        rule.reset(idx, sim.raw)
        steps = int(sim_s / sim.dt)
        t0 = time.time()
        t_loop = 0.0
        for i in range(steps):
            now = np.full(n, int(t_loop * 1000), np.int64)
            l, r = rule.step_frame(sim, now, np.trunc(sim.yaw_dps()).astype(np.int64), p_torn=P_TORN)
            sim.tick(l, r)
            t_loop += sim.dt
            if i % 4 == 0:
                sim.progress()
        wall = time.time() - t0
        print("n=%5d: %.2fs 시뮬 × %d대 / 벽시계 %.2fs → 합산 %.0fx 실시간 (틱 %.2fms)"
              % (n, sim_s, n, wall, n * sim_s / wall, 1000 * wall / steps))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worlds", type=int, default=6)
    ap.add_argument("--tracks", default="mrc,photo,gen1,gen2")
    ap.add_argument("--procs", type=int, default=2)
    ap.add_argument("--bench", action="store_true")
    args = ap.parse_args()
    tnames = args.tracks.split(",")

    print("== ① 노이즈 0 결정론 월드 (지연 15ms 고정)")
    with Pool(args.procs) as pool:
        c0 = pool.map(core_run, [(t, 0, True) for t in tnames])
    f0 = fast_run(tnames, [0], True)
    for a, b in zip(c0, f0):
        print("  %-6s core %-7s %6.2fs 도달 %5.1f%% 횡편차 %4.1f | fast %-7s %6.2fs 도달 %5.1f%% 횡편차 %4.1f"
              % (a["track"], a["reason"], a["time_s"], 100 * a["frac"], a["lat"],
                 b["reason"], b["time_s"], 100 * b["frac"], b["lat"]))

    print("== ② 랜덤 월드 %d개 × 트랙 %d개" % (args.worlds, len(tnames)))
    ks = list(range(args.worlds))
    t0 = time.time()
    with Pool(args.procs) as pool:
        cr = pool.map(core_run, [(t, k, False) for t in tnames for k in ks])
    tc = time.time() - t0
    t0 = time.time()
    fr = fast_run(tnames, ks, False)
    tf = time.time() - t0
    summarize(cr, "core")
    summarize(fr, "fastsim")
    print("  벽시계: core %.0fs (%d프로세스) / fastsim %.1fs (1프로세스)" % (tc, args.procs, tf))
    same = sum(1 for a, b in zip(cr, fr) if (a["reason"] == "finish") == (b["reason"] == "finish"))
    print("  완주 여부 일치: %d/%d" % (same, len(cr)))
    if args.bench:
        print("== 속도")
        bench()


if __name__ == "__main__":
    main()
