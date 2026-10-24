# =====================================================================
#  GPU 대규모 스윕 — 3센서 배치 × 설정 × 월드 × 트랙을 한 번에 (TorchSim + TorchRule)
#
#  sweep_lf3.py(core, 실기 파일 exec)는 정확하지만 느리다. 여기서 수천 개 설정을
#  빠르게 걸러 상위 후보만 sweep_lf3.validate_configs로 core에서 확인한다.
#  월드 분포·점수식은 sweep_lf3와 같고, '찢어진 읽기'는 core와 같은 2/3로 둔다.
#
#    ~/.venvs/linefollowing/bin/python -m rl.sweep_gpu --n 192 --worlds 6
# =====================================================================
import argparse
import json
import os
import time

import numpy as np
import torch

from . import _SIM_DIR
import core
from .fastsim import TrackPool
from .parity_test import world
from .rule_lf3 import load_constants
from .torchsim import TorchPool, TorchRule, TorchSim, pick_device, F32
from . import tracks as gen

import sweep_lf3 as SL                       # 같은 파라미터 공간·점수식

LAYOUTS = {   # 이름: (센서 위치 [페어좌, 페어우, 센티넬], 고정값, 발자국mm, 대비배율)
    "pf130": ([[90, 12.5], [90, -12.5], [130, 0]], {}, None, None),
    "pf110": ([[90, 12.5], [90, -12.5], [110, 0]], {}, None, None),
    "pf150": ([[90, 12.5], [90, -12.5], [150, 0]], {}, None, None),
    "pf130_fp12": ([[90, 12.5], [90, -12.5], [130, 0]], {}, 12.0, None),
    "pair2": ([[90, 12.5], [90, -12.5], [130, 0]], {"USE_SENTINEL": False}, None, None),
    "row3": ([[90, 25], [90, -25], [90, 0]], {}, None, None),
    "geo": ([[170, 23], [170, -23], [170, 0]], {}, None, None),   # 실기 실측 (spike_3s_geo)
    "row3_70": ([[70, 25], [70, -25], [70, 0]], {}, None, None),
    "row3_110": ([[110, 25], [110, -25], [110, 0]], {}, None, None),
}
TRACK_LIMIT = {"mrc": 40.0, "photo": 30.0}       # mrc 3센서 완주 최고가 ~32s
GEN_TRACKS = (("hard", 11), ("medium", 12))          # 학습 시드 (홀드아웃 아님)
P_TORN = 2.0 / 3.0
STUCK_MS = 5000


# 파워 100 고정 (사용자 방침 2026-10-17: 감속 없이 완주) — 감속 관련 상수를 전부 묶는다
NO_SLOW = {"POWER": 100, "SPD_K10": 0, "MIN_POWER": 100, "RECOVER_POWER": 100, "PB_MS": 0, "PB_POWER": 100}


def world_r(base, k, lag=(10, 25), noise=(0.5, 2.0)):
    """parity_test.world(=sweep_lf3.make_world)와 같은 난수열, 지연·노이즈 범위만 바꿈 → 모터·장착오차 동일."""
    rng = np.random.default_rng(7000 + k)
    w = dict(tau_up=base["motor_tau_s"] * rng.uniform(0.85, 1.15),
             battery=rng.uniform(0.85, 1.0), noise=rng.uniform(noise[0], noise[1]),
             lag_ms=rng.uniform(lag[0], lag[1]))
    pert = np.asarray([(rng.uniform(-3, 3), rng.uniform(-1.5, 1.5)) for _ in range(4)])
    w["pos_err"] = pert[:len(base["sensor_ports"])]
    w["off"] = float(rng.uniform(-8, 8))
    return w


def build_tasks(layouts, n_cfg, worlds, rng, best=None, fixed=None, given=None):
    """(레이아웃, 설정) 목록 — 0번 = 실기 파일 기본값, 1번 = 검증된 최고값(있으면), 나머지 국소/전역.
    fixed: 모든 설정에 덮어쓰는 고정값(탐색 안 함). given: 탐색 대신 이 설정들만 평가."""
    fixed = fixed or {}
    if given is not None:
        return [(lay, dict(c, **fixed)) for lay in layouts for c in given]
    dflt = SL.lf3_defaults()
    cfgs = []
    for lay in layouts:
        lst = [dict(fixed)]
        if best and best.get(lay):
            lst.append(dict(best[lay], **fixed))
        n_local = (n_cfg - len(lst)) // 2
        center = dict(dflt)
        if best and best.get(lay):
            center.update(best[lay])
        for i in range(n_cfg - len(lst)):
            local = i < n_local
            cfg = {}
            for spec in SL.LF3_SPACE:
                if spec[0] in fixed:
                    continue
                c = center[spec[0]] if local else None
                if c is not None and spec[3] != "c":
                    c = float(c)
                cfg[spec[0]] = SL._draw(spec, rng, c)
            lst.append(dict(cfg, **fixed))
        cfgs += [(lay, c) for c in lst]
    return cfgs


def run(layouts, n_cfg, worlds, device, seed=0, best=None, limit_scale=1.0,
        lag=(10, 25), noise=(0.5, 2.0), fixed=None, given=None):
    dev = pick_device(device)
    base = core.load_profile("spike_3s")
    names, trs = [], []
    for nm in TRACK_LIMIT:
        names.append(nm)
        trs.append(core.make_track(nm, 20.0))
    for lv, sd in GEN_TRACKS:
        names.append("%s%d" % (lv, sd))
        trs.append(gen.make_track(gen.generate(sd, lv)))
    pool = TrackPool(trs, names)
    tpool = TorchPool(pool, dev)
    rng = np.random.default_rng(seed)
    cfgs = build_tasks(layouts, n_cfg, worlds, rng, best, fixed, given)
    combos = [(ci, ti, k) for ci in range(len(cfgs)) for ti in range(len(names)) for k in worlds]
    n = len(combos)
    print("GPU 스윕: 설정 %d × 트랙 %d × 월드 %d = %d대 (%s)" % (len(cfgs), len(names), len(worlds), n, dev), flush=True)
    sim = TorchSim(n, tpool, base, dev, seed=seed)
    rule = TorchRule(n, dev, load_constants())
    idx_all = torch.arange(n, device=dev)
    ci_arr = np.array([c[0] for c in combos])
    ti_arr = np.array([c[1] for c in combos])
    k_arr = np.array([c[2] for c in combos])
    # 월드 (sweep_lf3.make_world와 같은 분포·같은 난수열) + 레이아웃 기하
    ws = {k: world_r(base, k, lag, noise) for k in worlds}
    w = dict(tau_up=np.array([ws[k]["tau_up"] for k in k_arr]),
             battery=np.array([ws[k]["battery"] for k in k_arr]),
             noise=np.array([ws[k]["noise"] for k in k_arr]),
             lag_ms=np.array([ws[k]["lag_ms"] for k in k_arr]),
             pos_err=np.stack([ws[k]["pos_err"] for k in k_arr]))
    pos_abs = np.zeros((n, 3, 2))
    fp = np.full(n, base["sensor_footprint_mm"], float)
    white_k = np.ones((n, 3))
    for i, (lay, _) in enumerate(cfgs):
        pos, _, f, con = LAYOUTS[lay]
        m = ci_arr == i
        pos_abs[m] = pos
        if f:
            fp[m] = f
        if con:
            wh = np.asarray(base["sensor_white"][:3], float)
            bk = np.asarray(base["sensor_black"][:3], float)
            white_k[m] = (bk + (wh - bk) * con) / wh
    w.update(pos_abs=pos_abs, footprint_mm=fp, white_k=white_k)
    sim.set_world(idx_all, w)
    sim.set_tracks(idx_all, torch.as_tensor(ti_arr, device=dev))
    off = torch.as_tensor([ws[k]["off"] for k in k_arr], dtype=F32, device=dev)
    sim.reset_pose(idx_all, torch.zeros(n, dtype=torch.int64, device=dev), off, torch.zeros(n, device=dev))
    # 설정: 레이아웃 고정값 + 설정값 (env 묶음별로)
    for i, (lay, cfg) in enumerate(cfgs):
        sel = torch.as_tensor(np.nonzero(ci_arr == i)[0], device=dev)
        merged = dict(LAYOUTS[lay][1])
        merged.update(cfg)
        rule.set_consts(sel, merged)
    rule.set_value(idx_all, sim.white, sim.black)
    rule.reset(idx_all, sim.raw)

    limits = torch.as_tensor([int(1000 * limit_scale * TRACK_LIMIT.get(names[t], 40.0)) for t in ti_arr], device=dev)
    length = tpool.length[sim.ti]
    done = torch.zeros(n, dtype=torch.bool, device=dev)
    code = torch.zeros(n, dtype=torch.int64, device=dev)
    t_end = torch.zeros(n, dtype=torch.int64, device=dev)
    lat_sum = torch.zeros(n, device=dev)
    lat_cnt = torch.zeros(n, device=dev)
    mark = sim.max_s.clone()
    t_mark = torch.zeros(n, dtype=torch.int64, device=dev)
    zero = torch.zeros(n, device=dev)
    p_torn = torch.full((n,), P_TORN, device=dev)
    t0 = time.time()
    tick = 0
    while True:
        now = sim.now_ms()
        gyro = sim.yaw_dps().trunc().to(torch.int64)
        l, r = rule.step_frame(sim, now, gyro, p_torn, gate=sim.ctrl_gate())
        sim.tick(torch.where(done, zero, l), torch.where(done, zero, r))
        tick += 1
        if tick % 7:
            continue
        s, lat, _ = sim.progress()
        now = sim.now_ms()
        live = ~done
        lat_sum += torch.where(live, lat.abs(), zero)
        lat_cnt += live.to(F32)
        moved = sim.max_s > mark + 50.0
        mark = torch.where(moved, sim.max_s, mark)
        t_mark = torch.where(moved, now, t_mark)
        c = torch.where(lat.abs() > core.Sim.LOST_MM, 1,
                        torch.where(s > length - core.Sim.FINISH_MM, 2,
                                    torch.where(now - t_mark > STUCK_MS, 3,
                                                torch.where(now > limits, 4, 0))))
        newly = live & (c > 0)
        code = torch.where(newly, c, code)
        t_end = torch.where(newly, now, t_end)
        done = done | newly
        if tick % 7000 == 0:
            print("  %.1fs 시뮬 · 완료 %d/%d · 벽시계 %.0fs" % (tick * sim.dt, int(done.sum()), n, time.time() - t0), flush=True)
        if bool(done.all()):
            break
    frac = (sim.max_s / length).cpu().numpy()
    code = code.cpu().numpy()
    t_end = t_end.cpu().numpy() / 1000.0
    latm = (lat_sum / lat_cnt.clamp_min(1)).cpu().numpy()
    yellow = rule.yellow.cpu().numpy()
    print("완료: 벽시계 %.0fs" % (time.time() - t0), flush=True)
    rows = []
    for i in range(n):
        reason = ["", "lost", "finish", "stuck", "timeout"][code[i]]
        rows.append(dict(cell=cfgs[ci_arr[i]][0], cfg=cfgs[ci_arr[i]][1], track=names[ti_arr[i]],
                         k=int(k_arr[i]), reason=reason, time_s=float(t_end[i]),
                         max_mm=float(frac[i] * pool.length[ti_arr[i]]), len_mm=float(pool.length[ti_arr[i]]),
                         lat=float(latm[i]), yellow=int(yellow[i])))
    return rows, names


def summarize(rows, names):
    groups = {}
    for r in rows:
        groups.setdefault((r["cell"], json.dumps(r["cfg"], sort_keys=True)), []).append(r)
    out = []
    for (cell, cfg_s), rs in groups.items():
        per = {}
        for t in names:
            sub = [r for r in rs if r["track"] == t]
            f = [r for r in sub if r["reason"] == "finish"]
            per[t] = dict(fin=len(f), n=len(sub), time=round(float(np.mean([r["time_s"] for r in f])), 2) if f else None)
        out.append(dict(cell=cell, cfg=json.loads(cfg_s), score=round(float(np.mean([SL.score(r) for r in rs])), 1),
                        fin=sum(r["reason"] == "finish" for r in rs), n=len(rs), tracks=per))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layouts", default=",".join(LAYOUTS))
    ap.add_argument("--n", type=int, default=192)
    ap.add_argument("--worlds", default="100,101,102,103,104,105")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(_SIM_DIR, "sweep_gpu_result.json"))
    ap.add_argument("--lag", default="10,25", help="센서 지연 범위 ms (실측에 맞출 것)")
    ap.add_argument("--noise", default="0.5,2.0", help="센서 노이즈 σ 범위")
    ap.add_argument("--no-slow", action="store_true", help="파워 100 고정 (감속 상수 전부 묶음)")
    ap.add_argument("--center", default=None, help="국소 탐색 중심 설정 JSON 파일 (dict)")
    ap.add_argument("--configs", default=None, help="탐색 대신 이 설정 목록(JSON list)만 평가")
    ap.add_argument("--rank", choices=["score", "finish"], default="score",
                    help="finish = 완주 수 우선 (사진 트랙 → 전체), 같으면 점수")
    ap.add_argument("--top-out", default=None, help="상위 설정 목록을 이 JSON 파일로 저장")
    ap.add_argument("--top", type=int, default=3)
    args = ap.parse_args()
    best = {}
    vpath = os.path.join(_SIM_DIR, "sweep_lf3_validate.json")
    if os.path.exists(vpath):                       # core 검증 최고값을 국소 탐색 중심으로
        summ = SL.summarize(json.load(open(vpath)))
        for lay in LAYOUTS:
            sub = sorted([s for s in summ if s["cell"] == lay and s["cfg"]], key=lambda s: s["score"])
            if sub:
                best[lay] = sub[0]["cfg"]
    layouts = args.layouts.split(",")
    if args.center:
        c = json.load(open(args.center))
        best = {lay: c for lay in layouts}
    if args.worlds.count(":") == 1:                       # a:b = range(a, b)
        a, b = args.worlds.split(":")
        worlds = list(range(int(a), int(b)))
    else:
        worlds = [int(k) for k in args.worlds.split(",")]
    given = json.load(open(args.configs)) if args.configs else None
    rows, names = run(layouts, args.n, worlds, args.device, args.seed, best,
                      lag=tuple(float(x) for x in args.lag.split(",")),
                      noise=tuple(float(x) for x in args.noise.split(",")),
                      fixed=NO_SLOW if args.no_slow else None, given=given)
    json.dump(rows, open(args.out, "w"), ensure_ascii=False)
    summ = summarize(rows, names)

    def key(s):
        if args.rank == "finish":
            ph = s["tracks"].get("photo", {}).get("fin", 0)
            return (-ph, -s["fin"], s["score"])
        return s["score"]
    if args.top_out:
        tops = []
        for lay in layouts:
            tops += [s["cfg"] for s in sorted([s for s in summ if s["cell"] == lay], key=key)[:args.top]]
        json.dump(tops, open(args.top_out, "w"), ensure_ascii=False)
    print("\n=== 배치별 상위 %d (%s, 완주/런) ===" % (args.top, "완주 우선" if args.rank == "finish" else "점수 낮을수록 좋음"))
    for lay in layouts:
        sub = sorted([s for s in summ if s["cell"] == lay], key=key)[:args.top]
        for s in sub:
            print("%-11s %6.1f %3d/%d  %s  %s" % (lay, s["score"], s["fin"], s["n"],
                  " ".join("%s %d/%d %s" % (t, v["fin"], v["n"], v["time"]) for t, v in s["tracks"].items()),
                  "기본값" if not s["cfg"] else ""))


if __name__ == "__main__":
    main()
