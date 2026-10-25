# =====================================================================
#  실측 치수 GPU 스윕 — 3센서 규칙 코드(TorchRule = 실기 파일과 같은 식) 설정 수천 개를 한 번에
#
#    ~/.venvs/linefollowing/bin/python sim/rl/sweep_geo.py --n 1200 --worlds 12 --fwd 100 --out 결과.json
#    (--center '{"KP": 0.088, ...}' : 이 설정 주변을 주로 탐색 · --given 파일 : 주어진 설정들만 평가)
#
#  월드 = sim/eval_lf3.py와 같은 난수열(모터·장착오차·출발 위치·구동/자이로 지연·회전 권한·바퀴 흔들림),
#  실기 AUTO_CAL 흉내(출발 때 흰색 평균의 정수 내림 ≈ 흰−1). 파워 100 고정, 감속 파라미터 없음.
#  출력: 설정별 트랙 완주·평균 시간·조향 떨림(라인 볼 때 스텝당 |Δ조향|)·뱅킹 횟수
# =====================================================================
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np          # noqa: E402
import torch                # noqa: E402
import core                 # noqa: E402
import sweep_lf3 as SL      # noqa: E402
from rl.fastsim import TrackPool                                        # noqa: E402
from rl.torchsim import TorchPool, TorchRule, TorchSim, pick_device, F32  # noqa: E402
from rl.rule_lf3 import load_constants                                    # noqa: E402
from rl.sweep_gpu import world_r                                          # noqa: E402

TRACKS = ("photo", "rightangle")
LIM = {"photo": 40.0, "rightangle": 20.0}
SPACE = [("KP", 0.03, 0.9, "lf"), ("KD", 0.0, 10.0, "f"), ("KG", 20, 500, "i"), ("G_CLAMP", 20, 80, "i"),
         ("ALPHA", 500, 950, "i"), ("D_FILTER", 150, 600, "i"), ("MAX_STEER_DELTA", 8, 100, "i"),
         ("W_NORM", 75, 92, "i"), ("SIDE_ERR", 60, 160, "i"), ("GAP_MS", 1, 25, "i"),
         ("AHEAD_HOLD_MS", 5, 60, "i"), ("ST_BANK", 30, 100, "i"), ("BLACK_MID", 35, 65, "i"),
         ("ERR_MODE", [0, 1], None, "c"), ("CF_GAIN", 3, 22, "i"), ("SIDE_SEE", 3, 20, "i"),
         ("DFLOOR", 4, 25, "i"), ("KSP", 0, 1500, "i"), ("DELAY_MS", 20, 45, "i"),
         ("YAW_PER_STEER", 4, 6, "i"), ("TAU_MS", 70, 130, "i"),
         ("KP2", 0.08, 0.9, "lf"), ("KG2", 20, 500, "i"), ("CURVE_YAW", 50, 400, "i"), ("CURVE_TAU_MS", 50, 500, "i"),
         ("KPN", 0.0, 0.8, "f"), ("FF", 0, 1200, "i"), ("G_REL", 0, 1000, "i"), ("CEMA_BANK", [0, 1], None, "c")]


def extra(k, weff, wob):
    r = np.random.default_rng(95000 + k)
    wb = r.uniform(*wob) if wob[1] > 0 else 0.0
    return dict(cmd_lag_ms=float(np.random.default_rng(91000 + k).uniform(0, 35)),
                imu_lag_ms=float(np.random.default_rng(93000 + k).uniform(0, 20)),
                w_eff=float(np.random.default_rng(92000 + k).uniform(*weff)),
                wobble=wb * r.uniform(0.5, 1.5, 2), wobble_ph=r.uniform(0, 2 * math.pi, 2))


def base_profile(profile, fwd):
    base = core.load_profile(profile)
    if fwd:
        sp = abs(float(base["sensor_pos_mm"][0][1]))
        base = dict(base, sensor_pos_mm=[[fwd, sp], [fwd, -sp], [fwd, 0.0]], track_ref_mm=float(fwd))
    return base


def run(cfgs, nw, profile="spike_3s_geo", fwd=0.0, noise=(0.3, 1.0), lag=(3, 8), weff=(1.15, 1.5),
        wob=(0.005, 0.035), seed=0, device="auto", verbose=True, prof=None, tracks=None):
    global TRACKS
    if tracks:
        TRACKS = tuple(tracks)
    dev = pick_device(device)
    base = base_profile(profile, fwd)
    if prof:
        base = dict(base, **prof)
    trs = [core.make_track(t, base["line_width_mm"]) for t in TRACKS]
    tp = TorchPool(TrackPool(trs, list(TRACKS)), dev)
    combos = [(ci, ti, k) for ci in range(len(cfgs)) for ti in range(len(TRACKS)) for k in range(nw)]
    n = len(combos)
    if verbose:
        print("GPU 스윕: 설정 %d × 트랙 %d × 월드 %d = %d대 (%s, 센서 %smm)" % (len(cfgs), len(TRACKS), nw, n, dev,
              int(fwd) if fwd else "프로파일"), flush=True)
    sim = TorchSim(n, tp, base, dev, seed=seed)
    rule = TorchRule(n, dev, load_constants())
    idx = torch.arange(n, device=dev)
    ws = {k: dict(world_r(base, k, lag, noise), **extra(k, weff, wob)) for k in range(nw)}
    ks = [c[2] for c in combos]
    keys = ("tau_up", "battery", "noise", "lag_ms", "cmd_lag_ms", "imu_lag_ms", "w_eff", "wobble", "wobble_ph")
    w = {key: np.array([ws[k][key] for k in ks]) for key in keys}
    w["pos_err"] = np.stack([ws[k]["pos_err"] for k in ks])
    sim.set_world(idx, w)
    sim.set_tracks(idx, torch.as_tensor([c[1] for c in combos], device=dev))
    off = torch.as_tensor([ws[k]["off"] for k in ks], dtype=F32, device=dev)
    sim.reset_pose(idx, torch.zeros(n, dtype=torch.int64, device=dev), off, torch.zeros(n, device=dev))
    ci_arr = np.array([c[0] for c in combos])
    for i, cfg in enumerate(cfgs):
        if cfg:
            rule.set_consts(torch.as_tensor(np.nonzero(ci_arr == i)[0], device=dev), cfg)
    rule.set_value(idx, sim.white - 1.0, sim.black)      # 실기 AUTO_CAL
    rule.reset(idx, sim.raw)
    length = tp.length[sim.ti]
    lim = torch.as_tensor([int(LIM.get(TRACKS[c[1]], 40.0) * 1000) for c in combos], device=dev)
    done = torch.zeros(n, dtype=torch.bool, device=dev)
    code = torch.zeros(n, dtype=torch.int64, device=dev)
    tend = torch.zeros(n, dtype=torch.int64, device=dev)
    zero = torch.zeros(n, device=dev)
    jit = torch.zeros(n, device=dev)
    jn = torch.zeros(n, device=dev)
    yel_end = torch.zeros(n, dtype=torch.int64, device=dev)
    prev = rule.steer_out.clone()
    tick, t0 = 0, time.time()
    while True:
        now = sim.now_ms()
        gyro = sim.yaw_dps().trunc().to(torch.int64)
        l, r = rule.step_frame(sim, now, gyro, None, gate=sim.ctrl_gate())
        live = ~done & ~rule.was_off
        jit += torch.where(live, (rule.steer_out - prev).abs().to(F32), zero)
        jn += live.to(F32)
        prev = rule.steer_out.clone()
        sim.tick(torch.where(done, zero, l), torch.where(done, zero, r))
        tick += 1
        if tick % 7:
            continue
        s, lat, _ = sim.progress()
        now = sim.now_ms()
        c = torch.where(lat.abs() > core.Sim.LOST_MM, 1,
                        torch.where(s > length - core.Sim.FINISH_MM, 2, torch.where(now > lim, 4, 0)))
        newly = ~done & (c > 0)
        code = torch.where(newly, c, code)
        tend = torch.where(newly, now, tend)
        yel_end = torch.where(newly, rule.yellow, yel_end)
        done = done | newly
        if verbose and tick % 7000 == 0:
            print("  %.1fs 시뮬 · 완료 %d/%d · 벽시계 %.0fs" % (tick * sim.dt, int(done.sum()), n, time.time() - t0), flush=True)
        if bool(done.all()):
            break
    cd, te = code.cpu().numpy(), tend.cpu().numpy() / 1000.0
    jj = (jit / jn.clamp_min(1)).cpu().numpy()
    yel = yel_end.cpu().numpy()
    ti_arr = np.array([c[1] for c in combos])
    res = []
    for i, cfg in enumerate(cfgs):
        row = dict(cfg=cfg)
        for ti, t in enumerate(TRACKS):
            m = (ci_arr == i) & (ti_arr == ti)
            fin = cd[m] == 2
            row[t] = dict(fin=int(fin.sum()), n=int(m.sum()), t=float(te[m][fin].mean()) if fin.any() else 99.0,
                          jit=float(jj[m].mean()), banks=float(yel[m].mean()))
        row["score"] = sum(row[t]["fin"] / row[t]["n"] for t in TRACKS)
        res.append(row)
    if verbose:
        print("완료: 벽시계 %.0fs" % (time.time() - t0), flush=True)
    return res


def draw(rng, center=None, frac=0.25):
    c = {}
    for name, lo, hi, kind in SPACE:
        if kind == "c":
            c[name] = center[name] if (center and rng.random() < 0.7) else lo[int(rng.integers(len(lo)))]
            continue
        if center is not None and name in center:
            v = float(center[name])
            v = v * math.exp(rng.normal(0, 0.4)) if kind == "lf" else v + rng.normal(0, (hi - lo) * frac)
        elif kind == "lf":
            v = math.exp(rng.uniform(math.log(lo), math.log(hi)))
        else:
            v = rng.uniform(lo, hi)
        v = min(hi, max(lo, v))
        c[name] = int(round(v)) if kind == "i" else round(v, 4)
    if rng.random() < 0.5:
        c["KSP"] = 0
    if center is None and rng.random() < 0.3:
        c["KP2"] = 0.0                      # 스케줄링 끔도 일부 탐색
    if rng.random() < 0.2:
        c["KG2"] = -1
    return c


def fmt(r):
    return " | ".join("%s %2d/%d %4.1fs 떨림%.2f 뱅킹%4.1f" % (t, r[t]["fin"], r[t]["n"], r[t]["t"] if r[t]["t"] < 90 else 0,
                      r[t]["jit"], r[t]["banks"]) for t in TRACKS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1200)
    ap.add_argument("--worlds", type=int, default=12)
    ap.add_argument("--fwd", type=float, default=0.0)
    ap.add_argument("--profile", default="spike_3s_geo")
    ap.add_argument("--center", default=None, help="이 설정(JSON) 주변 2/3 + 전역 1/3")
    ap.add_argument("--given", default=None, help="이 JSON 목록의 설정들만 평가")
    ap.add_argument("--noise", default="0.3,1.0")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument("--out", default=None)
    ap.add_argument("--weff", default="1.15,1.5", help="유효 윤거 배율 범위 (월드별 무작위)")
    ap.add_argument("--prof", default=None, help="프로파일 덮어쓰기 JSON (예: {\"yaw_tau_s\": 0.05})")
    ap.add_argument("--tracks", default="photo,rightangle")
    ap.add_argument("--rank", default="jit", choices=("jit", "time"),
                    help="완주 다음 순위 기준: jit = 조향 떨림 적은 순(기존) / time = 빠른 순(흐르듯 주행)")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    if args.given:
        cfgs = json.load(open(args.given, encoding="utf-8"))
    else:
        center = json.loads(args.center) if args.center else None
        cfgs = [{}] + ([dict(center)] if center else [])
        while len(cfgs) < args.n:
            local = center is not None and rng.random() < 2 / 3
            cfgs.append(draw(rng, center if local else None))
    noise = tuple(float(x) for x in args.noise.split(","))
    res = run(cfgs, args.worlds, args.profile, args.fwd, noise=noise, seed=args.seed,
              weff=tuple(float(x) for x in args.weff.split(",")), prof=json.loads(args.prof) if args.prof else None,
              tracks=args.tracks.split(","))
    if args.out:
        json.dump(res, open(args.out, "w", encoding="utf-8"), ensure_ascii=False)
    print("E2(파일 기본값): " + fmt(res[0]))
    if args.rank == "time":
        best = sorted(range(len(res)), key=lambda i: (-res[i]["score"], sum(res[i][t]["t"] for t in TRACKS)))
    else:
        best = sorted(range(len(res)), key=lambda i: (-res[i]["score"], sum(res[i][t]["jit"] for t in TRACKS)))
    for i in best[:args.top]:
        print("#%d %s | %s" % (i, fmt(res[i]), json.dumps(res[i]["cfg"], ensure_ascii=False)))


if __name__ == "__main__":
    main()
