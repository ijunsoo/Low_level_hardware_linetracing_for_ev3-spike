# 직각 트랙 첫 직각에서: 센서가 코너를 얼마나 지나쳤나, 새 라인에 몇 도로 다시 들어오나, 그 뒤 반대 뱅킹 수
#   python3 ra_trace.py 변형.json --src 파일 --prof '{"gear_ratio": 3.3333}' [--png 경로]
import argparse
import contextlib
import io
import json
import math
import multiprocessing as mp
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import numpy as np      # noqa: E402
import core             # noqa: E402
import eval_lf3 as EV   # noqa: E402

ANCHOR = "        steering_dc(steer, power)\n\n\n"


def make_src(path, over):
    s = open(path, encoding="utf-8").read().replace("AUTO_STOP = True", "AUTO_STOP = False")
    for k, v in over.items():
        s = EV.setc(s, k, v)
    assert s.count(ANCHOR) == 1
    rec = ("        _S = __import__('core').SIM\n"
           "        REC.append((now, steer, _S.x, _S.y, _S.h, _S._pi, 1 if was_off else 0, bank_dir))\n")
    return s.replace(ANCHOR, rec + ANCHOR)


def one(job):
    core.Sim.LOST_MM = job[-1]["lost"]
    name, src, k, a = job
    base = EV.base_profile(a["profile"], a["fwd"])
    if a.get("prof"):
        base = dict(base, **a["prof"])
    p, off = EV.world(base, k, a)
    tr = core.make_track("rightangle", base["line_width_mm"])
    sim = core.Sim(p, tr, start_lateral_mm=off, time_limit_s=20.0, seed=k)
    core.SIM = sim
    g = {"__name__": "__main__", "REC": []}
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, name, "exec"), g)
        except (core.SimDone, SystemExit):
            pass
    R = np.array(g["REC"], dtype=float)
    P = tr.path
    hd = np.unwrap(np.arctan2(np.diff(P[:, 1]), np.diff(P[:, 0])))
    hd = np.append(hd, hd[-1])
    # 첫 직각: 진행 방향이 처음으로 60° 넘게 바뀐 지점
    i_c = int(np.argmax(np.abs(hd - hd[0]) > math.radians(60)))
    cx, cy = P[i_c]
    h_new = hd[min(len(hd) - 1, i_c + 40)]
    fwd = a["fwd"]
    sx = R[:, 2] + fwd * np.cos(R[:, 4])
    sy = R[:, 3] + fwd * np.sin(R[:, 4])
    # 코너 근처 구간 (진행 인덱스가 코너 ±600mm 안)
    s = tr.s
    near = (np.abs(s[R[:, 5].astype(int)] - s[i_c]) < 600)
    out = dict(reason=(sim.result or {}).get("reason"), t=(sim.result or {}).get("time_s", 0))
    # 지나침: 센서가 새 라인 진행 방향의 수직선(코너)을 넘어 원래 방향으로 간 최대 거리
    h0 = hd[0]
    over = (sx - cx) * np.cos(h0) + (sy - cy) * np.sin(h0)
    out["over"] = float(over[near].max()) if near.any() else float("nan")
    # 첫 직각 뒤 첫 재포착 순간의 진입각 (로봇 방향과 새 라인 방향 차)
    off_ = R[:, 6] > 0
    idx = np.nonzero(near & ~off_)[0]
    rec = np.nonzero(off_[:-1] & ~off_[1:] & near[1:])[0] + 1
    ang = float("nan")
    if len(rec):
        j = rec[0]
        d = (R[j, 4] - h_new + math.pi) % (2 * math.pi) - math.pi
        ang = abs(math.degrees(d))
    out["entry"] = ang
    # 그 뒤 반대쪽 뱅킹 수 (첫 뱅킹 방향과 다른 뱅킹 시작)
    starts = np.nonzero(~off_[:-1] & off_[1:] & near[1:])[0] + 1
    out["banks"] = len(starts)
    out["opp"] = int(sum(1 for j in starts[1:] if R[j, 7] != R[starts[0], 7])) if len(starts) else 0
    out["path"] = (R[near][:, [2, 3]].tolist(), np.c_[sx, sy][near].tolist(), (cx, cy), float(h0), float(h_new))
    return name, k, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("variants")
    ap.add_argument("--src", required=True)
    ap.add_argument("--profile", default="spike_3s_geo")
    ap.add_argument("--fwd", type=float, default=170.0)
    ap.add_argument("--prof", default=None)
    ap.add_argument("--lost", type=float, default=200.0, help="이탈 판정 거리 mm (시뮬 기본 200)")
    ap.add_argument("--worlds", type=int, default=24)
    ap.add_argument("--png", default=None)
    args = ap.parse_args()
    v = args.variants
    variants = json.load(open(v)) if os.path.exists(v) else json.loads(v)
    a = dict(profile=args.profile, fwd=args.fwd, prof=json.loads(args.prof) if args.prof else None, lost=args.lost,
             noise=[0.3, 1.0], lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=[1.15, 1.5], wob=[0.005, 0.035])
    jobs = [(nm, make_src(os.path.join(ROOT, args.src), over), k, a) for nm, over in variants for k in range(args.worlds)]
    with mp.Pool(max(1, os.cpu_count() - 1)) as pool:
        res = pool.map(one, jobs, chunksize=1)
    for nm, _ in variants:
        rows = [r[2] for r in res if r[0] == nm]
        fin = [r for r in rows if r["reason"] == "finish"]
        en = [r["entry"] for r in rows if not math.isnan(r["entry"])]
        print("%-18s 완주 %2d/%d | 센서 지나침 %4.0fmm | 재진입각 %3.0f° (60° 넘음 %2d/%d) | 첫 직각 뱅킹 %3.1f회 · 반대 %3.1f회" % (
            nm, len(fin), len(rows), np.mean([r["over"] for r in rows]), np.mean(en) if en else float("nan"),
            sum(1 for x in en if x > 60), len(en), np.mean([r["banks"] for r in rows]), np.mean([r["opp"] for r in rows])))
    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.family"] = "AppleGothic"
        plt.rcParams["axes.unicode_minus"] = False
        n = len(variants)
        fig, axes = plt.subplots(1, n, figsize=(5.2 * n, 5.2), squeeze=False)
        tr = core.make_track("rightangle", 19.0)
        for ax, (nm, _) in zip(axes[0], variants):
            rows = [r[2] for r in res if r[0] == nm][:6]
            cx, cy = rows[0]["path"][2]
            ax.plot(tr.path[:, 0] - cx, tr.path[:, 1] - cy, color="#222", lw=7, alpha=0.25, solid_capstyle="butt")
            for r in rows:
                axle, sens = np.array(r["path"][0]), np.array(r["path"][1])
                ax.plot(sens[:, 0] - cx, sens[:, 1] - cy, lw=1.2)
            ax.set_xlim(-250, 350)
            ax.set_ylim(-120, 330)
            ax.set_aspect("equal")
            ax.set_title(nm)
            ax.grid(alpha=0.3)
        fig.suptitle("2배 속도, 첫 직각에서 센서 궤적 (아래 직선을 → 방향으로 달려와 위로 꺾임, 회색 = 라인, 시뮬 6개 월드)")
        fig.tight_layout()
        fig.savefig(args.png, dpi=110)
        print("그림:", args.png)


if __name__ == "__main__":
    main()
