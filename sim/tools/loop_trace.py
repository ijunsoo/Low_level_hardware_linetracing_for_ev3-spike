# 사진 트랙 8자(루프 + 교차로)에서 무슨 일이 생기나: 월드별 결과·실패 위치, 그리고 8자 구역의 바퀴축·가운데 센서 궤적 그림
#   python3 loop_trace.py --src spike/... --profile spike_3s_geo2 --prof '{"gear_ratio": 3.3333}' --track photo --png out.png
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
    s = open(path if path.startswith("/") else os.path.join(ROOT, path), encoding="utf-8").read()
    s = s.replace("AUTO_STOP = True", "AUTO_STOP = False")
    for k, v in over.items():
        s = EV.setc(s, k, v)
    assert s.count(ANCHOR) == 1
    rec = ("        _S = __import__('core').SIM\n"
           "        REC.append((now, steer, _S.x, _S.y, _S.h, _S._pi, 1 if was_off else 0, bank_dir, _S._lat_now))\n")
    return s.replace(ANCHOR, rec + ANCHOR)


def crossings(P):
    # 경로 자기교차점 (선분 교차) — 8자 교차로 위치
    out = []
    seg = P[::4]
    n = len(seg)
    for i in range(n - 1):
        a, b = seg[i], seg[i + 1]
        for j in range(i + 3, n - 1):
            c, d = seg[j], seg[j + 1]
            den = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0])
            if abs(den) < 1e-9:
                continue
            t = ((c[0] - a[0]) * (d[1] - c[1]) - (c[1] - a[1]) * (d[0] - c[0])) / den
            u = ((c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])) / den
            if 0 <= t <= 1 and 0 <= u <= 1:
                out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), i * 4, j * 4))
    return out


def one(job):
    name, src, k, a = job
    core.Sim.LOST_MM = a["lost"]
    base = EV.base_profile(a["profile"], a["fwd"])
    if a.get("prof"):
        base = dict(base, **a["prof"])
    p, off = EV.world(base, k, a)
    tr = core.make_track(a["track"], base["line_width_mm"])
    sim = core.Sim(p, tr, start_lateral_mm=off, time_limit_s=a["limit"], seed=k)
    core.SIM = sim
    g = {"__name__": "__main__", "REC": []}
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, name, "exec"), g)
        except (core.SimDone, SystemExit):
            pass
    r = sim.result or {}
    R = np.array(g["REC"], dtype=float) if g["REC"] else np.zeros((0, 9))
    fwd = float(np.mean([f for f, _ in p["sensor_pos_mm"]]))
    return name, k, r.get("reason"), r.get("time_s", 0.0), int(r.get("progress_mm", 0)), R[::3].tolist(), fwd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--over", default="{}")
    ap.add_argument("--profile", default="spike_3s_geo2")
    ap.add_argument("--fwd", type=float, default=0.0)
    ap.add_argument("--prof", default=None)
    ap.add_argument("--track", default="photo")
    ap.add_argument("--lost", type=float, default=450.0)
    ap.add_argument("--limit", type=float, default=25.0)
    ap.add_argument("--worlds", type=int, default=16)
    ap.add_argument("--weff", default="1.4,1.6")
    ap.add_argument("--png", default=None)
    ap.add_argument("--box", default="-1200,-1000,-500,-300", help="그림 범위 x0,y0,x1,y1 (트랙 좌표 mm)")
    args = ap.parse_args()
    a = dict(profile=args.profile, fwd=args.fwd, prof=json.loads(args.prof) if args.prof else None, lost=args.lost,
             track=args.track, limit=args.limit, noise=[0.3, 2.0], lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20],
             weff=EV.rng2(args.weff), wob=[0.005, 0.035])
    src = make_src(args.src, json.loads(args.over))
    jobs = [(os.path.basename(args.src), src, k, a) for k in range(args.worlds)]
    with mp.Pool(max(1, os.cpu_count() - 1)) as pool:
        res = pool.map(one, jobs, chunksize=1)
    tr = core.make_track(args.track, 19.0)
    cx = crossings(tr.path)
    print("트랙 길이 %.0fmm, 자기교차 %d곳: %s" % (tr.length, len(cx), ", ".join(
        "(%.0f,%.0f) s=%.0f/%.0f" % (x, y, tr.s[i], tr.s[j]) for x, y, i, j in cx)))
    for name, k, reason, t, prog, R, fwd in res:
        print("월드 %2d: %-7s %5.2fs 진행 %5dmm" % (k, reason, t, prog))
    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        have = {f.name for f in font_manager.fontManager.ttflist}    # 한글 글꼴: macOS · Windows · Linux 순
        plt.rcParams["font.family"] = next((f for f in ("AppleGothic", "Malgun Gothic", "NanumGothic", "Noto Sans CJK KR")
                                            if f in have), plt.rcParams["font.family"])
        plt.rcParams["axes.unicode_minus"] = False
        x0, y0, x1, y1 = (float(v) for v in args.box.split(","))
        show = res[:8]
        fig, axes = plt.subplots(2, 4, figsize=(18, 9.5), squeeze=False)
        for ax, (name, k, reason, t, prog, R, fwd) in zip(axes.flat, show):
            ax.plot(tr.path[:, 0], tr.path[:, 1], color="#222", lw=5, alpha=0.25, solid_capstyle="butt")
            for x, y, _, _ in cx:
                ax.plot(x, y, "o", ms=14, mfc="none", mec="#d62728", mew=1.5)
            R = np.array(R)
            if len(R):
                sx = R[:, 2] + fwd * np.cos(R[:, 4])
                sy = R[:, 3] + fwd * np.sin(R[:, 4])
                ax.plot(R[:, 2], R[:, 3], lw=1.0, color="#1f77b4", label="바퀴축")
                ax.plot(sx, sy, lw=1.0, color="#ff7f0e", label="가운데 센서")
                off = R[:, 6] > 0
                ax.plot(sx[off], sy[off], ".", ms=2, color="#d62728", label="라인 놓침(뱅킹)")
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
            ax.set_aspect("equal")
            ax.set_title("월드 %d: %s %.1fs 진행 %dmm" % (k, reason, t, prog), fontsize=10)
            ax.grid(alpha=0.25)
        axes.flat[0].legend(loc="lower left", fontsize=8)
        fig.suptitle("%s · %s · 8자 구역 (빨간 원 = 교차로)" % (os.path.basename(args.src), args.track))
        fig.tight_layout()
        fig.savefig(args.png, dpi=90)
        print("그림:", args.png)


if __name__ == "__main__":
    main()
