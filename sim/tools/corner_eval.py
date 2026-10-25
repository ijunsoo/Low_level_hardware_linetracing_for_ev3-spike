# 변형 평가 (루프 속도·--base 공통 상수·--dump 월드별 저장 포함): 완주·시간 + 센서 기준점이 라인 중심에서 가장 멀어진 거리(최대 이탈) + 코너 모드 발동·뱅킹 횟수
#   python3 corner_eval.py 변형.json --src spike/... --profile spike_3s_geo --fwd 170 --prof '{"gear_ratio": 3.3333}'
import argparse
import contextlib
import io
import json
import multiprocessing as mp
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import numpy as np      # noqa: E402
import core             # noqa: E402
import eval_lf3 as EV   # noqa: E402

ANCHOR = "        steering_dc(steer, power)\n\n\n"
LIMIT = {"rightangle": 20.0}


def make_src(path, over):
    s = open(path, encoding="utf-8").read().replace("AUTO_STOP = True", "AUTO_STOP = False")
    for k, v in over.items():
        s = EV.setc(s, k, v)
    assert s.count(ANCHOR) == 1
    cd = "1 if corner_dir else 0" if "corner_dir" in s else "0"     # 최종판(spike_final 등)엔 코너 고정 기능이 없음
    rec = ("        _S = __import__('core').SIM\n"
           "        REC.append((now, steer, _S._lat_now, 1 if was_off else 0, %s))\n" % cd)
    return s.replace(ANCHOR, rec + ANCHOR)


def one(job):
    core.Sim.LOST_MM = job[-1]["lost"]
    name, src, tname, k, a = job
    base = EV.base_profile(a["profile"], a["fwd"])
    if a.get("prof"):
        base = dict(base, **a["prof"])
    p, off = EV.world(base, k, a)
    tr = core.make_track(tname, base["line_width_mm"])
    sim = core.Sim(p, tr, start_lateral_mm=off, time_limit_s=LIMIT.get(tname, 40.0), seed=k)
    core.SIM = sim
    g = {"__name__": "__main__", "REC": []}
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, name, "exec"), g)
        except (core.SimDone, SystemExit):
            pass
    r = sim.result or {}
    R = np.array(g["REC"], dtype=float) if g["REC"] else np.zeros((0, 5))
    m = {}
    if len(R) > 10:
        m["latmax"] = float(R[:, 2].max())
        m["banks"] = int(((R[1:, 3] > 0) & (R[:-1, 3] == 0)).sum())
        m["corners"] = int(((R[1:, 4] > 0) & (R[:-1, 4] == 0)).sum())
        m["pivot"] = float(np.mean(np.abs(np.clip(R[:, 1], -100, 100)) >= 80))
        m["hz"] = float(len(R) * 1000.0 / max(1.0, R[-1, 0] - R[0, 0]))
    return name, tname, r.get("reason"), r.get("time_s", 0.0), int(r.get("progress_mm", 0)), m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("variants")
    ap.add_argument("--src", required=True)
    ap.add_argument("--profile", default="spike_3s_geo")
    ap.add_argument("--fwd", type=float, default=170.0)
    ap.add_argument("--prof", default=None)
    ap.add_argument("--tracks", default="rightangle,main")
    ap.add_argument("--lost", type=float, default=200.0, help="이탈 판정 거리 mm (시뮬 기본 200)")
    ap.add_argument("--worlds", type=int, default=32)
    ap.add_argument("--noise", default="0.3,2.0")
    ap.add_argument("--base", default="{}", help="모든 변형에 먼저 넣을 상수 JSON")
    ap.add_argument("--weff", default="1.15,1.5", help="회전 권한 범위 (geo2는 1.4,1.6)")
    ap.add_argument("--dump", default=None, help="월드별 결과 JSON 저장 경로")
    args = ap.parse_args()
    v = args.variants
    variants = json.load(open(v, encoding="utf-8")) if os.path.exists(v) else json.loads(v)
    a = dict(profile=args.profile, fwd=args.fwd, prof=json.loads(args.prof) if args.prof else None, lost=args.lost,
             noise=EV.rng2(args.noise), lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=EV.rng2(args.weff), wob=[0.005, 0.035])
    tracks = args.tracks.split(",")
    bo = json.loads(args.base)
    jobs = [(nm, make_src(os.path.join(ROOT, args.src), dict(bo, **over)), t, k, a) for nm, over in variants for t in tracks
            for k in range(args.worlds)]
    with mp.Pool(max(1, os.cpu_count() - 1)) as pool:
        res = pool.map(one, jobs, chunksize=2)
    if args.dump:
        json.dump([[r[0], r[1], r[2], r[3], r[4], r[5], j[3]] for r, j in zip(res, jobs)], open(args.dump, "w", encoding="utf-8"))
    for t in tracks:
        print("-- " + t)
        for nm, _ in variants:
            rows = [r for r in res if r[0] == nm and r[1] == t]
            fin = [r for r in rows if r[2] == "finish"]

            def avg(key):
                xs = [r[5][key] for r in rows if key in r[5]]
                return sum(xs) / len(xs) if xs else float("nan")
            print("%-16s 완주 %2d/%d %5.2fs | 최대 이탈 %4.0fmm | 뱅킹 %4.1f회 피벗 %3.0f%% | 루프 %4.0f/s | 실패 위치 %s" % (
                nm, len(fin), len(rows), sum(r[3] for r in fin) / max(1, len(fin)), avg("latmax"),
                avg("banks"), 100 * avg("pivot"), avg("hz"), sorted(r[4] for r in rows if r[2] != "finish")[:6]))


if __name__ == "__main__":
    main()
