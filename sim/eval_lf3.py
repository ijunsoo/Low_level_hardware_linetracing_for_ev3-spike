# =====================================================================
#  3센서 규칙 코드(실기 파일 그대로) CPU 평가 — 상수만 바꿔 여러 설정을 같은 월드에서 비교
#
#    python3 sim/eval_lf3.py '[["E2", {}], ["E5", {"KP": 0.088, "ERR_MODE": 1}]]' --worlds 48
#    python3 sim/eval_lf3.py 설정.json --fwd 100 --noise 0.5,1.5
#
#  월드: 실측 치수 프로파일(spike_3s_geo) + 무작위 구동 지연·자이로 지연·회전 권한·바퀴 흔들림·노이즈
#  출력: 트랙별 완주, 평균 시간, 직선 흔들림(요레이트 SD·조향 SD·스텝당 조향 변화·좌우 SD·초당 조향 부호 바뀜),
#        흐름(피벗 = 전체 주행 중 |조향| ≥ 80 — 거의 제자리 회전 — 비율: 높을수록 뚝뚝 끊김)
# =====================================================================
import argparse
import contextlib
import io
import json
import math
import multiprocessing as mp
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np      # noqa: E402
import core             # noqa: E402
import sweep_lf3        # noqa: E402

ROOT = os.path.dirname(HERE)
LIMIT = {"photo": 40.0, "rightangle": 20.0, "mrc": 40.0}


def setc(src, name, val):
    pat = re.compile(r"(?m)^(%s = )([^#\n]*?)(\s*#.*)?$" % re.escape(name))
    if not pat.search(src):
        raise KeyError(name)
    return pat.sub(lambda m: m.group(1) + repr(val) + (m.group(3) or ""), src, count=1)


def make_src(path, over):
    s = open(path, encoding="utf-8").read().replace("AUTO_STOP = True", "AUTO_STOP = False")
    for k, v in over.items():
        s = setc(s, k, v)
    a = "        steering_dc(steer, power)\n\n\n"
    assert s.count(a) == 1, "기록 앵커 없음"
    s = s.replace(a, "        STP.append((now, steer, __import__('core').SIM.x, __import__('core').SIM.y,"
                     " __import__('core').SIM.h, __import__('core').SIM._pi, 1 if was_off else 0))\n" + a)
    return s


def base_profile(profile, fwd):
    base = core.load_profile(profile)
    if fwd:
        sp = abs(float(base["sensor_pos_mm"][0][1]))
        base = dict(base, sensor_pos_mm=[[fwd, sp], [fwd, -sp], [fwd, 0.0]], track_ref_mm=float(fwd))
    return base


def world(base, k, a):
    cfg = {"_ASYNC": True, "_CMDLAG": a["cmdlag"], "_IMULAG": a["imulag"], "_WEFF_RANGE": a["weff"],
           "_NOISE": a["noise"], "_LAG_MS": a["lag"]}
    p, off = sweep_lf3.make_world(base, k, cfg)
    if a["wob"][1] > 0:
        p = dict(p, wobble_seed=k, wheel_wobble=float(np.random.default_rng(95000 + k).uniform(*a["wob"])))
    return p, off


_TR = {}


def track(name):
    if name not in _TR:
        tr = core.make_track(name, 19.0)
        p = tr.path
        hd = np.unwrap(np.arctan2(np.diff(p[:, 1]), np.diff(p[:, 0])))
        hd = np.append(hd, hd[-1])
        s = tr.s
        i0 = np.searchsorted(s, s - 120)
        i1 = np.minimum(np.searchsorted(s, s + 120), len(s) - 1)
        _TR[name] = (tr, hd, np.abs(hd[i1] - hd[i0]) < math.radians(4))
    return _TR[name]


def one(job):
    name, src, tname, k, a = job
    base = base_profile(a["profile"], a["fwd"])
    if a.get("prof"):
        base = dict(base, **a["prof"])
    p, off = world(base, k, a)
    tr, hd, straight = track(tname)
    sim = core.Sim(p, tr, start_lateral_mm=off, time_limit_s=LIMIT.get(tname, 40.0), seed=k)
    core.SIM = sim
    g = {"__name__": "__main__", "STP": []}
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, name, "exec"), g)
        except (core.SimDone, SystemExit):
            pass
    r = sim.result or {}
    m = {}
    S = np.array(g["STP"], dtype=float) if g["STP"] else np.zeros((0, 7))
    if len(S) > 50:
        m["pivot"] = float(np.mean(np.abs(np.clip(S[:, 1], -100, 100)) >= 80))
        pi = S[:, 5].astype(int)
        off_ = S[:, 6] > 0
        t_ms = S[:, 0]
        last_off = np.maximum.accumulate(np.where(off_, t_ms, -1e9))
        mask = straight[pi] & ~off_ & (t_ms - last_off > 300) & (t_ms - t_ms[0] > 500)
        t = t_ms / 1000.0
        tg = np.arange(t[0], t[-1], 0.01)
        if len(tg) > 3:
            hg = np.interp(tg, t, np.unwrap(S[:, 4]))
            yr = np.degrees(np.diff(hg)) / 0.01
            mg = np.interp(tg[:-1], t, mask.astype(float)) > 0.99
            if mg.sum() > 20:
                m["yaw_sd"] = float(np.std(yr[mg]))
        if mask.sum() > 20:
            st = S[:, 1]
            m["steer_sd"] = float(np.std(st[mask]))
            d = np.abs(np.diff(st))[mask[1:] & mask[:-1]]
            m["dsteer"] = float(d.mean()) if len(d) else 0.0
            sg = np.sign(np.where(np.abs(st) > 8, st, 0))[mask]
            sg = sg[sg != 0]
            dur = mask.sum() / max(1, len(S)) * (t[-1] - t[0])
            m["flip_hz"] = float((np.diff(sg) != 0).sum() / max(1e-3, dur))
            m["bank_frac"] = float(off_.mean())
    return name, tname, k, r.get("reason"), r.get("time_s", 0.0), int(r.get("progress_mm", 0)), m


def run(variants, a, procs=None):
    src_path = os.path.join(ROOT, a["src"])
    jobs = [(nm, make_src(src_path, over), t, k, a) for nm, over in variants for t in a["tracks"] for k in range(a["worlds"])]
    with mp.Pool(procs or max(1, os.cpu_count() - 1)) as pool:
        res = pool.map(one, jobs, chunksize=2)
    out = {}
    for nm, t, k, reason, tt, prog, m in res:
        out.setdefault((nm, t), []).append((reason, tt, prog, m))
    return out


def report(variants, out, tracks):
    lines = []
    for t in tracks:
        lines.append("-- " + t)
        for nm, _ in variants:
            rows = out[(nm, t)]
            fin = [r for r in rows if r[0] == "finish"]

            def avg(key):
                xs = [r[3][key] for r in rows if key in r[3]]
                return sum(xs) / len(xs) if xs else float("nan")
            lines.append("%-14s 완주 %2d/%d %5.1fs | 직선 요SD %5.1f 조향SD %5.1f Δ조향 %4.2f 왕복 %4.1f/s 뱅킹 %3.0f%% 피벗 %3.0f%% | 이탈 위치 %s" % (
                nm, len(fin), len(rows), sum(r[1] for r in fin) / max(1, len(fin)), avg("yaw_sd"), avg("steer_sd"),
                avg("dsteer"), avg("flip_hz"), 100 * avg("bank_frac"), 100 * avg("pivot"),
                sorted(r[2] for r in rows if r[0] != "finish")[:8]))
    return "\n".join(lines)


def rng2(s):
    lo, hi = (float(x) for x in s.split(","))
    return [lo, hi]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("variants", help="JSON 목록 [[이름, {상수: 값}], ...] 또는 그 파일")
    ap.add_argument("--worlds", type=int, default=24)
    ap.add_argument("--tracks", default="photo,rightangle")
    ap.add_argument("--profile", default="spike_3s_geo")
    ap.add_argument("--fwd", type=float, default=0.0, help="센서 바퀴축 앞 거리 mm (0 = 프로파일 그대로)")
    ap.add_argument("--src", default="spike/spike_lf3_rule.py")
    ap.add_argument("--noise", default="0.3,1.0")
    ap.add_argument("--lag", default="3,8")
    ap.add_argument("--cmdlag", default="0,35")
    ap.add_argument("--imulag", default="0,20")
    ap.add_argument("--weff", default="1.15,1.5")
    ap.add_argument("--wob", default="0.005,0.035")
    ap.add_argument("--json", default=None, help="결과 저장")
    ap.add_argument("--prof", default=None, help="프로파일 덮어쓰기 JSON (예: 센서 흰/검 값으로 대비 바꾸기)")
    args = ap.parse_args()
    v = args.variants
    variants = json.load(open(v)) if os.path.exists(v) else json.loads(v)
    a = dict(worlds=args.worlds, tracks=args.tracks.split(","), profile=args.profile, fwd=args.fwd, src=args.src,
             noise=rng2(args.noise), lag=rng2(args.lag), cmdlag=rng2(args.cmdlag), imulag=rng2(args.imulag),
             weff=rng2(args.weff), wob=rng2(args.wob), prof=json.loads(args.prof) if args.prof else None)
    out = run(variants, a)
    print(report(variants, out, a["tracks"]))
    if args.json:
        json.dump({"%s|%s" % k: v for k, v in out.items()}, open(args.json, "w"), ensure_ascii=False)


if __name__ == "__main__":
    main()
