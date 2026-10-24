# =====================================================================
#  턴 직후(뱅킹 → 라인 재포착) 동작 비교: 실기 블랙박스 ↔ CPU 시뮬(실기 파일 그대로)
#
#    python3 sim/recap_stats.py --bb diag_logs/bb_2026-10-24_E7_130.txt
#    python3 sim/recap_stats.py '[["E7", {}], ["X", {"KG2": 100}]]' --src spike/spike_lf3_rule_e7_130.py --fwd 130
#
#  재포착마다 (b = 직전 뱅킹 방향):
#   회전   = 재포착 순간 뱅킹 방향으로 도는 요레이트 (°/s)
#   반대조향 = 100ms 안 뱅킹 반대쪽 조향 최대 (+면 반대로 꺾음) · 포화 = 그게 95 이상인 비율
#   되튐   = 250ms 안 뱅킹 반대 방향 요레이트 최대 (°/s, 지나쳐서 반대로 돌아 나감)
#   재뱅킹 = 400ms 안 다음 뱅킹이 같은 쪽 / 반대쪽인 비율
#   속도   = 150ms 안 최저 전진 속도 (직진 대비 %) — 시뮬만
#   이탈거리 = 뱅킹 시작~재포착 300ms 뒤까지 센서 기준점이 라인 중심에서 가장 멀어진 거리 (mm) — 시뮬만
#   과회전 = 재포착 뒤 300ms 안 로봇 방향이 새 라인 방향을 뱅킹 쪽으로 넘어 돈 최대 각 (°) — 시뮬만
#   루프 속도를 바꿔 보려면 --prof '{"call_cost_us": {"color_read": 47.152, "gyro_read": 30, "motor_dc": 415}}'
#   (motor_dc = 스텝당 파이썬 연산 비용을 dc 2회에 나눠 실은 값: 695 ≈ 641/s(실기 raw E7), 560 ≈ 735/s)
# =====================================================================
import argparse
import contextlib
import io
import json
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np      # noqa: E402
import core             # noqa: E402
import eval_lf3 as EV   # noqa: E402
import bb_parse as BP   # noqa: E402

ROOT = os.path.dirname(HERE)
ANCHOR = "        steering_dc(steer, power)\n\n\n"


def recap_events(t, steer, yaw, off, bdir, v=None, lat=None, over=None):
    t, steer, yaw, off, bdir = (np.asarray(x, dtype=float) for x in (t, steer, yaw, off, bdir))
    out = []
    starts = np.nonzero((off[:-1] == 0) & (off[1:] > 0))[0] + 1
    for i in np.nonzero((off[:-1] > 0) & (off[1:] == 0))[0] + 1:
        b = 1.0 if bdir[i - 1] > 0 else -1.0
        t0 = t[i]
        w100 = (t >= t0) & (t < t0 + 100)
        w150 = (t >= t0) & (t < t0 + 150)
        w250 = (t >= t0) & (t < t0 + 250)
        e = dict(rot=-b * yaw[i - 1], cnt=float(np.max(-b * steer[w100])), swing=float(np.max(b * yaw[w250])))
        j = np.nonzero((off[i:] > 0) & (t[i:] < t0 + 400))[0]
        e["next"] = 0 if not len(j) else (1 if (1.0 if bdir[i + j[0]] > 0 else -1.0) == b else -1)
        if v is not None:
            e["vmin"] = float(np.min(np.asarray(v)[w150]))
        w300 = (t >= t0) & (t < t0 + 300)
        if lat is not None:
            sb = starts[starts < i]
            tb = t[sb[-1]] if len(sb) else t0
            e["lat"] = float(np.max(np.asarray(lat)[(t >= tb) & (t < t0 + 300)]))
        if over is not None:
            e["over"] = float(np.max(b * np.asarray(over)[w300]))
        out.append(e)
    return out


def summary(evs):
    if not evs:
        return "재포착 없음"
    a = {k: np.array([e[k] for e in evs if k in e]) for k in ("rot", "cnt", "swing", "next", "vmin", "lat", "over")}
    s = "재포착 %3d | 회전 %4.0f°/s | 반대조향 %+5.0f 포화 %3.0f%% | 되튐 %4.0f°/s (150↑ %3.0f%%) | 재뱅킹 같은쪽 %3.0f%% 반대 %3.0f%%" % (
        len(evs), a["rot"].mean(), a["cnt"].mean(), 100 * (a["cnt"] >= 95).mean(), a["swing"].mean(),
        100 * (a["swing"] > 150).mean(), 100 * (a["next"] == 1).mean(), 100 * (a["next"] == -1).mean())
    if len(a["vmin"]):
        s += " | 최저속도 %3.0f%%" % (100 * a["vmin"].mean())
    if len(a["lat"]):
        s += " | 이탈거리 %3.0fmm 과회전 %3.0f°" % (a["lat"].mean(), a["over"].mean())
    return s


def from_bb(path):
    hdr, rows = BP.parse(open(path, encoding="utf-8").read())
    d = BP.decode(hdr, rows)
    return recap_events([x["t"] for x in d], [x["steer"] for x in d], [x["yaw"] for x in d],
                        [1 if x["bank"] else 0 for x in d], [1 if x["bdir"] == "R" else -1 for x in d])


def make_src(path, over):
    s = open(path, encoding="utf-8").read().replace("AUTO_STOP = True", "AUTO_STOP = False")
    for k, v in over.items():
        s = EV.setc(s, k, v)
    assert s.count(ANCHOR) == 1, "기록 앵커 없음"
    rec = ("        _S = __import__('core').SIM\n"
           "        REC.append((now, steer, _S.imu_rate_dps(), 1 if was_off else 0, bank_dir, _S.wl + _S.wr,"
           " _S._lat_now, _S.h, _S._pi))\n")
    return s.replace(ANCHOR, rec + ANCHOR)


def one(job):
    name, src, tname, k, a = job
    base = EV.base_profile(a["profile"], a["fwd"])
    if a.get("prof"):
        base = dict(base, **a["prof"])
    p, off = EV.world(base, k, a)
    tr = core.make_track(tname, base["line_width_mm"])
    ph = np.unwrap(np.arctan2(np.diff(tr.path[:, 1]), np.diff(tr.path[:, 0])))
    ph = np.append(ph, ph[-1])
    sim = core.Sim(p, tr, start_lateral_mm=off, time_limit_s=EV.LIMIT.get(tname, 40.0), seed=k)
    core.SIM = sim
    g = {"__name__": "__main__", "REC": []}
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, name, "exec"), g)
        except (core.SimDone, SystemExit):
            pass
    r = sim.result or {}
    R = np.array(g["REC"], dtype=float) if g["REC"] else np.zeros((0, 9))
    evs, hz = [], 0.0
    if len(R) > 50:
        v = R[:, 5] / max(1e-6, np.percentile(R[:, 5], 95))
        dh = (ph[R[:, 8].astype(int)] - R[:, 7] + np.pi) % (2 * np.pi) - np.pi     # 라인 방향 − 로봇 방향
        evs = recap_events(R[:, 0], R[:, 1], R[:, 2], R[:, 3], R[:, 4], v, R[:, 6], np.degrees(dh))
        hz = 1000.0 * len(R) / max(1.0, R[-1, 0] - R[0, 0])
    return name, tname, r.get("reason"), r.get("time_s", 0.0), evs, hz


def run(variants, a, procs=None):
    src_path = os.path.join(ROOT, a["src"])
    jobs = [(nm, make_src(src_path, over), t, k, a) for nm, over in variants for t in a["tracks"] for k in range(a["worlds"])]
    with mp.Pool(procs or max(1, os.cpu_count() - 1)) as pool:
        res = pool.map(one, jobs, chunksize=2)
    lines = []
    for t in a["tracks"]:
        lines.append("-- " + t)
        for nm, _ in variants:
            rows = [r for r in res if r[0] == nm and r[1] == t]
            fin = [r for r in rows if r[2] == "finish"]
            evs = [e for r in rows for e in r[4]]
            lines.append("%-10s %4.0f/s 완주 %2d/%d %5.1fs | %s" % (nm, np.mean([r[5] for r in rows]), len(fin), len(rows),
                         sum(r[3] for r in fin) / max(1, len(fin)), summary(evs)))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("variants", nargs="?", default=None, help="JSON 목록 [[이름, {상수: 값}], ...] 또는 그 파일")
    ap.add_argument("--bb", default=None, help="실기 블랙박스 파일 (여러 개는 쉼표)")
    ap.add_argument("--worlds", type=int, default=16)
    ap.add_argument("--tracks", default="photo,rightangle")
    ap.add_argument("--profile", default="spike_3s_geo")
    ap.add_argument("--fwd", type=float, default=0.0)
    ap.add_argument("--src", default="spike/spike_lf3_rule.py")
    ap.add_argument("--noise", default="0.3,1.0")
    ap.add_argument("--prof", default=None, help="프로파일 덮어쓰기 JSON (예: 루프 속도 = call_cost_us)")
    ap.add_argument("--weff", default="1.15,1.5", help="유효 윤거 배율 범위")
    args = ap.parse_args()
    if args.bb:
        for f in args.bb.split(","):
            print("실기 %s\n  %s" % (os.path.basename(f), summary(from_bb(f))))
    if args.variants:
        v = args.variants
        variants = json.load(open(v)) if os.path.exists(v) else json.loads(v)
        a = dict(worlds=args.worlds, tracks=args.tracks.split(","), profile=args.profile, fwd=args.fwd, src=args.src,
                 noise=EV.rng2(args.noise), lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=EV.rng2(args.weff),
                 wob=[0.005, 0.035], prof=json.loads(args.prof) if args.prof else None)
        print(run(variants, a))


if __name__ == "__main__":
    main()
