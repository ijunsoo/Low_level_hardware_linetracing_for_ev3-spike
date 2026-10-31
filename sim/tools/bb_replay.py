# 블랙박스 재생: 실기 블랙박스에 기록된 조향 명령을 시뮬 구동 모형에 그대로 넣고, 시뮬 요레이트가 실기 자이로와
#   얼마나 맞는지 기록 전체로 비교 (pivot_resp.py는 제자리 회전 시작 몇 번만 봄 — 이건 모든 순간)
#   완성판 8바이트 로그용 (조향 → 모터는 완성판 steering_dc 그대로, 파워 0 = 브레이크 기록은 지원 안 함)
#   python3 sim/tools/bb_replay.py diag_logs/bb_2026-10-31_2x_6s.txt diag_logs/bb_2026-10-31_2x_final.txt@-1100 --profile spike_3s_geo2x
#   python3 sim/tools/bb_replay.py <로그...> --profile spike_3s_geo2x --fit      (지연 · 회전 시정수 · w_eff 격자 탐색)
#   로그@-1100 = 정지 1.1초 전부터만 (앞쪽이 걸리거나 들린 구간일 때). 끝에 세 센서가 모두 0 근처인 구간(들어 올림)은 저절로 뺌
#   2026-10-31 2배 로그 7.3초: 예전 2배 모형(지연 32ms · 회전 0.10초 · w_eff 1.5) 154°/s → 지연 40ms · 회전 0.05초 · w_eff 3.1 62°/s
import argparse
import contextlib
import io
import itertools
import json
import math
import os
import sys
from multiprocessing import Pool

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import numpy as np      # noqa: E402
import bb_parse as B    # noqa: E402
import core             # noqa: E402

WARM_MS = 200           # 첫 명령으로 맞춘 정상 상태가 실제와 다를 수 있어 처음 0.2초는 오차에서 뺌


def load(spec):
    path, _, t_from = spec.partition("@")
    with contextlib.redirect_stdout(io.StringIO()):
        hdr, rows = B.parse(open(path, encoding="utf-8").read())
    if hdr["w"] < 8:
        raise SystemExit("%s: 기록 간격이 없는 예전 로그 (완성판 8바이트 로그만)" % path)
    d = [r for r in B.decode(hdr, rows) if r["t"] >= (float(t_from) if t_from else -1e18)]
    end = len(d)
    while end > 0 and max(d[end - 1]["rL"], d[end - 1]["rR"], d[end - 1]["rF"]) <= 2:
        end -= 1
    if any(r["power"] == 0 for r in d[:end]):
        print("※ %s: 브레이크 기록(파워 0)이 있음 — 브레이크 방향을 몰라 그 순간은 조향대로 재생" % path)
    return d[:end]


def duties(steer, power):
    # 완성판 steering_dc: 조향 ±100 제한, 100이면 안쪽 바퀴 역회전
    s = max(-100, min(100, steer))
    slope = power * 2 / 100
    if s < 0:
        return round(slope * s + power, 2), power
    return power, round(-slope * s + power, 2)


def replay(profile, over, d):
    p = dict(core.load_profile(profile), **over)
    p["wheel_wobble"] = 0.0
    sim = core.Sim(p, core.make_track("main", p["line_width_mm"]), seed=0)
    sim.charge = lambda kind: None                        # 호출 비용으로 시계가 움직이지 않게 (시각은 로그대로)
    sim._check_progress = lambda *a, **k: None            # 트랙 이탈·완주 판정 끔 (요레이트만 봄)
    sim.time_limit = 1e9
    core.SIM = sim
    ports = (p["left_motor_port"], p["right_motor_port"])
    sign = {pt: (1 if sim._motors[pt][1] > 0 else -1) for pt in ports}

    def cmd(r):
        lr = duties(r["steer"], r["power"] or 100)
        for pt, duty in zip(ports, lr):
            sim.motor_dc(pt, duty * sign[pt])

    def run_to(t):
        while sim.t < t - 1e-9:
            sim.advance(min(0.001, t - sim.t))

    cmd(d[0])
    run_to(0.6)                                           # 첫 명령으로 정상 상태까지
    t0 = d[0]["t"] / 1000.0 - sim.t
    pred = np.zeros(len(d))
    for i, r in enumerate(d):
        run_to(r["t"] / 1000.0 - t0)
        pred[i] = math.degrees(getattr(sim, "_w_true", 0.0))   # 그 줄의 조향 명령 전에 읽힌 요레이트
        cmd(r)
    return pred


def error(profile, over, d):
    pred = replay(profile, over, d)
    meas = np.array([r["yaw"] + 4 for r in d], float)            # 기록은 //8 내림 → +4가 가운데
    t = np.array([r["t"] for r in d], float)
    m = t - t[0] >= WARM_MS
    return float(np.sqrt(np.mean((pred[m] - meas[m]) ** 2))), int(m.sum())


def _job(args):
    profile, over, logs = args
    return over, [error(profile, over, d) for d in logs]


def total(errs):
    return math.sqrt(sum(e * e * n for e, n in errs) / max(1, sum(n for _, n in errs)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+", help="블랙박스 파일 (경로@-1100 = 정지 1.1초 전부터만)")
    ap.add_argument("--profile", default="spike_3s_geo2x")
    ap.add_argument("--prof", default="", help='프로파일 덮어쓰기 JSON, 예: \'{"w_eff_scale": 3.1}\'')
    ap.add_argument("--fit", action="store_true", help="구동 지연 · 회전 시정수 · w_eff 격자 탐색")
    args = ap.parse_args()
    logs = [load(s) for s in args.logs]
    for s, d in zip(args.logs, logs):
        print("%s: 기록 %d개 (%.1f초)" % (s, len(d), (d[-1]["t"] - d[0]["t"]) / 1000.0))
    over = json.loads(args.prof) if args.prof else {}
    p = dict(core.load_profile(args.profile), **over)
    errs = [error(args.profile, over, d) for d in logs]
    print("%s (지연 %.0fms · 회전 %.3f초 · w_eff %.2f): 실기 자이로와 차이 %s · 합 %.0f°/s" % (
        args.profile, p.get("cmd_lag_ms", 0), p.get("yaw_tau_s", 0), p.get("w_eff_scale", 1), " / ".join(
            "%.0f" % e for e, _ in errs), total(errs)))
    if not args.fit:
        return
    grid = [dict(over, cmd_lag_ms=L, yaw_tau_s=y, w_eff_scale=w)
            for L, y, w in itertools.product([24, 32, 40, 48, 56], [0.035, 0.05, 0.075, 0.1, 0.15, 0.2],
                                             [1.3, 1.5, 1.8, 2.1, 2.5, 2.8, 3.1, 3.4, 3.8])]
    with Pool() as pool:
        res = pool.map(_job, [(args.profile, g, logs) for g in grid], chunksize=4)
    res.sort(key=lambda x: total(x[1]))
    print("가장 잘 맞는 10개:")
    for g, e in res[:10]:
        print("  지연 %2dms · 회전 %.3f초 · w_eff %.1f → %s · 합 %.0f°/s" % (
            g["cmd_lag_ms"], g["yaw_tau_s"], g["w_eff_scale"], " / ".join("%.0f" % x for x, _ in e), total(e)))


if __name__ == "__main__":
    main()
