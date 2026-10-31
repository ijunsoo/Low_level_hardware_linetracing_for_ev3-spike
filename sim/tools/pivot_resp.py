# 제자리 회전(|조향| ≥ 85) 시작 뒤 회전 속도가 얼마나 빨리 붙나 — 실기 블랙박스와 시뮬을 같은 블랙박스 기록으로 비교
#   (완성판 로그는 기록마다 실제 간격을 적어 그 시각으로 셈, 예전 7바이트 로그·시뮬은 4ms 간격)
#   python3 pivot_resp.py diag_logs/bb_*.txt                                  (실기 로그만)
#   python3 pivot_resp.py diag_logs/bb_2026-10-24_E8R_2x_short.txt --sim spike_3s_geo2x --sim spike_3s_geo2:'{"gear_ratio": 3.3333}'
#   기어만 2배로 바꾼 geo2는 40ms에 +344°/s, 실기 2배는 +144°/s → 토크 한계를 넣은 geo2x(시정수 ×2: +199, 60ms +248 vs 실기 +200)가 실기에 가까움
#   (2026-10-31: 회전 시작 몇 번이 아니라 2배 로그 7.3초의 조향 명령 전체를 재생해 geo2x를 다시 맞춤 — 지연 40ms · 회전 0.05초 · w_eff 3.1.
#    위 숫자는 그 전 모형. 재생 맞추기는 docs/PHYSICS_ENGINE.ko.md 6절)
import argparse
import contextlib
import io
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import numpy as np      # noqa: E402
import bb_parse as B    # noqa: E402

HORIZON = (20, 40, 60, 80, 100)


def onsets(steer, yaw, t):
    # t: 기록 시각(ms) — 실기 완성판 로그는 기록마다 실제 간격, 예전 로그·시뮬은 4ms 간격
    out = []
    n = len(steer)
    for i in range(1, n):
        if abs(steer[i]) >= 85 and abs(steer[i - 1]) < 60:
            sg = 1 if steer[i] > 0 else -1          # 조향 + = 우회전 = 요레이트 −
            row = []
            for h in HORIZON:
                j = i
                while j < n and t[j] - t[i] < h:
                    j += 1
                same = j < n and all(abs(steer[k]) >= 85 and (steer[k] > 0) == (sg > 0) for k in range(i, j + 1))
                row.append(-sg * (yaw[j] - yaw[i]) if same else np.nan)
            out.append(row)
    return np.array(out) if out else np.zeros((0, len(HORIZON)))


def real(path):
    with contextlib.redirect_stdout(io.StringIO()):
        hdr, rows = B.parse(open(path, encoding="utf-8").read())
    d = B.decode(hdr, rows)
    return onsets([r["steer"] for r in d], [r["yaw"] for r in d], [r["t"] for r in d])


def sim(profile, prof, worlds, track):
    import core
    import eval_lf3 as EV
    src = open(os.path.join(ROOT, "spike/spike_lf3_rule_e8r8_170.py"), encoding="utf-8").read()
    src = src.replace("AUTO_STOP = True", "AUTO_STOP = False")
    for kk, v in {"RAW_MODE": False, "VALUE": [34, 6, 22, 3, 21, 3], "AUTO_CAL": False, "LOG_N": 6000}.items():
        src = EV.setc(src, kk, v)
    base = dict(EV.base_profile(profile, 170), **prof)
    w = base.get("w_eff_scale", 1.5)                # 미끄러짐은 모형 값 ±7% (geo2 1.5 → 1.4~1.6, geo2x 3.1 → 2.9~3.3)
    a = dict(noise=[0.3, 2.0], lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=[w * 1.4 / 1.5, w * 1.6 / 1.5],
             wob=[0.005, 0.035])
    core.Sim.LOST_MM = 450.0
    ev = []
    for k in range(worlds):
        p, off = EV.world(base, k, a)
        s = core.Sim(p, core.make_track(track, base["line_width_mm"]), start_lateral_mm=off, time_limit_s=20.0, seed=k)
        core.SIM = s
        g = {"__name__": "__main__"}
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                exec(compile(src, "sim", "exec"), g)
            except (core.SimDone, SystemExit):
                pass
        L, n = g["LOG"], g["STAT"][1]
        N = len(L) // 7
        rec = [L[(i % N) * 7:(i % N) * 7 + 7] for i in range(max(0, n - N), n)]
        ev.append(onsets([r[3] - 128 for r in rec], [(r[6] - 128) * 8 for r in rec], [4 * k for k in range(len(rec))]))
    return np.vstack(ev)


def show(name, E):
    cnt = [int(np.sum(~np.isnan(E[:, i]))) for i in range(E.shape[1])] if len(E) else [0] * len(HORIZON)
    m = [float(np.nanmean(E[:, i])) if c else float("nan") for i, c in enumerate(cnt)]
    print("%-40s 회전 %3d번 | 시작 뒤 회전 속도 변화(°/s) %s  (표본 %s)" % (
        name, len(E), " · ".join("%dms %5.0f" % (h, v) for h, v in zip(HORIZON, m)), cnt))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="*", help="실기 블랙박스 로그 (bb_parse 형식)")
    ap.add_argument("--sim", action="append", default=[], help="프로파일 또는 프로파일:'{덮어쓸 값 JSON}'")
    ap.add_argument("--worlds", type=int, default=6)
    ap.add_argument("--track", default="main")
    args = ap.parse_args()
    for path in args.logs:
        show("실기 " + os.path.basename(path), real(path))
    for spec in args.sim:
        name, _, over = spec.partition(":")
        show("시뮬 " + spec, sim(name, json.loads(over) if over else {}, args.worlds, args.track))


if __name__ == "__main__":
    import warnings
    warnings.filterwarnings("ignore")
    main()
