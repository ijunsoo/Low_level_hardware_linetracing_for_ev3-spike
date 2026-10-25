# =====================================================================
#  실기 진단 로그(spike_diag.py / ev3_diag.py 출력) 해석 → 시뮬 보정값 제안
#  + 같은 진단 프로그램을 시뮬(core+목)에서 돌려 '시뮬 지문'과 나란히 비교
#
#  사용법 (sim/ 기준, numpy 로컬 파이썬):
#    python3 sim/diag_parse.py diag_log.txt                      # 실기 로그 해석
#    python3 sim/diag_parse.py --sim spike_4s                    # 시뮬 지문만 (파서 검증용)
#    python3 sim/diag_parse.py diag_log.txt --sim spike_4s       # 실기 vs 시뮬 비교
#
#  추정 방법
#   - 찢어진 읽기 p_torn: 읽기 순서상 이웃한 센서쌍 (i, i+1)마다
#       같은 루프에서 함께 바뀜(A) / 뒤쪽 i+1이 한 루프 먼저 바뀜(B) / 반대 순서(C)
#     를 세어, 동기 프레임이면 B만 커진다(앞쪽 센서는 옛 값을 한 번 더 읽음).
#     쌍별 찢어짐 확률 ≈ (B−C)/(A+B−C), p_torn ≈ 쌍별 확률의 합 (fastsim 정의와 같음)
#   - 센서 지연: [4]에서 라인 진입·이탈 감지 위치(mm)를 그때 속도(mm/s)에 회귀 →
#     기울기 = 지연(s). 정적 오프셋(문턱·발자국·장착 위치)은 절편으로 빠진다.
# =====================================================================
import argparse
import contextlib
import io
import json
import os
import re
import sys

import numpy as np

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)


# ------------------------------------------------------------ 로그 해석
def parse(text):
    d = {"masks": {}, "lag": [], "gaps": {}, "updrate": {}, "hist": {}, "torn": {}}
    for line in text.splitlines():
        p = line.strip().split(",")
        if not p or p[0].startswith("#"):
            continue
        k = p[0]
        if k == "DIAG":
            d["platform"], d["n"] = p[1], int(p[3])
        elif k == "COST":
            d["cost"] = {p[i]: int(p[i + 1]) for i in range(1, len(p) - 1, 2)}
        elif k == "UPD":
            d.setdefault("upd", {})[p[1]] = dict(iters=int(p[3]), ms=int(p[5]))
        elif k == "UPDRATE":
            d["updrate"][p[1]] = [int(x) for x in p[2:]]
        elif k == "MASKHIST":
            d["hist"][p[1]] = [int(x) for x in p[2:]]
        elif k == "TORN":
            d["torn"][p[1]] = int(p[2])
        elif k == "GAPS":
            d["gaps"].setdefault(p[1], {})[int(p[2])] = [int(x) for x in p[3:]]
        elif k == "MASKS":
            d["masks"].setdefault(p[1], {})[int(p[2])] = p[3]
        elif k == "STEPS":
            d["steps"] = dict(loops=int(p[2]), steps=int(p[4]), ms=int(p[6]))
        elif k == "LAGCFG":
            d["deg_mm"] = int(p[2]) / 1000.0
        elif k == "LAGMODE":
            d["lag_mode"] = p[1]
        elif k == "IMUX":
            d.setdefault("imux", []).append(dict(rc=int(p[1]), dir=int(p[2]), i=int(p[3]),
                                                 a_in=int(p[4]) / 100.0, w_in=int(p[5]),
                                                 a_out=int(p[6]) / 100.0, w_out=int(p[7])))
        elif k == "LAGRUN":
            d.setdefault("lagruns", []).append(dict(duty=int(p[1]), rep=int(p[2]), a0=int(p[3])))
        elif k == "LAG" and p[3] != "miss":
            d["lag"].append(dict(duty=int(p[1]), i=int(p[2]), hit_a=int(p[3]), hit_v=int(p[4]),
                                 out_a=int(p[5]), out_v=int(p[6]), white=int(p[7]), low=int(p[8]),
                                 run=len(d.get("lagruns", [])) - 1))
    for tag, chunks in d["masks"].items():
        s = "".join(chunks[a] for a in sorted(chunks))
        d["masks"][tag] = np.array([int(c, 16) for c in s], dtype=np.int64)
    return d


def tear_stats(masks, n):
    """이웃 센서쌍별 (A 동시, B 뒤쪽이 한 루프 먼저, C 반대) → 찢어짐 확률."""
    bit = [(masks >> i) & 1 for i in range(n)]
    out = []
    for i in range(n - 1):
        a, b = bit[i], bit[i + 1]
        A = int((a & b).sum())
        B = int((b[:-1] & a[1:]).sum())       # i+1(뒤)이 k에, i(앞)가 k+1에
        C = int((a[:-1] & b[1:]).sum())       # 반대 순서 (우연 일치 기준선)
        p = max(0, B - C) / max(1, A + max(0, B - C))
        out.append(dict(pair=(i, i + 1), A=A, B=B, C=C, p=round(p, 3)))
    return out


def lag_fit(rows, deg_mm):
    """센서별: 감지 위치 = 절편 + 지연 × 속도 (진입·이탈 각각) → 지연 ms."""
    res = {}
    for i in sorted(set(r["i"] for r in rows)):
        sub = [r for r in rows if r["i"] == i]
        if len(sub) < 3:
            continue
        fits = []
        for a_key, v_key in (("hit_a", "hit_v"), ("out_a", "out_v")):
            x = np.array([r[v_key] * deg_mm for r in sub])      # mm/s
            y = np.array([r[a_key] * deg_mm for r in sub])      # mm
            if np.ptp(x) < 50:
                continue
            A = np.vstack([np.ones_like(x), x]).T
            coef, res_, _, _ = np.linalg.lstsq(A, y, rcond=None)
            fits.append(coef[1] * 1000.0)                        # s → ms
        if fits:
            res[i] = dict(lag_ms=round(float(np.mean(fits)), 1),
                          entry_exit=[round(f, 1) for f in fits], n=len(sub))
    return res


V_SAFE = 350      # deg/s: 이 속도 이하로 달린 뒤엔 급제동 미끄러짐(출발점 밀림)이 1° 이하 (2026-10-17 실기)


def lag_pairs(rows, deg_mm):
    """자동복귀 로그용: 연속한 두 런의 감지 위치 차 / 속도 차 = 지연. 앞 런이 V_SAFE 이하일 때만
    (빠른 런 뒤엔 급제동 미끄러짐으로 출발점이 앞으로 밀려 감지 위치가 당겨짐 → 지연이 음수로 왜곡).
    반환: (지연 ms, 표준오차 ms, 쌍 수), 같은 속도 반복 쌍의 밀림 [(속도, °)]."""
    runs = sorted(set(r["run"] for r in rows))
    by = {k: [r for r in rows if r["run"] == k] for k in runs}
    vbar = {k: float(np.mean([r["hit_v"] for r in by[k]])) for k in runs}
    num = den = 0.0
    vals = []
    drift = []
    for a, b in zip(runs, runs[1:]):
        if b != a + 1:
            continue
        for e in ("hit", "out"):
            for ra in by[a]:
                rb = [r for r in by[b] if r["i"] == ra["i"]]
                if not rb:
                    continue
                rb = rb[0]
                da = rb[e + "_a"] - ra[e + "_a"]
                dv = rb[e + "_v"] - ra[e + "_v"]
                if abs(dv) < 60:                          # 같은 속도 반복 → 밀림 관측
                    drift.append((round(vbar[a]), -da))
                elif vbar[a] <= V_SAFE:
                    num += da * dv
                    den += dv * dv
                    vals.append(da / dv)
    if den == 0:
        return None, drift
    L = num / den
    se = float(np.std(vals) / max(1, len(vals) - 1) ** 0.5) if len(vals) > 1 else float("nan")
    return (L * 1000.0, se * 1000.0, len(vals)), drift


def imu_lag(rows):
    """[5] 자이로 기준 지연: (속도, 센서)마다 반시계·시계 검정구간 중심 차 Δ = 2ω·지연 + 정적치우침.
    Δ를 2ω에 회귀한 기울기 = 지연. 반환 (지연 ms, 표준오차 ms, 점 수, [(ω, Δ, 센서)])."""
    pts = []
    for rc in sorted(set(r["rc"] for r in rows)):
        for i in sorted(set(r["i"] for r in rows)):
            cc = [r for r in rows if r["rc"] == rc and r["i"] == i and r["dir"] > 0]
            cw = [r for r in rows if r["rc"] == rc and r["i"] == i and r["dir"] < 0]
            if not cc or not cw:
                continue
            mid = lambda rr: float(np.mean([(r["a_in"] + r["a_out"]) / 2 for r in rr]))
            w = float(np.mean([abs(r["w_in"] + r["w_out"]) / 2 for r in cc + cw]))
            pts.append((w, mid(cc) - mid(cw), i))
    if len(set(round(p[0]) for p in pts)) < 2:
        return None
    x = np.array([2 * p[0] for p in pts])
    y = np.array([p[1] for p in pts])
    A = np.vstack([x, np.ones_like(x)]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ coef
    s2 = res @ res / max(1, len(y) - 2)
    se = float(np.sqrt(s2 * np.linalg.pinv(A.T @ A)[0, 0]))
    return coef[0] * 1000.0, se * 1000.0, len(pts), pts


def report(d, label):
    n = d.get("n", 0)
    print("\n================ %s (%s, 센서 %d) ================" % (label, d.get("platform"), n))
    out = {}
    if "cost" in d:
        c = d["cost"]
        print("호출 비용 µs: read %d · imu %d · dc %d · stopwatch %d · 빈루프 %d" % (
            c.get("read_us", -1), c.get("imu_us", -1), c.get("dc_us", -1),
            c.get("stopwatch_us", -1), c.get("empty_loop_us", -1)))
        out["call_cost_us"] = {"color_read": c.get("read_us"), "motor_dc": c.get("dc_us")}
    for tag in ("hand", "drive"):
        if tag not in d.get("upd", {}):
            continue
        u = d["upd"][tag]
        loop_us = 1000.0 * u["ms"] / max(1, u["iters"])
        print("[%s] 루프 %.0fµs (%.0fHz) · 센서별 값변화 %s Hz · 찢어짐 간이집계 %d" % (
            tag, loop_us, 1e6 / loop_us, d["updrate"].get(tag), d["torn"].get(tag, 0)))
        if tag in d["masks"] and n > 1:
            ts = tear_stats(d["masks"][tag], n)
            for t in ts:
                print("   쌍 %s: 동시 %d · 뒤쪽 먼저 %d · 반대 %d → 찢어짐 %.2f" % (
                    t["pair"], t["A"], t["B"], t["C"], t["p"]))
            p_torn = round(sum(t["p"] for t in ts), 2)
            sync = sum(t["A"] for t in ts) / max(1, sum(t["A"] + t["B"] + t["C"] for t in ts))
            print("   → p_torn ≈ %.2f · 동시변화 비율 %.2f (1에 가까우면 센서들이 같은 프레임으로 갱신)"
                  % (p_torn, sync))
            out.setdefault("p_torn", {})[tag] = p_torn
    if "steps" in d:
        s = d["steps"]
        print("[주행] 루프 %d/s · 프레임동기 통과(제어) %d/s" % (
            s["loops"] * 1000 // max(1, s["ms"]), s["steps"] * 1000 // max(1, s["ms"])))
        out["steps_per_s"] = s["steps"] * 1000 // max(1, s["ms"])
    if d["lag"]:
        if d.get("lag_mode") != "return":
            print("[지연] 주의: 속도마다 사람이 다시 놓은 로그(옛 [4]) — 놓은 위치 차이가 기울기에 섞여 신뢰 낮음."
                  " 새 spike_diag.py(한 번 놓고 자동 복귀)로 다시 재기")
        lf = lag_fit(d["lag"], d.get("deg_mm", 1.0))
        for i, r in lf.items():
            print("[지연] 센서 %d: %.1f ms (진입/이탈 %s, 런 %d) — 전체 회귀" % (i, r["lag_ms"], r["entry_exit"], r["n"]))
        if d.get("lag_mode") == "return" and all("run" in r for r in d["lag"]):
            pr, drift = lag_pairs(d["lag"], d.get("deg_mm", 1.0))
            if drift:
                agg = {}
                for v, x in drift:
                    agg.setdefault(v, []).append(x)
                print("[지연] 같은 속도 반복 쌍의 출발점 밀림(°, +=앞으로): " + ", ".join(
                    "%d°/s→%.1f" % (v, float(np.mean(x))) for v, x in sorted(agg.items())))
            if pr:
                print("[지연] ★저속 런 쌍(앞 런 ≤%d°/s) 추정: %.1f ± %.1f ms (%d쌍) — 밀림에 안 흔들리는 값"
                      % (V_SAFE, pr[0], pr[1], pr[2]))
                print("       ※ 모터각 대비 상대 지연. Motor.angle() 자체가 늦으면 그만큼 실제 지연이 더 큼")
                out["sensor_lag_ms"] = round(pr[0], 1)
            elif lf:
                out["sensor_lag_ms"] = round(float(np.mean([r["lag_ms"] for r in lf.values()])), 1)
    if d.get("imux"):
        il = imu_lag(d["imux"])
        if il:
            print("[지연·자이로] ★%.1f ± %.1f ms (속도×센서 %d점) — 모터각 지연·미끄러짐과 무관한 값" % il[:3])
            for w, dm, i in il[3]:
                print("       ω %4.0f°/s 센서 %d: 반시계−시계 중심차 %+.2f°" % (w, i, dm))
            out["sensor_lag_ms_imu"] = round(il[0], 1)
            out["sensor_lag_ms"] = round(il[0], 1)
    print("프로파일 제안:", json.dumps(out, ensure_ascii=False))
    return out


# ------------------------------------------------------------ 시뮬에서 같은 진단 실행
def run_in_sim(profile, diag_file):
    import core
    import numpy as np  # noqa: F811
    src0 = open(diag_file, encoding="utf-8").read().replace("AUTO = False", "AUTO = True")
    logs = []
    base = core.load_profile(profile)
    # (1·2·3단계) 직선 라인 위 / (4단계) 흰 바닥 + 수직 라인 하나
    core.TRACKS["diag_line"] = [('S', 3000)]
    line = core.make_track("diag_line", base["line_width_mm"])
    cross_pts = np.array([(250.0, y) for y in np.arange(-200, 201, 2.0)])
    cross = core.Track(core.build_path([('S', 3000)]), base["line_width_mm"],
                       stamp_pts=cross_pts, stamp_path=False)
    for stages, track in (("(1, 2, 3, 5)", line), ("(4,)", cross)):
        src = re.sub(r"^STAGES = \([^)]*\)", "STAGES = %s" % stages, src0, count=1, flags=re.M)
        p = dict(base)
        sim = core.Sim(p, track, time_limit_s=120.0, seed=1)
        sim.LOST_MM = 1e9                     # 진단 주행은 트랙 이탈 판정 안 함
        sim.FINISH_MM = -1e9
        core.SIM = sim

        def reset(sim=sim):                   # 사람이 로봇을 출발점에 다시 놓는 것에 해당
            sim.x = sim.y = sim.h = 0.0
            sim.wl = sim.wr = 0.0
            sim._pi = 0

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            try:
                exec(compile(src, diag_file, "exec"), {"__name__": "__main__", "SIM_RESET": reset})
            except core.SimDone:
                pass
        logs.append(buf.getvalue())
    return "\n".join(logs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", help="실기 진단 로그 (diag_log.txt)")
    ap.add_argument("--sim", default=None, help="같은 진단을 이 프로파일 시뮬에서 실행해 비교")
    ap.add_argument("--diag", default=os.path.join(ROOT, "spike", "spike_diag.py"))
    ap.add_argument("--save-sim-log", default=None)
    args = ap.parse_args()
    real = sim = None
    if args.log:
        real = report(parse(open(args.log, encoding="utf-8").read()), "실기")
    if args.sim:
        text = run_in_sim(args.sim, args.diag)
        if args.save_sim_log:
            open(args.save_sim_log, "w", encoding="utf-8").write(text)
        sim = report(parse(text), "시뮬 %s" % args.sim)
    if real and sim:
        print("\n== 실기 vs 시뮬 ==")
        for k in sorted(set(real) | set(sim)):
            print("  %-15s 실기 %-28s 시뮬 %s" % (k, real.get(k), sim.get(k)))


if __name__ == "__main__":
    main()
