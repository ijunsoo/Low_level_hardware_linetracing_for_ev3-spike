# =====================================================================
#  게인 자동 탐색 + 도메인 랜덤화
#  - V1Controller(원본 pid_control 이식본)의 kp/kd/st 조합을 무작위 탐색
#  - 각 조합을 '조건을 흔든' 여러 세계(tau/전압/노이즈/시작오프셋 변주)에서
#    돌려 평균 점수로 랭킹 → 시뮬 1등이 아니라 '흔들어도 강한' 값을 뽑는다
#
#  사용법:
#    python3 sim/sweep.py                     # 40조합 × 3세계, main 트랙
#    python3 sim/sweep.py --n 80 --worlds 5 --track hard
# =====================================================================
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core                       # noqa: E402
from controller import V1Controller   # noqa: E402

# MRC_line_following.py 하단의 현재 실측 보정값 (2026-07-04 라인스캔)
VALUE = [45, 5, 42, 5, 31, 3, 25, 2]
W_OFFSET = 8
DEC_V = 0.00001
KI = 0.00017


def randomize(base, rng):
    p = dict(base)
    p["motor_tau_s"] = base["motor_tau_s"] * rng.uniform(0.7, 1.3)
    p["battery_scale"] = rng.uniform(0.85, 1.0)
    p["sensor_noise"] = rng.uniform(0.5, 2.0)
    p["sensor_spacing_mm"] = base["sensor_spacing_mm"] + rng.uniform(-2, 2)
    return p


def score(result, track_len):
    # 낮을수록 좋음. 완주: 시간 + 흔들림 페널티 / 미완주: 진행도만큼 감점된 큰 페널티
    if result["reason"] == "finish":
        return result["time_s"] + 0.3 * result["mean_lat_mm"]
    return 1000.0 - 900.0 * result["progress_mm"] / track_len


def run_one(gains, params, track, seed, offset):
    sim = core.Sim(params, track, start_lateral_mm=offset, seed=seed,
                   time_limit_s=60.0)
    kp, kd, st, power = gains
    factory = lambda eo: V1Controller(power, DEC_V, st, kp, KI, kd,
                                      VALUE, W_OFFSET, error_offset=eo)
    return core.run_controller(sim, factory)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40, help="탐색할 게인 조합 수")
    ap.add_argument("--worlds", type=int, default=3, help="조합당 랜덤 세계 수")
    ap.add_argument("--track", default="mrc", choices=sorted(core.TRACKS))
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    base = core.load_params()
    track = core.make_track(args.track, base["line_width_mm"])

    # 현재 실기값을 0번 후보로 포함 (베이스라인)
    configs = [(0.26, 3.0, 75, 100)]
    for _ in range(args.n - 1):
        configs.append((
            round(float(np.exp(rng.uniform(np.log(0.05), np.log(0.6)))), 3),  # kp
            round(float(rng.uniform(0.5, 6.0)), 2),                           # kd
            int(rng.uniform(50, 90)),                                         # st
            int(rng.uniform(60, 100)),                                        # power
        ))

    rows = []
    for ci, g in enumerate(configs):
        scores, results = [], []
        for w in range(args.worlds):
            wp = randomize(base, rng)
            off = float(rng.uniform(-10, 10))
            r = run_one(g, wp, core.make_track(args.track, base["line_width_mm"]),
                        seed=1000 * ci + w, offset=off)
            scores.append(score(r, track.length))
            results.append(r)
        fin = sum(1 for r in results if r["reason"] == "finish")
        rows.append({
            "kp": g[0], "kd": g[1], "st": g[2], "power": g[3],
            "score": round(float(np.mean(scores)), 2),
            "finish": "%d/%d" % (fin, args.worlds),
            "time_s": round(float(np.mean([r["time_s"] for r in results
                                           if r["reason"] == "finish"] or [0])), 2),
            "lat_mm": round(float(np.mean([r["mean_lat_mm"] for r in results])), 1),
        })
        tag = " ← 현재 실기값" if ci == 0 else ""
        print("[%2d/%d] kp=%.3f kd=%.2f st=%d pw=%d → score %.1f (완주 %s)%s"
              % (ci + 1, len(configs), g[0], g[1], g[2], g[3],
                 rows[-1]["score"], rows[-1]["finish"], tag))

    rows.sort(key=lambda r: r["score"])
    print("\n=== TOP 10 (점수 낮을수록 좋음, 실기 검증 후보) ===")
    print("%8s %6s %4s %5s | %7s %7s %8s %8s" %
          ("kp", "kd", "st", "power", "score", "완주", "시간(s)", "흔들림(mm)"))
    for r in rows[:10]:
        print("%8.3f %6.2f %4d %5d | %7.1f %7s %8.2f %8.1f" %
              (r["kp"], r["kd"], r["st"], r["power"], r["score"], r["finish"],
               r["time_s"], r["lat_mm"]))

    out = os.path.join(core.PKG_DIR, "sweep_result.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    print("\n전체 결과 저장:", out)


if __name__ == "__main__":
    main()
