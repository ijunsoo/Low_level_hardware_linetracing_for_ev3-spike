# =====================================================================
#  v7(기어2 고속) 대규모 탐색 — ★무감속 제약: 순항/턴 모두 파워 100 고정
#  (브레이크 펄스 없음, 피벗도 조향 차동만 사용)
#  tau를 0.21~0.49로 랜덤화: 기어로봇 실측 전 불확실성을 통째로 흔들어
#  "tau가 뭐가 나와도 사는" 조합을 찾는다.
#
#  사용법: python3 sim/sweep7.py --n 120 --worlds 2
# =====================================================================
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core                        # noqa: E402
from controller import V7Controller    # noqa: E402

VALUE = [45, 5, 42, 5, 31, 3, 25, 2]


def randomize(base, rng):
    p = dict(base)
    p["motor_tau_s"] = base["motor_tau_s"] * rng.uniform(0.85, 1.15)  # ★실측 tau ±15%
    p["battery_scale"] = rng.uniform(0.85, 1.0)
    p["sensor_noise"] = rng.uniform(0.5, 2.0)
    p["sensor_spacing_mm"] = base["sensor_spacing_mm"] + rng.uniform(-1.5, 1.5)
    return p


def score(result, track_len):
    if result["reason"] == "finish":
        return result["time_s"] + 0.3 * result["mean_lat_mm"]
    return 1000.0 - 900.0 * result["progress_mm"] / track_len


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=120)
    ap.add_argument("--worlds", type=int, default=2)
    ap.add_argument("--track", default="mrc", choices=sorted(core.TRACKS))
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    base = core.load_params()
    base["gear_ratio"] = 2.0
    track = core.make_track(args.track, base["line_width_mm"])

    configs = []
    for _ in range(args.n):
        configs.append(dict(
            # ── 무감속 고정 ── #
            power=100, spd_k=0.0, min_power=100,
            brake_frames=0, brake_duty=0,
            turn_power=100, settle_frames=0, settle_power=100,
            # ── 탐색 변수 ── #
            kp=round(float(np.exp(rng.uniform(np.log(0.008), np.log(0.08)))), 3),
            kd=round(float(rng.uniform(2.0, 18.0)), 1),
            max_steer_delta=int(rng.uniform(15, 60)),
            ema_alpha=int(rng.uniform(400, 900)),
            d_filter=int(rng.uniform(150, 500)),
            turn_st=int(rng.uniform(70, 100)),
            corner_black=int(rng.uniform(15, 40)),
            reacquire=int(rng.uniform(35, 65)),
            white_confirm=int(rng.uniform(3, 20)),
            mem_frames=int(rng.uniform(60, 220)),
            st_bank=int(rng.uniform(60, 100)),
            edge_err=int(rng.uniform(40, 140)),
        ))

    rows = []
    for ci, cfg in enumerate(configs):
        scores, results = [], []
        for w in range(args.worlds):
            wp = randomize(base, rng)
            off = float(rng.uniform(-8, 8))
            sim = core.Sim(wp, track, start_lateral_mm=off,
                           seed=1000 * ci + w, time_limit_s=120.0)
            factory = lambda eo: V7Controller(VALUE, error_offset=eo, **cfg)
            r = core.run_controller(sim, factory)
            scores.append(score(r, track.length))
            results.append(r)
        fin = sum(1 for r in results if r["reason"] == "finish")
        row = dict(cfg)
        row.update({
            "score": round(float(np.mean(scores)), 2),
            "finish": "%d/%d" % (fin, args.worlds),
            "time_s": round(float(np.mean([r["time_s"] for r in results
                                           if r["reason"] == "finish"] or [0])), 2),
            "prog": int(np.mean([r["progress_mm"] for r in results])),
        })
        rows.append(row)
        print("[%3d/%d] kp=%.3f kd=%4.1f slew=%d ema=%d df=%d tst=%d wc=%d"
              " → score %.1f 완주%s 진행%dmm"
              % (ci + 1, len(configs), cfg["kp"], cfg["kd"],
                 cfg["max_steer_delta"], cfg["ema_alpha"], cfg["d_filter"],
                 cfg["turn_st"], cfg["white_confirm"],
                 row["score"], row["finish"], row["prog"]), flush=True)

    rows.sort(key=lambda r: r["score"])
    print("\n=== TOP 15 (gear 2.0, tau 0.21~0.49 랜덤) ===")
    for r in rows[:15]:
        print(json.dumps(r, ensure_ascii=False))

    out = os.path.join(core.PKG_DIR, "sweep7_result.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    print("저장:", out)


if __name__ == "__main__":
    main()
