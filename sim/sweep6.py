# =====================================================================
#  v6(정규화 + 코너피벗) 파라미터 자동 탐색
#
#  사용법:
#    python3 sim/sweep6.py --n 50 --worlds 2 --gear 1.0
#    python3 sim/sweep6.py --gear 2.0     # 기어업: 순항파워 40까지 탐색
# =====================================================================
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core                        # noqa: E402
from controller import V6Controller    # noqa: E402

VALUE = [45, 5, 42, 5, 31, 3, 25, 2]
TAU_BASE = 0.115


def randomize(base, rng):
    p = dict(base)
    p["motor_tau_s"] = base["motor_tau_s"] * rng.uniform(0.7, 1.3)
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
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--worlds", type=int, default=2)
    ap.add_argument("--track", default="mrc", choices=sorted(core.TRACKS))
    ap.add_argument("--gear", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    base = core.load_params()
    base["gear_ratio"] = args.gear
    base["motor_tau_s"] = TAU_BASE * args.gear * args.gear
    track = core.make_track(args.track, base["line_width_mm"])

    pw_lo = 40 if args.gear > 1.2 else 80   # 기어업이면 저속 순항도 탐색

    configs = [dict(kp=0.08, kd=1.0, power=100, turn_power=45, turn_st=95,
                    corner_black=25, edge_err=80)]   # 0번 = v6 기본값
    for _ in range(args.n - 1):
        configs.append(dict(
            kp=round(float(np.exp(rng.uniform(np.log(0.03), np.log(0.25)))), 3),
            kd=round(float(rng.uniform(0.3, 4.0)), 2),
            power=int(rng.uniform(pw_lo, 100)),
            turn_power=int(rng.uniform(30, 70)),
            turn_st=int(rng.uniform(70, 100)),
            corner_black=int(rng.uniform(15, 40)),
            edge_err=int(rng.uniform(40, 140)),
        ))

    rows = []
    for ci, cfg in enumerate(configs):
        scores, results = [], []
        for w in range(args.worlds):
            wp = randomize(base, rng)
            off = float(rng.uniform(-8, 8))
            sim = core.Sim(wp, track, start_lateral_mm=off,
                           seed=1000 * ci + w, time_limit_s=90.0)
            factory = lambda eo: V6Controller(VALUE, error_offset=eo, **cfg)
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
        tag = " ← v6 기본값" if ci == 0 else ""
        print("[%2d/%d] kp=%.3f kd=%.2f pw=%d tpw=%d tst=%d cb=%d edge=%d"
              " → score %.1f 완주%s 진행%dmm%s"
              % (ci + 1, len(configs), cfg["kp"], cfg["kd"], cfg["power"],
                 cfg["turn_power"], cfg["turn_st"], cfg["corner_black"],
                 cfg["edge_err"], row["score"], row["finish"], row["prog"], tag))

    rows.sort(key=lambda r: r["score"])
    print("\n=== TOP 10 (gear %.1f) ===" % args.gear)
    print("%7s %5s %4s %4s %4s %4s %5s | %7s %6s %7s %7s" %
          ("kp", "kd", "pw", "tpw", "tst", "cb", "edge",
           "score", "완주", "시간", "진행mm"))
    for r in rows[:10]:
        print("%7.3f %5.2f %4d %4d %4d %4d %5d | %7.1f %6s %7.2f %7d" %
              (r["kp"], r["kd"], r["power"], r["turn_power"], r["turn_st"],
               r["corner_black"], r["edge_err"],
               r["score"], r["finish"], r["time_s"], r["prog"]))

    out = os.path.join(core.PKG_DIR, "sweep6_g%.0f_result.json" % (args.gear * 10))
    with open(out, "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    print("저장:", out)


if __name__ == "__main__":
    main()
