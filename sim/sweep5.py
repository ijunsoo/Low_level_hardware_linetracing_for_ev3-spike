# =====================================================================
#  v5(정규화판) 파라미터 자동 탐색 — 코너/이탈복구 파라미터 포함
#
#  사용법:
#    python3 sim/sweep5.py --n 40 --worlds 2 --gear 1.0
#    python3 sim/sweep5.py --gear 2.0        # 기어업 상정
#
#  기어비에 따라 tau를 물리 스케일(×gear²)로 자동 조정한다.
# =====================================================================
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core                        # noqa: E402
from controller import V5Controller    # noqa: E402

VALUE = [45, 5, 42, 5, 31, 3, 25, 2]
TAU_BASE = 0.115    # 실측 (기어 1배)


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


def run_one(cfg, params, track, seed, offset):
    sim = core.Sim(params, track, start_lateral_mm=offset, seed=seed,
                   time_limit_s=90.0)
    factory = lambda eo: V5Controller(VALUE, error_offset=eo, **cfg)
    return core.run_controller(sim, factory)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40)
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

    # 0번 = v5 기본값 (베이스라인)
    configs = [dict(kp=0.08, kd=1.0, edge_err=80, st_bank=75,
                    recover_power=70, gap_limit=10, power=100)]
    for _ in range(args.n - 1):
        configs.append(dict(
            kp=round(float(np.exp(rng.uniform(np.log(0.03), np.log(0.25)))), 3),
            kd=round(float(rng.uniform(0.3, 4.0)), 2),
            edge_err=int(rng.uniform(30, 150)),
            st_bank=int(rng.uniform(50, 95)),
            recover_power=int(rng.uniform(40, 85)),
            gap_limit=int(rng.uniform(4, 25)),
            power=int(rng.uniform(75, 100)),
        ))

    rows = []
    for ci, cfg in enumerate(configs):
        scores, results = [], []
        for w in range(args.worlds):
            wp = randomize(base, rng)
            off = float(rng.uniform(-8, 8))
            r = run_one(cfg, wp, track, seed=1000 * ci + w, offset=off)
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
        tag = " ← v5 기본값" if ci == 0 else ""
        print("[%2d/%d] kp=%.3f kd=%.2f edge=%d bank=%d rec=%d gap=%d pw=%d"
              " → score %.1f 완주%s 진행%dmm%s"
              % (ci + 1, len(configs), cfg["kp"], cfg["kd"], cfg["edge_err"],
                 cfg["st_bank"], cfg["recover_power"], cfg["gap_limit"],
                 cfg["power"], row["score"], row["finish"], row["prog"], tag))

    rows.sort(key=lambda r: r["score"])
    print("\n=== TOP 10 (gear %.1f) ===" % args.gear)
    print("%7s %5s %5s %5s %4s %4s %4s | %7s %6s %7s %7s" %
          ("kp", "kd", "edge", "bank", "rec", "gap", "pw",
           "score", "완주", "시간", "진행mm"))
    for r in rows[:10]:
        print("%7.3f %5.2f %5d %5d %4d %4d %4d | %7.1f %6s %7.2f %7d" %
              (r["kp"], r["kd"], r["edge_err"], r["st_bank"],
               r["recover_power"], r["gap_limit"], r["power"],
               r["score"], r["finish"], r["time_s"], r["prog"]))

    out = os.path.join(core.PKG_DIR, "sweep5_result.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    print("저장:", out)


if __name__ == "__main__":
    main()
