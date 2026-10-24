# =====================================================================
#  v8(무감속 기어2 + 지그재그 직진관통) 국소 정밀 탐색
#  - sweep7 상위(3.4m 도달) 조합 주변을 변이시켜 코너·물방울 통과를 안정화
#  - 지그재그 구간(오도메트리 3450~4450mm)은 피벗 끄고 직진 관통
#  사용법: python3 sim/sweep8.py --n 120 --worlds 3
# =====================================================================
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core                        # noqa: E402
from controller import V8Controller    # noqa: E402

VALUE = [45, 5, 42, 5, 31, 3, 25, 2]

NODECEL = dict(power=100, spd_k=0.0, min_power=100, brake_frames=0, brake_duty=0,
               turn_power=100, settle_frames=0, settle_power=100)

# sweep7 상위 2개 (3.4m 도달) — 변이의 씨앗
SEEDS_CFG = [
    dict(kp=0.072, kd=8.1, max_steer_delta=35, ema_alpha=544, d_filter=153,
         turn_st=83, corner_black=27, reacquire=48, white_confirm=3,
         mem_frames=85, st_bank=70, edge_err=80),
    dict(kp=0.068, kd=12.0, max_steer_delta=20, ema_alpha=456, d_filter=313,
         turn_st=72, corner_black=30, reacquire=53, white_confirm=3,
         mem_frames=189, st_bank=91, edge_err=131),
]

INT_KEYS = {"max_steer_delta", "ema_alpha", "d_filter", "turn_st",
            "corner_black", "reacquire", "white_confirm", "mem_frames",
            "st_bank", "edge_err"}
BOUNDS = {"kp": (0.005, 0.15), "kd": (1, 20), "max_steer_delta": (10, 60),
          "ema_alpha": (350, 950), "d_filter": (100, 600), "turn_st": (60, 100),
          "corner_black": (10, 45), "reacquire": (30, 70),
          "white_confirm": (2, 25), "mem_frames": (40, 260),
          "st_bank": (50, 100), "edge_err": (30, 160)}


def mutate(cfg, rng, scale=0.25):
    out = {}
    for k, v in cfg.items():
        lo, hi = BOUNDS[k]
        nv = v * (1 + rng.normal(0, scale))
        nv = min(hi, max(lo, nv))
        out[k] = int(round(nv)) if k in INT_KEYS else round(float(nv), 3)
    return out


def randomize(base, rng):
    p = dict(base)
    p["motor_tau_s"] = base["motor_tau_s"] * rng.uniform(0.9, 1.1)
    p["battery_scale"] = rng.uniform(0.9, 1.0)
    p["sensor_noise"] = rng.uniform(0.5, 1.5)
    return p


def score(result, track_len):
    if result["reason"] == "finish":
        return result["time_s"] + 0.3 * result["mean_lat_mm"]
    return 1000.0 - 900.0 * result["progress_mm"] / track_len


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=120)
    ap.add_argument("--worlds", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    base = core.load_params()
    track = core.make_track("mrc", base["line_width_mm"])

    configs = list(SEEDS_CFG)
    while len(configs) < args.n:
        configs.append(mutate(SEEDS_CFG[len(configs) % 2], rng))

    rows = []
    for ci, cfg in enumerate(configs):
        scores, results = [], []
        for w in range(args.worlds):
            wp = randomize(base, rng)
            off = float(rng.uniform(-6, 6))
            sim = core.Sim(wp, track, start_lateral_mm=off,
                           seed=1000 * ci + w, time_limit_s=90.0)
            factory = lambda eo: V8Controller(
                VALUE, error_offset=eo, zones=((3450, 4450),),
                **NODECEL, **cfg)
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
        print("[%3d/%d] kp=%.3f kd=%4.1f wc=%d tst=%d → score %.1f 완주%s 진행%dmm"
              % (ci + 1, len(configs), cfg["kp"], cfg["kd"],
                 cfg["white_confirm"], cfg["turn_st"],
                 row["score"], row["finish"], row["prog"]), flush=True)

    rows.sort(key=lambda r: r["score"])
    print("\n=== TOP 10 ===")
    for r in rows[:10]:
        print(json.dumps(r, ensure_ascii=False))
    with open(os.path.join(core.PKG_DIR, "sweep8_result.json"), "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    main()
