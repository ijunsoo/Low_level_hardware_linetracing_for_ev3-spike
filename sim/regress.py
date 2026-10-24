# =====================================================================
#  시뮬 회귀 테스트 — core/목을 고친 뒤에도 기존 결과가 '비트 단위로'
#  같은지 확인한다. (궤적 해시까지 비교)
#
#  사용법 (numpy가 로컬에 있는 파이썬으로 — README 참고):
#    python3 sim/regress.py                    # 골든과 비교
#    python3 sim/regress.py --capture          # 골든 저장/갱신
#    python3 sim/regress.py --only lf3,curve   # 이름 접두어로 일부만
#
#  케이스
#    curve          : 센서 응답곡선 (페어 ±12.5 / 일렬 0·±25)
#    spike3s_smoke  : SPIKE 3센서 목 스모크 (sim/tests)
#    ev3_smoke      : EV3 목 스모크 (sim/tests)
#    lf3_spike·ev3  : 3센서 규칙기반 실기 파일(spike/spike_lf3_rule.py, ev3/ev3_lf3_rule.py) 무수정 실행
#    lf3_parity     : 같은 물리·입출력에서 SPIKE·EV3 파일 궤적이 같은지 (두 파일 로직 동일성)
#  새 케이스는 골든이 없으면 NEW로 표시만 한다.
# =====================================================================
import argparse
import collections
import hashlib
import json
import os
import re
import sys
import time
from multiprocessing import Pool

import numpy as np

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)
import core  # noqa: E402

GOLDEN = os.path.join(SIM_DIR, "regress_golden.json")


def _digest(trace):
    a = np.round(np.asarray(trace, dtype=float), 6)
    return hashlib.sha1(a.tobytes()).hexdigest()[:16]


def _randomize_world(base, rng):
    # 모터 시정수·배터리·센서 노이즈·센서 지연·장착 오차를 무작위로 (골든을 만든 절차 그대로)
    p = dict(base)
    p["motor_tau_s"] = base["motor_tau_s"] * rng.uniform(0.85, 1.15)
    p["battery_scale"] = rng.uniform(0.85, 1.0)
    p["sensor_noise"] = rng.uniform(0.5, 2.0)
    p["sensor_lag_ms"] = rng.uniform(10, 25)
    p["sensor_pos_mm"] = [[f + rng.uniform(-3, 3), l + rng.uniform(-1.5, 1.5)]
                          for f, l in base["sensor_pos_mm"]]
    return p


def _world(base, k):
    # rng(1000+k) → 랜덤 월드 → 시작 오프셋
    rng = np.random.default_rng(1000 + k)
    p = _randomize_world(base, rng)
    return p, float(rng.uniform(-8, 8))


def _load_base(profile):
    if profile and hasattr(core, "load_profile"):
        return core.load_profile(profile)
    return core.load_params()


def job_file(job):
    """실기 파일을 pybricks 목 위에서 실행. LED 이벤트 수도 기록."""
    import pybricks.hubs as hubs
    leds = collections.Counter()
    hubs._Light.on = lambda self, color=None: leds.update([repr(color)])

    src = open(os.path.join(ROOT, job["file"]), encoding="utf-8").read()
    for a, b in job.get("subs", []):
        assert a in src, (job["file"], a)
        src = src.replace(a, b)
    src = re.sub(r"^AUTO_STOP = True", "AUTO_STOP = False", src, flags=re.M)   # 시험용 자동정지 끔

    base = _load_base(job.get("profile"))
    base.update(job.get("override", {}))
    if job.get("value_from_profile"):        # 실기 파일 VALUE(실측) 대신 시뮬 센서 흰/검 → 정규화 일치
        w, b = base["sensor_white"], base["sensor_black"]
        src, n = re.subn(r"^VALUE = \[[^\]]*\]", "VALUE = [%d, %d, %d, %d, %d, %d]"
                         % (w[0], b[0], w[1], b[1], w[2], b[2]), src, count=1, flags=re.M)
        assert n == 1, job["file"]
    if job.get("randomize", True):
        p, off = _world(base, job["k"])
    else:
        p, off = base, 0.0
    track = core.make_track(job["track"], base["line_width_mm"])
    sim = core.Sim(p, track, start_lateral_mm=off,
                   time_limit_s=job.get("limit", 60.0), seed=job["k"])
    core.SIM = sim
    try:
        exec(compile(src, job["file"], "exec"), {"__name__": "__main__"})
    except core.SimDone:
        pass
    out = dict(sim.result or {"reason": "none"})
    out["leds"] = dict(sorted(leds.items()))
    out["trace"] = _digest(sim.trace)
    out["n_trace"] = len(sim.trace)
    if "override" in job:
        # 패리티용: 부호 있는 0(-0.0/+0.0)만 같게 보는 해시. 모터 수가 다르면
        # 출발 전 듀티 기록이 -0.0 vs +0.0으로 갈린다(물리는 같음)
        a = np.round(np.asarray(sim.trace, dtype=float), 6) + 0.0
        out["trace_z"] = hashlib.sha1(a.tobytes()).hexdigest()[:16]
    return job["key"], out


def job_curve(job):
    p = core.load_params()
    tr = core.Track(core.build_path([('S', 600)]), p["line_width_mm"])
    fp = core._footprint_offsets(p["sensor_footprint_mm"] / 2.0)

    def n_at(lat, y):
        b = float(tr.blackness(300.0 + fp[:, 0], (lat - y) + fp[:, 1]).mean())
        return round(100 * (1 - b))

    ys = [float(y) for y in np.arange(-30, 30.5, 1.0)]
    out = {"pair": [[n_at(12.5, y), n_at(-12.5, y)] for y in ys],
           "row3": [[n_at(25, y), n_at(0, y), n_at(-25, y)] for y in ys]}
    return job["key"], out


def jobs():
    js = [dict(key="curve", fn="curve")]
    js.extend(extra_jobs())
    return js


def extra_jobs():
    """SPIKE 3센서·EV3 케이스. 프로파일이 있어야 실행."""
    if not hasattr(core, "load_profile"):
        return []
    js = []
    for k in range(3):
        js.append(dict(key="spike3s_smoke/w%d" % k, fn="file",
                       file="sim/tests/spike3s_smoke.py", profile="spike_3s",
                       track="mrc", k=k, limit=20.0))
        js.append(dict(key="ev3_smoke/w%d" % k, fn="file",
                       file="sim/tests/ev3_smoke.py", profile="ev3_3s",
                       track="mrc", k=k, limit=20.0))
    # 3센서 규칙기반 (2단계) — SPIKE·EV3 각 3월드
    for k in range(3):
        js.append(dict(key="lf3_spike/w%d" % k, fn="file", value_from_profile=True, file="spike/spike_lf3_rule.py",
                       profile="spike_3s", track="mrc", k=k, limit=30.0))
        js.append(dict(key="lf3_ev3/w%d" % k, fn="file", value_from_profile=True, file="ev3/ev3_lf3_rule.py",
                       profile="ev3_3s", track="photo", k=k, limit=30.0))
    # 패리티: EV3 파일을 SPIKE 3센서와 같은 물리·입출력(호출비용·이상적 자이로)으로
    # 돌리면 SPIKE 파일과 궤적까지 같아야 한다 (두 파일 로직 동일성)
    for name, f, over in (("spike", "spike/spike_lf3_rule.py", {"platform": "spike"}),
                          ("ev3", "ev3/ev3_lf3_rule.py", EV3_PARITY)):
        js.append(dict(key="lf3_parity/" + name, fn="file", value_from_profile=True, file=f, profile="spike_3s",
                       override=over, track="photo", k=0, limit=20.0))
    return js


EV3_PARITY = {
    "sensor_ports": ["S1", "S2", "S3"],
    "motors": {"A": ["L", 1], "B": ["L", -1], "C": ["R", -1], "D": ["R", 1]},
    "gyro": {"source": "sensor", "port": "S4", "mount": 1, "lag_ms": 0.0,
             "noise_dps": 0.0, "bias_dps": 0.0, "max_dps": None, "update_hz": 1e9},
    "call_cost_us": {"color_read": 47.152},
    "pybricks_api": "v4", "platform": "ev3",
}


FN = {"file": job_file, "curve": job_curve}


def run_job(job):
    t0 = time.time()
    key, out = FN[job["fn"]](job)
    return key, out, time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture", action="store_true", help="골든 저장/갱신")
    ap.add_argument("--only", default=None, help="쉼표목록 키 접두어")
    ap.add_argument("--procs", type=int, default=max(2, (os.cpu_count() or 4) - 1))
    args = ap.parse_args()

    js = jobs()
    if args.only:
        pre = tuple(args.only.split(","))
        js = [j for j in js if j["key"].startswith(pre)]
    t0 = time.time()
    with Pool(args.procs) as pool:
        rows = pool.map(run_job, js, chunksize=1)
    print("실행 %d건 / %.0fs" % (len(rows), time.time() - t0))

    golden = {}
    if os.path.exists(GOLDEN):
        with open(GOLDEN) as f:
            golden = json.load(f)

    if args.capture:
        for key, out, _ in rows:
            golden[key] = out
        with open(GOLDEN, "w") as f:
            json.dump(golden, f, indent=1, ensure_ascii=False, sort_keys=True)
        print("골든 저장:", GOLDEN, "(%d건 갱신)" % len(rows))
        for key, out, dt in rows:
            if key != "curve":
                print("  %-18s %s  (%.1fs)" % (key, {k: out[k] for k in out
                                                    if k not in ("trace",)}, dt))
        return

    bad = new = 0
    for key, out, dt in rows:
        g = golden.get(key)
        if g is None:
            new += 1
            print("NEW  %-18s %s" % (key, {k: out[k] for k in out if k != "trace"}))
        elif json.dumps(g, sort_keys=True) == json.dumps(out, sort_keys=True):
            print("OK   %-18s (%.1fs)" % (key, dt))
        else:
            bad += 1
            diff = {k: (g.get(k), out.get(k)) for k in set(g) | set(out)
                    if g.get(k) != out.get(k)}
            print("DIFF %-18s %s" % (key, diff if key != "curve" else "응답곡선 다름"))
    print("\n결과: 일치 %d / 다름 %d / 신규 %d" % (len(rows) - bad - new, bad, new))
    par = {key: out for key, out, _ in rows if key.startswith("lf3_parity/")}
    if len(par) == 2:
        a, b = ({k: v for k, v in par[x].items() if k != "trace"}
                for x in ("lf3_parity/spike", "lf3_parity/ev3"))
        same = a == b
        print("SPIKE↔EV3 패리티:", "OK (궤적 동일)" if same else "FAIL %s" % par)
        bad += 0 if same else 1
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
