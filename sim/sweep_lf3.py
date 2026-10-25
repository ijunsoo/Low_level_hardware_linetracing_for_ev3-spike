# =====================================================================
#  3센서 배치·상수 스윕 (실기 파일을 그대로 실행)
#
#  공정 비교 원칙
#   - 모든 셀은 '실기 파일 그대로' 실행 (튜닝값은 소스의 상수 줄만 치환)
#   - 셀마다 같은 탐색 예산 (기본값 1 + 국소 15 + 전역 16)
#   - 같은 월드(모터·배터리·노이즈·센서지연·장착오차·시작오프셋)와 같은 트랙
#
#  사용법 (sim/ 기준, numpy 로컬 파이썬):
#    python3 sim/sweep_lf3.py --phase screen          # 셀×32설정×(2월드×2트랙)
#    python3 sim/sweep_lf3.py --phase validate        # 셀별 기본값+상위3 → 12월드×2트랙
#    python3 sim/sweep_lf3.py --phase screen --cells pf130,row3 --n 8   # 일부만
# =====================================================================
import argparse
import contextlib
import io
import json
import math
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

TRACKS = {"mrc": 50.0, "photo": 30.0}          # 트랙: 제한시간(s) — validate는 이 둘
TRACK_LIMITS = dict(TRACKS, rightangle=20.0)     # run_one이 아는 모든 트랙
STUCK_S = 5.0                                   # 이 시간 동안 최대진행이 50mm도 안 늘면 중단

# ------------------------- 파라미터 공간 ------------------------- #
# (이름, 하한, 상한, 종류) — 종류: f=실수, i=정수, lf=로그실수, c=선택지(하한 자리에 목록)
LF3_SPACE = [
    ("KP", 0.08, 0.9, "lf"), ("KD", 0.0, 20.0, "f"), ("KG", 50, 600, "i"),
    ("G_CLAMP", 20, 80, "i"), ("ALPHA", 500, 950, "i"), ("D_FILTER", 150, 600, "i"),
    ("MAX_STEER_DELTA", 20, 100, "i"), ("W_NORM", 75, 92, "i"), ("SIDE_ERR", 60, 160, "i"),
    ("GAP_MS", 1, 20, "i"), ("AHEAD_HOLD_MS", 5, 60, "i"),
    ("LOCK_MS", [0, 0, 150, 300], None, "c"), ("SEARCH_DEG", [0, 70, 110, 150], None, "c"),
    ("ST_BANK", 50, 100, "i"), ("RECOVER_POWER", 30, 100, "i"),
    ("SPD_K10", 0, 12, "i"), ("MIN_POWER", 30, 100, "i"),
    ("PB_MS", [0, 100, 210, 350], None, "c"), ("PB_POWER", 20, 70, "i"),
]

def _pf(pos):
    # 3센서 기하: 포트 순서(C=페어좌, B=페어우, A=센티넬)
    return {"sensor_pos_mm": pos}


CELLS = {
    # ---- 3센서 (spike/spike_lf3_rule.py, 프로파일 spike_3s 위에 기하만 바꿈) ----
    "pf130": dict(kind="lf3", over=_pf([[90, 12.5], [90, -12.5], [130, 0]])),
    "pf110": dict(kind="lf3", over=_pf([[90, 12.5], [90, -12.5], [110, 0]])),
    "pf150": dict(kind="lf3", over=_pf([[90, 12.5], [90, -12.5], [150, 0]])),
    "pf130_fp12": dict(kind="lf3", over=dict(_pf([[90, 12.5], [90, -12.5], [130, 0]]),
                                              sensor_footprint_mm=12.0)),
    # 센서를 높여 발자국을 키우면 대비도 떨어짐 — 비관적 가정(흰-검 폭 60%)
    "pf130_fp12_lowc": dict(kind="lf3", over=dict(_pf([[90, 12.5], [90, -12.5], [130, 0]]),
                                                   sensor_footprint_mm=12.0, contrast=0.6)),
    "pair2": dict(kind="lf3", over=_pf([[90, 12.5], [90, -12.5], [130, 0]]),
                  fixed={"USE_SENTINEL": "False"}),
    # 일렬 3개 (참고 EV3 파일 방식): 좌우 ±25 = 오차, 가운데 = '라인 있음' 신호
    "row3": dict(kind="lf3", over=_pf([[90, 25], [90, -25], [90, 0]])),
    "row3_70": dict(kind="lf3", over=_pf([[70, 25], [70, -25], [70, 0]])),
    "row3_110": dict(kind="lf3", over=_pf([[110, 25], [110, -25], [110, 0]])),
    # 강화학습 잔차 정책 실기 파일 (rl/export_policy.py가 생성) — 일렬 배치
    "rl_row3": dict(kind="lf3rl", over=_pf([[90, 25], [90, -25], [90, 0]])),
}

SPACES = {"lf3": LF3_SPACE, "lf3rl": []}
FILES = {"lf3": "spike/spike_lf3_rule.py", "lf3rl": "spike/spike_lf3_rl.py"}


# ------------------------- 소스 준비 ------------------------- #

def _set_const(src, name, val):
    # 'NAME = 값  # 주석' 줄(들여쓰기 허용)의 값만 바꿈. 정확히 1개여야 함.
    pat = re.compile(r"^(\s*%s\s*=\s*)([^#\n]*?)(\s*(#.*)?)$" % re.escape(name), re.M)
    n = len(pat.findall(src))
    if n != 1:
        raise ValueError("상수 %s: %d개 일치" % (name, n))
    return pat.sub(lambda m: m.group(1) + str(val) + (m.group(3) or ""), src)


POLICY_ANCHOR = "            DS = (ap0 * DSTEER_MAX + 500) // 1000\n"   # 정책 호출 바로 다음 줄


def _value6(p):
    w, b = p["sensor_white"], p["sensor_black"]
    return "[%d, %d, %d, %d, %d, %d]" % (w[0], b[0], w[1], b[1], w[2], b[2])


def build_source(cell_name, cfg, params):
    cell = CELLS[cell_name]
    kind = cell["kind"]
    src = open(os.path.join(ROOT, FILES[kind]), encoding="utf-8").read()
    pus = cfg.get("_POLICY_US", (params.get("call_cost_us") or {}).get("policy", 0))
    cfg = {k: v for k, v in cfg.items() if not k.startswith("_")}   # '_' 키 = 시뮬 조건 (상수 아님)
    if kind in ("lf3", "lf3rl"):
        src = _set_const(src, "VALUE", _value6(params))      # 기하에 맞춰 재측정한 셈
        if re.search(r"^AUTO_STOP\s*=", src, re.M):
            src = _set_const(src, "AUTO_STOP", False)          # 시뮬은 자체 이탈 판정
        for k, v in cell.get("fixed", {}).items():
            src = _set_const(src, k, v)
        for k, v in cfg.items():
            src = _set_const(src, k, v)
        if kind == "lf3rl" and pus:
            # 허브가 정책을 계산하는 동안 센서·모터 갱신이 멈추는 것을 재현 (값: 실기 시작 시 POLICY_US)
            if src.count(POLICY_ANCHOR) != 1:
                raise ValueError("정책 호출 앵커를 못 찾음")
            src = src.replace(POLICY_ANCHOR, "            __import__('core').SIM.wait_ms(%r)\n"
                              % (pus / 1000.0) + POLICY_ANCHOR)
        return src
    raise ValueError("알 수 없는 셀 종류: %s" % kind)

DEFAULTS = {   # 파일 기본값 — 국소 탐색 중심
    "lf3": None,  # 파일에서 읽음
    "lf3rl": {},
}


def lf3_defaults():
    src = open(os.path.join(ROOT, FILES["lf3"]), encoding="utf-8").read()
    d = {}
    for name, *_ in LF3_SPACE:
        m = re.search(r"^%s\s*=\s*([^#\n]+?)\s*(#.*)?$" % name, src, re.M)
        d[name] = m.group(1).strip()
    return d


# ------------------------- 월드 ------------------------- #

# 설정 dict의 '_' 키로 바꿀 수 있는 월드 조건 (없으면 아래 기본 범위 — 예전 결과와 같은 월드)
#   _LAG_MS [lo, hi] 센서 지연 · _NOISE [lo, hi] 센서 노이즈 σ · _ASYNC true 센서별 독립 갱신
#   _HZ 센서 프레임 Hz · _PROFILE 3센서 셀의 프로파일 이름 · _POLICY_US RL 정책 연산 µs · _MU 마찰 한계
WORLD_DEFAULTS = {"_LAG_MS": (10, 25), "_NOISE": (0.5, 2.0)}


def make_world(base, k, cfg=None):
    """센서 수와 무관하게 같은 난수열 → 3센서·4센서 셀이 같은 월드를 공유.
    범위를 바꿔도 난수 소비 순서가 같아 모터·장착오차·출발 오프셋은 그대로 (짝 비교 가능)."""
    cfg = cfg or {}
    if cfg.get("_SPAN") or cfg.get("_FWD"):   # 일렬 3센서 배치 바꾸기: 좌우 센서 측방 ±_SPAN, 전방 _FWD (mm)
        base = dict(base)
        f = float(cfg.get("_FWD") or base["sensor_pos_mm"][0][0])
        sp = float(cfg.get("_SPAN") or abs(base["sensor_pos_mm"][0][1]))
        base["sensor_pos_mm"] = [[f, sp], [f, -sp], [f, 0.0]]
    lag = cfg.get("_LAG_MS", WORLD_DEFAULTS["_LAG_MS"])
    noise = cfg.get("_NOISE", WORLD_DEFAULTS["_NOISE"])
    rng = np.random.default_rng(7000 + k)
    p = dict(base)
    p["motor_tau_s"] = base["motor_tau_s"] * rng.uniform(0.85, 1.15)
    p["battery_scale"] = rng.uniform(0.85, 1.0)
    p["sensor_noise"] = rng.uniform(noise[0], noise[1])
    p["sensor_lag_ms"] = rng.uniform(lag[0], lag[1])
    if cfg.get("_ASYNC"):
        p["sensor_async"] = True
    if cfg.get("_HZ"):
        p["sensor_hz"] = float(cfg["_HZ"])
    if cfg.get("_MU"):                     # 횡가속 한계 μ (마찰 — 넘으면 언더스티어로 미끄러짐)
        p["mu"] = float(cfg["_MU"])
    if cfg.get("_WEFF"):                   # 유효 윤거 배율 (회전 권한 — 실기 추정 불확실성 시험)
        p["w_eff_scale"] = float(cfg["_WEFF"])
    if cfg.get("_CMDLAG"):                 # 구동 지연 ms [lo, hi] — 난수열 뒤에 따로 뽑음 (기존 월드 불변)
        lo, hi = cfg["_CMDLAG"]
        p["cmd_lag_ms"] = float(np.random.default_rng(91000 + k).uniform(lo, hi))
    if cfg.get("_IMULAG"):                 # IMU 요레이트 측정 지연 ms [lo, hi]
        lo, hi = cfg["_IMULAG"]
        p["imu_lag_ms"] = float(np.random.default_rng(93000 + k).uniform(lo, hi))
    if cfg.get("_WEFF_RANGE"):             # 회전 권한 불확실성 범위 [lo, hi]
        lo, hi = cfg["_WEFF_RANGE"]
        p["w_eff_scale"] = float(np.random.default_rng(92000 + k).uniform(lo, hi))
    pert = [(rng.uniform(-3, 3), rng.uniform(-1.5, 1.5)) for _ in range(4)]
    p["sensor_pos_mm"] = [[f + df, l + dl] for (f, l), (df, dl)
                          in zip(base["sensor_pos_mm"], pert)]
    off = float(rng.uniform(-8, 8))
    return p, off


def cell_params(cell_name, profile=None):
    cell = CELLS[cell_name]
    if cell["kind"] in ("lf3", "lf3rl"):
        p = core.load_profile(profile or "spike_3s")
        over = dict(cell.get("over", {}))
        c = over.pop("contrast", None)
        p.update(over)
        if c is not None:
            p["sensor_white"] = [int(round(b + (w - b) * c))
                                 for w, b in zip(p["sensor_white"], p["sensor_black"])]
        return p
    raise ValueError("알 수 없는 셀 종류: %s" % cell["kind"])


class StuckSim(core.Sim):
    """최대진행이 STUCK_S 동안 50mm도 안 늘면 'stuck'으로 끝냄 (탐색 회전·역주행 컷)."""

    def _update_sensors(self):
        before = self.max_progress_mm
        super()._update_sensors()
        if self.max_progress_mm > getattr(self, "_mark", 0.0) + 50.0:
            self._mark = self.max_progress_mm
            self._t_mark = self.t
        elif self.t - getattr(self, "_t_mark", 0.0) > STUCK_S:
            self._finish("stuck")
        del before


_TRACK_CACHE = {}


def run_one(task):
    cell_name, cfg, track_name, k = task
    base = cell_params(cell_name, cfg.get("_PROFILE"))
    p, off = make_world(base, k, cfg)
    if track_name not in _TRACK_CACHE:
        _TRACK_CACHE[track_name] = core.make_track(track_name, base["line_width_mm"])
    track = _TRACK_CACHE[track_name]
    sim = StuckSim(p, track, start_lateral_mm=off, time_limit_s=TRACK_LIMITS[track_name], seed=k)
    core.SIM = sim
    src = build_source(cell_name, cfg, base)
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, cell_name, "exec"), {"__name__": "__main__"})
        except core.SimDone:
            pass
    r = sim.result or {"reason": "none", "time_s": sim.t, "mean_lat_mm": 0.0}
    yellow = sum(1 for _, kk, v in sim.events if kk == "light" and v == "YELLOW")
    return dict(cell=cell_name, cfg=cfg, track=track_name, k=k, reason=r["reason"],
                time_s=r["time_s"], max_mm=round(sim.max_progress_mm, 1),
                len_mm=round(track.length, 1), lat=r["mean_lat_mm"], yellow=yellow)


def score(row):
    if row["reason"] == "finish":
        return row["time_s"] + 0.3 * row["lat"]
    return 1000.0 - 900.0 * row["max_mm"] / row["len_mm"]


# ------------------------- 샘플링 ------------------------- #

def _draw(spec, rng, center=None):
    name, lo, hi, kind = spec
    if kind == "c":
        if center is not None and rng.random() < 0.5:
            return center
        return lo[int(rng.integers(len(lo)))]
    if center is not None:
        c = float(center)
        span = (hi - lo) * 0.25
        v = min(hi, max(lo, c + rng.normal(0.0, span)))
    elif kind == "lf":
        v = math.exp(rng.uniform(math.log(lo), math.log(hi)))
    else:
        v = rng.uniform(lo, hi)
    if kind == "i":
        return int(round(v))
    return round(v, 3)


def sample_configs(cell_name, n, rng):
    kind = CELLS[cell_name]["kind"]
    space = SPACES[kind]
    if not space:
        return [{}]
    dflt = lf3_defaults() if kind == "lf3" else DEFAULTS[kind]
    cfgs = [{}]                                   # 0번 = 파일 기본값 그대로
    n_local = (n - 1) // 2
    for i in range(n - 1):
        local = i < n_local
        cfg = {}
        for spec in space:
            c = dflt[spec[0]] if local else None
            if c is not None and spec[3] != "c":
                c = float(c)
            cfg[spec[0]] = _draw(spec, rng, c)
        cfgs.append(cfg)
    return cfgs


# ------------------------- 실행 ------------------------- #

def run_tasks(tasks, procs, log_path):
    rows = []
    t0 = time.time()
    with Pool(procs) as pool, open(log_path, "w", encoding="utf-8") as log:
        for i, row in enumerate(pool.imap_unordered(run_one, tasks, chunksize=1)):
            rows.append(row)
            if (i + 1) % 20 == 0 or i + 1 == len(tasks):
                msg = "[%d/%d] %.0fs" % (i + 1, len(tasks), time.time() - t0)
                print(msg, flush=True)
                log.write(msg + "\n")
                log.flush()
    return rows


def summarize(rows, key=lambda r: (r["cell"], json.dumps(r["cfg"], sort_keys=True))):
    groups = {}
    for r in rows:
        groups.setdefault(key(r), []).append(r)
    out = []
    for (cell, cfg_s), rs in groups.items():
        fin = [r for r in rs if r["reason"] == "finish"]
        per_track = {}
        for tr in TRACKS:
            sub = [r for r in rs if r["track"] == tr]
            f = [r for r in sub if r["reason"] == "finish"]
            per_track[tr] = dict(fin=len(f), n=len(sub),
                                 time=round(float(np.mean([r["time_s"] for r in f])), 2) if f else None,
                                 prog=round(float(np.mean([r["max_mm"] / r["len_mm"] for r in sub])), 3) if sub else None)
        out.append(dict(cell=cell, cfg=json.loads(cfg_s), n=len(rs), fin=len(fin),
                        score=round(float(np.mean([score(r) for r in rs])), 2),
                        yellow=round(float(np.mean([r["yellow"] for r in rs])), 1),
                        lat=round(float(np.mean([r["lat"] for r in rs])), 1),
                        tracks=per_track))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", choices=["screen", "validate"], default="screen")
    ap.add_argument("--cells", default=",".join(CELLS))
    ap.add_argument("--n", type=int, default=32, help="셀당 설정 수 (screen)")
    ap.add_argument("--worlds", type=int, default=12, help="validate 월드 수")
    ap.add_argument("--top", type=int, default=3, help="validate로 넘길 셀별 상위 설정 수")
    ap.add_argument("--procs", type=int, default=max(2, (os.cpu_count() or 4) - 1))
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--configs", default=None, help="validate: [[셀, 설정dict], ...] JSON 파일을 그대로 검증")
    ap.add_argument("--out", default=None)
    ap.add_argument("--tracks", default=None, help="validate --configs 트랙 목록 (예: photo,rightangle). 기본 mrc,photo")
    args = ap.parse_args()
    cells = [c for c in args.cells.split(",") if c]
    screen_path = os.path.join(SIM_DIR, "sweep_lf3_screen.json")
    val_path = os.path.join(SIM_DIR, "sweep_lf3_validate.json")

    if args.phase == "screen":
        rng = np.random.default_rng(args.seed)
        tasks = []
        for cell in cells:
            for cfg in sample_configs(cell, args.n, rng):
                for tr in TRACKS:
                    for k in (100, 101):          # 스크리닝 월드 (검증 월드 0..11과 분리)
                        tasks.append((cell, cfg, tr, k))
        print("스크리닝: %d셀 %d태스크 / %d프로세스" % (len(cells), len(tasks), args.procs), flush=True)
        rows = run_tasks(tasks, args.procs, os.path.join(SIM_DIR, "sweep_lf3_screen.log"))
        old = []
        if os.path.exists(screen_path):
            old = [r for r in json.load(open(screen_path, encoding="utf-8")) if r["cell"] not in cells]
        json.dump(old + rows, open(screen_path, "w", encoding="utf-8"), ensure_ascii=False)
        summ = summarize(rows)
        print("\n=== 스크리닝 셀별 최고 (점수 낮을수록 좋음) ===")
        for cell in cells:
            sub = sorted([s for s in summ if s["cell"] == cell], key=lambda s: s["score"])
            base = [s for s in sub if s["cfg"] == {}]
            b = sub[0]
            print("%-16s 최고 %6.1f 완주 %d/%d | 기본값 %6.1f 완주 %d/%d" % (
                cell, b["score"], b["fin"], b["n"],
                base[0]["score"] if base else float("nan"),
                base[0]["fin"] if base else 0, base[0]["n"] if base else 0))
        return

    # ---- validate ----
    tasks = []
    if args.configs:
        pairs = json.load(open(args.configs, encoding="utf-8"))
        cells = sorted(set(c for c, _ in pairs), key=list(CELLS).index)
        for cell, cfg in pairs:
            for tr in (args.tracks.split(",") if args.tracks else TRACKS):
                for k in range(args.worlds):
                    tasks.append((cell, cfg, tr, k))
    else:
        screen = json.load(open(screen_path, encoding="utf-8"))
        summ = summarize([r for r in screen if r["cell"] in cells])
        for cell in cells:
            sub = sorted([s for s in summ if s["cell"] == cell], key=lambda s: s["score"])
            picks = [{}] + [s["cfg"] for s in sub if s["cfg"] != {}][:args.top]
            for cfg in picks:
                for tr in TRACKS:
                    for k in range(args.worlds):
                        tasks.append((cell, cfg, tr, k))
    if args.out:
        val_path = args.out
    print("검증: %d셀 %d태스크 / %d프로세스" % (len(cells), len(tasks), args.procs), flush=True)
    rows = run_tasks(tasks, args.procs, os.path.join(SIM_DIR, "sweep_lf3_validate.log"))
    old = []
    if os.path.exists(val_path):
        old = [r for r in json.load(open(val_path, encoding="utf-8")) if r["cell"] not in cells]
    json.dump(old + rows, open(val_path, "w", encoding="utf-8"), ensure_ascii=False)
    summ = summarize(rows)
    print("\n=== 검증 (%d월드 × mrc·photo) ===" % args.worlds)
    for cell in cells:
        sub = sorted([s for s in summ if s["cell"] == cell], key=lambda s: s["score"])
        for s in sub:
            tag = "기본값" if s["cfg"] == {} else "튜닝"
            t = s["tracks"]
            print("%-16s %-4s 점수 %6.1f | mrc %2d/%d %6s 진행%.0f%% | photo %2d/%d %6s 진행%.0f%% | 노랑 %.0f 횡편차 %.1f" % (
                cell, tag, s["score"], t["mrc"]["fin"], t["mrc"]["n"], t["mrc"]["time"],
                100 * t["mrc"]["prog"], t["photo"]["fin"], t["photo"]["n"], t["photo"]["time"],
                100 * t["photo"]["prog"], s["yellow"], s["lat"]))


if __name__ == "__main__":
    main()
