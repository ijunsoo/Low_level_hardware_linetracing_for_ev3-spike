# =====================================================================
#  센서 배치(지오메트리) A/B 스윕 — "안쪽 2개를 바퀴축으로, 바깥 2개를
#  전방으로 쭉 빼서 이탈감지 전용" 제안을 4배 가속(gear 4)에서 검증.
#
#  공정 비교를 위해 모든 배치를 '같은' GeoController로 돌린다:
#   - PID 오차원: row4 = (s1+s2)-(s3+s4) / inner2 = 2*(s2-s3)
#   - 코너 기억: 바깥(전방) 센서 한쪽만 진검정 → 방향 기억 (+선택 사전브레이크)
#   - 이탈: 제어센서 흰색 지속 → 기억 방향으로 뱅킹, 재포착 시 PID 리셋
#   - 지그재그 오도메트리 관통(V8), 조향비례 감속(spd_k), 자이로 댐핑(kg)
#
#  사용법:
#    python3 sim/sweep_geo.py --mode battle --gear 4 --n 40 --worlds 3
#    python3 sim/sweep_geo.py --mode curve  --n 20 --worlds 2
# =====================================================================
import argparse
import json
import math
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core                                # noqa: E402

VALUE = [30, 2, 30, 2, 30, 1, 25, 2]       # 실측 흰/검 (robot_params와 동일)

# 배치 정의: sensor_pos_mm = [전방mm, 측방mm(+좌)] × 4 (좌→우)
GEOMETRIES = {
    # 현행: 4개 일렬, 바퀴축 앞 70mm
    "row70":  dict(pos=[[70, 37.5], [70, 12.5], [70, -12.5], [70, -37.5]],
                   error_mode="row4"),
    # 일렬 그대로 바퀴쪽으로 당김 (제안의 '제어센서 후퇴' 성분만)
    "row40":  dict(pos=[[40, 37.5], [40, 12.5], [40, -12.5], [40, -37.5]],
                   error_mode="row4"),
    # 일렬 그대로 앞으로 밀기 (반대가설: 룩어헤드 증가)
    "row110": dict(pos=[[110, 37.5], [110, 12.5], [110, -12.5], [110, -37.5]],
                   error_mode="row4"),
    "row150": dict(pos=[[150, 37.5], [150, 12.5], [150, -12.5], [150, -37.5]],
                   error_mode="row4"),
    # ★제안: 안쪽 2개 바퀴축 근처(25mm) + 바깥 2개 전방 돌출(140mm, 이탈감지용)
    "split":  dict(pos=[[140, 37.5], [25, 12.5], [25, -12.5], [140, -37.5]],
                   error_mode="inner2"),
    # 제안 변형: 안쪽 후퇴 + 전방센서를 더 멀리
    "split190": dict(pos=[[190, 37.5], [25, 12.5], [25, -12.5], [190, -37.5]],
                     error_mode="inner2"),
    # ★대조군: 제어센서는 현행 70mm 유지, 감지센서만 전방 돌출
    #   (제안에서 '전방 이탈감지' 성분만 취하고 '제어센서 후퇴'는 버림)
    "hybrid":  dict(pos=[[170, 37.5], [70, 12.5], [70, -12.5], [170, -37.5]],
                    error_mode="inner2"),
    # 하이브리드 변형: 제어 페어를 일렬스캔 최적점(90mm)으로 전진
    "hybrid90": dict(pos=[[170, 37.5], [90, 12.5], [90, -12.5], [170, -37.5]],
                     error_mode="inner2"),
    # 규정(25cm 상자) 대응: 감지센서 돌출을 줄인 변형 — 몇 mm까지 버티나
    "hyb150": dict(pos=[[150, 37.5], [70, 12.5], [70, -12.5], [150, -37.5]],
                   error_mode="inner2"),
    "hyb130": dict(pos=[[130, 37.5], [70, 12.5], [70, -12.5], [130, -37.5]],
                   error_mode="inner2"),
    "hyb110": dict(pos=[[110, 37.5], [70, 12.5], [70, -12.5], [110, -37.5]],
                   error_mode="inner2"),
    # 실빌드 한계(바퀴반경+프레임 → 전방 ~100mm) 시나리오
    "hyb100": dict(pos=[[100, 37.5], [70, 12.5], [70, -12.5], [100, -37.5]],
                   error_mode="inner2"),
    "row90":  dict(pos=[[90, 37.5], [90, 12.5], [90, -12.5], [90, -37.5]],
                   error_mode="row4"),
    # ★진짜 현행 배치(사용자 실측 2026-07-09): 4개 일렬, 바퀴축 앞 170mm
    "row170": dict(pos=[[170, 37.5], [170, 12.5], [170, -12.5], [170, -37.5]],
                   error_mode="row4"),
}


class GeoController:
    """지오메트리 실험용 통합 컨트롤러 (V5~V8 로직 정리판)."""

    def __init__(self, value, error_offset=0, error_mode="row4", power=100,
                 kp=0.3, ki=0.0, kd=5.0, kg=0.0,
                 ema_alpha=700, d_filter=300, max_steer_delta=30,
                 w_norm=82, black_mid=50, corner_black=25, mem_frames=300,
                 gap_limit=10, st_bank=90, recover_power=80,
                 spd_k=0.0, min_power=100,
                 pre_brake_frames=0, pre_brake_power=50,
                 zones=(), zone_cap=15):
        S = 100000
        self.S = S
        self.KP_S = round(kp * S)
        self.KI_S = round(ki * S)
        self.KD_S = round(kd * S)
        self.KG_S = round(kg * 1000)
        self.E_S = 1000
        self.ALPHA = int(ema_alpha)
        self.A_COMP = self.E_S - int(ema_alpha)
        self.D_FILTER = int(d_filter)
        self.D_COMP = self.E_S - int(d_filter)

        self.WH = [value[0], value[2], value[4], value[6]]
        self.BK = [value[1], value[3], value[5], value[7]]
        self.RG = [self.WH[i] - self.BK[i] for i in range(4)]

        self.error_mode = error_mode
        self.error_offset = error_offset if error_mode == "row4" else 0
        self.POWER = power
        self.W_NORM = w_norm
        self.BLACK_MID = black_mid
        self.CORNER_BLACK = corner_black
        self.MEM_FRAMES = int(mem_frames)
        self.GAP_LIMIT = int(gap_limit)
        self.ST_BANK = st_bank
        self.RECOVER_POWER = recover_power
        self.MAX_STEER_DELTA = int(max_steer_delta)
        self.SPD_K = spd_k
        self.MIN_POWER = min_power
        self.PB_FRAMES = int(pre_brake_frames)
        self.PB_POWER = pre_brake_power
        self.ZONES = zones
        self.ZONE_CAP = zone_cap

        self.integral = 0
        self.last_error = 0
        self.d_filt = 0
        self.direction = 0
        self.mem_dir = None
        self.mem_age = 0
        self.gap = 0
        self.was_off = False
        self.prev_steering = 0
        self.last_good_steer = 0
        self.pb_cnt = 0
        self.gyro_w = 0.0
        self.dist_mm = 0.0
        self.raw_prev = None
        self.ps = None
        self._out = (0, power)

    # run_controller 훅들
    def norm4(self, r):
        return [(min(max(r[i], self.BK[i]), self.WH[i]) - self.BK[i]) * 100
                // self.RG[i] for i in range(4)]

    def prime(self, r1, r2, r3, r4):
        self.raw_prev = (r1, r2, r3, r4)
        self.ps = self.norm4((r1, r2, r3, r4))

    def on_gyro(self, w_deg_s):
        self.gyro_w = w_deg_s

    def on_distance(self, mm):
        self.dist_mm = mm

    def _error(self, s):
        if self.error_mode == "row4":
            return (s[0] + s[1]) - (s[2] + s[3]) - self.error_offset
        return 2 * (s[1] - s[2])            # inner2: 안쪽 페어만

    def _lost(self, n):
        if self.error_mode == "row4":
            return all(v > self.W_NORM for v in n)
        return n[1] > self.W_NORM and n[2] > self.W_NORM

    def step(self, r1, r2, r3, r4):
        raw = (r1, r2, r3, r4)
        if self.raw_prev is None:
            self.prime(r1, r2, r3, r4)
        if raw == self.raw_prev:
            return self._out                # 프레임 동기
        self.raw_prev = raw
        n = self.norm4(raw)

        # ── 지그재그 오도메트리 관통 (V8) ── #
        if any(a <= self.dist_mm <= b for a, b in self.ZONES):
            self.ps = n[:]
            self.mem_dir = None
            self.gap = 0
            self.was_off = False
            self.pb_cnt = 0
            if self._lost(n):
                st = 0
            else:
                st = max(-self.ZONE_CAP, min(self.ZONE_CAP,
                         (self.KP_S * self._error(n)) // self.S))
            self.prev_steering = st
            self.last_error = 0
            self.d_filt = 0
            self._out = (st, self.POWER)
            return self._out

        s = [0] * 4
        for i in range(4):
            s[i] = (self.ALPHA * n[i] + self.A_COMP * self.ps[i]) // self.E_S
        self.ps = s

        # ── 코너 방향 기억: 바깥(전방) 센서 한쪽만 진검정 ── #
        left_deep = n[0] < self.CORNER_BLACK and n[3] > 60
        right_deep = n[3] < self.CORNER_BLACK and n[0] > 60
        if left_deep or right_deep:
            fresh = self.mem_dir is None
            self.mem_dir = 0 if left_deep else 1
            self.mem_age = 0
            if fresh and self.PB_FRAMES > 0:
                self.pb_cnt = self.PB_FRAMES     # ★전방센서 접촉 → 사전 감속
        elif self.mem_dir is not None:
            self.mem_age += 1
            if self.mem_age > self.MEM_FRAMES:
                self.mem_dir = None

        lost = self._lost(n)
        error = self._error(s)

        if not lost:
            if self.was_off:                 # 재포착 → PID 리셋
                self.integral = 0
                self.d_filt = 0
                self.last_error = error
                self.prev_steering = 0
            self.was_off = False
            self.gap = 0

            if n[0] < self.BLACK_MID and n[3] > 60:
                self.direction = 0
            elif n[3] < self.BLACK_MID and n[0] > 60:
                self.direction = 1

            self.integral += error
            diff = error - self.last_error
            self.d_filt = (self.D_FILTER * diff + self.D_COMP * self.d_filt) // self.E_S
            steering = (self.KP_S * error + self.KI_S * self.integral
                        + self.KD_S * self.d_filt) // self.S
            self.last_error = error

            # 교차점: 제어센서 동시 진검정 → 직진 유지
            if self.error_mode == "row4":
                if sum(s) < 4 * self.BLACK_MID - 80:
                    steering = 0
            elif s[1] < self.BLACK_MID and s[2] < self.BLACK_MID:
                steering = 0

            steering += (self.KG_S * int(self.gyro_w)) // 1000   # 자이로 댐핑
            power = self.POWER
            self.last_good_steer = steering
        else:
            self.gap += 1
            if self.gap <= self.GAP_LIMIT:
                steering = self.last_good_steer
                power = self.POWER
            else:                            # 뱅킹: 기억(신선) 우선, 없으면 최근 방향
                self.was_off = True
                d = self.mem_dir if self.mem_dir is not None else self.direction
                steering = -self.ST_BANK if d == 0 else self.ST_BANK
                power = self.RECOVER_POWER

        if self.pb_cnt > 0:                  # 사전브레이크 (전방센서 경보)
            self.pb_cnt -= 1
            power = min(power, self.PB_POWER)

        # 조향비례 감속
        sched = self.POWER - self.SPD_K * abs(steering)
        power = min(power, max(self.MIN_POWER, sched))

        delta = steering - self.prev_steering
        if delta > self.MAX_STEER_DELTA:
            steering = self.prev_steering + self.MAX_STEER_DELTA
        elif delta < -self.MAX_STEER_DELTA:
            steering = self.prev_steering - self.MAX_STEER_DELTA
        self.prev_steering = steering

        self._out = (steering, power)
        return self._out


# ------------------------- 스윕 하네스 ------------------------- #

def sample_cfg(rng, use_gyro):
    return dict(
        kp=round(float(np.exp(rng.uniform(np.log(0.03), np.log(1.5)))), 3),
        kd=round(float(rng.uniform(1.0, 40.0)), 1),   # 지연 보상: 위상앞섬 상한 확대
        kg=(round(float(rng.uniform(0.05, 0.6)), 3) if use_gyro else 0.0),
        ema_alpha=int(rng.uniform(400, 950)),
        d_filter=int(rng.uniform(150, 600)),
        max_steer_delta=int(rng.uniform(15, 80)),
        gap_limit=int(rng.uniform(3, 40)),
        mem_frames=int(rng.uniform(100, 700)),
        corner_black=int(rng.uniform(15, 40)),
        st_bank=int(rng.uniform(60, 100)),
        recover_power=int(rng.uniform(40, 100)),
        spd_k=round(float(rng.uniform(0.0, 1.2)), 2),
        min_power=int(rng.uniform(30, 100)),
        pre_brake_frames=int(rng.choice([0, 0, 150, 300, 600])),
        pre_brake_power=int(rng.uniform(20, 70)),
    )


def randomize_world(base, rng):
    p = dict(base)
    p["motor_tau_s"] = base["motor_tau_s"] * rng.uniform(0.85, 1.15)
    p["battery_scale"] = rng.uniform(0.85, 1.0)
    p["sensor_noise"] = rng.uniform(0.5, 2.0)
    p["sensor_lag_ms"] = rng.uniform(10, 25)   # EV3 UART 전송지연 (실기 주범)
    # 장착 오차: 센서별 측방 ±1.5mm, 전방 ±3mm
    p["sensor_pos_mm"] = [[f + rng.uniform(-3, 3), l + rng.uniform(-1.5, 1.5)]
                          for f, l in base["sensor_pos_mm"]]
    return p


def score(result, track_len):
    if result["reason"] == "finish":
        return result["time_s"] + 0.3 * result["mean_lat_mm"]
    return 1000.0 - 900.0 * result["progress_mm"] / track_len


_G = {}


def _init_worker(track_name):
    _G["track"] = core.make_track(track_name, core.load_params()["line_width_mm"])
    # 지그재그 관통 구간(오도메트리 트릭)은 끈다 (2026-10-17).
    #  - 예전 값 3450~4450mm는 1.6배 확대 전 트랙 기준이라 mrc의 실제 지그재그
    #    (≈5759mm~)와 안 맞았고, run_controller가 거리에 기어비를 한 번 더 곱해
    #    기어2·4에선 첫 직선(1724~2224 / 862~1112mm)에서 켜지고 있었다.
    #    → 과거 sweep_geo 결과는 사실상 '구간 트릭 없이' 얻은 것.
    #  - 대회 트랙은 당일 공개라 트랙 특화 트릭은 설계에서 제외.
    _G["zones"] = ()


def run_one(task):
    """task = (geo_key, pos, error_mode, gear, cfg, worlds, seed0)"""
    geo_key, pos, error_mode, gear, cfg, worlds, seed0 = task
    base = core.load_params()
    base["gear_ratio"] = gear
    base["sensor_pos_mm"] = pos
    rng = np.random.default_rng(seed0)
    track = _G["track"]
    results = []
    for w in range(worlds):
        wp = randomize_world(base, rng)
        off = float(rng.uniform(-8, 8))
        sim = core.Sim(wp, track, start_lateral_mm=off,
                       seed=seed0 + w, time_limit_s=60.0)
        factory = lambda eo: GeoController(VALUE, error_offset=eo,
                                           error_mode=error_mode,
                                           zones=_G.get("zones", ()),
                                           **cfg)
        results.append(core.run_controller(sim, factory))
    scores = [score(r, track.length) for r in results]
    fin = sum(1 for r in results if r["reason"] == "finish")
    times = [r["time_s"] for r in results if r["reason"] == "finish"]
    row = dict(cfg)
    row.update(geo=geo_key, gear=gear,
               score=round(float(np.mean(scores)), 2),
               fin=fin, n=worlds,
               time_s=round(float(np.mean(times)), 2) if times else None,
               prog=int(np.mean([r["progress_mm"] for r in results])),
               lat=round(float(np.mean([r["mean_lat_mm"] for r in results])), 1))
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="battle", choices=["battle", "curve"])
    ap.add_argument("--gear", type=float, default=4.0)
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--worlds", type=int, default=3)
    ap.add_argument("--track", default="mrc")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--procs", type=int, default=max(2, mp.cpu_count() - 2))
    ap.add_argument("--geo", default=None,
                    help="쉼표목록으로 배치 제한 (예: hybrid,hybrid+gyro)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    tasks = []
    if args.mode == "battle":
        cells = [(k, False) for k in GEOMETRIES] + \
                [("row70", True), ("split", True), ("hybrid", True),
                 ("hybrid90", True), ("hyb150", True), ("hyb130", True),
                 ("hyb110", True), ("hyb100", True), ("row90", True),
                 ("row170", True), ("row40", True), ("row110", True),
                 ("row150", True)]  # +자이로 변형
        if args.geo:
            want = set(args.geo.split(","))
            cells = [(g, gy) for g, gy in cells
                     if (g + ("+gyro" if gy else "")) in want]
        for ci, (geo, gyro) in enumerate(cells):
            g = GEOMETRIES[geo]
            key = geo + ("+gyro" if gyro else "")
            for i in range(args.n):
                cfg = sample_cfg(rng, use_gyro=gyro)
                tasks.append((key, g["pos"], g["error_mode"], args.gear,
                              cfg, args.worlds, 100000 * ci + 100 * i))
        out = args.out or os.path.join(core.PKG_DIR,
                                       "sweep_geo_g%g_result.json" % args.gear)
    else:  # curve: 일렬 배치의 전방거리 스캔 × 기어
        fwds = [25, 40, 55, 70, 90, 110, 130, 155]
        gears = [2.0, 4.0]
        ci = 0
        for gear in gears:
            for f in fwds:
                pos = [[f, 37.5], [f, 12.5], [f, -12.5], [f, -37.5]]
                for i in range(args.n):
                    cfg = sample_cfg(rng, use_gyro=False)
                    tasks.append(("f%d" % f, pos, "row4", gear,
                                  cfg, args.worlds, 100000 * ci + 100 * i))
                ci += 1
        out = args.out or os.path.join(core.PKG_DIR, "sweep_geo_curve_result.json")

    print("총 %d 태스크 / %d 프로세스" % (len(tasks), args.procs), flush=True)
    rows = []
    with mp.Pool(args.procs, initializer=_init_worker,
                 initargs=(args.track,)) as pool:
        for i, row in enumerate(pool.imap_unordered(run_one, tasks, chunksize=1)):
            rows.append(row)
            print("[%4d/%d] %-11s g%g kp=%.3f kd=%4.1f kg=%.2f spdk=%.2f pb=%d"
                  " → %s 완주%d/%d 진행%5dmm t=%s"
                  % (i + 1, len(tasks), row["geo"], row["gear"], row["kp"],
                     row["kd"], row["kg"], row["spd_k"], row["pre_brake_frames"],
                     ("%.1f" % row["score"]).rjust(6), row["fin"], row["n"],
                     row["prog"], row["time_s"]), flush=True)

    with open(out, "w") as f:
        json.dump(rows, f, indent=1, ensure_ascii=False)
    print("저장:", out, flush=True)

    # 셀별 요약
    keys = sorted(set((r["geo"], r["gear"]) for r in rows))
    print("\n=== 셀 요약 (배치 × 기어) ===")
    for k in keys:
        sub = sorted([r for r in rows if (r["geo"], r["gear"]) == k],
                     key=lambda r: r["score"])
        best = sub[0]
        fins = sum(r["fin"] for r in sub)
        tot = sum(r["n"] for r in sub)
        print("%-12s g%-4g best score %7.1f  완주 %d/%d (전체 %d/%d)  "
              "best: kp=%.3f kd=%.1f kg=%.2f spdk=%.2f pb=%d t=%s prog=%d"
              % (k[0], k[1], best["score"], best["fin"], best["n"], fins, tot,
                 best["kp"], best["kd"], best["kg"], best["spd_k"],
                 best["pre_brake_frames"], best["time_s"], best["prog"]))


if __name__ == "__main__":
    main()
