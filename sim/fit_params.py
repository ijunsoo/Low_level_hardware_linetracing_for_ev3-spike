# =====================================================================
#  MRC_spike_calibration.py 출력 로그 → robot_params.json 피팅
#
#  사용법: python3 sim/fit_params.py calib_log.txt
#
#  피팅 내용:
#    - dc_table: 파워별 정상속도 (deg/s)
#    - motor_tau_s: 가속 시정수 — 1차지연 모델 ang(t)=v(t - tau(1-e^-t/tau))
#    - 데드존: 속도-파워 직선의 x절편
#    - sensor_white/black + 발자국 폭 (라인스캔)
#  + 가속 성능 요약(0→90% 도달시간, 최대가속) 출력
# =====================================================================
import json
import math
import os
import sys

import numpy as np

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.join(PKG_DIR, "robot_params.json")


def parse(path):
    steps = {}   # power -> [(t_ms, angL, angR)]
    scan = []    # (ang, r1..r4)
    hz = {}
    batt = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            if parts[0] == "STEP" and len(parts) == 5:
                p = int(parts[1])
                steps.setdefault(p, []).append(tuple(int(v) for v in parts[2:]))
            elif parts[0] == "SCAN" and len(parts) == 6:
                scan.append(tuple(int(v) for v in parts[1:]))
            elif parts[0] == "HZ":
                hz = {"loop": int(parts[2]), "sensor": int(parts[4])}
            elif parts[0] == "BATT":
                batt.append(int(parts[1]))
    return steps, scan, hz, batt


def steady_speed(samples, drive_ms=1200):
    """구동 구간 후반 40% 기울기로 정상속도(deg/s)만 추정 (전반부 잘린 로그용)."""
    arr = np.asarray([s for s in samples if s[0] <= drive_ms], dtype=float)
    t = arr[:, 0] / 1000.0
    ang = (np.abs(arr[:, 1]) + np.abs(arr[:, 2])) / 2.0
    cut = t.min() + 0.6 * (t.max() - t.min())
    m = t >= cut
    return float(np.polyfit(t[m], ang[m], 1)[0])


def fit_step(samples, drive_ms=1200):
    """1차지연 모델 피팅: tau 그리드 탐색 + 각 tau에서 v 최소자승."""
    arr = np.asarray([s for s in samples if s[0] <= drive_ms], dtype=float)
    t = arr[:, 0] / 1000.0
    ang = (np.abs(arr[:, 1]) + np.abs(arr[:, 2])) / 2.0   # 좌우 평균

    best = (None, None, math.inf)
    for tau in np.arange(0.02, 0.45, 0.005):
        f = t - tau * (1.0 - np.exp(-t / tau))
        denom = float(f @ f)
        if denom <= 0:
            continue
        v = float(ang @ f) / denom
        res = float(((ang - v * f) ** 2).sum())
        if res < best[2]:
            best = (v, float(tau), res)
    v, tau, _ = best

    # 가속 요약: 0→90% 속도 도달시간, 초기 최대가속
    t90 = tau * math.log(10.0)          # 1차지연 해석해
    a0 = v / tau                        # t=0 가속 (deg/s^2)
    return v, tau, t90, a0


def fit_scan(scan, wheel_d):
    if len(scan) < 30:
        print("=== 센서 피팅 ===")
        print("  SCAN 샘플이 %d개뿐 → 스킵 (로봇이 안 움직였거나 로그 누락)" % len(scan))
        return None
    arr = np.asarray(scan, dtype=float)
    mm = arr[:, 0] / 360.0 * math.pi * wheel_d
    whites, blacks, widths = [], [], []
    for i in range(4):
        v = arr[:, i + 1]
        w = float(np.percentile(v, 95))
        # 검정 구간은 스캔의 극히 일부라 퍼센타일이 과대추정 → 최저 5개 평균 사용
        b = float(np.mean(np.sort(v)[:5]))
        whites.append(round(w))
        blacks.append(round(b))
        # 발자국 폭 ≈ 진입 에지에서 10%→90% 어두워지는 데 걸린 거리
        depth = w - v
        lo, hi = 0.1 * (w - b), 0.9 * (w - b)
        i_lo = np.argmax(depth > lo)
        i_hi = np.argmax(depth > hi)
        if i_hi > i_lo > 0:
            widths.append(abs(mm[i_hi] - mm[i_lo]))
    fp = float(np.median(widths)) if widths else None
    return whites, blacks, fp


def main():
    if len(sys.argv) < 2:
        print("사용법: python3 sim/fit_params.py calib_log.txt")
        sys.exit(1)
    steps, scan, hz, batt = parse(sys.argv[1])
    if not steps:
        print("STEP 데이터가 없습니다. 로그 파일을 확인하세요.")
        sys.exit(1)

    params = {}
    if os.path.exists(PARAMS_PATH):
        with open(PARAMS_PATH) as f:
            params = json.load(f)
    wheel_d = params.get("wheel_diameter_mm", 56.0)

    print("=== 모터 가속(스텝응답) 피팅 ===")
    table, taus = [], []
    for p in sorted(steps):
        first_t = min(s[0] for s in steps[p])
        if first_t > 1100:
            print("  dc(%3d): 구동구간(≤1.2s) 데이터 없음 → 스킵" % p)
            continue
        if first_t > 50:
            # 로그 전반부가 잘림(가속 구간 없음) → 정상속도만 사용, tau 제외
            v = steady_speed(steps[p])
            table.append([p, round(v, 1)])
            print("  dc(%3d): 정상속도 %6.1f deg/s | (t=%dms부터라 가속 피팅 생략)"
                  % (p, v, first_t))
            continue
        v, tau, t90, a0 = fit_step(steps[p])
        table.append([p, round(v, 1)])
        if p >= 40:
            taus.append(tau)   # 저파워는 마찰 영향이 커서 tau 평균에서 제외
        print("  dc(%3d): 정상속도 %6.1f deg/s | tau %4.0f ms | "
              "0→90%% %4.0f ms | 초기가속 %5.0f deg/s²"
              % (p, v, tau * 1000, t90 * 1000, a0))

    # 데드존: 속도-파워 직선 x절편 (파워 2개 이상일 때만)
    tp = np.asarray(table, dtype=float)
    if len(tp) >= 2:
        slope, icpt = np.polyfit(tp[:, 0], tp[:, 1], 1)
        deadzone = max(0.0, min(18.0, -icpt / slope))
        print("  데드존 추정: dc %.1f 이하에서 정지" % deadzone)
    else:
        deadzone = params.get("dc_table", [[0, 0], [8, 0]])[1][0]
        print("  파워가 1개뿐이라 데드존은 기존값(%.1f) 유지" % deadzone)

    dc_table = [[0, 0], [round(deadzone, 1), 0]] + [[int(p), v] for p, v in table]
    params["dc_table"] = dc_table
    params["motor_tau_s"] = round(float(np.mean(taus)), 4) if taus else 0.15

    if scan:
        fitted = fit_scan(scan, wheel_d)
        if fitted:
            whites, blacks, fp = fitted
            if min(whites[i] - blacks[i] for i in range(4)) < 10:
                print("=== 센서 피팅 ===")
                print("  흰/검 대비가 너무 작음 → 스킵 (라인을 안 지나간 듯)")
            else:
                params["sensor_white"] = whites
                params["sensor_black"] = blacks
                if fp:
                    params["sensor_footprint_mm"] = round(fp, 1)
                print("=== 센서 피팅 ===")
                print("  white:", whites, "black:", blacks,
                      "footprint: %.1f mm" % fp if fp else "(발자국 피팅 실패)")

    if hz:
        params["loop_hz"] = float(hz["loop"])
        params["sensor_hz"] = float(hz["sensor"])
        print("=== 속도 벤치마크 ===  loop %d Hz / sensor %d Hz"
              % (hz["loop"], hz["sensor"]))
    if len(batt) >= 2:
        print("배터리: %d → %d mV (측정 중 강하 %d mV)"
              % (batt[0], batt[-1], batt[0] - batt[-1]))

    with open(PARAMS_PATH, "w") as f:
        json.dump(params, f, indent=2, ensure_ascii=False)
    print("저장:", PARAMS_PATH)
    print("※ 기하값(wheel_diameter, axle_track, sensor_forward/spacing)은 자로 재서 직접 수정")


if __name__ == "__main__":
    main()
