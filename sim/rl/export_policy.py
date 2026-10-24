# =====================================================================
#  학습한 잔차 정책(policy.pt) → 실기 MicroPython 파일 생성 (SPIKE·EV3)
#
#  규칙기반 실기 파일(spike/spike_lf3_rule.py, ev3/ev3_lf3_rule.py)을 그대로 읽어
#  정해진 자리에 정책 코드만 끼워 넣는다 → 규칙 로직은 한 곳에만 있다.
#   - 신경망: 반복문 없는 직선 코드(가중치 리터럴)로 펼쳐 MicroPython에서 빠르게
#   - 관측 22개: TorchEnv.obs()와 같은 식 (센서 정규화값 최근 4결정, 규칙 오차·조향,
#     요레이트, 좌우 바퀴속도, 직전 행동, 센티넬·이탈 경과, 마지막 방향)
#   - 결정 주기 DECISION_MS(5ms)마다 행동 → Δ조향(규칙 조향에 더함), Δ파워(규칙 파워에 더함)
#
#    ~/.venvs/linefollowing/bin/python -m rl.export_policy rl/runs/<이름>/policy.pt
#  → spike/spike_lf3_rl.py, ev3/ev3_lf3_rl.py
# =====================================================================
import argparse
import math
import os
import re

import torch

from . import _SIM_DIR
import core

ROOT = os.path.dirname(_SIM_DIR)

PLATFORM = {
    "spike": dict(src="spike/spike_lf3_rule.py", out="spike/spike_lf3_rl.py", profile="spike_3s",
                  yaw="hub.imu.angular_velocity(YAW_AXIS) * YAW_SIGN", vl="lm.speed()", vr="rm.speed()"),
    "ev3": dict(src="ev3/ev3_lf3_rule.py", out="ev3/ev3_lf3_rl.py", profile="ev3_3s",
                yaw="(gyro.speed() - GBIAS)", vl="LM[0].speed()", vr="RM[0].speed()"),
}


def _f(x):
    return "%.6g" % float(x)


def quantize(sd):
    """정수 고정소수점(×1000) 파라미터: 정규화 평균 M·역표준편차 INV, 층별 W·B."""
    mean = sd["onorm.mean"].cpu().numpy()
    std = (sd["onorm.var"].cpu().numpy() + 1e-8) ** 0.5
    q = dict(M=[int(round(m * 1000)) for m in mean], INV=[int(round(1000.0 / s)) for s in std], layers=[])
    for i in (0, 2, 4):
        W = sd["actor.%d.weight" % i].cpu().numpy()
        b = sd["actor.%d.bias" % i].cpu().numpy()
        q["layers"].append(([[int(round(w * 1000)) for w in row] for row in W], [int(round(x * 1000)) for x in b]))
    return q


def policy_int(q, X):
    """생성 코드와 똑같은 정수 연산 (검증용). X: 관측 ×1000 정수 22개 → (A0, A1) ×1000."""
    z = []
    for x, m, inv in zip(X, q["M"], q["INV"]):
        v = (x - m) * inv // 1000
        z.append(-8000 if v < -8000 else (8000 if v > 8000 else v))
    h = z
    for L, (W, B) in enumerate(q["layers"]):
        last = L == len(q["layers"]) - 1
        out = []
        for row, b in zip(W, B):
            v = b + sum(w * x for w, x in zip(row, h)) // 1000
            out.append(v if last or v > 0 else 0)
        h = out
    return [-1000 if a < -1000 else (1000 if a > 1000 else a) for a in h]


SMALL_INT_MAX = (1 << 30) - 1     # MicroPython 32비트 small int 한계 (넘으면 힙 long int 또는 OverflowError)


def check_bounds(q):
    """생성 코드의 정수 중간값이 가질 수 있는 최악 크기 (입력 z는 ±8000으로 잘림).
    small int 한계의 절반을 넘으면 실패시킨다 → 허브에서 오버플로·힙 할당이 생길 수 없음을 보장."""
    worst = max(8000000 + inv for inv in q["INV"])      # 정규화 곱: 범위를 먼저 봐서 이 이하
    lo = [-8000] * len(q["M"])
    hi = [8000] * len(q["M"])
    nl = len(q["layers"])
    for L, (W, B) in enumerate(q["layers"]):
        nlo, nhi = [], []
        for row, b in zip(W, B):
            worst = max(worst, sum(abs(w) * max(-a, c, a, -c) for w, a, c in zip(row, lo, hi)))
            up = b + sum(w * (c if w > 0 else a) for w, a, c in zip(row, lo, hi)) // 1000
            dn = b + sum(w * (a if w > 0 else c) for w, a, c in zip(row, lo, hi)) // 1000
            if L < nl - 1:
                up, dn = max(up, 0), max(dn, 0)
            nlo.append(dn)
            nhi.append(up)
        lo, hi = nlo, nhi
    assert worst < SMALL_INT_MAX // 2, "정수 중간값 최악 %d — small int 여유 부족" % worst
    return worst


def gen_policy_code(sd, meta):
    """정규화 + MLP(ReLU)를 정수(×1000) 직선 코드로. 입력 x0..x21 → (a0, a1) ∈ [-1000, 1000].
    float·리스트가 없어 Pybricks(SPIKE·EV3 둘 다)에서 쓰레기(GC)를 만들지 않는다."""
    assert meta.get("act", "relu") == "relu", "정수 내보내기는 ReLU 액터만 지원"
    q = quantize(sd)
    check_bounds(q)
    n_in = len(q["M"])
    lines = ["def policy(%s):" % ", ".join("x%d" % i for i in range(n_in)),
             "    # 관측 정규화 z = (x - 평균) * 역표준편차 // 1000 (×1000 정수), ±8000으로 자름.",
             "    # 범위를 곱하기 전에 봐서 곱이 ±8,000,000을 넘지 않음 → small int 안"]
    for i in range(n_in):
        m, inv = q["M"][i], q["INV"][i]
        if inv == 0:
            lines.append("    z%d = 0" % i)
            continue
        lim = 8000000 // inv            # |x-m| ≤ lim ⇔ 곱해도 ±8000 안 (자르는 식과 결과 동일)
        lines.append("    z%d = x%d - %d" % (i, i, m))
        lines.append("    if z%d > %d:\n        z%d = 8000\n    elif z%d < -%d:\n        z%d = -8000\n"
                     "    else:\n        z%d = z%d * %d // 1000" % (i, lim, i, i, lim, i, i, i, inv))
    prev = ["z%d" % i for i in range(n_in)]
    nl = len(q["layers"])
    for L, (W, B) in enumerate(q["layers"]):
        last = L == nl - 1
        names = []
        for j, (row, b) in enumerate(zip(W, B)):
            terms = " + ".join("%d * %s" % (w, prev[i]) for i, w in enumerate(row) if w != 0)
            nm = ("a%d" % j) if last else ("h%d_%d" % (L, j))
            lines.append("    %s = %d + (%s) // 1000" % (nm, b, terms or "0"))
            if not last:
                lines.append("    if %s < 0:\n        %s = 0" % (nm, nm))
            names.append(nm)
        prev = names
    lines.append("    if a0 < -1000:\n        a0 = -1000\n    elif a0 > 1000:\n        a0 = 1000")
    lines.append("    if a1 < -1000:\n        a1 = -1000\n    elif a1 > 1000:\n        a1 = 1000")
    lines.append("    return a0, a1")
    code = "\n".join(lines) + "\n"
    _selftest(code, q)
    return code


def _selftest(code, q, n=4000, seed=0):
    """생성 코드 == policy_int (극단값 포함 무작위 입력)."""
    import random
    ns = {}
    exec(code, ns)
    rnd = random.Random(seed)
    for t in range(n):
        span = (2000, 20000, 10 ** 6)[t % 3]
        X = [rnd.randint(-span, span) for _ in q["M"]]
        assert list(ns["policy"](*X)) == policy_int(q, X), "생성 코드와 정수 기준 불일치"


def _insert_after(src, anchor, text):
    assert src.count(anchor) == 1, "앵커가 1개가 아님: %r" % anchor[:60]
    return src.replace(anchor, anchor + text)


def _insert_before(src, anchor, text):
    assert src.count(anchor) == 1, "앵커가 1개가 아님: %r" % anchor[:60]
    return src.replace(anchor, text + anchor)


def _set_const(src, name, val):
    pat = re.compile(r"^(%s\s*=\s*)([^#\n]*?)(\s*(#.*)?)$" % re.escape(name), re.M)
    assert len(pat.findall(src)) == 1, name
    return pat.sub(lambda m: m.group(1) + str(val) + (m.group(3) or ""), src)


def export(policy_path, platform, out_path=None):
    ck = torch.load(policy_path, map_location="cpu", weights_only=False)
    assert isinstance(ck, dict) and "state" in ck, "배포 메타데이터가 있는 policy.pt가 필요 (ppo.py v2 이후)"
    assert ck["mode"] in ("residual2", "steer_only"), "내보내기는 residual2(Δ조향·Δ파워)·steer_only(Δ조향, 파워 100)만 지원"
    if ck["mode"] == "steer_only":
        ck = dict(ck, dpower_max=0.0)               # 파워는 규칙(100 고정) 그대로 — Δ파워 없음
    P = PLATFORM[platform]
    prof = core.load_profile(ck.get("profile") or P["profile"])   # 학습한 하드웨어 프로파일 (실측 치수 등)
    src = open(os.path.join(ROOT, P["src"]), encoding="utf-8").read()
    # 1) 규칙기반 상수 = 학습 때 쓴 값
    for k, v in ck["consts"].items():
        if k in ("USE_SENTINEL",):
            v = bool(v)
        elif k in ("KP", "KD"):
            v = float(v)
        else:
            v = int(v)
        src = _set_const(src, k, v)
    dec_ms = round(ck["decision_ticks"] * 1000.0 / prof["sensor_hz"], 1)
    mm_per_motor_deg = math.pi * prof["wheel_diameter_mm"] / 360.0 * prof.get("gear_ratio", 1.0)
    header = (
        "\n# ==================== 강화학습 잔차 정책 (자동 생성 — 직접 고치지 말 것) ====================\n"
        "# 원본: %s\n# 구조: 관측 22 → %d → %d → 2 (%s), 결정 주기 %sms, Δ조향 ±%g, Δ파워 ±%g\n"
        "# 생성: sim/rl/export_policy.py — 규칙기반 로직은 %s와 같음\n"
        "# 빛값 VALUE·포트는 실기에 맞게 바꿔도 됨 (정책은 정규화 0~100만 봄). 위의 규칙 상수는\n"
        "# 정책이 그 값 위에서 학습됐으므로 바꾸면 재학습 권장. 시작 시 POLICY_US = 정책 1회 µs\n"
        % (os.path.relpath(policy_path, ROOT), ck["actor_hid"], ck["actor_hid"], ck["act"], dec_ms,
           ck["dsteer_max"], ck["dpower_max"], P["src"]))
    consts = ("DECISION_MS = %d             # 정책 결정 주기\nDSTEER_MAX = %d\nDPOWER_MAX = %d\n"
              "KV = %d                  # 모터 deg/s → 정규화 바퀴속도 ×1000 (×KV//1000)\n"
              % (max(1, round(dec_ms)), round(ck["dsteer_max"]), round(ck["dpower_max"]),
                 round(mm_per_motor_deg / ck["v_ref"] * 1e6)))
    pol = gen_policy_code(ck["state"], ck)
    anchor_consts = "FORCE_MS = 2            # 센서값이 안 변해도 이 간격마다 한 번은 제어\n"
    src = _insert_after(src, anchor_consts, header + consts + "\n" + pol)
    # 2) run() 초기화: 결정 상태
    src = _insert_before(src, "    sw = StopWatch()\n",
                         "    t_dec = -1000           # 마지막 정책 결정 시각 (음수 = 아직 없음)\n"
                         "    DS = 0                  # 현재 Δ조향 (정수)\n"
                         "    DP = 0                  # 현재 Δ파워\n"
                         "    ap0 = ap1 = 0           # 직전 행동 (×1000)\n"
                         "    err_o = 0               # 직전 스텝 규칙 오차·최종 조향 (관측용)\n"
                         "    steer_o = 0\n"
                         "    # 센서 정규화값 최근 4결정 [현재 L·R·F, 1전, 2전, 3전] — 리스트 대신 변수 (GC 없음)\n"
                         "    q0 = q1 = q2 = q3 = q4 = q5 = q6 = q7 = q8 = q9 = q10 = q11 = 0\n"
                         "    tb = StopWatch()        # 정책 1회 연산시간(µs) 실측 → 시뮬 call_cost_us.policy에 넣을 값\n"
                         "    for _ in range(50):\n"
                         "        policy(500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500,\n"
                         "               0, 0, 0, 300, 300, 0, 0, 0, 0, 0)\n"
                         "    print(\"POLICY_US\", tb.time() * 20)\n")
    # 3) 프레임마다 정규화 직후: 결정 시점이면 관측 → 정책 → 행동 갱신
    yaw, vl, vr = P["yaw"], P["vl"], P["vr"]
    decide = (
        "        if now - t_dec >= DECISION_MS:\n"
        "            if t_dec < 0:                           # 첫 결정: 이력을 현재값으로 채움\n"
        "                q3 = q6 = q9 = nL\n"
        "                q4 = q7 = q10 = nR\n"
        "                q5 = q8 = q11 = nF\n"
        "            else:                                   # 이력 한 칸 밀기\n"
        "                q9 = q6\n                q10 = q7\n                q11 = q8\n"
        "                q6 = q3\n                q7 = q4\n                q8 = q5\n"
        "                q3 = q0\n                q4 = q1\n                q5 = q2\n"
        "            q0 = nL\n            q1 = nR\n            q2 = nF\n"
        "            t_dec = now\n"
        "            sa = (now - t_ahead) * 2\n"
        "            sa = 0 if sa < 0 else (1000 if sa > 1000 else sa)\n"
        "            sl = 0\n"
        "            if t_lost >= 0:\n"
        "                sl = (now - t_lost) * 2\n"
        "                sl = 0 if sl < 0 else (1000 if sl > 1000 else sl)\n"
        "            ap0, ap1 = policy(q0 * 10, q1 * 10, q2 * 10, q3 * 10, q4 * 10, q5 * 10,\n"
        "                              q6 * 10, q7 * 10, q8 * 10, q9 * 10, q10 * 10, q11 * 10,\n"
        "                              err_o * 5, steer_o * 10, int((%s) * 2.5),\n"
        "                              %s * KV // 1000, %s * KV // 1000,\n"
        "                              ap0, ap1, sa, sl, side * 1000)\n"
        "            DS = (ap0 * DSTEER_MAX + 500) // 1000\n"
        "            DP = ap1 * DPOWER_MAX // 1000\n" % (yaw, vl, vr))
    src = _insert_after(src, "        nF = (min(max(rF, bF), wF) - bF) * 100 // gF\n", decide)
    # 4) 잔차 적용: 조향(속도스케줄 전), 파워(사전감속 뒤)
    src = _insert_before(src, "        if SPD_K10 > 0:                             # 조향비례 감속\n",
                         "        steer += DS                                 # ★정책 Δ조향\n")
    src = _insert_before(src, "        delta = steer - prev_steer                  # 슬루 리미터\n",
                         "        power = power + DP                          # ★정책 Δ파워\n"
                         "        power = 0 if power < 0 else (100 if power > 100 else power)\n")
    # 5) 관측용: 이번 스텝의 오차·최종 조향 기록
    src = _insert_after(src, "            steer = prev_steer - MAX_STEER_DELTA\n        prev_steer = steer\n",   # 슬루 리미터 끝 (직각·피벗 블록의 prev_steer와 구분)
                        "        err_o = error\n        steer_o = steer\n")
    src = src.replace('"STAT,rule,', '"STAT,rl,')          # 실기 STAT 줄이 어느 프로그램 것인지
    src = src.replace("#  spike_lf3_rule: SPIKE Prime 3센서 라인트레이싱 (v16 계보 이식)",
                      "#  spike_lf3_rl: SPIKE Prime 3센서 + 강화학습 잔차 정책 (자동 생성)")
    src = src.replace("#  ev3_lf3_rule: EV3 3센서 라인트레이싱 (Pybricks v4 EV3 네이티브)",
                      "#  ev3_lf3_rl: EV3 3센서 + 강화학습 잔차 정책 (자동 생성)")
    out = out_path or os.path.join(ROOT, P["out"])
    open(out, "w", encoding="utf-8").write(src)
    compile(src, out, "exec")                       # 문법 확인
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("--platforms", default="spike,ev3")
    ap.add_argument("--out", default=None, help="출력 파일 (플랫폼 하나일 때)")
    args = ap.parse_args()
    for pf in args.platforms.split(","):
        print("생성:", export(args.policy, pf, args.out))


if __name__ == "__main__":
    main()
