# =====================================================================
#  MRC 라인트레이싱 v3  ── 옛 코드(빠른 dc 버전) + 최소 추정기
# ---------------------------------------------------------------------
#  옛 코드의 유일한 약점: raw_off(4센서 전부 흰색)가 뜨면 '즉시' 뱅킹 →
#    가는 라인이 센서 틈으로 잠깐 사라져도 거짓이탈 → 핑퐁 → 흔들림.
#  최소 수정(여기만 추가): '즉시 뱅킹' 앞에 짧은 '틈 브리지' 한 단계.
#    틈이 GAP_BRIDGE 틱 이내면  → 직전 조향을 그대로 유지(dead-reckon)하며 통과.
#    틈이 GAP_BRIDGE 틱을 넘으면 → 그때서야 진짜 이탈로 보고 옛날처럼 뱅킹.
#  나머지(dc 풀파워 / 정수연산 / 대기없는 타이트 루프 / 게인)는 옛날 그대로.
# =====================================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port
from pybricks.tools import wait

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
c1 = PUPDevice(Port.A)    # 좌측부터 1 컬러센서
c2 = PUPDevice(Port.C)    # 2 컬러센서
c3 = PUPDevice(Port.B)    # 3 컬러센서
c4 = PUPDevice(Port.D)    # 4 컬러센서

# --------------------------- 함수 --------------------------- #

# dc함수를 조향값으로 제어
def steering_dc(steering, power):
    steering = max(-100, min(100, steering))
    slope = (power * 2) / 100
    if steering < 0:
        st_B = round(slope * steering + power, 2)
        st_C = power
    else:
        st_B = power
        st_C = round(-slope * steering + power, 2)
    lm.dc(st_B)
    rm.dc(st_C)

# 4개 센서값 균형잡기
def calibrate_error_offset(samples=10):
    total = 0
    for _ in range(samples):
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]
        total += (r1 + r2) - (r3 + r4)
        wait(50)
    return total / samples

# 라인트레이싱
#   gap_bridge: 틈을 '직진 유지'로 메우는 최대 틱수. 이 안이면 이탈 아님(브리지),
#               넘으면 진짜 이탈. 가는 라인이 잠깐 사라지는 시간 < 진짜 이탈 시간.
#               흔들리면 ↑(틈 더 오래 무시), 진짜 이탈복구가 굼뜨면 ↓.
def pid_control(power, dec_v, st, kp, ki, kd, value, w_offset, gap_bridge):
    S = 100000
    KP = round(kp * S)
    KI = round(ki * S)
    KD = round(kd * S)

    ALPHA  = 700
    A_COMP = 300
    E_S    = 1000

    wh1, wh2, wh3, wh4 = value[0], value[2], value[4], value[6]
    b1,  b2,  b3,  b4  = value[1], value[3], value[5], value[7]

    integral   = 0
    max_error  = ((wh1 - b1) + (wh2 - b2) + (wh3 - b3) + (wh4 - b4)) // 8
    last_error = 0

    repower   = power
    direction = 0

    gap        = 0          # ★ 연속 '틈(raw_off)' 틱수
    line_off   = False      # 진짜 이탈 상태(gap_bridge 초과)

    prev_steering = 0          # Slew Rate Limiter 용 이전 조향값
    MAX_STEER_DELTA = 15       # 한 사이클당 최대 조향 변화량

    l_black = (wh1 + b1) // 2
    r_black = (wh4 + b4) // 2
    cross   = (b1 + b2 + b3 + b4) // 4 + 20
    w1 = wh1 - w_offset
    w2 = wh2 - w_offset
    w3 = wh3 - w_offset
    w4 = wh4 - w_offset

    ps1 = c1.read(mode=0)[0]
    ps2 = c2.read(mode=0)[0]
    ps3 = c3.read(mode=0)[0]
    ps4 = c4.read(mode=0)[0]

    while True:
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]

        s1 = (ALPHA * min(max(r1, b1), wh1) + A_COMP * ps1) // E_S
        s2 = (ALPHA * min(max(r2, b2), wh2) + A_COMP * ps2) // E_S
        s3 = (ALPHA * min(max(r3, b3), wh3) + A_COMP * ps3) // E_S
        s4 = (ALPHA * min(max(r4, b4), wh4) + A_COMP * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        raw_off = (s1 > w1 and s2 > w2 and s3 > w3 and s4 > w4)

        # ★ 틈 카운터 + 재포착 처리
        if raw_off:
            gap += 1
        else:
            if gap > 0:
                # 방금 라인 재포착 → 미분 튐 방지(현재 센서로 오차 초기화, 적분 리셋)
                last_error = (s1 + s2) - (s3 + s4) - error_offset
                integral   = 0
            gap = 0

        line_off = raw_off and (gap > gap_bridge)   # gap_bridge 넘어야 진짜 이탈

        # 방향 갱신(라인 볼 때만 갱신 → 틈에선 마지막 본 방향 유지)
        if s1 < l_black:
            direction = 0
        elif s4 < r_black:
            direction = 1

        error = (s1 + s2) - (s3 + s4) - error_offset

        if raw_off:
            # ── 틈 구간 ──
            if line_off:
                # 진짜 이탈: 옛날처럼 마지막 방향으로 뱅킹 + 감속
                integral = 0
                repower  = max(50, repower - dec_v)
                steering = -st if direction == 0 else st
            else:
                # 짧은 틈: 직전 조향 그대로 유지(dead-reckon)하며 통과, 파워 유지
                repower  = power
                steering = prev_steering
                # last_error / integral 은 동결(갱신 안 함)
        else:
            # ── 정상 추종(옛 코드 그대로) ──
            repower = power
            integral += error
            if KI != 0:
                max_integral = (S * 100 - abs(max_error * KP)) // KI
                integral = max(-max_integral, min(max_integral, integral))

            diff     = error - last_error
            steering = (KP * error + KI * integral + KD * diff) // S
            last_error = error

            if s1 + s2 + s3 + s4 < cross:   # 교차점(전부 검정) → 직진
                steering = 0

        # ★ Slew Rate Limiter: 프레임간 조향 변화를 제한하여 부드럽게 전환
        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        steering_dc(steering, repower)

# --------------------------- 메인 --------------------------- #

value = [64, 9, 52, 7, 53, 6, 61, 8]

w_offset = 17

error_offset = round(calibrate_error_offset(20))

# 옛날 튜닝 그대로 + gap_bridge 한 개만 추가.
#   gap_bridge 튜닝: 흔들리면 ↑(8→12→...), 진짜이탈 복구가 굼뜨면 ↓(8→5→...)
#번호: 4243
pid_control(100, 0.00001, 75, 0.26, 0.00017, 3, value, w_offset, 8)
