from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis
from pybricks.tools import wait

# =============================================================
#  v11 → 3센서판 (MRC_line_following_v11.py 로직 그대로, 센서만 4개 → 3개)
#
#  배치: 좌 C(+25mm) · 가운데 A · 우 B(-25mm), 셋 다 전방 90mm 일렬 / 모터 E(좌) · F(우, 역방향)
#  바꾼 것 (이것만):
#   - 바깥 c1·c4 → 좌·우, 안쪽 c2·c3 → 가운데 하나
#     오차 (c1+c2)-(c3+c4) → (좌+가운데)-(가운데+우) = 좌-우
#   - 센서마다 흰/검 폭이 달라서(좌 52, 우·가운데 35) v11 원래 스케일(검 2 ~ 흰 30)로 맞춰 읽음
#     → kp·kd·st·w_offset·문턱 숫자는 v11 그대로 씀
#   - 포트·모터 방향을 지금 로봇에 맞춤
#  그대로: EMA 700, kp 0.65 / ki 0 / kd 2.22, 자이로 댐핑 0.1, 조향 변화 제한 15,
#   세 센서 흰색 = 이탈 → 즉시 마지막 본 쪽으로 66 뱅킹,
#   코너 사전 피벗 (좌/우가 진검정 밟은 기억 + 흰색 220루프 지속 → 95 피벗 → 가운데 재포착 시 복귀),
#   교차점 직진, 출발 시 좌우 균형 보정 (가운데 센서를 라인 위에 두고 시작)
# =============================================================

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()

# ★요(yaw)축 자동감지: 정지 상태에서 중력(±9810)이 실리는 가속도 축 = 로봇 상하축.
#   (반드시 로봇이 바닥에 정지한 상태로 프로그램을 시작할 것)
wait(200)
ax = hub.imu.acceleration(Axis.X)
ay = hub.imu.acceleration(Axis.Y)
az = hub.imu.acceleration(Axis.Z)
if abs(ax) >= abs(ay) and abs(ax) >= abs(az):
    YAW_AXIS, YAW_SIGN = Axis.X, (1 if ax > 0 else -1)
elif abs(ay) >= abs(az):
    YAW_AXIS, YAW_SIGN = Axis.Y, (1 if ay > 0 else -1)
else:
    YAW_AXIS, YAW_SIGN = Axis.Z, (1 if az > 0 else -1)

lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
cL = PUPDevice(Port.C)    # 좌 (v11 c1)
cF = PUPDevice(Port.A)    # 가운데 (v11 c2·c3)
cR = PUPDevice(Port.B)    # 우 (v11 c4)

# spike_value.py 출력 그대로: [흰, 검] × (좌, 우, 가운데)
VALUE = [34, 6, 22, 3, 21, 3]
WL, BL, WR, BR, WF, BF = VALUE

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

# 세 센서를 v11 스케일(검 2 ~ 흰 30)로 읽기 → (좌, 가운데, 우)
def read3():
    l = cL.read(mode=0)[0]
    f = cF.read(mode=0)[0]
    r = cR.read(mode=0)[0]
    l = BL if l < BL else (WL if l > WL else l)
    f = BF if f < BF else (WF if f > WF else f)
    r = BR if r < BR else (WR if r > WR else r)
    return (2 + (l - BL) * 28 // (WL - BL),
            2 + (f - BF) * 28 // (WF - BF),
            2 + (r - BR) * 28 // (WR - BR))

# 좌우 센서값 균형잡기 (v11: (c1+c2)-(c3+c4) → 좌-우)
def calibrate_error_offset(samples=10):
    total = 0
    for _ in range(samples):
        r1, r2, r4 = read3()
        total += r1 - r4
        wait(50)
    return total / samples

# ★자이로 댐핑 게인: 조향 += KG × 요레이트(deg/s)
#   ※ 적용 후 흔들림이 오히려 '커지면' 부호를 반대로 (KG_S = -100)
KG_S = 100    # = 0.1 × 1000

# ★코너 사전 피벗 (v11)
#   원리: 코너에선 전체 흰색 '전에' 바깥(좌/우) 센서가 라인을 밟는다.
#   그 접촉을 기억 → 전체 흰색이 지속되면 그 방향으로 즉시 풀파워 피벗.
CORNER_ON  = True
C_WHITE    = 220    # 피벗 확정 전 전체흰색 지속 루프수
C_MEM      = 500    # 바깥센서 접촉 기억 유지 루프수
C_TURN_MIN = 150    # 최소 피벗 루프수 (조기종료 방지)
C_TURN_MAX = 8000   # 피벗 포기
C_TURN_ST  = 95     # 피벗 조향

# 라인트레이싱
def pid_control(power, dec_v, st, kp, ki, kd, w_offset):
    S = 100000
    KP = round(kp * S)
    KI = round(ki * S)
    KD = round(kd * S)

    ALPHA  = 700
    A_COMP = 300
    E_S    = 1000

    WH, BK = 30, 2                      # v11 센서 스케일 (read3가 맞춰 줌)

    integral   = 0
    max_error  = (WH - BK) * 4 // 8
    last_error = 0

    repower   = power
    direction = 0
    on_count  = 0
    ON_CONFIRM = 2
    line_off = False

    prev_steering = 0          # Slew Rate Limiter 용 이전 조향값
    MAX_STEER_DELTA = 15       # 한 사이클당 최대 조향 변화량

    l_black = (WH + BK) // 2
    r_black = (WH + BK) // 2
    cross   = BK + 20           # v11: 4센서 합과 비교 → 좌 + 가운데×2 + 우
    w       = WH - w_offset     # 흰색 문턱

    ps1, ps2, ps4 = read3()

    # ★코너 피벗 상태 (v11)
    turning = False
    turn_dir = 0
    mem_dir = -1
    mem_age = 0
    white_cnt = 0
    turn_frames = 0
    cb  = BK + (WH - BK) * 3 // 10      # 바깥(좌/우) 센서 '진검정' 문턱
    mid = (BK + WH) // 2                # 가운데 재포착 문턱

    while True:
        r1, r2, r4 = read3()            # 좌, 가운데, 우

        # ── ★코너 사전 피벗 (v11) ── #
        if CORNER_ON:
            if turning:
                turn_frames += 1
                if turn_frames >= C_TURN_MIN and r2 < mid:
                    turning = False           # 가운데 재포착 → PID 리셋 복귀
                    integral = 0
                    last_error = 0
                    prev_steering = 0
                    line_off = False
                    on_count = 0
                    mem_dir = -1
                    steering_dc(0, power)
                    continue
                if turn_frames > C_TURN_MAX:
                    turning = False
                    mem_dir = -1
                else:
                    steering_dc(-C_TURN_ST if turn_dir == 0 else C_TURN_ST, power)
                    continue
            if r1 < cb and r4 > w:
                mem_dir = 0
                mem_age = 0
            elif r4 < cb and r1 > w:
                mem_dir = 1
                mem_age = 0
            elif mem_dir >= 0:
                mem_age += 1
                if mem_age > C_MEM:
                    mem_dir = -1
            if mem_dir >= 0 and r1 > w and r2 > w and r4 > w:
                white_cnt += 1
            else:
                white_cnt = 0
            if white_cnt >= C_WHITE:
                white_cnt = 0
                turning = True
                turn_dir = mem_dir
                mem_dir = -1
                turn_frames = 0
                steering_dc(-C_TURN_ST if turn_dir == 0 else C_TURN_ST, power)
                continue

        s1 = (ALPHA * r1 + A_COMP * ps1) // E_S
        s2 = (ALPHA * r2 + A_COMP * ps2) // E_S
        s4 = (ALPHA * r4 + A_COMP * ps4) // E_S
        ps1, ps2, ps4 = s1, s2, s4

        # ★이탈 판정은 '생값'으로 — EMA 지연 없이 흰색 뜨는 즉시 트리거
        raw_off = (r1 > w and r2 > w and r4 > w)

        if raw_off:
            on_count = 0
            line_off = True
        else:
            if line_off:
                on_count += 1
                if on_count >= ON_CONFIRM:
                    line_off = False
                    on_count = 0
                    integral = 0
                    last_error = s1 - s4 - error_offset
            else:
                line_off = False

        error = s1 - s4 - error_offset

        if not line_off:
            integral += error
            if KI != 0:
                max_integral = (S * 100 - abs(max_error * KP)) // KI
                integral = max(-max_integral, min(max_integral, integral))

        diff     = error - last_error
        steering = (KP * error + KI * integral + KD * diff) // S
        last_error = error

        if s1 < l_black:
            direction = 0
        elif s4 < r_black:
            direction = 1

        # ── 이탈/교차점 로직 ── #
        if line_off:
            repower = max(50, repower - dec_v)
            if on_count == 0:
                steering = -st if direction == 0 else st
        else:
            repower = power
            if s1 + 2 * s2 + s4 < cross:
                steering = 0

        # ★ 자이로 댐핑 (좌회전 + → 우조향 +), 이탈(뱅킹) 중에는 끔
        if not line_off:
            steering += (KG_S * YAW_SIGN * int(hub.imu.angular_velocity(YAW_AXIS))) // 1000

        # ★ Slew Rate Limiter
        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        steering_dc(steering, repower)

# --------------------------- 메인 --------------------------- #

w_offset = 5

error_offset = round(calibrate_error_offset(20))

# v11 그대로: power 100, dec_v 0.00001, st 66, kp 0.65, ki 0, kd 2.22
pid_control(100, 0.00001, 66, 0.65, 0, 2.22, w_offset)
