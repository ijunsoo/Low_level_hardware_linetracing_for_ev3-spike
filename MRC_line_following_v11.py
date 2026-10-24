from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis, Color
from pybricks.tools import wait

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()

# ★요(yaw)축 자동감지: 정지 상태에서 중력(±9810)이 실리는 가속도 축 = 로봇 상하축.
#   허브를 어느 방향으로 달아도, 펌웨어가 좌표 변환을 해주든 안 하든 항상 맞는다.
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

lm = Motor(Port.B, Direction.COUNTERCLOCKWISE)
rm = Motor(Port.F)
c1 = PUPDevice(Port.A)    # 좌측부터 1 컬러센서
c2 = PUPDevice(Port.C)    # 2 컬러센서
c3 = PUPDevice(Port.E)    # 3 컬러센서
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

# ★자이로 댐핑 게인: 조향 += KG × 요레이트(deg/s)
#   라인센서는 지연(수십ms)이 있지만 자이로는 즉각적이라,
#   지연으로 인한 좌우 흔들림을 D게인 대신 자이로가 감쇠한다 (시뮬 검증:
#   지연 10~40ms 전 구간 완주, 흔들림 ±11~13mm).
#   ※ 적용 후 흔들림이 오히려 '커지면' 부호를 반대로 (KG_S = -100)
KG_S = 100    # = 0.1 × 1000

# ★코너 사전 피벗 (v11, 실험 스위치) — 빠른 직진 뒤 코너 인식 지연 대응.
#   원리: 코너에선 전체 흰색 '전에' 바깥 센서가 라인을 밟는다.
#   그 접촉을 기억 → 전체 흰색이 지속되면 그 방향으로 즉시 풀파워 피벗.
#   ※ 시뮬에서는 v10 뱅킹 대비 이득이 확인되지 않음 — 실기에서 A/B 해볼 것.
CORNER_ON  = True
C_WHITE    = 220    # 피벗 확정 전 전체흰색 지속 루프수 (~40ms)
C_MEM      = 500    # 바깥센서 접촉 기억 유지 루프수
C_TURN_MIN = 150    # 최소 피벗 루프수 (조기종료 방지)
C_TURN_MAX = 8000   # 피벗 포기
C_TURN_ST  = 95     # 피벗 조향

# 라인트레이싱
def pid_control(power, dec_v, st, kp, ki, kd, value, w_offset):
    S = 100000
    KP = round(kp * S)
    KI = round(ki * S)
    KD = round(kd * S)

    # ★ EMA 수정 (유일한 버그픽스: 700+300=1000)
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
    on_count  = 0
    ON_CONFIRM = 2
    line_off = False

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

    # ★코너 피벗 상태 (v11)
    turning = False
    turn_dir = 0
    mem_dir = -1
    mem_age = 0
    white_cnt = 0
    turn_frames = 0
    cb1 = b1 + (wh1 - b1) * 3 // 10   # 바깥센서 '진검정' 문턱
    cb4 = b4 + (wh4 - b4) * 3 // 10
    mid2 = (b2 + wh2) // 2
    mid3 = (b3 + wh3) // 2

    while True:
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]

        # ── ★코너 사전 피벗 (v11) ── #
        if CORNER_ON:
            if turning:
                turn_frames += 1
                if turn_frames >= C_TURN_MIN and (r2 < mid2 or r3 < mid3):
                    turning = False           # 중앙 재포착 → PID 리셋 복귀
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
            if r1 < cb1 and r4 > w4:
                mem_dir = 0
                mem_age = 0
            elif r4 < cb4 and r1 > w1:
                mem_dir = 1
                mem_age = 0
            elif mem_dir >= 0:
                mem_age += 1
                if mem_age > C_MEM:
                    mem_dir = -1
            if mem_dir >= 0 and r1 > w1 and r2 > w2 and r3 > w3 and r4 > w4:
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

        s1 = (ALPHA * min(max(r1, b1), wh1) + A_COMP * ps1) // E_S
        s2 = (ALPHA * min(max(r2, b2), wh2) + A_COMP * ps2) // E_S
        s3 = (ALPHA * min(max(r3, b3), wh3) + A_COMP * ps3) // E_S
        s4 = (ALPHA * min(max(r4, b4), wh4) + A_COMP * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        # ★이탈 판정은 '생값'으로 (v4 검증) — EMA 지연 없이 흰색 뜨는 즉시 트리거
        raw_off = (r1 > w1 and r2 > w2 and r3 > w3 and r4 > w4)
        
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
                    # ★ 수정 완료: 현재 루프의 센서값을 바탕으로 오차를 미리 계산하여 초기화 (미분 튐 완벽 방지)
                    last_error = (s1 + s2) - (s3 + s4) - error_offset 
            else:
                line_off = False

        error = (s1 + s2) - (s3 + s4) - error_offset

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
            if s1 + s2 + s3 + s4 < cross:
                steering = 0

        # ★ 자이로 댐핑 (좌회전 + → 우조향 +)
        #   이탈(뱅킹) 중에는 끔 — 복구 회전까지 감쇠해서 이탈감지가 굼떠지는 것 방지
        if not line_off:
            steering += (KG_S * YAW_SIGN * int(hub.imu.angular_velocity(YAW_AXIS))) // 1000

        # ★ Slew Rate Limiter: 프레임간 조향 변화를 제한하여 부드럽게 전환
        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        steering_dc(steering, repower)

# --------------------------- 메인 --------------------------- #

# 실기에서 실제 사용 중인 보정값 (사용자 튜닝)
value = [30, 2, 30, 2, 30, 1, 25, 2]

w_offset = 5

error_offset = round(calibrate_error_offset(20))

# ★ 변경: EMA 버그 수정 + PID 게인만 조정
#    진동 줄이기: kp를 적당히, kd로 진동 억제
#    기존: kp=0.03, ki=0, kd=5
#    수정: kp=0.15, ki=0, kd=3
#번호: 4243
# 자이로 댐핑판 게인 (시뮬 최적): kp 0.65, ki 0(중요!), kd 2.22 유지, KG 0.1
pid_control(100, 0.00001 ,66, 0.65, 0, 2.22, value, w_offset)
#100, 0, 78, 0.34, 0.00015, 3.2
#100, 0, 70, 0.3, 0.00015, 2.8  
#미완성100, 0.002, 75, 0.22, 0.00013, 2.8