from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis, Color
from pybricks.tools import wait

# =============================================================
#  v13 = 사용자 실기코드(v10 계열) + 직각코너 대응 3가지만 추가
#   ① 코너 트립와이어: 바깥 전방센서(170mm)가 코너 가로줄을 밟는
#      순간(한쪽만 진검정) → 방향 기억 + 사전 감속 시작
#      → 이탈을 '감지 후 반응'이 아니라 '도착 전 준비'로 바꿈
#   ② 이탈 시 즉시 감속: dec_v 점진감속(사실상 무감속) → 즉시 RECOVER_POWER
#   ③ 뱅킹 방향: 신선한 코너 기억 우선, 없으면 기존 direction
#  나머지 로직·게인은 실기 코드 그대로.
# =============================================================

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
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
c1 = PUPDevice(Port.A)    # 좌측부터 1 컬러센서 (전방 170mm)
c2 = PUPDevice(Port.C)    # 2 컬러센서 (전방 90mm)
c3 = PUPDevice(Port.B)    # 3 컬러센서 (전방 90mm)
c4 = PUPDevice(Port.D)    # 4 컬러센서 (전방 170mm)

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

# ★자이로 댐핑 게인 (실기 확정값 유지)
KG_S = 150

# ★코너 트립와이어/사전감속 파라미터
PB_FRAMES = 1600      # 사전감속 유지 프레임 (약 300ms @5kHz루프) — 접근+코너 통과까지
PB_POWER  = 25        # 사전감속 파워 (체감되게 강하게)
MEM_FRAMES = 1500     # 코너 방향기억 유효 프레임 (약 300ms)
RECOVER_POWER = 50    # 이탈(뱅킹) 파워 (브레이크 이후)
BRAKE_FRAMES = 500    # ★이탈 첫 순간 진짜 브레이크(단락제동) 유지 (~95ms)

# 라인트레이싱
def pid_control(power, dec_v, st, kp, ki, kd, value, w_offset):
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
    on_count  = 0
    ON_CONFIRM = 2
    line_off = False

    prev_steering = 0
    MAX_STEER_DELTA = 15

    l_black = (wh1 + b1) // 2
    r_black = (wh4 + b4) // 2
    cross   = (b1 + b2 + b3 + b4) // 4 + 20
    w1 = wh1 - w_offset
    w2 = wh2 - w_offset
    w3 = wh3 - w_offset
    w4 = wh4 - w_offset

    # ★코너 트립와이어 문턱 (생값 기준): 진검정 = 검정+레인지 35%,
    #   반대쪽 '라인 없음' = 검정+레인지 60% 이상
    deep1 = b1 + (wh1 - b1) * 35 // 100
    deep4 = b4 + (wh4 - b4) * 35 // 100
    lite1 = b1 + (wh1 - b1) * 60 // 100
    lite4 = b4 + (wh4 - b4) * 60 // 100
    deep1e = deep1 + (wh1 - b1) // 4      # EMA 잔상용 완화 문턱
    deep4e = deep4 + (wh4 - b4) // 4

    mem_dir = None       # 코너 방향기억 (0=좌, 1=우)
    mem_age = 0
    pb_cnt  = 0          # 사전감속 잔여 프레임
    brake_cnt = 0        # 이탈 브레이크 잔여 프레임

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

        # ★① 코너 트립와이어: 바깥 한쪽만 진검정 = 전방 170mm에 직각 가로줄.
        #   가로줄 스침이 ~12ms로 짧아 생값 샘플이 놓칠 수 있음 →
        #   생값(즉시) 또는 EMA(잔상, 문턱 완화 +25%)의 OR로 잡는다.
        left_deep  = (r1 < deep1 or s1 < deep1e) and r4 > lite4
        right_deep = (r4 < deep4 or s4 < deep4e) and r1 > lite1
        if left_deep or right_deep:
            if mem_dir is None:          # 신선한 첫 감지 → 사전감속 발동
                pb_cnt = PB_FRAMES
                hub.light.on(Color.RED)  # ★디버그: 사전감속 발동 = 빨강
            mem_dir = 0 if left_deep else 1
            mem_age = 0
        elif mem_dir is not None:
            mem_age += 1
            if mem_age > MEM_FRAMES:
                mem_dir = None

        # 이탈 판정은 '생값'으로 (기존 유지)
        raw_off = (r1 > w1 and r2 > w2 and r3 > w3 and r4 > w4)

        if raw_off:
            on_count = 0
            if not line_off:             # ★이탈 첫 진입 → 즉시 브레이크 발동
                brake_cnt = BRAKE_FRAMES
                hub.light.on(Color.YELLOW)   # ★디버그: 이탈 브레이크 = 노랑
            line_off = True
        else:
            if line_off:
                on_count += 1
                if on_count >= ON_CONFIRM:
                    line_off = False
                    on_count = 0
                    integral = 0
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
            # ★② 이탈 첫 ~95ms: 단락제동으로 관성부터 죽임 (duty 감속은 밀림)
            if brake_cnt > 0:
                brake_cnt -= 1
                lm.brake()
                rm.brake()
                prev_steering = 0
                continue
            repower = RECOVER_POWER          # 브레이크 후 뱅킹 파워
            if on_count == 0:
                # ★③ 신선한 코너 기억 우선, 없으면 기존 direction
                d = mem_dir if mem_dir is not None else direction
                steering = -st if d == 0 else st
        else:
            repower = power
            if s1 + s2 + s3 + s4 < cross:
                steering = 0

        # 자이로 댐핑 (이탈 중엔 끔 — 기존 유지)
        if not line_off:
            steering += (KG_S * YAW_SIGN * int(hub.imu.angular_velocity(YAW_AXIS))) // 1000

        # ★① 사전감속: 트립와이어 발동 후 PB_FRAMES 동안 파워 제한
        if pb_cnt > 0:
            pb_cnt -= 1
            if repower > PB_POWER:
                repower = PB_POWER
            if pb_cnt == 0:
                hub.light.on(Color.GREEN)   # 사전감속 종료 = 초록

        # Slew Rate Limiter (기존 유지)
        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        steering_dc(steering, repower)

# --------------------------- 메인 --------------------------- #

# 실기에서 실제 사용 중인 보정값 (사용자 튜닝)
value = [28, 2, 26, 2, 25, 2, 21, 1]

w_offset = 12

error_offset = round(calibrate_error_offset(20))

# 스캔 베스트(지연 15ms 시뮬): kp 0.65 유지, kd 2.22→8 (지연 위상보상), KG 0.15
pid_control(100, 0.00001, 60, 0.65, 0, 8, value, w_offset)
