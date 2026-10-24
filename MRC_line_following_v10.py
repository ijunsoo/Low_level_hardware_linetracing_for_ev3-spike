from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis
from pybricks.tools import wait

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
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
            steering += (KG_S * int(hub.imu.angular_velocity(Axis.Z))) // 1000

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