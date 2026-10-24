from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis
from pybricks.tools import wait, StopWatch

# =============================================================
#  흔들림 지문 로거 v2 = v14 주행 로직 '그대로' + 기록만 추가.
#  라인 위(직선 구간 시작점)에 놓고 실행:
#   캘리브레이션 → 자이로 자가판별 → 3초 라인트레이싱하며
#   10ms 간격 기록 → 자동 정지 → CSV 출력 (전체 복사해서 붙여넣기)
# =============================================================

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
c1 = PUPDevice(Port.A)
c2 = PUPDevice(Port.C)
c3 = PUPDevice(Port.B)
c4 = PUPDevice(Port.D)

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

KG_S = 450

def gyro_sign_check():
    for duty in (40, 60):
        lm.dc(-duty)
        rm.dc(duty)
        wait(150)
        total = 0
        for _ in range(20):
            total += hub.imu.angular_velocity(Axis.Z)
            wait(10)
        lm.dc(duty)
        rm.dc(-duty)
        wait(350)
        lm.dc(0)
        rm.dc(0)
        wait(300)
        if total > 600:
            return 1
        if total < -600:
            return -1
    hub.speaker.beep()
    return 1

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
    w2 = b2 + (wh2 - b2) * 50 // 100
    w3 = b3 + (wh3 - b3) * 50 // 100
    OFF_CONFIRM = 4
    off_count = 0

    # ── 로깅 준비 ── #
    sw = StopWatch()
    log = []
    next_log = 0
    sw.reset()

    ps1 = c1.read(mode=0)[0]
    ps2 = c2.read(mode=0)[0]
    ps3 = c3.read(mode=0)[0]
    ps4 = c4.read(mode=0)[0]

    while sw.time() < 3000:              # ★3초만 주행하고 자동 정지
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]

        s1 = (ALPHA * min(max(r1, b1), wh1) + A_COMP * ps1) // E_S
        s2 = (ALPHA * min(max(r2, b2), wh2) + A_COMP * ps2) // E_S
        s3 = (ALPHA * min(max(r3, b3), wh3) + A_COMP * ps3) // E_S
        s4 = (ALPHA * min(max(r4, b4), wh4) + A_COMP * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        raw_off = (r2 > w2 and r3 > w3)

        if raw_off:
            on_count = 0
            off_count += 1
            if off_count >= OFF_CONFIRM:
                line_off = True
        else:
            off_count = 0
            if line_off:
                on_count += 1
                if on_count >= ON_CONFIRM:
                    line_off = False
                    on_count = 0
                    integral = 0
                    last_error = (s1 + s2) - (s3 + s4) - error_offset
                    prev_steering = 0
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

        if line_off:
            repower = power
            if on_count == 0:
                steering = -st if direction == 0 else st
        else:
            repower = power
            if s1 + s2 + s3 + s4 < cross:
                steering = 0

        gz = int(hub.imu.angular_velocity(Axis.Z))
        if not line_off:
            g = (KG_EFF * gz) // 1000
            if g > 40:
                g = 40
            elif g < -40:
                g = -40
            steering += g

        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        steering_dc(steering, repower)

        # ── 10ms 간격 기록 (약 300행) ── #
        t = sw.time()
        if t >= next_log:
            next_log = t + 10
            log.append((t, error, gz, steering, 1 if line_off else 0))

    lm.dc(0)
    rm.dc(0)
    return log

value = [28, 2, 26, 2, 25, 2, 21, 1]
w_offset = 12
error_offset = round(calibrate_error_offset(20))

KG_EFF = KG_S * gyro_sign_check()
print("gyro_sign:", 1 if KG_EFF > 0 else -1)

log = pid_control(100, 0.00001, 60, 0.4, 0, 8, value, w_offset)

print("t_ms,error,gyro,steer,off")
for row in log:
    print("%d,%d,%d,%d,%d" % row)
