from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis
from pybricks.tools import wait

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

KG_S = 450    # 실기 A/B 확정(2026-07-11): 450 > 200. 실제 센서지연이 큰
              # 영역이라 강댐핑이 이득 (200 하향 실험은 기각됨).

# ★자이로 부호 자가판별 v2: 확실히 돌 만큼 세게(40→안되면 60) 좌회전시키고,
#   '평균 30deg/s 이상 감지'를 통과해야 채택. (duty 22는 데드존 언저리라
#   마찰로 안 돌면 노이즈 부호 = 동전던지기가 됐음 — 실측 로그로 확인)
def gyro_sign_check():
    for duty in (40, 60):
        lm.dc(-duty)
        rm.dc(duty)            # 제자리 좌회전 (반시계)
        wait(150)
        total = 0
        for _ in range(20):
            total += hub.imu.angular_velocity(Axis.Z)
            wait(10)
        lm.dc(duty)
        rm.dc(-duty)           # 되돌리기
        wait(350)
        lm.dc(0)
        rm.dc(0)
        wait(300)
        if total > 600:        # 20샘플 평균 +30deg/s↑: 확실한 정부호
            return 1
        if total < -600:
            return -1
    hub.speaker.beep()         # 판별 실패 경고 (기본 +1 사용)
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
    # 이탈 문턱: 안쪽 페어만 사용, 레인지 50% 시점 발동 (센서별 균등)
    w2 = b2 + (wh2 - b2) * 50 // 100
    w3 = b3 + (wh3 - b3) * 50 // 100
    OFF_CONFIRM = 4      # 연속 N샘플 흰색이어야 이탈 확정 (~3ms, 위빙 오발 차단)
    off_count = 0

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
                    prev_steering = 0      # 재포착: 뱅킹 조향 이력 리셋
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
            repower = power              # 감속 없음: 뱅킹도 풀파워
            if on_count == 0:
                steering = -st if direction == 0 else st
        else:
            repower = power
            if s1 + s2 + s3 + s4 < cross:
                steering = 0

        if not line_off:
            # 자이로 댐핑 — 기여분 ±40 클램프: 직선 위빙(±40deg/s)엔 그대로,
            # 복구 직후 고속회전(±350deg/s)에선 폭주 차단 (실측 로그: 무클램프시
            # 재포착 순간 +157이 조향을 날려 좌우 핑퐁 스핀 유발)
            g = (KG_EFF * int(hub.imu.angular_velocity(Axis.Z))) // 1000
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

value = [28, 2, 26, 2, 25, 2, 21, 1]
w_offset = 12
error_offset = round(calibrate_error_offset(20))

KG_EFF = KG_S * gyro_sign_check()   # 자이로 부호 자동 보정 (시작 시 0.7초)
# 스캔 확정(2026-07-11): kp 0.4 + kd 8 + KG 450 (지연 25ms 가정 저흔들림 베스트)
pid_control(100, 0.00001, 60, 0.4, 0, 8, value, w_offset)
