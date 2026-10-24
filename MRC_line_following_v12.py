from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis

# =============================================================
#  v12: 투센서 라인트레이싱 (시뮬 챔피언 이식판)
#
#  ★반드시 하드웨어를 먼저 바꿀 것★
#   1) 가운데 센서 2개(c2, c3)를 바퀴축 앞 70mm로 후퇴
#      (바깥 c1, c4는 지금 170mm 그대로)
#   2) 기어 4배속
#
#  역할 분리:
#   - c2, c3 (안쪽 페어) = 라인트레이싱 전담 (PID 오차원)
#   - c1, c4 (바깥 페어) = 코너 방향기억 전담 (PID에 안 들어감)
#   - 자이로 = 흔들림 감쇠 (D항 대체)
#
#  시뮬 성적: 기어4, 12/12 완주, 12.1초/14.8m, 평균이탈 5mm
# =============================================================

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
c1 = PUPDevice(Port.A)    # 좌측부터 1 컬러센서 (전방 170mm, 코너감지)
c2 = PUPDevice(Port.C)    # 2 컬러센서 (전방 70mm, 라인트레이싱)
c3 = PUPDevice(Port.B)    # 3 컬러센서 (전방 70mm, 라인트레이싱)
c4 = PUPDevice(Port.D)    # 4 컬러센서 (전방 170mm, 코너감지)

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

# ★자이로 댐핑 게인 (시뮬 챔피언: kg=0.258)
#   ※ 적용 후 흔들림이 오히려 '커지면' 부호를 반대로 (KG_S = -258)
KG_S = 258    # = 0.258 × 1000

# ★지그재그 오도메트리 관통 구간 [(시작mm, 끝mm), ...]
#   실측 전이라 기본 비활성. 측정법: 한 바퀴 천천히 돌리며 지그재그
#   진입/탈출 지점의 주행거리를 hub 로그로 찍어서 채울 것.
ZONES = ()
GEAR = 4          # 기어비 (거리 환산용: 바퀴각 = 모터각 × GEAR)
WHEEL_MM = 56     # 바퀴 지름

# 라인트레이싱 (시뮬 GeoController 챔피언 설정 이식)
def pid_control(power, kp, kd, value):
    S = 100000
    KP = round(kp * S)
    KD = round(kd * S)

    # EMA (챔피언: alpha=831)
    ALPHA  = 831
    A_COMP = 1000 - ALPHA
    E_S    = 1000
    # 미분 저역필터 (챔피언: 351)
    D_FILTER = 351
    D_COMP   = 1000 - D_FILTER

    # ── 챔피언 파라미터 ── #
    MAX_STEER_DELTA = 61   # 프레임당 조향 변화 제한
    GAP_LIMIT   = 4        # 이탈 후 직전조향 유지 프레임 수
    MEM_FRAMES  = 333      # 코너 방향기억 유효 프레임
    CORNER_BLACK = 36      # 바깥센서 진검정 문턱 (정규화)
    BLACK_MID   = 50       # 검정 중간 문턱 (정규화)
    W_NORM      = 82       # 흰색(이탈) 문턱 (정규화)
    ST_BANK     = 83       # 이탈복구 뱅킹 조향
    RECOVER_POWER = 73     # 뱅킹 중 파워
    SPD_K10     = 8        # 조향비례 감속 0.8 (×10 정수)
    MIN_POWER   = 77       # 감속 하한
    ZONE_CAP    = 15       # 관통 구간 조향 상한

    wh1, wh2, wh3, wh4 = value[0], value[2], value[4], value[6]
    b1,  b2,  b3,  b4  = value[1], value[3], value[5], value[7]
    rg1, rg2, rg3, rg4 = wh1 - b1, wh2 - b2, wh3 - b3, wh4 - b4

    # 주행거리 환산: 모터각 → mm  (× π×56/360 × GEAR, 정수화 ×1000)
    DIST_K = round(3.14159 * WHEEL_MM / 360 * GEAR * 1000)

    integral   = 0
    last_error = 0
    d_filt     = 0
    direction  = 0
    mem_dir    = None      # 코너 방향기억 (0=좌, 1=우)
    mem_age    = 0
    gap        = 0
    was_off    = False
    prev_steering   = 0
    last_good_steer = 0

    # ── 정규화: 각 센서를 0(검정)~100(흰색)으로 ── #
    #  센서마다 흰/검 레인지가 달라(30/25 등) 생값 합차는 비대칭 바이어스가
    #  생김(v5에서 검증된 픽스). 정규화하면 error_offset 보정도 불필요.
    def norm4(r1, r2, r3, r4):
        n1 = (min(max(r1, b1), wh1) - b1) * 100 // rg1
        n2 = (min(max(r2, b2), wh2) - b2) * 100 // rg2
        n3 = (min(max(r3, b3), wh3) - b3) * 100 // rg3
        n4 = (min(max(r4, b4), wh4) - b4) * 100 // rg4
        return n1, n2, n3, n4

    pr1 = c1.read(mode=0)[0]
    pr2 = c2.read(mode=0)[0]
    pr3 = c3.read(mode=0)[0]
    pr4 = c4.read(mode=0)[0]
    ps1, ps2, ps3, ps4 = norm4(pr1, pr2, pr3, pr4)

    while True:
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]

        # 프레임 동기: 센서 갱신(1.4kHz)이 루프(5kHz)보다 느림 —
        # 새 샘플이 없으면 건너뜀 (EMA/미분이 새 샘플 기준으로 튜닝됨)
        if r1 == pr1 and r2 == pr2 and r3 == pr3 and r4 == pr4:
            continue
        pr1, pr2, pr3, pr4 = r1, r2, r3, r4

        n1, n2, n3, n4 = norm4(r1, r2, r3, r4)

        # ── 지그재그 오도메트리 관통 (ZONES 설정 시) ── #
        if ZONES:
            dist = (abs(lm.angle()) + abs(rm.angle())) * DIST_K // 2000
            in_zone = False
            for za, zb in ZONES:
                if za <= dist <= zb:
                    in_zone = True
                    break
            if in_zone:
                ps1, ps2, ps3, ps4 = n1, n2, n3, n4
                mem_dir = None
                gap = 0
                was_off = False
                if n2 > W_NORM and n3 > W_NORM:
                    steering = 0
                else:
                    steering = max(-ZONE_CAP, min(ZONE_CAP,
                                   (KP * 2 * (n2 - n3)) // S))
                prev_steering = steering
                last_error = 0
                d_filt = 0
                steering_dc(steering, power)
                continue

        # EMA (정규화값 기준)
        s1 = (ALPHA * n1 + A_COMP * ps1) // E_S
        s2 = (ALPHA * n2 + A_COMP * ps2) // E_S
        s3 = (ALPHA * n3 + A_COMP * ps3) // E_S
        s4 = (ALPHA * n4 + A_COMP * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        # ── 코너 방향기억: 바깥(전방) 센서 한쪽만 진검정 ── #
        left_deep  = n1 < CORNER_BLACK and n4 > 60
        right_deep = n4 < CORNER_BLACK and n1 > 60
        if left_deep or right_deep:
            mem_dir = 0 if left_deep else 1
            mem_age = 0
        elif mem_dir is not None:
            mem_age += 1
            if mem_age > MEM_FRAMES:
                mem_dir = None

        # ── 이탈 판정: 안쪽 페어 둘 다 흰색 (생값 정규화 기준) ── #
        lost = n2 > W_NORM and n3 > W_NORM

        # ── 오차 = 안쪽 페어 차이 (투센서 라인트레이싱) ── #
        error = 2 * (s2 - s3)

        if not lost:
            if was_off:            # 재포착 → PID 리셋 (미분 튐 방지)
                integral = 0
                d_filt = 0
                last_error = error
                prev_steering = 0
            was_off = False
            gap = 0

            if n1 < BLACK_MID and n4 > 60:
                direction = 0
            elif n4 < BLACK_MID and n1 > 60:
                direction = 1

            integral += error
            diff = error - last_error
            d_filt = (D_FILTER * diff + D_COMP * d_filt) // E_S
            steering = (KP * error + KD * d_filt) // S
            last_error = error

            # 교차점: 안쪽 페어 동시 진검정 → 직진 유지
            if s2 < BLACK_MID and s3 < BLACK_MID:
                steering = 0

            # ★자이로 댐핑 (좌회전 + → 우조향 +)
            steering += (KG_S * int(hub.imu.angular_velocity(Axis.Z))) // 1000
            out_power = power
            last_good_steer = steering
        else:
            gap += 1
            if gap <= GAP_LIMIT:       # 짧은 공백: 직전 조향 유지
                steering = last_good_steer
                out_power = power
            else:                      # 진짜 이탈: 기억 방향으로 뱅킹
                was_off = True
                d = mem_dir if mem_dir is not None else direction
                steering = -ST_BANK if d == 0 else ST_BANK
                out_power = RECOVER_POWER

        # ★조향비례 감속: 꺾는 만큼 자동 감속 (사전브레이크 대체)
        sched = power - (SPD_K10 * abs(steering)) // 10
        if sched < MIN_POWER:
            sched = MIN_POWER
        if out_power > sched:
            out_power = sched

        # Slew Rate Limiter
        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        steering_dc(steering, out_power)

# --------------------------- 메인 --------------------------- #

# 실기에서 실제 사용 중인 보정값 (사용자 튜닝)
# 정규화 도입으로 error_offset 보정은 불필요해짐 (비대칭이 자동 상쇄)
value = [30, 2, 30, 2, 30, 1, 25, 2]

# ★시뮬 챔피언 (기어4, 12/12 완주 12.1s):
#   power=100, kp=0.762, kd=10.1, kg=0.258(위 KG_S),
#   spd_k=0.8/min77, slew=61, gap=4, bank=83/73, mem=333
pid_control(100, 0.762, 10.1, value)
