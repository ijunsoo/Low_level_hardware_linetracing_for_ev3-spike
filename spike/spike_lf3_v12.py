from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis

# =============================================================
#  v12 → 3센서판 (MRC_line_following_v12.py 로직 그대로, 센서만 4개 → 3개)
#
#  배치: 좌 C(+25mm) · 가운데 A · 우 B(-25mm), 셋 다 전방 90mm 일렬 / 모터 E(좌) · F(우, 역방향)
#  바꾼 것 (이것만):
#   - 안쪽 페어 c2·c3 (PID 오차) → 좌·우 (포트 C·B 그대로)
#   - 바깥 페어 c1·c4 (코너 방향기억) → 바깥 센서가 없어서 좌·우가 같이 맡음
#   - 이탈 = 페어 둘 다 흰색 → 세 센서 모두 흰색
#     (좌우가 ±25mm로 벌어져서 라인 위에서도 좌우는 흰색 — 가운데가 '라인 있음'을 알려 줌)
#   - 지그재그 관통 구간(ZONES, 원래 꺼져 있던 기능) 삭제
#  그대로: 정규화 0~100, EMA 831, 미분필터 351, kp 0.762 / kd 10.1, 자이로 0.258,
#   조향 변화 제한 61, 이탈 후 4프레임 직전조향 유지 → 기억 방향으로 83 뱅킹, 교차점 직진
#  ★v12 원본은 감속이 있음: 조향비례 감속(SPD_K10 8, 하한 77) + 뱅킹 중 파워 73
#   파워 100 고정으로 비교하려면 아래 SPD_K10 = 0, RECOVER_POWER = 100
# =============================================================

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
cL = PUPDevice(Port.C)    # 좌 (v12 c2 + 코너감지 c1 역할)
cR = PUPDevice(Port.B)    # 우 (v12 c3 + 코너감지 c4 역할)
cF = PUPDevice(Port.A)    # 가운데 (이탈 판정용)

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

# 라인트레이싱 (v12 챔피언 설정)
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
    CORNER_BLACK = 36      # 좌/우 센서 진검정 문턱 (정규화)
    BLACK_MID   = 50       # 검정 중간 문턱 (정규화)
    W_NORM      = 82       # 흰색(이탈) 문턱 (정규화)
    ST_BANK     = 83       # 이탈복구 뱅킹 조향
    RECOVER_POWER = 73     # 뱅킹 중 파워 (파워 100 고정 비교: 100)
    SPD_K10     = 8        # 조향비례 감속 0.8 (×10 정수) (파워 100 고정 비교: 0)
    MIN_POWER   = 77       # 감속 하한

    whL, bL, whR, bR, whF, bF = value
    rgL, rgR, rgF = whL - bL, whR - bR, whF - bF

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
    def norm3(rl, rr, rf):
        nl = (min(max(rl, bL), whL) - bL) * 100 // rgL
        nr = (min(max(rr, bR), whR) - bR) * 100 // rgR
        nf = (min(max(rf, bF), whF) - bF) * 100 // rgF
        return nl, nr, nf

    prl = cL.read(mode=0)[0]
    prr = cR.read(mode=0)[0]
    prf = cF.read(mode=0)[0]
    psl, psr, psf = norm3(prl, prr, prf)

    while True:
        rl = cL.read(mode=0)[0]
        rr = cR.read(mode=0)[0]
        rf = cF.read(mode=0)[0]

        # 프레임 동기: 새 센서 샘플이 없으면 건너뜀 (EMA/미분이 새 샘플 기준으로 튜닝됨)
        if rl == prl and rr == prr and rf == prf:
            continue
        prl, prr, prf = rl, rr, rf

        nl, nr, nf = norm3(rl, rr, rf)

        # EMA (정규화값 기준)
        sl = (ALPHA * nl + A_COMP * psl) // E_S
        sr = (ALPHA * nr + A_COMP * psr) // E_S
        psl, psr = sl, sr

        # ── 코너 방향기억: 좌/우 한쪽만 진검정 (v12 바깥 페어 역할) ── #
        left_deep  = nl < CORNER_BLACK and nr > 60
        right_deep = nr < CORNER_BLACK and nl > 60
        if left_deep or right_deep:
            mem_dir = 0 if left_deep else 1
            mem_age = 0
        elif mem_dir is not None:
            mem_age += 1
            if mem_age > MEM_FRAMES:
                mem_dir = None

        # ── 이탈 판정: 세 센서 모두 흰색 (v12: 안쪽 페어 둘 다 흰색) ── #
        lost = nl > W_NORM and nr > W_NORM and nf > W_NORM

        # ── 오차 = 좌우 차이 ── #
        error = 2 * (sl - sr)

        if not lost:
            if was_off:            # 재포착 → PID 리셋 (미분 튐 방지)
                integral = 0
                d_filt = 0
                last_error = error
                prev_steering = 0
            was_off = False
            gap = 0

            if nl < BLACK_MID and nr > 60:
                direction = 0
            elif nr < BLACK_MID and nl > 60:
                direction = 1

            integral += error
            diff = error - last_error
            d_filt = (D_FILTER * diff + D_COMP * d_filt) // E_S
            steering = (KP * error + KD * d_filt) // S
            last_error = error

            # 교차점: 좌우 동시 진검정 → 직진 유지
            if sl < BLACK_MID and sr < BLACK_MID:
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

        # ★조향비례 감속: 꺾는 만큼 자동 감속
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

# spike_value.py 출력 그대로: [흰, 검] × (좌, 우, 가운데)
value = [34, 6, 22, 3, 21, 3]

# v12 챔피언: power=100, kp=0.762, kd=10.1, kg=0.258(위 KG_S)
pid_control(100, 0.762, 10.1, value)
