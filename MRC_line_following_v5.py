# =====================================================================
#  MRC 라인트레이싱 v5  ── v4 + ★센서별 정규화(0~100)
# ---------------------------------------------------------------------
#  [v4 → v5 변경 이유: 시뮬레이터로 확정한 실기 버그]
#   실측 센서 흰색이 45/42/31/25로 제각각(다이내믹레인지 40 vs 23).
#   생값 차이로 오차를 계산하면:
#    1) 교차 마커에서 c1·c4가 동시에 검정을 밟아도 좌우가 상쇄되지 않고
#       가짜 오차(-17)가 생겨 홱 꺾음 → 이탈 → '중앙틈' 오판 → 표류 사망.
#    2) 좌/우로 같은 거리 벗어나도 오차 크기가 달라 게인이 비대칭으로 먹음.
#   → 각 센서를 (r-black)*100//(white-black) 으로 정규화(0=검정,100=흰색).
#     문턱값(이탈/교차/방향)도 전 센서 공통 스케일이 됨.
#
#  [게인 스케일 주의] 오차 단위가 raw(±77)→정규화(±200)로 바뀌어
#   KP/KI/KD/EDGE_ERR를 raw 평균레인지(≈31/100) 비율로 환산해둠.
#   시뮬 스윕(sim/sweep5.py)으로 재튜닝할 것.
# =====================================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port
from pybricks.tools import wait, StopWatch

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.B, Direction.COUNTERCLOCKWISE)
rm = Motor(Port.F)
c1 = PUPDevice(Port.A)    # 좌측부터 1 EV3 컬러센서
c2 = PUPDevice(Port.C)    # 2
c3 = PUPDevice(Port.E)    # 3
c4 = PUPDevice(Port.D)    # 4
SENSORS = (c1, c2, c3, c4)

# ===================== 튜닝 파라미터(여기만 만지면 됨) ===================== #
# --- 캘리브레이션(흰/검 기준값, 2026-07-04 라인스캔 실측) --- #
VALUE = [45, 5, 42, 5, 31, 3, 25, 2]   # [w1,b1, w2,b2, w3,b3, w4,b4]

# --- ★정규화 스케일 문턱값 (전 센서 공통, 0=검정 100=흰색) --- #
W_NORM    = 82    # 정규화값이 이보다 크면 '흰색' (이탈판정용)
BLACK_MID = 50    # 이보다 작으면 '검정 밟음' (방향 기억용)
CROSS_SUM = 60    # 4센서 정규화 합이 이보다 작으면 교차점(전부 검정)

# --- 속도/파워 --- #
POWER = 100              # dc 듀티(100=풀파워). 코스 못 따라가면 ↓
RECOVER_POWER = 70       # 진짜(가장자리)이탈 복구 뱅킹 시 파워

# --- 조향 PID (정규화 스케일로 환산: v4값 × raw평균레인지 0.31) --- #
KP = 0.08
KI = 0.00005
KD = 1.0
I_LIMIT = 15    # 적분항이 조향에 기여하는 최대치(±)

# --- 조향 보조 --- #
ST_BANK         = 75    # 진짜 이탈시 마지막 방향으로 꺾는 뱅킹량
MAX_STEER_DELTA = 15    # 1스텝당 조향 변화 상한(슬루)
EMA_ALPHA       = 700   # /1000 센서 평활
D_FILTER        = 300   # /1000 미분 전용 필터

# --- 이탈감지(위치로 구분: 중앙틈 vs 가장자리이탈) --- #
GAP_LIMIT = 10
EDGE_ERR  = 80   # 정규화 스케일 (센서 1개 풀검정 = 100)

# --- 디버그/튜닝 계측 --- #
DEBUG       = False
DEBUG_EVERY = 30
BENCH_MS    = 800
# ====================================================================== #

S = 100000
KP_S = round(KP * S)
KI_S = round(KI * S)
KD_S = round(KD * S)
E_S    = 1000
A_COMP = E_S - EMA_ALPHA
D_COMP = E_S - D_FILTER

WH = [VALUE[0], VALUE[2], VALUE[4], VALUE[6]]
BK = [VALUE[1], VALUE[3], VALUE[5], VALUE[7]]
RG = [WH[i] - BK[i] for i in range(4)]     # ★ 센서별 다이내믹레인지


# --------------------------- 함수 --------------------------- #

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


def benchmark(ms):
    prev = (c1.read(mode=0)[0], c2.read(mode=0)[0],
            c3.read(mode=0)[0], c4.read(mode=0)[0])
    sw = StopWatch()
    loops = 0
    updates = 0
    while sw.time() < ms:
        cur = (c1.read(mode=0)[0], c2.read(mode=0)[0],
               c3.read(mode=0)[0], c4.read(mode=0)[0])
        loops += 1
        if cur != prev:
            updates += 1
            prev = cur
    el = sw.time()
    if el < 1:
        el = 1
    return loops * 1000 // el, updates * 1000 // el


# ★ 센서별 정규화: 0(검정) ~ 100(흰색)
def norm4(r1, r2, r3, r4):
    n1 = (min(max(r1, BK[0]), WH[0]) - BK[0]) * 100 // RG[0]
    n2 = (min(max(r2, BK[1]), WH[1]) - BK[1]) * 100 // RG[1]
    n3 = (min(max(r3, BK[2]), WH[2]) - BK[2]) * 100 // RG[2]
    n4 = (min(max(r4, BK[3]), WH[3]) - BK[3]) * 100 // RG[3]
    return n1, n2, n3, n4


# 4개 센서값 균형잡기 (★정규화 스케일)
def calibrate_error_offset(samples=10):
    total = 0
    for _ in range(samples):
        n1, n2, n3, n4 = norm4(c1.read(mode=0)[0], c2.read(mode=0)[0],
                               c3.read(mode=0)[0], c4.read(mode=0)[0])
        total += (n1 + n2) - (n3 + n4)
        wait(50)
    return total / samples


# 라인트레이싱 (프레임 동기형 + 정규화)
def pid_control(error_offset):
    integral   = 0
    last_error = 0
    d_filt     = 0

    repower   = POWER
    direction = 0
    was_off   = False
    prev_steering = 0
    gap        = 0
    last_good_steer = 0

    r1 = c1.read(mode=0)[0]
    r2 = c2.read(mode=0)[0]
    r3 = c3.read(mode=0)[0]
    r4 = c4.read(mode=0)[0]
    raw_prev = (r1, r2, r3, r4)
    ps1, ps2, ps3, ps4 = norm4(r1, r2, r3, r4)

    dbg = 0

    while True:
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]
        raw = (r1, r2, r3, r4)

        # 새 프레임(값 변화)에서만 1스텝 진행
        if raw == raw_prev:
            continue
        raw_prev = raw

        # ── ★정규화 → EMA ── #
        n1, n2, n3, n4 = norm4(r1, r2, r3, r4)
        s1 = (EMA_ALPHA * n1 + A_COMP * ps1) // E_S
        s2 = (EMA_ALPHA * n2 + A_COMP * ps2) // E_S
        s3 = (EMA_ALPHA * n3 + A_COMP * ps3) // E_S
        s4 = (EMA_ALPHA * n4 + A_COMP * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        # 이탈 판정은 생(정규화) 값으로 → EMA 지연 0
        raw_off = (n1 > W_NORM and n2 > W_NORM and n3 > W_NORM and n4 > W_NORM)
        error = (s1 + s2) - (s3 + s4) - error_offset

        if not raw_off:
            if was_off:
                last_error = error
                integral   = 0
                d_filt     = 0
            was_off = False
            gap = 0

            if s1 < BLACK_MID:
                direction = 0
            elif s4 < BLACK_MID:
                direction = 1

            integral += error
            if KI_S != 0:
                max_integral = I_LIMIT * S // KI_S
                if integral > max_integral:
                    integral = max_integral
                elif integral < -max_integral:
                    integral = -max_integral

            diff     = error - last_error
            d_filt   = (D_FILTER * diff + D_COMP * d_filt) // E_S
            steering = (KP_S * error + KI_S * integral + KD_S * d_filt) // S
            last_error = error

            if s1 + s2 + s3 + s4 < CROSS_SUM:   # 교차점(전부 검정) → 직진
                steering = 0
            repower = POWER
            last_good_steer = steering
        else:
            gap += 1
            if gap <= GAP_LIMIT:
                steering = last_good_steer
                repower  = POWER
            elif abs(last_error) < EDGE_ERR:
                # 중앙에서 사라짐 = 센서틈 → 뱅킹 금지, 직전 조향 유지
                steering = last_good_steer
                repower  = POWER
            else:
                # 가장자리에서 사라짐 = 진짜 이탈 → 뱅킹
                was_off  = True
                integral = 0
                steering = -ST_BANK if direction == 0 else ST_BANK
                repower  = RECOVER_POWER

        delta = steering - prev_steering
        if delta > MAX_STEER_DELTA:
            steering = prev_steering + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steering = prev_steering - MAX_STEER_DELTA
        prev_steering = steering

        steering_dc(steering, repower)

        if DEBUG:
            dbg += 1
            if dbg >= DEBUG_EVERY:
                dbg = 0
                print("st", steering, "off", int(raw_off), "gap", gap, "le", last_error)


# --------------------------- 메인 --------------------------- #

hub.speaker.beep(700, 80)
loop_hz, upd_hz = benchmark(BENCH_MS)
print("LOOP", loop_hz, "Hz / SENSOR_UPDATE ~", upd_hz, "Hz")
hub.speaker.beep(min(2000, 400 + upd_hz * 4), 120)

error_offset = round(calibrate_error_offset(20))
print("error_offset", error_offset)
hub.speaker.beep(1200, 100)

pid_control(error_offset)
