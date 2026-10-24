# =====================================================================
#  MRC 라인트레이싱 v8  ── 기어2배·무감속 실기 검증판
# ---------------------------------------------------------------------
#  [전제] 기어업 2배 (바퀴 = 모터×2, 실측: dc100=모터 1626deg/s, tau 115ms)
#  [원칙] 감속 없음 — 순항/턴/복구 전부 파워 100
#  [v6 대비 변경]
#   1) ★엣지 추종(EDGE_BIAS): 라인 20mm < 센서간격 25mm → 정중앙은 신호가
#      끊기는 데드밴드. 타깃을 가장자리로 옮겨 연속 신호 확보 (고속 안정 핵심)
#   2) ★지그재그 직진 관통: 오도메트리(모터각) 3450~4450mm 구간은 피벗 끄고
#      직진 (봉우리 147mm — 돌지 않고 뚫는 게 빠르고 안전)
#   3) 게인: 시뮬 스윕 상위값 (kp0.072 kd8.1, EMA 544, D필터 153)
#  [시뮬 예측] 직선·마커 통과, 직각 코너(1.5m 지점)가 최난관 —
#   실기 결과가 다르면 그 차이로 시뮬을 보정한다.
# ---------------------------------------------------------------------
#  (구 v6 설명) v5(정규화) + 코너 피벗
# ---------------------------------------------------------------------
#  [v5 → v6: 시뮬레이터로 검증한 코너 처리]
#   풀파워 직각/예각 코너는 PID·뱅킹으로 못 잡는다(50조합 스윕 전멸).
#   코너와 교차선은 '순간 센서 패턴'이 같아 즉시 판정 불가 →
#   ★시간 순서로 구별한다:
#     1) 바깥 센서(n1/n4) 하나만 진한 검정 → "그쪽 급코너 후보" 기억
#        (양쪽 동시 검정 = 교차마커 → 기억 안 함)
#     2) 직후 전체 흰색(라인 소실) → 코너 확정 → 저파워 피벗
#        (교차선이면 중앙 라인이 유지되므로 기억만 만료 → 오발동 없음)
#     3) 중앙 센서 재포착 → PID 리셋 후 복귀
#   시뮬 결과: 실측 파라미터 기어1배로 8.5m 풀코스(직각·예각지그재그·
#   물방울/8자 교차 포함) 20.9초 완주.
# ---------------------------------------------------------------------
#  (이하 v5 설명)  v4 + 센서별 정규화(0~100)
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
POWER = 100              # 순항 파워 (무감속 원칙: 고정)
RECOVER_POWER = 100      # 복구 뱅킹도 감속 없음

# --- 조향 PID (정규화 스케일로 환산: v4값 × raw평균레인지 0.31) --- #
# 실기 1차 피드백(직선 과민 진동) 반영: KD 8.1→3.5 + 필터 강화
# 그래도 떨리면: KD ↓(3.5→2) → EMA_ALPHA ↓(450→380) → D_FILTER ↓(120→90) 순서로
# (EMA_ALPHA/D_FILTER는 '새 값 비중'이라 ↓일수록 더 부드러움)
KP = 0.072
KI = 0.0
KD = 3.5
I_LIMIT = 15    # 적분항이 조향에 기여하는 최대치(±)

# --- 조향 보조 --- #
ST_BANK         = 70    # 진짜 이탈시 마지막 방향으로 꺾는 뱅킹량
MAX_STEER_DELTA = 20    # 1스텝당 조향 변화 상한(슬루, ↓=부드러움)
EMA_ALPHA       = 450   # /1000 센서 평활 (↓=더 평활=덜 떨림)
D_FILTER        = 120   # /1000 미분 전용 필터 (↓=D 더 평활)

# --- 이탈감지(위치로 구분: 중앙틈 vs 가장자리이탈) --- #
GAP_LIMIT = 10
EDGE_ERR  = 80   # 정규화 스케일 (센서 1개 풀검정 = 100)

# --- ★코너 피벗 (v6) --- #
CORNER_BLACK = 27    # 바깥 센서가 이보다 어두우면 '진한 검정'(코너 후보)
MEM_FRAMES   = 85    # 코너 후보 기억 유지 프레임수 (~78ms @1278Hz)
WHITE_CONF   = 50    # 피벗 진입 전 '전체 흰색' 연속 확인 프레임수(~38ms)
                     # (센서 틈 깜빡임=짧음 / 진짜 코너=길게 지속 으로 구별)
SETTLE_FR    = 0     # 무감속: 정착 감속 사용 안 함
SETTLE_PW    = 100
TURN_ST      = 100   # 피벗 조향량 (실기: 전류제한/미끄러짐 보상 위해 최대)
TURN_POWER   = 100   # 피벗 중 파워 (무감속)
REACQUIRE    = 48    # 중앙 센서가 이보다 어두우면 라인 재포착
TURN_TIMEOUT = 2000  # 피벗 포기 프레임수 (~1.6s)
TURN_MIN     = 60    # ★최소 회전 프레임(~45ms): 이전에 재포착 판정 금지
                     #  (실기에서 피벗이 즉시 끊겨 회전 못하는 문제 대응)

# --- ★엣지 추종 (v8) --- #
EDGE_BIAS = 30       # 추종 타깃을 라인 가장자리로(정규화 스케일). 반대편이 낫으면 -30

# --- ★코너 피벗 무장 창 (v8.2, 오도메트리) --- #
#  실기 피드백: 직선에서 피벗 오발동 연쇄(좌우 포인트턴 반복) → 이탈.
#  포인트턴이 필요한 직각은 코스에 하나(1.5m 지점)뿐 → 그 근처에서만 무장.
#  (지그재그는 관통, 커브/루프는 PID+뱅킹이 담당)
PIVOT_WIN_A = 1350   # 무장 시작 (주행거리 mm)
PIVOT_WIN_B = 1850   # 무장 끝

# --- ★지그재그 직진 관통 (v8, 오도메트리) --- #
GEAR      = 2.0                       # 바퀴 = 모터 × GEAR
DIST_K    = 3.14159 * 56 / 360 * GEAR # 모터각 1도당 이동거리(mm)
ZZ_START  = 3450     # 지그재그 관통 시작 (주행거리 mm)
ZZ_END    = 4450     # 관통 끝
ZZ_CAP    = 15       # 관통 중 조향 상한 (다리 추종 방지)

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

    # ★코너 피벗 상태 (v6)
    turning     = False
    turn_dir    = 0       # 0=좌, 1=우
    mem_dir     = -1      # -1=없음
    mem_age     = 0
    turn_frames = 0
    white_cnt   = 0
    settle      = 0

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

        # ── ★정규화 ── #
        n1, n2, n3, n4 = norm4(r1, r2, r3, r4)

        # ── ★지그재그 직진 관통 (v8) ── #
        dist = (abs(lm.angle()) + abs(rm.angle())) * 0.5 * DIST_K
        if ZZ_START <= dist <= ZZ_END:
            ps1, ps2, ps3, ps4 = n1, n2, n3, n4
            turning = False
            mem_dir = -1
            white_cnt = 0
            if n1 > W_NORM and n2 > W_NORM and n3 > W_NORM and n4 > W_NORM:
                st = 0                        # 라인 안 보임 → 직진 유지
            else:
                e = (n1 + n2) - (n3 + n4) - error_offset
                st = (KP_S * e) // S
                if st > ZZ_CAP:
                    st = ZZ_CAP
                elif st < -ZZ_CAP:
                    st = -ZZ_CAP
            prev_steering = st
            last_good_steer = 0
            last_error = 0
            steering_dc(st, POWER)
            continue

        # ── ★코너 피벗 (v6) ── #
        if turning:
            ps1, ps2, ps3, ps4 = n1, n2, n3, n4   # EMA 리프라임
            turn_frames += 1
            if turn_frames >= TURN_MIN and (n2 < REACQUIRE or n3 < REACQUIRE):
                # 중앙 재포착 → PID 리셋 후 정상 복귀
                turning = False
                integral = 0
                d_filt = 0
                last_error = 0
                was_off = False
                gap = 0
                prev_steering = 0
                last_good_steer = 0
                mem_dir = -1
                settle = SETTLE_FR
                steering_dc(0, SETTLE_PW)
                continue
            if turn_frames > TURN_TIMEOUT:
                turning = False        # 포기 → 아래 기존 로직에 맡김
                mem_dir = -1
            else:
                st = -TURN_ST if turn_dir == 0 else TURN_ST
                prev_steering = st
                steering_dc(st, TURN_POWER)
                continue

        # ★피벗은 코너 무장 창 안에서만 (직선 오발동 연쇄 차단, v8.2)
        if not (PIVOT_WIN_A <= dist <= PIVOT_WIN_B):
            mem_dir = -1
            white_cnt = 0

        # 진검정 기억 갱신 (양쪽 동시 검정=교차마커 → 기억 안 함)
        if n1 < CORNER_BLACK and n4 > 60:
            mem_dir = 0
            mem_age = 0
        elif n4 < CORNER_BLACK and n1 > 60:
            mem_dir = 1
            mem_age = 0
        elif mem_dir >= 0:
            mem_age += 1
            if mem_age > MEM_FRAMES:
                mem_dir = -1

        # 라인 소실 '지속' + 기억 신선 → 코너 확정, 피벗 진입
        # (센서 틈으로 인한 짧은 전체흰색 깜빡임은 white_cnt 로 걸러냄)
        if mem_dir >= 0 and n1 > W_NORM and n2 > W_NORM and n3 > W_NORM and n4 > W_NORM:
            white_cnt += 1
        else:
            white_cnt = 0
        if white_cnt >= WHITE_CONF:
            white_cnt = 0
            turning = True
            turn_dir = mem_dir
            turn_frames = 0
            ps1, ps2, ps3, ps4 = n1, n2, n3, n4
            st = -TURN_ST if turn_dir == 0 else TURN_ST
            prev_steering = st
            steering_dc(st, TURN_POWER)
            continue

        # ── EMA ── #
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

        if settle > 0:
            # ★피벗 복귀 직후 정착: 감속 유지로 오버슈트 방지
            settle -= 1
            if repower > SETTLE_PW:
                repower = SETTLE_PW

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

error_offset = round(calibrate_error_offset(20)) + EDGE_BIAS   # ★엣지 추종
print("error_offset(+bias)", error_offset)
hub.speaker.beep(1200, 100)

lm.reset_angle(0)     # ★오도메트리 기준점 (출발선에서 시작할 것)
rm.reset_angle(0)
pid_control(error_offset)
