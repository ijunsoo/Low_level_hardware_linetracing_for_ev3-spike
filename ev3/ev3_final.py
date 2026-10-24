from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait, StopWatch
try:
    from pybricks.iodevices import LUMPDevice   # 센서 모드를 직접 골라 읽음 (빨간 불 반사광)
except ImportError:
    LUMPDevice = None

# =============================================================
#  EV3 3센서 라인트레이서 — 최종판 (2026-10-24, E8R8과 같은 동작) · Pybricks v4
#
#  사용법
#   1) ev3/ev3_value.py로 빛값을 재서, 나온 VALUE 줄을 맨 아래 line_trace(...)의 VALUE에 붙여 넣는다
#      (빨간 불로 달릴 땐 두 파일 다 RED_LIGHT = True: ev3_value.py가 주는 VALUE_RED·VALUE0 줄을 고급 설정에)
#   2) 로봇을 라인 위에 놓고(가운데 센서는 라인 위, 좌우 센서는 흰 바닥) 실행 → 센서가 준비되면 바로 출발
#   3) 브릭 화면: START · PORT WAIT(장치 연결 다시 시도) · SENSOR CHECK · WAIT(주황 불, 센서 기다림) · READY
#      · ERROR(빨간 불) · LOG...(기록 출력 중) → DONE
#      주행 중 불빛: 빨강 = 앞에서 라인이 끊김(코너 예고) · 초록 = 라인 다시 찾음 · 노랑 = 라인 놓쳐서 강제 회전
#   4) 연습 땐 AUTO_STOP=True: 라인을 오래 잃으면 스스로 멈추고 마지막 2초 기록을 출력한다
#      (python3 sim/bb_parse.py 로그.txt 로 해석). 대회 땐 AUTO_STOP=False
#  배선: 컬러센서 S1 좌 · S2 가운데 · S3 우 (일렬 23mm 간격, 바퀴축 앞 17cm) / 모터 A·B 좌 · C·D 우
#  자이로 없이 좌우 모터 각도 차로 회전 속도를 잰다 (YAW_ENC_K)
#  센서: 기본은 RGB 반사광 — ColorSensor.reflection()은 Pybricks 4.1부터 빨강·초록·파랑 세 색으로 잰 반사광의
#   평균이다(그래서 컬러빛이 깜빡임). 실기(2026-10-24)에서 빨간 불 반사광보다 라인을 잘 따라가서 기본으로 쓴다.
#   RED_LIGHT = True면 SPIKE 최종판처럼 빨간 불 반사광 (LUMPDevice raw 모드 3, 안 되면 반사광 % 모드 0)
#  모든 회전은 파워 100 그대로 조향만으로 한다 (예외: 긴 직선 끝 직각에서 50ms 브레이크)
#  만든 과정·실험 기록: README.md, github.com/ijunsoo/Low_level_hardware_linetracing_for_ev3-spike
# =============================================================

# ======= 고급 설정 (실험으로 맞춘 값 — 보통은 그대로. 바꿀 값은 맨 아래 한 줄) ======= #
RED_LIGHT = False       # 센서 빛: False = RGB 반사광(기본, 실기에서 더 잘 달림) / True = 빨간 불 반사광 → 아래 세 줄을 씀
VALUE_RED = [87, 17, 121, 21, 121, 19]  # 빨간 불 raw 빛값 (EV3 실측 2026-10-24) — RED_LIGHT = True면 맨 아래 VALUE 대신 씀
RAW_MODE = True         # 빨간 불: raw 모드 3 (% 반사광보다 값이 약 3배 촘촘). False면 반사광 %(모드 0)와 VALUE0
VALUE0 = [22, 2, 22, 2, 37, 4]  # 빨간 불 반사광 %(모드 0) 빛값 — raw가 3초 넘게 0만 줄 때도 이걸로 바꿔 씀 (지금은 SPIKE 임시값)
AUTO_CAL = True         # 출발 자세(가운데 = 라인, 좌우 = 흰 바닥)에서 흰색을 다시 재서 VALUE 보정
KG2 = 216               # 곡선에서 흔들림 잡기 (-1 = KG 그대로)
G_CLAMP = 71            # 회전 감쇠가 조향에 더할 수 있는 최대값
CURVE_YAW = 153         # 회전 평균이 이 값(°/s)이면 '곡선' 100% → KP2·KG2를 씀
CURVE_TAU_MS = 119      # 그 회전 평균을 내는 시간 (ms)
STR_YAW = 40            # 회전 평균이 이 값(°/s)보다 작으면 '직진 중'
ALPHA = 950             # 센서 값 부드럽게 (×1000, 클수록 새 값 비중↑)
MAX_STEER_DELTA = 79    # 한 번에 바꿀 수 있는 조향 크기
W_NORM = 81             # 흰색 문턱 (정규화 0~100)
BLACK_MID = 51          # 검정 문턱
SIDE_ERR = 60           # 오차가 이보다 크게 기울면 '라인이 그쪽'으로 기억 (놓쳤을 때 돌 방향)
GAP_MS = 2              # 라인이 안 보여도 이 시간(ms)은 직전 조향 유지
AHEAD_HOLD_MS = 4       # 가운데 센서가 검정을 본 뒤 이 시간(ms)까지는 '앞에 라인 있음'
SENSOR_GAP_MM = 23      # 센서 간격 (mm, 실측)
CF_GAIN = 4             # 가운데 센서만 라인을 볼 때: 가운데가 완전히 하얘지면 이 mm만큼 벗어난 것으로 봄
SIDE_SEE = 16           # 옆 센서 진하기(100 − 정규화) 합이 이보다 크면 '옆에서 라인 보임'
DFLOOR = 25             # 진하기에서 빼는 바닥값 (흰 바닥의 작은 떨림을 라인으로 착각하지 않게)
RA_KQ = 40              # 직각 회전이 RA_YAW를 넘으면 넘은 1°/s당 조향을 0.40씩 줄임 (×100)
BR_ST = 100             # 직진 중 직각 신호(가운데 + 한쪽 옆이 진한 검정, 반대쪽 흰색)를 보면 미리 꺾는 세기
BR_DARK = 20            # 직각 신호 판정: 둘 다 이보다 어두워야 함 (정규화 %), 3스텝 연속
BRK_OUT = -100          # 브레이크 중 바깥 바퀴 파워 (−100 = 양쪽 다 역회전)
BRK_STR_MS = 300        # 이 시간(ms) 넘게 곧게 달린 뒤의 직각에서만 브레이크
FORCE_MS = 2            # 센서 값이 안 바뀌어도 이 간격(ms)마다 한 번은 제어
OFF_STOP_MS = 800       # AUTO_STOP: 세 센서가 모두 흰색으로 이만큼(ms) 지속되면 멈춤
DARK_STOP_MS = 300      #            모두 검정
GRAY_STOP_MS = 600      #            모두 회색 (매트 밖)
LOG_MS = 4              # 블랙박스 기록 간격 (ms)
LOG_N = 500             # 블랙박스 기록 수 (500 × 4ms = 2초)

# ============================= 장치 설정 ============================= #
ev3 = EV3Brick()


def show(*a):
    # 브릭 화면에 글자 표시 (화면 기능이 없으면 그냥 넘어감)
    try:
        ev3.screen.print(*a)
    except Exception:
        pass


show("START")


def dev(make, port, *a):
    # 장치 연결: 실패하면(바로 다시 실행하면 센서가 재시작 중이라 '장치 없음') 최대 5초 다시 시도 — 화면 PORT WAIT
    for _ in range(50):
        try:
            return make(port, *a)
        except OSError:
            show("PORT WAIT")
            wait(100)
    return make(port, *a)                 # 그래도 안 되면 오류를 그대로 (화면 ERROR)


LM = (dev(Motor, Port.A), dev(Motor, Port.B))     # 좌 모터 2개 (정방향 = 전진)
RM = (dev(Motor, Port.C, Direction.COUNTERCLOCKWISE), dev(Motor, Port.D, Direction.COUNTERCLOCKWISE))   # 우 모터 2개 (역방향 = 전진)
# ↑ 출발하자마자 제자리에서 돌거나 뒤로 가면 그쪽 모터의 Direction을 반대로 바꿀 것
RED = RED_LIGHT and LUMPDevice is not None     # 빨간 불로 읽나 (펌웨어에 LUMPDevice가 없으면 RGB로)
if RED:                             # 빨간 불 반사광: 센서 모드를 직접 골라 읽음
    cL = dev(LUMPDevice, Port.S1)   # 좌 센서
    cF = dev(LUMPDevice, Port.S2)   # 가운데 센서
    cR = dev(LUMPDevice, Port.S3)   # 우 센서
else:                               # RGB 반사광: ColorSensor.reflection()
    cL = dev(ColorSensor, Port.S1)
    cF = dev(ColorSensor, Port.S2)
    cR = dev(ColorSensor, Port.S3)

# =============================== 함수 =============================== #
BRK_CMD = [0, 0]        # 브레이크 중 [왼쪽, 오른쪽] 파워 (둘 다 0이면 평소 조향)


def steering_dc(steering, power):
    # 조향(−100 왼쪽 ~ +100 오른쪽)을 좌우 모터 파워로: 안쪽 바퀴를 줄이고, 100이면 안쪽 바퀴가 거꾸로 돈다
    if BRK_CMD[0] or BRK_CMD[1]:
        left, right = BRK_CMD[0], BRK_CMD[1]
    else:
        steering = max(-100, min(100, steering))
        slope = (power * 2) / 100
        if steering < 0:
            left, right = round(slope * steering + power, 2), power
        else:
            left, right = power, round(-slope * steering + power, 2)
    for m in LM:
        m.dc(left)
    for m in RM:
        m.dc(right)


YAW_K = 245             # 맨 아래 YAW_ENC_K 값으로 바뀜
encL, encR = LM[0], RM[0]   # 회전 속도 계산용 (좌·우 첫 모터 엔코더)
_YH = [[0, 0, 0] for _ in range(5)]   # 4ms마다 저장한 [시각, 좌 각도, 우 각도] (≈16~20ms치)
_YS = [0, -100]                       # [다음 저장 칸, 마지막 저장 시각]
_YW = StopWatch()


def yaw_rate():
    # 좌회전 + °/s (자이로 대신): 최근 ≈16ms 동안 (우 − 좌) 모터 각도 변화 × YAW_K
    t = _YW.time()
    aL = int(encL.angle())
    aR = int(encR.angle())
    i = _YS[0]
    if t - _YS[1] >= 4:
        h = _YH[i]
        h[0] = t
        h[1] = aL
        h[2] = aR
        _YS[1] = t
        i = i + 1 if i < 4 else 0
        _YS[0] = i
    o = _YH[i]                      # 가장 오래된 저장값
    dt = t - o[0]
    if dt <= 0 or dt > 200:
        return 0
    return ((aR - o[2]) - (aL - o[1])) * YAW_K // dt


STAT = [0, 0]           # [제어 횟수, 블랙박스 기록 수]
RUN_VAL = [None, 0]     # 실제로 쓴 빛값, 블랙박스에 줄여 적은 비트 수 (블랙박스 머리줄용)
LOG = bytearray(LOG_N * 7)   # 블랙박스 [센서 좌·우·가운데, 조향+128, 파워, 상태, 회전속도/8+128]


def run(VALUE, POWER, KP, KP2, KG, ST_BANK, ST_BANK_STR, RA_YAW, BRK_MS, AUTO_STOP):
    S = 100000
    KP_S = round(KP * S)
    KP2_S = round(KP2 * S)
    A_COMP = 1000 - ALPHA
    if RED:
        rdL, rdR, rdF = cL.read, cR.read, cF.read
        md = 3 if RAW_MODE else 0                   # 3 = 빨간 불 raw, 0 = 빨간 불 반사광 %
        RUN_VAL[0] = VALUE_RED if RAW_MODE else VALUE0
    else:
        md = -1                                     # −1 = RGB 반사광 (ColorSensor.reflection)
        RUN_VAL[0] = VALUE
        if RED_LIGHT:
            print("LUMPDevice 없음 → RGB 반사광으로 읽음 (맨 아래 VALUE 사용)")
    wL, bL, wR, bR, wF, bF = RUN_VAL[0]
    sh = 2 if md == 3 else 0                       # 블랙박스에는 raw ÷4로 기록
    RUN_VAL[1] = sh

    def rd3():
        if md < 0:
            return int(cL.reflection()), int(cR.reflection()), int(cF.reflection())
        if md:
            a = rdL(3)
            b = rdR(3)
            c = rdF(3)
            return abs(a[0] - a[1]), abs(b[0] - b[1]), abs(c[0] - c[1])
        return rdL(0)[0], rdR(0)[0], rdF(0)[0]

    # ---- 출발 대기: 센서가 0만 읽히는 동안은 출발하지 않음 (화면 WAIT + 주황 불) ---- #
    print("출발 준비: 센서 확인 중")
    show("SENSOR CHECK")
    t_rdy = StopWatch()
    waited = False
    t_msg = 0
    while True:
        try:
            a, b, c = rd3()
        except OSError:                             # 센서 재시작 중 읽기 오류 → 0으로 보고 계속 기다림
            a = b = c = 0
        if a + b > 4:                               # 좌우 센서가 바닥 값을 읽으면 준비됨
            break
        if not waited:
            ev3.light.on(Color.ORANGE)
        waited = True
        tt = t_rdy.time()
        if tt >= t_msg:                             # 1초마다
            print("센서 값 0 — 기다리는 중: 좌", a, "우", b, "가운데", c, "(케이블·포트 S1 좌·S2 가운데·S3 우 확인)")
            show("WAIT", a, b, c)
            if tt >= 1000:                          # 모드를 한 번 바꿨다 돌려 센서를 깨움
                try:
                    if md < 0:
                        cL.ambient()
                        cR.ambient()
                        cF.ambient()
                    elif md:
                        rdL(0)
                        rdR(0)
                        rdF(0)
                except OSError:
                    pass
            if md > 0 and tt >= 3000:               # 빨간 불 raw가 계속 0 → 반사광 %(모드 0)로 바꿔 달림
                md = 0
                sh = 0
                RUN_VAL[0] = VALUE0
                RUN_VAL[1] = 0
                wL, bL, wR, bR, wF, bF = VALUE0
                print("raw 모드 응답 없음 → 반사광 %(모드 0)로 전환 (VALUE0)")
            t_msg += 1000
        wait(10)
    if waited:
        wait(100)                                   # 첫 값 직후의 오래된 값 버림
    print("센서 준비됨: 좌", a, "우", b, "가운데", c,
          "(RGB 반사광)" if md < 0 else ("(빨간 불 raw)" if md else "(빨간 불 반사광 %)"))
    show("READY", a, b, c)
    if AUTO_CAL:                                    # 출발 자세에서 흰색 다시 재기
        tL = tR = tF = 0
        for _ in range(20):
            a, b, c = rd3()
            tL += a
            tR += b
            tF += c
            wait(5)
        mL, mR, mF = tL // 20, tR // 20, tF // 20
        if mL - bL > 8 and mR - bR > 8 and 2 * (mL - bL) > wL - bL and 2 * (mR - bR) > wR - bR:
            if 10 * (mF - bF) > 8 * (wF - bF) and 2 * mF >= min(mL, mR):
                wF = mF                             # 가운데도 흰 바닥 → 잰 값 그대로
            else:
                wF = wF * (mL + mR) // (wL + wR)    # 가운데가 라인 위 → 좌우와 같은 비율로
            wL, wR = mL, mR
            print("자동보정 VALUE", [wL, bL, wR, bR, wF, bF])
        else:
            print("자동보정 안 함 (좌우 센서를 흰 바닥에 두고 출발할 것) — VALUE 그대로")
    gL, gR, gF = wL - bL, wR - bR, wF - bF

    pL, pR, pF = rd3()
    sL = (min(max(pL, bL), wL) - bL) * 100 // gL
    sR = (min(max(pR, bR), wR) - bR) * 100 // gR
    sF = (min(max(pF, bF), wF) - bF) * 100 // gF

    prev_steer = 0
    last_good = 0           # 라인이 보일 때 마지막 조향
    side = -1               # 라인이 있던 쪽 기억 (−1 왼쪽, +1 오른쪽) = 놓쳤을 때 돌 방향
    ysgn = -1               # 가운데 센서만 라인을 볼 때 라인 쪽 (좌 +)
    was_off = False         # 라인을 놓쳐 강제 회전 중
    armed = False           # 코너 예고 (앞에서 라인이 끊김)
    t_ahead = 0
    t_lost = -1
    t_step = 0
    bank_dir = 0            # 강제 회전 방향
    bank_st = ST_BANK       # 이번 강제 회전 세기
    ra_on = False           # 이번 강제 회전이 직각(직진 중 놓침)인가
    brk_on = False          # 직각 브레이크 중
    brk_dir = 0
    t_brk = -10000
    t_str0 = 0              # 이번 직진이 시작된 시각
    br_n = 0                # 같은 쪽 직각 신호를 연속으로 본 횟수
    br_last = 0
    c_ema = 0               # 곡선 판단용 회전 평균 (°/s ×1000, 좌회전 +)
    li = 0                  # 블랙박스 기록 수
    t_allw = -1             # 세 센서 모두 흰색이 시작된 시각
    t_alld = -1             # 모두 검정
    t_allg = -1             # 모두 회색
    t_log = 0
    sw = StopWatch()

    while True:
        # ---- 센서 읽기 ---- #
        if md < 0:                                  # RGB 반사광
            rL = int(cL.reflection())
            rR = int(cR.reflection())
            rF = int(cF.reflection())
        elif md:                                    # 빨간 불 raw: 두 값의 차
            a = rdL(3)
            rL = a[0] - a[1] if a[0] > a[1] else a[1] - a[0]
            a = rdR(3)
            rR = a[0] - a[1] if a[0] > a[1] else a[1] - a[0]
            a = rdF(3)
            rF = a[0] - a[1] if a[0] > a[1] else a[1] - a[0]
        else:                                       # 빨간 불 반사광 %
            rL = rdL(0)[0]
            rR = rdR(0)[0]
            rF = rdF(0)[0]
        now = sw.time()
        if rL == pL and rR == pR and rF == pF and now - t_step < FORCE_MS:
            continue                                # 새 센서 값이 올 때만 제어
        pL, pR, pF = rL, rR, rF
        STAT[0] += 1
        dt = now - t_step
        t_step = now

        nL = (min(max(rL, bL), wL) - bL) * 100 // gL   # 0 = 검정 ~ 100 = 흰색
        nR = (min(max(rR, bR), wR) - bR) * 100 // gR
        nF = (min(max(rF, bF), wF) - bF) * 100 // gF

        # ---- 자동 정지 (연습용) ---- #
        if AUTO_STOP:
            if nL > W_NORM and nR > W_NORM and nF > W_NORM:
                if t_allw < 0:
                    t_allw = now
            else:
                t_allw = -1
            if nL < BLACK_MID and nR < BLACK_MID and nF < BLACK_MID:
                if t_alld < 0:
                    t_alld = now
            else:
                t_alld = -1
            if 30 < nL < W_NORM and 30 < nR < W_NORM and 30 < nF < W_NORM:
                if t_allg < 0:
                    t_allg = now
            else:
                t_allg = -1
            if ((t_allw >= 0 and now - t_allw > OFF_STOP_MS) or (t_alld >= 0 and now - t_alld > DARK_STOP_MS)
                    or (t_allg >= 0 and now - t_allg > GRAY_STOP_MS)):
                for m in LM + RM:
                    m.brake()
                print("AUTO_STOP", "흰색 계속" if t_allw >= 0 else ("검정 계속" if t_alld >= 0 else "회색 계속"), now)
                raise SystemExit
        sL = (ALPHA * nL + A_COMP * sL) // 1000
        sR = (ALPHA * nR + A_COMP * sR) // 1000
        sF = (ALPHA * nF + A_COMP * sF) // 1000

        # ---- 라인이 보이나: 좌우가 흰색이어도 가운데가 방금 검정을 봤으면 라인 위 ---- #
        pair_white = nL > W_NORM and nR > W_NORM
        if nF < BLACK_MID:
            t_ahead = now
            if armed:
                armed = False
                ev3.light.on(Color.GREEN)
        ahead = now - t_ahead <= AHEAD_HOLD_MS
        if not ahead and not pair_white and not armed:
            armed = True                            # 코너 예고: 앞이 하얘짐
            ev3.light.on(Color.RED)

        # ---- 오차: 세 센서로 라인 위치(mm, 좌 +)를 구함 ---- #
        dL = 100 - DFLOOR - sL
        dR = 100 - DFLOOR - sR
        dF = 100 - DFLOOR - sF
        if dL < 0:
            dL = 0
        if dR < 0:
            dR = 0
        if dF < 0:
            dF = 0
        if dL + dR > SIDE_SEE:                      # 옆 센서가 보면 세 센서 무게중심
            ysgn = 1 if dL > dR else -1
            ymm = (dL - dR) * SENSOR_GAP_MM // (dL + dF + dR)
        elif dF > SIDE_SEE:                         # 가운데만 보면 크기 = 가운데가 하얘진 정도, 방향 = 마지막 옆
            ymm = ysgn * CF_GAIN * (100 - DFLOOR - dF) // (100 - DFLOOR)
        else:
            ymm = ysgn * SENSOR_GAP_MM
        error = -ymm * 87 // 10                     # 옆 센서 하나가 완전 검정 = ±200
        if error > SIDE_ERR:
            side = 1
        elif error < -SIDE_ERR:
            side = -1

        if not pair_white or ahead:
            # ---- 라인 보임: 조향 = 오차 × 게인 + 회전 감쇠 ---- #
            t_lost = -1
            if was_off:
                was_off = False
                prev_steer = 0
            yr = yaw_rate()
            kp_s = KP_S
            kg = KG
            if KP2 > 0:                             # 한쪽으로 오래 돌수록(곡선) KP2·KG2 쪽으로
                c_ema += (yr * 1000 - c_ema) * dt // CURVE_TAU_MS
                f = (c_ema if c_ema > 0 else -c_ema) // CURVE_YAW   # 0~1000 = 곡선 정도
                if f > 1000:
                    f = 1000
                kp_s = KP_S + (KP2_S - KP_S) * f // 1000
                if KG2 >= 0:
                    kg = KG + (KG2 - KG) * f // 1000
            steer = kp_s * error // S
            if sL < BLACK_MID and sR < BLACK_MID:
                steer = 0                           # 교차로: 직진
            g = (kg * yr) // 1000
            if g > G_CLAMP:
                g = G_CLAMP
            elif g < -G_CLAMP:
                g = -G_CLAMP
            steer += g
            straight = -STR_YAW * 1000 < c_ema < STR_YAW * 1000
            if not straight:
                t_str0 = now                        # 곡선 중 → 직진 시간 다시 셈
            br = 0                                  # 직진 중 직각 신호: 가운데 + 한쪽 진한 검정, 반대쪽 흰색
            if straight and nF < BR_DARK:
                if nL < BR_DARK and nR > W_NORM:
                    br = -1
                elif nR < BR_DARK and nL > W_NORM:
                    br = 1
            if br != 0 and br == br_last:
                br_n += 1
            else:
                br_n = 1 if br != 0 else 0
            br_last = br
            if br_n >= 3:                           # 3번 연속일 때만
                if BR_ST:
                    steer = br * BR_ST              # 직각 쪽으로 미리 꺾기
                if BRK_MS and not brk_on and now - t_brk > 500 and now - t_str0 >= BRK_STR_MS:
                    brk_on = True                   # 긴 직선 끝 직각 → 브레이크
                    brk_dir = br
                    t_brk = now
            power = POWER
            last_good = steer
        else:
            # ---- 라인 놓침: 잠깐은 직전 조향, 그다음 기억한 쪽으로 강제 회전 ---- #
            t_str0 = now
            if t_lost < 0:
                t_lost = now
            if now - t_lost <= GAP_MS:
                steer = last_good
                power = POWER
            else:
                if not was_off:
                    was_off = True
                    ev3.light.on(Color.YELLOW)
                    bank_dir = side
                    bank_st = ST_BANK
                    ra_on = False
                    if ST_BANK_STR and -STR_YAW * 1000 < c_ema < STR_YAW * 1000:
                        bank_st = ST_BANK_STR       # 직진 중 놓침 = 직각 → 제자리 회전
                        ra_on = RA_YAW > 0
                steer = bank_dir * bank_st
                if ra_on:                           # 직각 회전이 너무 빠르면 넘은 만큼 덜 꺾음
                    rate = -bank_dir * yaw_rate()
                    if rate > RA_YAW:
                        steer = bank_dir * (bank_st - RA_KQ * (rate - RA_YAW) // 100)
                power = POWER

        delta = steer - prev_steer                  # 한 번에 너무 크게 바뀌지 않게
        if delta > MAX_STEER_DELTA:
            steer = prev_steer + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steer = prev_steer - MAX_STEER_DELTA
        prev_steer = steer

        if brk_on:                                  # 직각 브레이크: 안쪽 바퀴 −100, 바깥 바퀴 BRK_OUT
            if now - t_brk >= BRK_MS:
                brk_on = False
                BRK_CMD[0] = 0
                BRK_CMD[1] = 0
            else:
                if brk_dir < 0:
                    BRK_CMD[0] = -100
                    BRK_CMD[1] = BRK_OUT
                else:
                    BRK_CMD[0] = BRK_OUT
                    BRK_CMD[1] = -100
                power = 0                           # 블랙박스: 파워 0 = 브레이크 중
        if now - t_log >= LOG_MS:                   # 블랙박스 기록
            t_log = now
            j = li % LOG_N * 7
            x = rL >> sh
            LOG[j] = x if x < 255 else 255
            x = rR >> sh
            LOG[j + 1] = x if x < 255 else 255
            x = rF >> sh
            LOG[j + 2] = x if x < 255 else 255
            v = steer + 128
            LOG[j + 3] = 0 if v < 0 else (255 if v > 255 else v)
            LOG[j + 4] = power if 0 <= power <= 255 else 0
            LOG[j + 5] = ((1 if (not pair_white or ahead) else 0) | (2 if ahead else 0) | (4 if pair_white else 0)
                          | (8 if was_off else 0) | (16 if side > 0 else 0) | (32 if bank_dir > 0 else 0)
                          | (64 if armed else 0))
            v = yaw_rate() // 8 + 128
            LOG[j + 6] = 0 if v < 0 else (255 if v > 255 else v)
            li += 1
            STAT[1] = li
        steering_dc(steer, power)


# =============================== 메인 =============================== #
def line_trace(VALUE, YAW_ENC_K, POWER, KP, KP2, KG, ST_BANK, ST_BANK_STR, RA_YAW, BRK_MS, AUTO_STOP, BB_PRINT):
    # 주행 → 멈추면(정지 버튼·AUTO_STOP·오류) 모터 세우고 블랙박스 출력
    global YAW_K
    YAW_K = YAW_ENC_K
    wait(200)                   # 켜진 직후 센서 값이 자리 잡을 때까지
    print("회전 속도 = 모터 엔코더 (자이로 없음)")
    t_run = StopWatch()
    err = False
    try:
        run(VALUE, POWER, KP, KP2, KG, ST_BANK, ST_BANK_STR, RA_YAW, BRK_MS, AUTO_STOP)
    except Exception:
        err = True
        show("ERROR")           # 화면 ERROR + 빨간 불 = 오류로 멈춤 (내용은 PC 콘솔)
        ev3.light.on(Color.RED)
        raise
    finally:                    # 정지 버튼·AUTO_STOP·오류 모두 여기로
        BRK_CMD[0] = BRK_CMD[1] = 0
        for mo in LM + RM:      # 모터부터 세움
            try:
                mo.brake()
            except OSError:
                pass
        print("STAT,rule,steps,%d,ms,%d" % (STAT[0], t_run.time()))
        if BB_PRINT:
            if not err:
                show("LOG...")                      # 화면 LOG... = 기록 출력 중 = 아직 실행 중
            n = STAT[1]                             # 블랙박스: 정지 직전 2초 (sim/bb_parse.py로 해석)
            m = n if n < LOG_N else LOG_N
            print("BB,%d,%d,7,%s,new" % (LOG_MS, m, ",".join(str(x >> RUN_VAL[1]) for x in (RUN_VAL[0] or VALUE))))
            hx = "0123456789abcdef"
            s = ""
            for k in range(n - 1, n - 1 - m, -1):   # 최신 → 과거: 출력이 도중에 끊겨도 정지 직전이 남음
                j = k % LOG_N * 7
                for q in range(7):
                    b = LOG[j + q]
                    s += hx[b >> 4] + hx[b & 15]
                if len(s) >= 196:
                    print("BBD," + s)
                    s = ""
                    wait(30)                        # 출력이 밀려 잘리지 않게
            if s:
                print("BBD," + s)
            wait(200)
            if not err:
                show("DONE")


# ===================== ★ 맨 아래 한 줄만 바꾸세요 ★ ===================== #
#  VALUE        빛값 [흰, 검] × (좌, 우, 가운데) — ev3_value.py로 재서 나온 VALUE 줄(RGB 반사광 %)을 붙여 넣기
#               (고급 설정 RED_LIGHT = True면 이 값 대신 VALUE_RED·VALUE0을 씀)
#  YAW_ENC_K    모터 각도 → 몸 회전 환산. 기어 20:12 그대로면 245, 기어를 2배로 올렸으면 490
#  POWER        속도 (0~100). 기본 100. 낮추면 회전도 같이 약해진다
#  KP           직선에서 조향 세기. 크면 라인을 세게 쫓고(흔들림↑), 작으면 부드럽지만 늦게 쫓는다
#  KP2          곡선(원·급커브)에서 조향 세기. 곡선 바깥으로 밀려나면 ↑
#  KG           흔들림 잡기. 직선에서 좌우로 출렁이면 ↑, 곡선에서 덜 돌면 ↓
#  ST_BANK      곡선에서 라인을 놓쳤을 때 강제 회전 세기 (0~100). 원에서 너무 휘두르면 ↓, 급커브에서 밀려나면 ↑
#  ST_BANK_STR  직진하다 라인을 놓쳤을 때(직각) 회전 세기. 100 = 제자리 회전
#  RA_YAW       직각에서 도는 빠르기 상한 (°/s). 직각을 지나쳐 너무 돌면 ↓, 늦게 돌면 ↑
#  BRK_MS       긴 직선 끝 직각에서 브레이크 거는 시간 (ms). 0 = 브레이크 끔
#  AUTO_STOP    연습 = True (라인을 오래 잃으면 스스로 멈춤) / 대회 = False
#  BB_PRINT     멈출 때 블랙박스(마지막 2초) 출력. False면 멈추자마자 바로 끝남
line_trace(VALUE=[28, 3, 31, 3, 28, 4], YAW_ENC_K=245, POWER=100, KP=0.088, KP2=0.433, KG=209, ST_BANK=85, ST_BANK_STR=100, RA_YAW=300, BRK_MS=50, AUTO_STOP=True, BB_PRINT=True)
