# =====================================================================
#  MRC 라인트레이싱 v9 ── 클린시트 설계 (라인팔로잉 정석 + 세션 검증 기법)
# ---------------------------------------------------------------------
#  [설계 원리 — 왜 이 구조인가]
#
#  1) ★가중 무게중심 위치 추정 (라인어레이 표준 기법)
#     기존 (s1+s2)-(s3+s4)는 '좌우 밝기 차이'지 위치가 아니다 → 비선형,
#     게인이 위치에 따라 다르게 먹음. 무게중심 pos = Σ(어두움×센서위치)/Σ어두움
#     은 라인 위치를 0.1mm 단위 연속값으로 줘서 PD가 전 구간 일관 동작.
#
#  2) ★센서별 정규화 (v5에서 검증) — 감도 비대칭(레인지 40vs23) 상쇄.
#     마커에서 가짜 오차로 홱 꺾이는 실기 버그의 근본 해결.
#
#  3) ★엣지 타깃 (이 세션 최대 발견) — 라인 20mm < 센서간격 25mm라
#     정중앙은 무신호 데드밴드. 타깃을 라인 가장자리(센서 하나 걸침)로
#     옮겨 연속 신호 확보. TARGET 부호로 좌/우 엣지 선택.
#
#  4) ★프레임 동기 (v4에서 검증) — 센서 새 프레임에서만 PID 1스텝.
#     중복 프레임 미분 오염 제거.
#
#  5) ★위치 기억 비례 복구 — 이탈 시 마지막 '위치'(연속값)를 기억,
#     그 크기에 비례해 뱅킹. (기존의 이진 방향기억 + 고정 뱅킹보다 부드러움)
#     중앙 근처에서 사라짐 = 센서 틈 → 직진 유지 (v4 검증).
#
#  6) ★교차점/마커 — 어두운 센서 2개 이상 & 위치 급변 → 직진 유지.
#
#  7) ★코너 피벗 (v6 검증, 시간순서 판별) — 바깥 센서 진검정 직후
#     라인 소실 '지속' → 그쪽으로 피벗. 교차선은 중앙 라인이 유지되므로
#     오발동 없음. WHITE_CONF로 틈 깜빡임 필터 (실기 피드백 반영: 50).
#
#  [감속 없음] 순항/피벗/복구 전부 POWER 고정 (사용자 원칙).
#  [정수 연산] 마이크로파이썬 성능 고려, 전부 정수 스케일.
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
c1 = PUPDevice(Port.A)    # 좌측부터 1
c2 = PUPDevice(Port.C)
c3 = PUPDevice(Port.E)
c4 = PUPDevice(Port.D)

# ===================== 튜닝 파라미터 ===================== #
# --- 캘리브레이션 (흰/검 기준값 — 라인스캔 실측으로 갱신할 것) --- #
VALUE = [45, 5, 42, 5, 31, 3, 25, 2]   # [w1,b1, w2,b2, w3,b3, w4,b4]

# --- 속도 (무감속 원칙: 전 상태 공통) --- #
POWER = 100

# --- 위치 PD (pos 단위 = 0.1mm, 조향 ±100) --- #
#  풀스케일: 라인이 바깥 센서(±37.5mm) 아래일 때 pos=±375
#  kp=0.20이면 그때 조향 75. 떨리면 KD부터 ↓, 코너 놓치면 KP ↑
KP = 0.20      # 조향/0.1mm
KD = 1.8       # 조향/(0.1mm/프레임)
TARGET = -125  # ★엣지 타깃 (0.1mm): -125 = 라인을 3번 센서 밑에(우엣지)
               #  +125로 바꾸면 좌엣지. 0은 데드밴드라 금지.

# --- 필터/슬루 --- #
EMA_A    = 600   # /1000 센서 평활 (↓=부드러움)
D_FILT   = 250   # /1000 미분 평활 (↓=부드러움)
SLEW     = 25    # 1프레임당 조향 변화 상한

# --- 라인 검출/이탈 --- #
SUM_MIN  = 25    # Σ어두움이 이보다 작으면 '라인 안 보임' (0~400 스케일)
GAP_HOLD = 12    # 짧은 소실(틈) 동안 직전 조향 유지할 프레임수
CENTER_LOST = 160  # |마지막 pos-TARGET| < 이 값이면 중앙 소실 → 직진 유지
BANK_K   = 30    # 엣지 소실 뱅킹: 조향 = ±(BANK_K + |pos|//8), 상한 95

# --- 교차점/마커 --- #
CROSS_SUM = 180  # Σ어두움이 이보다 크면(2개 이상 진검정) 교차 → 직진

# --- 코너 피벗 (v6 시간순서 판별) --- #
CORNER_DARK = 75   # 바깥 센서 어두움(0~100)이 이보다 크면 '진검정'
MEM_FRAMES  = 100  # 코너 후보 기억 유지
WHITE_CONF  = 50   # 피벗 진입 전 전체소실 연속 확인 (~38ms)
TURN_ST     = 100  # 피벗 조향
REACQUIRE   = 45   # 중앙 어두움이 이보다 크면 재포착
TURN_MIN    = 60   # 최소 피벗 프레임 (조기 종료 방지)
TURN_MAX    = 2000 # 피벗 포기
# ========================================================= #

E_S = 1000
A_C = E_S - EMA_A
D_C = E_S - D_FILT
WH = (VALUE[0], VALUE[2], VALUE[4], VALUE[6])
BK = (VALUE[1], VALUE[3], VALUE[5], VALUE[7])
RG = (WH[0]-BK[0], WH[1]-BK[1], WH[2]-BK[2], WH[3]-BK[3])
XPOS = (375, 125, -125, -375)   # 센서 좌표 (0.1mm, 좌=+)


def steering_dc(steering, power):
    if steering > 100:
        steering = 100
    elif steering < -100:
        steering = -100
    slope = power * 2 / 100
    if steering < 0:
        lm.dc(slope * steering + power)
        rm.dc(power)
    else:
        lm.dc(power)
        rm.dc(-slope * steering + power)


def dark4(r1, r2, r3, r4):
    # 정규화 '어두움' 0(흰)~100(검)
    d1 = 100 - (min(max(r1, BK[0]), WH[0]) - BK[0]) * 100 // RG[0]
    d2 = 100 - (min(max(r2, BK[1]), WH[1]) - BK[1]) * 100 // RG[1]
    d3 = 100 - (min(max(r3, BK[2]), WH[2]) - BK[2]) * 100 // RG[2]
    d4 = 100 - (min(max(r4, BK[3]), WH[3]) - BK[3]) * 100 // RG[3]
    return d1, d2, d3, d4


def line_pos(d1, d2, d3, d4):
    # 가중 무게중심 (0.1mm). 라인 없으면 None 의미로 total 반환해 판단.
    total = d1 + d2 + d3 + d4
    if total < SUM_MIN:
        return None, total
    pos = (d1*XPOS[0] + d2*XPOS[1] + d3*XPOS[2] + d4*XPOS[3]) // total
    return pos, total


def pid_control():
    last_err = 0
    d_filt = 0
    prev_st = 0
    gap = 0
    last_pos = TARGET       # 마지막 유효 위치 기억
    # 코너 피벗 상태
    turning = False
    turn_dir = 0
    mem_dir = -1
    mem_age = 0
    white_cnt = 0
    turn_frames = 0
    # EMA 상태
    r1 = c1.read(mode=0)[0]; r2 = c2.read(mode=0)[0]
    r3 = c3.read(mode=0)[0]; r4 = c4.read(mode=0)[0]
    raw_prev = (r1, r2, r3, r4)
    ps1, ps2, ps3, ps4 = dark4(r1, r2, r3, r4)

    while True:
        r1 = c1.read(mode=0)[0]; r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]; r4 = c4.read(mode=0)[0]
        raw = (r1, r2, r3, r4)
        if raw == raw_prev:          # ★프레임 동기
            continue
        raw_prev = raw
        n1, n2, n3, n4 = dark4(r1, r2, r3, r4)

        # ── 코너 피벗 상태기계 ── #
        if turning:
            ps1, ps2, ps3, ps4 = n1, n2, n3, n4
            turn_frames += 1
            if turn_frames >= TURN_MIN and (n2 > REACQUIRE or n3 > REACQUIRE):
                turning = False
                last_err = 0; d_filt = 0; prev_st = 0
                gap = 0; mem_dir = -1
                last_pos = TARGET
                steering_dc(0, POWER)
                continue
            if turn_frames > TURN_MAX:
                turning = False
                mem_dir = -1
            else:
                st = -TURN_ST if turn_dir == 0 else TURN_ST
                prev_st = st
                steering_dc(st, POWER)
                continue

        # 코너 후보 기억 (바깥 하나만 진검정; 양쪽 동시=마커 → 무시)
        if n1 > CORNER_DARK and n4 < 40:
            mem_dir = 0; mem_age = 0
        elif n4 > CORNER_DARK and n1 < 40:
            mem_dir = 1; mem_age = 0
        elif mem_dir >= 0:
            mem_age += 1
            if mem_age > MEM_FRAMES:
                mem_dir = -1

        # ── EMA 평활 ── #
        s1 = (EMA_A * n1 + A_C * ps1) // E_S
        s2 = (EMA_A * n2 + A_C * ps2) // E_S
        s3 = (EMA_A * n3 + A_C * ps3) // E_S
        s4 = (EMA_A * n4 + A_C * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        pos, total = line_pos(s1, s2, s3, s4)

        # 코너 확정: 기억 신선 + 소실 지속
        if pos is None and mem_dir >= 0:
            white_cnt += 1
        else:
            white_cnt = 0
        if white_cnt >= WHITE_CONF:
            white_cnt = 0
            turning = True
            turn_dir = mem_dir
            mem_dir = -1
            turn_frames = 0
            st = -TURN_ST if turn_dir == 0 else TURN_ST
            prev_st = st
            steering_dc(st, POWER)
            continue

        if pos is not None:
            # ── 정상 추종: 위치 PD ── #
            gap = 0
            last_pos = pos
            if total > CROSS_SUM:
                st = 0                    # ★교차점/마커: 직진
                last_err = 0; d_filt = 0
            else:
                err = pos - TARGET        # +면 라인이 타깃보다 왼쪽
                diff = err - last_err
                d_filt = (D_FILT * diff + D_C * d_filt) // E_S
                # 라인이 왼쪽(+err) → 좌회전(음수 조향)
                st = -int(KP * err + KD * d_filt)
                last_err = err
        else:
            # ── 라인 소실 ── #
            gap += 1
            if gap <= GAP_HOLD:
                st = prev_st              # 짧은 틈: 유지
            elif abs(last_pos - TARGET) < CENTER_LOST:
                st = 0                    # 중앙 소실 = 센서 틈: 직진
            else:
                # 엣지 소실: 마지막 위치 방향·크기에 비례해 뱅킹
                mag = BANK_K + abs(last_pos) // 8
                if mag > 95:
                    mag = 95
                st = mag if last_pos < 0 else -mag

        # 슬루 제한
        d = st - prev_st
        if d > SLEW:
            st = prev_st + SLEW
        elif d < -SLEW:
            st = prev_st - SLEW
        prev_st = st

        steering_dc(st, POWER)


# --------------------------- 메인 --------------------------- #
hub.speaker.beep(700, 80)
wait(300)
pid_control()
