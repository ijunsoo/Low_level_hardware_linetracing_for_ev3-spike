from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait, StopWatch

# =============================================================
#  ev3_lf3_rule_e8: EV3 3센서 라인트레이싱 — SPIKE E8(spike_lf3_rule_e8_130.py)의 EV3판 (2026-10-24)
#
#  로직·상수 = SPIKE E8 (실기 17cm에서 제일 좋았던 E7 + GAP_MS 24→2). 입출력만 EV3용:
#   - 로봇(사용자 사진 2026-10-24): 컬러센서 3개 일렬, 렌즈가 바퀴축 앞 ≈17cm·간격 ≈23mm,
#     뒤 2바퀴(≈80mm) 윤거 ≈205mm·기어 20:12, 앞 볼 캐스터 — SPIKE 로봇과 같은 치수
#   - 포트: 컬러 S1 좌·S2 가운데·S3 우 / 모터 A·B 좌(정방향 = 전진)·C·D 우(역방향 = 전진)
#   - ★자이로센서 없음 → 요레이트(회전 속도)를 좌우 모터 엔코더 각도 차(최근 ≈16ms)로 계산 (yaw_rate)
#     계수 YAW_ENC_K는 SPIKE 실측 회전 권한(제자리 회전 ≈500°/s, 미끄러짐 포함 유효 윤거 272mm)에 맞춤
#   - Pybricks EV3 펌웨어(v4 네이티브)·EV3 MicroPython v2.0 공통 API만 씀
#   - 센서·모터 읽기는 int()로 감쌈 (EV3 펌웨어에 따라 reflection()이 실수로 나옴 → 정수 연산·블랙박스 오류 방지)
#  처음엔 ev3_value.py로 빛값(VALUE)을 잴 것. 블랙박스(BB/BBD)는 sim/bb_parse.py로 해석 (SPIKE판과 같은 형식)
#  ★E8R (2026-10-24, 사용자 '2배 가속했더니 직진 뒤 바로 오는 직각을 늦게 알아챔'): E8에서 두 가지만 바꿈
#   1) 직진 중(곡선 평균 회전 < 40°/s) 라인을 놓치면 뱅킹 85 대신 100(제자리 회전) — 곡선 중엔 85 그대로
#   2) 가운데 센서 유지 AHEAD_HOLD_MS 16→8ms (속도 2배라 같은 시간에 2배 멀리 감)
#   시뮬(17cm, 기어 2배, 월드 32): 직각 트랙 6→27/32 완주(최대 이탈 195→178mm), main 32/32(2.79→3.02s),
#   hard 30→32/32. 원래 속도에선 셋 다 32/32지만 5% 느려짐(직각 4.71→4.86s) — 2배 속도용.
#   기어를 올려 모터 반응까지 2배 느려졌다면(시뮬 τ 0.2) 어떤 설정도 첫 직각 1/32 → 그땐 기어를 조금 낮출 것
#  ★E8R2 (2026-10-24, 실기 '직진 뒤 직각에서 너무 멀리 가서 75도쯤으로 들어와 이상해짐'): 2배 기어에선 제자리 회전이
#   ≈900°/s까지 붙어 새 라인을 보고도 못 멈추고 180° 가까이 돎(시뮬 재현) → 직진 중 이탈 뱅킹만 회전 속도가 350°/s를
#   넘으면 넘은 만큼 반대로 조향 (RA_YAW 350·RA_KQ 50) — 새 라인을 가로지르지 않고 바깥에서 완만하게 붙음.
#   시뮬(2배 속도, 실기처럼 크게 돌아와도 계속 달리게 이탈 판정 450mm): 직각 4.38→3.81s, main 3.02→2.38s,
#   hard 2.04→1.76s, 모두 32/32. 모터 반응 1.5배 느려도 같은 경향(직각 5.41→4.76s). 원래 속도에선 거의 같음
#  ★E8R3 (2026-10-24, 사용자 '조금만 더 완만하게'): RA_YAW 350→300, RA_KQ 50→40 — 직각 회전을 조금 더 느리고 부드럽게.
#   시뮬(2배 속도, 이탈 판정 450mm): 직각 3.81→3.70s, 모터 1.5배 느릴 때 4.76→4.34s, main·hard 같음, 모두 32/32.
#   기본 판정(200mm) 직각 27/32 그대로. 첫 회전 세기 ST_BANK_STR 90은 처음에 더 멀리 나가서 손해라 100 유지
#  ★E8R4 (2026-10-24, 사용자 '더 많이 벗어나지 않도록'): 가지 선회전 BR_ST 100 + 가운데 유지 AHEAD_HOLD_MS 8→4
#   시뮬(2배 속도, 월드 64): 직각에서 센서가 라인에서 벗어나는 최대 거리 187→163mm, 기본 판정 직각 완주 52→58/64,
#   main·hard 64/64 그대로, 시간 같음. 원래 속도: 최대 108→91mm, 직각 63/64(트랙 끝 1회)·main·hard 64/64.
#   (아래 E8R5에서 속도 기준으로 끝내는 직각 브레이크를 넣음 — 고정 시간 브레이크보다 덜 느려짐)
#  ★E8R5 (2026-10-24, 사용자 '직진 뒤 직각 구간엔 정지를 강하게'): 직각 브레이크 — '감속 없음' 원칙에서 이것만 예외.
#   직진 중 가지(옆 센서가 꺾이는 라인을 밟음)를 보는 즉시 양쪽 바퀴 −100(역회전)으로 세게 서고, 모터 평균 속도가
#   350°/s 아래로 떨어지면(최대 200ms) 바로 풀어 평소대로 회전. 곡선에선 가지가 안 보여 거의 안 걸림.
#   시뮬(2배 속도, 월드 48): 직각 최대 이탈 166→125mm, 기본 판정 직각 44→48/48, 직각 시간 3.60→3.57s,
#   main 2.36→2.43s·hard 1.75→1.81s. 모터 1.5배 느려도 200→158mm. 원래 속도에선 93→68mm지만 직각 0.3s 느려짐.
#   블랙박스에서 파워 0 = 브레이크 중
#   ★EV3: 기어를 2배로 올렸다면 YAW_ENC_K도 2배(245→490)로 — 엔코더 요레이트 계산에 기어비가 들어감
#  LED: 빨강 = 코너 예고, 초록 = 앞 라인 복귀, 노랑 = 이탈 → 뱅킹
# =============================================================

ev3 = EV3Brick()
LM = (Motor(Port.A), Motor(Port.B))                     # 좌 구동 (정방향 = 전진)
RM = (Motor(Port.C, Direction.COUNTERCLOCKWISE), Motor(Port.D, Direction.COUNTERCLOCKWISE))   # 우 구동 (역방향 = 전진)
# ↑ 출발하자마자 제자리에서 돌거나 뒤로 가면 그쪽 모터 Direction을 반대로 바꿀 것
cL = ColorSensor(Port.S1)      # 좌 (바퀴축 앞 ≈170mm, 측방 +23)
cF = ColorSensor(Port.S2)      # 가운데
cR = ColorSensor(Port.S3)      # 우 (측방 −23)
YAW_ENC_K = 245         # 엔코더 요레이트 ×1000: 기어 20/12 × 바퀴 80mm ÷ (2 × 유효 윤거 272mm)
                        #   = 좌우 모터 축 속도 차(°/s) → 몸 회전(°/s). 바퀴·기어·윤거가 바뀌면 다시 계산

# ------------- 튜닝 파라미터 (한 줄에 하나 — 시뮬 스윕이 이 줄들을 바꿔 넣음) ------------- #
# ★E8 상수 (SPIKE spike_lf3_rule_e8_130.py와 같음): 직선은 약한 조향 KP 0.088 + 강한 회전 감쇠 KG 209,
#   오래 한쪽으로 도는 곡선에서만 KP2 0.433 (곡선 게인 스케줄링), 라인 놓치면 2ms 만에 뱅킹 85 (GAP_MS 2)
# ★파워 100 고정 = 감속 없음 (사용자 방침). ★실기 루프 속도(734 제어/s)·기어 20:12·회전 지연(모터/자이로
#   0~35ms 무작위)에 맞춘 시뮬에서 160설정 탐색 → D1: 이탈하면 조향 96(≈제자리 회전)으로 강제 턴, 단 옆 센서가
#   라인 가장자리(W_NORM 85%)를 보는 즉시 멈춤 (이 로봇은 멈춘 뒤에도 30~40ms 더 돌아서, v11처럼 가운데를
#   기다리면 지나침 — 시뮬에서 v11식 PIVOT은 12월드 평균 0~2 완주). 사진 12/12·직각 12/12, 놓침 13회/바퀴
# ★E2 (2026-10-17 실기 D1 피드백 "작은 폭으로 자주 왕복"): 코너 처리는 D1 그대로, 직선은 조용하게 —
#   조향 변화 제한 56→15·미분 KD 1.6→0(계단식 센서값을 미분하면 조향이 떪)·KP 0.616→0.45·EMA 875→800·KG 256→180.
#   시뮬(실기 루프 734/s): 사진·직각 12/12 유지, 스텝당 조향 변화 8.8→5.8
# ★E3 (2026-10-17 실기 "미세하게 자주 떨림·반대로 턴", 빛값 34/22/21로 대비 절반): 좌우 센서 90% 넘으면 흰색으로 보고(WHITE_DB)
#   EMA 800→600. 시뮬(노이즈 0.5~1.5카운트): 사진 9→13/16, 반대 뱅킹 31→15·23→12, 조향 떨림 2.7→1.3 / 노이즈 작을 때도 완주 같고 떨림 −35~65%
#   → 실기에서 E2보다 나쁨(중간에 멈추다시피 함) → 기본값 E2로 되돌림 (WHITE_DB 0, ALPHA 800)
VALUE = [28, 3, 31, 3, 28, 4]   # [흰, 검] × (좌, 우, 가운데) — 2026-10-24 사용자 EV3에서 ev3_value.py로 잰 값 (매트·조명 바뀌면 다시)
AUTO_CAL = True         # 출발 시 정지 자세(가운데 센서=라인 위, 좌우=흰 바닥)에서 흰색을 다시 재 VALUE 보정
                        # (매트가 VALUE 잰 곳보다 어두우면 이탈을 못 느껴 직각에서 직진하던 문제 방지)
POWER = 100
KP = 0.088
KD = 0.0
KG = 209                # 자이로 댐핑 ×1000 (좌회전 + 요레이트에 곱해 조향에 더함)
G_CLAMP = 71            # 자이로 기여 상한 (재포착 스핀 폭주 차단, v14)
ALPHA = 950             # 센서 EMA ×1000
D_FILTER = 259          # 미분 저역필터 ×1000
MAX_STEER_DELTA = 79    # 제어 1스텝당 조향 변화 상한
W_NORM = 81             # 흰색 문턱 (정규화 0~100)
BLACK_MID = 51          # 검정 문턱
SIDE_ERR = 60          # 오차가 이보다 크게 기울면 '라인이 그쪽'으로 기억 (직선 위빙 ±50보다 크게)
EDGE = 0                # ★엣지 추종: 0 = 끔(좌우 센서 차이만 → 라인 20mm < 센서 간격 25mm라 가운데 ±12mm에선 오차 0,
                        #   옆 센서에 닿으면 갑자기 꺾음 = 작은 폭으로 자주 왕복). −1 = 가운데 센서로 라인 왼쪽 가장자리,
                        #   +1 = 오른쪽 가장자리를 따라감 → 센서값이 연속으로 변해 부드러운 비례 제어
EDGE_TGT = 50           # 가장자리 목표: 가운데 센서 정규화 % (50 = 반쯤 걸침)
EDGE_K = 2              # 가운데 센서 오차 배율
GAP_MS = 2              # 라인이 안 보여도 이 시간은 직전 조향 유지
AHEAD_HOLD_MS = 4      # (E8R: 16→8, E8R4: →4) 센티넬이 검정을 본 뒤 이 시간까지는 '앞에 라인 있음'
LOCK_MS = 0             # 코너 예고 뒤 처음 기운 방향을 이 시간 동안 고정 (0 = 끔 — 촘촘한 지그재그에선 역효과)
SEARCH_DEG = 0        # 뱅킹으로 이만큼 돌아도 라인이 없으면 반대쪽으로 (0 = 끔)
USE_SENTINEL = True     # False면 센티넬 무시 (= 페어만, v16식 이탈판정)
ST_BANK = 85            # 이탈 시 뱅킹 조향
PIVOT_ST = 0           # ★v11식 강제 턴: 옆 센서가 진검정을 밟은 뒤 세 센서가 흰색이 되는 즉시 그쪽으로 풀파워 피벗 (0 = 끔)
PIVOT_MIN_MS = 56       # 피벗 최소 시간 (조기 종료 방지)
PIVOT_MAX_MS = 3000     # 이만큼 돌아도 못 찾으면 포기
PIVOT_MEM_MS = 152      # v11 C_MEM: 옆 센서가 진한 검정(30% 미만)을 밟은 뒤 이 시간 안에 이탈해야 강제 턴
                        # (직선에서 라인이 센서 사이에 걸려 잠깐 흰흰흰이 되는 건 강제 턴 안 함 → 일반 뱅킹)
                        # 끝내는 조건: 최소 시간 뒤 '가운데' 센서가 검정을 찾을 때만 (옆 센서는 무시) → PD 리셋 복귀
WHITE_DB = 0            # 좌우 센서가 이 % 넘으면 흰색(100)으로 봄 — 흰 바닥 ±1카운트 떨림이 조향 떨림이 되는 것 방지 (0 = 끔)
SIDE_TTL_MS = 0         # 옆 센서가 라인을 본 지 이보다 오래면 '기억 방향'을 버리고, 최근 회전 방향(자이로)으로 라인 쪽을 판단 (0 = 끔)
                        # (가운데만 서서히 라인에서 빠질 땐 좌우가 라인을 못 봐서 기억이 오래전 값 → 반대로 뱅킹하던 문제)
YAW_TAU_MS = 150        # 위 판단에 쓰는 자이로 평균 시간
ERR_MODE = 1            # 오차 계산: 0 = 좌우 센서 차이(E2) / 1 = 세 센서로 라인 위치(mm)를 끊김 없이 추정
                        #   (좌우가 흰색이어도 가운데 센서가 얼마나 하얘졌는지로 벗어난 정도를 계속 앎 → 작은 폭 잦은 왕복 줄임)
SENSOR_GAP_MM = 23      # 센서 간격 (실측)
CF_GAIN = 4            # 가운데 센서만 라인을 볼 때: 가운데가 완전히 하얘지면 이 mm만큼 벗어난 것으로 봄
SIDE_SEE = 16            # 진하기(100−정규화) 합이 이보다 크면 '라인 보임'
DFLOOR = 25             # 진하기에서 빼는 바닥값 — 흰 바닥 ±1카운트 떨림(빛값 대비가 작아 1카운트 ≈5%)을 라인으로 착각하지 않게
KSP = 0                 # ★지연 보상(스미스 예측기) ×1000: 조향 명령 후 아직 몸에 안 나타난 회전을 자이로 값에 미리 더함 (0 = 끔)
DELAY_MS = 29           # 조향 명령 → 회전 시작까지 순수 지연 (실기 블랙박스 24~40ms)
YAW_PER_STEER = 6       # 파워 100에서 조향 1당 정상상태 요레이트 °/s (제자리 회전 ≈500°/s)
TAU_MS = 89            # 모터 응답 시간상수 (실측 ≈0.1s)
KP2 = 0.433               # ★곡선 게인 스케줄링: 한쪽으로 오래 도는 중(원형·급곡선)일수록 KP를 이 값까지 올림 (0 = 끔)
                        #   직선의 좌우 흔들림은 회전 방향이 계속 바뀌어 평균이 0 → 직선은 약한 KP 그대로
KG2 = 216                # 곡선일 때 자이로 댐핑 (-1 = KG 그대로)
CURVE_YAW = 153         # 부호 있는 요레이트 평균이 이 값(°/s)이면 곡선 게인 100%
CURVE_TAU_MS = 119      # 그 평균을 내는 시간
BANK_KG = 0             # 뱅킹 중 자이로 감쇠 ×1000: 회전이 빨라질수록 뱅킹 조향을 줄여 회전속도 제한
                        # (구동 지연 때문에 빠르게 돌다 라인을 찾으면 지나쳐 반대로 뱅킹 → 좌우 왕복, 실기 블랙박스)
CORNER_DEG = 0          # ★직각 회전 고정: '한쪽+가운데 동시 검정, 반대쪽 흰색'(가지)이 CORNER_CONFIRM_MS 유지되면
                        # 그쪽으로 제자리 회전하고, 자이로로 이 각도를 돌기 전엔 멈추지 않음 (0 = 끔)
                        # (실기: 15°쯤 돌았을 때 가운데 센서가 새 라인을 비스듬히 밟아 '정렬'로 오인 → 직진)
CORNER_ARM_MS = 150     # 가지 패턴(한쪽+가운데 검정, 반대쪽 흰)을 본 뒤 이 시간 안에 세 센서가 모두 흰색이 되면
                        # (= 원래 라인이 끝남 = 직각) 그 즉시 가지 쪽으로 회전 고정. 교차선은 지나도 가운데가
                        # 원래 라인 위라 흰흰흰이 안 됨 → 무시. 세 센서 모두 검정(교차로)이면 기억 지움
CORNER_MAX = 150        # 이만큼 돌아도 라인을 못 찾으면 해제
CORNER_YAW = 150        # 이미 그쪽으로 이보다 빠르게 도는 중(뱅킹·복구 회전)이면 직각으로 보지 않음 (°/s)
CORNER_ST = 100         # 회전 조향 (100 = 제자리 회전)
STR_YAW = 40            # ★'직진 중' 판단: 곡선 평균 회전 |c_ema| (KP2 > 0일 때 계산)가 이 값(°/s)보다 작으면 직진
ST_BANK_STR = 100       # ★직진 중 라인을 놓쳤을 때(= 직선 끝 직각) 뱅킹 조향 (0 = ST_BANK와 같음, 100 = 제자리 회전)
                        #   곡선 중에 놓쳤을 땐 ST_BANK 85 그대로 → 곡선은 지금처럼, 빠른 직선 뒤 직각만 세게
RA_YAW = 300              # ★직각 회전 속도 상한 °/s: 직진 중 이탈 뱅킹에서 이보다 빨리 돌면 넘은 만큼 반대로 조향 (0 = 끔)
                        #   (2배 기어면 제자리 회전이 ≈900°/s까지 붙어 새 라인을 보고도 못 멈추고 180° 가까이 돎)
RA_KQ = 40              # 넘은 속도 1°/s당 조향 감소 ×100 (40 = 0.4)
BR_ST = 100             # ★가지 선회전: 직진 중 '가지'(한쪽+가운데 검정, 반대쪽 흰 = 직각 꺾이는 라인을 옆 센서가 밟음)를
                        #   보면 라인이 끝나기 전에 그쪽으로 이 조향으로 미리 꺾기 시작 (0 = 끔)
                        #   마커·교차선에서 순간 휙 꺾이면 60으로 낮추거나 0으로
BRK_MS = 200              # ★직각 브레이크(ms): 직진 중 가지(= 직각 신호)를 보면 이 시간 동안 세게 멈춤 (0 = 끔)
                        #   (사용자 2026-10-24: '감속 없음' 원칙에서 직진 뒤 직각만 예외로 허용)
BRK_V = 350               # 바퀴 모터 평균 속도(°/s)가 이 아래로 떨어지면 브레이크를 일찍 끝냄 (0 = 시간만)
BRK_OUT = -100          # 브레이크 중 바깥 바퀴 파워 (−100 = 양쪽 다 역회전, 0 = 안쪽만 역회전하며 미리 돌기)
BRK_AT = 0              # 0 = 가지를 보는 즉시 / 1 = 직진 중 라인을 놓친 순간(세 센서 흰색)
RECOVER_POWER = 100      # 뱅킹 파워
SPD_K10 = 0             # 조향비례 감속 ×10 (0 = 끔)
MIN_POWER = 100          # 조향비례 감속 하한
PB_MS = 0             # 코너 예고 시 사전감속 시간 (0 = 끔)
PB_POWER = 100           # 사전감속 파워
FORCE_MS = 2            # 센서값이 안 변해도 이 간격마다 한 번은 제어


BRK_CMD = [0, 0]        # 브레이크 중 [왼쪽, 오른쪽] 듀티 (둘 다 0이면 평소 조향)


def steering_dc(steering, power):
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


encL, encR = LM[0], RM[0]   # 요레이트 계산용 (좌·우 첫 모터 엔코더)
_YH = [[0, 0, 0] for _ in range(5)]   # 4ms마다 저장한 [시각, 좌 각도, 우 각도] (≈16~20ms치)
_YS = [0, -100]                       # [다음 저장 칸, 마지막 저장 시각]
_YW = StopWatch()


def yaw_rate():
    # 좌회전 + °/s (자이로 대신): 최근 ≈16ms 동안 (우 − 좌) 모터 각도 변화 × YAW_ENC_K
    #   Motor.speed()는 펌웨어 기본이 100ms 평균이라 ≈50ms 늦음 → 회전 감쇠가 늦게 걸려 오히려 흔들림 (시뮬 확인)
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
    return ((aR - o[2]) - (aL - o[1])) * YAW_ENC_K // dt


def fwd_speed():
    # 좌우 모터 평균 속도 °/s (브레이크 해제 판단용, 마지막 저장값 기준)
    i = _YS[0]
    n = _YH[i - 1 if i > 0 else 4]
    o = _YH[i]
    dt = n[0] - o[0]
    if dt <= 0:
        return 0
    return ((n[1] - o[1]) + (n[2] - o[2])) * 500 // dt


STAT = [0, 0]           # [제어 스텝 수, 블랙박스 기록 수] — 정지 버튼을 누르면 STAT 줄로 출력
LOG_MS = 4              # 블랙박스: 이 간격으로 기록, 마지막 LOG_N개(2초)를 정지 시 BBD 줄로 출력
LOG_N = 1000
AUTO_STOP = True        # 시험용: 세 센서가 흰색만 OFF_STOP_MS 넘게 또는 검정만 DARK_STOP_MS 넘게 보이면
                        # 스스로 멈추고 블랙박스 출력 (정지 버튼보다 출력이 안 잘림). 대회 땐 False
OFF_STOP_MS = 800
DARK_STOP_MS = 300
GRAY_STOP_MS = 600      # 세 센서 모두 진한 검정(30%)도 흰색(W_NORM)도 아닌 회색이 이만큼 지속 (매트 밖)
LOG = bytearray(LOG_N * 7)   # [생값 좌·우·가운데, 조향+128, 파워, 상태비트, 요레이트/8+128]


def run():
    S = 100000
    KP_S = round(KP * S)
    KD_S = round(KD * S)
    KP2_S = round(KP2 * S)
    A_COMP = 1000 - ALPHA
    D_COMP = 1000 - D_FILTER
    wL, bL, wR, bR, wF, bF = VALUE
    t_rdy = StopWatch()                             # ★센서 준비 대기: 0만 읽히는 동안은 출발하지 않음
    waited = False                                  #   (실기 2026-10-24: SPIKE·EV3 모두 0이 5초 넘게 이어진 회차 → 출발 즉시 '검정 계속' 정지)
    t_kick = 1000
    while True:
        a, b, c = (int(cL.reflection()), int(cR.reflection()), int(cF.reflection()))
        if a + b > 4:                               # 좌우 센서가 바닥 값을 읽으면 준비됨 (검정 위여도 3+3 > 4)
            break
        waited = True
        if t_rdy.time() > t_kick:                   # 1초마다 모드를 한 번 바꿨다 돌려 센서를 깨움
            cL.ambient()
            cR.ambient()
            cF.ambient()
            if t_kick == 3000:
                print("센서 값이 계속 0 — 들어올 때까지 기다림 (센서 케이블·포트 S1 좌·S2 가운데·S3 우 확인)")
            t_kick += 1000
        wait(10)
    if waited:
        wait(100)                                   # 첫 값 직후의 오래된 값 버림
    if AUTO_CAL:
        tL = tR = tF = 0
        for _ in range(20):
            tL += int(cL.reflection())
            tR += int(cR.reflection())
            tF += int(cF.reflection())
            wait(5)
        mL, mR, mF = tL // 20, tR // 20, tF // 20
        if mL - bL > 8 and mR - bR > 8 and 2 * (mL - bL) > wL - bL and 2 * (mR - bR) > wR - bR:
            if 10 * (mF - bF) > 8 * (wF - bF) and 2 * mF >= min(mL, mR):
                wF = mF                             # 가운데도 흰 바닥 → 잰 값 그대로
            else:
                wF = wF * (mL + mR) // (wL + wR)    # 가운데가 라인 위·가장자리 → 좌우와 같은 비율로
            wL, wR = mL, mR
            print("자동보정 VALUE", [wL, bL, wR, bR, wF, bF])
        else:
            print("자동보정 안 함 (좌우 센서를 흰 바닥에 두고 출발할 것) — VALUE 그대로")
    gL, gR, gF = wL - bL, wR - bR, wF - bF

    pL = int(cL.reflection())
    pR = int(cR.reflection())
    pF = int(cF.reflection())
    sL = (min(max(pL, bL), wL) - bL) * 100 // gL
    sR = (min(max(pR, bR), wR) - bR) * 100 // gR
    sF = (min(max(pF, bF), wF) - bF) * 100 // gF

    last_error = 0
    d_filt = 0
    prev_steer = 0
    last_good = 0
    side = -1               # 방향기억 초기값: 왼쪽 (v16 direction=0과 같음)
    was_off = False
    armed = False           # 코너 예고 중
    lock_side = 0           # 코너 방향 잠금 (0 = 없음)
    t_lock = 0
    t_ahead = 0
    t_lost = -1
    pb_until = 0
    t_step = 0
    bank_dir = 0            # 현재 뱅킹 방향
    bank_st = ST_BANK       # 이번 뱅킹 조향 세기
    ra_on = False           # 이번 뱅킹이 직진 중 이탈(직각)인가
    brk_on = False          # 직각 브레이크 중
    brk_dir = 0
    t_brk = -10000
    turned = 0              # 뱅킹 방향으로 돈 각도 (deg×1000)
    flips = 0
    pivot_dir = 0           # 강제 턴 방향 (−1 좌, +1 우, 0 없음)
    t_pivot = 0
    mem_dir = 0             # 옆 센서가 마지막으로 진한 검정을 밟은 쪽 (v11 mem_dir)
    t_mem = -10000
    corner_dir = 0          # 코너 회전 고정 방향 (−1 좌, +1 우, 0 없음)
    corner_turn = 0         # 고정 뒤 그쪽으로 돈 각도 (deg×1000)
    cc_dir = 0              # 직각 신호 후보 방향
    t_cc = 0
    li = 0                  # 블랙박스 기록 수
    t_allw = -1             # 세 센서 모두 흰색이 시작된 시각
    t_alld = -1             # 세 센서 모두 검정이 시작된 시각
    t_allg = -1             # 세 센서 모두 회색이 시작된 시각
    t_log = 0
    t_side = 0              # 옆 센서가 마지막으로 방향기억을 갱신한 시각
    y_ema = 0               # 요레이트 평균 (°/s ×1000, 좌회전 +)
    ysgn = -1               # ERR_MODE 1: 가운데만 라인을 볼 때 라인 방향 (마지막 옆 센서 접촉, 좌 +)
    r_mod = 0               # KSP: 조향 명령이 만들 요레이트 모델 (°/s ×1000)
    r_n = DELAY_MS * 735 // 1000 + 1
    r_hist = [0] * r_n       # r_mod의 지연 버퍼 (DELAY_MS 전 값)
    r_i = 0
    c_ema = 0               # 곡선 판단용 요레이트 평균 (°/s ×1000, 좌회전 +)
    sw = StopWatch()

    while True:
        rL = int(cL.reflection())
        rR = int(cR.reflection())
        rF = int(cF.reflection())
        now = sw.time()
        # 프레임 동기: 센서 새 값에서만 제어 (값이 오래 안 변하면 FORCE_MS마다 한 번)
        if rL == pL and rR == pR and rF == pF and now - t_step < FORCE_MS:
            continue
        pL, pR, pF = rL, rR, rF
        STAT[0] += 1                                # 제어 스텝 수 (멈출 때 출력)
        dt = now - t_step
        t_step = now

        nL = (min(max(rL, bL), wL) - bL) * 100 // gL
        nR = (min(max(rR, bR), wR) - bR) * 100 // gR
        nF = (min(max(rF, bF), wF) - bF) * 100 // gF
        if AUTO_STOP:                               # 길을 오래 잃으면 멈추고 기록 출력 (시험용)
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

        pair_white = nL > W_NORM and nR > W_NORM
        ahead = False
        if USE_SENTINEL:
            if nF < (W_NORM if EDGE != 0 else BLACK_MID):   # 엣지 추종이면 가장자리에 걸쳐도 '라인 있음'
                t_ahead = now
                if armed:
                    armed = False
                    ev3.light.on(Color.GREEN)
            ahead = now - t_ahead <= AHEAD_HOLD_MS
            if not ahead and not pair_white and not armed:
                armed = True                        # 코너 예고: 앞이 하얘짐
                lock_side = 0                       # 새 코너 → 방향 잠금 새로
                pb_until = now + PB_MS
                ev3.light.on(Color.RED)

        if ERR_MODE == 1:                           # 연속 라인 위치 ymm (좌 +): 옆 센서가 보면 세 센서 무게중심,
            dL = 100 - DFLOOR - sL                  #   가운데만 보면 크기 = 가운데가 하얘진 정도, 방향 = 마지막 옆 접촉
            dR = 100 - DFLOOR - sR
            dF = 100 - DFLOOR - sF
            if dL < 0:
                dL = 0
            if dR < 0:
                dR = 0
            if dF < 0:
                dF = 0
            if dL + dR > SIDE_SEE:
                ysgn = 1 if dL > dR else -1
                ymm = (dL - dR) * SENSOR_GAP_MM // (dL + dF + dR)
            elif dF > SIDE_SEE:
                ymm = ysgn * CF_GAIN * (100 - DFLOOR - dF) // (100 - DFLOOR)
            else:
                ymm = ysgn * SENSOR_GAP_MM
            error = -ymm * 87 // 10                 # mm → E2 오차 단위 (옆 센서 하나 완전 검정 = ±200)
        elif WHITE_DB > 0:
            eL = 100 if sL > WHITE_DB else sL
            eR = 100 if sR > WHITE_DB else sR
            error = 2 * (eL - eR)
        else:
            error = 2 * (sL - sR)                   # 음수 = 라인이 왼쪽
        if SIDE_TTL_MS > 0:
            y_ema += (yaw_rate() * 1000 - y_ema) * dt // YAW_TAU_MS
        if EDGE != 0:                               # 엣지 추종: 가운데 센서가 목표보다 하야면 라인 몸통 쪽으로
            error -= EDGE * (sF - EDGE_TGT) * EDGE_K
        if error > SIDE_ERR:
            side = 1
            t_side = now
        elif error < -SIDE_ERR:
            side = -1
            t_side = now
        if LOCK_MS > 0 and armed and lock_side == 0 and (error > SIDE_ERR or error < -SIDE_ERR):
            lock_side = side                        # 코너 방향 잠금
            t_lock = now
        elif lock_side != 0 and not was_off and now - t_lock > LOCK_MS:
            lock_side = 0

        if CORNER_DEG > 0:
            if corner_dir == 0:
                if nL < BLACK_MID and nF < BLACK_MID and nR < BLACK_MID:
                    cc_dir = 0                      # 세 센서 모두 검정 = 교차로 → 가지 기억 지움
                elif nL < BLACK_MID and nF < BLACK_MID and nR > W_NORM:
                    cc_dir = -1                     # 왼쪽 가지 패턴 (20mm 라인 하나로는 불가능)
                    t_cc = now
                elif nR < BLACK_MID and nF < BLACK_MID and nL > W_NORM:
                    cc_dir = 1
                    t_cc = now
                if (cc_dir != 0 and now - t_cc <= CORNER_ARM_MS and nL > W_NORM and nR > W_NORM and nF > W_NORM
                        and not was_off and -cc_dir * yaw_rate() < CORNER_YAW):
                    corner_dir = cc_dir             # 가지를 본 직후 원래 라인이 끝남 → 직각: 즉시 회전 고정
                    corner_turn = 0
                    cc_dir = 0
            else:
                corner_turn -= corner_dir * yaw_rate() * dt
                if corner_turn >= CORNER_MAX * 1000:
                    corner_dir = 0
                elif corner_turn >= CORNER_DEG * 1000 and (nF < BLACK_MID or (nL if corner_dir < 0 else nR) < BLACK_MID):
                    corner_dir = 0                  # 충분히 돈 뒤 가운데·도는 쪽 센서가 새 라인을 찾음 → PD로
                    d_filt = 0
                    last_error = error
                    prev_steer = 0

        if nL < BLACK_MID and nR < BLACK_MID:      # 양쪽이 함께 검정 = 교차로 → 강제 턴 기억 지움
            mem_dir = 0
        elif nL < 30 and nR > W_NORM:                # v11식 바깥 센서 접촉 기억
            mem_dir = -1
            t_mem = now
        elif nR < 30 and nL > W_NORM:
            mem_dir = 1
            t_mem = now
        if (PIVOT_ST > 0 and pivot_dir == 0 and mem_dir != 0 and now - t_mem <= PIVOT_MEM_MS
                and nL > W_NORM and nR > W_NORM and nF > W_NORM):
            pivot_dir = mem_dir                     # ★흰흰흰 되는 즉시 강제 턴 (대기 없음)
            t_pivot = now
            mem_dir = 0
            was_off = True
            t_lost = now
            bank_dir = pivot_dir
            ev3.light.on(Color.YELLOW)
        if pivot_dir != 0 and ((now - t_pivot >= PIVOT_MIN_MS and nF < BLACK_MID) or now - t_pivot > PIVOT_MAX_MS):
            pivot_dir = 0                           # 가운데가 라인을 다시 찾음 → PD 복귀 (미분·슬루 리셋)
            was_off = False
            t_lost = -1
            lock_side = 0
            d_filt = 0
            last_error = error
            prev_steer = 0
            ev3.light.on(Color.GREEN)

        if pivot_dir != 0:                          # ★v11식 강제 턴 중: PD·자이로 없이 피벗 유지
            steer = pivot_dir * PIVOT_ST
            power = POWER
        elif corner_dir != 0:                       # 코너 회전 고정 중
            steer = corner_dir * CORNER_ST
            power = POWER
            side = corner_dir
            last_good = steer
            t_lost = -1
            was_off = False
        elif not pair_white or ahead:               # 라인 보임 (페어 사이 틈 포함)
            t_lost = -1
            if was_off:                             # 재포착 → 미분 리셋
                was_off = False
                lock_side = 0
                d_filt = 0
                last_error = error
                prev_steer = 0
            d_filt = (D_FILTER * (error - last_error) + D_COMP * d_filt) // 1000
            last_error = error
            yr = yaw_rate()
            kp_s = KP_S
            kg = KG
            if KP2 > 0:                             # 곡선 게인 스케줄링
                c_ema += (yr * 1000 - c_ema) * dt // CURVE_TAU_MS
                f = (c_ema if c_ema > 0 else -c_ema) // CURVE_YAW   # 0~1000 = 곡선 정도
                if f > 1000:
                    f = 1000
                kp_s = KP_S + (KP2_S - KP_S) * f // 1000
                if KG2 >= 0:
                    kg = KG + (KG2 - KG) * f // 1000
            steer = (kp_s * error + KD_S * d_filt) // S
            if sL < BLACK_MID and sR < BLACK_MID:
                steer = 0                           # 교차: 직진 유지
            if KSP > 0:                             # 지연 보상: 명령했지만 아직 몸에 안 나타난 회전을 미리 감쇠
                yr += KSP * (r_mod - r_hist[r_i]) // 1000000
            g = (kg * yr) // 1000
            if g > G_CLAMP:
                g = G_CLAMP
            elif g < -G_CLAMP:
                g = -G_CLAMP
            steer += g
            if BR_ST and -STR_YAW * 1000 < c_ema < STR_YAW * 1000 and nF < BLACK_MID:
                if nL < BLACK_MID and nR > W_NORM:
                    steer = -BR_ST                  # 왼쪽 가지 → 미리 왼쪽으로
                elif nR < BLACK_MID and nL > W_NORM:
                    steer = BR_ST                   # 오른쪽 가지 → 미리 오른쪽으로
            if (BRK_MS and BRK_AT == 0 and not brk_on and now - t_brk > 500 and nF < BLACK_MID
                    and -STR_YAW * 1000 < c_ema < STR_YAW * 1000):
                if nL < BLACK_MID and nR > W_NORM:
                    brk_on = True                   # 왼쪽 직각 → 브레이크
                    brk_dir = -1
                    t_brk = now
                elif nR < BLACK_MID and nL > W_NORM:
                    brk_on = True
                    brk_dir = 1
                    t_brk = now
            power = POWER
            last_good = steer
        else:
            if t_lost < 0:
                t_lost = now
            if now - t_lost <= GAP_MS:              # 짧은 공백: 직전 조향 유지
                steer = last_good
                power = POWER
            else:                                   # 진짜 이탈: 기억한 쪽으로 뱅킹
                if not was_off:
                    was_off = True
                    ev3.light.on(Color.YELLOW)
                    bank_dir = lock_side if lock_side != 0 else side
                    if SIDE_TTL_MS > 0 and lock_side == 0 and now - t_side > SIDE_TTL_MS:
                        bank_dir = 1 if y_ema > 0 else -1   # 좌로 돌던 중 = 로봇이 라인 왼쪽으로 샘 → 라인은 오른쪽
                    turned = 0
                    flips = 0
                    bank_st = ST_BANK
                    ra_on = False
                    if ST_BANK_STR and -STR_YAW * 1000 < c_ema < STR_YAW * 1000:
                        bank_st = ST_BANK_STR       # 직진 중 놓침 = 직각 → 제자리 회전으로
                        ra_on = RA_YAW > 0
                        if BRK_MS and BRK_AT == 1 and not brk_on and now - t_brk > 500:
                            brk_on = True           # 직진 중 놓침 = 직각 → 브레이크
                            brk_dir = bank_dir
                            t_brk = now
                elif SEARCH_DEG > 0:
                    # 한정 탐색: 뱅킹 방향으로 돈 각도를 자이로로 적분 (좌회전 + 레이트)
                    turned -= bank_dir * yaw_rate() * dt
                    if flips < 2 and turned > SEARCH_DEG * 1000:
                        bank_dir = -bank_dir        # 그쪽엔 라인이 없음 → 반대로
                        turned = 0
                        flips += 1
                steer = bank_dir * bank_st
                if ra_on:                           # 직각 회전 속도 상한
                    rate = -bank_dir * yaw_rate()     # 뱅킹 쪽으로 도는 속도 (°/s)
                    if rate > RA_YAW:
                        steer = bank_dir * (bank_st - RA_KQ * (rate - RA_YAW) // 100)
                if BANK_KG > 0:
                    gb = BANK_KG * yaw_rate() // 1000
                    if gb > ST_BANK + 30:
                        gb = ST_BANK + 30
                    elif gb < -ST_BANK - 30:
                        gb = -ST_BANK - 30
                    steer += gb
                power = RECOVER_POWER

        if SPD_K10 > 0:                             # 조향비례 감속
            sched = POWER - SPD_K10 * abs(steer) // 10
            if sched < MIN_POWER:
                sched = MIN_POWER
            if power > sched:
                power = sched
        if now < pb_until and power > PB_POWER:     # 코너 예고 사전감속
            power = PB_POWER

        if pivot_dir != 0:                          # 강제 턴: 슬루 제한 없이 즉시 풀 조향
            steer = pivot_dir * PIVOT_ST
            power = POWER
            prev_steer = steer
        elif corner_dir != 0:                       # 직각 회전 고정: 슬루 제한 없이 즉시
            steer = corner_dir * CORNER_ST
            power = POWER
            prev_steer = steer
        delta = steer - prev_steer                  # 슬루 리미터
        if delta > MAX_STEER_DELTA:
            steer = prev_steer + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steer = prev_steer - MAX_STEER_DELTA
        prev_steer = steer
        if KSP > 0:                                 # 요레이트 모델 갱신 (모터 1차 지연) + 지연 버퍼
            r_mod += (-steer * YAW_PER_STEER * 1000 - r_mod) * dt // TAU_MS
            r_hist[r_i] = r_mod
            r_i += 1
            if r_i >= r_n:
                r_i = 0

        if brk_on:                                  # 직각 브레이크 (안쪽 바퀴 −100, 바깥 바퀴 BRK_OUT)
            if now - t_brk >= BRK_MS or (BRK_V and fwd_speed() < BRK_V):
                brk_on = False
                BRK_CMD[0] = 0
                BRK_CMD[1] = 0
            else:
                if brk_dir < 0:                     # 왼쪽 직각: 안쪽(왼쪽) 바퀴 역회전
                    BRK_CMD[0] = -100
                    BRK_CMD[1] = BRK_OUT
                else:
                    BRK_CMD[0] = BRK_OUT
                    BRK_CMD[1] = -100
                power = 0                           # 블랙박스: 파워 0 = 브레이크 중
        if now - t_log >= LOG_MS:                   # 블랙박스 (주행 중 부담 ≈ 4ms마다 수 µs)
            t_log = now
            j = li % LOG_N * 7
            LOG[j] = rL if rL < 255 else 255
            LOG[j + 1] = rR if rR < 255 else 255
            LOG[j + 2] = rF if rF < 255 else 255
            v = steer + 128
            LOG[j + 3] = 0 if v < 0 else (255 if v > 255 else v)
            LOG[j + 4] = power if 0 <= power <= 255 else 0
            LOG[j + 5] = ((1 if (not pair_white or ahead) else 0) | (2 if ahead else 0) | (4 if pair_white else 0)
                          | (8 if was_off else 0) | (16 if side > 0 else 0) | (32 if bank_dir > 0 else 0)
                          | (64 if armed else 0) | (128 if (corner_dir != 0 or pivot_dir != 0) else 0))
            v = yaw_rate() // 8 + 128
            LOG[j + 6] = 0 if v < 0 else (255 if v > 255 else v)
            li += 1
            STAT[1] = li
        steering_dc(steer, power)


wait(200)                   # 켜진 직후 센서 값이 자리 잡을 때까지 (SPIKE판 YAW_FIXED와 같은 0.2초)
print("요레이트 = 모터 엔코더 (자이로 없음)")
t_run = StopWatch()
try:
    run()
finally:                    # 정지 버튼(SystemExit)에서도 실행됨
    print("STAT,rule,steps,%d,ms,%d" % (STAT[0], t_run.time()))
    n = STAT[1]                                 # 블랙박스: 정지 직전 2초 (sim/bb_parse.py로 해석)
    m = n if n < LOG_N else LOG_N
    print("BB,%d,%d,7,%s,new" % (LOG_MS, m, ",".join(str(x) for x in VALUE)))
    hx = "0123456789abcdef"
    s = ""
    for k in range(n - 1, n - 1 - m, -1):       # 최신 → 과거: 출력이 도중에 끊겨도 정지 직전이 남음
        j = k % LOG_N * 7
        for q in range(7):
            b = LOG[j + q]
            s += hx[b >> 4] + hx[b & 15]
        if len(s) >= 196:
            print("BBD," + s)
            s = ""
            wait(30)                            # 블루투스 출력이 밀려 잘리지 않게
    if s:
        print("BBD," + s)
    wait(200)

# 페어+센티넬 배치용 상수 (core 12월드 사진 트랙 9/12 20.2s, 1.6배 0/12):
#   KP 0.265 KD 5.181 KG 99 G_CLAMP 46 ALPHA 866 D_FILTER 341 MAX_STEER_DELTA 57 W_NORM 77
#   SIDE_ERR 88 GAP_MS 9 AHEAD_HOLD_MS 12 SEARCH_DEG 110 ST_BANK 72 RECOVER_POWER 100
#   SPD_K10 8 MIN_POWER 62 PB_MS 210 PB_POWER 52

# 이전 기본값(감속 있음, 지연 10~25ms 기준 GPU 최고 — 사진 90/100·9.0s, mrc 26/100):
#   KP 0.384 KG 223 G_CLAMP 68 ALPHA 950 D_FILTER 423 MAX_STEER_DELTA 34 SIDE_ERR 84 GAP_MS 1 AHEAD_HOLD_MS 14 SEARCH_DEG 0 ST_BANK 56 RECOVER_POWER 66 SPD_K10 8 MIN_POWER 89 PB_POWER 42
# 직전 기본값(B, 빠르게 도는 옛 시뮬 기준 — 실기에서 직각 직진·흔들림): AHEAD_HOLD_MS 25 ALPHA 886 CORNER_DEG 50 GAP_MS 3 G_CLAMP 75 KG 161 KP 0.535 MAX_STEER_DELTA 26 SEARCH_DEG 110 ST_BANK 50 W_NORM 75
# 직전 기본값(R1 — 구동 지연 모르고 맞춘 값, 실기에서 직각 뒤 좌우 왕복): KP 0.443 KD 0.0 D_FILTER 474 KG 212 G_CLAMP 90 ALPHA 781 MAX_STEER_DELTA 20 W_NORM 68 SIDE_ERR 66 GAP_MS 4 AHEAD_HOLD_MS 22 ST_BANK 69
# 직전 기본값(C3): AHEAD_HOLD_MS 29 ALPHA 793 D_FILTER 442 GAP_MS 15 G_CLAMP 96 KD 0.32 KG 226 KP 0.548 MAX_STEER_DELTA 9 PIVOT_MEM_MS 150 PIVOT_MIN_MS 30 PIVOT_ST 95 SIDE_ERR 85 ST_BANK 67 W_NORM 67
# 직전 기본값(D1): KP 0.616 KD 1.636 MAX_STEER_DELTA 56 ALPHA 875 KG 256
