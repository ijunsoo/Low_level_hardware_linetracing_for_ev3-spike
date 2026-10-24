from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis, Color
from pybricks.tools import wait, StopWatch

# =============================================================
#  spike_lf3_rl: SPIKE Prime 3센서 + 강화학습 잔차 정책 (자동 생성)
#
#  배치 — ★일렬(권장, 시뮬 1등): cL 좌(+25mm)·cR 우(-25mm)·cF 가운데, 셋 다 전방 90mm
#   (참고 EV3 파일과 같은 일렬. 오차 = 좌우, 가운데 = '라인 있음' 신호)
#   대안 '페어+센티넬': cL·cR = ±12.5mm @90mm, cF = 중앙 @130mm (상수는 맨 아래 주석)
#   시뮬 (2026-10-17, 파워 100, 실기형 동역학+구동지연): 사진 20/20·17.8s, 직각 20/20, 흔들림 ≈10mm
#
#  v16과 달라진 점:
#   1) 이탈 = 페어 둘 다 흰색 '이고' 센티넬도 AHEAD_HOLD_MS 동안 검정을 못 봄.
#      v16은 라인이 페어 사이 틈에 들어가도 이탈로 봐서 직선에서도 뱅킹을
#      수십~수백 번 했다(시뮬). 센티넬이 '앞에 라인 있음'을 따로 알려 준다.
#   2) 코너 예고 = 페어는 라인 위인데 센티넬이 하얘짐 (v16 바깥 트립와이어 대체)
#   3) 방향기억 = 페어 오차가 마지막으로 SIDE_ERR보다 크게 기운 쪽. 단 코너 예고
#      뒤 처음 기운 방향은 LOCK_MS 동안 잠금 (예각 꼭짓점에서 페어가 여러 선을
#      지나며 기억이 뒤집혀 반대로 뱅킹 → 유턴하던 문제, 시뮬 확인)
#   4) 타이머는 ms(StopWatch). 자이로 부호는 중력으로 요축을 찾아 정함 (스핀 없음)
#
#  LED: 빨강 = 코너 예고(사전감속), 초록 = 앞 라인 복귀, 노랑 = 이탈 → 뱅킹
#  EV3판(ev3/ev3_lf3_rule.py)과 로직이 같다 — 고치면 둘 다 고칠 것
#  (sim/regress.py의 lf3_parity가 두 파일이 같은 궤적을 내는지 확인)
# =============================================================

hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
cL = PUPDevice(Port.C)    # 좌 (전방 90mm, 측방 +25)
cR = PUPDevice(Port.B)    # 우 (전방 90mm, 측방 -25)
cF = PUPDevice(Port.A)    # 가운데 (전방 90mm) — 페어+센티넬 배치면 전방 130mm 중앙

# ------------- 튜닝 파라미터 (한 줄에 하나 — 시뮬 스윕이 이 줄들을 바꿔 넣음) ------------- #
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
VALUE = [34, 6, 22, 3, 21, 3]   # [흰, 검] × (좌, 우, 가운데) — 2026-10-17 spike_value.py 재측정 (매트·센서 높이 바뀌면 다시 잴 것)
AUTO_CAL = True         # 출발 시 정지 자세(가운데 센서=라인 위, 좌우=흰 바닥)에서 흰색을 다시 재 VALUE 보정
                        # (매트가 VALUE 잰 곳보다 어두우면 이탈을 못 느껴 직각에서 직진하던 문제 방지)
POWER = 100
KP = 0.45
KD = 0.0
KG = 180                # 자이로 댐핑 ×1000 (좌회전 + 요레이트에 곱해 조향에 더함)
G_CLAMP = 58            # 자이로 기여 상한 (재포착 스핀 폭주 차단, v14)
ALPHA = 800             # 센서 EMA ×1000
D_FILTER = 325          # 미분 저역필터 ×1000
MAX_STEER_DELTA = 15    # 제어 1스텝당 조향 변화 상한
W_NORM = 85             # 흰색 문턱 (정규화 0~100)
BLACK_MID = 50          # 검정 문턱
SIDE_ERR = 108          # 오차가 이보다 크게 기울면 '라인이 그쪽'으로 기억 (직선 위빙 ±50보다 크게)
EDGE = 0                # ★엣지 추종: 0 = 끔(좌우 센서 차이만 → 라인 20mm < 센서 간격 25mm라 가운데 ±12mm에선 오차 0,
                        #   옆 센서에 닿으면 갑자기 꺾음 = 작은 폭으로 자주 왕복). −1 = 가운데 센서로 라인 왼쪽 가장자리,
                        #   +1 = 오른쪽 가장자리를 따라감 → 센서값이 연속으로 변해 부드러운 비례 제어
EDGE_TGT = 50           # 가장자리 목표: 가운데 센서 정규화 % (50 = 반쯤 걸침)
EDGE_K = 2              # 가운데 센서 오차 배율
GAP_MS = 23              # 라인이 안 보여도 이 시간은 직전 조향 유지
AHEAD_HOLD_MS = 32      # 센티넬이 검정을 본 뒤 이 시간까지는 '앞에 라인 있음'
LOCK_MS = 0             # 코너 예고 뒤 처음 기운 방향을 이 시간 동안 고정 (0 = 끔 — 촘촘한 지그재그에선 역효과)
SEARCH_DEG = 0        # 뱅킹으로 이만큼 돌아도 라인이 없으면 반대쪽으로 (0 = 끔)
USE_SENTINEL = True     # False면 센티넬 무시 (= 페어만, v16식 이탈판정)
ST_BANK = 96            # 이탈 시 뱅킹 조향
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
RECOVER_POWER = 100      # 뱅킹 파워
SPD_K10 = 0             # 조향비례 감속 ×10 (0 = 끔)
MIN_POWER = 100          # 조향비례 감속 하한
PB_MS = 0             # 코너 예고 시 사전감속 시간 (0 = 끔)
PB_POWER = 100           # 사전감속 파워
FORCE_MS = 2            # 센서값이 안 변해도 이 간격마다 한 번은 제어

# ==================== 강화학습 잔차 정책 (자동 생성 — 직접 고치지 말 것) ====================
# 원본: sim/rl/runs/geo_steer_v2/policy_it150.pt
# 구조: 관측 22 → 16 → 16 → 2 (relu), 결정 주기 4.7ms, Δ조향 ±40, Δ파워 ±0
# 생성: sim/rl/export_policy.py — 규칙기반 로직은 spike/spike_lf3_rule.py와 같음
# 빛값 VALUE·포트는 실기에 맞게 바꿔도 됨 (정책은 정규화 0~100만 봄). 위의 규칙 상수는
# 정책이 그 값 위에서 학습됐으므로 바꾸면 재학습 권장. 시작 시 POLICY_US = 정책 1회 µs
DECISION_MS = 5             # 정책 결정 주기
DSTEER_MAX = 40
DPOWER_MAX = 0
KV = 979                  # 모터 deg/s → 정규화 바퀴속도 ×1000 (×KV//1000)

def policy(x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21):
    # 관측 정규화 z = (x - 평균) * 역표준편차 // 1000 (×1000 정수), ±8000으로 자름.
    # 범위를 곱하기 전에 봐서 곱이 ±8,000,000을 넘지 않음 → small int 안
    z0 = x0 - 810
    if z0 > 2735:
        z0 = 8000
    elif z0 < -2735:
        z0 = -8000
    else:
        z0 = z0 * 2924 // 1000
    z1 = x1 - 791
    if z1 > 2789:
        z1 = 8000
    elif z1 < -2789:
        z1 = -8000
    else:
        z1 = z1 * 2868 // 1000
    z2 = x2 - 669
    if z2 > 3290:
        z2 = 8000
    elif z2 < -3290:
        z2 = -8000
    else:
        z2 = z2 * 2431 // 1000
    z3 = x3 - 810
    if z3 > 2735:
        z3 = 8000
    elif z3 < -2735:
        z3 = -8000
    else:
        z3 = z3 * 2924 // 1000
    z4 = x4 - 791
    if z4 > 2789:
        z4 = 8000
    elif z4 < -2789:
        z4 = -8000
    else:
        z4 = z4 * 2868 // 1000
    z5 = x5 - 669
    if z5 > 3290:
        z5 = 8000
    elif z5 < -3290:
        z5 = -8000
    else:
        z5 = z5 * 2431 // 1000
    z6 = x6 - 810
    if z6 > 2735:
        z6 = 8000
    elif z6 < -2735:
        z6 = -8000
    else:
        z6 = z6 * 2924 // 1000
    z7 = x7 - 791
    if z7 > 2789:
        z7 = 8000
    elif z7 < -2789:
        z7 = -8000
    else:
        z7 = z7 * 2868 // 1000
    z8 = x8 - 668
    if z8 > 3290:
        z8 = 8000
    elif z8 < -3290:
        z8 = -8000
    else:
        z8 = z8 * 2431 // 1000
    z9 = x9 - 810
    if z9 > 2735:
        z9 = 8000
    elif z9 < -2735:
        z9 = -8000
    else:
        z9 = z9 * 2924 // 1000
    z10 = x10 - 791
    if z10 > 2789:
        z10 = 8000
    elif z10 < -2789:
        z10 = -8000
    else:
        z10 = z10 * 2868 // 1000
    z11 = x11 - 668
    if z11 > 3290:
        z11 = 8000
    elif z11 < -3290:
        z11 = -8000
    else:
        z11 = z11 * 2431 // 1000
    z12 = x12 - 19
    if z12 > 4244:
        z12 = 8000
    elif z12 < -4244:
        z12 = -8000
    else:
        z12 = z12 * 1885 // 1000
    z13 = x13 - -28
    if z13 > 5047:
        z13 = 8000
    elif z13 < -5047:
        z13 = -8000
    else:
        z13 = z13 * 1585 // 1000
    z14 = x14 - 26
    if z14 > 3065:
        z14 = 8000
    elif z14 < -3065:
        z14 = -8000
    else:
        z14 = z14 * 2610 // 1000
    z15 = x15 - 456
    if z15 > 2997:
        z15 = 8000
    elif z15 < -2997:
        z15 = -8000
    else:
        z15 = z15 * 2669 // 1000
    z16 = x16 - 497
    if z16 > 2707:
        z16 = 8000
    elif z16 < -2707:
        z16 = -8000
    else:
        z16 = z16 * 2955 // 1000
    z17 = x17 - -60
    if z17 > 4920:
        z17 = 8000
    elif z17 < -4920:
        z17 = -8000
    else:
        z17 = z17 * 1626 // 1000
    z18 = x18 - 267
    if z18 > 4405:
        z18 = 8000
    elif z18 < -4405:
        z18 = -8000
    else:
        z18 = z18 * 1816 // 1000
    z19 = x19 - 199
    if z19 > 2050:
        z19 = 8000
    elif z19 < -2050:
        z19 = -8000
    else:
        z19 = z19 * 3901 // 1000
    z20 = x20 - 43
    if z20 > 956:
        z20 = 8000
    elif z20 < -956:
        z20 = -8000
    else:
        z20 = z20 * 8367 // 1000
    z21 = x21 - -42
    if z21 > 7992:
        z21 = 8000
    elif z21 < -7992:
        z21 = -8000
    else:
        z21 = z21 * 1001 // 1000
    h0_0 = -31 + (-422 * z0 + -614 * z1 + -179 * z2 + 155 * z3 + -463 * z4 + 213 * z5 + -31 * z6 + -48 * z7 + 227 * z8 + 53 * z9 + -252 * z10 + -118 * z11 + 279 * z12 + 212 * z13 + -721 * z14 + -152 * z15 + -433 * z16 + -116 * z17 + -83 * z18 + -223 * z19 + 433 * z20 + -77 * z21) // 1000
    if h0_0 < 0:
        h0_0 = 0
    h0_1 = 163 + (172 * z0 + -167 * z1 + -256 * z2 + 53 * z3 + 526 * z4 + -565 * z5 + 144 * z6 + 67 * z7 + 625 * z8 + 128 * z9 + 418 * z10 + -556 * z11 + -200 * z12 + -287 * z13 + -85 * z14 + 212 * z15 + 52 * z16 + -28 * z17 + -45 * z18 + 404 * z19 + 41 * z20 + -657 * z21) // 1000
    if h0_1 < 0:
        h0_1 = 0
    h0_2 = 81 + (-91 * z0 + -54 * z1 + 21 * z2 + 141 * z3 + -222 * z4 + 239 * z5 + -106 * z6 + -461 * z7 + 652 * z8 + 171 * z9 + -519 * z10 + 340 * z11 + 285 * z12 + -89 * z13 + 288 * z14 + -73 * z15 + 871 * z16 + -155 * z17 + 154 * z18 + 302 * z19 + 129 * z20 + -71 * z21) // 1000
    if h0_2 < 0:
        h0_2 = 0
    h0_3 = 99 + (-149 * z0 + -173 * z1 + 636 * z2 + -190 * z3 + 308 * z4 + 769 * z5 + -384 * z6 + 224 * z7 + 141 * z8 + 270 * z9 + 374 * z10 + 60 * z11 + -271 * z12 + -293 * z13 + 398 * z14 + 261 * z15 + 88 * z16 + 159 * z17 + -179 * z18 + 143 * z19 + 288 * z20 + -620 * z21) // 1000
    if h0_3 < 0:
        h0_3 = 0
    h0_4 = 53 + (117 * z0 + -145 * z1 + -124 * z2 + 688 * z3 + -40 * z4 + 205 * z5 + 240 * z6 + 414 * z7 + 557 * z8 + 93 * z9 + 381 * z10 + 395 * z11 + -379 * z12 + -141 * z13 + 261 * z14 + -165 * z15 + 289 * z16 + -104 * z17 + 198 * z18 + -561 * z19 + -227 * z20 + -239 * z21) // 1000
    if h0_4 < 0:
        h0_4 = 0
    h0_5 = 113 + (869 * z0 + -217 * z1 + -2 * z2 + 254 * z3 + -104 * z4 + 506 * z5 + -211 * z6 + -644 * z7 + 571 * z8 + 210 * z9 + -22 * z10 + 168 * z11 + 460 * z12 + 247 * z13 + 167 * z14 + 133 * z15 + -515 * z16 + 326 * z17 + -1 * z18 + -45 * z19 + -61 * z20 + 489 * z21) // 1000
    if h0_5 < 0:
        h0_5 = 0
    h0_6 = 126 + (-270 * z0 + 89 * z1 + -166 * z2 + 406 * z3 + 738 * z4 + 348 * z5 + -442 * z6 + 45 * z7 + -17 * z8 + -52 * z9 + 282 * z10 + -79 * z11 + 4 * z12 + -148 * z13 + -460 * z14 + -102 * z15 + 281 * z16 + 73 * z17 + -123 * z18 + 129 * z19 + -66 * z20 + 861 * z21) // 1000
    if h0_6 < 0:
        h0_6 = 0
    h0_7 = 8 + (-452 * z0 + 225 * z1 + -563 * z2 + -28 * z3 + -40 * z4 + 87 * z5 + -36 * z6 + -360 * z7 + -94 * z8 + -202 * z9 + -49 * z10 + 357 * z11 + -249 * z12 + -521 * z13 + 283 * z14 + -551 * z15 + -518 * z16 + 230 * z17 + -198 * z18 + -53 * z19 + -130 * z20 + -495 * z21) // 1000
    if h0_7 < 0:
        h0_7 = 0
    h0_8 = 16 + (126 * z0 + 540 * z1 + -152 * z2 + 683 * z3 + -200 * z4 + 111 * z5 + 69 * z6 + -108 * z7 + -205 * z8 + 209 * z9 + -153 * z10 + -113 * z11 + -370 * z12 + -64 * z13 + -222 * z14 + 336 * z15 + -255 * z16 + -252 * z17 + 329 * z18 + 554 * z19 + 266 * z20 + -210 * z21) // 1000
    if h0_8 < 0:
        h0_8 = 0
    h0_9 = 9 + (155 * z0 + 171 * z1 + -86 * z2 + -335 * z3 + 339 * z4 + 203 * z5 + -238 * z6 + -284 * z7 + 161 * z8 + 54 * z9 + 290 * z10 + 485 * z11 + -212 * z12 + 427 * z13 + -425 * z14 + 236 * z15 + 202 * z16 + -631 * z17 + -174 * z18 + -116 * z19 + -423 * z20 + -431 * z21) // 1000
    if h0_9 < 0:
        h0_9 = 0
    h0_10 = 21 + (-63 * z0 + 104 * z1 + -207 * z2 + 141 * z3 + 343 * z4 + -443 * z5 + -422 * z6 + 253 * z7 + -66 * z8 + -373 * z9 + -553 * z10 + 12 * z11 + -103 * z12 + -28 * z13 + 353 * z14 + 616 * z15 + -267 * z16 + -351 * z17 + -266 * z18 + -466 * z19 + 400 * z20) // 1000
    if h0_10 < 0:
        h0_10 = 0
    h0_11 = 39 + (-17 * z0 + -188 * z1 + 151 * z2 + 298 * z3 + 455 * z4 + 26 * z5 + 474 * z6 + -614 * z7 + -257 * z8 + 420 * z9 + -539 * z10 + -238 * z11 + -212 * z12 + 348 * z13 + 276 * z14 + -62 * z15 + 249 * z16 + 174 * z17 + -397 * z18 + -421 * z19 + -284 * z20 + -132 * z21) // 1000
    if h0_11 < 0:
        h0_11 = 0
    h0_12 = 10 + (-250 * z0 + -266 * z1 + 205 * z2 + 262 * z3 + 333 * z4 + 27 * z5 + 448 * z6 + 34 * z7 + 336 * z8 + -48 * z9 + -253 * z10 + 479 * z11 + 692 * z12 + 45 * z13 + 96 * z14 + -39 * z15 + -456 * z16 + -183 * z17 + -63 * z18 + 511 * z19 + -407 * z20 + -84 * z21) // 1000
    if h0_12 < 0:
        h0_12 = 0
    h0_13 = 70 + (103 * z0 + 72 * z1 + -384 * z2 + -384 * z3 + 150 * z4 + 199 * z5 + 64 * z6 + 254 * z7 + -179 * z8 + 793 * z9 + -214 * z10 + -276 * z11 + 74 * z12 + -636 * z13 + 278 * z14 + -101 * z15 + -127 * z16 + -587 * z17 + 138 * z18 + -40 * z19 + -168 * z20 + 309 * z21) // 1000
    if h0_13 < 0:
        h0_13 = 0
    h0_14 = 108 + (-114 * z0 + 282 * z1 + -188 * z2 + -510 * z3 + 128 * z4 + 453 * z5 + 444 * z6 + 380 * z7 + 566 * z8 + 43 * z9 + -168 * z10 + 284 * z11 + -624 * z12 + 357 * z13 + -178 * z14 + 514 * z15 + -38 * z16 + 365 * z17 + -144 * z18 + 250 * z19 + 105 * z20 + 709 * z21) // 1000
    if h0_14 < 0:
        h0_14 = 0
    h0_15 = 10 + (-155 * z0 + -178 * z1 + 265 * z2 + -283 * z3 + 259 * z4 + -73 * z5 + -54 * z6 + -258 * z7 + 132 * z8 + -150 * z9 + 18 * z10 + -71 * z11 + -601 * z12 + 338 * z13 + 240 * z14 + -769 * z15 + -444 * z16 + -228 * z17 + 538 * z18 + 43 * z19 + 257 * z20 + 271 * z21) // 1000
    if h0_15 < 0:
        h0_15 = 0
    h1_0 = 31 + (-228 * h0_0 + -345 * h0_1 + 218 * h0_2 + -383 * h0_3 + 494 * h0_4 + -342 * h0_5 + -64 * h0_6 + 400 * h0_7 + -17 * h0_8 + -822 * h0_9 + 534 * h0_10 + 94 * h0_11 + 51 * h0_12 + -242 * h0_13 + 167 * h0_14 + -179 * h0_15) // 1000
    if h1_0 < 0:
        h1_0 = 0
    h1_1 = 106 + (-387 * h0_0 + 142 * h0_1 + 319 * h0_2 + 402 * h0_3 + 770 * h0_4 + -947 * h0_5 + 65 * h0_6 + -243 * h0_7 + 385 * h0_8 + 508 * h0_9 + -308 * h0_10 + -323 * h0_11 + 66 * h0_12 + 403 * h0_13 + 57 * h0_14 + -455 * h0_15) // 1000
    if h1_1 < 0:
        h1_1 = 0
    h1_2 = 148 + (-812 * h0_0 + 569 * h0_1 + -211 * h0_2 + 138 * h0_3 + -394 * h0_4 + -328 * h0_5 + 124 * h0_6 + -225 * h0_7 + 587 * h0_8 + -142 * h0_9 + 278 * h0_10 + -351 * h0_11 + -103 * h0_12 + -229 * h0_13 + -207 * h0_14 + 389 * h0_15) // 1000
    if h1_2 < 0:
        h1_2 = 0
    h1_3 = 206 + (-488 * h0_0 + 219 * h0_1 + -61 * h0_2 + 116 * h0_3 + -15 * h0_4 + -61 * h0_5 + -793 * h0_6 + -152 * h0_7 + -580 * h0_8 + 191 * h0_9 + 108 * h0_10 + 421 * h0_11 + -207 * h0_12 + 125 * h0_13 + 101 * h0_14 + -587 * h0_15) // 1000
    if h1_3 < 0:
        h1_3 = 0
    h1_4 = 72 + (-233 * h0_0 + -604 * h0_1 + -298 * h0_2 + -447 * h0_3 + 350 * h0_4 + 118 * h0_5 + -122 * h0_6 + -286 * h0_7 + 83 * h0_8 + 63 * h0_9 + -311 * h0_10 + 586 * h0_11 + -137 * h0_12 + 504 * h0_13 + -28 * h0_14 + 594 * h0_15) // 1000
    if h1_4 < 0:
        h1_4 = 0
    h1_5 = -83 + (202 * h0_0 + 49 * h0_1 + -544 * h0_2 + 151 * h0_3 + 459 * h0_4 + -144 * h0_5 + -410 * h0_6 + 788 * h0_7 + -258 * h0_8 + 5 * h0_9 + -91 * h0_10 + -344 * h0_11 + -317 * h0_12 + -160 * h0_13 + -422 * h0_14 + 571 * h0_15) // 1000
    if h1_5 < 0:
        h1_5 = 0
    h1_6 = 78 + (196 * h0_0 + 41 * h0_1 + -40 * h0_2 + 397 * h0_3 + -47 * h0_4 + 161 * h0_5 + -575 * h0_6 + -81 * h0_7 + 85 * h0_8 + -196 * h0_9 + 177 * h0_10 + 195 * h0_11 + 514 * h0_12 + 135 * h0_13 + 1005 * h0_14 + 464 * h0_15) // 1000
    if h1_6 < 0:
        h1_6 = 0
    h1_7 = 85 + (-619 * h0_0 + 115 * h0_1 + -115 * h0_2 + 329 * h0_3 + 227 * h0_4 + 566 * h0_5 + 684 * h0_6 + 193 * h0_7 + -604 * h0_8 + -500 * h0_9 + -461 * h0_10 + -40 * h0_11 + 24 * h0_12 + 393 * h0_13 + 421 * h0_14 + 175 * h0_15) // 1000
    if h1_7 < 0:
        h1_7 = 0
    h1_8 = 3 + (-38 * h0_0 + 128 * h0_1 + -102 * h0_2 + 611 * h0_3 + -137 * h0_4 + -348 * h0_5 + 213 * h0_6 + 415 * h0_7 + 406 * h0_8 + -227 * h0_9 + 154 * h0_10 + 768 * h0_11 + -556 * h0_12 + 336 * h0_13 + 33 * h0_14 + -92 * h0_15) // 1000
    if h1_8 < 0:
        h1_8 = 0
    h1_9 = 103 + (-240 * h0_0 + 281 * h0_1 + -217 * h0_2 + -440 * h0_3 + 311 * h0_4 + 510 * h0_5 + 230 * h0_6 + 206 * h0_7 + 192 * h0_8 + 510 * h0_9 + 147 * h0_10 + 385 * h0_11 + -279 * h0_12 + -519 * h0_13 + 863 * h0_14 + -170 * h0_15) // 1000
    if h1_9 < 0:
        h1_9 = 0
    h1_10 = 16 + (-316 * h0_0 + 336 * h0_1 + 372 * h0_2 + -478 * h0_3 + -234 * h0_4 + -54 * h0_5 + -144 * h0_6 + 699 * h0_7 + -27 * h0_8 + 79 * h0_9 + -391 * h0_10 + 557 * h0_11 + 628 * h0_12 + 46 * h0_13 + -81 * h0_14 + 138 * h0_15) // 1000
    if h1_10 < 0:
        h1_10 = 0
    h1_11 = -12 + (-41 * h0_0 + 103 * h0_1 + -40 * h0_2 + 232 * h0_3 + 337 * h0_4 + 886 * h0_5 + -512 * h0_6 + 74 * h0_7 + 564 * h0_8 + 42 * h0_9 + 173 * h0_10 + -132 * h0_11 + 300 * h0_12 + 341 * h0_13 + -534 * h0_14 + -471 * h0_15) // 1000
    if h1_11 < 0:
        h1_11 = 0
    h1_12 = 16 + (279 * h0_0 + 34 * h0_1 + 262 * h0_2 + 319 * h0_3 + 374 * h0_4 + 237 * h0_5 + -373 * h0_6 + -337 * h0_7 + 182 * h0_8 + -130 * h0_9 + -828 * h0_10 + 313 * h0_11 + 32 * h0_12 + -826 * h0_13 + -220 * h0_14 + 51 * h0_15) // 1000
    if h1_12 < 0:
        h1_12 = 0
    h1_13 = 39 + (-98 * h0_0 + 322 * h0_1 + 126 * h0_2 + 433 * h0_3 + 534 * h0_4 + -12 * h0_5 + 130 * h0_6 + -534 * h0_7 + -306 * h0_8 + 432 * h0_9 + 687 * h0_10 + 480 * h0_11 + 575 * h0_12 + -237 * h0_13 + -498 * h0_14 + 109 * h0_15) // 1000
    if h1_13 < 0:
        h1_13 = 0
    h1_14 = 54 + (352 * h0_0 + 800 * h0_1 + 724 * h0_2 + -288 * h0_3 + 373 * h0_4 + 241 * h0_5 + -298 * h0_6 + -320 * h0_7 + -24 * h0_8 + 18 * h0_9 + 184 * h0_10 + -32 * h0_11 + -380 * h0_12 + 278 * h0_13 + 28 * h0_14 + 295 * h0_15) // 1000
    if h1_14 < 0:
        h1_14 = 0
    h1_15 = 2 + (-427 * h0_0 + -797 * h0_1 + 773 * h0_2 + 270 * h0_3 + -226 * h0_4 + 392 * h0_5 + 41 * h0_6 + 304 * h0_7 + -22 * h0_8 + 318 * h0_9 + 121 * h0_10 + -74 * h0_11 + -340 * h0_12 + -177 * h0_13 + 196 * h0_14 + 415 * h0_15) // 1000
    if h1_15 < 0:
        h1_15 = 0
    a0 = 8 + (-3 * h1_0 + -208 * h1_1 + -130 * h1_2 + -47 * h1_3 + 97 * h1_4 + 86 * h1_5 + 32 * h1_6 + 157 * h1_7 + -10 * h1_8 + 139 * h1_9 + 34 * h1_10 + -71 * h1_11 + -81 * h1_12 + -121 * h1_13 + -38 * h1_14 + 51 * h1_15) // 1000
    a1 = 550 + (-44 * h1_0 + 55 * h1_1 + 29 * h1_2 + -214 * h1_3 + -32 * h1_4 + -14 * h1_5 + -51 * h1_6 + 26 * h1_7 + -2 * h1_8 + -91 * h1_9 + -38 * h1_10 + 1 * h1_11 + -47 * h1_12 + -51 * h1_13 + -38 * h1_14 + -28 * h1_15) // 1000
    if a0 < -1000:
        a0 = -1000
    elif a0 > 1000:
        a0 = 1000
    if a1 < -1000:
        a1 = -1000
    elif a1 > 1000:
        a1 = 1000
    return a0, a1


def steering_dc(steering, power):
    steering = max(-100, min(100, steering))
    slope = (power * 2) / 100
    if steering < 0:
        lm.dc(round(slope * steering + power, 2))
        rm.dc(power)
    else:
        lm.dc(power)
        rm.dc(round(-slope * steering + power, 2))


def find_yaw():
    # 정지 상태에서 중력이 실리는 가속도 축 = 로봇 상하축 (v11 방식).
    # 허브를 어떻게 달아도 '좌회전 = +' 요레이트를 얻는다. 반드시 정지 상태로 시작.
    for _ in range(40):
        wait(5)
    ax = hub.imu.acceleration(Axis.X)
    ay = hub.imu.acceleration(Axis.Y)
    az = hub.imu.acceleration(Axis.Z)
    if abs(ax) >= abs(ay) and abs(ax) >= abs(az):
        return Axis.X, (1 if ax > 0 else -1)
    if abs(ay) >= abs(az):
        return Axis.Y, (1 if ay > 0 else -1)
    return Axis.Z, (1 if az > 0 else -1)


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
    A_COMP = 1000 - ALPHA
    D_COMP = 1000 - D_FILTER
    wL, bL, wR, bR, wF, bF = VALUE
    if AUTO_CAL:
        tL = tR = tF = 0
        for _ in range(20):
            tL += cL.read(0)[0]
            tR += cR.read(0)[0]
            tF += cF.read(0)[0]
            wait(5)
        mL, mR, mF = tL // 20, tR // 20, tF // 20
        if 2 * mF < min(mL, mR) and mL - bL > 8 and mR - bR > 8:
            wF = wF * (mL + mR) // (wL + wR)        # 가운데 흰색은 좌우와 같은 비율로
            wL, wR = mL, mR
            print("자동보정 VALUE", [wL, bL, wR, bR, wF, bF])
        else:
            print("자동보정 안 함 (가운데 센서를 라인 위, 좌우를 흰 바닥에 두고 출발할 것) — VALUE 그대로")
    gL, gR, gF = wL - bL, wR - bR, wF - bF

    pL = cL.read(0)[0]
    pR = cR.read(0)[0]
    pF = cF.read(0)[0]
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
    t_dec = -1000           # 마지막 정책 결정 시각 (음수 = 아직 없음)
    DS = 0                  # 현재 Δ조향 (정수)
    DP = 0                  # 현재 Δ파워
    ap0 = ap1 = 0           # 직전 행동 (×1000)
    err_o = 0               # 직전 스텝 규칙 오차·최종 조향 (관측용)
    steer_o = 0
    # 센서 정규화값 최근 4결정 [현재 L·R·F, 1전, 2전, 3전] — 리스트 대신 변수 (GC 없음)
    q0 = q1 = q2 = q3 = q4 = q5 = q6 = q7 = q8 = q9 = q10 = q11 = 0
    tb = StopWatch()        # 정책 1회 연산시간(µs) 실측 → 시뮬 call_cost_us.policy에 넣을 값
    for _ in range(50):
        policy(500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500,
               0, 0, 0, 300, 300, 0, 0, 0, 0, 0)
    print("POLICY_US", tb.time() * 20)
    sw = StopWatch()

    while True:
        rL = cL.read(0)[0]
        rR = cR.read(0)[0]
        rF = cF.read(0)[0]
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
        if now - t_dec >= DECISION_MS:
            if t_dec < 0:                           # 첫 결정: 이력을 현재값으로 채움
                q3 = q6 = q9 = nL
                q4 = q7 = q10 = nR
                q5 = q8 = q11 = nF
            else:                                   # 이력 한 칸 밀기
                q9 = q6
                q10 = q7
                q11 = q8
                q6 = q3
                q7 = q4
                q8 = q5
                q3 = q0
                q4 = q1
                q5 = q2
            q0 = nL
            q1 = nR
            q2 = nF
            t_dec = now
            sa = (now - t_ahead) * 2
            sa = 0 if sa < 0 else (1000 if sa > 1000 else sa)
            sl = 0
            if t_lost >= 0:
                sl = (now - t_lost) * 2
                sl = 0 if sl < 0 else (1000 if sl > 1000 else sl)
            ap0, ap1 = policy(q0 * 10, q1 * 10, q2 * 10, q3 * 10, q4 * 10, q5 * 10,
                              q6 * 10, q7 * 10, q8 * 10, q9 * 10, q10 * 10, q11 * 10,
                              err_o * 5, steer_o * 10, int((hub.imu.angular_velocity(YAW_AXIS) * YAW_SIGN) * 2.5),
                              lm.speed() * KV // 1000, rm.speed() * KV // 1000,
                              ap0, ap1, sa, sl, side * 1000)
            DS = (ap0 * DSTEER_MAX + 500) // 1000
            DP = ap1 * DPOWER_MAX // 1000
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
                lm.brake()
                rm.brake()
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
                    hub.light.on(Color.GREEN)
            ahead = now - t_ahead <= AHEAD_HOLD_MS
            if not ahead and not pair_white and not armed:
                armed = True                        # 코너 예고: 앞이 하얘짐
                lock_side = 0                       # 새 코너 → 방향 잠금 새로
                pb_until = now + PB_MS
                hub.light.on(Color.RED)

        if WHITE_DB > 0:
            eL = 100 if sL > WHITE_DB else sL
            eR = 100 if sR > WHITE_DB else sR
            error = 2 * (eL - eR)
        else:
            error = 2 * (sL - sR)                   # 음수 = 라인이 왼쪽
        if SIDE_TTL_MS > 0:
            y_ema += (int(hub.imu.angular_velocity(YAW_AXIS)) * YAW_SIGN * 1000 - y_ema) * dt // YAW_TAU_MS
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
                        and not was_off and -cc_dir * int(hub.imu.angular_velocity(YAW_AXIS)) * YAW_SIGN < CORNER_YAW):
                    corner_dir = cc_dir             # 가지를 본 직후 원래 라인이 끝남 → 직각: 즉시 회전 고정
                    corner_turn = 0
                    cc_dir = 0
            else:
                corner_turn -= corner_dir * int(hub.imu.angular_velocity(YAW_AXIS)) * YAW_SIGN * dt
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
            hub.light.on(Color.YELLOW)
        if pivot_dir != 0 and ((now - t_pivot >= PIVOT_MIN_MS and nF < BLACK_MID) or now - t_pivot > PIVOT_MAX_MS):
            pivot_dir = 0                           # 가운데가 라인을 다시 찾음 → PD 복귀 (미분·슬루 리셋)
            was_off = False
            t_lost = -1
            lock_side = 0
            d_filt = 0
            last_error = error
            prev_steer = 0
            hub.light.on(Color.GREEN)

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
            steer = (KP_S * error + KD_S * d_filt) // S
            if sL < BLACK_MID and sR < BLACK_MID:
                steer = 0                           # 교차: 직진 유지
            g = (KG * int(hub.imu.angular_velocity(YAW_AXIS)) * YAW_SIGN) // 1000
            if g > G_CLAMP:
                g = G_CLAMP
            elif g < -G_CLAMP:
                g = -G_CLAMP
            steer += g
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
                    hub.light.on(Color.YELLOW)
                    bank_dir = lock_side if lock_side != 0 else side
                    if SIDE_TTL_MS > 0 and lock_side == 0 and now - t_side > SIDE_TTL_MS:
                        bank_dir = 1 if y_ema > 0 else -1   # 좌로 돌던 중 = 로봇이 라인 왼쪽으로 샘 → 라인은 오른쪽
                    turned = 0
                    flips = 0
                elif SEARCH_DEG > 0:
                    # 한정 탐색: 뱅킹 방향으로 돈 각도를 자이로로 적분 (좌회전 + 레이트)
                    turned -= bank_dir * int(hub.imu.angular_velocity(YAW_AXIS)) * YAW_SIGN * dt
                    if flips < 2 and turned > SEARCH_DEG * 1000:
                        bank_dir = -bank_dir        # 그쪽엔 라인이 없음 → 반대로
                        turned = 0
                        flips += 1
                steer = bank_dir * ST_BANK
                if BANK_KG > 0:
                    gb = BANK_KG * int(hub.imu.angular_velocity(YAW_AXIS)) * YAW_SIGN // 1000
                    if gb > ST_BANK + 30:
                        gb = ST_BANK + 30
                    elif gb < -ST_BANK - 30:
                        gb = -ST_BANK - 30
                    steer += gb
                power = RECOVER_POWER

        steer += DS                                 # ★정책 Δ조향
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
        power = power + DP                          # ★정책 Δ파워
        power = 0 if power < 0 else (100 if power > 100 else power)
        delta = steer - prev_steer                  # 슬루 리미터
        if delta > MAX_STEER_DELTA:
            steer = prev_steer + MAX_STEER_DELTA
        elif delta < -MAX_STEER_DELTA:
            steer = prev_steer - MAX_STEER_DELTA
        prev_steer = steer
        err_o = error
        steer_o = steer

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
            v = int(hub.imu.angular_velocity(YAW_AXIS)) * YAW_SIGN // 8 + 128
            LOG[j + 6] = 0 if v < 0 else (255 if v > 255 else v)
            li += 1
            STAT[1] = li
        steering_dc(steer, power)


YAW_AXIS, YAW_SIGN = find_yaw()
print("요축", YAW_AXIS, "부호", YAW_SIGN)
t_run = StopWatch()
try:
    run()
finally:                    # 정지 버튼(SystemExit)에서도 실행됨
    print("STAT,rl,steps,%d,ms,%d" % (STAT[0], t_run.time()))
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
