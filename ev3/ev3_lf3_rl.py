from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait, StopWatch

# =============================================================
#  ev3_lf3_rl: EV3 3센서 + 강화학습 잔차 정책 (자동 생성)
#
#  spike/spike_lf3_rule.py와 로직이 같다 (입출력만 EV3용):
#   - 컬러센서 3개 ★일렬(권장): S1 좌(+25)·S2 우(-25)·S3 가운데, 셋 다 전방 90mm
#     (대안 페어+센티넬: S1·S2 = ±12.5mm @90, S3 = 중앙 @130 — 상수는 맨 아래 주석)
#   - 자이로센서 S4: 레이트 모드, 좌회전 + 로 설정 (EV3엔 내장 IMU 없음)
#   - 모터 4개 스키드 (A·B 좌, C·D 우). 2모터면 LM·RM을 하나씩만 남길 것
#  고치면 SPIKE판도 같이 고칠 것 (sim/regress.py의 lf3_parity가 확인)
#  LED: 빨강 = 코너 예고(사전감속), 초록 = 앞 라인 복귀, 노랑 = 이탈 → 뱅킹
# =============================================================

ev3 = EV3Brick()
LM = (Motor(Port.A), Motor(Port.B, Direction.COUNTERCLOCKWISE))   # 좌 구동
RM = (Motor(Port.C, Direction.COUNTERCLOCKWISE), Motor(Port.D))   # 우 구동
cL = ColorSensor(Port.S1)      # 좌 (전방 90mm, 측방 +25)
cR = ColorSensor(Port.S2)      # 우 (전방 90mm, 측방 -25)
cF = ColorSensor(Port.S3)      # 가운데 (전방 90mm) — 페어+센티넬 배치면 전방 130mm 중앙
# 센서 윗면(빨간 점)이 위를 보게 달면 COUNTERCLOCKWISE = 좌회전 +.
# 뒤집어 달았으면 Direction.CLOCKWISE로 바꿀 것.
gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)

# ------------- 튜닝 파라미터 (한 줄에 하나 — 시뮬 스윕이 이 줄들을 바꿔 넣음) ------------- #
# 일렬 배치 GPU 스윕 최고값 (core 재검증). 사전감속·탐색·잠금·KD 모두 0이 이김
VALUE = [26, 2, 25, 2, 28, 2]   # [흰, 검] × (페어좌, 페어우, 센티넬) — EV3에서 재측정할 것
POWER = 100
KP = 0.384
KD = 0.0
KG = 223                # 자이로 댐핑 ×1000 (좌회전 + 요레이트에 곱해 조향에 더함)
G_CLAMP = 68            # 자이로 기여 상한 (재포착 스핀 폭주 차단, v14)
ALPHA = 950             # 센서 EMA ×1000
D_FILTER = 423          # 미분 저역필터 ×1000
MAX_STEER_DELTA = 34    # 제어 1스텝당 조향 변화 상한
W_NORM = 75             # 흰색 문턱 (정규화 0~100)
BLACK_MID = 50          # 검정 문턱
SIDE_ERR = 84          # 오차가 이보다 크게 기울면 '라인이 그쪽'으로 기억 (직선 위빙 ±50보다 크게)
GAP_MS = 1              # 라인이 안 보여도 이 시간은 직전 조향 유지
AHEAD_HOLD_MS = 14      # 센티넬이 검정을 본 뒤 이 시간까지는 '앞에 라인 있음'
LOCK_MS = 0             # 코너 예고 뒤 처음 기운 방향을 이 시간 동안 고정 (0 = 끔 — 촘촘한 지그재그에선 역효과)
SEARCH_DEG = 0        # 뱅킹으로 이만큼 돌아도 라인이 없으면 반대쪽으로 (0 = 끔)
USE_SENTINEL = True     # False면 센티넬 무시 (= 페어만, v16식 이탈판정)
ST_BANK = 56            # 이탈 시 뱅킹 조향
RECOVER_POWER = 66      # 뱅킹 파워
SPD_K10 = 8             # 조향비례 감속 ×10 (0 = 끔)
MIN_POWER = 89          # 조향비례 감속 하한
PB_MS = 0             # 코너 예고 시 사전감속 시간 (0 = 끔)
PB_POWER = 42           # 사전감속 파워
FORCE_MS = 2            # 센서값이 안 변해도 이 간격마다 한 번은 제어

# ==================== 강화학습 잔차 정책 (자동 생성 — 직접 고치지 말 것) ====================
# 원본: sim/rl/runs/row3_res2_h16_v3/policy.pt
# 구조: 관측 22 → 16 → 16 → 2 (relu), 결정 주기 4.8ms, Δ조향 ±40, Δ파워 ±40
# 생성: sim/rl/export_policy.py — 규칙기반 로직은 ev3/ev3_lf3_rule.py와 같음
# 빛값 VALUE·포트는 실기에 맞게 바꿔도 됨 (정책은 정규화 0~100만 봄). 위의 규칙 상수는
# 정책이 그 값 위에서 학습됐으므로 바꾸면 재학습 권장. 시작 시 POLICY_US = 정책 1회 µs
DECISION_MS = 5             # 정책 결정 주기
DSTEER_MAX = 40
DPOWER_MAX = 40
KV = 308                  # 모터 deg/s → 정규화 바퀴속도 ×1000 (×KV//1000)

def policy(x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, x20, x21):
    # 관측 정규화 z = (x - 평균) * 역표준편차 // 1000 (×1000 정수), ±8000으로 자름.
    # 범위를 곱하기 전에 봐서 곱이 ±8,000,000을 넘지 않음 → small int 안
    z0 = x0 - 797
    if z0 > 2682:
        z0 = 8000
    elif z0 < -2682:
        z0 = -8000
    else:
        z0 = z0 * 2982 // 1000
    z1 = x1 - 796
    if z1 > 2679:
        z1 = 8000
    elif z1 < -2679:
        z1 = -8000
    else:
        z1 = z1 * 2986 // 1000
    z2 = x2 - 577
    if z2 > 3401:
        z2 = 8000
    elif z2 < -3401:
        z2 = -8000
    else:
        z2 = z2 * 2352 // 1000
    z3 = x3 - 797
    if z3 > 2682:
        z3 = 8000
    elif z3 < -2682:
        z3 = -8000
    else:
        z3 = z3 * 2982 // 1000
    z4 = x4 - 796
    if z4 > 2679:
        z4 = 8000
    elif z4 < -2679:
        z4 = -8000
    else:
        z4 = z4 * 2986 // 1000
    z5 = x5 - 577
    if z5 > 3401:
        z5 = 8000
    elif z5 < -3401:
        z5 = -8000
    else:
        z5 = z5 * 2352 // 1000
    z6 = x6 - 797
    if z6 > 2682:
        z6 = 8000
    elif z6 < -2682:
        z6 = -8000
    else:
        z6 = z6 * 2982 // 1000
    z7 = x7 - 796
    if z7 > 2680:
        z7 = 8000
    elif z7 < -2680:
        z7 = -8000
    else:
        z7 = z7 * 2985 // 1000
    z8 = x8 - 576
    if z8 > 3401:
        z8 = 8000
    elif z8 < -3401:
        z8 = -8000
    else:
        z8 = z8 * 2352 // 1000
    z9 = x9 - 797
    if z9 > 2683:
        z9 = 8000
    elif z9 < -2683:
        z9 = -8000
    else:
        z9 = z9 * 2981 // 1000
    z10 = x10 - 796
    if z10 > 2680:
        z10 = 8000
    elif z10 < -2680:
        z10 = -8000
    else:
        z10 = z10 * 2985 // 1000
    z11 = x11 - 576
    if z11 > 3402:
        z11 = 8000
    elif z11 < -3402:
        z11 = -8000
    else:
        z11 = z11 * 2351 // 1000
    z12 = x12 - 1
    if z12 > 4113:
        z12 = 8000
    elif z12 < -4113:
        z12 = -8000
    else:
        z12 = z12 * 1945 // 1000
    z13 = x13 - 6
    if z13 > 2630:
        z13 = 8000
    elif z13 < -2630:
        z13 = -8000
    else:
        z13 = z13 * 3041 // 1000
    z14 = x14 - 19
    if z14 > 5780:
        z14 = 8000
    elif z14 < -5780:
        z14 = -8000
    else:
        z14 = z14 * 1384 // 1000
    z15 = x15 - 511
    if z15 > 1362:
        z15 = 8000
    elif z15 < -1362:
        z15 = -8000
    else:
        z15 = z15 * 5870 // 1000
    z16 = x16 - 517
    if z16 > 1409:
        z16 = 8000
    elif z16 < -1409:
        z16 = -8000
    else:
        z16 = z16 * 5675 // 1000
    z17 = x17 - 1
    if z17 > 3440:
        z17 = 8000
    elif z17 < -3440:
        z17 = -8000
    else:
        z17 = z17 * 2325 // 1000
    z18 = x18 - 371
    if z18 > 3472:
        z18 = 8000
    elif z18 < -3472:
        z18 = -8000
    else:
        z18 = z18 * 2304 // 1000
    z19 = x19 - 134
    if z19 > 1637:
        z19 = 8000
    elif z19 < -1637:
        z19 = -8000
    else:
        z19 = z19 * 4885 // 1000
    z20 = x20 - 15
    if z20 > 424:
        z20 = 8000
    elif z20 < -424:
        z20 = -8000
    else:
        z20 = z20 * 18828 // 1000
    z21 = x21 - -3
    if z21 > 8000:
        z21 = 8000
    elif z21 < -8000:
        z21 = -8000
    else:
        z21 = z21 * 1000 // 1000
    h0_0 = -91 + (-545 * z0 + -574 * z1 + -166 * z2 + 22 * z3 + -471 * z4 + 136 * z5 + -202 * z6 + -109 * z7 + 92 * z8 + -146 * z9 + -288 * z10 + -277 * z11 + 171 * z12 + 9 * z13 + -631 * z14 + -237 * z15 + -345 * z16 + -92 * z17 + -228 * z19 + 141 * z20 + -198 * z21) // 1000
    if h0_0 < 0:
        h0_0 = 0
    h0_1 = -19 + (148 * z0 + -447 * z1 + -347 * z2 + 83 * z3 + 247 * z4 + -640 * z5 + 192 * z6 + -196 * z7 + 527 * z8 + 155 * z9 + 168 * z10 + -619 * z11 + 61 * z12 + -229 * z13 + -197 * z14 + 234 * z15 + -36 * z16 + 5 * z17 + -101 * z18 + 276 * z19 + 52 * z20 + -568 * z21) // 1000
    if h0_1 < 0:
        h0_1 = 0
    h0_2 = 84 + (-160 * z0 + -178 * z1 + 19 * z2 + 133 * z3 + -318 * z4 + 187 * z5 + -94 * z6 + -520 * z7 + 580 * z8 + 243 * z9 + -580 * z10 + 256 * z11 + 299 * z12 + -92 * z13 + 336 * z14 + -89 * z15 + 936 * z16 + -144 * z17 + 275 * z18 + 327 * z19 + 103 * z20 + 167 * z21) // 1000
    if h0_2 < 0:
        h0_2 = 0
    h0_3 = 89 + (-214 * z0 + -465 * z1 + 625 * z2 + -254 * z3 + 134 * z4 + 748 * z5 + -419 * z6 + 116 * z7 + 35 * z8 + 270 * z9 + 309 * z10 + -51 * z11 + -153 * z12 + -508 * z13 + 156 * z14 + 439 * z15 + -136 * z16 + 401 * z17 + -108 * z18 + 80 * z19 + 382 * z20 + -476 * z21) // 1000
    if h0_3 < 0:
        h0_3 = 0
    h0_4 = 98 + (-8 * z0 + -53 * z1 + -168 * z2 + 569 * z3 + 37 * z4 + 180 * z5 + 157 * z6 + 502 * z7 + 533 * z8 + 89 * z9 + 430 * z10 + 381 * z11 + -538 * z12 + -157 * z13 + 179 * z14 + -137 * z15 + 130 * z16 + 132 * z17 + 229 * z18 + -643 * z19 + -211 * z20 + 26 * z21) // 1000
    if h0_4 < 0:
        h0_4 = 0
    h0_5 = 40 + (843 * z0 + -247 * z1 + -291 * z2 + 175 * z3 + -118 * z4 + 262 * z5 + -300 * z6 + -614 * z7 + 389 * z8 + 93 * z9 + 12 * z10 + 74 * z11 + 482 * z12 + 19 * z13 + 54 * z14 + 224 * z15 + -566 * z16 + 263 * z17 + 30 * z18 + 48 * z19 + -234 * z20 + 364 * z21) // 1000
    if h0_5 < 0:
        h0_5 = 0
    h0_6 = -91 + (-422 * z0 + -78 * z1 + -358 * z2 + 220 * z3 + 528 * z4 + 220 * z5 + -615 * z6 + -174 * z7 + -95 * z8 + -283 * z9 + 18 * z10 + -83 * z11 + -15 * z12 + -228 * z13 + -558 * z14 + -252 * z15 + 59 * z16 + 102 * z17 + -57 * z18 + 220 * z19 + -17 * z20 + 493 * z21) // 1000
    if h0_6 < 0:
        h0_6 = 0
    h0_7 = 42 + (-514 * z0 + 293 * z1 + -693 * z2 + -121 * z3 + 4 * z4 + -16 * z5 + -150 * z6 + -255 * z7 + -194 * z8 + -294 * z9 + 72 * z10 + 282 * z11 + -328 * z12 + -385 * z13 + 273 * z14 + -423 * z15 + -402 * z16 + 259 * z17 + -280 * z18 + -12 * z19 + -419 * z20 + -500 * z21) // 1000
    if h0_7 < 0:
        h0_7 = 0
    h0_8 = 39 + (173 * z0 + 452 * z1 + -15 * z2 + 716 * z3 + -252 * z4 + 237 * z5 + 93 * z6 + -133 * z7 + -101 * z8 + 278 * z9 + -103 * z10 + -23 * z11 + -269 * z12 + -112 * z13 + -317 * z14 + 231 * z15 + -469 * z16 + -225 * z17 + 301 * z18 + 736 * z19 + 422 * z20 + -76 * z21) // 1000
    if h0_8 < 0:
        h0_8 = 0
    h0_9 = 47 + (157 * z0 + -64 * z1 + -23 * z2 + -270 * z3 + 122 * z4 + 263 * z5 + -123 * z6 + -432 * z7 + 205 * z8 + 180 * z9 + 202 * z10 + 515 * z11 + -47 * z12 + 534 * z13 + -469 * z14 + 208 * z15 + 124 * z16 + -764 * z17 + -82 * z18 + -285 * z19 + -261 * z20 + -246 * z21) // 1000
    if h0_9 < 0:
        h0_9 = 0
    h0_10 = 58 + (-127 * z0 + -109 * z1 + -24 * z2 + 12 * z3 + 210 * z4 + -245 * z5 + -573 * z6 + 181 * z7 + 94 * z8 + -554 * z9 + -582 * z10 + 170 * z11 + -47 * z12 + 40 * z13 + 559 * z14 + 445 * z15 + -104 * z16 + -377 * z17 + -204 * z18 + -284 * z19 + 505 * z20 + -157 * z21) // 1000
    if h0_10 < 0:
        h0_10 = 0
    h0_11 = 104 + (-61 * z0 + -277 * z1 + 346 * z2 + 304 * z3 + 416 * z4 + 145 * z5 + 494 * z6 + -602 * z7 + -200 * z8 + 486 * z9 + -501 * z10 + -214 * z11 + -170 * z12 + 358 * z13 + 109 * z14 + 106 * z15 + 220 * z16 + 207 * z17 + -267 * z18 + -443 * z19 + -296 * z20 + 306 * z21) // 1000
    if h0_11 < 0:
        h0_11 = 0
    h0_12 = 39 + (-327 * z0 + -140 * z1 + 377 * z2 + 170 * z3 + 464 * z4 + 195 * z5 + 408 * z6 + 120 * z7 + 449 * z8 + -26 * z9 + -227 * z10 + 556 * z11 + 532 * z12 + -136 * z13 + 46 * z14 + 42 * z15 + -463 * z16 + -47 * z17 + 64 * z18 + 643 * z19 + -447 * z20 + 105 * z21) // 1000
    if h0_12 < 0:
        h0_12 = 0
    h0_13 = -89 + (113 * z0 + -62 * z1 + -299 * z2 + -477 * z3 + 59 * z4 + 301 * z5 + -96 * z6 + 247 * z7 + -85 * z8 + 651 * z9 + -168 * z10 + -167 * z11 + 217 * z12 + -631 * z13 + 197 * z14 + -291 * z15 + -428 * z16 + -339 * z17 + 218 * z18 + -77 * z19 + 65 * z20 + 248 * z21) // 1000
    if h0_13 < 0:
        h0_13 = 0
    h0_14 = 96 + (-158 * z0 + 190 * z1 + -392 * z2 + -545 * z3 + -14 * z4 + 283 * z5 + 391 * z6 + 323 * z7 + 444 * z8 + -49 * z9 + -166 * z10 + 187 * z11 + -546 * z12 + 472 * z13 + 17 * z14 + 220 * z15 + -28 * z16 + 224 * z17 + -208 * z18 + 206 * z19 + 289 * z20 + 528 * z21) // 1000
    if h0_14 < 0:
        h0_14 = 0
    h0_15 = 57 + (-81 * z0 + -23 * z1 + 288 * z2 + -183 * z3 + 353 * z4 + -56 * z5 + 48 * z6 + -132 * z7 + 134 * z8 + -32 * z9 + 120 * z10 + -96 * z11 + -635 * z12 + 544 * z13 + 154 * z14 + -711 * z15 + -469 * z16 + -246 * z17 + 496 * z18 + 67 * z19 + 201 * z20 + 363 * z21) // 1000
    if h0_15 < 0:
        h0_15 = 0
    h1_0 = 155 + (-249 * h0_0 + -335 * h0_1 + 266 * h0_2 + -349 * h0_3 + 532 * h0_4 + -102 * h0_5 + 49 * h0_6 + 494 * h0_7 + 130 * h0_8 + -782 * h0_9 + 517 * h0_10 + 65 * h0_11 + 243 * h0_12 + -292 * h0_13 + 344 * h0_14 + -150 * h0_15) // 1000
    if h1_0 < 0:
        h1_0 = 0
    h1_1 = 55 + (-370 * h0_0 + 86 * h0_1 + 197 * h0_2 + 419 * h0_3 + 652 * h0_4 + -704 * h0_5 + 120 * h0_6 + -122 * h0_7 + 327 * h0_8 + 331 * h0_9 + -444 * h0_10 + -328 * h0_11 + 71 * h0_12 + 321 * h0_13 + 213 * h0_14 + -359 * h0_15) // 1000
    if h1_1 < 0:
        h1_1 = 0
    h1_2 = 29 + (-597 * h0_0 + 580 * h0_1 + -528 * h0_2 + 4 * h0_3 + -547 * h0_4 + -31 * h0_5 + -35 * h0_6 + -283 * h0_7 + 661 * h0_8 + -156 * h0_9 + 113 * h0_10 + -417 * h0_11 + -79 * h0_12 + -361 * h0_13 + 6 * h0_14 + 375 * h0_15) // 1000
    if h1_2 < 0:
        h1_2 = 0
    h1_3 = 179 + (-418 * h0_0 + 229 * h0_1 + -79 * h0_2 + 24 * h0_3 + 5 * h0_4 + -83 * h0_5 + -835 * h0_6 + -199 * h0_7 + -610 * h0_8 + 153 * h0_9 + 113 * h0_10 + 402 * h0_11 + -270 * h0_12 + 30 * h0_13 + 45 * h0_14 + -613 * h0_15) // 1000
    if h1_3 < 0:
        h1_3 = 0
    h1_4 = 104 + (-301 * h0_0 + -698 * h0_1 + -380 * h0_2 + -468 * h0_3 + 392 * h0_4 + 51 * h0_5 + -139 * h0_6 + -200 * h0_7 + 224 * h0_8 + 62 * h0_9 + -279 * h0_10 + 506 * h0_11 + -113 * h0_12 + 419 * h0_13 + 76 * h0_14 + 780 * h0_15) // 1000
    if h1_4 < 0:
        h1_4 = 0
    h1_5 = 23 + (9 * h0_0 + 103 * h0_1 + -571 * h0_2 + 195 * h0_3 + 577 * h0_4 + -91 * h0_5 + -310 * h0_6 + 789 * h0_7 + -265 * h0_8 + 223 * h0_9 + -28 * h0_10 + -436 * h0_11 + -159 * h0_12 + -211 * h0_13 + -241 * h0_14 + 505 * h0_15) // 1000
    if h1_5 < 0:
        h1_5 = 0
    h1_6 = -25 + (140 * h0_0 + 3 * h0_1 + -103 * h0_2 + 573 * h0_3 + -22 * h0_4 + 29 * h0_5 + -590 * h0_6 + -109 * h0_7 + 129 * h0_8 + -174 * h0_9 + 164 * h0_10 + -145 * h0_11 + 566 * h0_12 + 201 * h0_13 + 789 * h0_14 + 439 * h0_15) // 1000
    if h1_6 < 0:
        h1_6 = 0
    h1_7 = 42 + (-688 * h0_0 + 81 * h0_1 + -206 * h0_2 + 309 * h0_3 + 125 * h0_4 + 517 * h0_5 + 587 * h0_6 + 193 * h0_7 + -728 * h0_8 + -380 * h0_9 + -372 * h0_10 + -102 * h0_11 + -4 * h0_12 + 246 * h0_13 + 371 * h0_14 + 123 * h0_15) // 1000
    if h1_7 < 0:
        h1_7 = 0
    h1_8 = 53 + (124 * h0_0 + 163 * h0_1 + -95 * h0_2 + 648 * h0_3 + -198 * h0_4 + -254 * h0_5 + 211 * h0_6 + 420 * h0_7 + 283 * h0_8 + -212 * h0_9 + 262 * h0_10 + 885 * h0_11 + -576 * h0_12 + 252 * h0_13 + 5 * h0_14 + -106 * h0_15) // 1000
    if h1_8 < 0:
        h1_8 = 0
    h1_9 = 88 + (-183 * h0_0 + 193 * h0_1 + -66 * h0_2 + -358 * h0_3 + 445 * h0_4 + 456 * h0_5 + 132 * h0_6 + 282 * h0_7 + 195 * h0_8 + 633 * h0_9 + 212 * h0_10 + 212 * h0_11 + -281 * h0_12 + -473 * h0_13 + 788 * h0_14 + -62 * h0_15) // 1000
    if h1_9 < 0:
        h1_9 = 0
    h1_10 = 22 + (-412 * h0_0 + 350 * h0_1 + 461 * h0_2 + -499 * h0_3 + -155 * h0_4 + -143 * h0_5 + -262 * h0_6 + 738 * h0_7 + -64 * h0_8 + 165 * h0_9 + -465 * h0_10 + 455 * h0_11 + 634 * h0_12 + 10 * h0_13 + -131 * h0_14 + 190 * h0_15) // 1000
    if h1_10 < 0:
        h1_10 = 0
    h1_11 = 80 + (-29 * h0_0 + 78 * h0_1 + -126 * h0_2 + 432 * h0_3 + 354 * h0_4 + 938 * h0_5 + -179 * h0_6 + 244 * h0_7 + 600 * h0_8 + -38 * h0_9 + 117 * h0_10 + -149 * h0_11 + 360 * h0_12 + 463 * h0_13 + -453 * h0_14 + -475 * h0_15) // 1000
    if h1_11 < 0:
        h1_11 = 0
    h1_12 = -4 + (114 * h0_0 + -264 * h0_1 + 265 * h0_2 + 229 * h0_3 + 371 * h0_4 + 248 * h0_5 + -183 * h0_6 + -203 * h0_7 + 230 * h0_8 + -381 * h0_9 + -978 * h0_10 + 373 * h0_11 + -25 * h0_12 + -765 * h0_13 + -62 * h0_14 + 119 * h0_15) // 1000
    if h1_12 < 0:
        h1_12 = 0
    h1_13 = 90 + (-123 * h0_0 + 256 * h0_1 + 32 * h0_2 + 542 * h0_3 + 502 * h0_4 + 16 * h0_5 + 347 * h0_6 + -428 * h0_7 + -237 * h0_8 + 382 * h0_9 + 651 * h0_10 + 504 * h0_11 + 604 * h0_12 + -173 * h0_13 + -347 * h0_14 + 138 * h0_15) // 1000
    if h1_13 < 0:
        h1_13 = 0
    h1_14 = -31 + (296 * h0_0 + 815 * h0_1 + 633 * h0_2 + -296 * h0_3 + 348 * h0_4 + 135 * h0_5 + -273 * h0_6 + -387 * h0_7 + -14 * h0_8 + -231 * h0_9 + 118 * h0_10 + -69 * h0_11 + -557 * h0_12 + 412 * h0_13 + -208 * h0_14 + 234 * h0_15) // 1000
    if h1_14 < 0:
        h1_14 = 0
    h1_15 = 108 + (-382 * h0_0 + -517 * h0_1 + 866 * h0_2 + 232 * h0_3 + -121 * h0_4 + 372 * h0_5 + -159 * h0_6 + 301 * h0_7 + 55 * h0_8 + 512 * h0_9 + 111 * h0_10 + 79 * h0_11 + -258 * h0_12 + -184 * h0_13 + 93 * h0_14 + 392 * h0_15) // 1000
    if h1_15 < 0:
        h1_15 = 0
    a0 = -56 + (121 * h1_0 + -53 * h1_1 + 155 * h1_2 + -6 * h1_3 + 160 * h1_4 + 44 * h1_5 + -40 * h1_6 + -69 * h1_8 + 64 * h1_9 + 133 * h1_10 + -129 * h1_11 + 80 * h1_12 + -91 * h1_13 + -47 * h1_14 + 62 * h1_15) // 1000
    a1 = 74 + (67 * h1_0 + 13 * h1_1 + -106 * h1_2 + -219 * h1_3 + -37 * h1_4 + 41 * h1_5 + 1 * h1_6 + 86 * h1_7 + 36 * h1_8 + 28 * h1_9 + 44 * h1_10 + 65 * h1_11 + 36 * h1_12 + 49 * h1_13 + -44 * h1_14 + 29 * h1_15) // 1000
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
        left, right = round(slope * steering + power, 2), power
    else:
        left, right = power, round(-slope * steering + power, 2)
    for m in LM:
        m.dc(left)
    for m in RM:
        m.dc(right)


def gyro_bias():
    # 정지 상태 레이트 평균 = 바이어스 (반드시 정지 상태로 시작). SPIKE판의
    # 요축 감지와 같은 200ms를 쓴다.
    total = 0
    for _ in range(40):
        total += gyro.speed()
        wait(5)
    return total // 40


STAT = [0]              # 제어 스텝 수 — 정지 버튼을 누르면 STAT 줄로 출력 (실기 스텝률 → 시뮬 보정)


def run():
    S = 100000
    KP_S = round(KP * S)
    KD_S = round(KD * S)
    A_COMP = 1000 - ALPHA
    D_COMP = 1000 - D_FILTER
    wL, bL, wR, bR, wF, bF = VALUE
    gL, gR, gF = wL - bL, wR - bR, wF - bF

    pL = cL.reflection()
    pR = cR.reflection()
    pF = cF.reflection()
    sL = (min(max(pL, bL), wL) - bL) * 100 // gL
    sR = (min(max(pR, bR), wR) - bR) * 100 // gR

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
        rL = cL.reflection()
        rR = cR.reflection()
        rF = cF.reflection()
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
                              err_o * 5, steer_o * 10, int(((gyro.speed() - GBIAS)) * 2.5),
                              LM[0].speed() * KV // 1000, RM[0].speed() * KV // 1000,
                              ap0, ap1, sa, sl, side * 1000)
            DS = (ap0 * DSTEER_MAX + 500) // 1000
            DP = ap1 * DPOWER_MAX // 1000
        sL = (ALPHA * nL + A_COMP * sL) // 1000
        sR = (ALPHA * nR + A_COMP * sR) // 1000

        pair_white = nL > W_NORM and nR > W_NORM
        ahead = False
        if USE_SENTINEL:
            if nF < BLACK_MID:
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

        error = 2 * (sL - sR)                       # 음수 = 라인이 왼쪽
        if error > SIDE_ERR:
            side = 1
        elif error < -SIDE_ERR:
            side = -1
        if LOCK_MS > 0 and armed and lock_side == 0 and (error > SIDE_ERR or error < -SIDE_ERR):
            lock_side = side                        # 코너 방향 잠금
            t_lock = now
        elif lock_side != 0 and not was_off and now - t_lock > LOCK_MS:
            lock_side = 0

        if not pair_white or ahead:                 # 라인 보임 (페어 사이 틈 포함)
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
            g = (KG * (gyro.speed() - GBIAS)) // 1000
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
                    ev3.light.on(Color.YELLOW)
                    bank_dir = lock_side if lock_side != 0 else side
                    turned = 0
                    flips = 0
                elif SEARCH_DEG > 0:
                    # 한정 탐색: 뱅킹 방향으로 돈 각도를 자이로로 적분 (좌회전 + 레이트)
                    turned -= bank_dir * (gyro.speed() - GBIAS) * dt
                    if flips < 2 and turned > SEARCH_DEG * 1000:
                        bank_dir = -bank_dir        # 그쪽엔 라인이 없음 → 반대로
                        turned = 0
                        flips += 1
                steer = bank_dir * ST_BANK
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

        steering_dc(steer, power)


GBIAS = gyro_bias()
print("자이로 바이어스", GBIAS)
t_run = StopWatch()
try:
    run()
finally:                    # 정지 버튼(SystemExit)에서도 실행됨
    print("STAT,rl,steps,%d,ms,%d" % (STAT[0], t_run.time()))

# 페어+센티넬 배치용 상수 (core 12월드 사진 트랙 9/12 20.2s, 1.6배 0/12):
#   KP 0.265 KD 5.181 KG 99 G_CLAMP 46 ALPHA 866 D_FILTER 341 MAX_STEER_DELTA 57 W_NORM 77
#   SIDE_ERR 88 GAP_MS 9 AHEAD_HOLD_MS 12 SEARCH_DEG 110 ST_BANK 72 RECOVER_POWER 100
#   SPD_K10 8 MIN_POWER 62 PB_MS 210 PB_POWER 52
