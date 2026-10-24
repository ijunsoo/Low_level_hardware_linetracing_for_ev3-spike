# =====================================================================
#  MRC 라인트레이싱 v4  ── EV3 컬러센서용 / 옛 dc 코어 + 프레임동기(루프=센서속도)
# ---------------------------------------------------------------------
#  [센서 = EV3 컬러센서, 리서치 근거]
#   1) EV3 UART: 핸드셰이크 2400→57600 baud. 데이터프레임 전송규칙 =
#      "최소 1ms, 최대 100ms마다 1개" (leJOS/Pybricks UART 프로토콜 문서).
#      즉 새 값은 1kHz~10Hz 사이로, 실측 관행은 ~100Hz대 + 지연 큼(ev3dev 10ms).
#      → SPIKE 센서보다 느리고 지연 큼.
#   2) PUPDevice.read()는 블로킹 아님 → 허브 캐시의 '가장 최근 프레임'을 즉시 반환.
#   3) 따라서 Prime 허브 제어루프는 EV3 새프레임보다 빨라 '같은 값'을 여러 번 읽음.
#      중복 프레임 위에서 미분(diff) 계산 → D항이 0/스파이크 반복 → 떨림.
#   4) StopWatch 해상도 1ms → per-cycle dt 못 잼 → per-'프레임' PID가 정답.
#
#  [해법] 루프는 최대속도로 돌리되, '센서값이 바뀐 프레임'에서만 PID 1스텝 진행
#         (= 제어율을 센서 새프레임율에 동기). 중복 프레임은 모터 유지만.
#         이탈은 옛날처럼 '즉시'(전부 흰색이면 바로 뱅킹) — 틈브리지 제거됨.
#  [시작 시] 루프Hz + 센서 실제 갱신Hz 둘 다 측정·출력(튜닝 기준 제공).
#  [유지] dc() 풀파워 / 정수연산 / 옛 게인.
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
# --- 캘리브레이션(흰/검 기준값) --- #
# 2026-07-04 라인스캔 실측으로 갱신 (구값: [64,9, 52,7, 53,6, 61,8] / W_OFFSET 17)
VALUE    = [45, 5, 42, 5, 31, 3, 25, 2]   # [w1,b1, w2,b2, w3,b3, w4,b4]
W_OFFSET = 8                               # 이탈판정 흰색 여유

# --- 속도/파워 --- #
POWER = 100              # dc 듀티(100=풀파워, 가장 빠름). 코스 못 따라가면 ↓
RECOVER_POWER = 70       # 진짜(가장자리)이탈 복구 뱅킹 시 파워. ↓일수록 오버슈트↓(천천히 복구)

# --- 조향 PID(옛 검증값. 이제 '프레임당'이라 의미가 깨끗함) --- #
KP = 0.26
KI = 0.00017
KD = 3.0
I_LIMIT = 15    # ★ 적분항이 조향에 기여하는 최대치(±). 코너후 느린 휘젓음 심하면 ↓
                #   (이전엔 사실상 ±93까지 풀려서 휘젓음의 주범이었음)

# --- 조향 보조 --- #
ST_BANK         = 75    # 진짜 이탈시 마지막 방향으로 꺾는 뱅킹량
MAX_STEER_DELTA = 15    # 1스텝당 조향 변화 상한(슬루; 클수록 민첩/거칢)
EMA_ALPHA       = 700   # /1000 센서 평활(700=0.7새+0.3이전). ↑빠름/거칢, ↓느림/매끈
D_FILTER        = 300   # /1000 ★미분 전용 필터. ↓일수록 D 매끈=떨림↓(반응 약간↓), ↑빠름

# --- 이탈감지(★위치로 구분: 중앙틈 vs 가장자리이탈) --- #
#   전부 흰색이 GAP_LIMIT 프레임 이내면 → 거짓이탈로 보고 직전 조향 유지.
GAP_LIMIT = 10
#   GAP_LIMIT 넘어도, 사라지기 직전 오차로 '중앙틈/가장자리이탈' 구분:
#     |last_error| < EDGE_ERR → '중앙'에서 사라짐 = 가운데 센서틈에 빠짐
#         → 라인은 직진 앞에 있음 → 뱅킹 금지, 거의 직진 유지(휘젓음 차단 핵심).
#     |last_error| >= EDGE_ERR → '가장자리'에서 사라짐 = 코너 진짜 이탈 → 뱅킹.
#   휘저으면 EDGE_ERR ↑(중앙판정 넓힘) / 코너에서 라인 놓치면 ↓.
EDGE_ERR = 30

# --- 디버그/튜닝 계측 --- #
DEBUG       = True      # True면 error/steer/off 주기 출력(BT→IDE)
DEBUG_EVERY = 30
BENCH_MS    = 800       # 시작시 루프속도/센서갱신율 측정 창(ms)
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


# --------------------------- 함수 --------------------------- #

# dc함수를 조향값으로 제어 (옛날 그대로 = 가장 빠름)
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


# ★ 루프속도 + 센서 실제 갱신율 동시 실측
#   loop_hz : 4센서 읽는 루프가 도는 속도
#   upd_hz  : 읽은 값이 '실제로 바뀐' 횟수/초 = EV3 센서가 새 프레임 주는 속도(근사)
#   보통 loop_hz >> upd_hz → 그래서 '바뀐 프레임에서만' 제어하는 게 맞다는 증거.
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


# 4개 센서값 균형잡기
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


# 라인트레이싱 (프레임 동기형)
def pid_control(error_offset):
    wh1, wh2, wh3, wh4 = WH[0], WH[1], WH[2], WH[3]
    b1,  b2,  b3,  b4  = BK[0], BK[1], BK[2], BK[3]

    integral   = 0
    last_error = 0
    d_filt     = 0          # ★ 필터링된 미분값(떨림 억제)

    repower   = POWER
    direction = 0
    was_off   = False
    prev_steering = 0
    gap        = 0          # ★ 연속 '전부 흰색' 프레임수
    last_good_steer = 0     # ★ 마지막 정상추종 조향(틈 브리지에서 이어감)

    l_black = (wh1 + b1) // 2
    r_black = (wh4 + b4) // 2
    cross   = (b1 + b2 + b3 + b4) // 4 + 20
    w1 = wh1 - W_OFFSET
    w2 = wh2 - W_OFFSET
    w3 = wh3 - W_OFFSET
    w4 = wh4 - W_OFFSET

    r1 = c1.read(mode=0)[0]
    r2 = c2.read(mode=0)[0]
    r3 = c3.read(mode=0)[0]
    r4 = c4.read(mode=0)[0]
    raw_prev = (r1, r2, r3, r4)
    ps1, ps2, ps3, ps4 = r1, r2, r3, r4

    dbg = 0

    while True:
        r1 = c1.read(mode=0)[0]
        r2 = c2.read(mode=0)[0]
        r3 = c3.read(mode=0)[0]
        r4 = c4.read(mode=0)[0]
        raw = (r1, r2, r3, r4)

        # ★ 새 프레임(값 변화)에서만 1스텝 진행 → 중복 프레임 스킵(모터 유지)
        if raw == raw_prev:
            continue
        raw_prev = raw

        # ── 프레임 1스텝 처리 ── #
        s1 = (EMA_ALPHA * min(max(r1, b1), wh1) + A_COMP * ps1) // E_S
        s2 = (EMA_ALPHA * min(max(r2, b2), wh2) + A_COMP * ps2) // E_S
        s3 = (EMA_ALPHA * min(max(r3, b3), wh3) + A_COMP * ps3) // E_S
        s4 = (EMA_ALPHA * min(max(r4, b4), wh4) + A_COMP * ps4) // E_S
        ps1, ps2, ps3, ps4 = s1, s2, s3, s4

        # ★ 이탈 판정은 '생(raw)' 값으로 → EMA 지연 0, 흰색 뜨는 즉시 트리거
        raw_off = (r1 > w1 and r2 > w2 and r3 > w3 and r4 > w4)
        error = (s1 + s2) - (s3 + s4) - error_offset   # 오차/조향은 평활값 유지

        if not raw_off:
            # ── 라인 봄: 정상 추종 ── #
            if was_off:
                # 진짜 이탈에서 복구 직후 재포착 → 미분 튐 방지
                last_error = error
                integral   = 0
                d_filt     = 0
            was_off = False
            gap = 0

            if s1 < l_black:
                direction = 0
            elif s4 < r_black:
                direction = 1

            integral += error
            if KI_S != 0:
                # ★ 적분 '기여'를 ±I_LIMIT 조향으로 제한 → 코너 윈드업/휘젓음 차단
                max_integral = I_LIMIT * S // KI_S
                if integral > max_integral:
                    integral = max_integral
                elif integral < -max_integral:
                    integral = -max_integral

            diff     = error - last_error
            # ★ 미분 전용 필터: D항 노이즈만 죽임(P는 그대로 빠릿)
            d_filt   = (D_FILTER * diff + D_COMP * d_filt) // E_S
            steering = (KP_S * error + KI_S * integral + KD_S * d_filt) // S
            last_error = error

            if s1 + s2 + s3 + s4 < cross:   # 교차점(전부 검정) → 직진
                steering = 0
            repower = POWER
            last_good_steer = steering      # ★ 브리지에서 이어갈 조향 저장
        else:
            # ── 전부 흰색(라인 안 보임) ── #
            gap += 1
            if gap <= GAP_LIMIT:
                # 짧은 틈(거짓이탈) → 직전 조향 유지
                steering = last_good_steer
                repower  = POWER
            elif abs(last_error) < EDGE_ERR:
                # ★ '중앙'에서 사라짐 = 가는 라인이 가운데 센서틈에 빠짐
                #   → 라인은 거의 직진 앞 → 뱅킹 금지, 직전(거의 직진) 조향 유지
                steering = last_good_steer
                repower  = POWER
            else:
                # ★ '가장자리'에서 사라짐 = 코너 진짜 이탈 → 그쪽으로 뱅킹(속도↓ 오버슈트↓)
                was_off  = True
                integral = 0
                steering = -ST_BANK if direction == 0 else ST_BANK
                repower  = RECOVER_POWER

        # 슬루 제한
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

# 1) 루프속도 + 센서 실제 갱신율 실측 (튜닝 기준)
hub.speaker.beep(700, 80)
loop_hz, upd_hz = benchmark(BENCH_MS)
print("LOOP", loop_hz, "Hz / SENSOR_UPDATE ~", upd_hz, "Hz")
hub.speaker.beep(min(2000, 400 + upd_hz * 4), 120)

# 2) 센서 균형 캘리브레이션
error_offset = round(calibrate_error_offset(20))
print("error_offset", error_offset)
hub.speaker.beep(1200, 100)

# 3) 주행
pid_control(error_offset)
