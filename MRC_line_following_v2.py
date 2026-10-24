# =====================================================================
#  MRC 라인트레이싱 v4  (SPIKE Prime / Pybricks) — 라인위치 추정기(Estimator)
# ---------------------------------------------------------------------
#  근본문제(데이터+리서치 확정): 라인이 가늘어 센서 사이 '틈'으로 사라짐(sum=0) →
#    위치신호 불연속 → 거짓이탈 → 복구핑퐁 → 심한 흔들림. PID로는 못 잡음(센싱 문제).
#  해법(논문·a1k0n 칼만필터·차량횡제어 공통): 라인이 틈으로 사라져도 '이탈' 아님 →
#    마지막 추정 위치를 '유지(dead-reckoning)'하며 PID 지속. 센서 잡으면 추정 갱신.
#    → PID는 '연속 추정위치 line_pos'를 추종 → 거짓이탈/핑퐁 원천 제거.
#  ⚠ 하드웨어 팁: 흰색값 낮음(센서 너무 높음) → 센서를 표면 16mm까지 내리면 틈↓.
#  속도단위 = 모터 deg/s (2배기어라 바퀴는 2배)
# =====================================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port
from pybricks.tools import wait, StopWatch

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm  = Motor(Port.E)                              # 좌 구동모터
rm  = Motor(Port.F, Direction.COUNTERCLOCKWISE)  # 우 구동모터
lm.control.limits(acceleration=20000)
rm.control.limits(acceleration=20000)
c1  = PUPDevice(Port.A)   # 좌측부터 1 컬러센서
c2  = PUPDevice(Port.C)   # 2 컬러센서
c3  = PUPDevice(Port.B)   # 3 컬러센서
c4  = PUPDevice(Port.D)   # 4 컬러센서
SENSORS = (c1, c2, c3, c4)

# ===================== 튜닝 파라미터(여기만 만지면 됨) ===================== #
VALUE    = [33, 3, 30, 3, 21, 1, 23, 2]   # [w1,b1, w2,b2, w3,b3, w4,b4]
W_OFFSET = 17

DEBUG = False           # True면 pos/meas/gap/err 출력

CONTROL_MS = 5          # 제어주기(ms)

# --- ★라인위치 추정기 (핵심) --- #
DETECT    = 150         # sum 이상이면 '라인 봄'=측정사용. 미만이면 '틈'=추정유지. 노이즈로 틈오판 잦으면 ↓
MEAS_GAIN = 0.5         # 측정으로 추정 보정속도(0~1). 클수록 센서 즉답(덜 평활), 작을수록 부드러움
HOLD_VEL  = 0.5         # 틈일때 직전 속도로 이어가는 정도(0=완전정지유지, 1=완전외삽). 곡선 틈 통과용
REAL_LOST = 30          # 이 틱수 연속 '틈'이면 진짜 이탈(복구). 200Hz면 30=150ms
LOST_POS  = 190         # 추정위치가 ±이값 넘으면 라인이 어레이 밖 = 진짜 이탈

# --- 속도 (모터 deg/s) --- #
MAX_SPEED = 1000
MIN_SPEED = 750
SLOW_K    = 3.5
BRAKE_K   = 1.0
SPEED_UP_RATE = 30

# --- 조향 PID (연속 추정위치에 작용 → KD 깨끗하게 먹음) --- #
KP = 0.45
KI = 0.3
KD = 9.0
D_FILTER  = 0.5
I_LIMIT   = 45
I_LEAK    = 0.99

MAX_STEER_DELTA = 40

# --- 급코너(직각) --- #
SHARP_N     = 680
SHARP_ERR   = 100
SHARP_BOOST = 100
SHARP_SPEED = 350

S_ALPHA = 0.45

CROSS_SUM = 3200
RECOVER_STEER = 90      # 복구 뱅킹
RECOVER_SPEED = 650

POS = (-150, -50, 50, 150)
# ====================================================================== #

W_REF = [VALUE[0], VALUE[2], VALUE[4], VALUE[6]]
B_REF = [VALUE[1], VALUE[3], VALUE[5], VALUE[7]]
SPAN  = [max(1, W_REF[i] - B_REF[i]) for i in range(4)]


# --------------------------- 함수 --------------------------- #

def drive(steering, speed):
    if steering > 200:    steering = 200
    elif steering < -200: steering = -200
    if steering < 0:
        sl = speed * (1 - (-steering) / 100.0)
        sr = speed
    else:
        sl = speed
        sr = speed * (1 - steering / 100.0)
    lm.run(sl)
    rm.run(sr)


def read_blackness(ema):
    n = [0, 0, 0, 0]
    for i in range(4):
        r = SENSORS[i].read(mode=0)[0]
        f = S_ALPHA * r + (1 - S_ALPHA) * ema[i]
        ema[i] = f
        bl = (W_REF[i] - f) * 1000 // SPAN[i]
        if bl < 0:      bl = 0
        elif bl > 1000: bl = 1000
        n[i] = bl
    return n


def calibrate_offset(samples=20):
    ema = [SENSORS[i].read(mode=0)[0] for i in range(4)]
    total = 0
    for _ in range(samples):
        n = read_blackness(ema)
        s = n[0] + n[1] + n[2] + n[3]
        if s > 0:
            total += (POS[0]*n[0] + POS[1]*n[1] + POS[2]*n[2] + POS[3]*n[3]) / s
        wait(20)
    return total / samples


# --------------------------- 메인 제어 --------------------------- #

def line_follow(offset):
    ema = [SENSORS[i].read(mode=0)[0] for i in range(4)]
    line_pos   = 0.0        # ★추정 라인위치(연속)
    line_vel   = 0.0        # 추정 위치속도(틈 외삽·미분용)
    last_error = 0.0
    integral   = 0.0
    d_filt     = 0.0
    prev_steer = 0.0
    prev_speed = MIN_SPEED
    direction  = 0
    gap        = 0          # 연속 '틈' 틱수
    clock = StopWatch()
    last_ctrl = clock.time()
    dbg_n = 0

    while True:
        n = read_blackness(ema)            # 매 루프 평활
        now = clock.time()

        if now - last_ctrl < CONTROL_MS:   # 제어주기 게이트
            continue
        dt = now - last_ctrl
        last_ctrl = now

        sum_n = n[0] + n[1] + n[2] + n[3]

        # ===== 라인위치 추정기 ===== #
        if sum_n >= DETECT:
            measured = (POS[0]*n[0] + POS[1]*n[1] + POS[2]*n[2] + POS[3]*n[3]) / sum_n
            prev = line_pos
            line_pos += MEAS_GAIN * (measured - line_pos)   # 측정으로 보정(평활)
            line_vel = 0.6 * line_vel + 0.4 * (line_pos - prev)
            gap = 0
        else:
            gap += 1
            line_pos += HOLD_VEL * line_vel   # 틈: 직전 속도로 이어감(dead-reckon)
            line_vel *= 0.9                   # 신뢰도 감쇠
            if line_pos > 200:    line_pos = 200
            elif line_pos < -200: line_pos = -200

        if n[0] > 500:   direction = 0
        elif n[3] > 500: direction = 1

        if DEBUG:
            dbg_n += 1
            if dbg_n >= 8:
                print("pos", round(line_pos), "gap", gap, "err", round(last_error), "st", round(prev_steer))
                dbg_n = 0

        # ===== 진짜 이탈(오래 사라짐 OR 어레이밖) → 복구 뱅킹 ===== #
        if gap > REAL_LOST or abs(line_pos) > LOST_POS:
            integral = 0.0
            steer = -RECOVER_STEER if direction == 0 else RECOVER_STEER
            d = steer - prev_steer
            if d > MAX_STEER_DELTA:    steer = prev_steer + MAX_STEER_DELTA
            elif d < -MAX_STEER_DELTA: steer = prev_steer - MAX_STEER_DELTA
            prev_steer = steer
            drive(steer, RECOVER_SPEED)
            continue

        # ===== PID (연속 추정위치 사용) ===== #
        cross = sum_n > CROSS_SUM
        error = 0.0 if cross else (line_pos - offset)
        if cross:
            integral = 0.0

        raw_d = (error - last_error) * 1000.0 / dt
        d_filt = D_FILTER * raw_d + (1 - D_FILTER) * d_filt
        last_error = error
        integral = integral * I_LEAK + error * dt / 1000.0
        if integral > I_LIMIT:    integral = I_LIMIT
        elif integral < -I_LIMIT: integral = -I_LIMIT
        steer = KP * error + KI * integral + KD * d_filt * 0.001

        # 속도 프로파일
        speed = MAX_SPEED - SLOW_K * abs(error) - BRAKE_K * abs(d_filt)
        if speed > MAX_SPEED:   speed = MAX_SPEED
        elif speed < MIN_SPEED: speed = MIN_SPEED

        # 급코너(직각)만
        hard_left  = n[0] > SHARP_N and n[2] < 250 and n[3] < 250
        hard_right = n[3] > SHARP_N and n[1] < 250 and n[0] < 250
        if abs(error) > SHARP_ERR and (hard_left or hard_right):
            steer += SHARP_BOOST if error > 0 else -SHARP_BOOST
            speed = SHARP_SPEED

        if speed > prev_speed + SPEED_UP_RATE:
            speed = prev_speed + SPEED_UP_RATE
        prev_speed = speed

        d = steer - prev_steer
        if d > MAX_STEER_DELTA:    steer = prev_steer + MAX_STEER_DELTA
        elif d < -MAX_STEER_DELTA: steer = prev_steer - MAX_STEER_DELTA
        prev_steer = steer

        drive(steer, speed)


# --------------------------- 실행 --------------------------- #
hub.speaker.beep(800, 100)
offset = calibrate_offset(20)
hub.speaker.beep(1200, 100)

line_follow(offset)
