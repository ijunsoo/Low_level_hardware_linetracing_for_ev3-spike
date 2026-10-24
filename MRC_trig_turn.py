from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Direction, Port, Stop, Axis
from pybricks.tools import wait

# =============================================================
#  MRC 정밀 턴 (삼각함수 원리) — SPIKE Prime + Pybricks
#
#  로봇 실측 기하:
#    전장 155mm, 바퀴 지름 56mm, 좌우 바퀴 중심거리(윤거) 82mm
#
#  사용하는 수학 (고2 '대수' — 삼각함수 단원):
#   [1] 일반각과 호도법:  θ(rad) = θ(°) × π/180
#   [2] 호의 길이:        s = r·θ   (θ는 라디안)
#       → 로봇이 θ만큼 돌 때 각 바퀴가 굴러야 할 거리가 '호의 길이'
#   [3] 단위원 사인·코사인 정의: 턴 후 로봇 위치 예측
#       → Δ전방 = R·sinθ,  Δ측방 = R·(1 − cosθ)
#   [4] 코사인법칙: c² = a² + b² − 2ab·cosC
#       → 피벗 회전 시 로봇 모서리가 그리는 최대 반지름 (25cm 규정 검증)
#   [5] 주기성(일반각): 헤딩 오차를 −180°~+180° 범위로 정규화
# =============================================================

try:
    import umath as math          # Pybricks(마이크로파이썬)
except ImportError:
    import math                   # PC/시뮬 호환

# ------------------------- 실측 기하 (mm) ------------------------- #
WHEEL_D  = 56.0                   # 바퀴 지름
TRACK    = 82.0                   # 좌우 바퀴 중심 간 거리 (회전의 지름)
BODY_LEN = 155.0                  # 전장 (스윕 반경 계산용)
GEAR     = 2.0                    # 모터→바퀴 증속비 (바퀴각 = 모터각 × GEAR)
                                  # ※ 직결이면 1.0. 외접기어 단수에 따라
                                  #   회전방향이 뒤집히면 Direction을 교체할 것.
GYRO_SIGN = 1                     # calibrate_turn()이 자동 측정해 덮어씀.
                                  # (실기 로그 2026-07-11: 수동 기본값 -1이 틀려서
                                  #  트림 루프가 ±180° 반대편을 목표로 진동했음 —
                                  #  이제 시작 시 +45° 시험 피벗으로 자가판별)

# ------------------------- 장치설정 ------------------------- #
hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)


# ================= [1]+[2] 호도법과 호의 길이 ================= #
def wheel_deg_for_arc(arc_mm):
    """바퀴가 arc_mm만큼 구르는 데 필요한 '모터' 회전각(°).

    바퀴 원주 = π·D 이므로, 바퀴 회전각 = 360 × (호의 길이 / 원주).
    증속기어가 있으면 모터는 그 1/GEAR 만 돌면 된다.
    """
    wheel_deg = 360.0 * arc_mm / (math.pi * WHEEL_D)
    return wheel_deg / GEAR


def pivot_wheel_arc(turn_deg):
    """제자리(피벗) 턴에서 각 바퀴가 굴러야 할 호의 길이 s = r·θ.

    피벗 턴의 회전중심 = 두 바퀴의 가운데.
    각 바퀴는 반지름 r = TRACK/2 인 원 위를 움직이므로
        s = (TRACK/2) × θ(rad)          ← [2] 호의 길이 공식
    """
    theta = math.radians(abs(turn_deg))        # [1] 호도법 변환
    return (TRACK / 2.0) * theta


# ================= [3] 사인·코사인으로 위치 예측 ================= #
def predict_arc_move(turn_deg, radius_mm):
    """반지름 R 원호를 θ만큼 돌았을 때 로봇의 변위 예측.

    회전중심을 원점에 둔 단위원 정의 그대로:
        전방 이동  Δx = R·sinθ
        측방 이동  Δy = R·(1 − cosθ)
    θ=90°면 (R, R), θ=180°면 (0, 2R) — 그래프로 그려보면 직관과 일치.
    """
    theta = math.radians(abs(turn_deg))
    dx = radius_mm * math.sin(theta)
    dy = radius_mm * (1.0 - math.cos(theta))
    return dx, dy


# ================= [4] 코사인법칙: 스윕 반경 ================= #
def pivot_sweep_radius(front_mm=BODY_LEN / 2.0, half_width_mm=TRACK / 2.0 + 20):
    """피벗 턴 때 로봇 앞모서리가 그리는 원의 반지름.

    회전중심(축 가운데)에서 '앞으로 a, 옆으로 b' 떨어진 모서리까지의
    거리 c는 코사인법칙으로:
        c² = a² + b² − 2ab·cos90°  (cos90° = 0)
        c  = √(a² + b²)
    → 25cm 상자 규정: 스윕 지름 2c ≤ 250mm 인지 시작 전에 확인.
    """
    a, b = front_mm, half_width_mm
    c = math.sqrt(a * a + b * b - 2 * a * b * math.cos(math.pi / 2.0))
    return c


# ================= [5] 일반각: 헤딩 오차 정규화 ================= #
def norm_angle(deg):
    """각도를 −180°~+180°로 정규화 (삼각함수의 주기 360° 이용)."""
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def read_heading():
    """자이로 헤딩(°). 좌회전 = +가 되도록 부호 통일."""
    return GYRO_SIGN * hub.imu.heading()


def _ff_pivot(turn_deg, speed):
    """피드포워드 피벗: 삼각함수로 계산한 모터각만큼만 회전 (보정 없음)."""
    arc = pivot_wheel_arc(turn_deg)                 # s = r·θ
    motor_deg = wheel_deg_for_arc(arc)              # 호 길이 → 모터각
    sign = 1 if turn_deg > 0 else -1                # 좌턴: 좌바퀴 후진
    lm.run_angle(speed, -sign * motor_deg, wait=False)
    rm.run_angle(speed, sign * motor_deg, wait=True)
    while not lm.done():
        wait(5)


# ================= 자이로 부호·스케일 자가판별 ================= #
def calibrate_turn(speed=200):
    """+45° 시험 피벗으로 (1)자이로 부호 (2)회전 스케일을 실측.

    부호: 명령 +45(좌)에 heading()이 어느 방향으로 움직이는지 측정.
        (하드웨어 조립·기어 단수·펌웨어에 따라 달라 자동 측정이 안전)
    스케일: |실측/45|가 1.0에서 크게 벗어나면 GEAR 설정 오류
        (0.5배면 GEAR가 실제의 2배, 2배면 절반으로 설정된 것).
    측정 후 −45°로 되돌린다.
    """
    global GYRO_SIGN
    raw0 = hub.imu.heading()
    _ff_pivot(45, speed)
    wait(300)                                       # 진동 안정 대기
    delta = hub.imu.heading() - raw0
    _ff_pivot(-45, speed)
    wait(300)

    if abs(delta) < 15:
        hub.speaker.beep()
        print("경고: 시험 피벗 45도에 자이로 %.1f도 — 모터/기어 확인 필요" % delta)
        return False
    GYRO_SIGN = 1 if delta > 0 else -1
    scale = abs(delta) / 45.0
    print("자가판별: GYRO_SIGN=%d, 스케일 %.2f" % (GYRO_SIGN, scale))
    if scale < 0.85 or scale > 1.15:
        hub.speaker.beep()
        print("경고: 스케일 %.2f — GEAR=%.1f 재확인 (0.5배면 GEAR 과대, 2배면 과소)"
              % (scale, GEAR))
    return True


# ======================= 정밀 피벗 턴 ======================= #
def pivot_turn(turn_deg, speed=300, trim=True):
    """제자리에서 turn_deg만큼 정밀 회전 (+ = 좌회전/반시계).

    1단계(피드포워드): 삼각함수로 계산한 모터각을 그대로 명령.
        바퀴 호의 길이 s = (TRACK/2)·θrad  →  모터각으로 환산.
        (약분하면 바퀴각 = θ × TRACK/WHEEL_D — 90°턴이면 90×82/56 ≈ 132°)
    2단계(피드백): 자이로 잔차를 P제어로 미세 보정 (기어 백래시·
        바퀴 미끄러짐 등 수학 모델 밖의 오차 흡수).
    """
    start = read_heading()
    la0, ra0 = lm.angle(), rm.angle()

    # -- 1단계: 계산된 각도만큼 양쪽 바퀴 반대 방향 -- #
    _ff_pivot(turn_deg, speed)
    ff = read_heading()
    print("  [FF] 자이로 %.1f→%.1f, 모터 L%+d R%+d"
          % (start, ff, lm.angle() - la0, rm.angle() - ra0))

    # -- 2단계: 자이로 잔차 보정 (P제어, 데드존 하한 duty 20) -- #
    if not trim:
        return
    err = norm_angle((start + turn_deg) - ff)
    if abs(err) > 120:      # 안티포드 가드: 부호 설정이 틀렸다는 뜻 —
        hub.speaker.beep()  # 반대편을 향해 싸우다 진동하므로 트림 포기
        print("  경고: 잔차 %.0f도 — 부호/기어 설정 오류, 트림 생략" % err)
        return
    for _ in range(400):                            # 최대 ~2초
        err = norm_angle((start + turn_deg) - read_heading())
        if abs(err) < 0.7:                          # 허용 오차 0.7°
            break
        duty = max(20.0, min(60.0, abs(err) * 2.5))
        d = 1 if err > 0 else -1                    # +err = 더 좌회전 필요
        lm.dc(-d * duty)
        rm.dc(d * duty)
        wait(5)
    lm.brake()
    rm.brake()


# ======================= 정밀 원호 턴 ======================= #
def arc_turn(turn_deg, radius_mm, speed=400):
    """반지름 radius_mm 원호를 따라 turn_deg 회전 (+ = 좌회전).

    회전중심에서 바깥쪽/안쪽 바퀴까지의 반지름이 다르므로
    호의 길이도 다르다 ([2]를 두 번 적용):
        s_바깥 = (R + TRACK/2)·θrad
        s_안쪽 = (R − TRACK/2)·θrad
    두 바퀴가 '동시에' 끝나야 원호가 되므로 속도도 호 길이에 비례.
    radius_mm는 TRACK/2(=41mm)보다 커야 양쪽 다 전진.
    """
    theta = math.radians(abs(turn_deg))             # [1]
    r_out = radius_mm + TRACK / 2.0
    r_in  = radius_mm - TRACK / 2.0
    s_out, s_in = r_out * theta, r_in * theta       # [2]
    deg_out = wheel_deg_for_arc(s_out)
    deg_in  = wheel_deg_for_arc(s_in)
    spd_in  = speed * s_in / s_out                  # 속도비 = 호 길이비

    dx, dy = predict_arc_move(turn_deg, radius_mm)  # [3] 위치 예측
    print("예측 변위: 전방 %dmm, 측방 %dmm" % (round(dx), round(dy)))

    la0, ra0 = lm.angle(), rm.angle()
    if turn_deg > 0:      # 좌턴: 오른바퀴가 바깥
        rm.run_angle(speed, deg_out, wait=False)
        lm.run_angle(spd_in, deg_in, wait=True)
        while not rm.done():
            wait(5)
    else:                 # 우턴: 왼바퀴가 바깥
        lm.run_angle(speed, deg_out, wait=False)
        rm.run_angle(spd_in, deg_in, wait=True)
        while not lm.done():
            wait(5)
    lm.brake()
    rm.brake()
    print("  [FF] 모터 실회전 L%+d R%+d (명령 안 %.0f / 바깥 %.0f)"
          % (lm.angle() - la0, rm.angle() - ra0, deg_in, deg_out))


# --------------------------- 데모 --------------------------- #
if __name__ == "__main__":
    hub.imu.reset_heading(0)
    wait(200)

    sweep = pivot_sweep_radius()
    print("피벗 스윕 반경 %.0fmm (지름 %.0fmm / 규정 250mm)" % (sweep, 2 * sweep))

    calibrate_turn()          # ★자이로 부호·스케일 자가판별 (+45° 시험 피벗)
    hub.imu.reset_heading(0)
    wait(200)

    print("90도 좌 피벗 (계산 바퀴각 %.1f도)" % (90 * TRACK / WHEEL_D))
    pivot_turn(90)
    print("  자이로 실측:", read_heading())
    wait(500)

    print("180도 우 피벗")
    pivot_turn(-180)
    print("  자이로 실측:", read_heading())
    wait(500)

    print("반경 150mm, 90도 좌 원호턴")
    arc_turn(90, 150)
    print("  자이로 실측:", read_heading())
