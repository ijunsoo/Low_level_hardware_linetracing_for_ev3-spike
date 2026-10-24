# =====================================================================
#  EV3 3센서 목 경로 스모크 테스트 (프로파일 ev3_3s, Pybricks v4 API)
#  EV3Brick + ev3devices Motor 4개 + ColorSensor 3개 + GyroSensor(S4).
#  v4 API 충실도(구 API가 실기처럼 실패하는지)와 배선·부호를 점검한다.
#    python3 sim/run_sim.py sim/tests/ev3_smoke.py --profile ev3_3s
# =====================================================================
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()

# ── v4 API 충실도: v2/v1 전용 코드는 실기 v4에서처럼 시뮬에서도 실패해야 함 ──
try:
    from pybricks.media.ev3dev import SoundFile   # noqa: F401  (v2 전용)
    raise AssertionError("v4에는 pybricks.media.ev3dev가 없어야 함")
except ImportError:
    pass

LM = (Motor(Port.A), Motor(Port.B, Direction.COUNTERCLOCKWISE))
RM = (Motor(Port.C, Direction.COUNTERCLOCKWISE), Motor(Port.D))
try:
    LM[0].stop(Stop.BRAKE)                        # v1 API → v2/v4에선 TypeError
    raise AssertionError("stop()은 인자를 받지 않아야 함")
except TypeError:
    pass

cL, cR, cF = ColorSensor(Port.S1), ColorSensor(Port.S2), ColorSensor(Port.S3)
gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)   # 좌회전 + (v16 규약)
try:
    ColorSensor(Port.S4)
    raise AssertionError("S4에는 컬러센서가 없어야 함")
except OSError:
    pass

VALUE = [26, 2, 25, 2, 28, 2]     # [흰, 검] × (L, R, F)


def norm(r, i):
    w, b = VALUE[2 * i], VALUE[2 * i + 1]
    return (min(max(r, b), w) - b) * 100 // (w - b)


def drive(steering, power):
    steering = max(-100, min(100, steering))
    slope = (power * 2) / 100
    if steering < 0:
        l, r = round(slope * steering + power, 2), power
    else:
        l, r = power, round(-slope * steering + power, 2)
    for m in LM:
        m.dc(l)
    for m in RM:
        m.dc(r)


# ── 배선·부호 점검 ──
drive(0, 50)
wait(200)
assert all(m.angle() > 0 for m in LM + RM), [m.angle() for m in LM + RM]
assert all(m.speed() > 0 for m in LM + RM), [m.speed() for m in LM + RM]
drive(-100, 50)                       # 제자리 좌회전
wait(120)
assert gyro.speed() > 0, ("좌회전인데 자이로 레이트가 +가 아님", gyro.speed())
drive(100, 50)                        # 되돌리기
wait(120)
for m in LM + RM:
    m.brake()
wait(200)

KP10, KG, ST_BANK, BANK_POWER = 3, 200, 65, 60
side, t_ahead = 1, 0
sw = StopWatch()
while True:
    nL = norm(cL.reflection(), 0)
    nR = norm(cR.reflection(), 1)
    nF = norm(cF.reflection(), 2)
    now = sw.time()
    if nF < 50:
        t_ahead = now
    e = 2 * (nL - nR)
    if e > 60:
        side = 1
    elif e < -60:
        side = -1
    lost = nL > 82 and nR > 82 and now - t_ahead > 30
    if lost:
        drive(side * ST_BANK, BANK_POWER)
    else:
        g = KG * gyro.speed() // 1000
        drive(KP10 * e // 10 + max(-40, min(40, g)), 100)
