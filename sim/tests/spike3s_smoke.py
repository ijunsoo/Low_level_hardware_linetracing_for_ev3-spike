# =====================================================================
#  SPIKE Prime 3센서 목 경로 스모크 테스트 (프로파일 spike_3s)
#  실기 API 그대로: PrimeHub + pupdevices.Motor + PUPDevice 3개 + 허브 IMU.
#  성능용 컨트롤러가 아니라 배선·부호·3센서 읽기 경로 점검용 최소 로직.
#    python3 sim/run_sim.py sim/tests/spike3s_smoke.py --profile spike_3s
# =====================================================================
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Direction, Port, Axis
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
lm = Motor(Port.E)
rm = Motor(Port.F, Direction.COUNTERCLOCKWISE)
cL = PUPDevice(Port.C)    # 페어 좌 (전방 90mm, 측방 +12.5)
cR = PUPDevice(Port.B)    # 페어 우 (전방 90mm, 측방 -12.5)
cF = PUPDevice(Port.A)    # 전방 센티넬 (전방 130mm, 중앙)

VALUE = [26, 2, 25, 2, 28, 2]     # [흰, 검] × (L, R, F)


def norm(r, i):
    w, b = VALUE[2 * i], VALUE[2 * i + 1]
    return (min(max(r, b), w) - b) * 100 // (w - b)


def steering_dc(steering, power):
    steering = max(-100, min(100, steering))
    slope = (power * 2) / 100
    if steering < 0:
        lm.dc(round(slope * steering + power, 2))
        rm.dc(power)
    else:
        lm.dc(power)
        rm.dc(round(-slope * steering + power, 2))


# 배선·부호 점검: 짧게 전진하면 양쪽 엔코더가 +, 직진이면 요레이트 ≈ 0
steering_dc(0, 40)
wait(200)
assert lm.angle() > 0 and rm.angle() > 0, ("엔코더 부호", lm.angle(), rm.angle())
assert abs(hub.imu.angular_velocity(Axis.Z)) < 20, "직진 중 요레이트가 큼"

KP10, KG, ST_BANK, BANK_POWER = 3, 200, 65, 60
side, t_ahead = 1, 0
sw = StopWatch()
while True:
    nL = norm(cL.read(0)[0], 0)
    nR = norm(cR.read(0)[0], 1)
    nF = norm(cF.read(0)[0], 2)
    now = sw.time()
    if nF < 50:
        t_ahead = now                    # 앞에 라인 있음
    e = 2 * (nL - nR)                    # 음수 = 라인이 왼쪽
    if e > 60:
        side = 1
    elif e < -60:
        side = -1
    lost = nL > 82 and nR > 82 and now - t_ahead > 30
    if lost:
        steering_dc(side * ST_BANK, BANK_POWER)
    else:
        g = KG * int(hub.imu.angular_velocity(Axis.Z)) // 1000
        steering_dc(KP10 * e // 10 + max(-40, min(40, g)), 100)
