# pybricks.robotics 목 — DriveBase: 바퀴 속도를 Motor.run()으로 내는 개루프 근사 (실기는 폐루프)
#  drive/stop/brake/distance/angle/state/reset/settings와, 그 위에 만든 straight/turn (블록킹).
#  turn_rate·angle은 실기처럼 + = 시계방향(오른쪽). 바퀴 지름·축간 거리는 프로그램이 준 값으로 계산하므로
#  실기처럼 그 값이 프로파일(실제 로봇)과 다르면 거리·각도가 틀린다. 기어가 있으면 Motor(gears=...)를 줄 것, use_gyro(True)면 각도 = 실제 방향.
import math

import core
from .tools import wait as _wait


class DriveBase:
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        self._l = left_motor
        self._r = right_motor
        self._d = float(wheel_diameter)
        self._w = float(axle_track)
        self._set = [200, 600, 100, 400]    # 직진 속도 mm/s, 직진 가속, 회전 속도 °/s, 회전 가속
        self._d0 = 0.0
        self._a0 = 0.0
        self._gyro = False                  # use_gyro(True): 각도를 허브 IMU(시뮬의 실제 방향)로

    def _deg(self, mm_s):                   # 바퀴 속도 mm/s → 모터 °/s
        return mm_s * 360.0 / (math.pi * self._d)

    def _dist(self):
        return (self._l.angle() + self._r.angle()) / 2 * math.pi * self._d / 360

    def _ang(self):
        if self._gyro:
            return -math.degrees(core.SIM.h)    # 시뮬 방향은 반시계 + → 시계 +로
        return (self._l.angle() - self._r.angle()) * self._d / (2 * self._w)

    def drive(self, speed, turn_rate):
        dv = math.radians(turn_rate) * self._w / 2
        self._l.run(self._deg(speed + dv))
        self._r.run(self._deg(speed - dv))

    def stop(self):
        self._l.stop()
        self._r.stop()

    def brake(self):
        self._l.brake()
        self._r.brake()

    def distance(self):
        return int(self._dist() - self._d0)

    def angle(self):
        return int(self._ang() - self._a0)

    def state(self):
        v = (self._l.speed() + self._r.speed()) / 2 * math.pi * self._d / 360
        w = (self._l.speed() - self._r.speed()) * self._d / (2 * self._w)
        return (self.distance(), int(v), self.angle(), int(w))

    def reset(self, distance=0, angle=0):
        self._d0 = self._dist() - distance
        self._a0 = self._ang() - angle

    def settings(self, straight_speed=None, straight_acceleration=None, turn_rate=None, turn_acceleration=None):
        new = (straight_speed, straight_acceleration, turn_rate, turn_acceleration)
        if all(v is None for v in new):
            return tuple(self._set)
        for i, v in enumerate(new):
            if v is not None:
                self._set[i] = v

    def straight(self, distance, then=None, wait=True):
        d0 = self._dist()
        s = self._set[0]
        self.drive(s if distance >= 0 else -s, 0)
        while abs(self._dist() - d0) < abs(distance):
            _wait(5)
        self.brake()

    def turn(self, angle, then=None, wait=True):
        a0 = self._ang()
        r = self._set[2]
        self.drive(0, r if angle >= 0 else -r)
        while abs(self._ang() - a0) < abs(angle):
            _wait(5)
        self.brake()

    def done(self):
        return True

    def stalled(self):
        return False

    def use_gyro(self, use_gyro):
        a = self.angle()
        self._gyro = bool(use_gyro)
        self.reset(self.distance(), a)

    def _unsupported(self, *a, **k):
        raise NotImplementedError("the simulator's DriveBase supports drive(), stop(), brake(), straight(), turn(), "
                                  "distance(), angle(), state(), reset() and settings() only")

    curve = arc = _unsupported
