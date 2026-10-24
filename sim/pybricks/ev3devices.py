# EV3 장치 목 — Pybricks v4(EV3 네이티브) API 기준. v2(ev3dev)와 공통인 부분.
#  - Motor: dc/run/stop/brake/hold/angle/speed/reset_angle (SPIKE 목과 같은 모델)
#  - ColorSensor: reflection() = 프로파일 sensor_ports의 시뮬 센서
#  - GyroSensor: speed()/angle()/reset_angle() = 프로파일 gyro 모델
#    positive_direction: 센서 윗면(빨간 점)에서 봤을 때 + 방향 (기본 시계방향)
#  - Touch/Ultrasonic/Infrared: 주행과 무관 — 중립값만 돌려줌
import core
from .parameters import Color, Direction
from .pupdevices import Motor as _Motor


class Motor(_Motor):
    """EV3 Large/Medium Motor."""


class ColorSensor:
    def __init__(self, port):
        self._port = port.name
        core.SIM.color_check(self._port)

    def reflection(self):
        return core.SIM.sensor_read(self._port)

    def color(self):
        # 근사: 반사값이 흰/검 중간보다 어두우면 BLACK
        i = core.SIM._port_idx[self._port]
        mid = (core.SIM.p["sensor_white"][i] + core.SIM.p["sensor_black"][i]) / 2
        return Color.BLACK if self.reflection() < mid else Color.WHITE

    def ambient(self):
        core.SIM.sensor_read(self._port)      # 시간 비용만
        return 0

    def rgb(self):
        r = self.reflection()
        return (r, r, r)


class GyroSensor:
    def __init__(self, port, positive_direction=Direction.CLOCKWISE):
        self._port = port.name
        core.SIM.gyro_check(self._port)
        # 시뮬 값은 반시계 + → 시계 + 가 기본인 실기 규약으로 뒤집는다
        self._sign = -1 if positive_direction == Direction.CLOCKWISE else 1
        self._off = 0.0

    def speed(self):
        rate, _ = core.SIM.gyro_read()
        return rate * self._sign

    def angle(self):
        _, ang = core.SIM.gyro_read()
        return int(ang * self._sign - self._off)

    def reset_angle(self, angle=0):
        _, ang = core.SIM.gyro_read()
        self._off = ang * self._sign - angle


class TouchSensor:
    def __init__(self, port):
        self._port = port.name

    def pressed(self):
        return False


class UltrasonicSensor:
    def __init__(self, port):
        self._port = port.name

    def distance(self, silent=False):
        return 2550          # 장애물 없음

    def presence(self):
        return False


class InfraredSensor:
    def __init__(self, port):
        self._port = port.name

    def distance(self):
        return 100

    def buttons(self, channel=1):
        return set()

    def keypad(self):
        return set()
