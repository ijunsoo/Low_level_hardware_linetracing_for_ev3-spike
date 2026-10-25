import core
from .parameters import Color, Direction


class _Control:
    """motor.control 스텁 (limits/pid 설정은 시뮬 모터모델에 영향 없음)."""

    def limits(self, *a, **k):
        return (0, 0, 0)

    def pid(self, *a, **k):
        return (0, 0, 0, 0, 0)

    def target_tolerances(self, *a, **k):
        return (0, 0)

    def stall_tolerances(self, *a, **k):
        return (0, 0)


class Motor:
    """실기 Motor의 dc 제어 + 엔코더 읽기 재현. 포트로 프로파일 motors 맵의
    측(L/R)·장착부호를 찾는다. angle()/speed()는 '모터축' 기준(= 바퀴/기어비)."""

    def __init__(self, port, positive_direction=Direction.CLOCKWISE, gears=None,
                 reset_angle=True, profile=None, **k):
        self._sign = 1 if positive_direction == Direction.CLOCKWISE else -1
        self._g = 1.0                       # 출력축 회전 / 모터축 회전 (실기처럼 gears=[20, 12]이면 20/12)
        if gears:
            for train in (gears if isinstance(gears[0], (list, tuple)) else [gears]):
                self._g *= train[0] / train[-1]
        self._port = port.name
        core.SIM.motor_check(self._port)
        self._off = 0.0
        self._off = self._raw_angle()       # 생성 시 0 기준
        self.control = _Control()

    def _raw_angle(self):
        return core.SIM.motor_angle_phys(self._port) * self._sign

    def dc(self, duty):
        # 실기: dc값 × 생성자 방향 = 물리 duty
        core.SIM.motor_dc(self._port, duty * self._sign)

    def run(self, speed):
        core.SIM.motor_run(self._port, speed / self._g * self._sign)

    def stop(self):
        core.SIM.motor_stop(self._port, "coast")

    def brake(self):
        core.SIM.motor_stop(self._port, "brake")

    def hold(self):
        core.SIM.motor_stop(self._port, "hold")

    def angle(self):
        return int((self._raw_angle() - self._off) * self._g)

    def speed(self):
        return int(core.SIM.motor_speed_phys(self._port) * self._sign * self._g)

    def reset_angle(self, angle=0):
        self._off = self._raw_angle() - (angle or 0) / self._g

    def _unsupported(self, *a, **k):
        raise NotImplementedError("the simulator's Motor supports dc(), run(), stop(), brake(), hold(), angle(), speed() and reset_angle() only")

    run_angle = run_target = run_time = run_until_stalled = _unsupported
    track_target = done = stalled = _unsupported


class _Lights:
    """ColorSensor.lights 스텁 (빛 세기는 시뮬 센서에 영향 없음)."""

    def on(self, *a, **k):
        pass

    def off(self):
        pass


class ColorSensor:
    """SPIKE Color Sensor: reflection()은 이 포트에 있는 시뮬 센서 (프로파일 sensor_ports).
    EV3 ColorSensor 목과 같은 센서 모델 — 밝기 범위는 프로파일 sensor_white/sensor_black."""

    def __init__(self, port):
        self._port = port.name
        core.SIM.color_check(self._port)
        self.lights = _Lights()

    def reflection(self):
        return core.SIM.sensor_read(self._port)

    def ambient(self):
        core.SIM.sensor_read(self._port)      # 시간 비용만
        return 0

    def color(self, surface=True):
        # 근사: 반사값이 흰/검 중간보다 어두우면 BLACK
        i = core.SIM._port_idx[self._port]
        mid = (core.SIM.p["sensor_white"][i] + core.SIM.p["sensor_black"][i]) / 2
        return Color.BLACK if self.reflection() < mid else Color.WHITE

    def hsv(self, surface=True):
        return self.color(surface)

    def detectable_colors(self, colors=None):
        return (Color.BLACK, Color.WHITE)
