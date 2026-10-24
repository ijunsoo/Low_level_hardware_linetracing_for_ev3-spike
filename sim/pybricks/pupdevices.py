import core
from .parameters import Direction


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
        core.SIM.motor_run(self._port, speed * self._sign)

    def stop(self):
        core.SIM.motor_stop(self._port, "coast")

    def brake(self):
        core.SIM.motor_stop(self._port, "brake")

    def hold(self):
        core.SIM.motor_stop(self._port, "hold")

    def angle(self):
        return int(self._raw_angle() - self._off)

    def speed(self):
        return int(core.SIM.motor_speed_phys(self._port) * self._sign)

    def reset_angle(self, angle=0):
        self._off = self._raw_angle() - (angle or 0)

    def _unsupported(self, *a, **k):
        raise NotImplementedError("시뮬 목은 dc()/run()/stop()/brake()/hold()만 지원")

    run_angle = run_target = run_time = run_until_stalled = _unsupported
    track_target = done = stalled = _unsupported
