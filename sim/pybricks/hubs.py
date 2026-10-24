import math

import core


class _Battery:
    def voltage(self):
        if not core.SIM:
            return 8000
        return int(core.SIM.p.get("battery_mv", 8000) * core.SIM.p["battery_scale"])

    def current(self):
        return 0


class _Buttons:
    def pressed(self):
        return set()


class _Speaker:
    def beep(self, *a, **k):
        pass

    def play_notes(self, *a, **k):
        pass


class _Light:
    def on(self, color=None):
        if core.SIM:
            core.SIM.log_event("light", repr(color))

    def off(self):
        if core.SIM:
            core.SIM.log_event("light", "OFF")

    def blink(self, *a, **k):
        pass


class _Stub:
    """화면·디스플레이·system 등 주행과 무관한 API — 호출은 받고 아무것도 안 함."""

    def __getattr__(self, name):
        return lambda *a, **k: None


class _IMU:
    """SPIKE 허브 IMU: 요레이트만 재현 (deg/s, 좌회전 +, 지연·노이즈 0).
    Z축 외(X/Y) 각속도는 0, 가속도는 Z축에 중력만 (v11 요축 자동감지 대응).
    heading()은 좌회전 + 누적각 — 실기 부호와 다를 수 있어 코드가 자가판별할 것."""

    def __init__(self):
        self._h0 = core.SIM.h if core.SIM else 0.0

    def angular_velocity(self, axis=None):
        if axis is not None and getattr(axis, "name", "Z") != "Z":
            return 0.0
        return core.SIM.imu_rate_dps()

    def acceleration(self, axis=None):
        if axis is not None and getattr(axis, "name", "Z") == "Z":
            return 9810.0
        return 0.0 if axis is not None else (0.0, 0.0, 9810.0)

    def heading(self):
        return math.degrees(core.SIM.h - self._h0)

    def reset_heading(self, angle=0):
        self._h0 = core.SIM.h - math.radians(angle)

    def ready(self):
        return True

    def stationary(self):
        return abs(core.SIM.imu_rate_dps()) < 1.0


class PrimeHub:
    def __init__(self, *a, **k):
        self.battery = _Battery()
        self.buttons = _Buttons()
        self.speaker = _Speaker()
        self.imu = _IMU()
        self.light = _Light()
        self.display = _Stub()
        self.system = _Stub()


class EV3Brick:
    """EV3 브릭 (Pybricks v4 네이티브 / v2 ev3dev 공통 부분). 내장 IMU 없음 —
    요레이트는 ev3devices.GyroSensor(외장)나 모터 엔코더로."""

    def __init__(self, *a, **k):
        self.battery = _Battery()
        self.buttons = _Buttons()
        self.speaker = _Speaker()
        self.light = _Light()
        self.screen = _Stub()
        self.system = _Stub()
