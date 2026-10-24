# pybricks.robotics 목 — DriveBase는 import만 되고 주행 명령은 미지원
# (라인트레이서는 Motor.dc()로 직접 구동하므로 시뮬 모델에 없음)
class DriveBase:
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, name):
        def _unsupported(*a, **k):
            raise NotImplementedError("시뮬 목은 DriveBase.%s 미지원 — Motor.dc() 사용" % name)
        return _unsupported
