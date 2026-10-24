import core


class PUPDevice:
    """SPIKE 허브에 꽂은 EV3 컬러센서(반사광 mode=0)만 재현. read() 1회마다
    시뮬 시간이 호출 비용(기본 1/(loop_hz×센서수))만큼 흘러 실기 루프 속도가 재현된다."""

    def __init__(self, port):
        self._port = port.name
        if self._port not in core.SIM.p["sensor_ports"]:
            raise ValueError("unknown sensor port: %s" % port)

    def read(self, mode=0):
        return (core.SIM.sensor_read(self._port),)

    def info(self):
        return {"id": 29}   # EV3 color sensor
