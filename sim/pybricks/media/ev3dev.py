# pybricks.media.ev3dev 목 — Pybricks v2(ev3dev 기반 EV3 MicroPython) 전용 모듈.
# 프로파일 pybricks_api가 "v2"일 때만 import 성공. v4(EV3 네이티브)에선 실기처럼
# ImportError가 나서, v2용 코드를 v4에 그대로 올리면 깨지는 걸 시뮬에서도 잡는다.
import core


class _Names:
    def __getattr__(self, name):
        return name


_STUBS = {"SoundFile": _Names(), "ImageFile": _Names(), "Font": _Names(),
          "Image": _Names()}


def __getattr__(name):
    api = core.SIM.p.get("pybricks_api", "v4") if core.SIM else "v4"
    if api != "v2":
        raise ImportError("pybricks.media.ev3dev는 Pybricks %s에 없음 (v2/ev3dev 전용)" % api)
    if name in _STUBS:
        return _STUBS[name]
    raise AttributeError(name)
