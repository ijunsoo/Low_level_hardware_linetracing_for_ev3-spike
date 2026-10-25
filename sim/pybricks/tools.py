import core


def wait(ms):
    core.SIM.wait_ms(ms)


class StopWatch:
    def __init__(self):
        self._t0 = core.SIM.t
        self._paused = None             # 멈춘 시각 (None = 흐르는 중)

    def time(self):
        core.SIM.poll()                 # StopWatch만 읽는 무한 반복은 stall로 끝냄
        t = self._paused if self._paused is not None else core.SIM.t
        return int((t - self._t0) * 1000)

    def reset(self):
        self._t0 = core.SIM.t
        if self._paused is not None:
            self._paused = core.SIM.t

    def pause(self):
        if self._paused is None:
            self._paused = core.SIM.t

    def resume(self):
        if self._paused is not None:
            self._t0 += core.SIM.t - self._paused
            self._paused = None


class DataLog:
    def __init__(self, *cols, **k):
        self.rows = []

    def log(self, *row):
        self.rows.append(row)
