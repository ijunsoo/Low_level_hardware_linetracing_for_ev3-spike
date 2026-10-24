import core


def wait(ms):
    core.SIM.wait_ms(ms)


class StopWatch:
    def __init__(self):
        self._t0 = core.SIM.t

    def time(self):
        return int((core.SIM.t - self._t0) * 1000)

    def reset(self):
        self._t0 = core.SIM.t

    def pause(self):
        pass

    def resume(self):
        pass


class DataLog:
    def __init__(self, *cols, **k):
        self.rows = []

    def log(self, *row):
        self.rows.append(row)
