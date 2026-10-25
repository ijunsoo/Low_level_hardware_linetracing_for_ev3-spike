# Browser runner: runs a Pybricks program on the simulator (sim/core.py + the pybricks mock) inside
# Pyodide and streams the run to the page. Loaded by web/worker.js; the sim files live in /sim.
import math
import sys
import traceback

sys.path.insert(0, "/sim")
import numpy as np                       # noqa: E402
import core                              # noqa: E402
import js                                # noqa: E402  (Pyodide: postMessage to the page)
from pyodide.ffi import to_js            # noqa: E402

STREAM_EVERY_S = 0.05                    # send new trace rows every this much simulated time
MAX_CONSOLE = 400                        # program output lines kept per run


def _post(msg):
    js.postMessage(to_js(msg, dict_converter=js.Object.fromEntries))


def info():
    return {"profiles": core.list_profiles(), "tracks": core.track_names()}


def track_image(name, profile, mm_per_px=2):
    # The line raster the sensors see (markers and photo points included), shrunk to mm_per_px
    p = core.load_profile(profile)
    tk = core.make_track(name, p["line_width_mm"])
    k = max(1, int(round(mm_per_px / tk.res)))
    H, W = tk.img.shape
    h, w = H // k, W // k
    small = tk.img[:h * k, :w * k].reshape(h, k, w, k).max(axis=(1, 3))
    return {"w": w, "h": h, "x0": tk.x0, "y0": tk.y0, "res": tk.res * k,
            "data": (small > 0.5).astype(np.uint8).tobytes(),
            "start": [float(v) for v in tk.path[0]], "end": [float(v) for v in tk.path[-1]],
            "length": tk.length}


class _Console:
    # Program print() → page console, a line at a time
    def __init__(self):
        self.buf = ""
        self.n = 0

    def write(self, s):
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self.n += 1
            if self.n <= MAX_CONSOLE:
                _post({"type": "console", "text": line})
            elif self.n == MAX_CONSOLE + 1:
                _post({"type": "console", "text": "… (more output hidden)"})
        return len(s)

    def flush(self):
        pass


def _world_params(profile, k):
    # Randomized world k, as in the regression sweeps: motor lag, battery, sensor noise/lag, command lag,
    # IMU lag, turning authority and wheel wobble
    import eval_lf3 as EV
    weff = [1.4, 1.6] if profile.startswith("spike_3s_geo2") else [1.15, 1.5]
    a = dict(noise=[0.3, 2.0], lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=weff, wob=[0.005, 0.035])
    return EV.world(core.load_profile(profile), k, a)


def run_program(code, profile, track, offset=0.0, seed=0, lost=450.0, limit=60.0, world=-1, stream=True):
    if world >= 0:
        params, off = _world_params(profile, world)
        seed = world
    else:
        params, off = core.load_profile(profile), offset
    sim = core.Sim(params, core.make_track(track, params["line_width_mm"]), start_lateral_mm=off,
                   time_limit_s=limit, seed=seed)
    sim.LOST_MM = lost
    core.SIM = sim
    n_sens = len(params["sensor_ports"])
    if stream:
        _post({"type": "robot", "sensors": [list(map(float, s)) for s in params["sensor_pos_mm"]],
               "axle": float(params["axle_track_mm"]), "wheel": float(params["wheel_diameter_mm"]),
               "white": [float(v) for v in params["sensor_white"]], "black": [float(v) for v in params["sensor_black"]],
               "cols": 7 + n_sens})
    state = {"sent": 0, "next": 0.0}

    def flush():
        rows = sim.trace[state["sent"]:]
        if rows:
            state["sent"] += len(rows)
            _post({"type": "frames", "data": np.asarray(rows, dtype=np.float32).ravel().tobytes()})

    if stream:
        orig = sim.advance

        def advance(dt):
            orig(dt)
            if sim.t >= state["next"]:
                state["next"] = sim.t + STREAM_EVERY_S
                flush()
        sim.advance = advance

    console = _Console()
    old_out = sys.stdout
    sys.stdout = console
    error = None
    try:
        exec(compile(code, "program.py", "exec"), {"__name__": "__main__"})
    except (core.SimDone, SystemExit):
        pass
    except Exception:
        tb = traceback.format_exc().splitlines()
        start = next((i for i, ln in enumerate(tb) if 'File "program.py"' in ln), len(tb) - 1)
        error = "\n".join([tb[0]] + tb[start:][-14:])     # from the program's own frame on (hide the runner)
    finally:
        sys.stdout = old_out
    if console.buf:
        console.write("\n")
    r = dict(sim.result or {"reason": "error" if error else "none", "time_s": round(sim.t, 3)})
    if stream:
        flush()
        lights = [[float(t), str(v)] for t, kind, v in sim.events if kind == "light"]
        _post({"type": "result", "result": r, "error": error, "lights": lights})
        return None
    step = max(1, int(round(0.02 / 0.005)))
    trail = np.asarray([(row[1], row[2]) for row in sim.trace[::step]], dtype=np.float32).ravel().tobytes()
    return {"result": r, "error": error, "trail": trail}


def evaluate(code, profile, track, worlds=8, lost=450.0, limit=60.0):
    # Robustness check, like evaluating an RL policy: the same program in `worlds` randomized worlds
    for k in range(worlds):
        out = run_program(code, profile, track, lost=lost, limit=limit, world=k, stream=False)
        _post({"type": "world", "k": k, "result": out["result"], "error": out["error"], "trail": out["trail"]})
        if out["error"]:
            break
    _post({"type": "eval_done"})
