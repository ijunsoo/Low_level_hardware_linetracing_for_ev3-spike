# Browser runner: runs a Pybricks program on the simulator (sim/core.py + the pybricks mock) inside
# Pyodide and streams the run to the page. Loaded by web/worker.js; the sim files live in /sim.
#
# Robots and arenas arrive as JSON text:
#   robot  "spike_3s_geo2"                       a built-in profile
#          {"base": "spike_3s_geo2", "overrides": {...profile keys...}}   a robot made in the page
#   arena  "main"                                a built-in track
#          {"random": {"level": "hard", "seed": 7}}                       a random competition-rule arena
#          {"spec": {"segs": [["S", 500], ["A", 150, 90], ["T", -90]], "extras": [[[x, y], [x, y]]]}}
import json
import math
import sys
import traceback

sys.path.insert(0, "/sim")
import numpy as np                       # noqa: E402
import core                              # noqa: E402

STREAM_EVERY_S = 0.05                    # send new trace rows every this much simulated time
MAX_CONSOLE = 400                        # program output lines kept per run
MAX_PATH_MM = 60000                      # an arena's line may be at most 60 m long …
MAX_SPAN_MM = 12000                      # … and fit in 12 m × 12 m …
MAX_PIXELS = 25_000_000                  # … and its 1 mm raster (with the 150 mm margin) in about 25 m² (100 MB)
LINE_WIDTH_MM = (5.0, 60.0)              # allowed line widths
CMD_LAG_SPREAD_MS = 8.0                  # random worlds draw the command lag within the robot's own value ± this
EDIT_KEYS = ["sensor_ports", "sensor_pos_mm", "sensor_white", "sensor_black", "motors", "wheel_diameter_mm",
             "axle_track_mm", "gear_ratio", "sensor_footprint_mm", "line_width_mm"]


def _post(msg):
    try:
        import js                        # Pyodide: postMessage to the page
        from pyodide.ffi import to_js
    except ImportError:                  # on a PC (web/check_presets.py) there is no page to send to
        return
    js.postMessage(to_js(msg, dict_converter=js.Object.fromEntries))


def info():
    return {"profiles": core.list_profiles(), "tracks": core.track_names()}


# ---------------------------------------------------------------- robots
def _base_name(robot):
    return robot if isinstance(robot, str) else robot["base"]


def robot_params(robot_json):
    robot = json.loads(robot_json)
    if isinstance(robot, str):
        return core.load_profile(robot)
    p = core.load_profile(robot["base"])
    over = robot.get("overrides") or {}
    p.update(over)
    if "sensor_pos_mm" in over and p.get("track_ref_mm", 0) > 0:
        p["track_ref_mm"] = max(float(s[0]) for s in p["sensor_pos_mm"])   # judge progress at the sensors
    return p


def robot_fields(name):
    # The fields the page's robot editor shows, from a built-in profile
    p = core.load_profile(name)
    out = {k: p.get(k) for k in EDIT_KEYS}
    n = len(p["sensor_ports"])
    out["sensor_white"] = list(p["sensor_white"])[:n]
    out["sensor_black"] = list(p["sensor_black"])[:n]
    if not out["motors"]:
        out["motors"] = {p["left_motor_port"]: ["L", p["left_mount_sign"]],
                         p["right_motor_port"]: ["R", p["right_mount_sign"]]}
    return json.dumps(out)


# ---------------------------------------------------------------- arenas
def random_spec(level, seed):
    from rl import tracks as RT
    spec = RT.generate(int(seed), level)
    return json.dumps({"segs": [list(s) for s in spec["segs"]],
                       "extras": [[list(map(float, p)) for p in e] for e in (spec.get("extras") or [])]})


def builtin_spec(name):
    # Segment list of a built-in track for the arena editor (None for photo tracks: they are point clouds)
    spec = core.TRACKS.get(name)
    if spec is None:
        return json.dumps(None)
    if isinstance(spec, dict):
        return json.dumps({"segs": [list(s) for s in spec["segs"]],
                           "extras": [[list(map(float, p)) for p in e] for e in (spec.get("extras") or [])]})
    return json.dumps({"segs": [list(s) for s in spec], "extras": []})


def _spec_track(spec, line_width):
    segs = []
    for s in spec.get("segs") or []:
        kind = str(s[0]).upper()
        if kind == "S" and len(s) == 2 and float(s[1]) > 0:
            segs.append(("S", float(s[1])))
        elif kind == "A" and len(s) == 3 and float(s[1]) >= 10:
            segs.append(("A", float(s[1]), float(s[2])))
        elif kind == "T" and len(s) == 2:
            segs.append(("T", float(s[1])))
        else:
            raise ValueError("bad segment %r: use S length, A radius angle (radius ≥ 10) or T angle" % (list(s),))
    total = sum(s[1] if s[0] == "S" else (abs(math.radians(s[2])) * s[1] if s[0] == "A" else 0.0) for s in segs)
    if total < 200:
        raise ValueError("the arena's line must be at least 200 mm long")
    if total > MAX_PATH_MM:
        raise ValueError("the arena's line is longer than %d m" % (MAX_PATH_MM // 1000))
    path = core.build_path(segs)
    extras = [[[float(a), float(b)] for a, b in e] for e in (spec.get("extras") or []) if len(e) >= 2]
    pts = np.vstack([path] + [np.asarray(e) for e in extras])
    span = pts.max(axis=0) - pts.min(axis=0)
    if float(span.max()) > MAX_SPAN_MM:
        raise ValueError("the arena is larger than %d m × %d m" % (MAX_SPAN_MM // 1000, MAX_SPAN_MM // 1000))
    if (span[0] + 300) * (span[1] + 300) > MAX_PIXELS:   # the raster core.Track would allocate (1 mm/px)
        raise ValueError("the arena is too big for the browser: %.1f m × %.1f m (about %d m² at most)"
                         % (span[0] / 1000, span[1] / 1000, MAX_PIXELS // 1_000_000))
    lw = _line_width(spec.get("line_width_mm") or line_width)
    return core.Track(path, lw, extras=extras or None)


def _line_width(lw):
    lw = float(lw)
    if not LINE_WIDTH_MM[0] <= lw <= LINE_WIDTH_MM[1]:
        raise ValueError("the line width must be %g–%g mm" % LINE_WIDTH_MM)
    return lw


def arena(arena_json, line_width):
    line_width = _line_width(line_width)
    a = json.loads(arena_json)
    if isinstance(a, str):
        return core.make_track(a, line_width)
    if "random" in a:
        return _spec_track(json.loads(random_spec(a["random"].get("level", "hard"), a["random"].get("seed", 1))), line_width)
    return _spec_track(a["spec"], line_width)


def track_image(arena_json, robot_json, mm_per_px=2):
    # The line raster the sensors see (markers and photo points included), shrunk to mm_per_px
    p = robot_params(robot_json)
    tk = arena(arena_json, p["line_width_mm"])
    k = max(1, int(round(mm_per_px / tk.res)))
    H, W = tk.img.shape
    h, w = H // k, W // k
    small = tk.img[:h * k, :w * k].reshape(h, k, w, k).max(axis=(1, 3))
    return {"w": w, "h": h, "x0": tk.x0, "y0": tk.y0, "res": tk.res * k,
            "data": (small > 0.5).astype(np.uint8).tobytes(),
            "start": [float(v) for v in tk.path[0]], "end": [float(v) for v in tk.path[-1]],
            "heading": math.atan2(tk.path[1][1] - tk.path[0][1], tk.path[1][0] - tk.path[0][0]),
            "length": tk.length}


# ---------------------------------------------------------------- runs
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


def _world(params, base, k):
    # Randomized world k: motor lag, battery, sensor noise/lag, command lag, IMU lag, turning authority and
    # wheel wobble. The command lag is drawn around the robot's own value (32 ms → 24–40 ms, the spread fitted
    # on real SPIKE black boxes); the research sweeps draw 0–35 ms, which made most worlds easier than ▶ Run.
    import eval_lf3 as EV
    weff = [1.4, 1.6] if base.startswith("spike_3s_geo2") else [1.15, 1.5]
    lag = float(params.get("cmd_lag_ms") or 0.0)
    a = dict(noise=[0.3, 2.0], lag=[3, 8], cmdlag=[max(0.0, lag - CMD_LAG_SPREAD_MS), lag + CMD_LAG_SPREAD_MS],
             imulag=[0, 20], weff=weff, wob=[0.005, 0.035])
    return EV.world(params, k, a)


def _world_info(params):
    return {"cmd_lag_ms": float(params.get("cmd_lag_ms") or 0.0), "battery": float(params.get("battery_scale", 1.0))}


def run_program(code, robot, arena_, offset=0.0, seed=0, lost=450.0, limit=60.0, world=-1, stream=True):
    params = robot_params(robot)
    if world >= 0:
        params, offset = _world(params, _base_name(json.loads(robot)), world)
        seed = world
    sim = core.Sim(params, arena(arena_, params["line_width_mm"]), start_lateral_mm=offset,
                   time_limit_s=limit, seed=seed)
    sim.LOST_MM = lost
    core.SIM = sim
    if stream:
        _post({"type": "robot", "sensors": [list(map(float, s)) for s in params["sensor_pos_mm"]],
               "axle": float(params["axle_track_mm"]), "wheel": float(params["wheel_diameter_mm"]),
               "white": [float(v) for v in params["sensor_white"]], "black": [float(v) for v in params["sensor_black"]],
               "cols": 7 + len(params["sensor_ports"])})
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
    return {"result": r, "error": error, "trail": trail, "world": _world_info(params)}


def evaluate(code, robot, arena_, worlds=8, lost=450.0, limit=60.0):
    # Robustness check, like evaluating an RL policy: the same program in `worlds` randomized worlds
    base = _world_info(robot_params(robot))          # the default world of ▶ Run, shown for comparison
    for k in range(worlds):
        out = run_program(code, robot, arena_, lost=lost, limit=limit, world=k, stream=False)
        _post({"type": "world", "k": k, "result": out["result"], "error": out["error"], "trail": out["trail"],
               "world": out["world"], "base": base})
        if out["error"]:
            break
    _post({"type": "eval_done"})
