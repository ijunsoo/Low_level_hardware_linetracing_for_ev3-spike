# =====================================================================
#  Checks the programs in the browser simulator's menu the way the page runs them (web/runner.py):
#  each program on its own track and robot in the default world (▶ Run) must finish within the expected
#  time, and the competition programs must also finish in most of the 8 randomized worlds.
#  It also fails if a simulator adjustment in web/build.py no longer matches its program.
#
#    python web/check_presets.py             # CI runs this; the Pages deploy waits for it
#    python web/check_presets.py --measure   # print the current numbers (to update EXPECT on purpose)
# =====================================================================
import argparse
import contextlib
import io
import json
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "sim"))
sys.path.insert(0, HERE)
import build    # noqa: E402  (the program menu and its simulator adjustments)
import runner   # noqa: E402  (the page's runner: robots, arenas, randomized worlds)

# program id: (time in the default world, s, (min, max)), finishes needed out of 8 random worlds (None = not run)
EXPECT = {
    "ev3_final": ((4.3, 5.3), 7),
    "spike_final": ((3.6, 4.5), 7),
    "spike_rl": ((2.8, 3.5), None),
    "spike_simple": ((3.9, 4.8), None),
    "spike_drivebase": ((4.4, 5.4), None),
    "ev3_simple": ((3.9, 4.8), None),
}
WORLDS = 8


def program(pid):
    for p in build.PROGRAMS:
        if p[0] == pid:
            src = build.read(p[1])
            for a, b in p[6]:
                if a not in src:
                    raise SystemExit("%s: the simulator adjustment %r no longer matches %s" % (pid, a, p[1]))
                src = src.replace(a, b)
            return src, p[2], p[3]
    raise SystemExit("unknown program " + pid)


def job(args):
    pid, k = args
    src, profile, track = program(pid)
    with contextlib.redirect_stdout(io.StringIO()):
        out = runner.run_program(src, json.dumps(profile), json.dumps(track), lost=450.0, limit=40.0,
                                 world=k, stream=False)
    return pid, k, out["result"], out["error"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--measure", action="store_true", help="print the numbers without judging them")
    args = ap.parse_args()
    menu = [p[0] for p in build.PROGRAMS]
    missing = [pid for pid in menu if pid not in EXPECT]
    if missing:
        raise SystemExit("add the new menu programs to EXPECT: %s" % ", ".join(missing))
    jobs = []
    for pid in menu:
        jobs.append((pid, -1))
        if EXPECT[pid][1] is not None or args.measure:
            jobs += [(pid, k) for k in range(WORLDS)]
    with Pool(max(2, (os.cpu_count() or 4) - 1)) as pool:
        res = pool.map(job, jobs, chunksize=1)
    bad = 0
    for pid in menu:
        (lo, hi), need = EXPECT[pid]
        _, _, r, err = next(x for x in res if x[0] == pid and x[1] < 0)
        ok = r["reason"] == "finish" and lo <= r["time_s"] <= hi and not err
        line = "%-16s ▶ %-7s %5.2f s (expect finish in %.1f–%.1f s)" % (pid, r["reason"], r["time_s"], lo, hi)
        ws = [x for x in res if x[0] == pid and x[1] >= 0]
        if ws:
            fin = sum(x[2]["reason"] == "finish" for x in ws)
            line += " · random worlds %d/%d%s" % (fin, len(ws), " (need %d)" % need if need else "")
            ok = ok and (need is None or fin >= need)
        if err:
            line += "\n    " + err.replace("\n", "\n    ")
        print(("     " if args.measure else ("OK   " if ok else "FAIL ")) + line)
        bad += not ok
    if not args.measure:
        print("\n%d/%d programs as expected" % (len(menu) - bad, len(menu)))
        sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
