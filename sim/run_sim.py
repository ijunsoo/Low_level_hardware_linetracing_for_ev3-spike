# =====================================================================
#  Run an unmodified Pybricks robot program (SPIKE Prime / EV3) in the
#  simulator and save a plot of the run.
#
#  Usage:
#    python sim/run_sim.py examples/spike_simple_follower.py --profile spike_3s_geo --track main
#    python sim/run_sim.py ev3/ev3_final.py --profile ev3_3s_geo --track hard --no-plot
#    python sim/run_sim.py my_robot.py --profile spike_3s_geo2 \
#        --sub "RAW_MODE = True=>RAW_MODE = False"   # replace text in the program before running
#    python sim/run_sim.py --list                    # list profiles and tracks
#
#  The program file itself is never changed. `import pybricks...` inside it
#  resolves to the mock package in sim/pybricks/.
# =====================================================================
import argparse
import collections
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Run a Pybricks line-following program in the simulator.")
    ap.add_argument("target", nargs="?", default=None, help="robot program to run (e.g. examples/spike_simple_follower.py)")
    ap.add_argument("--track", default="mrc",
                    help="track to drive on (default: mrc; see --list). photo@SCALE scales the photo track, e.g. photo@1.3")
    ap.add_argument("--profile", default=None,
                    help="robot hardware profile from sim/profiles (e.g. spike_3s_geo, ev3_3s_geo; see --list)")
    ap.add_argument("--params", default=None, help="path to a robot params JSON instead of a profile")
    ap.add_argument("--sub", action="append", default=[],
                    help="replace text in the program before running: 'OLD=>NEW' (repeatable; the file is not changed)")
    ap.add_argument("--offset", type=float, default=0.0, help="start with this sideways offset from the line (mm)")
    ap.add_argument("--seed", type=int, default=0, help="random seed for sensor noise")
    ap.add_argument("--limit", type=float, default=60.0, help="time limit (s)")
    ap.add_argument("--lost", type=float, default=core.Sim.LOST_MM,
                    help="end the run as 'lost' this far from the line (mm, default %(default)s)")
    ap.add_argument("--out", default="sim_result.png", help="where to save the plot (default: ./sim_result.png)")
    ap.add_argument("--no-plot", action="store_true", help="don't draw a plot (no matplotlib needed)")
    ap.add_argument("--list", action="store_true", help="list available profiles and tracks")
    args = ap.parse_args()
    if not args.list and not args.target:
        ap.error("give a robot program to run (e.g. examples/spike_simple_follower.py --profile spike_3s_geo)")

    if args.list:
        print("profiles:", ", ".join(core.list_profiles()))
        print("tracks:", ", ".join(core.track_names()), "(+ photo@SCALE)")
        return
    if args.track not in core.track_names() and not args.track.startswith("photo@"):
        ap.error("unknown track %r (choose from %s, or photo@SCALE)" % (args.track, ", ".join(core.track_names())))

    params = (core.load_profile(args.profile) if args.profile
              else core.load_params(args.params))
    track = core.make_track(args.track, params["line_width_mm"])
    sim = core.Sim(params, track, start_lateral_mm=args.offset,
                   time_limit_s=args.limit, seed=args.seed)
    sim.LOST_MM = args.lost
    core.SIM = sim

    src = open(args.target, encoding="utf-8").read()
    for s in args.sub:
        old, new = s.split("=>", 1)
        if old not in src:
            sys.exit("--sub: text not found in the program: %r" % old)
        src = src.replace(old, new)

    sys.path.insert(0, os.path.dirname(os.path.abspath(args.target)))   # the program's own modules (e.g. robot.py)
    print("running:", os.path.abspath(args.target), "/ track:", args.track,
          "/ profile:", args.profile or (args.params or "robot_params.json"))
    try:
        exec(compile(src, os.path.abspath(args.target), "exec"), {"__name__": "__main__"})
        print("note: the program ended by itself before the run was over (no main loop?)")
    except core.SimDone:
        pass

    r = sim.result or {"reason": "none"}
    print("result:", r)
    leds = collections.Counter(v for _, k, v in sim.events if k == "light")
    if leds:
        print("light events:", dict(leds))
    if not args.no_plot:
        plot(sim, args.out)


def plot(sim, out):
    import numpy as np
    try:
        import matplotlib
    except ImportError:
        print("no plot: matplotlib is not installed (pip install matplotlib, or use --no-plot)")
        return
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tr = np.asarray(sim.trace) if sim.trace else np.zeros((1, 6))
    tk = sim.track

    fig, axes = plt.subplots(3, 1, figsize=(9, 12),
                             gridspec_kw={"height_ratios": [3, 1, 1]})
    ax = axes[0]
    H, W = tk.img.shape
    ax.imshow(tk.img, cmap="gray_r", origin="lower",
              extent=[tk.x0, tk.x0 + W * tk.res, tk.y0, tk.y0 + H * tk.res])
    sc = ax.scatter(tr[:, 1], tr[:, 2], c=tr[:, 0], s=2, cmap="viridis")
    fig.colorbar(sc, ax=ax, label="time (s)")
    ax.plot(tr[0, 1], tr[0, 2], "g^", markersize=10, label="start")
    ax.plot(tr[-1, 1], tr[-1, 2], "rx", markersize=10, label="end")
    ax.set_title("trajectory  [%s: %.2fs, lat %.1fmm]" % (
        sim.result["reason"], sim.result["time_s"], sim.result["mean_lat_mm"])
        if sim.result else "trajectory")
    ax.legend()
    ax.set_aspect("equal")

    axes[1].plot(tr[:, 0], tr[:, 3])
    axes[1].axhline(sim.LOST_MM, color="r", ls="--", lw=0.8)
    axes[1].set_ylabel("lateral offset (mm)")
    axes[1].grid(alpha=0.3)

    axes[2].plot(tr[:, 0], tr[:, 4], label="duty L", lw=0.8)
    axes[2].plot(tr[:, 0], tr[:, 5], label="duty R", lw=0.8)
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("motor duty (%)")
    axes[2].legend()
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("plot saved:", out)


if __name__ == "__main__":
    main()
