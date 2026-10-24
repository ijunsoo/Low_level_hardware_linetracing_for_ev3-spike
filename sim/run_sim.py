# =====================================================================
#  실기 코드를 '수정 없이' 시뮬레이터에서 실행하고 결과를 그림으로 저장.
#
#  사용법:
#    python3 sim/run_sim.py spike/spike_lf3_rule.py --profile spike_3s_geo2 --track photo
#    python3 sim/run_sim.py ev3/ev3_lf3_rule.py --profile ev3_3s_geo --track hard
#    python3 sim/run_sim.py 파일.py --profile spike_3s_geo2 \
#        --sub "RAW_MODE = True=>RAW_MODE = False"   # 소스 치환 (시뮬 센서 목은 반사 모드만)
#    python3 sim/run_sim.py --list                 # 프로파일·트랙 목록
# =====================================================================
import argparse
import collections
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", nargs="?", default=None, help="실행할 실기 파일 (예: spike/spike_lf3_rule.py)")
    ap.add_argument("--track", default="mrc", choices=core.track_names())
    ap.add_argument("--profile", default=None,
                    help="하드웨어 프로파일 (spike_3s_geo2, spike_3s_geo2x, ev3_3s_geo ... — --list로 전체)")
    ap.add_argument("--params", default=None, help="robot_params.json 경로 (프로파일 대신)")
    ap.add_argument("--sub", action="append", default=[],
                    help="소스 치환 'OLD=>NEW' (여러 번 가능, 실기 파일은 안 바뀜)")
    ap.add_argument("--offset", type=float, default=0.0, help="시작 좌우 오프셋(mm)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--limit", type=float, default=60.0, help="제한시간(s)")
    ap.add_argument("--out", default=os.path.join(core.PKG_DIR, "sim_result.png"))
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--list", action="store_true", help="프로파일·트랙 목록")
    args = ap.parse_args()
    if not args.list and not args.target:
        ap.error("실행할 실기 파일을 주세요 (예: spike/spike_lf3_rule.py --profile spike_3s_geo2)")

    if args.list:
        print("프로파일:", ", ".join(core.list_profiles()))
        print("트랙:", ", ".join(core.track_names()))
        return

    params = (core.load_profile(args.profile) if args.profile
              else core.load_params(args.params))
    track = core.make_track(args.track, params["line_width_mm"])
    sim = core.Sim(params, track, start_lateral_mm=args.offset,
                   time_limit_s=args.limit, seed=args.seed)
    core.SIM = sim

    src = open(args.target, encoding="utf-8").read()
    for s in args.sub:
        old, new = s.split("=>", 1)
        if old not in src:
            sys.exit("--sub 대상 문자열이 파일에 없음: %r" % old)
        src = src.replace(old, new)

    print("실행:", os.path.abspath(args.target), "/ 트랙:", args.track,
          "/ 프로파일:", args.profile or (args.params or "robot_params.json"))
    try:
        exec(compile(src, os.path.abspath(args.target), "exec"), {"__name__": "__main__"})
        print("경고: 파일이 스스로 종료됨 (pid_control 무한루프가 아님?)")
    except core.SimDone:
        pass

    r = sim.result or {"reason": "none"}
    print("결과:", r)
    leds = collections.Counter(v for _, k, v in sim.events if k == "light")
    if leds:
        print("LED 이벤트:", dict(leds))
    if not args.no_plot:
        plot(sim, args.out)


def plot(sim, out):
    import numpy as np
    import matplotlib
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
    print("그림 저장:", out)


if __name__ == "__main__":
    main()
