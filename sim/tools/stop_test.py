# 정지 버튼 시험: 달리는 중 STOP_MS에 SystemExit(= Pybricks 정지 버튼이 보내는 것)를 넣고, 그 뒤 finally에서 생기는 일
#   - 정지 후 출력 바이트·줄 수, finally가 붙잡는 시간(wait 합, 블루투스 전송 시간은 빼고), 그동안 바퀴가 더 굴러간 거리
#   python3 stop_test.py '[["이름", "spike/파일.py", {"BB_PRINT": false}], ...]' --prof '{"gear_ratio": 3.3333}'
import argparse
import contextlib
import io
import json
import math
import multiprocessing as mp
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim"))
import core             # noqa: E402
import eval_lf3 as EV   # noqa: E402
import bb_parse         # noqa: E402

ANCHOR = "        steering_dc(steer, power)\n\n\n"


def make_src(path, over, stop_ms, base_over):
    s = open(os.path.join(ROOT, path), encoding="utf-8").read()
    # 시뮬 센서 목은 반사 모드(0)만 → 다른 시뮬 도구(brk_dur.py)와 같은 RAW_MODE False + 시뮬 밝기 VALUE
    for k, v in dict(base_over, **over).items():
        s = EV.setc(s, k, v)
    assert s.count(ANCHOR) == 1
    inj = ("        if now >= %d:\n"
           "            _S = __import__('core').SIM\n"
           "            MARK.append((_S.t, _S.ang_l, _S.ang_r))\n"
           "            print('@@STOP')\n"
           "            raise SystemExit\n") % stop_ms
    return s.replace(ANCHOR, inj + ANCHOR)


def one(job):
    name, src, k, a = job
    core.Sim.LOST_MM = 1e9                  # 정지 뒤 로봇이 라인을 벗어나도 시뮬을 끝내지 않음 (더 간 거리를 재려고)
    base = EV.base_profile(a["profile"], a["fwd"])
    if a.get("prof"):
        base = dict(base, **a["prof"])
    p, off = EV.world(base, k, a)
    tr = core.make_track(a["track"], base["line_width_mm"])
    sim = core.Sim(p, tr, start_lateral_mm=off, time_limit_s=120.0, seed=k)
    core.SIM = sim
    g = {"__name__": "__main__", "MARK": []}
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        try:
            exec(compile(src, name, "exec"), g)
        except (SystemExit, core.SimDone):
            pass
    if not g["MARK"]:
        return name, k, None
    t0, l0, r0 = g["MARK"][0]
    after = out.getvalue().split("@@STOP\n", 1)[1]
    nbb = 0
    if "BB," in after:
        with contextlib.redirect_stdout(io.StringIO()):
            nbb = len(bb_parse.parse(after)[1])
    return name, k, dict(bytes=len(after.encode("utf-8")), lines=after.count("\n"),
                         bbd=sum(1 for ln in after.splitlines() if ln.startswith("BBD,")), nbb=nbb,
                         hold_s=sim.t - t0, moved=(abs(sim.ang_l - l0) + abs(sim.ang_r - r0)) / 2 * math.pi * sim.p["wheel_diameter_mm"] / 360)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("variants")
    ap.add_argument("--profile", default="spike_3s_geo")
    ap.add_argument("--fwd", type=float, default=170.0)
    ap.add_argument("--prof", default=None)
    ap.add_argument("--track", default="main")
    ap.add_argument("--stop", type=int, default=1500, help="정지 버튼을 누르는 시각 ms (주행 시작 기준)")
    ap.add_argument("--worlds", type=int, default=8)
    ap.add_argument("--base", default='{"RAW_MODE": false, "VALUE": [34, 6, 22, 3, 21, 3]}', help="모든 변형에 먼저 넣을 상수 (EV3는 {})")
    args = ap.parse_args()
    variants = json.loads(args.variants)
    a = dict(profile=args.profile, fwd=args.fwd, prof=json.loads(args.prof) if args.prof else None, track=args.track,
             noise=[0.3, 1.0], lag=[3, 8], cmdlag=[0, 35], imulag=[0, 20], weff=[1.15, 1.5], wob=[0.005, 0.035])
    jobs = [(nm, make_src(path, over, args.stop, json.loads(args.base)), k, a) for nm, path, over in variants for k in range(args.worlds)]
    with mp.Pool(max(1, os.cpu_count() - 1)) as pool:
        res = pool.map(one, jobs, chunksize=1)
    for nm, _, _ in variants:
        rows = [r[2] for r in res if r[0] == nm and r[2]]

        def avg(key):
            return sum(r[key] for r in rows) / max(1, len(rows))
        print("%-26s 정지 후 출력 %6.0fB %4.0f줄 (BBD %2.0f줄, 해석된 기록 %4.0f개) | finally가 붙잡은 시간 %4.2fs"
              " (블루투스 전송 시간 빼고) | 그동안 바퀴가 더 굴러간 거리 %5.0fmm  [%d개 월드]" % (
                  nm, avg("bytes"), avg("lines"), avg("bbd"), avg("nbb"), avg("hold_s"), avg("moved"), len(rows)))


if __name__ == "__main__":
    main()
