# =====================================================================
#  학습 중 체크포인트를 '실기용 파일'로 내보내 CPU 시뮬(core.py)에서 바로 평가 — GPU 학습과 병렬
#
#    python3 sim/rl/cpu_eval_ckpt.py sim/rl/runs/<이름> 결과.jsonl [--worlds 12] [--profile spike_3s_geo]
#
#  새 policy_itN.pt가 생길 때마다: export_policy로 Pybricks 파일 생성 → 사진·직각 트랙 × 월드 N개를
#  CPU 코어 전부로 돌림 → 결과 한 줄을 jsonl에 추가 (watch_train.py --cpu 결과.jsonl 로 실시간 표시).
#  맨 처음 한 번은 같은 월드에서 규칙 코드(E2)도 돌려 기준으로 기록.
# =====================================================================
import argparse
import contextlib
import io
import json
import multiprocessing as mp
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM)
sys.path.insert(0, SIM)
import numpy as np        # noqa: E402
import core               # noqa: E402
import sweep_lf3          # noqa: E402

# 정책 내보내기(export_policy)를 돌릴 torch 있는 파이썬: 환경변수 TORCH_PY > 기본 venv > 지금 파이썬
_VENV_PY = os.path.expanduser("~/.venvs/linefollowing/bin/python")
TORCH_PY = os.environ.get("TORCH_PY") or (_VENV_PY if os.path.exists(_VENV_PY) else sys.executable)
TRACKS = {"photo": 40.0, "rightangle": 20.0}
CFG = {"_ASYNC": True, "_CMDLAG": [0, 32], "_IMULAG": [0, 20], "_WEFF_RANGE": [1.15, 1.5],
       "_NOISE": [0.3, 1.0], "_LAG_MS": [3, 8]}


def run_one(job):
    src, profile, track, k = job
    base = core.load_profile(profile)
    p, off = sweep_lf3.make_world(base, k, CFG)
    p = dict(p, wobble_seed=k, wheel_wobble=float(np.random.default_rng(95000 + k).uniform(0.005, 0.035)))
    sim = core.Sim(p, core.make_track(track, 19.0), start_lateral_mm=off, time_limit_s=TRACKS[track], seed=k)
    core.SIM = sim
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            exec(compile(src, "policy", "exec"), {"__name__": "__main__"})
        except (core.SimDone, SystemExit):
            pass
    r = sim.result or {}
    return track, r.get("reason"), r.get("time_s", 0.0), r.get("progress_mm", 0.0)


def evaluate(pool, src, profile, worlds):
    src = src.replace("AUTO_STOP = True", "AUTO_STOP = False")
    res = pool.map(run_one, [(src, profile, t, k) for t in TRACKS for k in range(worlds)], chunksize=1)
    out = {}
    for t in TRACKS:
        rows = [r for r in res if r[0] == t]
        fin = [r for r in rows if r[1] == "finish"]
        out[t] = dict(fin=len(fin), n=len(rows), time=round(float(np.mean([r[2] for r in fin])), 2) if fin else None,
                      prog=[int(r[3]) for r in rows if r[1] != "finish"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("out")
    ap.add_argument("--worlds", type=int, default=12)
    ap.add_argument("--profile", default="spike_3s_geo")
    ap.add_argument("--once", default=None, help="이 체크포인트 하나만 평가하고 끝")
    args = ap.parse_args()
    seen = set()
    tmp = os.path.join(os.path.dirname(os.path.abspath(args.out)), "rl_eval_tmp.py")
    with mp.Pool(max(1, os.cpu_count() - 4)) as pool:      # 4코어는 GPU 학습 프로세스(커널 발행) 몫 — 다 쓰면 학습이 2배 느려짐
        if not os.path.exists(args.out) or not open(args.out, encoding="utf-8").read().strip():
            rule = open(os.path.join(ROOT, "spike/spike_lf3_rule.py"), encoding="utf-8").read()
            t0 = time.time()
            r = evaluate(pool, rule, args.profile, args.worlds)
            with open(args.out, "a", encoding="utf-8") as f:
                f.write(json.dumps(dict(it="E2", res=r, sec=round(time.time() - t0, 1)), ensure_ascii=False) + "\n")
            print("E2", r, flush=True)
        else:
            for line in open(args.out, encoding="utf-8"):
                try:
                    seen.add(json.loads(line)["it"])
                except ValueError:
                    pass
        while True:
            cks = [args.once] if args.once else sorted(
                (f for f in os.listdir(args.run_dir) if re.match(r"policy_it\d+\.pt$", f)),
                key=lambda f: int(re.findall(r"\d+", f)[0]))
            todo = [c for c in cks if (int(re.findall(r"\d+", os.path.basename(c))[0]) if "it" in os.path.basename(c) else c) not in seen]
            for c in todo:
                it = int(re.findall(r"\d+", os.path.basename(c))[0])
                path = os.path.abspath(c if os.path.isabs(c) else os.path.join(args.run_dir, c))   # 내보내기는 cwd=sim에서 돎
                time.sleep(1.0)                                   # 저장이 끝나도록
                e = subprocess.run([TORCH_PY, "-m", "rl.export_policy", path, "--platforms", "spike", "--out", tmp],
                                   cwd=SIM, capture_output=True, encoding="utf-8", errors="replace")
                if e.returncode != 0:
                    print("내보내기 실패", it, e.stderr[-400:], flush=True)
                    seen.add(it)
                    continue
                t0 = time.time()
                r = evaluate(pool, open(tmp, encoding="utf-8").read(), args.profile, args.worlds)
                with open(args.out, "a", encoding="utf-8") as f:
                    f.write(json.dumps(dict(it=it, res=r, sec=round(time.time() - t0, 1)), ensure_ascii=False) + "\n")
                print(it, r, flush=True)
                seen.add(it)
            if args.once:
                break
            if os.path.exists(os.path.join(args.run_dir, "policy.pt")) and not todo and \
                    os.path.getmtime(os.path.join(args.run_dir, "policy.pt")) > time.time() - 10 ** 6 and \
                    any(f.startswith("policy_it") for f in os.listdir(args.run_dir)) and \
                    os.path.exists(os.path.join(args.run_dir, "done.flag")):
                break
            time.sleep(5)


if __name__ == "__main__":
    main()
