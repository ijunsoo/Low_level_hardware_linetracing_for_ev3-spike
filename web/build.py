# =====================================================================
#  Build the browser simulator into a folder of static files (GitHub Pages uses the same build).
#
#    python web/build.py                 # → web/dist/
#    python web/build.py --out _site
#    python -m http.server 8000 --directory web/dist     # then open http://localhost:8000
#
#  sim_bundle.json  the simulator (sim/core.py, the pybricks mock, profiles, photo track) + web/runner.py
#  programs.json    the programs in the program menu, adjusted for the simulator where needed
# =====================================================================
import argparse
import glob
import json
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Brightness of the simulated sensors on the 3-sensor profiles: [white, black] × (left, right, middle)
SIM_VALUE = "[34, 6, 22, 3, 21, 3]"

# id, file, profile, track, English title, Korean title, text replacements for the simulator
PROGRAMS = [
    ("ev3_final", "ev3/ev3_final.py", "ev3_3s_geo", "rightangle",
     "EV3 · competition program", "EV3 · 대회용 완성판",
     [("VALUE=[28, 3, 31, 3, 28, 4]", "VALUE=" + SIM_VALUE),
      ("AUTO_STOP=True, BB_PRINT=True)", "AUTO_STOP=False, BB_PRINT=False)")]),
    ("spike_final", "spike/spike_final.py", "spike_3s_geo2x", "rightangle",
     "SPIKE · competition program (2× gears)", "SPIKE · 대회용 완성판 (기어 2배)",
     [("RAW_MODE = True ", "RAW_MODE = False "), ("VALUE=[68, 12, 72, 12, 116, 12]", "VALUE=" + SIM_VALUE),
      ("AUTO_STOP=True, BB_PRINT=True)", "AUTO_STOP=False, BB_PRINT=False)")]),
    ("spike_rl", "spike/spike_lf3_rl.py", "spike_3s_geo2", "main",
     "SPIKE · neural-network policy (RL)", "SPIKE · 강화학습 신경망 정책", []),
    ("spike_simple", "examples/spike_simple_follower.py", "spike_3s_geo2", "main",
     "SPIKE · simple follower (example)", "SPIKE · 간단한 라인 따라가기 (예제)", []),
    ("spike_drivebase", "examples/spike_drivebase_follower.py", "spike_3s_geo2", "main",
     "SPIKE · DriveBase follower (Pybricks docs style)", "SPIKE · DriveBase 라인 따라가기 (공식 문서 방식)", []),
    ("ev3_simple", "examples/ev3_simple_follower.py", "ev3_3s_geo", "main",
     "EV3 · simple follower (example)", "EV3 · 간단한 라인 따라가기 (예제)", []),
]

SIM_FILES = (["core.py", "eval_lf3.py", "sweep_lf3.py", "micropython.py", "umath.py", "urandom.py",
              "robot_params.json", "viz/track_pts.json"]
             + sorted(os.path.relpath(p, os.path.join(ROOT, "sim")) for p in glob.glob(os.path.join(ROOT, "sim", "profiles", "*.json")))
             + sorted(os.path.relpath(p, os.path.join(ROOT, "sim")) for p in glob.glob(os.path.join(ROOT, "sim", "pybricks", "**", "*.py"), recursive=True)))


def read(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as f:
        return f.read()


def main():
    ap = argparse.ArgumentParser(description="Build the browser simulator")
    ap.add_argument("--out", default=os.path.join(HERE, "dist"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    bundle = {f.replace(os.sep, "/"): read(os.path.join("sim", f)) for f in SIM_FILES}
    bundle["runner.py"] = read("web/runner.py")
    with open(os.path.join(args.out, "sim_bundle.json"), "w", encoding="utf-8") as f:
        json.dump(bundle, f, ensure_ascii=False)

    progs = []
    for pid, path, profile, track, en, ko, edits in PROGRAMS:
        code = read(path)
        for old, new in edits:
            if code.count(old) != 1:
                raise SystemExit("%s: expected one %r to replace for the simulator" % (path, old))
            code = code.replace(old, new)
        progs.append({"id": pid, "file": path, "profile": profile, "track": track, "title": {"en": en, "ko": ko},
                      "code": code})
    with open(os.path.join(args.out, "programs.json"), "w", encoding="utf-8") as f:
        json.dump(progs, f, ensure_ascii=False)

    for name in ("index.html", "worker.js"):
        shutil.copy(os.path.join(HERE, name), os.path.join(args.out, name))
    print("built %s: %d sim files, %d programs" % (args.out, len(bundle), len(progs)))


if __name__ == "__main__":
    main()
