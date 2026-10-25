# Simulator (CPU · GPU) and reinforcement learning

English | [한국어](README.ko.md)

A digital twin that runs real Pybricks robot programs **unmodified** on a PC.
It models motor acceleration (asymmetric first-order lag), turning slip, the sensor footprint, noise and lag, and the control-loop rate measured on real hubs.
Both SPIKE Prime (with EV3 color sensors) and EV3 (Pybricks v4) are supported.

## Contents

1. [Setup](#1-setup)
2. [Layout](#2-layout)
3. [Profiles and tracks](#3-profiles-and-tracks)
4. [Running and evaluating](#4-running-and-evaluating)
5. [Analysis tools (tools/)](#5-analysis-tools-tools)
6. [Regression tests](#6-regression-tests)
7. [GPU simulator and RL (rl/)](#7-gpu-simulator-and-rl-rl)
8. [Calibrating against a real robot](#8-calibrating-against-a-real-robot)
9. [Limitations](#9-limitations)

The API the mock supports, the profile keys and the track format are documented in [`docs/SIM_API.md`](../docs/SIM_API.md).

---

## 1. Setup

Python 3.9 or newer. Run every command from the repository root.

```bash
pip install -r requirements.txt        # numpy, matplotlib
pip install -r requirements-gpu.txt    # optional: PyTorch for the GPU simulator and RL
```

| Use | Needs |
|---|---|
| CPU simulator, tools, regression tests | numpy |
| Plots (`run_sim.py` without `--no-plot`) | matplotlib |
| GPU simulator, GPU sweeps, PPO | torch — picks Apple MPS, CUDA or CPU automatically |

---

## 2. Layout

| Path | Role |
|---|---|
| `core.py` | Physics (motors, turning slip), sensors, tracks, simulation loop |
| `pybricks/` | Pybricks mock — robot programs `import` it unchanged (SPIKE `PrimeHub`, `Motor`, `ColorSensor`, `PUPDevice`; EV3 `EV3Brick`, `ColorSensor`, `LUMPDevice`; `DriveBase`, …) |
| `profiles/` | Hardware profiles (inherit with `extends`) |
| `run_sim.py` | Run one robot program and save a plot |
| `eval_lf3.py`, `recap_stats.py` | Evaluate over many randomized worlds, line-recapture statistics |
| `sweep_lf3.py` | Sweep constants of a robot program (CPU) |
| `regress.py` | Regression tests |
| `bb_parse.py` | Decode a real robot's black-box log |
| `diag_parse.py` | Decode a real robot's diagnostic log → calibrate a profile |
| `photo_track.py` | Competition-field photo → track (`viz/track_pts.json`) |
| `tools/` | Analysis tools ([section 5](#5-analysis-tools-tools)) |
| `rl/` | GPU simulator, GPU sweeps, reinforcement learning ([section 7](#7-gpu-simulator-and-rl-rl)) |
| `viz/gpu_view.html` | Replay viewer for the GPU simulator |
| `runs_geo/` | Figures used in the top-level README |

---

## 3. Profiles and tracks

Choose a profile with `--profile NAME`. To describe your own robot, see [`docs/SIM_API.md`](../docs/SIM_API.md).

| Profile | What it is |
|---|---|
| `spike_3s_geo2` | **SPIKE, 1× gearing (default).** Measured geometry (3 sensors 23 mm apart, 170 mm ahead of the axle; track width 205 mm; 80 mm wheels) + turn calibration (time constant 0.05 s, w_eff 1.5) |
| `spike_3s_geo2x` | **SPIKE, 2× gearing.** geo2 + gears doubled + motor/turn time constants ×3 (torque limit) |
| `ev3_3s_geo` | EV3 (sensor ports S1·S3·S2, motors A·B / C·D, no gyro) |
| `spike_3s_geo`, `spike_3s_real`, `spike_3s`, `ev3_3s`, `spike_4s` (= `robot_params.json`) | Earlier stages these profiles inherit from (older measurements) |
| `spike_3s1_geo`, `spike_4row_geo`, `spike_3s1f_geo` (+ `_geo2`) | 4-sensor experiments: 3 + D read only / 4 in a row / 3 + one ahead in the middle |

When evaluating with geo2 or geo2x, also pass `--weff 1.4,1.6`.

| Track | What it is |
|---|---|
| `easy`, `main`, `hard` | Basic curve tracks |
| `rightangle` | Straights followed by three right angles |
| `mrc`, `mrc_real` | Competition-style track (1.6× scale / real size) |
| `photo` | Track extracted from a photo of a competition field (195 cm mat, 6.89 m) |
| `photo@SCALE` | The photo track scaled up. `photo@1.3` makes the loop radius ≈124 mm, matching the real loop |

---

## 4. Running and evaluating

```bash
python sim/run_sim.py --list                                              # profiles and tracks
python sim/run_sim.py ev3/ev3_final.py --profile ev3_3s_geo --track hard  # saves ./sim_result.png
python sim/run_sim.py spike/spike_lf3_rule.py --profile spike_3s_geo2 --track photo --no-plot
```

A run ends with one of these results:

| `reason` | Meaning |
|---|---|
| `finish` | Reached the end of the track |
| `lost` | Got more than 200 mm from the line (`Sim.LOST_MM`; measured at the sensors on the measured-geometry profiles) |
| `timeout` | Hit the time limit (`--limit`, default 60 s) |
| `stall` | 200,000 calls in a row without simulated time passing — an endless loop that never waits or touches the hardware |

- **Simulated time**: `wait()` advances simulated time, and every motor, sensor and gyro call costs the time measured on real hubs (`_cost` in `core.py`), so loop rates match the robot. The program itself runs as fast as your PC allows.
- **Sensor modes**: the mock simulates reflected light (mode 0) only. Programs with `RAW_MODE = True` are run with `RAW_MODE False` and the simulated brightness `VALUE` (SPIKE `[34, 6, 22, 3, 21, 3]`), using `--sub` or `--base`.
- **Final programs** (`spike_final.py`, `ev3_final.py`) keep the values to change in their last `line_trace(...)` line, which `--base` cannot reach. Evaluate a copy with that line edited (`VALUE=[34, 6, 22, 3, 21, 3]`, `AUTO_STOP=False`) and set `RAW_MODE`/`AUTO_CAL` with `--base`.
  The EV3 final reads RGB reflection (`ColorSensor`) by default, so it needs no `RAW_MODE` change; run its red-light path with `--base '{"RED_LIGHT": true, "RAW_MODE": false, "VALUE0": [34, 6, 22, 3, 21, 3]}'`.

---

## 5. Analysis tools (tools/)

| File | Role |
|---|---|
| `corner_eval.py` | Evaluate variants: finish rate, time, max deviation, banking, pivots, loop rate. `--base` shared constants, `--dump` per-world JSON |
| `paired.py` | Compare `--dump` results world by world |
| `loop_trace.py` | Loop/crossing sections: results and axle/sensor trajectory plots |
| `ra_trace.py` | First right angle of the `rightangle` track: sensor overshoot, re-entry angle |
| `stop_test.py` | Stop-button test: output after stopping, how long the program holds on, how far the wheels keep rolling |
| `step_dump.py` | Step-by-step sensors, error, steering and position for one world |
| `pivot_resp.py` | How fast turning picks up after a pivot: real black box vs simulation |
| `make4.py`, `make_guard.py`, `make_sent.py`, `make_bank.py` | Generate experiment variants (4 sensors, loop guard, front sentinel, curve forced turn) → `tools/variants/` (not in git) |

```bash
B='{"RAW_MODE": false, "VALUE": [34, 6, 22, 3, 21, 3], "AUTO_CAL": false}'
python sim/tools/corner_eval.py '[["85", {"ST_BANK": 85}], ["75", {}]]' --src spike/spike_lf3_rule_e8r10_170.py \
  --profile spike_3s_geo2x --fwd 170 --weff 1.4,1.6 --base "$B" \
  --tracks rightangle,main,hard,photo@1.3 --lost 450 --worlds 32 --dump bank.json
python sim/tools/paired.py "bank.json:85" "bank.json:75"
python sim/tools/pivot_resp.py diag_logs/bb_2026-10-24_E8R_2x_short.txt --sim spike_3s_geo2x
```

`--fwd 170` places the three sensors in a row at that distance; `--fwd 0` keeps the profile's sensor positions (always use 0 for 4-sensor profiles).

---

## 6. Regression tests

Run these after changing `core.py` or the mock. Results are compared with the goldens (`regress_golden.json`) down to trajectory hashes.

```bash
python sim/regress.py              # compare (15 cases, about 10 s)
python sim/regress.py --only lf3   # a subset
python sim/regress.py --capture    # update the goldens after an intended change
```

Cases: sensor response curve, SPIKE/EV3 mock smoke tests (`tests/`), the 3-sensor rule programs (`spike_lf3_rule.py`, `ev3_lf3_rule.py`), and **SPIKE↔EV3 parity** (both programs give the same trajectory under the same physics and I/O).

---

## 7. GPU simulator and RL (rl/)

| File | Role |
|---|---|
| `tracks.py` | Random tracks generated from competition rules (20 mm line, min. radius 100 mm, right/acute angles, loop crossings, zigzags, S-bends, markers) |
| `fastsim.py` | Same physics and sensors as `core.py`, vectorized with numpy over N robots |
| `torchsim.py` | PyTorch version of the same simulator. 16,384 robots at once, about 650k decisions/s on an Apple GPU |
| `rule_lf3.py` | Vectorized version of the 3-sensor rule logic (constants read from `spike/spike_lf3_rule.py`) |
| `sweep_gpu.py`, `sweep_geo.py` | Large constant/layout searches on the GPU (only the top settings are re-checked on the CPU simulator) |
| `env.py`, `ppo.py` | RL environment and PPO (residual policy, asymmetric actor-critic) |
| `export_policy.py` | Trained policy → integer-arithmetic Pybricks program for the real robot |
| `parity_test.py`, `cpu_eval_ckpt.py`, `replay_test.py`, `watch_train.py` | CPU/GPU agreement, checkpoint evaluation, replay, live training monitor |

```bash
cd sim
python -m rl.torchsim --check --bench                      # agreement with the numpy version, speed
python -m rl.sweep_gpu --n 192 --no-slow --top-out top.json
python sweep_lf3.py --phase validate --configs top.json --out val.json   # re-check the top settings on the CPU
python -m rl.ppo --layout row3 --mode steer_only --actor-hid 16 --act relu
python -m rl.export_policy rl/runs/<name>/policy.pt
```

- **`--no-slow`**: power fixed at 100 (no slowing down). `ppo --mode steer_only` learns a steering correction only.
- **Deploying to a robot**: the network becomes straight-line fixed-point code (×1000 integers), so it runs without floats on both SPIKE and EV3. The exporter computes intermediate value ranges to rule out overflow, then self-checks the generated code against the reference on 4,000 inputs including extremes.
- **Kept training runs**: `runs/row3_res2_h16_v3` → `spike/spike_lf3_rl.py`, `ev3/ev3_lf3_rl.py` · `runs/geo_steer_v2` → `spike/spike_lf3_rl_geo.py`

RL v3 (3 sensors in a row, residual policy) vs the rule-based program, CPU simulator, 100 worlds:

| | `mrc` finished | `photo` finished |
|---|---|---|
| Rule-based (defaults at the time) | 26/100 · 16.91 s | 90/100 · 8.96 s |
| RL v3, 1 ms policy compute | 52/100 · 15.75 s | 98/100 · 8.71 s |

> RL v3 was trained on the dynamics before the 2× gearing and its torque limit. To use it on the current robot, retrain under `spike_3s_geo2x`.

---

## 8. Calibrating against a real robot

- **Black box**: the robot programs record [3 sensors, steering, power, state, turn rate] every 4 ms for 2 s (500 entries) and print them as `BB`/`BBD` lines when they stop → `python sim/bb_parse.py log.txt --events`
- **Diagnostics**: run `spike/spike_diag.py` (EV3: `ev3/ev3_diag.py`) on the robot → `python sim/diag_parse.py log.txt --sim spike_3s` compares call cost, sensor refresh and latency between the robot and the simulator.
- **Fitted so far**:
  - Real control loop ≈ 734 updates/s
  - Turning: time constant ≈ 0.05 s, actuation delay ≈ 36 ms, w_eff ≈ 1.5 (5 black boxes) → `spike_3s_geo2`
  - 2× gearing: turning picks up about as fast as with 1× → time constants ×3 → `spike_3s_geo2x` (rough: based on one real 2× pivot)

---

## 9. Limitations

- **Best in the simulator ≠ best on the robot.** Use it to shortlist candidates, then test on the robot.
- Slip is modeled only as a lateral-acceleration limit (understeer); voltage sag and floor-friction spread are absorbed by domain randomization.
- Sensors reproduce reflected light only — no raw mode (mode 3) and no color.
- Small loops are much more pessimistic on the 1× `photo` track than on the real field (the loop is smaller than the real one). Compare loop behavior on `photo@1.3`.
- On the robot, a line straddling two sensors made both read 30–45 % and produced a false right-angle signal; the simulator does not reproduce this.
- The torque limit of `spike_3s_geo2x` (time constants ×3) is a rough fit to one real 2× pivot; it will be refit as more 2 s black boxes come in.
