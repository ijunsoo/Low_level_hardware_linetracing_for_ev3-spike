# Pybricks line-following simulator · SPIKE Prime & EV3

English | [한국어](README.ko.md)

[![CI](https://github.com/ijunsoo/Low_level_hardware_linetracing_for_ev3-spike/actions/workflows/ci.yml/badge.svg)](https://github.com/ijunsoo/Low_level_hardware_linetracing_for_ev3-spike/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

Run your [Pybricks](https://pybricks.com) line-following program on a PC or in a web browser: the same file that runs on the robot, with its light values set for the simulated sensors.
The simulator is calibrated against real LEGO SPIKE Prime and EV3 robots.

**▶ [Open the browser simulator](https://ijunsoo.github.io/Low_level_hardware_linetracing_for_ev3-spike/)** — nothing to install.

![The EV3 competition program running on four simulated tracks](docs/img/sim_tracks.png)

**Contents** · [Features](#features) · [Quick start](#quick-start) · [Run your own program](#run-your-own-program) · [Browser simulator](#browser-simulator) · [The competition robots](#the-competition-robots) · [Documentation](#documentation) · [Repository layout](#repository-layout)

## Features

| Feature | Details |
|---|---|
| **The same program file** | A mock `pybricks` package runs the file you put on the robot. Set its light values for the simulated sensors and read them as reflected light, not raw mode (`--sub` does both without editing the file). |
| **Calibrated to real robots** | Motor lag, turning slip, gear ratio, sensor footprint, sensor noise and lag, and the hub's loop rate are fitted to black-box logs from real robots. |
| **Many tracks** | Curves, right angles, crossings and loops, a competition field traced from a photo, random competition arenas, or your own. |
| **Measured, not guessed** | Randomized worlds, paired A/B comparisons, regression tests, and a decoder for the robot's black-box logs. |
| **GPU and RL** | A PyTorch version runs 16,384 robots at once. Train a PPO policy and export it as integer-only Pybricks code. |
| **In the browser** | The same Python simulator runs in a web page with [Pyodide](https://pyodide.org), with a robot editor and an arena editor. |

## Quick start

**In the browser:** [open the simulator](https://ijunsoo.github.io/Low_level_hardware_linetracing_for_ev3-spike/), pick a program and press **▶ Run**.

**On a PC** (Python 3.9 or newer):

```bash
git clone https://github.com/ijunsoo/Low_level_hardware_linetracing_for_ev3-spike.git
cd Low_level_hardware_linetracing_for_ev3-spike
pip install -r requirements.txt
python sim/run_sim.py examples/spike_simple_follower.py --profile spike_3s_geo2 --track main
```

It prints the result and saves a plot:

```text
result: {'reason': 'finish', 'time_s': 4.347, 'progress_mm': 2136.3, 'track_mm': 2174.3, 'mean_lat_mm': 17.23}
plot saved: sim_result.png
```

For EV3, run `examples/ev3_simple_follower.py` with `--profile ev3_3s_geo`. More examples are in [`examples/`](examples/).

## Run your own program

1. Write a normal Pybricks program for SPIKE Prime or EV3.
2. Pick a robot profile that matches your wiring (`python sim/run_sim.py --list`), or [describe your own robot](docs/SIM_API.md).
3. Run it:

   ```bash
   python sim/run_sim.py my_program.py --profile spike_3s_geo --track main
   ```

Every run ends with one of these results:

| Result | Meaning |
|---|---|
| `finish` | Reached the end of the track, having been on the line (within 40 mm) in the last 500 mm. |
| `lost` | Went more than 200 mm from the line, or reached the end without being on the line in the last 500 mm. |
| `timeout` | Hit the time limit (60 s by default). |
| `stall` | The program kept looping without waiting, so simulated time stopped. |

Useful options:

| Option | Effect |
|---|---|
| `--track photo@1.3` | Pick a track; `@1.3` scales it. `--list` shows the tracks. |
| `--offset 20` | Start 20 mm beside the line. |
| `--seed 3` | Use different sensor noise. |
| `--lost 450` | Allow 450 mm before `lost`. |
| `--sub "OLD=>NEW"` | Change text in the program without editing the file. |
| `--no-plot` | Skip the plot (no matplotlib needed). |

What the mock supports, and what it doesn't, is listed in [`docs/SIM_API.md`](docs/SIM_API.md).

## Browser simulator

The page runs the repository's own simulator (`sim/core.py` and the Pybricks mock) in a Web Worker with Pyodide, so results match the PC.

- **Programs:** both competition programs, the reinforcement-learning policy and the examples. You can also paste your own.
- **Robot editor:** sensors (number, position, port, white and black brightness), wheel size, wheel spacing, gear ratio and motor ports, with a live top view.
- **Arenas:** the built-in tracks, random competition arenas (pick a number), or your own.
- **Testing:** run once and watch, or test in 8 randomized worlds.
- **Saving:** your robots and arenas stay in your browser. Arenas can be shared as JSON files.
- **Export results:** save a JSON file with the code, robot, arena, options, worlds, results and the simulator's commit, so anyone can repeat the run.

In the arena editor, write the route one piece per line. It starts at (0, 0), heading right.

| Line | Piece |
|---|---|
| `S 500` | Straight, 500 mm. |
| `A 150 90` | Curve with a 150 mm radius, 90° to the left (negative = right). |
| `T -90` | Sharp corner: turn 90° right on the spot. |
| `M x1 y1 x2 y2` | Marker line that the robot does not follow (the sensors see it). |
| `W 20` | Line width, 20 mm. |
| `# ...` | Comment. |

To run the page locally:

```bash
python web/build.py                                   # → web/dist/
python -m http.server 8000 --directory web/dist       # then open http://localhost:8000
```

GitHub Actions rebuilds and publishes the page on every push to `main` ([`pages.yml`](.github/workflows/pages.yml)).

## The competition robots

The simulator was built for two robots in the Robotex Korea LEGO line-following competition (20 mm line, right angles, crossings and loops).

- Both have three EV3 color sensors in a row, 23 mm apart and 170 mm ahead of the axle.
- They always drive at full power and only steer. The one exception is a 50 ms brake before a right angle at the end of a long straight.

| | SPIKE Prime | EV3 |
|---|---|---|
| Final program | [`spike/spike_final.py`](spike/spike_final.py) | [`ev3/ev3_final.py`](ev3/ev3_final.py) |
| Light-value tool | [`spike/spike_value.py`](spike/spike_value.py) | [`ev3/ev3_value.py`](ev3/ev3_value.py) |
| Sensor reading | EV3 color sensors through `PUPDevice`, raw red reflection | `ColorSensor.reflection()` (RGB) by default |

Everything a student needs to change is in the last line of each final program.
The code comments are in Korean. [`README.ko.md`](README.ko.md) has the wiring, the settings and the experiment results.

### What we learned

- **Small loops can trap the robot** when the sensor arm (170 mm) is longer than the loop radius. Whether the second turn catches the exit at the crossing decides it.
- **2× gears did not make turning any faster.** Replaying the steering from 7.3 s of real 2× black boxes through the simulator showed that the 2× robot turns at 97 % of the 1× robot's rate for the same steering: the motors run out of torque. The 2× model (`spike_3s_geo2x`) is fitted to these logs.
- **1.5× gears look a little better** (simulator). With one motor per side, the 2× robot uses only about a third of its top speed. At 1.5× with retuned constants the SPIKE final finished 49 of 54 runs instead of 45 at 2×, and right angles took 3.8 s instead of 4.2 s. The 1.5× model is not measured yet and may overstate turning. The SPIKE final defaults to `GEAR=1.5`; a `GEAR=2` line is kept for 2× robots. Retuning that line on the refit 2× model (120 candidates) did not beat it on unseen arenas, so it is unchanged.
- **Turning depends on tire slip.** The motor is publicly characterized and matches the simulator, but tire slip has to come from black-box logs ([physics engine review](docs/PHYSICS_ENGINE.md)).
- **On the EV3, RGB reflection followed the line better** than raw red reflection, even with both calibrated.

## Documentation

| Document | Contents |
|---|---|
| [`sim/README.md`](sim/README.md) | Simulator guide: profiles, tracks, evaluation tools, regression tests, GPU simulator and RL, calibration, limitations |
| [`docs/SIM_API.md`](docs/SIM_API.md) | Supported Pybricks API, robot profile keys, track format |
| [`docs/PHYSICS_ENGINE.md`](docs/PHYSICS_ENGINE.md) | Would a physics engine make the simulator match the real robot? Public motor data, MuJoCo tests, what still has to come from black boxes |
| [`examples/`](examples/) | Short beginner programs that run in the simulator and on the robot |
| [`README.ko.md`](README.ko.md) | Korean: the competition robots, wiring, settings, how they drive, experiment results |

## Repository layout

| Folder | Contents |
|---|---|
| `sim/` | Simulator, tools, regression tests; `sim/rl/` GPU simulator and RL |
| `spike/`, `ev3/` | Robot programs |
| `examples/` | Beginner examples |
| `web/` | Browser simulator (Pyodide), built by `web/build.py` |
| `docs/` | Reference documents and figures |
| `diag_logs/` | Black-box and diagnostic logs from the real robots |

## Contributing

Issues and pull requests are welcome, in English or Korean.
Before opening a pull request, run `python sim/regress.py`; CI also runs it on every push.

## License

[MIT](LICENSE)
