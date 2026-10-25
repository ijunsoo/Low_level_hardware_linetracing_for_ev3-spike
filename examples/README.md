# Examples

Two beginner line followers in plain Pybricks. They run unmodified on a real robot and in the simulator (commands from the repository root; drop `--no-plot` to also save a plot, which needs matplotlib).

| Program | Robot | Run in the simulator |
|---|---|---|
| `spike_simple_follower.py` | SPIKE Prime hub; motors E (left), F (right); EV3 Color Sensors C, A, B (left, middle, right) | `python sim/run_sim.py examples/spike_simple_follower.py --profile spike_3s_geo2 --track main --no-plot` |
| `spike_drivebase_follower.py` | Same SPIKE robot, written like the official Pybricks docs: SPIKE `ColorSensor` and `DriveBase.drive()` | `python sim/run_sim.py examples/spike_drivebase_follower.py --profile spike_3s_geo2 --track main --no-plot` |
| `ev3_simple_follower.py` | EV3 brick; motors A + B (left), C + D (right); Color Sensors S1, S2, S3 (left, middle, right) | `python sim/run_sim.py examples/ev3_simple_follower.py --profile ev3_3s_geo --track main --no-plot` |

Expected on `main`: `result: {'reason': 'finish', 'time_s': 4.347, ...}` for SPIKE, `4.92` s for the DriveBase version and `4.305` s for EV3. All three also finish `easy`, `hard` and `rightangle`; on the competition tracks `mrc`, `mrc_real` and `photo` they end with `lost` — crossings, acute corners and markers need more than a proportional follower.

How they work: every loop scales the three readings to 0 (black) … 100 (white) and steers by `GAIN × (left − right)`. When all three sensors see white, the robot keeps turning toward the side where it saw the line last.

To adapt: measure your sensors on white and black and update the `*_WHITE` / `*_BLACK` values, then tune `SPEED` and `GAIN`. You can try a value without editing the file: `--sub "SPEED = 40=>SPEED = 30"`. With the 2× gear profile `spike_3s_geo2x`, use `--sub "SPEED = 40=>SPEED = 20"`.

Everything the simulator supports is listed in [`docs/SIM_API.md`](../docs/SIM_API.md).
