# Would a physics engine make the simulator match the real robot?

*Review of 2026-10-26 · tool: [`sim/tools/physics_check.py`](../sim/tools/physics_check.py) · [한국어](PHYSICS_ENGINE.ko.md)*

**Short answer: not on its own.** The SPIKE Large Angular Motor is publicly characterized, and with those numbers the simulator's straight-line response follows from physics. Turning, however, is dominated by how much the tires slip on the mat. That is not published anywhere, and a physics engine does not know it either. It has to come from the robot's black-box logs.

## 1. MuJoCo with guessed values

The robot was built in [MuJoCo](https://mujoco.org) 3.17:
- 80 mm wheels, 205 mm apart;
- a ball caster under the sensor bar, with the color sensors 170 mm ahead of the axle;
- two DC motors driving the wheels through gears.

It was compared with the real robot on the same measure as [`pivot_resp.py`](../sim/tools/pivot_resp.py): how much the yaw rate picks up 20, 40 and 60 ms after a pivot command.

This first sweep tried 112 combinations of the unknowns: motor stall torque, motor inertia, tire and caster friction, center of mass, and speed before the pivot. The sweep is not in the tool; `--mujoco` runs the public-data version in section 4.
- The 1× gear's yaw rate at 60 ms ranged from about 40 to 150 °/s.
- 2× gears picked up 0.9–1.4 times as fast as 1×. The real robot: 1.75, from a single 2× pivot. (On 2026-10-31, 7.3 s of 2× logs showed the 2× robot turning about as fast as 1× → section 6.)
- The closest combination was still 30 % off, and it was chosen by looking at the real numbers.

With guessed values, an engine is no better than the current hand-fitted model.

## 2. What is published

| Source | Values |
|---|---|
| Pybricks firmware, [`servo_settings.c`](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/motor/servo_settings.c) (`model_technic_l_angular`) | Pybricks' identified model of the Large Angular Motor. Decoded with the scale factors in `observer.c`: <ul><li>torque 0.0623 N·m per volt</li><li>back-EMF term 0.0256 N·m·s/rad</li><li>inertia 0.00122 kg·m² at the output shaft (rotor and internal gearbox)</li><li>friction 0.023 N·m</li></ul>The motor alone has a 48 ms time constant. |
| Pybricks [`motor_model.py`](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/doc/control/motor_model.py) | How those numbers are made. Its Medium Angular Motor experiment reproduces the firmware's medium-motor numbers, which confirms the decoding (`physics_check.py` prints both). |
| Pybricks [`battery.c`](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/battery.c), [`pb_type_motor.c`](https://github.com/pybricks/pybricks-micropython/blob/master/pybricks/common/pb_type_motor.c) | `Motor.dc(duty)` applies battery voltage × duty, with no compensation (capped at 9 V). |
| [LEGO technical specification, 45602](https://assets.education.lego.com/v3/assets/blt293eea581807678a/bltb9abb42596a7f1b3/5f8801b5f4c5ce0e93db1587/le_spike-prime_tech-fact-sheet_45602_1hy19.pdf?locale=en-us) | At 7.2 V: <ul><li>no load: 175 rpm ± 15 %, 135 mA</li><li>stall: 25 N·cm, 1.9 A</li></ul>Encoder: 360 counts per turn, updated at 100 Hz. |
| [Philo's motor comparison](https://www.philohome.com/motors/motorcomp.htm) | Weight 71 g. At 9 V: <ul><li>no load: 213 rpm</li><li>stall: 33.5 N·cm, 1.8 A</li></ul> |
| Retail listing ([BuyWisely](https://buywisely.com.au/product/lego-education-45601-spike-prime-large-hub)) | Large Hub about 170 g. |

## 3. What it explains

**Driving straight.** If the tires do not slip, the forward time constant is τ = (J + M·(r·g)²/2) / k:
- J is the motor inertia, M the robot mass, r the wheel radius, g the gear ratio and k the back-EMF term.
- The robot mass is about 0.57–0.72 kg, from part weights.

With the Pybricks values, the 1× gear gives **0.10–0.11 s**. The simulator's 1× profile already uses 0.10 s, and MuJoCo agrees (88–114 ms). LEGO's spec-sheet stall torque would give 0.18–0.21 s, twice as slow, so the Pybricks model is the one that fits.

**Turning.** The turning time constant fitted from real black boxes is 0.05 s. That is the same as the motor-alone time constant (0.048 s), while gripping tires would make it about 0.1 s, because of the robot's own inertia.
- So while the robot pivots, the wheels slip a lot and the motors hardly feel the robot.
- This is probably also why the real robot turns about 1.5 times less than its wheel speeds say (`w_eff_scale` 1.5).

## 4. What it does not explain

**Tire slip.** How much the tires slip depends on these tires and this mat, and it decides how the robot turns.

**The 2× gear's turning.** Scaling the 1× time constants by no-slip physics gives 0.263 s forward and 0.13 s turning.
- With that, the SPIKE competition program wanders for 10.3 s at the first right angle of the default world. The real robot finishes.
- Keeping turning at 0.10 s and changing only forward to 0.263 s gives practically today's results.

| 2× model (forward / turning) | main | hard | rightangle | photo@1.3 |
|---|---|---|---|---|
| `spike_3s_geo2x` before 10-31 (0.20 / 0.10 s) | 2.4 s · 8/8 | 2.0 s · 8/8 | 4.0 s · 8/8 | lost · 1/8 |
| no-slip physics (0.263 / 0.13 s) | 2.6 s · 8/8 | 2.1 s · 8/8 | **10.3 s** · 8/8 | lost · 1/8 |
| physics for forward only (0.263 / 0.10 s) | 2.3 s · 8/8 | 2.0 s · 8/8 | 4.1 s · 8/8 | lost · 2/8 |

Each cell: time in the default world, then how many of 8 random worlds finish. All three rows use the slip from before 10-31 (w_eff 1.5, 32 ms command delay) and the 2× constants (the `GEAR=2` line). The refit model is in section 6.

**MuJoCo still disagrees on turning.** With the public motor values, its tires grip, so pivots pick up more slowly than on the robot.

| Yaw-rate pickup (°/s) | 20 ms | 40 ms | 60 ms | 2× ÷ 1× at 60 ms |
|---|---|---|---|---|
| MuJoCo, 1× gear | 6–34 | 33–70 | 58–101 | |
| Real robot, 1× gear | 54 | 84 | 114 | |
| MuJoCo, 2× vs 1× | | | | 0.83–1.24 |
| Real robot, 2× vs 1× (one pivot) | | | | 1.75 |

**The comparison data are thin.**
- The only real 2× pivot comes from a 0.39 s black box (as of 10-26; 7.3 s arrived on 10-31 → section 6).
- The 1× pivots come from a different build, with the sensors 130 mm ahead (log `E7_130`).

## 5. Conclusions

- **The motor does not need measuring.** Use the Pybricks model.
- **The mass** can be estimated from part weights, or weighed.
- **Tire slip** can only come from black-box logs. There are enough for the 1× robot. The 2× robot needs a few more runs of the competition program, which prints its last 2 s when stopped. → Fitted on 10-31 (section 6).
- **A full physics engine is not worth adding now.**
  - It does not know the slip either.
  - Its Python package does not run in the browser simulator (Pyodide).
  - The GPU batch simulator would need a separate implementation.
  - The current model already has the right structure: separate forward and turning time constants.

## 6. Update: the 2× robot refit on 7.3 s of logs (2026-10-31)

Two black boxes from the 2× robot running the competition program:
- `diag_logs/bb_2026-10-31_2x_6s.txt`: 6.2 s. The last 0.3 s, where all three sensors read almost 0 (the robot was probably lifted), is left out.
- The last 1.1 s of `diag_logs/bb_2026-10-31_2x_final.txt`. The first 0.9 s is left out: the steering was +60 to +100 but the robot hardly turned, so it was probably stuck.

**Method: replaying the log** ([`bb_replay.py`](../sim/tools/bb_replay.py)). Instead of looking at a few pivot starts, the recorded steering is fed into the simulator's drive model moment by moment. The simulated turn rate is then compared with the real gyro at every moment.

| 2× model | Command delay | Turning time constant | w_eff | Difference from the real gyro (6.2 s / 1.1 s) |
|---|---|---|---|---|
| before 10-31 (fitted to one pivot) | 32 ms | 0.10 s | 1.5 | 154 / 245 °/s |
| current `spike_3s_geo2x` | 40 ms | 0.05 s | 3.1 | 62 / 113 °/s |

- **For the same steering, the 2× robot turns at 97 % of the 1× robot's rate** (1×: gear 1.667 ÷ w_eff 1.5; 2×: 3.333 ÷ 3.1). It also responds as quickly as 1× (time constant 0.05 s).
  - The gears double the wheels' top speed, but while turning the motors run out of torque, so the robot turns only as fast as the 1×.
  - This fits section 3 (the wheels slip a lot while turning) and MuJoCo's 0.83–1.24 in section 1. The 1.75 from a single pivot did not hold.
- **The old model turned 1.5–2 times too much.** In the 6.2 s log, at moments with steering beyond ±75, the mean turn rate was 221 °/s on the robot, 319 °/s in the old model and 210 °/s in the current one. The peaks were 412, 757 and 418 °/s.
- **The forward time constant (0.2 s) is unchanged.** The black box has no wheel speeds, so it cannot be measured.
- **The 1.5× model (`spike_3s_geo2_15x`) is not measured.** It interpolates between 1× and the 2× model from before 10-31, so it may overstate turning.

The competition-program test of section 4 (2× constants), rerun on the current model:

| 2× model | main | hard | rightangle | photo@1.3 |
|---|---|---|---|---|
| before 10-31 (0.20 / 0.10 s · w_eff 1.5) | 2.4 s · 8/8 | 2.0 s · 8/8 | 4.0 s · 8/8 | lost · 1/8 |
| current `spike_3s_geo2x` (0.20 / 0.05 s · w_eff 3.1) | 2.6 s · 8/8 | 6.1 s · 8/8 | 3.9 s · 8/8 | lost · 0/8 |

The random worlds draw w_eff within ±7 % of the model's value (2.9–3.3 for the current model).

## Reproduce

```bash
python sim/tools/physics_check.py            # decode and check the motor model; time constants vs the profiles
pip install mujoco
python sim/tools/physics_check.py --mujoco   # MuJoCo robot vs the real black boxes
python sim/tools/physics_check.py --final    # SPIKE competition program: current and pre-10-31 2× models vs physics-scaled
python sim/tools/bb_replay.py diag_logs/bb_2026-10-31_2x_6s.txt diag_logs/bb_2026-10-31_2x_final.txt@-1100 --fit   # section 6
```
