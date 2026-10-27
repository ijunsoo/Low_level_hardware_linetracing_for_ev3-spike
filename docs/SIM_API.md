# Simulator API reference

What the CPU simulator in `sim/` supports, derived from `sim/core.py` and the
Pybricks mock in `sim/pybricks/`. Setup, the analysis tools and the GPU
simulator are described in [`sim/README.md`](../sim/README.md). Two beginner
programs that run unmodified are in [`examples/`](../examples/README.md).

1. [How a program runs](#1-how-a-program-runs)
2. [Supported Pybricks API](#2-supported-pybricks-api)
3. [Ports and simulated hardware](#3-ports-and-simulated-hardware)
4. [Robot profiles](#4-robot-profiles)
5. [Tracks](#5-tracks)

---

## 1. How a program runs

```bash
python sim/run_sim.py PROGRAM.py --profile PROFILE --track TRACK --no-plot
python sim/run_sim.py --list     # profiles and tracks
python sim/run_sim.py --help     # every option
```

`run_sim.py` puts `sim/` first on `sys.path`, so `from pybricks... import ...`
loads the mock in `sim/pybricks/`. It builds the robot from the profile, puts it
at the start of the track facing along the line, and runs the program as
`__main__` under CPython. The file is never changed; `--sub "OLD=>NEW"`
replaces text in memory first (repeatable, fails if `OLD` is not found).

| Option | Meaning |
|---|---|
| `--profile NAME` | robot profile from `sim/profiles/` ([section 4](#4-robot-profiles)). Without it (and without `--params`) `sim/robot_params.json` (= `spike_4s`) is used |
| `--params FILE` | a parameter JSON merged over the defaults instead of a profile — `extends` is **not** resolved |
| `--track NAME` | track (default `mrc`), or `photo@SCALE` |
| `--offset MM` | start this far left (+) or right (−) of the line |
| `--seed N` | random seed of the sensor noise (default 0) |
| `--limit S` | time limit in simulated seconds (default 60) |
| `--lost MM` | distance from the line that counts as lost (default 200) |
| `--no-plot` | skip the plot (otherwise saved to `--out`, needs matplotlib) |

### Result

The run stops at the first of these events and prints
`result: {'reason', 'time_s', 'progress_mm', 'track_mm', 'mean_lat_mm'}`:

| `reason` | When |
|---|---|
| `finish` | progress along the line reaches the last 40 mm (`Sim.FINISH_MM`), and the tracked point was within 40 mm of the centre line (`Sim.ON_LINE_MM`) somewhere in the last 500 mm (`Sim.FINISH_WINDOW_MM`) |
| `lost` | the tracked point is more than 200 mm (`Sim.LOST_MM`, `--lost`) from the centre line, or it reaches the end without having been on the line in the last 500 mm |
| `timeout` | simulated time passed `--limit` |
| `stall` | 200 000 mock calls in a row cost no simulated time (a busy loop, see below) |
| `none` | the program returned by itself (no main loop) |

The *tracked point* is `track_ref_mm` ahead of the middle of the wheel axle
(0 = the axle; the `*_geo*` profiles use 170 = the sensor row). Progress and
distance are measured against the track's centre line only — marker lines
drawn across the track do not count. Both are checked once per sensor frame
(`sensor_hz`). `progress_mm` is the distance along the line, `mean_lat_mm` the
mean distance of the tracked point from the line. `hub.light.on()` /
`off()` calls are counted and printed as `light events`.

### Simulated time

The clock only moves when the program calls the mock. Plain Python costs
nothing; the call costs of the measured profiles include the real robot's
Python overhead per loop.

| Call | Simulated time |
|---|---|
| `wait(ms)` | `ms` |
| `PUPDevice.read()`, `ColorSensor.reflection()` / `color()` / `ambient()` / `rgb()` | `call_cost_us.color_read` µs; default 1 / (`loop_hz` × number of sensors) s |
| `Motor.dc()` / `run()` / `stop()` / `brake()` / `hold()` | `call_cost_us.motor_dc` µs (default 0) |
| `Motor.angle()` / `speed()` / `reset_angle()`, creating a `Motor` | `call_cost_us.motor_read` µs (default 0) |
| `GyroSensor.speed()` / `angle()` / `reset_angle()` | `call_cost_us.gyro_read` µs (default 0) |
| everything else (`hub.imu`, battery, buttons, light, display, speaker, `StopWatch`, …) | 0 |

- A loop whose mock calls never move the clock — zero-cost motor or gyro
  calls, or polling `hub.buttons`, `hub.imu`, `hub.battery` or
  `StopWatch.time()` — ends with `stall` after 200 000 calls. For example
  `while not hub.buttons.pressed(): pass` ends with `stall` (`pressed()` is
  always empty). With a `wait()` inside it ends with `timeout`.

---

## 2. Supported Pybricks API

Only what is listed here exists. Anything else raises `ImportError` (names
that are not in a module) or `AttributeError` (methods that are not on an
object).

### `pybricks.hubs`

`PrimeHub(...)` and `EV3Brick(...)` accept and ignore any arguments.
`EV3Brick` has `screen` instead of `display` and **no** `imu`.
`InventorHub`, `EssentialHub`, `TechnicHub` and the other hubs do not exist.

| Attribute / method | In the simulator |
|---|---|
| `battery.voltage()` | `int(battery_mv × battery_scale)` → 8000 by default |
| `battery.current()` | 0 |
| `buttons.pressed()` | always an empty set |
| `speaker.beep(...)`, `speaker.play_notes(...)` | no-op (no time passes); other speaker methods do not exist |
| `light.on(color)`, `light.off()` | recorded as events; `light.blink(...)` is a no-op |
| `display.*`, `screen.*`, `system.*` | any method call is accepted and returns `None` |
| `imu.angular_velocity(Axis.Z)` | yaw rate in deg/s, left (counter-clockwise from above) turn positive. No noise or drift; delayed by `imu_lag_ms` |
| `imu.angular_velocity(Axis.X / Axis.Y)` | 0.0 |
| `imu.angular_velocity()` | `(0.0, 0.0, z)` — X, Y and Z like a real hub, with Z as above |
| `imu.acceleration(axis)` | Z: 9810.0 (gravity, mm/s²), X and Y: 0.0, no axis: `(0.0, 0.0, 9810.0)` |
| `imu.heading()`, `imu.reset_heading(angle=0)` | degrees turned since the hub was created or reset, clockwise (right turn) positive, as in the Pybricks documentation |
| `imu.ready()`, `imu.stationary()` | `True`; yaw rate below 1 deg/s |

### `pybricks.pupdevices`

`Motor` and `ColorSensor`. `ColorDistanceSensor`, `UltrasonicSensor`,
`ForceSensor` and the rest do not exist. EV3 Color Sensors on a SPIKE hub are
read through `iodevices.PUPDevice`.

| `ColorSensor` | In the simulator |
|---|---|
| `ColorSensor(port)` | the simulated light sensor on that port; `port` must be in `sensor_ports` (else `OSError`). Same sensor model as the EV3 `ColorSensor`, brightness range from the profile's `sensor_white` / `sensor_black` |
| `reflection()` | simulated reflection, `int` |
| `color(surface=True)`, `hsv(surface=True)` | `Color.BLACK` if the reflection is below the middle of that sensor's white and black, else `Color.WHITE` |
| `ambient()` | 0 (one read is still charged) |
| `detectable_colors(...)` | `(Color.BLACK, Color.WHITE)` |
| `lights.on(...)`, `lights.off()` | no-op |

| `Motor` | In the simulator |
|---|---|
| `Motor(port, positive_direction=Direction.CLOCKWISE, gears=None, reset_angle=True, profile=None)` | `port` must be in the profile's motor map (else `ValueError`). `gears` works as on a real hub (`[20, 12]` or a list of trains): `angle()`, `speed()` and `run()` are then in output-shaft degrees. The physical gearing between motor and wheel is the profile's `gear_ratio`. `reset_angle` and `profile` are ignored; the angle starts at 0 |
| `dc(duty)` | duty in % × direction sign = physical duty, clamped to ±100. The main way to drive |
| `run(speed)` | open loop: picks the duty that gives `speed` (deg/s) at steady state from `dc_table` — no speed regulation, no load |
| `stop()`, `brake()`, `hold()` | duty 0. Unless the profile sets `coast_tau_s` / `brake_tau_s` (no built-in profile does) all three slow down like `dc(0)`; `hold()` does not hold a position. No arguments: `stop(Stop.BRAKE)` raises `TypeError` |
| `angle()`, `speed()` | `int`, motor shaft (output shaft with `gears`): degrees since creation or `reset_angle()`, and deg/s (lagged by `speed_est_tau_s`) |
| `reset_angle(angle=0)` | sets the current angle |
| `control.limits()`, `pid()`, `target_tolerances()`, `stall_tolerances()` | accepted, return zeros, no effect |
| `run_angle`, `run_target`, `run_time`, `run_until_stalled`, `track_target`, `done`, `stalled` | raise `NotImplementedError` |

### `pybricks.ev3devices`

| Class / method | In the simulator |
|---|---|
| `Motor` | a subclass of `pupdevices.Motor` that behaves the same |
| `ColorSensor(port)` | `port` must be in `sensor_ports` (else `OSError`) |
| `ColorSensor.reflection()` | simulated reflection, `int` 0–100 |
| `ColorSensor.color()` | `Color.BLACK` if the reflection is below the middle of that sensor's `sensor_white` / `sensor_black`, else `Color.WHITE` — no other colours |
| `ColorSensor.ambient()`, `rgb()` | 0 (one read is still charged); `(r, r, r)` with r = reflection |
| `GyroSensor(port, positive_direction=Direction.CLOCKWISE)` | only on the port given in the profile's `gyro` (else `OSError`) |
| `GyroSensor.speed()`, `angle()`, `reset_angle(angle=0)` | `int` deg/s and degrees from the gyro model. With `mount` 1, clockwise is positive by default and counter-clockwise with `Direction.COUNTERCLOCKWISE` |
| `TouchSensor(port).pressed()` | `False` (any port) |
| `UltrasonicSensor(port)` | `distance()` → 2550, `presence()` → `False` (any port) |
| `InfraredSensor(port)` | `distance()` → 100, `buttons()` and `keypad()` → empty set (any port) |

### `pybricks.iodevices`

| Class / method | In the simulator |
|---|---|
| `PUPDevice(port)` | an EV3 Color Sensor on a SPIKE hub. `port` must be in `sensor_ports` (else `ValueError`) |
| `PUPDevice.read(mode=0)` | `(reflection,)`, a 1-tuple. **`mode` is ignored**: only reflected light (EV3 mode 0) is simulated, so raw mode 3 also returns % values |
| `PUPDevice.info()` | `{"id": 29}` (EV3 Color Sensor) |
| `LUMPDevice` | the same class (its name on EV3, Pybricks v4) |

There is no `write()`, and no `UARTDevice`, `I2CDevice` or other class.

### `pybricks.parameters`

| Name | Members |
|---|---|
| `Port` | `A` … `F`, `S1` … `S4` (objects with `.name`) |
| `Direction` | `CLOCKWISE` = 1, `COUNTERCLOCKWISE` = −1 |
| `Axis` | `X`, `Y`, `Z` |
| `Stop` | `COAST`, `BRAKE`, `HOLD` (names only) |
| `Button` | `LEFT`, `RIGHT`, `CENTER`, `BLUETOOTH`, `UP`, `DOWN` |
| `Color` | `NONE`, `BLACK`, `WHITE`, `GRAY`, `RED`, `GREEN`, `BLUE`, `YELLOW`, `ORANGE`, `BROWN`, `CYAN`, `MAGENTA`, `VIOLET` — names only, `Color(h, s, v)` raises `TypeError` |

`Icon` and `Side` do not exist.

### `pybricks.tools`

| Name | In the simulator |
|---|---|
| `wait(ms)` | advances simulated time by `ms` |
| `StopWatch()` | `time()` = simulated ms (`int`), `reset()`, `pause()`, `resume()` |
| `DataLog(*headers)` | `log(*values)` appends to the list `.rows`; no file is written |

`multitask`, `run_task` (async programs), `Matrix`, `vector` and `hub_menu` do not exist.

### `pybricks.robotics`

`DriveBase(left_motor, right_motor, wheel_diameter, axle_track)` drives each
wheel open loop with `Motor.run()` (a real hub regulates the speed in closed
loop). `turn_rate` and `angle` are clockwise-positive, as on a real hub.

| `DriveBase` | In the simulator |
|---|---|
| `drive(speed, turn_rate)` | wheel speeds from mm/s and deg/s, using the `wheel_diameter` and `axle_track` you pass |
| `straight(distance)`, `turn(angle)` | drive at `settings()` speed until the encoders (or the gyro) say done, then brake. Blocking; `then` and `wait` are ignored |
| `stop()`, `brake()` | coast / brake both motors |
| `distance()`, `angle()`, `state()`, `reset(distance=0, angle=0)` | from the wheel encoders. Declare the motors' `gears` if the wheels are geared, or distances are off by the gear ratio |
| `use_gyro(True)` | `angle()` and `turn()` use the robot's true heading (encoder angles ignore the turning slip of profiles with `w_eff` > 1) |
| `settings(...)` | get or set straight speed (default 200 mm/s) and turn rate (default 100 deg/s); accelerations are stored but not used |
| `done()`, `stalled()` | `True`, `False` |
| `curve()`, `arc()` | raise `NotImplementedError` |

`GyroDriveBase` and `Car` do not exist.

### `pybricks.media`

`pybricks.media.ev3dev` exists for Pybricks v2 (ev3dev) programs: `SoundFile`,
`ImageFile`, `Font` and `Image` import only when the profile's `pybricks_api`
is `"v2"`. Their attributes are plain strings (`SoundFile.HELLO` → `"HELLO"`)
and they cannot be called. With any other `pybricks_api` (every built-in
profile uses `"v4"`) the import raises `ImportError`, as on a real Pybricks v4
EV3.

### CPython, not MicroPython

Programs run on CPython. Small stand-ins in `sim/` make common MicroPython
imports work: `micropython` (`const()`, `native`/`viper` decorators that do
nothing, `mem_info()` …), `umath` (= `math`) and `urandom` (= `random`).
Other MicroPython-only modules such as `ustruct` and `utime` raise
`ImportError`.

---

## 3. Ports and simulated hardware

### Motors

- The profile key `motors` maps a port to a side and a mount sign:
  `{"A": ["L", 1], "C": ["R", -1]}`. If `motors` is `null` the map is
  `{left_motor_port: ["L", left_mount_sign], right_motor_port: ["R", right_mount_sign]}`.
  Each side needs at least one motor.
- Physical duty = the value given to `dc()` × direction sign (−1 for
  `Direction.COUNTERCLOCKWISE`). Physical duty × mount sign = forward duty.
  So positive `dc()` drives forward when the program uses
  `Motor(port)` for mount sign +1 and `Motor(port, Direction.COUNTERCLOCKWISE)`
  for mount sign −1.
- A side moves with the **average** forward duty of all its motors. Motors the
  program never creates or commands count as 0: driving only `A` and `C` on
  `ev3_3s_geo` (A + B left, C + D right) at `dc(60)` gives 342 deg/s at the
  wheels instead of 924.
- `angle()` and `speed()` report the motor shaft: wheel angle ÷ `gear_ratio` ×
  mount sign × direction sign, so they are positive when driving forward.

### Colour sensors

- `sensor_ports` lists the sensor ports. Its order is the sensor index used by
  `sensor_white`, `sensor_black` and `sensor_pos_mm` — it is not the port
  order (`ev3_3s_geo` uses `["S1", "S3", "S2"]` = left, right, middle).
- `sensor_pos_mm[i] = [forward_mm, left_mm]` from the middle of the wheel
  axle (+ = left). Without it every sensor is `sensor_forward_mm` ahead,
  `sensor_spacing_mm` apart, index 0 leftmost.
- A reading averages the track under a disc of diameter
  `sensor_footprint_mm` (16 sample points). With blackness b (0 = white,
  1 = black): `int(clip(white − (white − black) × b + noise, 0, 100))`,
  noise ~ N(0, `sensor_noise`).
- New values come every 1 / `sensor_hz` s (all sensors together, or each on
  its own clock with `sensor_async`) and become visible `sensor_lag_ms` later.
  With a lag, every read returns 50 until the first value is visible.

### Gyro

- `PrimeHub().imu` is always there and reports the true yaw rate.
- An EV3 `GyroSensor` exists only on the port named in the profile's `gyro`
  ([section 4](#gyro-object)).

### Built-in profiles

Mount +1 → `Motor(port)`; mount −1 → `Motor(port, Direction.COUNTERCLOCKWISE)`.
Sensors are listed in `sensor_ports` order as `port (forward, left)` in mm.

| Profile | Motors | Colour sensors | Gyro |
|---|---|---|---|
| `spike_4s` | E: L +1, F: R −1 | A (170, 37.5), C (90, 12.5), B (90, −12.5), D (170, −37.5) | – |
| `spike_3s`, `spike_3s_real` | E: L +1, F: R −1 | C (90, 25), B (90, −25), A (90, 0) | – |
| `spike_3s_geo`, `spike_3s_geo2`, `spike_3s_geo2x` | E: L +1, F: R −1 | C (170, 23), B (170, −23), A (170, 0) | – |
| `spike_3s1_geo`, `spike_3s1_geo2` | E: L +1, F: R −1 | as `spike_3s_geo` + D (170, 0) | – |
| `spike_3s1f_geo`, `spike_3s1f_geo2` | E: L +1, F: R −1 | as `spike_3s_geo` + D (195, 0) | – |
| `spike_4row_geo`, `spike_4row_geo2` | E: L +1, F: R −1 | D (170, 34.5), C (170, 11.5), B (170, −11.5), A (170, −34.5) | – |
| `ev3_3s` | A: L +1, B: L −1, C: R −1, D: R +1 | S1 (90, 25), S2 (90, −25), S3 (90, 0) | S4 |
| `ev3_3s_geo` | A: L +1, B: L +1, C: R −1, D: R −1 | S1 (170, 23), S3 (170, −23), S2 (170, 0) | – |

---

## 4. Robot profiles

### Loading and inheritance

- `--profile NAME` reads `sim/profiles/NAME.json`; `spike_4s` is an alias of
  `sim/robot_params.json`.
- The values start from `DEFAULT_PARAMS` in `sim/core.py`. A profile with
  `"extends": "PARENT"` loads the parent first (recursively, up to 8 parents
  deep) and its own keys replace the parent's. A missing file raises
  `FileNotFoundError`.
- The merge is shallow: an object value (`motors`, `gyro`, `call_cost_us`)
  replaces the parent's object as a whole, so repeat every field you need.
- Keys starting with `_` (`_note`, `_provisional`, …) are comments; unknown
  keys are ignored.

### Keys

Defaults are those of `DEFAULT_PARAMS`, or the fallback in the code for keys
that are not in it.

**Geometry**

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `wheel_diameter_mm` | mm | 56.0 | wheel diameter |
| `axle_track_mm` | mm | 112.0 | distance between the two wheels |
| `w_eff_scale` | × | 1.0 | multiplies `axle_track_mm` for turning (> 1: turns slower, e.g. skid slip) |
| `sensor_pos_mm` | [[mm, mm], …] | – | per sensor `[forward, left]` from the axle middle; one entry per sensor |
| `sensor_forward_mm` | mm | 60.0 | sensor distance ahead of the axle, used without `sensor_pos_mm` |
| `sensor_spacing_mm` | mm | 32.0 | spacing of neighbouring sensors, used without `sensor_pos_mm` |
| `sensor_footprint_mm` | mm | 14.0 | diameter of the spot a sensor sees |
| `line_width_mm` | mm | 19.0 | width the track line is drawn with |
| `track_ref_mm` | mm | 0 | tracked point ahead of the axle for progress, `lost` and `mean_lat_mm`; > 0 also narrows the forward search for the nearest line point from 300 to 120 mm |

**Ports**

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `motors` | {port: [side, sign]} | `null` | motor map ([section 3](#motors)) |
| `left_motor_port`, `right_motor_port` | port | `"B"`, `"F"` | used only when `motors` is `null` |
| `left_mount_sign`, `right_mount_sign` | ±1 | −1, +1 | used only when `motors` is `null` |
| `sensor_ports` | [port, …] | `["A", "C", "E", "D"]` | colour sensor ports; the order is the sensor index |
| `gyro` | object | `null` | EV3 gyro sensor model, see below |

**Motors and drive**

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `dc_table` | [[duty %, deg/s], …] | `[[0,0],[8,0],[20,190],[40,450],[60,680],[80,870],[100,1010]]` | steady-state motor-shaft speed for \|duty\|, linear between points (dead zone = rows with 0) |
| `gear_ratio` | × | 1.0 | wheel speed = motor speed × `gear_ratio` |
| `battery_scale` | × | 1.0 | multiplies every target speed and `battery.voltage()` |
| `motor_tau_s` | s | 0.15 | time constant while speeding up |
| `motor_tau_dn_s` | s | = `motor_tau_s` | time constant while slowing down |
| `yaw_tau_s` | s | 0 (off) | separate time constant for the turning part (difference of the wheel speeds) |
| `brake_tau_s`, `coast_tau_s` | s | `null` | slow-down time constant when every motor of a side is in `brake()`/`hold()`, or `stop()`; `null` = like `dc(0)` |
| `cmd_lag_ms` | ms | 0 | motor commands take effect this much later |
| `speed_est_tau_s` | s | 0.0 | lag of `Motor.speed()` |
| `mu` | – | `null` | grip: limits \|v·ω\| to `mu` × 9810 mm/s² (under-steer); `null` = no slip |
| `turn_load` | – | 0 | both wheels slow by `turn_load` × min(1, \|vR − vL\| / (2 × top speed)) while turning |
| `wheel_wobble` | fraction | 0 | wheel radius wobble per revolution (each wheel × 0.5–1.5, random phase) |
| `wobble_seed` | int | 0 | random seed of `wheel_wobble` |

**Sensors**

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `sensor_white`, `sensor_black` | reflection % | `[64, 52, 53, 61]`, `[9, 7, 6, 8]` | reading on the white floor and on the black line, one per sensor in `sensor_ports` order |
| `sensor_noise` | % (σ) | 1.0 | Gaussian noise added to each reading |
| `sensor_hz` | Hz | 1331.0 | how often sensor values update (and how often `lost` / `finish` are checked) |
| `sensor_async` | bool | `false` | each sensor updates on its own clock: `sensor_hz` × (1 ± `sensor_hz_jitter`), random phase |
| `sensor_hz_jitter` | fraction | 0.02 | used with `sensor_async` |
| `sensor_lag_ms` | ms | 0 | delay until a new value is visible to the program |
| `imu_lag_ms` | ms | 0 | delay of `hub.imu` |

**Timing and other**

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `loop_hz` | Hz | 5211.0 | measured loop rate when every sensor is read once per loop; sets the default colour-read cost 1 / (`loop_hz` × sensors) and the step of `wait()` |
| `call_cost_us` | {name: µs} | `null` | simulated time per call: `color_read`, `gyro_read`, `motor_dc`, `motor_read` ([section 1](#simulated-time)); a missing `color_read` uses the default above, the others 0 |
| `battery_mv` | mV | 8000 | `battery.voltage()` before `battery_scale` |
| `pybricks_api` | `"v2"` / `"v3"` / `"v4"` | `"v4"` | only `"v2"` makes `pybricks.media.ev3dev` importable |
| `platform` | text | `"spike"` | label only; the simulator does not read it |

<a id="gyro-object"></a>**`gyro` object** — `{"source": "sensor", "port": "S4", ...}`

| Field | Default | Meaning |
|---|---|---|
| `source` | – | `"sensor"` enables the gyro; anything else means no gyro |
| `port` | – | port for `GyroSensor(port)` |
| `mount` | 1 | ±1, multiplies rate and angle (1 = red dot up) |
| `update_hz` | 1000.0 | sample rate |
| `lag_ms` | 0 | delay until a sample is visible |
| `noise_dps`, `bias_dps` | 0, 0 | rate noise (σ) and constant bias, deg/s |
| `max_dps` | `null` | clamp of the reported rate |

The rate is reported as an `int` with noise and bias; the angle integrates the
true rate plus bias.

### Built-in profiles

| Profile | Extends | Robot |
|---|---|---|
| `spike_4s` | – | `sim/robot_params.json`: SPIKE hub with 4 EV3 colour sensors (two 90 mm, two 170 mm ahead), 56 mm wheels, 72 mm track, gear 2.0. Root of the SPIKE chain |
| `spike_3s` | `spike_4s` | 3 sensors in a row 90 mm ahead |
| `spike_3s_real` | `spike_3s` | values measured on the real robot: motor curve, 20T→12T gear (1.6667), sensor values, independent sensor clocks, 5 ms sensor lag, 32 ms command lag, call costs for ≈734 loops/s |
| `spike_3s_geo` | `spike_3s_real` | measured frame: sensors 170 mm ahead and 23 mm apart, 80 mm wheels, 205 mm track, 2 % wheel wobble, judged at the sensors |
| `spike_3s_geo2` | `spike_3s_geo` | turning fitted to real black-box logs (`yaw_tau_s` 0.05, `w_eff_scale` 1.5). Default SPIKE profile (1× gear) |
| `spike_3s_geo2x` | `spike_3s_geo2` | 2× gear (`gear_ratio` 3.3333) with motor and turn time constants × 3. Default for the 2× gear |
| `spike_3s1_geo`, `spike_3s1f_geo`, `spike_4row_geo` (+ `_geo2`) | `spike_3s_geo` (`_geo2`: + turn fit) | four-sensor experiments: extra sensor D in the middle / 25 mm further ahead / four in a row |
| `ev3_3s` | `spike_4s` | EV3 brick, 3 colour sensors 90 mm ahead, gyro sensor on S4, two motors per side; many values provisional (`_provisional`) |
| `ev3_3s_geo` | `spike_3s_geo` | the EV3 robot of this repository: same frame as `spike_3s_geo`, two motors per side, no gyro; call costs provisional |

### A minimal custom profile

An EV3 robot like `ev3_3s_geo` but with one motor per side and its own
light values. Save it as `sim/profiles/my_ev3.json` and run with
`--profile my_ev3`:

```json
{
  "_note": "My EV3 robot: like ev3_3s_geo, but one motor per side and my own light values",
  "extends": "ev3_3s_geo",
  "motors": {"B": ["L", 1], "C": ["R", -1]},
  "call_cost_us": {"color_read": 47.152, "motor_dc": 560, "motor_read": 30},
  "sensor_white": [30, 28, 29],
  "sensor_black": [4, 3, 3]
}
```

- `motors` replaces the parent's four-motor map. The program now creates only
  `Motor(Port.B)` and `Motor(Port.C, Direction.COUNTERCLOCKWISE)`;
  `Motor(Port.A)` would raise `ValueError`.
- `call_cost_us` repeats all of the parent's fields because objects are
  replaced as a whole. `motor_dc` is doubled so that two `dc()` calls per loop
  cost what the parent's four did.
- `sensor_white` / `sensor_black` follow the parent's `sensor_ports`,
  `["S1", "S3", "S2"]`: left, right, middle.

---

## 5. Tracks

| Track | Length | Shape |
|---|---|---|
| `easy` | 2.03 m | straight, gentle left and right 90° curves (radius 200) — quick check |
| `main` | 2.17 m | straight, two left 90° corners (r 150), S-chicane (r 120), left 90° corner (r 130) — basic tuning course |
| `hard` | 1.50 m | four 90° curves of radius 80 mm — tight curves, recovery |
| `rightangle` | 3.40 m | 1 m straight, then three sharp 90° corners (left, right, right) 800 mm apart |
| `mrc` | 14.85 m | competition-style lap from a photo of the real field, scaled 1.6×: sharp corners, a teardrop loop, an acute zigzag, an S-wave, loops and a figure-8 that cross the line at right angles; on the first straight a broken marker line beside the line (0.72 m) and a solid one across it (1.52 m) |
| `mrc_real` | 9.28 m | `mrc` at real size (÷ 1.6), so its curves are tighter |
| `photo` | 6.87 m | the line traced from a photo of the real 195 cm mat, with its markers (`sim/viz/track_pts.json`) |
| `photo@SCALE` | 6.87 m × SCALE | `photo` enlarged, line width unchanged, e.g. `photo@1.3`. The tightest traced loop (radius ≈ 76 mm) is below the rules' 100 mm minimum, so the real field may be larger |

### Track format

`core.TRACKS[name]` is a list of segments, or
`{"segs": [...], "extras": [polyline, ...]}`:

| Segment | Meaning |
|---|---|
| `('S', length_mm)` | straight |
| `('A', radius_mm, angle_deg)` | arc; positive angle = left (counter-clockwise) |
| `('T', angle_deg)` | turn on the spot → a sharp corner; positive = left |

The path starts at (0, 0) heading +x and is sampled every 2 mm; the robot
starts on its first point, facing along it (shifted by `--offset`).
`extras` are marker polylines `[(x_mm, y_mm), ...]` drawn with the same width:
the sensors see them, but progress and `lost` use only the path. The track is
rasterised at 1 mm per pixel with a 150 mm margin, drawn with the profile's
`line_width_mm`. A path may end where it started (a closed loop).

### Adding a track

Add an entry to `TRACKS` in `sim/core.py`; it then shows up in `--list` and
`--track`:

```python
TRACKS["oval"] = [('S', 800), ('A', 300, 180), ('S', 800), ('A', 300, 180)]
```

Without editing `core.py`, register it from a small launcher next to `sim/`
and reuse the normal command line (the track check runs inside `main()`):

```python
# my_tracks.py, in the repository root:
#   python my_tracks.py examples/spike_simple_follower.py --profile spike_3s_geo2 --track oval --no-plot
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim"))
import core
import run_sim

core.TRACKS["oval"] = [('S', 800), ('A', 300, 180), ('S', 800), ('A', 300, 180)]
run_sim.main()
```

For a track that is not made of segments, add a loader to
`core.TRACK_LOADERS[name]`: a function `(line_width_mm) -> core.Track(...)`.
`core.Track(path, line_width_mm, extras=None, stamp_pts=None, stamp_path=True)`
takes the centre line `path` (N × 2 points in mm, used for progress and
`lost`), marker polylines `extras`, and `stamp_pts`, a point cloud drawn as
line; `stamp_path=False` uses the path for judging only and draws just the
point cloud. This is how `photo` is built.

### Photo track (`sim/photo_track.py`)

`photo_track.py` is the one-off script that turned the field photo into the
`photo` track (`python sim/photo_track.py WARP.jpg`; needs OpenCV,
scikit-image, SciPy and matplotlib):

1. Reads a top-down, perspective-corrected photo of the field (the argument,
   default `/tmp/warp_1.8.jpg`; making that image is not part of the script)
   and sets the scale from `MAT_W_MM`, the mat width across the image (1950).
2. Dark pixels (gray < 100) are line. The largest connected part is the
   driving line; the other parts are kept as markers.
3. Walks the line's skeleton, preferring to go straight, so crossings are
   passed through. The start is found on the widest horizontal band (the
   marker straight of that field) and the direction is set to leave toward −x.
4. Converts to mm (rotated 180°, start at the origin) and writes
   `sim/viz/track_pts.json`, `sim/viz/track_bg.json` (the photo as a
   background image) and `sim/track_preview.png`.

`track_pts.json` holds `"black"`, a flat list `[x0, y0, x1, y1, ...]` in mm of
every black point (the line every 2 mm plus the markers), and `"coarse"`,
`{"px": [...], "py": [...], "s": [...], "length": mm}`, the centre line every
10 mm. The `photo` loader draws every black point and uses the coarse line
(resampled to 2 mm) for progress; `photo@SCALE` multiplies both. For another
field, adapt `MAT_W_MM` and the start search, and keep a copy of the original
`track_pts.json`: the script overwrites the `photo` track.
