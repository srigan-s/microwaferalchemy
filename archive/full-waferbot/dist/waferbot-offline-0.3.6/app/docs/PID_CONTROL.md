# Physical edge-following PID (0.3.6)

The follower reads the stock four-channel active-low IR bar through the verified
Yahboom I²C interface. A single latest-sample poller requests readings at 300 Hz;
the controller initially runs at 100 Hz. These are requests on Raspberry Pi
Linux, not hard real-time guarantees. The control CSV reports observed polling
and control rates, read latency, missed deadlines, sample age, computation time,
motor-write time, and feedback-to-actuation latency. A stale or missing sample
cannot be reused as a fresh measurement. The poller has no backlog of old data.

## Geometry and signs

Sensor positions are millimetres relative to the sensor-array centre and
increase to the robot's right (viewed from behind). The default layout A is
`[-31.25, -3.25, 3.25, 31.25]`; layout B for comparison is
`[-31.25, -17.25, 17.25, 31.25]`. These positions are configuration inputs;
verify actual placement before using physical results as distance measurements.

`1` means black. For Black-Left, the selected transition is `1→0`; for
Black-Right it is `0→1`. Each adjacent transition identifies an **interval**
`[s_i,s_(i+1)]`, not a precise edge. For the central S2–S3 interval the
controller uses its midpoint. For an outer interval it uses the **inner sensor
position** as a conservative control representative, while recording both
physical bounds. With layout A, Black-Left `1100` therefore gives robot
position 0 mm and `1000` gives +3.25 mm; the latter still permits the actual
edge anywhere between S1 (−31.25 mm) and S2 (−3.25 mm). Black-Right `0011`
gives 0 mm and `0001` gives −3.25 mm. Robot position is the negative of the
edge-relative control representative. A
missing selected transition, multiple selected transitions, all-black, or
all-white is invalid; it never becomes zero error. An opposite finite-band
transition reduces confidence. The controller does not average black pixels.

A distinct-bin change divided by actual sample time estimates robot lateral
velocity in mm/s. It is bounded at 250 mm/s and exponentially filtered with
time constant `velocity_filter_tau_s` (default 0.05 s). Repeated bins keep the
last filtered estimate; transitions less than 10 ms apart do not imply an
unbounded speed. This is a quantized proxy, not an encoder measurement. Because
the inner gap of layout A is only 6.5 mm while the outer gaps are 28 mm,
the ±3.25 mm outer-bin value must not be interpreted as a precise displacement
measurement. Transitions between adjacent control bins are 3.25 mm in layout A.

## Controller and mixer

For reference `r` (default 0 mm) and estimated robot position `x̂`, the
controller computes `e = r - x̂`, `P = Kp e`, `I[k] = clamp(I[k-1] + Ki e Δt)`,
and `D = -Kd v̂`. `u = P + I + D` is limited to `max_correction` PWM, then
limited by `max_correction_slew_pwm_per_s × Δt`. Gains are PWM/mm,
PWM/(mm·s), and PWM·s/mm respectively. `Ki=0` is the initial physical setting.
Conditional integration and rollback when the motor mixer clips prevent
integral windup. Invalid measurements reset the controller and command stop.
The controller also resets at each new run and after recovery.

The existing calibrated mecanum mapping is retained. For differential steering
the logical wheel tuple `(FL, RL, FR, RR)` uses `(V+u, V+u, V-u, V-u)`;
the hardware driver maps that tuple to physical motor channels and inversions.
Positive `u` is a right-turn command under this mapping. The supplied diagram
shows `(V-u, V-u, V+u, V+u)`, which would reverse the turn on this chassis.
The differential mixer changes yaw, while `lateral` and `blended` modes use
the existing mecanum mapping. Correction demand also slows forward `V` on
curves. Wheel targets are rounded to integer PWM on each update. Physical
`follow` and route-follow actions cap each wheel at ±5 PWM, including during
mixer saturation. All commands go through `Robot` so the
shared stop latch, motor communication fault, obstacle monitor, and watchdog
remain authoritative.

## Physical activation and logs

The four shipped nav profiles have `follow.controller_enabled: false`. Copy a
profile, verify the actual sensor geometry, bit order, motor inversion and
five-count motor cap, then set that field to `true` only in the operator copy.
Keep recovery disabled while tuning. The [robot runbook](ROBOT_TESTING.md)
contains the exact install and three-second test commands. Use
`--control-log-csv` for per-update PID and timing fields; `--log-csv` is broader
robot telemetry. A result ending in `LINE_LOST` is a stopped failed run.

To inspect a recorded `ir-strafe` or `edge-angle-sweep` CSV offline, run:

```bash
PYTHONPATH=src python3 scripts/replay_ir_pid.py input.csv replay.csv \
  --edge black-left --nav-config config/nav.windy-first-run.json
```

The replay requires the recorded `read_end_ns` and `black_s1..black_s4` columns,
preserves real timestamps, and issues no motor commands. Its PID output is a
counterfactual diagnostic; it cannot reconstruct cart displacement from binary
IR data alone. The known sequence `1100→1110→0111→0110→0010→0000` is
recognized as interval transitions followed by loss of the selected edge.

For an idealized finite-width tape and yaw-plant comparison, run:

```bash
PYTHONPATH=src python3 scripts/compare_pid.py --layout a --kp 0.7 --ki 0 \
  --kd 0.015 --tau-s 0.05 --poll-hz 300 --control-hz 100 \
  --output-prefix /tmp/waferbot-pid-a
PYTHONPATH=src python3 scripts/compare_pid.py --layout b --control-hz 200 \
  --output-prefix /tmp/waferbot-pid-b
```

This produces CSV and PNG plots of position error, correction, and four motor
commands. It uses the production estimator, PID and mixer, with idealized
kinematic coefficients, binary sensors, finite tape width, saturation and
quantization. It is **not** fitted to real motor friction, slip, lighting or
sensor response. Settling time is reported only if the estimated error remains
within the explicit tolerance (default ±1 mm) through the run's end. Do not infer that a
square 90° tape corner is physically negotiable from these plots. The current
synthetic 80.5° rising kink remains on the selected edge with the windy lateral
profile and stops at its iteration bound.

For tuning, first verify steering direction at a low cap. Increase `Kp`
gradually only while the selected edge stays in view. If the output oscillates
between quantized bins, reduce `Kp` or increase the velocity filter time
constant. Increase `Kd` only after reviewing measured sample timing and
velocity spikes. Leave `Ki=0` until a repeatable steady error under valid
measurements is observed; then use a small `Ki` with a finite
`integral_limit_pwm`. The five-count cap may be below the chassis's breakaway
PWM, which no PID gain can fix.

The prior `kp`, `kd`, `max_correction_delta`, and `derivative_filter_alpha`
fields are retained only to read older navigation configs. Current profiles
use explicit dimensional gain and time-based slew fields. There is one active
PID implementation in `sensing/pid.py`; the former follower PD code has been
retired. No ROS controller exists in this repository.
