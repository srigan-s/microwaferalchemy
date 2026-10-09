# PID refactor audit (0.3.5)

The repository has no ROS controller. Before this change,
`sensing/follower.py` directly combined an oriented edge estimate and
pitch-based P/D correction at the follow-loop frequency. `sensing/edge.py`
already normalized active-low IR bits and classified edges; it remains the
source of normalized detections and acquisition evidence. `Robot` already
provided the motor mapping, five-count profile cap, stop latch, watchdog and
fault behavior; those controls remain in the physical command path.

New ownership is: `sensing/poller.py` for the one latest-sample physical IR
reader; `sensing/position.py` for interval position and timestamped velocity;
`sensing/pid.py` for the only active PID law; `sensing/follower.py` for control
scheduling, existing mixer, acquisition/loss and motor commands; and
`sensing/control_log.py` for per-update diagnostics. `navconfig.py` holds
dimensional gains and separate 300/100 Hz rates. The old follower `_correction`
path is removed. Legacy config gain fields remain as fallback inputs for old
files, but current profiles only use dimensional fields.

The supplied diagram's left/right mixer sign is opposite the calibrated
project convention. Changing it literally would reverse yaw on the tested
synthetic plant, so the existing mapping is retained and documented. The
project's four mecanum wheels also support lateral and blended modes, which
are separate from diagram differential steering.

No experimental IR CSV was present in the working tree, so no measured trace
or displacement was fabricated. `scripts/replay_ir_pid.py` accepts a real
recording and preserves its timestamps. `scripts/compare_pid.py` is an
idealized, quantized offline comparison, not a physical fit. The synthetic
sharp-corner case currently stops with `LINE_LOST`; physical 90° following
remains unverified. Actual Pi polling/control rates and motor breakaway at
five PWM must be measured from the new control CSV.

Physical following and physical route execution refuse a shipped nav profile
until an operator explicitly enables `follow.controller_enabled` in a copy.
The 0.3.5 bundle is built separately from the prior 0.3.4 bundle and has not
been installed on the Pi during this refactor.
