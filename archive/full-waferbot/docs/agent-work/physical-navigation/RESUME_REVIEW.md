# September 28 acceptance checkpoint: implementation interrupted

The resumed Flash implementation stopped with HTTP 402 (DeepSeek insufficient
balance), before a final report, packaging, and acceptance. The child was
interrupted after its error notification. No replacement implementation model
has been used. The user was asked whether to authorize direct Astra completion
or restore credits for the required Flash worker.

## Evidence

- Current source: 473 collected pytest tests passed with `PYTHONPATH=src
  /tmp/waferbot-phase1-venv/bin/python -m pytest -q` (exit 0).
- The standalone read-only sensor diagnostic's six unittest tests passed.
- `python3 scripts/test_line_follow.py --edge both --json` completed with both
  synthetic runs reporting convergence. This is not physical validation.
- Existing `dist/waferbot-offline-0.1.0` has 58 matching manifest entries, and its
  standalone sensor diagnostic matches the accepted source.
- Source version is now 0.2.0, but there is no 0.2.0 distribution yet.
- Detailed log: `logs/resume-root-verification.log`.

## Saved work, not finally accepted

The worker added oriented boundary estimation, stationary acquisition, PD
filtering and slew limiting, a command-dependent kinematic tape simulator,
conservative first-run configs, tests, and draft operator instructions. It also
edited watchdog, command cleanup, switching timing, route handling, and wheel
calibration. Compare against `resume-baseline.json` and its saved snapshot.

## Concrete remaining findings

1. `scripts/test_line_follow.py:run_physical` directly calls
   `Robot.connected(..., stop_check=None)`. It bypasses CLI process ownership,
   shared stop latch, and verified-mapping checks. The guide's statement that a
   second-terminal stop prevents further commands is therefore false for this
   entry point. Use one guarded physical session path, add real injected-bus
   coverage for external stop/concurrent motion/config gates, then review.
2. `sensing/edge.py:EdgeDetector.update` still accepts future timestamps up to
   max_age and moves `_last_timestamp` backwards on rejected frames.
   `localization.py` still accepts future timestamps within max_age. The
   explicit RESUME timestamp contract is not complete despite passing tests.
3. `docs/ROBOT_TESTING.md` is a draft, not an executable handoff: it references
   absent 0.2.0 output, runs verification before installation, omits initial apt
   and I2C setup in the running-Pi path, and uses `config/` and `scripts/` from
   the bundle root even though these reside under `app/`. Its `cp` commands
   overwrite existing calibration despite the preservation claim.
4. `scripts/build_offline_bundle.py` does not allowlist ROBOT_TESTING.md. The
   final bundle, no-index installation, CLI outside the repo, complete file
   manifest and updated hardware report still require completion/verification.
5. The last `test_sensor_read_failure_stops_the_closed_loop` in
   `tests/test_tape_model.py` never injects a failure; it asserts the normal
   transport remains open. Replace it with meaningful mid-motion failure and
   final-zero-wheel assertions. Extend noise coverage to both selected edges.
6. Re-review all RESUME safety/localization/switching contracts against the
   actual final patch; no full acceptance was completed before the provider
   failure. Do not equate the passing suite with physical readiness.

No physical tests, SSH commands, deployments, SD-card operations, or automatic
commits were performed. The accepted immediate deliverable remains
`scripts/test_line_edge.py` and `docs/LINE_EDGE_TEST.md`. Existing 0.1.0 bundle
may supply this sensor diagnostic and short raised-wheel diagnostics; do not
present it as the new boundary-follower release.
