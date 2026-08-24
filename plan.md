# Lean HS3 replacement

## Objective and hard gates

Reduce HS3-specific production ownership from 3,868 to at most 1,200
nonblank, noncomment MATLAB lines. The final method must meet or beat committed
compact planner `2fbe13840411` on every maintained example and the
1/5/10/20-turn plus 12-hairpin benchmarks, with identical canonical inputs and
independent validation. Preserve nonzero endpoint derivatives, moving targets,
timed waits, expected no-path diagnostics, and general input-driven behavior.

## Frozen compact baseline

- Maintained matrix: 17 validated successes plus the validated expected
  no-path result; durations are recorded in `benchmark.csv` and
  `verification.md` under `8111d0f+compact-cutover-worktree`.
- Scaling durations: 6.60420575985, 19.8905274829, 36.3238796555, and
  68.3588042743 seconds for 1/5/10/20 turns; 140.56091613 seconds for 12
  hairpins. All independently validate.
- Compact obstacle-path ownership: 1,023 noncomment lines.
- Complete suite at the compact checkpoint: 133/133.

## Measured starting structure

- HS3-owned package: 3,868 noncomment lines; full executed closure including
  shared/public infrastructure: 5,101.
- Motion kernel: 902 lines (`solveHs3` 821, jerk objective 38, affine
  linearizer 17, diagnostics 26).
- At least 1,195 normalized lines duplicate corridor/shared responsibilities in
  topology, validation, result schema, intercept handling, and clustering.
- Current earliest-arrival HS3 is a local `fmincon` solve. Fixed-duration
  dynamics and endpoint constraints are affine in the jerk variables, making a
  direct QP the first bounded experiment.

## Completed 30-minute experiment: direct fixed-time QP

Primary question: could one fixed-duration HS3 QP reproduce an independently
valid current HS3 candidate with lower solver work, without changing route,
obstacles, limits, endpoint state, safety margin, or validator?

The isolated 8-second wait seed improved from 16 to 4 solver iterations and
from 2.084 to 1.886 seconds cold. A nonzero-endpoint case improved from 1.953
to 0.098 seconds after warm-up and retained endpoint errors below `2.35e-14`.
The four-accelerating maintained case regressed from 24.150 to 31.177 seconds
because the unscaled QP declared frozen dynamic corridors infeasible and
triggered additional recovery. The experiment failed its representative gate,
all QP-only code was removed, and the focused recovery test passed.

## Completed HS3 replacement

HS3 now composes the immutable compact planner with a default-off bounded HS3
improvement. An attempted HS3 candidate is independently validated and can be
selected only when it is no later, has no greater integrated squared jerk, and
strictly improves at least one of those measures. A failed compact baseline may
recover to any independently valid HS3 candidate. Rejected attempts, elapsed
work, solver diagnostics, unsupported refinement requests, and cooperative
time-limit overruns remain visible in composition diagnostics.

HS3-owned production is exactly 1,200 nonblank, noncomment MATLAB lines. This
is an ownership count, not a whole-execution-closure count: the default path
intentionally calls the shared/public compact baseline. Shared topology,
corridor construction, validation, result schema, moving-target policy,
geometry, clustering, and comparison invariants have one neutral owner.

Final gates on 2026-08-24:

- Code Analyzer: 94 files, zero findings.
- Complete test suite: 138/138 in 205.211 seconds.
- Maintained examples: both modes passed all 18 example contracts; each of the
  17 successes had exact physical trajectory and arrival parity, while the
  expected `noValidatedSeed` outcome remained diagnosable in both modes.
- Interleaved 1/5/10/20-turn comparison: exact trajectory parity; HS3/compact
  median runtime ratios 0.848, 0.991, 1.003, and 0.989.
- Twelve-hairpin comparison: exact trajectory parity and independent
  validation; median ratio 1.020 and HS3 observed maximum below compact.
- Nonzero endpoint velocity and acceleration, moving-target derivatives, timed
  waits, compact-failure recovery, optional-improver timing, and direct facade
  behavior have dedicated tests.
- Graphics: four valid success figures and two expected-failure diagnostic
  figures; failure returned zero selected seed without replanning.

The translating-obstacle certificate regression is also retained: a sweep
crossing the gap between disconnected endpoint envelopes is no longer falsely
certified, and adaptive collision checking detects it. Unrelated untracked
`docs/` and `tmp/` remain untouched.
