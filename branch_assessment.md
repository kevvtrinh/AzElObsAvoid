# Plan 325 branch assessment

## Evidence scope

This assessment applies to source commit `b238e6e` on branch `plan-325`.

- All 18 maintained examples ran in separate MATLAB processes.
- Seventeen examples returned validated success.
- The no-path example returned the expected validated failure.
- All 13 fixed-goal examples use earliest arrival.
- Five moving-target examples keep their target-time policy.
- The visible success and visible failure plot checks passed.
- All 51 tests passed.
- Code Analyzer checked 52 MATLAB files and returned 0 messages.

The implementation has 26 production MATLAB files and 7,000 physical
production lines. The complete MATLAB tree has 52 files and 11,621 physical
lines. Both hard limits pass. The preferred 10,500-line complete-tree target
does not pass.

## Current judgment

Plan 325 remains the best compact rebuild candidate in this repository. It
uses one public planner, a finite deterministic proposal set, a certified
finite-jerk first motion, bounded local HS3 work, and one independent
validator. It does not combine complete planner stacks from other branches.

The latest example policy is better. Fixed-goal examples now measure earliest
validated arrival. Moving-target examples can still state an exact intercept
time when that time is part of the request.

## Main strengths

### 1. Physical checks remain authoritative

Every successful headless run passed collision, kinematic, and applicable
certificate checks. The validator checks polynomial consistency, endpoint
events, state history, continuity, physical limits, safety provenance, and
moving collision intervals.

The no-path case returned a stable failure result and usable search
diagnostics. A failed finite proposal set is not reported as a proof that no
physical path exists.

### 2. The slalom now tests the intended topology

The slalom elevation interval is `[-5 5]` degrees. The protected upper
barriers reach the top bound, and the protected lower barrier reaches the
bottom bound. A route cannot pass above or below the complete obstacle set.
The returned route must alternate around the barriers.

The final slalom motion passed independent validation. It had a
16.0425349764-degree seed, a 16.7453475476-degree returned motion, and a
12.1834571132-second duration.

### 3. Earliest-arrival results are now comparable by meaning

The fixed-goal examples no longer report forced horizon durations. Examples
that changed from fixed to earliest arrival returned these durations:

| Example | Previous fixed duration (s) | Earliest duration (s) |
| --- | ---: | ---: |
| Alternating slalom | 22 | 12.1834571132 |
| Basic azimuth/elevation planning | 12 | 7.81726894407 |
| Dense concave obstacle | 15 | 8.7986387782 |
| Moving barrier | 12 | 10.5465620376 |
| Moving circle | 15 | 12.0310423352 |
| Obstacle free | 8 | 5 |

These are single runs. They are not repeated performance studies.

### 4. Dense cases still return useful motions

- The 40-circle case returned a 64.5710759977-second motion in
  31.6790719 seconds.
- The native moving-U.S. case returned a 25.614496552-second motion in
  167.6347756 seconds.
- The two-opposing-U case returned a 22.875124576-second motion in
  64.9545585 seconds.
- The large single-U case returned a 26.4922113988-second motion in
  50.8781644 seconds after its work limit increased to 75 seconds.

### 5. Plot output again has a stable visual language

The plotter uses the `main` branch route and obstacle style without restoring
the old large plotting implementation. The current plotter keeps the Plan 325
result schema and diagnostics. Programmatic checks verified the colors, line
styles, visible success figures, and visible failure figures.

## Main weaknesses

### 1. Proposal coverage remains incomplete

Spatial and time-layer searches use finite samples. Dense-envelope and cluster
reductions can remove a useful topology. Final validation prevents false
success, but it cannot make the proposal search complete.

### 2. Runtime is still high and variable

The native moving-U.S. case took 167.6347756 seconds. The three-region extreme
U.S. sequence took 488.9379664 seconds. The large-U example failed at its old
35-second work limit because validation did not finish. It passed with a
75-second work limit.

Deadline checks are cooperative. One active solver or geometry operation is
not preempted. The result can return after a configured check deadline.

### 3. HS3 has numerical conditioning warnings

`exampleAzElPlanning`, `exampleMovingBarrierWait`, and the extreme U.S.
sequence emitted fmincon conditioning warnings. Their returned motions passed
independent validation, but the warnings show that solver scaling still needs
work.

### 4. The first motion is conservative

The analytic motion stops at each geometric waypoint. This gives a clear
finite-jerk certificate, but it can be slower than a through-velocity motion.
Nonzero endpoint derivatives and earliest moving-target intercepts require
HS3.

### 5. The preferred size target still fails

The production code meets the 7,000-line hard limit exactly. The complete tree
is 1,121 lines above the 10,500-line target. There is no remaining production
size margin.

## Recommended next work

1. Keep the current public interface and independent validator.
2. Improve nonlinear scaling before adding a new solver stage.
3. Cache prepared obstacle history data for dense timed queries.
4. Add one general through-velocity finite-jerk primitive family.
5. Reduce production code before any feature adds more lines.
6. Keep moving-target time policy explicit in each target example.

## Final claim

Plan 325 is a compact, useful planner candidate. It supports static, moving,
and deforming obstacles; fixed and earliest target interception; timed waits;
and stable failure diagnostics.

It is not complete or globally optimal. Its dense cases can be slow. The next
changes should improve general numerical scaling and motion quality. They
should not add scenario-specific routes or another public planner.
