# Plan 325 branch assessment

## Evidence scope

This assessment applies to source commit `6ddabac` on branch `plan-325`.

- All 18 maintained examples ran in separate MATLAB processes.
- Seventeen examples returned validated success.
- The no-path example returned the expected validated failure.
- All 13 fixed-goal examples use earliest arrival.
- Five moving-target examples keep their target-time policy.
- The visible success and visible failure plot checks passed.
- All 52 tests passed.
- Code Analyzer checked 52 MATLAB files and returned 0 messages.

The implementation has 26 production MATLAB files and 7,000 physical
production lines. The complete MATLAB tree has 52 files and 11,653 physical
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

The new spatial diversity policy is also better. It keeps graph states with
different bounded 2-D homology signatures separate. It replaces route-side
restrictions and repeated edge removal with an input-driven class invariant.

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

### 3. Spatial route diversity now has a defined invariant

The search places one interior representative in each connected sampled
obstacle region. It augments a visibility node with the integer path integral
around those representatives. It returns the shortest discovered route for
each bounded signature class.

The two-opposing-U case found four classes in 146 augmented states. The
signatures were `[0 0]`, `[0 -1]`, `[1 0]`, and `[0 1]`. The single-U case
found two classes in 19 states. Neither search reached the 4,000-state cap.

This is a 2-D spatial proposal method. It is not a continuous Az/El/time
homotopy certificate. Final acceptance still comes only from the independent
motion validator.

### 4. Earliest-arrival results are now comparable by meaning

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

### 5. Dense cases still return useful motions

- The 40-circle case returned a 64.5557806844-second motion in
  24.0566557 seconds.
- The native moving-U.S. case returned a 25.614496552-second motion in
  140.0016750 seconds.
- The two-opposing-U case returned a 22.875114336-second motion in
  64.9563617 seconds.
- The large single-U case returned a 38.5495931039-second motion in
  68.5958720 seconds.

### 6. Plot output again has a stable visual language

The plotter uses the `main` branch route and obstacle style without restoring
the old large plotting implementation. The current plotter keeps the Plan 325
result schema and diagnostics. Programmatic checks verified the colors, line
styles, visible success figures, and visible failure figures.

## Main weaknesses

### 1. Proposal coverage remains incomplete

Spatial and time-layer searches use finite samples. Dense-envelope and cluster
reductions can remove a useful topology. Final validation prevents false
success, but it cannot make the proposal search complete.

Homology signatures separate routes around sampled spatial regions. They do
not separate all paths through continuous obstacle time. Reduced or clustered
regions can also merge classes before the signature search starts.

### 2. Runtime is still high and variable

The native moving-U.S. case took 140.0016750 seconds. The three-region extreme
U.S. sequence took 464.6851317 seconds. The large-U example took 68.5958720
seconds.

The homology change is not a uniform performance gain. Against source
`b238e6e`, the single-U duration increased from 26.4922113988 seconds to
38.5495931039 seconds, and wall time increased from 50.8781644 seconds to
68.5958720 seconds. The 40-circle wall time decreased from 31.6790719 seconds
to 24.0566557 seconds. These are single runs, not repeated benchmarks.

Deadline checks are cooperative. One active solver or geometry operation is
not preempted. The result can return after a configured check deadline.

### 3. HS3 has numerical conditioning warnings

`exampleAzElPlanning` reproduced the fmincon conditioning warnings. The two
related MATLAB warning identifiers were hidden in later long runs to keep the
logs bounded. All returned motions passed independent validation, but the
known warnings show that solver scaling still needs work.

### 4. The first motion is conservative

The analytic motion stops at each geometric waypoint. This gives a clear
finite-jerk certificate, but it can be slower than a through-velocity motion.
Nonzero endpoint derivatives and earliest moving-target intercepts require
HS3.

### 5. The preferred size target still fails

The production code meets the 7,000-line hard limit exactly. The complete tree
is 1,153 lines above the 10,500-line target. There is no remaining production
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
bounded homology-diverse spatial proposals; and stable failure diagnostics.

It is not complete or globally optimal. Its dense cases can be slow. The next
changes should improve general numerical scaling and motion quality. They
should not add scenario-specific routes or another public planner.
