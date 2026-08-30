# Novel replacement branch assessment

This file records the authoritative state of `novel-rep` and a concise ledger
of approaches already tried. Superseded benchmark matrices remain in
`benchmark.csv`; verification details remain in `verification.md`. Historical
work is retained here only when it records a mechanism, outcome, or warning
that should influence future planner work.

## Current verdict — 2026-08-29

The branch is a working research milestone, not yet the requested beta. It has
one public obstacle planner, a separately owned dimension-neutral BMTP
trajectory package, exact direct and certified event-based motions, stable
failure diagnostics, and independent continuous validation. It does not yet
meet the user's combined core-size, runtime, arrival, and path-record gates,
and the maintained matrix does not prove general completeness or global
optimality.

- Branch: `novel-rep`; the last fully verified algorithmic milestone is
  `5357a6a`.
- Tests: 66/66 passed in 41.3109654 s after the latest deletion.
- Maintained examples: the last complete matrix at `944a738` had 16
  validated successes and one expected validated no-path result.
- Non-plotting core: 8,467 lines across 50 files, 3,468 over the ceiling.
- Two opposing U: exact `649/30 s` arrival, 24.0968121271875 deg path, and
  3.0005152 s full wall. This gate passes.
- Runtime records: Straight Target is 52.0803077 s planner, Target Exits is
  21.9396096 s, and Extreme is 20.5615572 s planner plus 147.204954 s for the
  whole example. This gate fails.
- Arrival records: Obstacle Avoidance is about 46.57 ms late. Extreme is
  1.179 ms late but inside the user's 2.07 ms allowance. The literal gate
  still fails.
- Path records: Straight Target and Target Exits remain longer than their
  historical records. This gate fails.

## Current architecture and invariant ownership

- `obstacleAvoidance.planTrajectory` is the sole public fixed-goal planner.
- Planner input, obstacle, geometry, search, planning, and validation packages
  own their domain responsibilities.
- `trajectory/+bmtpEngine` owns dimension-neutral jerk-limited motion and does
  not import obstacle or Az/El packages.
- Obstacle construction, planning, validation, and the user-frozen plotting
  package remain separate; plotting is excluded from the core-size target.
- The public validator independently owns endpoint, time, workspace,
  velocity, acceleration, jerk, dynamics, collision, resolution, safety
  margin, and certificate acceptance.
- Obstacle grouping is admissible only when independent exact-region coverage
  is reconstructed; silent broad merging is prohibited.
- Expected no-path and bounded-work outcomes retain stable result and search
  diagnostic records.

The non-planner, non-plotting foundation is approximately 2,695 lines. Under
the 4,999-line gate, a replacement planner, search, and trajectory generator
have a combined budget of about 2,304 lines.

## Strongest current results

### Exact Two-U physical floor

The request requires 20 degrees of elevation travel from rest to rest under
`1 deg/s`, `0.75 deg/s^2`, and `2.5 deg/s^3`. The independent scalar switching
lower bound is exactly `649/30 s`. A retained degree-15 progress polynomial
attains it and passes every public continuous check with
0.00618966852407 deg protected clearance. Latest planner and full walls are
1.9101322 and 3.0005152 seconds. This proves global minimum arrival for that
request, not globally minimum path length.

### Other accepted current results

- Obstacle-free direct: 4.53112887414927 s.
- Static U: 20.7124477849715 s and 40.2550285014326 deg.
- Opening U: 11.5843333838333 s.
- Dense concave and moving circle: exact 8.5 s fixed-clock motions.
- Earliest affine moving-target intercept: exact `55/9 s`.
- Moving/deforming outline: 7.91666666666667 s fixed-clock motion; scenario
  construction remains expensive.

## Current blockers and next bounded gate

1. Remove or replace 3,468 counted core lines without weakening the public
   interface, diagnostics, generality, or independent validation.
2. Replace expensive fixed-arrival BMTP work on Straight Target and Target
   Exits.
3. Close the Obstacle Avoidance arrival gap and both path-record gaps without
   scenario branches or relaxed tolerances.
4. Reduce Extreme planner and scenario-construction wall while preserving
   exact protected geometry and the accepted arrival tolerance.
5. Rerun all maintained examples sequentially before another release claim.

The next fixed-arrival method must retain 20.8695652173913 s and beat
13.678271908 deg on Straight Target, then retain 24 s and beat the actual
20.2803317257 deg Target Exits record. Comparisons must include like-for-like
planner-scope work, and the unchanged public validator must pass.

## Experiment ledger

### 2026-08-21 — early low-dimensional spline replacement

- **Bounded quintic B-spline:** one-, two-, and five-turn cases constructed
  faster than HS3 but arrived 14–35% later. Ten turns remained colliding after
  mean, worst-clearance, and per-obstacle objectives; 20 turns was not run.
- **Fixed-stop septic Bezier:** interpolated vertices but required 28–84 s
  motion and violated the maintained polynomial format; removed.
- **Interior-route interpolation:** reduced 10-turn wall but worsened
  clearance because route reduction discarded topology; removed.
- **Scalar option sweeps:** timing reserve retained route detail but took
  140.74 s with only 0.000145 deg clearance. Duration weight, collision
  penalty, step size, and offset-bound changes did not repair the mechanism.
- **Feasibility-first hard corridor and worst-clearance ranking:** both failed
  focused gates and were removed.
- **Topology-preserving batching/reduction:** produced valid 10-turn splines
  in 5.22–8.45 s but with less than 0.001 deg clearance.
- **Affine corridor prototype:** reached certified 0.02 deg clearance on a
  10-turn route and a 12-wall maze. The 20-turn motion exceeded its horizon,
  so no production replacement was established.
### 2026-08-22 — corridor-only replacement development

- **Corridor-only cutover:** removed dormant HS3/NLP paths and established an
  input-driven visibility/corridor planner. It passed maintained cases but did
  not meet global arrival or size requirements.
- **Span-demand controller:** replaced 180 timing trials with bounded
  per-span feedback. It was faster on Single U with a measured arrival
  penalty; later superseded.
- **Batched affine corridor work:** reduced repeated timing and constraint
  construction. Broader timing variants regressed records and were removed.
- **Dynamic seed-slot reservation:** recovered one moving-circle field while
  preserving deterministic order; broader timed-route replacement failed.
- **Shallow collision-residual feedback:** recovered narrow penetrations.
  Applying it to deeper residuals regressed Extreme to 8.395298096 s, so the
  broad form was rejected.
- **Retry-exhausted boundary support:** workspace-corner nodes recovered the
  fixed moving-circle sweep. Earlier activation regressed a maintained result.

### 2026-08-23 — routing and runtime work

- **Combined method suite:** exposed separate corridor and HS3 choices without
  blending internals; later superseded by single-planner cutovers.
- **Ungrouped collision broad phase:** materially accelerated 40 circles with
  exact trajectory parity; retained historically.

### 2026-08-24 — compact planner and stress scenarios

- **Compact corridor cutover:** consolidated static, moving, deforming,
  timed-hold, and nonzero-derivative planning; later replaced. A cross-frame
  stress case exposed a seed collision that HS3 solved and the validator
  correctly rejected.
- **Standalone Hermite-Simpson sequence:** restored an independent comparison
  engine, made it dimension-neutral, and separated collocation from obstacle
  routing. Conditioning, runtime, and record quality remained weak, and BMTP
  later removed it.
- **Corridor regression recovery:** route-scaled representation and
  geometry-derived timing repaired large saved dense-route regressions.

### 2026-08-25 — HS3-only checkpoint

- **HS3-only production cutover:** removed corridor-quintic code and 2,525
  lines. Results validated, but runtime and record quality remained weak.
- **Compact C3 duration controller and exact derivative retimer:** exercised
  as corridor-only replacements. The retimer improved motivating U cases but
  not every maintained arrival record.

### 2026-08-26 — quality, timing, and architecture experiments

- **Dynamics-timescale mesh start:** improved long dynamic detours and
  paired-horizon replay consistency. Broad static-U use regressed arrival and
  was rejected.
- **Direct long-detour refinement:** jumping from 10 to 40 segments retained
  best 40-circle quality; the faster 30-segment point arrived later.
- **Deforming-outline localization:** found polygon buffering and nonlinear
  constraints dominant. A classification micro-optimization was removed.
- **Obstacle-free bounded arrival:** reused fixed-time feasibility search to
  improve arrival and wall.
- **Same-homology shortcutting:** removed route edges only when visibility and
  winding signatures were preserved.
- **Certified direct collinearity:** preserved Euclidean paths for eligible
  common-bottleneck endpoint states; no runtime claim.
- **Two-product and flat architectures:** tried separate Az/El and trajectory
  products, then flat planner packages; both were later superseded.
- **Fixed-arrival geometric lower bound:** length-ordered seeds could stop at
  the Euclidean lower bound. The final matrix was slightly slower than a prior
  record.
- **Fixed-arrival length-first ranking:** selected shortest validated motion
  before jerk tie-breaking and improved alternating occlusion.
- **Severe-static fixed-time quality solve:** a bounded fine solve improved an
  extreme static local minimum without fixture names.
- **Derivative-slack continuation:** refined velocity-dominated candidates;
  broader continuation produced regressions.
- **Dynamic spatial quality:** refined only already validated moving routes.
  Starting all seeds fine and broader defaults were rejected.
- **Shortest-route-first ordering:** improved Extreme selection and a
  paired-horizon replay.
- **Time-expanded retiming:** removed zero-length waits and retimed the same
  topology; retained only after validation.
- **Timed-arrival repair:** fixed horizon-sensitive warm starts and exhaustive
  failure reporting, but remained later than one saved historical result.

### 2026-08-27 — performance and invariant consolidation

- **Bounded sandbox planning:** reduced interactive work but made one moving
  result 26.93% later; retained only as preview policy.
- **Exact third-order switching and scalar progress:** synchronized axes and
  preserved straight multi-axis paths. One moving-target timing observation
  was unfavorable.
- **Unified seed equivalence:** removed duplicate code; runtime missed the 5%
  gate, so only ownership reduction was claimed.
- **Prepared dynamic boundary queries:** accelerated exact moving-edge work.
- **Convex direct-route arrival search:** used bounded fixed-time feasibility
  bisection on geometry-certified direct routes.
- **Monotonic direct-line progress:** certified nonnegative progress only for
  eligible rest-to-rest collinear routes.
- **Batched occupancy and deferred allocation:** preserved blocker semantics
  while reducing multi-ring work.
- **Static geometry and polynomial-map caches:** cached only proven invariant
  geometry and algebra; retained for measured reductions.
- **Prepared constraint-layout reuse:** improved Opening-U profiling. A timed
  moving-barrier CG variation regressed wall and was removed.

### 2026-08-28 — hybrid replacement experiments

- **Direct Ruckig before topology:** accelerated eligible direct requests and
  continued after collision; later removed from retained production.
- **Optional pass-through warm start:** improved Single U quality but did not
  establish a runtime win; later isolated behind a deletable boundary.
- **Nonuniform mesh:** retained explicit engine support. The adaptive policy
  did not improve selected Two-U or dense results and was removed.
- **Adaptive static hybrid:** improved selected cases but had fresh-run
  variability and several distinct mechanisms.
- **Alternating-slalom hybrid:** achieved a 10.7625 s validated result. A
  broader near-direct trigger was rejected.
- **Moving/deforming runtime gate:** query reuse was only 9.02%; skipping the
  coarse basin escalated to 192.03 s. A short iteration cap regressed path and
  was removed.

### 2026-08-29 — current replacement branch

- **Exact waypoint fallback:** composed exact jerk-limited stops along a failed
  spatial seed and recovered a failed multi-edge request. Swept-envelope HS3
  was rejected.
- **BMTP cutover:** replaced legacy production with a separated Bezier/plane
  engine, exact direct/event kernels, and complete region certificates.
- **Exact-clock progress polynomial:** attained the Two-U global arrival lower
  bound with sub-10-second wall and truthful route provenance.
- **Full fixed-corridor Bezier QP:** rejected. Static U produced
  30.047333018 s and about 45.6554866 deg versus retained 20.712447785 s and
  40.2550285 deg. Missing certificate ownership was diagnosed, but quality
  already failed, so experiment code and artifacts were removed.
- **Fixed-clock null-space family:** a 109-variable form beat the dense path
  target and validated but took 35.14 s. A seven-coefficient form retained
  quality, then passed Straight Target with a fully plane-certified
  13.582258304 deg path at the exact 20.869565217 s clock in 8.03538 helper
  seconds. Target Exits remained valid but took 22.1424908 helper seconds and
  produced 21.811076622 deg versus the 20.280331726 deg record. Shared
  preparation and direct validation were excluded from that timer, so no
  general or end-to-end speed claim was retained.
- **Rest-state safe-interval search:** rejected as a universal replacement.
  On Alternating Slalom, its 8-vertex visibility route was only
  16.0193197983 deg, but exact rest-to-rest jerk primitives summed to
  24.6732769008 s versus the retained 10.5 s fly-through motion. Stopping at
  every graph vertex cannot meet the maintained arrival records.
- **Open-quintic route smoother:** rejected at its first frozen gate. It made
  a fully validated Straight Target trajectory in 3.0661778 s candidate scope
  at the exact 20.869565217 s clock, but its 13.7395585901 deg path missed the
  13.678271908 deg record. Target Exits was not run after that gate failed.
- **Historical eight-span C3 sampled-barrier QP:** rejected at its first
  frozen gate. Straight Target remained fully valid at the exact
  20.869565217 s clock, but its 13.7165279811 deg path missed the
  13.678271908 deg record and projected planner scope was 5.0271313 s versus
  the 2.0964864 s wall record. Target Exits was not run, and all experiment
  code and artifacts were removed.
- **Continuous Bernstein safe-corridor QP:** rejected at its first frozen gate.
  Its 451-line candidate passed the public collision, kinematic, exact-clock,
  and seed-corridor checks on Straight Target, but produced a
  14.1505333646 deg path in 90.488413 s candidate scope versus the
  13.678271908 deg and 2.0964864 s records. Target Exits was not run, and all
  experiment code and artifacts were removed.

## Cross-cutting rejected solver and micro-optimization trials

- First-valid-seed stopping regressed Two-U arrival; broad early continuation
  regressed 40-circle runtime.
- A bit-exact vectorized static-corridor path and direct Bernstein caching won
  microbenchmarks but regressed end-to-end dense or Extreme cases.
- Broad SQP, global conjugate-gradient, limited-memory BFGS, and tested PCG
  settings timed out, regressed cases, or produced no meaningful gain.
- Broad average-speed initialization regressed earliest dense concavity and was
  restricted to fixed arrival.
- Tighter arrival tolerance recovered about one millisecond but made Two-U
  much slower. Globally skipping recovery made Alternating Slalom fail.
- `TypicalX`, sparse Jacobians, repeated-public-query removal, and cache
  variants missed their declared 5% gates.
- Nargout-sized polynomial allocation was slower and was removed.
- Increasing clearance expansion from 2% to 5% improved one runtime but
  materially regressed Wide-U arrival.
- Parallel Computing Toolbox and `parfor` paths were removed because no sound
  end-to-end benefit was established.
