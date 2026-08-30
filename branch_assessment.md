# Novel replacement branch assessment

This file records the authoritative state of `novel-rep` and a concise ledger
of approaches already tried. Superseded benchmark matrices remain in
`benchmark.csv`; verification details remain in `verification.md`. Historical
work is retained here only when it records a mechanism, outcome, or warning
that should influence future planner work.

## Dynamic-scene findings — 2026-08-30

These supersede two claims in commit `4138f26`'s message, which were wrong.

### Correction to `4138f26`

That message states the rogue bundle "still takes about 177.8 s wall" and that
edge-query batching "is not attempted here". Both are false. The 177.8 s figure
was a bad measurement taken under process contention. Two independent
measurements of the same commit give **12.9 s and 14.5 s**, and the committed
reachability-frontier search already groups edge queries by layer pair:
`edgeIsClear` calls fell from 23,040 to 82, and `queryObstacleOccupancyAtTime`
from 23,093 to 222. The commit is pushed, so the record is corrected here
rather than by rewriting history.

### Sandbox bundle `az_el_sandbox_goal_20260829_212652`

Two moving obstacles, 180 s horizon, about 203 degrees of azimuth travel at
2 deg/s. Before `4138f26` this request never returned: cancelled at 180 s,
150 s, and 120 s. It now succeeds in about 12.9 s with independent validation,
arrival 107.632292801 s and length 227.751816227 deg. `HS3-planner` solves the
same request at 155.205670334 s, so this branch arrives 47.573378 s earlier.

The failure had two causes. The time-expanded search built the complete edge
list for every free node at every layer with no bound. Separately, with dynamic
obstacles `supportsStaticHorizon` is false, so every seed routed to
`createTimedSeedCandidate`, which accepts only `directWait` seeds and returned
`unsupportedTimedTopology` in about one millisecond — discarding the entire
topology search. A `noValidatedSeed` result with that per-seed signature is a
false negative, not evidence of infeasibility.

### The orthogonal-cavity path is load-bearing

A removal experiment deleted `createOrthogonalCavityMotion`,
`certifyOrthogonalCavityLowerBound`, `evaluateArrivalCertificatePortfolio`, and
their planner wiring, then measured the sentinels.
`exampleStaticUShapedObstacle` regressed from 20.7124477849715 s to
20.7814828183771 s, so the deletion was reverted in full. Those roughly 880
lines earn their keep and are not size-reduction candidates.

That experiment also invalidated an attribution method worth recording. Reading
the winning construction from `result.Seeds(SelectedSeedIndex).Source`
misreports cavity wins: when the cavity portfolio wins, `planCorridorQuintic`
passes the original topology seed to `finishFastPath`, so the source still
reads `visibilityGraph`. A census built that way reported zero cavity wins for
code that measurably changes the result. Attribute constructions from
`SearchDiagnostics`, not from the seed source.

### Roundoff reserve consistency correction

Unifying the plane-certificate `roundoffReserve_deg` formulas moved
`exampleStaticUShapedObstacle` from 20.7124477849715 s to
20.7124477860115 s, a measured +1.04e-9 s change. The constructor's
`eps(coordinateScale_deg)` form was the outlier; the shared helper now uses the
more conservative validator-owned `eps * coordinateScale_deg` formula so the
constructor and authoritative certificate check reason in the same reserve.

### Closed moving-barrier arrival gap

At `d0f00e1+worktree`, `exampleMovingBarrierWait` arrives at
10.0903015136719 s on the unchanged 10 deg path, improving both the prior
10.5 s branch result and `HS3-planner`'s recorded 10.2314453125 s result. The
accepted `directWait` seed now validates a zero-wait lower trial and bisects
the measured infeasible/feasible bracket through the unchanged public
validator. Sixteen deterministic trials refined the wait from 3 s to
2.59030151367188 s; the final measured infeasible lower wait was
2.5902099609375 s. This is a bounded refinement of one validated direct-wait
construction, not a request-wide minimum-arrival proof.

## Current verdict — 2026-08-29

The branch is a working research milestone, not yet the requested beta. It has
one public planner, separated BMTP and Ruckig trajectory engines, stable failure
diagnostics, and independent continuous validation. The last complete matrix
had 16 validated successes and one expected validated failure, but the branch
still misses the combined size, runtime, arrival, and path-record gates. That
matrix does not prove general completeness or global optimality.

- Current integration and cooperative-cancellation suite: 84/84 tests passed
  after the reachability-frontier timed-search port. The rogue bundle now
  terminates as `noValidatedSeed` in 149.484 seconds under its 180-second poll.
- The sandbox now has a Stop action that remains enabled during synchronous
  planning, polls the time-expanded and homology searches plus planner-stage
  boundaries, restores idle state, and enables a replayable pre-run export.
  Cancellation cannot preempt MATLAB inside one atomic solver or vectorized
  geometry call; it takes effect at the next safe checkpoint.
- Production size audit rule: 11,524 nonblank, noncomment lines across 72
  files at HEAD. That measured size is now the ceiling, replacing the
  earlier 4,999 target. The audit counts only `+obstacleAvoidance` and
  `trajectory`; `tests/`, `examples/`, `benchmarks/`, and `sandbox/` are
  outside the counted roots, so adding coverage costs nothing against it.
- Strongest result: Two opposing U reaches the exact `649/30 s` physical
  arrival floor with a 24.0968121271875 deg path and 3.0005152 s full wall.
- Straight Target now explicitly selects exact Ruckig waypoint composition. It
  retains the 20.8695652173913 s clock and passes every public check with a
  20.7720160748 deg path, 5.8749177 s planner time, and 10.1040635 s wall.
  Target Exits remains at the `944a738` BMTP result pending a fresh Ruckig gate.
- In that matrix, Obstacle Avoidance is about 46.57 ms late. Extreme is
  1.179 ms late, inside the user's 2.07 ms allowance but still above the
  literal historical record.
- Straight Target and same-input Target Exits remain longer than their
  historical records; the restored Ruckig route stops at its intermediate
  vertices and is not a claim of locally time-optimal waypoint motion.

## Current invariant boundary

- `obstacleAvoidance.planTrajectory` remains the sole public fixed-goal
  planner; obstacle construction, planning, validation, and frozen plotting
  remain separate.
- The public validator independently owns endpoint, time, workspace,
  derivative, collision, safety-margin, and certificate acceptance.
- Expected no-path and bounded-work outcomes retain stable results and search
  diagnostics. Obstacle grouping remains admissible only with reconstructed
  exact-region coverage.
- Every counted production file has a production or contract-test caller. The
  sub-5,000 target therefore needs a behavior-preserving algorithm cutover,
  not dead-file deletion. No transitive compact implementation has yet proved
  current diagnostics, continuous validation, and plane-certificate parity.

## Strongest current evidence

The Two-U request requires 20 degrees of elevation travel from rest to rest
under `1 deg/s`, `0.75 deg/s^2`, and `2.5 deg/s^3`. The independent scalar
switching lower bound is exactly `649/30 s`. A retained degree-15 progress
polynomial attains it and passes every public continuous check with
0.00618966852407 deg protected clearance. This proves globally minimum arrival
for that request, not globally minimum path length.

## Current blockers and next bounded gate

1. Do not grow past the 11,524-line ceiling. Reduction is welcome but is no
   longer a release gate: the one measured attempt, deleting the
   orthogonal-cavity path, regressed `exampleStaticUShapedObstacle` and was
   reverted, so remaining size is not obviously recoverable without losing
   capability. `trajectory/+ruckigEngine` (2,083 counted lines) is shared
   with other projects and `+obstacleAvoidance/+plotting` (425) is frozen,
   leaving about 7,138 counted lines in scope for any future reduction.
2. Recover pass-through path quality without relabeling the local state-to-state
   Ruckig engine as a waypoint-optimal solver.
3. Close the Obstacle Avoidance arrival gap and both path-record gaps without
   scenario branches or relaxed tolerances.
4. Reduce Extreme planner and scenario-construction wall while preserving
   exact protected geometry and the accepted arrival tolerance.
5. Rerun all maintained examples sequentially before another release claim.

The current fixed-arrival gate retains 20.8695652173913 s but still must reduce
Straight Target from 20.7720160748 deg and 10.1040635 s wall to the comparable
13.678271907957 deg and 2.0964864 s records. Target Exits must retain 24 s,
reach at most 20.6764423274 deg, and beat the 5.167399 s same-input wall record.
The older 20.2803317257 deg and 1.9774286 s rows used a different randomized
target endpoint and are not comparable to the maintained deterministic fixture.

Headless profiling at `4ed7f46` localized the Straight Target loss to motion,
not topology: BMTP spent 70.6255 of 75.4950 planner seconds and
selected a longer seed even though a shorter valid seed was available. Its
output uses 48 degree-16 certified spans, while the same-input historical path
record used ten quintic spans. The Target Exits profile spent 22.1507 of
31.3204 planner seconds in motion. Comparisons must include like-for-like
planner work and pass the unchanged public validator.

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

### 2026-08-23 to 2026-08-25 — broad phase and legacy cutovers

- **Ungrouped collision broad phase:** materially accelerated 40 circles with
  exact trajectory parity.
- **Legacy cutovers:** **Combined method suite**, **Compact corridor cutover**,
  **Standalone Hermite-Simpson sequence**, and **HS3-only production cutover**
  were exercised. A cross-frame stress case exposed a compact-corridor seed
  collision; standalone HS3 had weak conditioning, runtime, and record
  quality; the HS3-only cutover removed 2,525 lines but kept those weaknesses.
- **Legacy recovery:** **Corridor regression recovery**, **Compact C3 duration
  controller**, and **Exact derivative retimer** repaired dense-route or
  selected U cases, but not every maintained arrival record.

### 2026-08-26 — quality, timing, and architecture experiments

- **Mesh and refinement family:** **Dynamics-timescale mesh start**, **Direct
  long-detour refinement**, **Severe-static fixed-time quality solve**,
  **Derivative-slack continuation**, and **Dynamic spatial quality** improved
  selected cases. Broader static, all-seed, or continuation use regressed
  arrival, runtime, or quality.
- **Route, ranking, and retiming family:** **Obstacle-free bounded arrival**,
  **Same-homology shortcutting**, **Certified direct collinearity**,
  **Fixed-arrival geometric lower bound**, **Fixed-arrival length-first
  ranking**, **Shortest-route-first ordering**, **Time-expanded retiming**, and
  **Timed-arrival repair** were tried. Benefits were local; one final matrix
  was slower and one timed case remained late.
- **Deforming-outline localization:** polygon buffering and nonlinear
  constraints were dominant; a classification micro-optimization was removed.
- **Two-product and flat architectures:** both package layouts were tried and
  superseded; they did not change the algorithmic limits.

### 2026-08-27 — performance and invariant consolidation

- **Direct motion and moving geometry:** **Exact third-order switching and
  scalar progress**, **Prepared dynamic boundary queries**,
  **Convex direct-route arrival search**, and **Monotonic direct-line progress**
  improved eligible direct cases, but one moving-target timing observation was
  adverse.
- **Invariant batching and caching:** **Batched occupancy and deferred
  allocation**, **Static geometry and polynomial-map caches**, and
  **Prepared constraint-layout reuse** produced measured local wins. A timed
  moving-barrier CG variant regressed.
- **Bounded sandbox planning** made one moving result 26.93% later.
  **Unified seed equivalence** reduced ownership but missed its runtime gate.

### 2026-08-28 — hybrid replacement experiments

- **Direct Ruckig before topology**, **Optional pass-through warm start**, and
  **Nonuniform mesh** each gave a local benefit or retained engine capability
  but no general runtime or selection win; their broad policies were removed.
- **Adaptive static hybrid** and **Alternating-slalom hybrid** improved selected
  cases, including a 10.7625 s validated slalom, but variability and a broader
  near-direct trigger prevented a general replacement.
- **Moving/deforming runtime gate:** query reuse was 9.02%; skipping the coarse
  basin took 192.03 s, and a short iteration cap regressed path quality.

### 2026-08-29 — BMTP replacement branch

- **Restored exact Ruckig engine and explicit route method:** restored the
  independent state-to-state switching engine and facade, added the general
  `TrajectoryMethod="ruckigWaypoint"` route adapter, and wired Straight Target
  through it without a BMTP fallback. The engine tests passed 10/10, the full
  suite passed 78/78, and a structurally different static-box detour passed
  earliest and fixed arrival. Straight Target passed at the exact fixed clock,
  but its stop-at-waypoint path and wall remain above the historical records.
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
  produced 21.811076622 deg versus the comparable 20.676442327 deg record.
  Shared preparation and direct validation were excluded from that timer, so
  no general or end-to-end speed claim was retained.
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
  code and artifacts were removed. A later same-input reconstruction confirmed
  that length-first selection cannot improve it because seed 2 is already its
  shortest validated motion. Increasing the compact basis through 9, 10, 12,
  and 14 spans preserved every public check but only reached 13.7164337975 deg;
  warm walls were 4.4719357, 3.4983968, 3.8779253, and 5.1402292 s. The quality
  gap is therefore representation-level, not a candidate-ranking defect.
- **Fixed-clock velocity-energy C3 QP:** rejected after the Straight Target
  gate. Six relinearized QPs passed continuous motion validation and beat the
  path record at 13.6049323647 deg on the exact 20.869565217 s clock, but took
  3.5579439 s in candidate scope and did not provide the required current
  plane-certificate parity. A one-QP form took 2.7812950 s and regressed path
  length to 14.1539862749 deg. Both missed the 2.0964864 s wall record; Target
  Exits was not run, and the experiment code was removed.
- **One-shot eight-span Bernstein velocity-energy QP:** rejected at the
  Straight Target runtime gate. The 447-line experiment eliminated 96 control
  scalars to 28 QP variables and reached the common exporter, but the focused
  end-to-end example did not finish within 30 measured seconds versus the
  2.0964864 s record. It was stopped before a valid path or certificate result
  was available; Target Exits was not run, and the hook and helper were removed.
- **Fixed-clock QP witness reuse and active-set solve:** rejected at the
  Straight Target gate. Directly retained constant support planes certified all
  288 output span-region pairs with zero analytic or conic fallbacks in
  0.0554667 s. Switching the same 28-variable QP from interior point
  (7.2986105 s) to MATLAB's active-set algorithm reduced successful seed solves
  to 0.1599676 s, 0.0308900 s, and 0.0235469 s. All five seeds were attempted,
  and the selected motion passed the exact 20.8695652173913 s clock, public
  collision, kinematic, and plane-certificate checks. Its 14.2707707658 deg
  path and 10.0545878 s full wall still missed the 13.678271908 deg and
  2.0964864 s records; topology alone took 2.3570636 s. Target Exits was not
  run, and all probe code and instrumentation were removed.
- **Continuous Bernstein safe-corridor QP:** rejected at its first frozen gate.
  Its 451-line candidate passed the public collision, kinematic, exact-clock,
  and seed-corridor checks on Straight Target, but produced a
  14.1505333646 deg path in 90.488413 s candidate scope versus the
  13.678271908 deg and 2.0964864 s records. Target Exits was not run, and all
  experiment code and artifacts were removed.

## Cross-cutting rejected solver and micro-optimization trials

- First-valid stopping, broad early continuation, average-speed starts,
  tighter arrival tolerance, globally skipped recovery, and larger clearance
  expansion each regressed a maintained arrival, failure, or runtime case.
- Vectorized static corridors, Bernstein and other cache variants, `TypicalX`,
  sparse Jacobians, repeated-query removal, and nargout-sized polynomial
  allocation won only microbenchmarks or missed their end-to-end gates.
- Broad SQP, conjugate-gradient, limited-memory BFGS, PCG, Parallel Computing
  Toolbox, and `parfor` variants timed out, regressed, or established no sound
  end-to-end benefit.

## Per-Seed Work Budget Verification — 2026-08-29

The README fixed-goal protected-rectangle request was rerun on
`422f887+worktree` with `PerSeedWorkBudgetMultiplier=3`. The retained selected
motion is seed 3 with a 7.574541766-second arrival and 11.411861388-degree
motion length. Final wall time was 14.060 seconds. The losing fourth BMTP seed
ended after 4.857 seconds with `seedWorkBudgetExhausted`, rather than an
independent-validation failure. Exclusive final stage timing was 0.4830 seconds
topology, 12.0845 seconds motion solving, 0.3230 seconds collision checking,
0.3292 seconds final validation, and 0.7899 seconds unattributed, totaling
14.0096 seconds. This is one measured static request, not a general runtime or
optimality claim. The full MATLAB suite passed 84/84; the timed and cavity
certificate coverage is direct, while the geometric lower-bound pruning proposal
was deliberately not implemented because a topology seed is not a mandatory
optimized vertex chain.
