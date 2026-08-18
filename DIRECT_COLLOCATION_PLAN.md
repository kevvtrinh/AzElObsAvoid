# Direct-Collocation Implementation Plan

Source context: local copy of the user-provided shared-chat plan. The shared
page was unavailable during implementation, so this complete local copy is the
branch specification.

## 1. Objective

Start from:

- Repository: https://github.com/kevvtrinh/AzElObsAvoid
- Baseline branch: `experimental`
- Trajectory-optimization reference:
  https://epubs.siam.org/doi/epdf/10.1137/16M1062569

Rebuild the production planner around **minimum-time direct trajectory optimization** rather than the current sequence of:

1. build a visibility-graph polyline,
2. smooth that fixed geometry,
3. retime the fixed smooth path with CoPP/TOPP3.

The new planner shall optimize the **spatial trajectory and timing together** so that it can carry nonzero velocity through turns, subject to per-axis velocity, acceleration, and jerk limits.

The target behavior is:

> Find the earliest validated azimuth/elevation trajectory from the supplied initial state to the supplied goal state while satisfying moving/static obstacle avoidance and velocity, acceleration, and jerk constraints. Visibility-graph routes are only initial guesses/topology seeds; they are not hard waypoints and do not define the final path.

The first production implementation shall use **Hermite–Simpson direct collocation** with jerk as the control. Do not begin with a pseudospectral or Radau implementation. Keep the architecture simple enough to validate thoroughly first.

---

## 2. Repository Rules

Before changing code, read and follow the repository-root `AGENTS.md` on the `experimental` branch.

In particular, preserve these repository principles:

- one maintained public planner entry point;
- general-purpose behavior, not scenario-specific logic;
- obstacle construction, planning, validation, and visualization remain separate;
- expected planning failure returns a stable result with `Success = false`;
- search/optimization diagnostics are first-class outputs;
- MATLAB is the source language and behavioral reference;
- safety margin is applied exactly once and the canonical obstacle data remains the source of truth;
- examples use the maintained public planner rather than implementing planning logic themselves;
- do not hide constraint violations by clipping or post-processing the returned trajectory.

Keep the public call pattern:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

Preserve the zero-input defaults call:

```matlab
options = planAzElMotion();
```

Do not create competing public planners for static obstacles, moving obstacles, minimum-time planning, or jerk-constrained planning.

---

## 3. Baseline the Existing `experimental` Branch

Before refactoring, run and record the existing tests and representative examples.

At minimum run:

- `tests/testMinimalAzElWorkflow.m`
- `tests/testAzElInternalUtilities.m`
- `tests/testCoppRetimer.m`
- `tests/testBenchmarkComparison.m`

Also run a representative subset of examples:

- `exampleAzElPlanning`
- `exampleUShapedAzElTimeSpace`
- `exampleTwoOpposingUVisibilityGraph`
- `exampleAlternatingSlalom`
- `exampleMovingCircleNoAzimuthWrap`
- `exampleFourAcceleratingCircles`
- `exampleStraightTargetAlternatingOcclusion`

Record:

- pass/fail state;
- arrival time;
- peak velocity;
- peak acceleration;
- peak jerk;
- minimum obstacle clearance if available;
- number of intermediate near-stops;
- planning time;
- current dependency on CoPP.

This baseline is for regression comparison only. Do not preserve an old algorithm merely to match old numerical values.

---

## 4. Current Architecture to Replace

The `experimental` branch currently has the production planner `planAzElMotion.m`, which performs adaptive visibility-route generation and then evaluates candidate routes.

The current candidate pipeline reaches:

```text
visibility route
    ↓
azElInternal.buildAzElSmoothPath
    ↓
azElExperimental.retimeCoppSmoothPath
    ↓
CoPP TOPP3 retiming of fixed geometry
    ↓
collision/constraint validation
```

The current evaluator explicitly constructs fixed collision-checked geometry and retimes it with CoPP/TOPP3.

That fixed-path separation is the part to remove from the production planner.

### Keep

Retain, simplify where possible, and reuse:

- `makeAzElObstacleData.m`
- `combineAzElObstacles.m`
- `buildAzElTimeObstacleField.m`
- `queryAzElTimeObstacle.m`
- `queryAzElTimedPathCollision.m`
- `recoverOriginalAzElObstacleField.m`
- `buildAzElVisibilityRoutes.m`
- the snapshot/event-detection pieces needed for moving-obstacle topology seeds;
- existing public validation and plotting infrastructure where it remains applicable.

### Remove from the production path

The new planner must no longer depend on:

- `azElInternal.buildAzElSmoothPath` as the final geometry generator;
- `azElExperimental.retimeCoppSmoothPath`;
- CoPP/TOPP3;
- the current smooth-then-retime candidate evaluation pipeline;
- any rule that forces zero velocity at an intermediate visibility vertex;
- any fixed geometric path that the optimizer is forbidden to leave.

After the new planner is validated, delete obsolete files and tests if they have no remaining supported use. Do not leave dead production code simply for historical reference.

If `+azElExperimental` becomes empty, remove it.

---

## 5. Retain the Visibility Graph Only as a Seed Generator

`buildAzElVisibilityRoutes.m` remains useful, but its responsibility changes.

Its production responsibility shall become:

> Produce a bounded set of collision-informed, topologically distinct geometric seed routes and the metadata needed to initialize local trajectory optimization.

It must **not** decide the final trajectory.

### Required behavior

For each retained visibility route:

- preserve the geometric route;
- preserve its source snapshot/time metadata;
- preserve obstacle/topology metadata useful for constructing obstacle constraints;
- preserve a simple geometric cost only for seed ordering;
- never impose its intermediate vertices as equality constraints in the direct-collocation problem.

A route such as:

```text
start -> vertex A -> vertex B -> goal
```

means:

> initialize the optimizer on this side of the obstacles.

It must not mean:

```matlab
position(tA,:) == vertexA;
position(tB,:) == vertexB;
```

The direct optimizer must be free to cut away from visibility vertices and create a smooth time-optimal turn.

### Seed ordering

Evaluate the direct start-to-goal seed first when geometrically plausible.

Then evaluate retained visibility seeds in increasing geometric cost or another deterministic seed-order heuristic.

This ordering is only a runtime heuristic. Final selection is based on the optimized, independently validated trajectory objective.

---

## 6. New Dynamic Model

Use the azimuth/elevation state:

\[
x(t) =
\begin{bmatrix}
az \\
el \\
v_{az} \\
v_{el} \\
a_{az} \\
a_{el}
\end{bmatrix}
\]

and jerk control:

\[
u(t) =
\begin{bmatrix}
j_{az} \\
j_{el}
\end{bmatrix}.
\]

The continuous dynamics are:

\[
\dot{az} = v_{az}
\]

\[
\dot{el} = v_{el}
\]

\[
\dot{v}_{az} = a_{az}
\]

\[
\dot{v}_{el} = a_{el}
\]

\[
\dot{a}_{az} = j_{az}
\]

\[
\dot{a}_{el} = j_{el}.
\]

Do not build a separate path-coordinate retimer around scalar path speed.

The optimizer acts directly in azimuth/elevation state space.

---

## 7. Minimum-Time Objective

For `GoalTimeMode = "earliestArrival"`, final time is a decision variable.

Let:

```matlab
t0 = initialState.time_s;
tf = decision variable;
```

with:

```matlab
t0 < tf <= goalState.time_s;
```

where `goalState.time_s` remains the latest allowed arrival/horizon.

Primary optimization objective:

\[
\min t_f
\]

or equivalently:

\[
\min (t_f - t_0).
\]

This is the actual minimum-time problem.

### Secondary smoothness tie-break

Do not distort the primary time objective with an arbitrary large jerk penalty.

After finding a valid minimum-time solution `tfStar`, optionally run a second optimization:

\[
\min \int_{t_0}^{t_f} \|j(t)\|^2 dt
\]

subject to:

\[
t_f \le t_f^\star + \epsilon_t
\]

and all original constraints.

Use this only as a near-equal-time tie-break so the planner can choose a less aggressive jerk history without turning the problem into a minimum-jerk planner.

Report both:

- stage-1 minimum arrival time;
- final tie-break arrival time;
- allowed time tolerance.

If the tie-break fails, retain the valid stage-1 minimum-time solution.

### Fixed arrival mode

Preserve `GoalTimeMode = "fixedArrival"`.

In fixed-arrival mode:

```matlab
tf = goalState.time_s;
```

and the primary task is feasibility.

Use integrated squared jerk as the optimization objective after feasibility is established.

Do not enumerate separate departure waits as the old retimer does. Waiting, accelerating, slowing, and moving are all part of the same optimized state trajectory.

---

## 8. Hermite–Simpson Transcription

Implement **separated-form Hermite–Simpson collocation** first because it is easier to inspect and debug.

For every segment `k`, define:

- knot state `x_k`;
- midpoint state `x_mid`;
- next knot state `x_kp1`;
- knot control `u_k`;
- midpoint control `u_mid`;
- next knot control `u_kp1`;
- segment duration `h_k`.

Use the Hermite–Simpson relations from Kelly:

\[
x_{mid}
=
\frac{1}{2}(x_k+x_{k+1})
+
\frac{h_k}{8}(f_k-f_{k+1})
\]

and:

\[
x_{k+1}-x_k
=
\frac{h_k}{6}
(f_k+4f_{mid}+f_{k+1}).
\]

The implementation must use one central dynamics function:

```matlab
stateDerivative = azElDynamics(state, jerk);
```

with:

```matlab
stateDerivative = [ ...
    velocity_deg_s, ...
    acceleration_deg_s2, ...
    jerk_deg_s3];
```

arranged consistently with the chosen state vector.

### Important correction in the supplied paper

The attached PDF includes an erratum for Equation 4.13.

When implementing method-consistent Hermite–Simpson state interpolation, use the **corrected Equation 4.13 from the erratum**, not the erroneous form printed in the original body of the paper.

Add a code comment beside the interpolation implementation citing the paper and noting that the corrected equation is used.

---

## 9. Decision Variables

For a mesh with `N` segments, construct one deterministic decision-vector layout containing:

- final time for earliest-arrival mode;
- knot states;
- midpoint states if using fully separated form;
- knot jerk controls;
- midpoint jerk controls.

Centralize packing/unpacking in the direct-collocation implementation.

Do not scatter decision-vector indexing through the planner.

Suggested internal record after unpacking:

```matlab
trajectory.time_s
trajectory.position_deg
trajectory.velocity_deg_s
trajectory.acceleration_deg_s2
trajectory.jerk_deg_s3
```

The public result schema should remain stable, but internal optimization code should use this clear representation.

---

## 10. Boundary Constraints

At the first knot enforce exactly:

```matlab
position_deg        == initialState.position_deg
velocity_deg_s      == initialState.velocity_deg_s
acceleration_deg_s2 == initialState.acceleration_deg_s2
```

At the final knot enforce exactly:

```matlab
position_deg        == goalState.position_deg
velocity_deg_s      == goalState.velocity_deg_s
acceleration_deg_s2 == goalState.acceleration_deg_s2
```

unless an already-supported terminal-state option explicitly relaxes one of those requirements.

Do not silently force terminal velocity or acceleration to zero when the caller supplied nonzero values.

---

## 11. Kinematic Path Constraints

Enforce per-axis limits at every knot and midpoint:

\[
|v_{az}| \le v_{az,max}
\]

\[
|v_{el}| \le v_{el,max}
\]

\[
|a_{az}| \le a_{az,max}
\]

\[
|a_{el}| \le a_{el,max}
\]

\[
|j_{az}| \le j_{az,max}
\]

\[
|j_{el}| \le j_{el,max}.
\]

Use the existing fields:

```matlab
limits.maxVelocity_deg_s
limits.maxAcceleration_deg_s2
limits.maxJerk_deg_s3
```

Do not replace per-axis limits with only a scalar resultant-magnitude limit.

### Between-collocation validation

Do not assume knot and midpoint satisfaction proves the complete interpolated trajectory is within bounds.

For the final solution:

- reconstruct the method-consistent polynomial on every segment;
- analytically inspect polynomial extrema where practical;
- at minimum certify jerk extrema from the quadratic jerk interpolant;
- inspect velocity and acceleration extrema from their segment polynomials;
- report the maximum continuous/interpolated value for every constrained axis.

Dense sampling may be used as an additional check, but should not be the only kinematic certificate when the polynomial extrema are directly computable.

---

## 12. Obstacle Constraints: Do Not Put Boolean Collision Queries Inside `fmincon`

This is a critical implementation requirement.

Do **not** use:

```matlab
queryAzElTimeObstacle(...)
```

as a Boolean nonlinear constraint evaluated directly inside the NLP.

Do not use raw `inside/outside`, `min`, or nearest-edge switching logic inside the optimizer when it introduces discontinuous gradients.

Those existing functions remain appropriate for independent validation, not as the primary differentiable NLP constraint.

### First implementation: seed-conditioned local separating constraints

For each seed trajectory:

1. sample the seed at the collocation knot and midpoint times;
2. obtain the obstacle polygon geometry at those times;
3. identify the relevant nearby boundary edge or supporting separator for each obstacle interaction;
4. freeze that discrete edge/side association for one NLP solve;
5. express local obstacle clearance as smooth half-space/separating constraints.

Conceptually:

\[
n(t)^T
\left(
q(t)-p_{edge}(t)
\right)
\ge d_{clear}
\]

where:

- `q(t)` is `[az, el]`;
- `p_edge(t)` is a point on the associated obstacle boundary;
- `n(t)` points toward the seed-side free space;
- `d_clear` is zero when the canonical obstacle is already safety-inflated, except for a small documented numerical tolerance.

For moving/deforming polygons, evaluate/interpolate the selected edge geometry consistently with obstacle time.

### Corridor/association relinearization

After an NLP solve:

- independently collision-check the continuous trajectory;
- if it leaves the valid local separating region or collides because the active boundary association changed, rebuild the associations from the new trajectory and solve again;
- limit this outer relinearization loop with an explicit option/budget;
- record every pass in diagnostics.

Do not hide a failed obstacle constraint by clipping the trajectory.

### Safety margin

The direct optimizer consumes the already-protected canonical obstacle field.

Do not inflate obstacles again in the optimizer.

Original versus protected geometry remains available for plotting and diagnostics.

---

## 13. Moving Obstacles and Time

Moving obstacles must be queried using the actual collocation time.

For each collocation knot/midpoint:

```matlab
t = collocation time;
polygon = obstacle geometry at t;
```

The trajectory optimizer therefore works directly in the effective azimuth/elevation/time free space.

It may naturally choose to:

- pass before an obstacle;
- pass after an obstacle;
- slow down;
- speed up;
- detour spatially;
- combine spatial and temporal avoidance.

Do not implement those behaviors as scenario-specific branches.

### Temporal seeds

Moving-obstacle optimization is nonconvex.

Use the existing snapshot/event machinery only to provide multiple initial guesses representing different temporal/topological opportunities.

A snapshot time may influence the initial time guess, but it must not become a hard departure or arrival equality unless required by the user input.

---

## 14. Initial Guess Construction

A nonlinear trajectory optimizer requires a useful initial guess.

For each visibility seed:

1. parameterize the seed polyline by cumulative geometric distance;
2. resample it onto the initial collocation mesh;
3. smooth only enough to create finite numerical derivative guesses;
4. initialize velocity and acceleration from the seed trajectory;
5. initialize jerk from the derivative of acceleration or zero when appropriate;
6. initialize final time from a conservative geometric travel-time estimate bounded by the goal horizon.

The initial guess does **not** have to be dynamically feasible.

Do not retain the old smoothed geometry as a hard path.

### Multiple starts

For difficult moving-obstacle cases, solve multiple initializations:

- direct seed;
- each retained topology seed;
- selected early/middle/late temporal initialization when snapshot topology indicates materially different opportunities.

Keep this bounded and deterministic.

Use `UseParallel` only for independent seed optimizations when Parallel Computing Toolbox is available.

---

## 15. Minimum File Architecture

Avoid replacing one large planner with a dozen public helpers.

Target a small architecture.

### Public production files

Keep:

```text
planAzElMotion.m
buildAzElVisibilityRoutes.m
makeAzElObstacleData.m
buildAzElTimeObstacleField.m
queryAzElTimeObstacle.m
queryAzElTimedPathCollision.m
validateAzElExampleResult.m
plot/animation public utilities
```

### Internal optimizer modules

The initial implementation put all HS-3 responsibilities in one internal
file. The file reached 4,661 lines and 78 local functions. It is now too large
to review or change safely. Keep one optimizer entry point, but divide its
implementation by mathematical responsibility.

```text
+azElInternal/optimizeAzElDirectCollocation.m
+azElInternal/buildAzElHs3InitialGuess.m
+azElInternal/solveAzElHs3Subproblem.m
+azElInternal/buildAzElOptimizationCorridor.m
+azElInternal/reconstructAzElHs3Trajectory.m
+azElInternal/certifyAzElHs3Trajectory.m
+azElInternal/propagateAzElHs3Control.m
+azElInternal/buildAzElHs3SegmentPolynomials.m
```

Responsibilities are:

- `optimizeAzElDirectCollocation` owns candidate-stage order, time-budget
  allocation, mesh-refinement decisions, retained-solution selection, and
  final candidate diagnostics.
- `buildAzElHs3InitialGuess` owns route progress, duration trials, bounded
  control projection, and initial seed feasibility.
- `solveAzElHs3Subproblem` owns decision packing, bounds, the minimum-time and
  fixed-arrival solvers, affine constraint maps, objective functions, and
  Hermite--Simpson equality constraints.
- `buildAzElOptimizationCorridor` owns moving/static obstacle association,
  separating half-spaces, repair clearances, and corridor constraints.
- `reconstructAzElHs3Trajectory` owns method-consistent dense interpolation
  and curve-subdivision error bounds.
- `certifyAzElHs3Trajectory` owns continuous polynomial extrema, dynamic
  defect certificates, and kinematic-limit certificates.
- `propagateAzElHs3Control` is the single shared state-propagation invariant.
- `buildAzElHs3SegmentPolynomials` is the single shared power/Bernstein
  representation invariant.

Do not create a command-string dispatcher, a structure of function handles,
classes, or separate static/moving optimizer implementations. Each module has
one typed function interface. Static and moving obstacles use the same
corridor interface.

Use these size targets after extraction:

- optimizer entry point: at most 1,500 lines and 25 local functions;
- each internal module: at most 1,100 lines;
- total HS-3 implementation: at most 4,200 nonblank lines.

Splitting files alone does not satisfy the target. Delete transferred local
functions, merge repeated schema assembly, and remove repeated constraint-map
construction when the map inputs have not changed.

The packed obstacle slice accessor is a measured exception. Keep its small
local form inside frequent collision-query functions unless a replacement is
measured within five percent of its runtime. The package-call version was
14.3 times slower in the maintained microbenchmark.

### Required extraction order

1. Lock the current unit-test, example, and profiler baselines.
2. Extract shared HS-3 propagation and polynomial construction. Compare their
   runtime with the local functions before deleting the local copies.
3. Extract continuous reconstruction and certification.
4. Extract obstacle-corridor construction and repair.
5. Extract initial-guess construction.
6. Extract the NLP and fixed-arrival affine subproblem.
7. Reduce the optimizer entry point to orchestration and stable diagnostics.
8. Profile the longest example and remove only measured repeated work.

After each step, run focused tests and one example process at a time. Reject an
extraction that changes the trajectory, certificate, deterministic seed order,
termination reason, or runtime by more than five percent without a documented
reason.

---

## 16. Rewrite `planAzElMotion.m` as an Orchestrator

Completely replace the old smooth-and-retime logic.

The top-level flow should become:

```text
1. Validate inputs and resolve defaults
2. Build protected obstacle field
3. Validate start and goal occupancy
4. Build direct/topology seed routes
5. For each seed:
       build initial collocation guess
       solve minimum-time Hermite–Simpson NLP
       refine obstacle corridor if necessary
       refine mesh if necessary
       independently validate complete trajectory
6. Select earliest valid candidate
7. Optionally perform jerk-squared tie-break
8. Assemble stable result and diagnostics
```

`planAzElMotion.m` should not contain the full NLP equations.

The NLP belongs in `+azElInternal/optimizeAzElDirectCollocation.m`.

---

## 17. Candidate Selection

For each optimized seed, store:

```matlab
Success
TerminationReason
ArrivalTime_s
MotionDuration_s
IntegratedSquaredJerk
SeedRouteIndex
SeedRoute_deg
MeshPassCount
SolverIterationCount
SolverElapsedTime_s
MaximumVelocity_deg_s
MaximumAcceleration_deg_s2
MaximumJerk_deg_s3
MinimumClearance_deg
CollisionFree
Validation
```

Selection order:

1. valid collision-free candidates only;
2. earliest arrival time;
3. within the documented time tie tolerance, lower integrated squared jerk;
4. deterministic final tie-break such as lower seed index.

Geometric route length must not outrank arrival time.

---

## 18. Mesh Refinement

Do not solve only one arbitrary fixed mesh.

Implement a simple `h`-refinement loop first.

Recommended first-pass structure:

```text
coarse Hermite–Simpson mesh
    ↓
solve
    ↓
reconstruct continuous polynomial
    ↓
estimate segment defects / constraint stress
    ↓
split bad segments
    ↓
warm-start from previous solution
    ↓
solve again
```

Refine segments with:

- large collocation/dynamics defect;
- small obstacle clearance;
- near-active velocity, acceleration, or jerk limits;
- large local curvature/change;
- failed continuous validation.

Do not globally double every segment unless the diagnostics justify it.

Expose only a small number of meaningful options, for example:

```matlab
InitialCollocationSegmentCount
MaximumMeshRefinementPasses
CollocationErrorTolerance
MaximumCollocationSegmentCount
```

Reuse the existing planning-time budget rather than introducing redundant timeout controls.

---

## 19. Solver

Use MATLAB `fmincon` for the first implementation so the branch has no new required external trajectory-optimization dependency.

Prefer:

```matlab
Algorithm = "interior-point"
```

initially, with explicit tolerances and iteration/evaluation limits.

Provide gradients only after the finite-difference implementation is correct and thoroughly tested.

Then add analytic/sparse gradients as a performance optimization without changing planner behavior.

Do not make IPOPT, SNOPT, GPOPS-II, OptimTraj, or another external package mandatory for the first implementation.

The Kelly paper may be used as the mathematical implementation reference; do not copy an entire external trajectory-optimization library into this repository.

---

## 20. Result Schema and Compatibility

Keep the public `result` success/failure schema stable.

At minimum preserve:

```matlab
result.Success
result.Message
result.TerminationReason
result.Options
result.azElData
result.originalAzElData
result.obstacleField
result.initialState
result.goalState
result.limits
result.obstacleSafetyMargins_deg
result.selectedCandidateIndex
result.selectedRoute_deg
result.SearchDiagnostics
result.ElapsedPlanningTime_s
result.Validation
```

`selectedRoute_deg` should represent the seed/topology route that initialized the selected optimization, not pretend to be the final trajectory.

The final optimized trajectory must be available directly in the result with:

```matlab
time_s
position_deg
velocity_deg_s
acceleration_deg_s2
jerk_deg_s3
```

If an existing field such as `timedSlopePath` must be retained for compatibility, populate it as a compatibility view of the direct-collocation trajectory and clearly set diagnostics such as:

```matlab
MotionType = "directCollocation";
RetimerType = "none";
```

Do not leave `RetimerType = "coppTopp3Socp"` or other misleading legacy metadata.

If `smoothPath` is retained only for schema compatibility, return a documented empty value rather than inventing a fake fixed path.

---

## 21. Optimization Diagnostics

Add direct-collocation diagnostics to `SearchDiagnostics`.

For every seed record:

```matlab
SeedIndex
SeedSource
SeedSnapshotTime_s
SeedRouteCost_deg
SolverExitFlag
SolverMessage
ObjectiveArrivalTime_s
IntegratedSquaredJerk
MeshPassCount
SegmentCountHistory
NlpIterationCount
NlpFunctionEvaluationCount
MaximumDynamicsDefect
MaximumVelocityViolation_deg_s
MaximumAccelerationViolation_deg_s2
MaximumJerkViolation_deg_s3
MinimumObstacleClearance_deg
CorridorRelinearizationCount
CollisionValidationPassed
ElapsedTime_s
```

At planner level include:

```matlab
CandidateSeedCount
OptimizedCandidateCount
FeasibleCandidateCount
SelectedCandidateIndex
BestValidatedArrival_s
OptimalityProven
OptimalityScope
```

For this nonlinear multi-start implementation:

```matlab
OptimalityProven = false;
```

unless a future algorithm genuinely proves global optimality.

Use wording such as:

> Best independently validated minimum-time solution found among the evaluated topology/time seeds.

Do not claim global optimality merely because all retained seeds were optimized.

---

## 22. Independent Validation

The planner may report `Success = true` only after independent validation.

Validation must check:

- strictly increasing finite time;
- exact initial state within tolerance;
- exact terminal state within tolerance;
- workspace bounds;
- azimuth wrapping policy;
- per-axis velocity limit;
- per-axis acceleration limit;
- per-axis jerk limit;
- complete time-varying obstacle collision freedom;
- canonical safety-margin policy;
- goal-time policy;
- method-consistent interpolation between collocation points.

Use the existing public collision checker for final collision validation.

Do not use the same local separating half-space constraints as the only collision validator.

---

## 23. Prove That the Planner Carries Speed Through Turns

Add a regression specifically for the behavior motivating this rewrite.

Create a static obstacle arrangement that requires approximately a 90-degree change in direction but has enough clearance to round the turn.

Test that:

1. the final path is not forced through the visibility vertex;
2. the optimized path bends continuously;
3. velocity is continuous;
4. acceleration is continuous;
5. jerk respects the configured limit;
6. speed at the point of maximum curvature is nonzero;
7. there is no intermediate mandatory stop;
8. the trajectory is collision-free;
9. arrival is no later than a deliberately stop-at-corner baseline.

Do not assert that speed must always remain nonzero in every scenario. A real minimum-time solution may legitimately slow substantially or stop when geometry, moving obstacles, endpoint conditions, or dynamic limits require it.

The invariant is:

> intermediate stops are optimizer decisions, not artifacts of fixed visibility vertices.

---

## 24. Required Test Matrix

### A. No obstacles

Straight start-to-goal motion.

Verify:

- symmetry when limits are symmetric;
- no unnecessary lateral motion;
- correct endpoint state;
- active velocity/acceleration/jerk constraints where expected;
- minimum-time result converges with mesh refinement.

### B. Single static blocker

Start and goal on opposite sides of a rectangle.

Verify:

- direct seed fails or becomes infeasible;
- at least two topology seeds are available when appropriate;
- optimizer rounds the selected side;
- route vertices are not hard waypoints.

### C. U-shaped obstacle

Use the existing U-shaped scenario.

Verify:

- nonconvex obstacle handling;
- safety margin is applied exactly once;
- selected trajectory exits without collision.

### D. Alternating slalom

Use multiple alternating obstacles.

Verify:

- continuous velocity through multiple turns;
- no stop generated at each graph corner;
- jerk limits remain satisfied.

### E. Moving circle without azimuth wrap

Use `exampleMovingCircleNoAzimuthWrap`.

Verify:

- no ±180-degree seam shortcut;
- trajectory may accelerate, slow, or spatially detour as needed;
- final trajectory respects the non-wrapping interval.

### F. Moving occlusion / moving slot

Use an existing moving-occlusion example or the slot/bar arrangement.

Verify:

- optimizer can trade waiting versus spatial detour;
- final time is optimized;
- departure time is not separately enumerated as a fixed retiming stage.

### G. Nonzero initial motion

Start with nonzero velocity and/or acceleration.

Verify:

- no unsupported artificial initial hold;
- the optimized dynamics directly continue the supplied state.

### H. Infeasible horizon

Make the goal horizon too short.

Verify:

```matlab
Success = false
TerminationReason = "goalTimeInfeasible"
```

or another documented stable reason.

### I. Impossible dynamic limits

Use a case where endpoint state or motion cannot satisfy jerk/acceleration limits.

Verify clean failure and diagnostics.

### J. Mesh convergence

Solve the same representative case with increasing refinement.

Verify arrival time and trajectory metrics converge within documented tolerance.

---

## 25. Update Existing Examples

All maintained examples should continue to follow the repository example template:

```text
create obstacles
    ↓
define initialState
define goalState
define limits
define options
    ↓
planAzElMotion
    ↓
independent validation
    ↓
workspace/search plot
animation
position/velocity/acceleration/jerk plots
```

Update plotting so the actual optimized trajectory is shown.

Where useful, also show the visibility seed as a thin/dashed reference so it is obvious that the optimizer is allowed to leave the graph route.

Do not plot the seed in a way that makes it look like the final path.

Add jerk to the kinematic plots because jerk is now a direct optimization control and hard constraint.

---

## 26. Remove CoPP After Direct-Collocation Validation

Once the new planner passes the required regression matrix:

1. remove the production call to `azElExperimental.retimeCoppSmoothPath`;
2. delete or archive `tests/testCoppRetimer.m` as appropriate;
3. remove CoPP-specific result metadata;
4. remove CoPP-specific options;
5. remove CoPP dependency checks from the public planner;
6. remove `+azElExperimental` if nothing supported remains there;
7. search the repository for:
   - `copp`
   - `TOPP3`
   - `RetimerType`
   - `buildAzElSmoothPath`
   - `retimeCoppSmoothPath`
   - legacy fixed-path timing options;
8. remove dead branches and obsolete comments.

Do not keep both complete planner architectures selectable through an option.

The purpose of this work is to simplify the repository around one maintained planner.

---

## 27. Performance Work Only After Correctness

After the direct-collocation planner is correct:

1. profile candidate optimization;
2. parallelize independent topology seeds;
3. cache obstacle interpolation used repeatedly at fixed collocation times;
4. add sparse analytic Jacobians for the linear dynamics and collocation defects;
5. add analytic gradients for separating obstacle constraints;
6. reduce duplicate seed routes;
7. warm-start mesh refinements;
8. terminate candidates whose lower bound can no longer beat the incumbent.

Do not compromise validation or generality to make one example faster.

---

## 28. Acceptance Criteria

This rewrite is complete only when all of the following are true.

### Architecture

- `planAzElMotion` remains the single public planner.
- The production planner no longer smooths a fixed route and retimes it.
- CoPP/TOPP3 is no longer required.
- Visibility graph output is used only as initialization/topology information.
- Intermediate visibility vertices are not trajectory equality constraints.

### Motion

- Position, velocity, and acceleration are continuous under the selected interpolation.
- Jerk is explicitly bounded per axis.
- The planner can carry nonzero speed through turns.
- It does not stop at every visibility corner.
- Final time is optimized for earliest-arrival mode.

### Obstacles

- Static and moving obstacles are supported.
- Canonical protected obstacle geometry is used directly.
- Safety margin is not re-applied.
- Final collision validation is independent of the NLP's local obstacle approximation.

### Numerical correctness

- Hermite–Simpson defects satisfy the configured tolerance.
- The corrected Hermite–Simpson interpolation equation from the supplied paper erratum is used.
- Mesh refinement is implemented and reported.
- Continuous/interpolated kinematic extrema are validated.

### Diagnostics

- Every candidate optimization has useful diagnostics.
- Expected infeasibility returns a stable failure result.
- `OptimalityProven` is not falsely set to true.
- The reported optimality scope is explicit.

### Examples/tests

- Existing representative examples still run through `planAzElMotion`.
- New speed-through-turn regression passes.
- Moving-obstacle regressions pass.
- Nonzero initial-state regression passes.
- Infeasible cases fail cleanly.
- MATLAB is the reference behavior; Octave may be used only as a secondary numerical smoke test where compatible.

---

## 29. Recommended Implementation Order

Implement in this exact order so failures remain understandable.

### Milestone 1 — Static, obstacle-free collocation

- Implement the six-state/two-control dynamics.
- Implement separated Hermite–Simpson.
- Optimize final time.
- Enforce endpoint and kinematic constraints.
- Validate against no-obstacle cases.

Do not add visibility graphs yet.

### Milestone 2 — Static obstacle, one seed

- Add seed construction.
- Add local smooth obstacle-separating constraints.
- Add independent collision validation.
- Demonstrate one rounded turn with nonzero speed.

### Milestone 3 — Multiple topology seeds

- Connect `buildAzElVisibilityRoutes`.
- Optimize each retained seed independently.
- Select earliest validated result.
- Add parallel candidate execution.

### Milestone 4 — Mesh refinement

- Add defect/constraint-driven `h` refinement.
- Warm-start refined solves.
- Add convergence diagnostics.

### Milestone 5 — Moving obstacles

- Evaluate obstacle geometry at actual collocation time.
- Use snapshot/event routes only as temporal/topological initializations.
- Add corridor relinearization where moving geometry changes active boundary association.

### Milestone 6 — Secondary jerk tie-break

- Preserve the minimum-time solution.
- Add near-equal-time integrated squared jerk minimization.
- Fall back to the stage-1 solution if tie-break optimization fails.

### Milestone 7 — Remove legacy retimer

- Delete CoPP production dependency.
- Remove obsolete smooth-path/retimer code and tests.
- Update examples and documentation.
- Run the full regression matrix.

### Milestone 8 — Reduce the HS-3 engine

- Apply the module boundaries and extraction order in Section 15.
- Reduce `optimizeAzElDirectCollocation.m` below 1,500 lines.
- Reduce the total nonblank HS-3 implementation below 4,200 lines.
- Preserve all public and diagnostic schemas.
- Compare focused runtime before and after every extraction.
- Profile the longest maintained example after the final extraction.

---

## 30. Source Notes

Primary mathematical reference:

- Matthew Kelly, *An Introduction to Trajectory Optimization: How to Do Your Own Direct Collocation*, SIAM Review 59(4), 2017:
  https://epubs.siam.org/doi/epdf/10.1137/16M1062569

Use in particular:

- Section 1.4 — trajectory-optimization problem formulation;
- Section 4 — Hermite–Simpson collocation;
- Section 5.1 — initialization;
- Section 5.2 — mesh refinement;
- Section 5.3 — error analysis;
- Section 5.5 — importance of smooth/consistent NLP functions;
- the correction appended to the supplied PDF for Equation 4.13.

Repository baseline:

- https://github.com/kevvtrinh/AzElObsAvoid/tree/experimental
- https://github.com/kevvtrinh/AzElObsAvoid/blob/experimental/AGENTS.md
- https://github.com/kevvtrinh/AzElObsAvoid/blob/experimental/planAzElMotion.m
- https://github.com/kevvtrinh/AzElObsAvoid/blob/experimental/buildAzElVisibilityRoutes.m
- https://github.com/kevvtrinh/AzElObsAvoid/blob/experimental/%2BazElInternal/evaluateAzElMotionCandidates.m
- https://github.com/kevvtrinh/AzElObsAvoid/blob/experimental/%2BazElExperimental/retimeCoppSmoothPath.m
