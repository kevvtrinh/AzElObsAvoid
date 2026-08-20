# Plan 325 Cleanup Audit: Unused, Redundant, and Inefficient Code

**Repository:** `kevvtrinh/AzElObsAvoid`
**Branch audited:** `plan-325`
**Audit date:** 2026-08-20
**Scope:** source-level review for code that is unnecessary, unused, redundant, duplicative, or inefficient. This is **not** a MATLAB Profiler run, so performance findings are ranked by code-path evidence rather than measured wall-clock percentages.

## Executive Summary

The branch is not dominated by obvious dead code. Most cleanup value is concentrated in three areas:

1. **Stale or disconnected code/API:** `RandomSeed` and `certifySeedCorridor`.
2. **Repeated work:** obstacle normalization/canonicalization is repeated at construction time and again throughout internal planner calls.
3. **Hot-loop geometry work:** topology search and HS3 repeatedly rebuild/query geometry or repeatedly construct linear maps that can be prepared once and reused.

The highest-value changes are therefore **not** broad file deletion. They are targeted removal of stale surfaces, separating public validation wrappers from trusted internal kernels, and preparing reusable geometry/query data once per planning call.

### Recommended order

| Priority | Finding | Action |
|---|---|---|
| P0 | CLN-001 | Remove unused `RandomSeed` option/result field |
| P0 | CLN-003 | Eliminate repeated normalization in `makeAzElObstacleData` → `inflateAzElObstacleData` |
| P0 | CLN-004 | Stop re-canonicalizing obstacles inside trusted internal planner functions |
| P0 | CLN-005 | Precompute obstacle broad-phase bounds instead of rescanning complete histories per occupancy query |
| P1 | CLN-002 | Remove or deliberately integrate `certifySeedCorridor` |
| P1 | CLN-006 | Replace repeated HS3 basis-polynomial reconstruction with a reusable linear map |
| P1 | CLN-007 | Consolidate duplicate polynomial-sampling helpers |
| P1 | CLN-008 | Consolidate duplicate boundary/edge projection primitives |
| P1 | CLN-009 | Factor duplicate protected/original normalization loops |
| P1 | CLN-010 | Batch/cache time-expanded visibility collision queries |
| P2 | CLN-011 | Preallocate dynamically grown arrays in search/validation paths |
| P2 | CLN-012 | Reduce repeated geometry reconstruction in moving HS3 corridor constraints |
| P2 | CLN-013 | Preallocate/vectorize continuous Bernstein inequality assembly |
| P2 | CLN-014 | Consider shrinking the moving-target intercept wrapper |
| Profile first | CLN-015 | Add broad-phase pruning to all-pairs visibility construction if profiling shows it dominates |

---

# A. Safe or Nearly-Safe Removal Candidates

## CLN-001 — `RandomSeed` is an unused planner option

**Category:** unused / stale API
**Confidence:** High
**Priority:** P0

### Evidence

`planAzElMotion.m`:

- defines `RandomSeed = 0` in planner defaults;
- validates it as a nonnegative integer;
- copies it into the result as `result.RandomSeed`.

However, the actual deterministic seed generator and HS3 solver do not consume `RandomSeed`. No stochastic generator state is initialized from it.

Relevant files:

- `planAzElMotion.m`
- `+azElInternal/generateAzElTopologySeeds.m`
- `+azElInternal/solveAzElHs3.m`

### Why it is unnecessary

The option currently implies that changing it can alter planning behavior, but it cannot. It is therefore dead configuration surface plus a dead result field.

### Recommendation

Remove:

- `RandomSeed` from `plannerDefaults`;
- its validation;
- `result.RandomSeed`;
- any documentation/example references that present it as a planner control.

Only restore it if randomized seed generation is actually introduced later.

### Risk

Low, except for external callers that may inspect the field. If backward compatibility matters, deprecate it for one release before removal.

---

## CLN-002 — `certifySeedCorridor.m` is disconnected from the production planner

**Category:** unused production helper / redundant certificate path
**Confidence:** High
**Priority:** P1

### Evidence

`+azElInternal/certifySeedCorridor.m` independently combines:

- seed-envelope containment checks;
- seed-corridor polynomial inequalities.

The production planner does not call this helper. The direct call found in the branch is in `tests/testHs3Planner.m`, where it is unit-tested in isolation.

The real planning path instead uses:

- `buildSeedCorridor`;
- `seedCorridorInequality` during optimization;
- the separate public `validateAzElTrajectory` acceptance step.

### Why it is unnecessary

At present it is production code whose contract is not part of production acceptance. That creates two possible interpretations of what a “seed corridor certificate” means without the planner choosing one.

### Recommendation

Choose one:

**Preferred if the certificate is not required:**

- delete `+azElInternal/certifySeedCorridor.m`;
- replace its focused test with tests of `seedEnvelopeContainsObstacles` and `seedCorridorInequality` directly.

**If it is intended to be a real planner invariant:**

- integrate it into the candidate acceptance path and document exactly what failure means.

Do not leave it test-only in the production package.

### Risk

Medium. The helper may represent an intended future safety contract, so confirm intent before deleting it.

---

# B. Redundant Work and Duplicate Logic

## CLN-003 — Obstacle construction normalizes the same data multiple times

**Category:** redundant computation
**Confidence:** High
**Priority:** P0

### Evidence

The static construction path is approximately:

`makeAzElObstacleData`
→ `normalizeAzElTimeObstacleData`
→ `inflateAzElObstacleData`
→ `combineAzElObstacles`
→ `normalizeAzElTimeObstacleData`
→ rebuild protected slices
→ `normalizeAzElTimeObstacleData` again for each returned obstacle.

This path is used even when `safetyMargin_deg == 0`. The slice-level inflation helper exits quickly for zero margin, but the outer normalization/copying work has already occurred.

Relevant files:

- `makeAzElObstacleData.m`
- `inflateAzElObstacleData.m`
- `combineAzElObstacles.m`
- `normalizeAzElTimeObstacleData.m`

### Why it is inefficient

The constructor itself created the canonical schema, so immediately sending it through a public “accept arbitrary container and fully revalidate” path does redundant field validation, cell reshaping, topology checks, and structure copying.

For dense histories this cost scales with every vertex and time slice.

### Recommendation

Separate public wrappers from trusted internal kernels, for example:

- `inflateAzElObstacleData(...)` remains the safe public entry point;
- internal `inflateCanonicalAzElObstacles(...)` assumes normalized canonical input;
- `makeAzElObstacleData` calls the internal kernel after its single normalization;
- add a direct zero-margin path that sets protected geometry equal to original geometry without another full canonicalization pass.

### Risk

Low to medium. Preserve one authoritative validation pass and keep tests proving the trusted internal path is only called with canonical data.

---

## CLN-004 — Canonical obstacles are repeatedly re-canonicalized inside the planner

**Category:** redundant validation / hot-path overhead
**Confidence:** High
**Priority:** P0

### Evidence

`planAzElMotion` already calls `combineAzElObstacles` once before planning.

The already-canonical obstacle array is then passed to internal code that calls `combineAzElObstacles` again:

- `+azElInternal/buildAzElStopWaypointMotion.m` inside `validateInputs`;
- `+azElInternal/solveAzElHs3.m` at solver setup;
- `validateAzElTrajectory.m` before validation;
- `queryAzElTimeObstacle.m` before every query.

The planner calls the analytic builder once per seed and can call HS3 repeatedly for multiple seeds, relinearization, and mesh refinement.

### Why it is inefficient

`combineAzElObstacles` is not just a cheap type assertion. It flattens containers and calls `normalizeAzElTimeObstacleData` for every obstacle. Repeating it after the public planner has already established canonical data multiplies work with the number of seed attempts and collision queries.

### Recommendation

Introduce a clear boundary:

- **public functions** accept arbitrary supported obstacle containers and normalize once;
- **internal functions** accept a canonical obstacle array and use a lightweight assertion only in debug/test mode.

A good pattern would be:

- `queryAzElTimeObstacle` public wrapper;
- `azElInternal.queryCanonicalObstacles` trusted kernel;
- analogous trusted paths for the analytic builder, HS3, and validator when called from `planAzElMotion`.

Keep `validateAzElTrajectory` independently authoritative when called by users; only avoid the extra canonicalization when the caller can prove the data came directly from the planner.

### Risk

Medium. This is a boundary-design change, so it needs contract tests around malformed public inputs.

---

## CLN-005 — Occupancy queries recompute full-history obstacle bounds every call

**Category:** avoidable repeated scanning
**Confidence:** High
**Priority:** P0

### Evidence

In `queryAzElTimeObstacle.m`, the occupancy-only fast path calls `obstacleHistoryBounds(obstacles)` on every function invocation.

`obstacleHistoryBounds` loops through every obstacle and every stored time slice and scans finite vertices to compute min/max azimuth/elevation.

The topology generator then repeatedly calls `queryAzElTimeObstacle`:

- once per time layer to classify all graph nodes;
- for every candidate moving graph edge through `motionEdgeIsClear`.

### Why it is inefficient

The complete-history bounding boxes are invariant for a fixed canonical obstacle input. Recomputing them for each occupancy query turns a broad-phase optimization into its own repeated full-history cost.

### Recommendation

Prepare an obstacle-query context once per planner invocation containing at least:

- canonical obstacle records;
- complete-history AABBs;
- per-slice AABBs if useful;
- optionally prebuilt static polyshapes / edge arrays.

Then pass that context to internal occupancy kernels.

### Risk

Low if the context is immutable and rebuilt whenever obstacle data changes.

---

## CLN-006 — HS3 rebuilds the jerk-to-state basis map one basis vector at a time

**Category:** inefficient linear algebra setup
**Confidence:** High
**Priority:** P1

### Evidence

`+azElInternal/solveAzElHs3.m`, inside `initialJerkGuess`, creates the linear map from control jerk values to sampled position/velocity/terminal derivatives by looping over every control index.

For each control basis vector it:

1. allocates a zero jerk-control matrix;
2. sets one control to one;
3. reconstructs the entire polynomial chain;
4. evaluates that polynomial at all evaluation times;
5. subtracts the zero-jerk baseline to recover one column of the map.

It then solves the resulting regularized least-squares system.

### Why it is inefficient

The HS3 state chain is linear in jerk control for a fixed segment count and duration. Building the map through repeated full trajectory reconstructions is equivalent to numerically assembling a matrix whose coefficients can be generated directly.

This setup is repeated for every HS3 solve, including relinearization and mesh-refinement solves.

### Recommendation

Derive/build a direct transition matrix for:

`control jerk → position, velocity, acceleration`

for a given `(segmentCount, duration)`.

At minimum:

- generate the scalar-axis map once;
- reuse it for azimuth and elevation;
- avoid allocating a two-axis basis trajectory for every control;
- cache reusable normalized-time matrices by `segmentCount`, applying duration scaling analytically.

### Risk

Medium. Add a numerical equivalence test against the current basis-reconstruction implementation before replacing it.

---

## CLN-007 — `samplePolynomial` is duplicated in both motion constructors

**Category:** duplicate code
**Confidence:** High
**Priority:** P1

### Evidence

Nearly identical local `samplePolynomial` functions appear in:

- `+azElInternal/buildAzElStopWaypointMotion.m`;
- `+azElInternal/solveAzElHs3.m`.

Both:

- build a uniform sample-time grid;
- append segment knot times and final time;
- call `azElInternal.evaluateAzElPolynomial`.

### Recommendation

Create one internal helper such as:

`azElInternal.sampleAzElPolynomial(polynomial, sampleTime_s)`

and call it from both constructors.

### Why it matters

This is not a major performance issue, but it is an easy reduction in redundant logic and prevents sampling behavior from drifting between analytic and HS3 trajectories.

### Risk

Low.

---

## CLN-008 — Boundary parsing and nearest-edge projection are reimplemented several times

**Category:** duplicated geometry primitives
**Confidence:** High
**Priority:** P1

### Evidence

Very similar logic appears in:

- `+azElInternal/pointPolygonClearance.m`;
- `+azElInternal/buildSeedCorridor.m` (`nearestBoundaryPoint`);
- `+azElInternal/solveAzElHs3.m` (`geometryEdges`, `nearestEdge`);
- `+azElInternal/generateAzElTopologySeeds.m` (`boundaryEdges` plus edge-intersection preparation).

Repeated operations include:

- identifying finite runs in NaN-separated boundaries;
- closing polygon rings;
- generating edge start/end arrays;
- projecting a point onto each edge;
- clamping projection fraction to `[0,1]`;
- selecting the minimum-distance edge;
- handling degenerate zero-length edges.

### Why it is redundant

The mathematical primitive is shared, but tolerance and degenerate-edge behavior are maintained independently. A future geometry fix can therefore repair one path while leaving another subtly different.

### Recommendation

Create low-level internal geometry primitives, for example:

- `boundaryToEdges(position_deg)`;
- `nearestPointOnEdges(point_deg, edgeStart_deg, edgeEnd_deg)`.

Keep higher-level semantics (`signed clearance`, `visibility rejection`, `corridor normal selection`) in their current callers.

### Risk

Low to medium. Preserve deterministic edge-ordering because some diagnostics/tests may rely on edge indices.

---

## CLN-009 — Protected and original obstacle histories use duplicated normalization loops

**Category:** duplicate validation logic
**Confidence:** High
**Priority:** P1

### Evidence

`normalizeAzElTimeObstacleData.m` separately loops over:

1. protected `az_deg` / `el_deg` slices;
2. original `originalAz_deg` / `originalEl_deg` slices.

Both loops perform the same basic work:

- numeric/vector validation;
- az/el length matching;
- column conversion to `double`;
- `normalizeBoundarySliceTopology`;
- removed-region accounting.

### Recommendation

Factor one local helper such as:

`normalizeBoundaryHistory(azimuthSlices, elevationSlices, sampleCount, role)`

that returns normalized slices and removal metadata.

Use `role = "protected"` or `"original"` only for error identifiers/messages.

### Risk

Low.

---

## CLN-010 — Time-expanded visibility repeatedly invokes a heavy occupancy API per edge

**Category:** inefficient graph collision checking
**Confidence:** High
**Priority:** P1

### Evidence

`generateAzElTopologySeeds.m`:

- computes node occupancy one layer at a time through `queryAzElTimeObstacle`;
- checks each moving transition with `motionEdgeIsClear`;
- `motionEdgeIsClear` samples **13** points along each space-time edge and invokes `queryAzElTimeObstacle` again.

Because the public query function also canonicalizes obstacles and recomputes whole-history bounds, the cost compounds with graph expansion.

### Recommendation

After CLN-004/005, expose a prepared internal occupancy kernel and then:

- batch all transitions for a layer when practical;
- reuse obstacle geometry prepared at shared query times;
- broad-phase reject an entire edge using its swept az/el bounding box before 13 point tests;
- retain the deterministic 13-point policy if it is part of seed-generation behavior.

### Risk

Medium. Seed collision checks are conservative heuristics, so preserve current accepted/rejected transition tests during optimization.

---

# C. Lower-Priority Performance Cleanup

## CLN-011 — Repeated dynamic array growth remains in search and validation loops

**Category:** MATLAB allocation inefficiency
**Confidence:** High
**Priority:** P2

### Evidence

Examples include:

`+azElInternal/generateAzElTopologySeeds.m`

- `seeds(end + 1, ...)`;
- repeated concatenation of sampled obstacle vertices;
- repeated extension of selected indices;
- repeated concatenation of boundary edges;
- `exploredNodes_deg(end + 1, :)`;
- path reconstruction by repeatedly prepending parent nodes;
- homology result arrays grown with `end + 1`;
- obstacle sample times repeatedly concatenated.

`validateAzElTrajectory.m`

- appends obstacle split times;
- grows adaptive interval stacks;
- grows issue strings.

`+azElInternal/solveAzElHs3.m`

- appends corridor associations;
- concatenates edge arrays.

### Recommendation

Prioritize the loops whose maximum sizes are known:

- preallocate path arrays to layer/state count and fill backward;
- preallocate corridor records to `controlCount * obstacleCount` and trim;
- collect variable-size boundary rings in cells, then `vertcat` once;
- collect obstacle times in cells, then concatenate once;
- use explicit stack capacity for adaptive validation.

Do not spend effort on tiny issue-string arrays unless profiling says otherwise.

### Risk

Low.

---

## CLN-012 — Moving HS3 corridor constraints repeatedly reconstruct geometry for the same associations

**Category:** repeated geometry work inside nonlinear constraints
**Confidence:** Medium
**Priority:** P2

### Evidence

For non-fixed moving associations, `corridorConstraints` repeatedly calls `obstacleShapeAtTime(..., true)` as `fmincon` evaluates nonlinear constraints. It then rebuilds finite-vertex arrays and, for edge-based associations, reconstructs edge arrays again.

For fixed/static associations the implementation already stores fixed normals and boundary offsets and therefore avoids this work. The moving case has no equivalent prepared representation.

### Recommendation

Prepare as much invariant association data as possible:

- ring/edge indexing topology;
- source-slice edge arrays;
- interpolation indices where final time is fixed;
- support-normal metadata.

For `fixedArrival`, control times are fixed during a solve, so interpolated geometry at those control times can potentially be prepared once before `fmincon` when the frozen-association model permits it.

For `earliestArrival`, final time changes, so cache only invariant topology/data and continue updating time-dependent coordinates.

### Risk

Medium to high. Do not cache geometry that actually depends on the optimization decision.

---

## CLN-013 — Continuous Bernstein constraints grow the inequality vector incrementally

**Category:** repeated allocation in nonlinear constraint callback
**Confidence:** High
**Priority:** P2

### Evidence

`continuousBoundConstraints` loops over every segment and both axes and repeatedly calls `appendBounds`. `appendBounds` concatenates new upper/lower Bernstein inequalities onto the existing vector.

This occurs inside the nonlinear constraint function, so it is repeated many times by `fmincon`.

`powerToBernstein.m` itself is already sensibly optimized with a persistent conversion matrix by polynomial degree; that part should be kept.

### Recommendation

Precompute the exact number of inequalities and fill a preallocated vector by index, or vectorize conversion across segments of the same polynomial degree.

The useful existing `powerToBernstein` matrix cache should remain.

### Risk

Low.

---

## CLN-014 — `planAzElMovingTargetIntercept` duplicates part of `planAzElMotion`'s moving-goal contract

**Category:** redundant API/validation layer
**Confidence:** Medium
**Priority:** P2

### Evidence

`planAzElMovingTargetIntercept.m` explicitly describes itself as an adapter to `planAzElMotion`. It independently validates:

- sampled target times;
- target positions;
- interpolation method;
- intercept-time range;

then builds a `goalState` containing `targetTime_s`, `targetPosition_deg`, and `InterpolationMethod` and calls `planAzElMotion`.

`planAzElMotion` already accepts and validates a sampled moving-goal history.

The wrapper does still provide unique convenience behavior:

- `InterceptMode` translation;
- optional target velocity/acceleration matching;
- numerical target-derivative estimation;
- compact `Intercept` metadata.

### Recommendation

Do **not** delete it blindly. Consider shrinking it into a thin adapter:

- move moving-target-history normalization to one shared helper used by both APIs;
- keep only intercept-specific policy and derivative estimation in the wrapper;
- alternatively fold these convenience options into `planAzElMotion` if the goal remains one public planner API.

### Risk

Medium because callers may rely on the convenience API.

---

## CLN-015 — Static visibility construction is bounded but still all-pairs against all boundary edges

**Category:** algorithmic hotspot
**Confidence:** High about complexity; unknown about measured impact
**Priority:** Profile first

### Evidence

The visibility graph builder considers pairs of candidate nodes and calls `segmentIsVisible`. Visibility checking evaluates segment/boundary intersections against the obstacle boundary-edge set.

The implementation deliberately caps candidate counts and has a work budget, which prevents unbounded growth, so this is **not** a correctness problem or obvious overengineering.

### Why it may still matter

For dense obstacles, cost is approximately proportional to:

`candidate-pair count × protected-boundary edge count`.

Even after vertex reduction, dense geographic examples can make this one of the largest seed-generation costs.

### Recommendation

Only optimize after profiling. Candidate improvements:

- segment AABB versus edge AABB broad-phase rejection;
- spatial bins / bounding-volume hierarchy for boundary edges;
- skip pairs that cannot improve the current graph under a conservative distance test;
- cache edge geometry for identical static shapes.

Do not replace deterministic visibility behavior merely to reduce code.

### Risk

Medium. Optimization must preserve exact visibility rejection semantics.

---

# D. Code That Looks Redundant but Should Not Be Removed

## Keep `validateAzElTrajectory` independent

The planner's optimizer constraints and the final validator overlap conceptually, but that is desirable. `validateAzElTrajectory` is an independent acceptance gate. It should not simply reuse the exact same collision/constraint code used to construct the trajectory, because that would weaken its value as an independent check.

The cleanup opportunity is to avoid unnecessary **input re-normalization when called internally**, not to eliminate independent validation mathematics.

## Keep `powerToBernstein`'s persistent matrix cache

`powerToBernstein.m` already caches the degree-specific basis conversion matrix. The optimization opportunity is in batching/preallocating caller-side constraint assembly, not deleting this helper.

## Keep the occupancy-only fast path

`queryAzElTimeObstacle` has a specialized occupancy-only path that avoids constructing detailed clearance outputs. That separation is useful. The issue is that the path currently rebuilds invariant bounds and passes through public canonicalization every time.

## Keep `makeMovingAzElObstacleData` delegating to `makeAzElObstacleData`

The moving-obstacle builder generates slice histories and delegates canonical protected-history construction to `makeAzElObstacleData`. That is good reuse. The redundant work is deeper in the `makeAzElObstacleData`/inflation normalization chain, not in this delegation itself.

## Keep example-private geographic helpers

The private U.S./geographic-region helpers are actually used by maintained examples and keep shapefile loading/union logic out of scenario scripts. They are not dead code merely because they are under `examples/private`.

---

# E. Suggested Cleanup Architecture

The single highest-leverage structural change would be to distinguish **public safe wrappers** from **trusted prepared internal data**.

## 1. Prepare obstacles once

Create a planning-local immutable structure conceptually like:

```text
PreparedObstacles
  Canonical              canonical obstacle array
  HistoryBounds_deg      one AABB per complete history
  SliceBounds_deg        optional AABB per source slice
  StaticShapes           optional cached polyshape for static slices
  StaticEdges            optional cached deterministic edge arrays
  TimeMetadata           source time arrays / topology metadata
```

The exact schema does not matter as much as the rule:

> arbitrary user data is validated once; internal hot loops do not keep rebuilding the same representation.

## 2. Separate wrapper and kernel functions

Examples:

```text
queryAzElTimeObstacle             public normalization + argument broadcasting
+azElInternal/queryPrepared...    repeated internal occupancy work

inflateAzElObstacleData           public arbitrary-container validation
+azElInternal/inflateCanonical... trusted canonical inflation

validateAzElTrajectory            public independent validator
+ internal call mode              skips only redundant canonicalization,
                                  not independent validation logic
```

## 3. Centralize shared geometry primitives

Create a very small set of primitives for:

- NaN-separated rings → deterministic edges;
- point → nearest edge projection;
- edge AABBs / segment broad-phase tests.

Then let topology, corridor, and clearance code apply their own higher-level semantics.

## 4. Precompute HS3 linear structure

Create reusable normalized-time operators for:

- jerk controls → acceleration;
- jerk controls → velocity;
- jerk controls → position;
- terminal state maps;
- Bernstein bound transformation/index layout.

Duration scaling can then be applied analytically rather than reconstructing every basis trajectory.

---

# F. Minimal Cleanup Plan

If the goal is to reduce risk and get immediate value, I would implement only these first:

1. Remove `RandomSeed`.
2. Decide whether `certifySeedCorridor` is real production behavior; remove it if not.
3. Give `makeAzElObstacleData` a canonical internal inflation path and a cheap zero-margin path.
4. Stop internal planner functions from repeatedly calling `combineAzElObstacles` on data normalized by `planAzElMotion`.
5. Precompute obstacle history bounds once and use a prepared internal occupancy query.
6. Replace the two local `samplePolynomial` implementations with one helper.
7. Centralize edge extraction and nearest-edge projection.
8. Refactor the duplicate protected/original normalization loops.
9. Replace the HS3 initial-guess basis reconstruction loop with an equivalent matrix operator.
10. Run MATLAB Profiler on the dense moving-obstacle and geographic examples before changing visibility-graph algorithms further.

This sequence removes genuinely stale code first, then removes repeated work without changing the planning method, and leaves algorithmic changes until after profiling.

---

# G. Verification Checklist After Cleanup

- [ ] All existing tests pass.
- [ ] Maintained examples preserve success/failure expectations.
- [ ] Seed count, source labels, and selected topology remain deterministic.
- [ ] Static and moving obstacle queries return identical occupancy results before/after prepared-query refactor.
- [ ] `makeAzElObstacleData(..., safetyMargin_deg=0)` returns byte/field-equivalent geometry where practical.
- [ ] Nonzero safety margins remain absolute rather than cumulative.
- [ ] HS3 initial jerk guess is numerically equivalent to the current basis-reconstruction method within a tight tolerance.
- [ ] Final independent validation remains separate from optimizer constraints.
- [ ] Moving-target fixed-arrival and earliest-arrival examples remain unchanged in behavior.
- [ ] Dense moving-obstacle examples are profiled before and after the query/canonicalization changes.
