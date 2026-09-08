# BMTP refactor: speed up cleanup and remove displaced code

## Implementation record — September 8, 2026

The original checklist below is retained as the reviewed specification. Its dated
implementation record is the [verified results report](docs/bmtp_refactor_results.md).

| Stage | Retained outcome |
| --- | --- |
| 0 | Pinned physical-request comparisons, separate profiles, all geographic cases, and missing feature regressions. |
| 1 | Bounded exact visibility, containment, and static plane verification, with independent prior references. |
| 2 | Immutable static source preparation reused within a request; authoritative public validation rebuilds it. |
| 3 | Existing exact direct law retained and shared; eligible analytic departure replaces repeated waiting trials. |
| 4 | Eligible source-facet corridors compile constraints once; broader unsupported geometry retains its solver. |
| 5 | Eligible limiting-axis clock avoids arrival search; cubic reversal replacement was evaluated and rejected for lack of measured speed benefit. |
| 6 | Positive speed quadrature in eligible convex solves; adaptive final arc and ranking independent of display sampling. |
| 7 | Single public planner, raw diagnostics, shared numerical core, and superseded implementations deleted. |

Final verification is 222 passing tests and 60 passing frozen quality comparisons.
The specified physical size gate passes at 15,088 lines versus 15,528. Executable
lines are 10,949 versus 10,516: report that growth separately from physical shrinkage.
The user's subsequent request for more aggressive production reduction prompted
another 282 executable lines of deletion after API consolidation. Standalone solver
scope remains unchanged while that additional scope choice is pending. Conditional
formulations do not replace unsupported broader cases, and the original bounded
search coverage is not promoted to a global-completeness claim.

## Objective

Improve `bmtp-cleanup-codex` using the successful formulations and implementation techniques from `bmtp-emptycore`. Reduce computation and production code while preserving cleanup's supported technical capabilities.

Replace overlapping implementations rather than adding another permanent solver pipeline. API compatibility and migration wrappers are not goals. Changes to function names or output structures are acceptable when they make the technical implementation clearer.

This plan does not authorize weakening geometry, physical constraints, or independent validation. It describes future implementation; no new performance results are claimed here.

## Reviewed starting revisions

- Implementation base: `bmtp-cleanup-codex`, `c04f3b280725edd30824947f18751afe07a7a0d9`.
- Reference implementation: `bmtp-emptycore`, `26c343b9ab7704d0cd5a3d8f72af058e99c936b1`.
- Cleanup production scope: 108 MATLAB files, 15,528 physical lines.
- Emptycore production scope: 47 MATLAB files, 5,462 physical lines.
- Counts include production packages and the root planner, comments, and blank lines; exclude examples, tests, sandbox interfaces, and reports.

Emptycore reports 64 passing tests and 18 passing historical benchmark gates. These are repository-reported measurements, not a fresh comparison on one machine. Some of its reduction accompanies narrower feature coverage; do not promise equivalent size or speed while retaining cleanup's capabilities.

Before implementation, read the active AGENTS.md and relevant repository instructions. If the branch has advanced, record the actual starting commit and recheck the affected implementations before applying this plan.

## Required technical capabilities

Preserve the behavior actually supported and verified at the cleanup baseline:

- Static and moving obstacles, including protected concave geometry, disconnected components, holes, and time-dependent activity.
- Fixed-arrival and earliest-arrival planning.
- Supported nonzero initial and terminal velocity and acceleration.
- Moving-target interception, including later-time search when an early meeting fails.
- Supported specified-time target derivative matching; do not imply earliest matching is already supported.
- Time-dependent route search and its existing bounded coverage.
- Obstacle-free periodic coordinates for fixed-position goals.
- Explicitly selected stop-at-waypoint fallback where currently supported; never enable it silently.
- Continuous physical-limit and obstacle checks on the returned motion.
- Useful failure evidence and timing measurements.

Preserve successful supported behavior, not every old implementation. A method may be deleted when its technical coverage has a verified replacement. Keep unsupported cases and incomplete searches explicit; neither is proof of physical infeasibility.

## Design rules

1. Keep one authoritative request and protected obstacle representation. Apply margins exactly once.
2. Reuse geometry during computation, but require the independent public validator to establish acceptance from authoritative inputs and the returned polynomial.
3. Use exact source geometry for acceptance. Search proposals do not establish clearance.
4. Keep analytic and reduced optimization methods conditional on mathematical eligibility: endpoint derivatives, timing policy, obstacle motion, and route geometry.
5. Fall through to the broader solver when a specialized formulation is inapplicable or cannot produce valid motion. Record why; do not conceal a heuristic retry or claim infeasibility.
6. Use bounded jerk with continuous position, velocity, and acceleration where supported. Do not impose continuous jerk unintentionally.
7. Explain why each major stage exists with short comments in the function that orchestrates it. Prefer clear names such as `check`, `attempt`, `reason`, `motion`, and `timeBounds`.
8. Avoid splitting simple logic into many one-use wrappers. Keep helpers when they are shared or substantially improve clarity.
9. Every optimization stage must identify which existing computation it replaces and which code can be removed afterward.
10. Keep benchmark inputs and acceptance tolerances unchanged. Do not alter examples to fit a formulation.

## Stage 0 — measure baseline cost and feature coverage

Why: optimize demonstrated costs while retaining capabilities that the historical examples may not exercise.

- [ ] Run the cleanup test suite and all maintained examples at the pinned starting revision.
- [ ] Record existing failures and unsupported cases separately from regressions.
- [ ] Benchmark both pinned revisions on identical physical inputs, explicit timing modes, and explicit per-axis limit vectors.
- [ ] Preserve cleanup's physical limit semantics. Its scalar allocation differs from emptycore; do not transplant emptycore normalization as a speed optimization.
- [ ] Measure full example runtime and internal geometry, route-search, solver, and validation time. Profile separately from timing measurements.
- [ ] Record optimizer calls, attempted routes, region counts, motion spans, active constraint pairs, success, arrival, and continuous motion length.
- [ ] Use the same MATLAB release, toolboxes, machine, warm-up policy, and at least three timed repetitions. Preserve individual runs and medians.
- [ ] Capture every geographic subcase, not only the last returned result.
- [ ] Add missing feature cases before changing production code: nonzero endpoints, later interception, dynamic topology changes, wrapping, and explicitly requested waypoint fallback.

Gate: a baseline report explains where cleanup spends time and which tests establish each retained capability. Use a common integration of polynomial speed when comparing motion lengths across branches.

## Stage 1 — batch exact geometry and plane verification

Why: reduce repeated small operations without changing the planning algorithm.

Reference areas in emptycore:

- `+obstacleAvoidance/+search/createVisibilityGraph.m`
- `trajectory/+bmtpEngine/verifyStaticSeparatingLines.m`
- `trajectory/+bmtpEngine/solveSeparatingLine.m`
- `trajectory/+bmtpEngine/verifySeparatingLine.m`

Work:

- [ ] Identify scalar visibility, containment, and static plane loops in cleanup that account for measured cost.
- [ ] Replace eligible scalar work with batched predicates and sparse triplet assembly.
- [ ] Reuse plane directions only after verifying them against the current curve, obstacle interval, and numerical reserves.
- [ ] Keep the same normal bounds, clearance inequalities, and roundoff treatment.
- [ ] Bound batch memory; large geographic cases must not require an impractical dense all-pairs array.
- [ ] Delete displaced scalar production paths after equivalence is established. Keep an independent reference in tests when useful.

Tests: scalar versus batch decisions and signed gaps; randomized geometry; near-clearance cases; large coordinate offsets; holes; disconnected regions; single-region and empty scenes. Check visibility routes and graph evidence, not only success.

Gate: equivalent decisions within justified floating-point tolerances and a measured runtime benefit on affected cases, with no validation regression.

## Stage 2 — prepare geometry once and reuse it

Why: repeated scene preparation and projection can dominate runtime before optimization begins.

Reference areas:

- Emptycore `prepareObstacles`, `snapshot`, and `createTimeCells`.
- Cleanup `preparePlanningScene`, `prepareStaticSolverGeometry`, and timed solver preparation.

Work:

- [ ] Inventory repeated preparation across direct attempts, route guesses, motion solves, and validation.
- [ ] Share immutable prepared source geometry within one planning request. Separate original geometry, protected geometry, and proposal geometry clearly.
- [ ] Cache only quantities determined by the request. Avoid persistent caches that can reuse stale geometry across calls.
- [ ] Preserve affine obstacle motion and absolute activity intervals where exact. Handle changing topology or incompatible vertex correspondence explicitly rather than forcing an affine representation.
- [ ] Keep independent validation able to rebuild or independently verify the authoritative representation; never trust an optimizer's `Passed` field.
- [ ] Remove duplicate conversion and projection helpers once their callers use the shared preparation.

Gate: translated time origins, changed obstacle histories, modified margins, and repeated calls cannot reuse incorrect geometry. Preparation counts and measured cost decrease on the targeted workloads.

## Stage 3 — replace eligible direct and waiting calculations

Why: exact analytic motion and departure intervals can eliminate repeated numerical solves.

Reference areas:

- Emptycore `createJerkLimitedChord` and `createDelayedChord`.
- Cleanup `tryDirectAndFixedTimeMotions`, `createWaitThenMoveMotion`, and direct-motion helpers.

Work:

- [ ] Inspect cleanup's existing analytic paths first. Retain them when they are already equivalent; do not port another copy.
- [ ] Use exact jerk-limited chord motion for eligible rest-to-rest requests and validate against the complete obstacle history.
- [ ] Use analytic departure scheduling for eligible moving-obstacle chords, deriving forbidden departure intervals from source geometry and timing.
- [ ] Compare scheduled chords with available detours under the actual objective. Do not return a valid waiting motion early unless its optimality or an adequate comparison justifies selection.
- [ ] Preserve nonzero-state and broader dynamic handling through the existing general formulation until replaced.
- [ ] Delete the displaced waiting trial loop only for coverage that the analytic method demonstrably replaces. Centralize eligibility and selection instead of accumulating nested fallback branches.

Tests: obstacle-free timing bound, barrier waiting, opening gate, multiple blocked departure intervals, a detour faster than waiting, nonzero start time, infeasible horizon, and nonzero endpoint requests that must bypass the rest-to-rest method.

Gate: no lost route quality or supported states; fewer solver calls and less repeated wait refinement on eligible cases.

## Stage 4 — compile exact static corridors

Why: static obstacle constraints can be represented once instead of being projected repeatedly for every motion span and solver iteration.

Reference areas:

- Emptycore `createStaticCorridor` and `solveStaticCorridor`.
- Cleanup `solveStaticBmtpTrajectory` and associated static preparation.

Work:

- [ ] Derive corridor boundaries from the selected visibility route and complete protected geometry.
- [ ] Use the corridor formulation only when its geometric eligibility holds. Preserve the general solver for routes that do not fit it.
- [ ] Construct source-facet intervals once and restrict the motion polynomials to those intervals.
- [ ] Assemble the resulting linear constraints without redundant span/region blocks.
- [ ] Independently check the final motion against every applicable authoritative region; a corridor's internal checks are insufficient for acceptance.
- [ ] Remove static assembly helpers superseded by the compiled representation. Avoid keeping two full static pipelines for identical requests.

Tests: monotone detours, dense coastlines, narrow passages, holes, concave obstacles, coordinate exchange and reversal, and a non-monotone route that must use a different formulation.

Gate: equal or better motion quality and lower constraint/preparation cost on eligible scenes; unchanged coverage elsewhere.

## Stage 5 — solve eligible detours on the physical timing bound

Why: many requests can reach the independent-axis timing bound without searching a sequence of larger arrival times.

Reference areas:

- Emptycore `createReachabilityWarmStart`, `createClockGuide`, `solveStaticCorridor`, and `solveCubicTrajectory`.

Work:

- [ ] Derive valid lower bounds for the supported request. Use rest-to-rest bounds only for rest-to-rest endpoints.
- [ ] For an eligible request, fix the limiting coordinate to its analytic clock and optimize the remaining coordinate under exact obstacle constraints.
- [ ] Handle tied limiting axes explicitly; do not assume one free coordinate always exists.
- [ ] Preserve final motion validation before treating a timing-bound candidate as feasible.
- [ ] If the bound attempt fails, retain the broader route and motion solve. Failure of a fixed clock is not proof that no trajectory exists at that arrival time.
- [ ] Evaluate integrated cubic jerk phases for routes that reverse the limiting coordinate, retaining them only if they replace existing costly work with a measured benefit.
- [ ] Remove obsolete arrival refinement and duplicated warm-start logic only after all affected cases are covered.

Tests: direct motion, monotone detour attaining the bound, detour needing extra time, axis reversal, equal axis bounds, unsupported nonzero endpoints taking the general path, and timed moving geometry.

Gate: preserve earliest-arrival quality while reducing solve counts. Any optimality claim must follow from an achieved valid lower bound, not merely solver convergence.

## Stage 6 — optimize actual motion length

Why: control-polygon length can favor a different curve from the shortest executable motion.

- [ ] Identify where cleanup optimizes or ranks candidates using a proxy for returned motion length.
- [ ] Introduce positive-weight quadrature of speed for eligible convex subproblems, following emptycore's formulation where appropriate.
- [ ] Independently measure final polynomial arc length with adaptive integration; quadrature used for optimization is not the acceptance measurement.
- [ ] Keep earliest arrival primary in earliest mode. Do not exchange a later arrival for a shorter path unintentionally.
- [ ] Remove redundant length-refinement passes when the main solve achieves their purpose.

Tests: curved detours where control-polygon and true lengths differ, coarse versus fine output sampling, fixed-arrival routes, and earliest-arrival ties.

Gate: no regression in arrival or measured executable length, with no dependence on display sampling.

## Stage 7 — consolidate and delete displaced code

Why: speed improvements alone will not reduce code if every old implementation remains reachable.

- [ ] Review the planner after each replacement and once again at the end.
- [ ] Remove unreachable solvers, duplicate geometry conversions, obsolete retry loops, duplicate motion assembly, and one-use forwarding wrappers.
- [ ] Keep broader nonzero-state, later-interception, timed-route, and periodic-coordinate capabilities in the smallest clear implementation that passes their tests.
- [ ] Keep optional stop-at-waypoint behavior explicit. Do not preserve its old implementation if a verified shared implementation can replace it.
- [ ] Simplify candidate selection and diagnostics around methods actually attempted.
- [ ] Preserve tests proving technical behavior; replace tests tied only to obsolete internal structure.
- [ ] Update technical documentation with method eligibility and remaining search limitations.

Suggested main flow:

1. Normalize the physical request and check endpoint feasibility.
2. Prepare protected geometry and timing once.
3. Try eligible exact direct or timing-bound motion.
4. Search necessary spatial or timed routes when no accepted bound solution is available.
5. Generate motion using the applicable reduced or general formulation, including departure scheduling where appropriate.
6. Independently validate candidates and compare them under the requested objective.
7. Return motion and compact failure/search evidence.

Do not force all requests through a static corridor, rest-to-rest clock, or initial snapshot. Keep general handling where the technical problem requires it.

Gate: production code is smaller than the measured cleanup baseline, and every retained subsystem has a demonstrated responsibility. Report files and physical lines by subsystem; do not hide code outside the measurement scope. No arbitrary emptycore-sized target is required.

## Verification and rollback

Each stage is a separate reviewable change. First run the tests that address the changed computation, then the full baseline feature suite and maintained examples before retaining it.

| Check | Required outcome |
| --- | --- |
| Independent validation | Every successful motion passes continuous endpoint, dynamics, workspace, derivative, and obstacle checks. |
| Feature preservation | All successful supported baseline cases remain successful; explicit fallback behavior remains explicit. |
| Motion quality | No arrival or independently integrated length regression beyond established numerical tolerances under the same objective. |
| Runtime | Demonstrated improvement on targeted bottlenecks; no unexplained slowdown elsewhere. Compare matched runs and medians. |
| Geometry | No reduced protected coverage, lost holes, altered activity intervals, or double margins. |
| Search claims | Incomplete search and formulation failure are never reported as a proof of global infeasibility or optimality. |
| Code reduction | End-state production code is smaller; displaced work is removed rather than retained as another permanent pipeline. |
| Generality | Every new formulation passes its motivating case and at least one structurally different regression case. |

A stage may temporarily add code while replacing an implementation, but must identify the deletion required before it is complete. Do not delete broader functionality merely to satisfy a line count.

If correctness or quality regresses, revert the responsible stage. For a runtime regression, perform one matched rerun to distinguish noise from a real change, then diagnose or revert; do not rerun repeatedly until a favorable number appears. Preserve the input, measurements, and reason for rejection outside production code. Any proposed tradeoff requires an explicit decision rather than weakening gates silently.

## Completion

The refactor is complete when cleanup retains its demonstrated technical coverage, runs faster on measured bottlenecks, has less production code, and explains failures clearly. Report the before/after runtime and motion metrics by case, code size by subsystem, removed implementations, and remaining limitations. Do not claim universal speedup or global optimality from a finite benchmark suite.

## Sources

- [Reviewed cleanup revision](https://github.com/kevvtrinh/AzElObsAvoid/tree/c04f3b280725edd30824947f18751afe07a7a0d9)
- [Reviewed emptycore revision](https://github.com/kevvtrinh/AzElObsAvoid/tree/26c343b9ab7704d0cd5a3d8f72af058e99c936b1)
- [Emptycore benchmark report and limitations](https://github.com/kevvtrinh/AzElObsAvoid/blob/26c343b9ab7704d0cd5a3d8f72af058e99c936b1/README.md)
