# Obstacle avoidance documentation outline

Draft updated for the execution/data-flow cleanup based on `3877017`. This is the proposed structure for a detailed engineering
manual, not a completed algorithm specification.

Audience: a junior engineer who knows MATLAB and basic motion planning.
Explain each concept in plain language before introducing its code name.
Keep the main explanation in execution order; put exhaustive field and function
references in appendices. Describe current behavior, not the development history.

## 1. What the planner does

- Problem statement: move an Az/El point from an initial state to a goal while
  avoiding protected obstacles and respecting workspace and derivative limits.
- Distinguish a geometric path, a timed trajectory, and a sampled plot of motion.
- Static, moving, rotating, and deforming obstacle examples.
- Earliest arrival versus shortest travel at a fixed arrival.
- What is guaranteed by successful validation; what bounded search does not prove.
- One small illustrated request used throughout the manual.

## 2. Public interface and input contract

- `planTrajectory(obstacles, initialState, goalState, limits, options)` and
  optional `[result, diagnosis]`; zero-input defaults request.
- Required/optional state fields, shapes, units, absolute time, and supported
  endpoint derivatives. Explain fixed-goal versus moving-goal representations.
- Workspace, velocity, acceleration, and jerk limits; finite/unbounded cases.
- Omitted, empty, partial, and unknown options; where normalization belongs.
- Azimuth wrapping: coordinate selection, validation, and display behavior.
- Input errors versus expected planning failures.
- Tables of every public field with type, dimensions, default, and example.
- Owners: `+input`, `planTrajectory`, and `planMovingTargetIntercept`.

## 3. Complete execution map

- Flowchart from input normalization through output assembly, with every early
  return, branch condition, validation gate, and recovery loop.
- Ownership table: raw obstacles → protected histories → prepared histories →
  route-search outline → path guesses → candidate motion → validated result.
- Which objects are reused; request-local static solver regions and moving-history
  enclosures; which geometry is constructed later and why.
- `createEmptyPathGuess` defines a record; `createPathGuesses` builds the initial
  collection; `createRoutePathGuesses` converts searched and recovery routes.
- Explain why scene preparation and route-search geometry are different tasks.
- Explain why an exact direct attempt and a direct optimizer guess both exist.
- Show precisely when route search is skipped and when later seeds are attempted.

## 4. Constructing and protecting obstacles

- `createObstacle`: accepted forms, canonical rebuild, boundary cleanup,
  disconnected regions, holes, and nonfinite ring separators.
- Safety-margin construction and provenance; applying the margin exactly once.
- `createMovingObstacle`: source-slice creation, motion-history inputs, and
  returned construction diagnostics.
- `combineObstacles`: nested inputs, flattening, and empty obstacle fields.
- Serial and background processing of dense histories: activation threshold,
  worker availability, fallback, and preservation of protected geometry.
- Source geometry versus protected geometry, with diagrams.
- Link the authoritative `obstacle_history_contract.md` rather than duplicate
  conflicting definitions of interpolation or activity.

## 5. Preparing and evaluating obstacle histories

- `prepareObstacles` and `prepareOneObstacle`: one source-snapshot owner, rebuilding
  only stale obstacles, cache validity,
  per-sample bounds/edges/shapes, interval models, and speed bounds.
- Ring correspondence, representation-only alignment, convexity checks,
  equivalent/nested shapes, and conservative interval enclosures.
- Exact sample times, between-sample interpolation, and activity outside history.
- Why sampled rotation uses corresponding-vertex motion rather than exact arcs.
- Public `shapeAtTime` and occupancy queries versus prepared internal versions.
- Boundary occupancy, clearance tolerance, query broadcasting, blocking-obstacle
  identity, and batching of repeated geometry evaluations.
- `preparePlanningScene` and `queryStaticHorizon`: classify the entire requested
  horizon, including mixed static/moving scenes and activity changes.

## 6. Endpoint feasibility and direct motion attempts

- Endpoint workspace, collision, derivative, and timing checks.
- `tryDirectAndFixedTimeMotions`: eligibility, attempt order, early acceptance conditions,
  retained candidates, and reasons an attempt can be skipped or rejected.
- Direct physical-time motion: duration calculation, coordinate synchronization,
  and the trajectory-engine interface.
- `tryFixedTimeDetour`: sideways offsets without extending the
  retained clock; basis/axis choices, peak-time candidates, amplitude search,
  inexpensive collision screening, independent checks, and travel refinement.
- Separate analytical motion construction from heuristic candidate enumeration.
- Worked examples: clear direct move, blocked direct move with valid detour,
  fixed-clock failure, and valid candidate retained for later comparison.

## 7. Building geometry for route search

- `createRouteSearchGeometry`: endpoints and wrapped goal coordinate.
- Sample-time construction: source times, interval midpoints, endpoints, and
  nine uniform request times; clipping and deduplication.
- Sampled obstacle union versus `denseSweptEnvelope`.
- Current 10,000 estimated-vertex-work threshold, estimation formula,
  endpoint-covering rejection, and resulting representation diagnostics.
- Why conservative enclosures can hide usable routes, and why a sampled union
  alone cannot certify the final moving-obstacle trajectory.
- Boundary-to-edge conversion and reuse during visibility checks.
- Distinguish this search outline from the static projection used by motion solving.

## 8. Connecting possible path points

- `createVisibilityGraph` and `createVisibilityAttempt` in execution order.
- Boundary offsets: initial scale, current 1e-3 deg minimum, floating-point
  reserve, fourfold retries, and workspace-sized stopping bound.
- Current 1e6 visibility-work budget: how it limits nodes and pair checks.
- Node selection: endpoint access, global supports, boundary coverage,
  workspace candidates, deduplication, and clipping.
- Candidate edges: Delaunay selection, boundary connections, and affordable
  all-pairs recovery; why disconnected attempts trigger retries.
- `checkVisibilitySegments`: interior crossings, boundary crossings, collinear
  overlap, numerical edge cases, and distinction from timed collision checking.
- Connected components, accepted/rejected edges, and attempt diagnostics.

## 9. Finding different spatial paths

- `searchDistinctSpatialRoutes`: state representation, distance cost, parents,
  expansion order, goal detection, and stopping conditions.
- Explain route classes as different ways around obstacles; then introduce
  reference points, accumulated angles, and winding signatures.
- Duplicate prevention, available class count, and seed-slot allocation.
- Visibility-based path shortening and preservation of the intended route.
- Multi-winding paths: generation, retention, and failure-only motion attempts.
- Limits of a finite graph and bounded path portfolio.

## 10. Searching paths through moving obstacles

- `timeExpandedVisibilitySearch`: node-plus-time states and parent reconstruction.
- Complete input-derived time layers; explain why the timed-motion segment cap
  does not thin the search-time list.
- Velocity-feasible transitions, waiting transitions, and spatial travel cost.
- Current 13-point edge screening, batching, and missed-between-sample risks.
- Safe-wait-connected arrival intervals and the condition allowing a later
  arrival to be discarded without losing the earlier state's waiting ability.
- Cost pruning, frontier updates, tie handling, goal-layer handling, and the
  distinction between full-horizon processing and early exit.
- Why moving-obstacle feasibility cannot generally be assumed monotone in time.
- Deferred timed search for dense histories; resume conditions and retained work.
- Best partial route, rejection counts, coverage, and unresolved outcomes.

## 11. Turning paths into solver guesses

- `createEmptyPathGuess` and `createPathGuesses`: complete seed schema and provenance.
- Required direct guess, timed path, and distinct spatial paths; ordering and
  deduplication where implemented.
- Path-length parameterization, normalized time, waits, and duration estimates.
- Why a lower-bound duration is not a hard deadline or a proof of feasibility.
- Default two primary seeds; `MaximumSeedCount` range 1–5.
- `tryAdditionalPathGuesses`: existing later seeds, deferred timed search,
  multi-winding recovery, stopping after validation, and shared budget ownership.
- A decision table showing every recovery trigger and suppression condition.

## 12. Constructing motion from each guess

- `solvePathGuess`: static/dynamic routing, timing ownership, and candidate checks.
- Static BMTP adapter and normalized solver goal.
- Dynamic branch order: conservative static projection → timed-cell BMTP →
  applicable direct-wait construction → explicit waypoint fallback.
- `createStationaryObstacleEnclosures`: exact static geometry, full-history
  conservative coverage, source mapping, and independent original-history checks.
- Timed-cell construction, route normalization, supported topology, segment
  budget, and unsupported-route reporting.
- `createWaitThenMoveMotion`: eligibility, dwell composition, timing refinement,
  bisection limits, and what is actually validated at each trial.
- `createRuckigWaypointMotion`: opt-in policy, at-most-two-segment restriction,
  rest-to-rest behavior, polynomial composition, and fallback disclosure.

### 12.1 Trajectory-engine boundary

- Request fields, outputs, time bases, derivative conventions, and failure signals.
- Degree-eight Bezier representation and static/timed span allocation.
- Warm starts, alternating motion/plane solves, conic constraints, duration
  changes, retained-horizon retry, stopping rules, and retained-best behavior.
- Travel refinement at the accepted clock; validation before replacement.
- Solver tolerances versus public physical/collision tolerances.
- Document every engine heuristic that changes obstacle-avoidance behavior;
  put detailed conic algebra in a linked trajectory-engine reference.
- Distinguish Ruckig utilities from a selectable obstacle-planner method.

## 13. Independent validation

- Public `validateTrajectory` and shared `validatePreparedTrajectory` ownership.
- Polynomial format, derivative chain, time ordering, knots, continuity,
  endpoint states, and agreement with sampled histories.
- Continuous workspace and derivative limits: Bernstein bounds, subdivision,
  stationary-point checks, and ambiguous numerical cases.
- Collision-check order: separating-plane certificate, seed-region certificate,
  then adaptive time-interval clearance checks.
- Verify complete obstacle/region/time coverage before accepting a certificate.
- Obstacle-motion bounds, event-time splitting, minimum interval size,
  clearance tolerances, and unresolved certification.
- Why sampled clearance, solver success flags, and a clear route are insufficient.
- Worked examples of collision, kinematic failure, invalid certificate, and
  unresolved interval; diagnostic fields identifying each.

## 14. Choosing the result and explaining failure

- `createCandidateSummary`: arrival, geometric path length, actual motion length,
  constraint violation, clearance, and validation status.
- `selectValidatedCandidate`: only passing motions enter success ranking.
- Exact ranking columns: earliest mode uses arrival, motion length, index;
  fixed-arrival mode uses motion length, index. Separate ranking from tolerances
  used inside motion construction and validation.
- Best-partial ranking: collision resolution, violation, clearance, and index.
- Complete termination-reason catalog, including where each reason originates
  and whether it means invalid input, exhausted search, unsupported construction,
  unresolved checking, or validated success.
- `createEmptyResult`, `assemblePlannerOutputs`, and `flattenDiagnosis`: stable empty
  values, optional details, field-path tables, and failure plotting without rerun.
- Stage timing: inclusive versus exclusive costs and avoidance of double counting.

## 15. Moving-target interception

- `planMovingTargetIntercept`: input forms, target history, interpolation,
  terminal derivative policy, fixed-time versus earliest interception.
- `findEarliestLinearIntercept`: applicability, switching regimes, roots,
  feasibility checks, and limits of the algebraic result.
- General trial-time search: coarse trials, refinement, limits, selection,
  repeated planner requests, and diagnosis propagation without rerunning a winner.
- Obstacle-free interception versus obstacle-constrained interception.
- Distinguish a target-history horizon from a guaranteed achievable arrival.

## 16. Plotting, sandbox, and processing scripts

- `plotTrajectory` and `createWrappedSpatialPath`: original/protected geometry,
  route versus actual motion, kinematics, limits, failures, and optional diagnosis.
- MATLAB sandbox: authored geometry, sampled polygon motion, run/plot/export,
  and Ctrl+C behavior.
- HTML sandbox: direct transform handles, final-pose ghost, motion interpolation,
  speed slider, rectangles, local preview, and preview/export consistency.
- Offline adapter: request conversion, units, target/obstacle histories,
  response projection, server transport errors, and restart after interruption.
- Diagnosis bundles: schema, creation, export, replay, and reproducibility limits.
- Example processing: option resolution, geographic boundary construction,
  independent validation, headless controls, and unchanged returned results.
- Benchmark processing: baseline capture, runtime stripping, comparison, stress
  scenarios, production-size auditing, CSV interpretation, and historical helpers.
- Manual-data export scripts: consumed fields, generated outputs, and regeneration.

## 17. Heuristic and numerical-policy register

One row per implemented decision, including decisions in local helpers. Each row
must state: owner and code location, trigger, formula/value and units, purpose,
what it can exclude or approximate, fallback, diagnostic evidence, and regression.
Classify each as a search heuristic, conservative approximation, numerical
safeguard, exact shortcut under stated assumptions, or presentation-only choice.

Required groups:

- Geometry cleanup/alignment, interpolation selection, history enclosure,
  construction parallelism, and cache invalidation.
- Direct-motion eligibility, excursion enumeration/screening/refinement.
- Dense-envelope work threshold and endpoint guard.
- Visibility offsets/retries, node selection, pair budget, and connectivity repair.
- Spatial route signatures, shortcutting, class limits, and deferred winding paths.
- Time-layer construction, transition samples, safe-wait pruning, and cost pruning.
- Seed ordering, slot reservation, deferred search, and recovery stopping.
- Static projection, timed topology, waits, and explicit waypoint fallback.
- Engine initialization, span/time allocation, iteration/convergence rules,
  retained-horizon retry, separating-plane reuse, and travel refinement.
- Certificate fast paths, adaptive collision bounds, numerical tolerances,
  candidate ranking, partial-result ranking, and intercept trial selection.

Do not label all constants as heuristics. State the evidence and assumptions
behind safety-preserving rules separately from search-quality tradeoffs.

## 18. Worked cases and verification plan

- Clear direct motion; static detour; concave/U-shaped obstacle; disconnected
  geometry/holes; moving barrier requiring a wait; rotating/deforming obstacle;
  dense outline; fixed-time target; earliest intercept; expected no-path.
- For each: inputs, branch decisions, geometry transformations, attempts,
  selected route, timed motion, validation, diagnosis, and limitations.
- Connect every heuristic to focused tests; identify missing coverage plainly.
- Report planner and independent-validation outcomes, path and motion lengths,
  motion duration, runtime scope, certificate status, and termination reason.
- Keep full run records in CSV; documentation contains decisions and selected
  explanatory examples, not another chronological benchmark log.

## Appendices

A. Complete public input, option, result, diagnosis, and validation field tables.

B. Internal data structures: prepared interval, proposal, graph, timed state,
route set, seed, candidate, polynomial, and certificate. Include producer/consumer
relationships, dimensions, units, and ownership.

C. Every error/termination identifier, trigger, diagnostic location, and action.

D. All constants and tolerances, grouped by owner; cross-reference the heuristic
register and distinguish configurable options from internal rules.

E. Plain-language glossary: path guess (seed), allowed motion region (corridor),
connection graph (visibility graph), different ways around obstacles (route
classes), enclosing history shape (swept envelope), solver, certificate, frontier,
dominance, warm start, and retained candidate. Give the precise technical meaning
after the approachable explanation.

F. File/function reference. Each entry should document purpose, actual inputs
used, outputs, side effects/caches, call sites, local processing helpers, branches,
complexity drivers, failure behavior, and tests. The inventory below establishes
coverage; it does not substitute for reading each implementation.

### Source inventory

- `+obstacleAvoidance/+geometry/boundaryToEdges.m`
- `+obstacleAvoidance/+geometry/boundaryToShape.m`
- `+obstacleAvoidance/+geometry/convexPolygonRegions.m`
- `+obstacleAvoidance/+geometry/pointPolygonClearance.m`
- `+obstacleAvoidance/+geometry/routeLength.m`
- `+obstacleAvoidance/+input/goalPositionAtTime.m`
- `+obstacleAvoidance/+input/normalizeLogicalScalar.m`
- `+obstacleAvoidance/+input/normalizePlannerRequest.m`
- `+obstacleAvoidance/+input/normalizePlannerState.m`
- `+obstacleAvoidance/+input/resolveOptions.m`
- `+obstacleAvoidance/+input/resolvePlannerOptions.m`
- `+obstacleAvoidance/+input/validatePlannerEndpoints.m`
- `+obstacleAvoidance/+obstacles/combineObstacles.m`
- `+obstacleAvoidance/+obstacles/createMovingObstacle.m`
- `+obstacleAvoidance/+obstacles/createObstacle.m`
- `+obstacleAvoidance/+obstacles/createStationaryObstacleEnclosures.m`
- `+obstacleAvoidance/+obstacles/prepareObstacles.m`
- `+obstacleAvoidance/+obstacles/prepareOneObstacle.m`
- `+obstacleAvoidance/+obstacles/preparePlanningScene.m`
- `+obstacleAvoidance/+obstacles/preparedShapeAtTime.m`
- `+obstacleAvoidance/+obstacles/queryObstacleOccupancyAtTime.m`
- `+obstacleAvoidance/+obstacles/queryPreparedObstacles.m`
- `+obstacleAvoidance/+obstacles/queryStaticHorizon.m`
- `+obstacleAvoidance/+obstacles/shapeAtTime.m`
- `+obstacleAvoidance/+planner/checkCandidateMotion.m`
- `+obstacleAvoidance/+planner/createCandidateSummary.m`
- `+obstacleAvoidance/+planner/createWaitThenMoveMotion.m`
- `+obstacleAvoidance/+planner/createEmptyResult.m`
- `+obstacleAvoidance/+planner/tryFixedTimeDetour.m`
- `+obstacleAvoidance/+planner/assemblePlannerOutputs.m`
- `+obstacleAvoidance/+planner/createRuckigWaypointMotion.m`
- `+obstacleAvoidance/+planner/resolveFixedEndpointForSolver.m`
- `+obstacleAvoidance/+planner/findEarliestLinearIntercept.m`
- `+obstacleAvoidance/+planner/flattenDiagnosis.m`
- `+obstacleAvoidance/+planner/tryAdditionalPathGuesses.m`
- `+obstacleAvoidance/+planner/selectValidatedCandidate.m`
- `+obstacleAvoidance/+planner/solveStaticBmtpTrajectory.m`
- `+obstacleAvoidance/+planner/solveDynamicPathGuess.m`
- `+obstacleAvoidance/+planner/tryDirectAndFixedTimeMotions.m`
- `+obstacleAvoidance/+planner/solvePathGuess.m`
- `+obstacleAvoidance/+planner/solveTimedBmtpTrajectory.m`
- `+obstacleAvoidance/+planner/stageTiming.m`
- `+obstacleAvoidance/+plotting/createWrappedSpatialPath.m`
- `+obstacleAvoidance/+plotting/plotTrajectory.m`
- `+obstacleAvoidance/+search/checkVisibilitySegments.m`
- `+obstacleAvoidance/+search/createRouteSearchGeometry.m`
- `+obstacleAvoidance/+search/createSearchDiagnostics.m`
- `+obstacleAvoidance/+search/createEmptyPathGuess.m`
- `+obstacleAvoidance/+search/createPathGuesses.m`
- `+obstacleAvoidance/+search/createVisibilityAttempt.m`
- `+obstacleAvoidance/+search/createVisibilityGraph.m`
- `+obstacleAvoidance/+search/denseSweptEnvelope.m`
- `+obstacleAvoidance/+search/searchDistinctSpatialRoutes.m`
- `+obstacleAvoidance/+search/searchRoutes.m`
- `+obstacleAvoidance/+search/timeExpandedVisibilitySearch.m`
- `+obstacleAvoidance/+validation/certifyPolynomialRange.m`
- `+obstacleAvoidance/+validation/certifySeedCorridor.m`
- `+obstacleAvoidance/+validation/checkObstacleClearance.m`
- `+obstacleAvoidance/+validation/validatePolynomialTrajectory.m`
- `+obstacleAvoidance/+validation/validatePreparedTrajectory.m`
- `+obstacleAvoidance/planMovingTargetIntercept.m`
- `+obstacleAvoidance/planTrajectory.m`
- `+obstacleAvoidance/validateTrajectory.m`
- `benchmarks/auditProductionSize.m`
- `benchmarks/benchmarkRandomMovingPolygonStress.m`
- `benchmarks/capturePlannerRefactorBaseline.m`
- `benchmarks/comparePlannerRefactorBaseline.m`
- `benchmarks/createHs3ReferenceCases.m`
- `benchmarks/createRepeatedTurnBenchmarkScenario.m`
- `benchmarks/stripPlannerRefactorRuntime.m`
- `examples/exampleAlternatingSlalom.m`
- `examples/exampleDenseConcaveObstacle.m`
- `examples/exampleFourAcceleratingCircles.m`
- `examples/exampleInterceptMovingTargetAtSetTime.m`
- `examples/exampleInterceptMovingTargetEarliest.m`
- `examples/exampleMovingBarrierWait.m`
- `examples/exampleMovingCircleNoAzimuthWrap.m`
- `examples/exampleMovingDeformingUSOutlineVisibility.m`
- `examples/exampleMovingRotatingObstacleField.m`
- `examples/exampleNoPath.m`
- `examples/exampleObstacleAvoidance.m`
- `examples/exampleObstacleFree.m`
- `examples/exampleOpeningUShapedObstacle.m`
- `examples/exampleStaticUShapedObstacle.m`
- `examples/exampleStraightTargetAlternatingOcclusion.m`
- `examples/exampleTargetExitsObstacle.m`
- `examples/exampleTwoOpposingUVisibilityGraph.m`
- `examples/exampleUSOutlineExtremeVisibility.m`
- `examples/private/createContiguousUSObstacle.m`
- `examples/private/createGeographicRegionObstacle.m`
- `examples/resolveExampleOptions.m`
- `examples/validateExampleResult.m`
- `offlinesandbox/+offlineSandbox/createDiagnosisBundle.m`
- `offlinesandbox/+offlineSandbox/replayDiagnosisBundle.m`
- `offlinesandbox/+offlineSandbox/runPlanningRequest.m`
- `offlinesandbox/+offlineSandbox/serveSandbox.m`
- `output/pdf/data/exportPlannerManualData.m`
- `sandbox/createSandboxPolygonMotionHistory.m`
- `sandbox/exportSandboxDiagnosis.m`
- `sandbox/obstacleAvoidanceSandbox.m`
- `trajectory/+bmtpEngine/accumulateConicDiagnostics.m`
- `trajectory/+bmtpEngine/checkFinalMotion.m`
- `trajectory/+bmtpEngine/createConstantJerkPowerCoefficients.m`
- `trajectory/+bmtpEngine/createCoordinateTolerances.m`
- `trajectory/+bmtpEngine/createDirectMotion.m`
- `trajectory/+bmtpEngine/createMotionOutput.m`
- `trajectory/+bmtpEngine/createMotionRecord.m`
- `trajectory/+bmtpEngine/createOffsetSplineMotion.m`
- `trajectory/+bmtpEngine/createPowerPolynomial.m`
- `trajectory/+bmtpEngine/createSolveRequest.m`
- `trajectory/+bmtpEngine/createWarmStart.m`
- `trajectory/+bmtpEngine/evaluatePolynomial.m`
- `trajectory/+bmtpEngine/findRequiredSegmentTime.m`
- `trajectory/+bmtpEngine/findSampledObstacleOverlaps.m`
- `trajectory/+bmtpEngine/maximumRestToRestDistance.m`
- `trajectory/+bmtpEngine/prepareFinalMotion.m`
- `trajectory/+bmtpEngine/refineTravel.m`
- `trajectory/+bmtpEngine/solve.m`
- `trajectory/+bmtpEngine/solveAlternatingTrajectory.m`
- `trajectory/+bmtpEngine/solveSeparatingLine.m`
- `trajectory/+bmtpEngine/solveTrajectoryStep.m`
- `trajectory/+bmtpEngine/verifySeparatingLine.m`
