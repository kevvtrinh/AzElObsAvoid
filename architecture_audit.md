# MATLAB Architecture Necessity Audit

Audit date: 2026-08-28

Scope: every one of the 123 maintained MATLAB sources present at audit start. `Rogue Examples/`, temporary baselines, and external worktrees were excluded. The classification is against the current architecture: exact Ruckig-derived direct motion, HS3 for obstacle-constrained motion, canonical obstacle geometry/query ownership, and one public obstacle-planner entry.

Decision meanings:

- **Retain**: owns active behavior, a stable public boundary, required diagnostics, or nonduplicated verification.
- **Retain, consolidated**: necessary file whose obsolete comparison/free-mode branches were removed in this pass.
- **Defer consolidation**: useful behavior remains, but ownership can be simplified only with a measured migration.
- **Removed**: caller-free historical artifact with no current benchmark or runtime ownership.

Key audit conclusions:

- The pass-through/hybrid and nonuniform-mesh files are not dead. They retain measured benchmark wins and therefore stay.
- `CollectAllSeedCandidates` and completed-candidate trajectory retention were comparison-only planner surface and are removed.
- Sandbox Free Mode was already unreachable; its segment-search and state machinery are removed.
- The moving-target wrapper and geographic example family are the next meaningful consolidation candidates, but both still own maintained coverage.
- The legacy 21-case script was replaced by a neutral fixture; active benchmarks now call the maintained engines directly.

| File | Decision | Behavioral ownership / evidence |
| --- | --- | --- |
| `+obstacleAvoidance/+geometry/boundaryToEdges.m` | Retain | Canonical geometry primitive used by obstacle queries, search, or validation. |
| `+obstacleAvoidance/+geometry/boundaryToShape.m` | Retain | Canonical geometry primitive used by obstacle queries, search, or validation. |
| `+obstacleAvoidance/+geometry/canonicalBoundaryToEdges.m` | Retain | Canonical geometry primitive used by obstacle queries, search, or validation. |
| `+obstacleAvoidance/+geometry/convexPolygonRegions.m` | Retain | Canonical geometry primitive used by obstacle queries, search, or validation. |
| `+obstacleAvoidance/+geometry/pointPolygonClearance.m` | Retain | Canonical geometry primitive used by obstacle queries, search, or validation. |
| `+obstacleAvoidance/+input/goalPositionAtTime.m` | Retain | Single public-boundary normalization/default owner. |
| `+obstacleAvoidance/+input/normalizeLogicalScalar.m` | Retain | Single public-boundary normalization/default owner. |
| `+obstacleAvoidance/+input/normalizePlannerRequest.m` | Retain | Single public-boundary normalization/default owner. |
| `+obstacleAvoidance/+input/resolveOptions.m` | Retain | Single public-boundary normalization/default owner. |
| `+obstacleAvoidance/+input/resolvePlannerOptions.m` | Retain, consolidated | Single public-boundary normalization/default owner. |
| `+obstacleAvoidance/+input/validatePlannerEndpoints.m` | Retain | Single public-boundary normalization/default owner. |
| `+obstacleAvoidance/+obstacles/combineObstacles.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+obstacles/createMovingObstacle.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+obstacles/createObstacle.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+obstacles/hasChangingHistory.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+obstacles/prepareDynamic.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+obstacles/queryBoundaryEdgeAtTime.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+obstacles/queryObstacleOccupancyAtTime.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+obstacles/shapeAtTime.m` | Retain | Canonical obstacle construction, preparation, interpolation, or query owner. |
| `+obstacleAvoidance/+planner/classifyPassThroughSearch.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `+obstacleAvoidance/+planner/createConstraintMatrices.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/createEmptyResult.m` | Retain, consolidated | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/createEmptySolverDiagnostics.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/createHybridActivityMesh.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `+obstacleAvoidance/+planner/evaluatePlannerPolynomial.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/evaluateTrajectoryConstraints.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/plan.m` | Retain, consolidated | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/ruckigWarmStart.m` | Retain, optional boundary | Sole planner-call boundary for deletable Ruckig-derived HS3 warm-start behavior. Its absence resolves the mode to ordinary HS3. |
| `+obstacleAvoidance/+planner/solvePassThroughSeedCandidate.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `+obstacleAvoidance/+planner/solveRouteCandidate.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/solveWaypointSeedCandidate.m` | Retain | Exact Ruckig composition fallback for failed multi-edge visibility seeds; covered by independent static-detour and Rogue-case validation. |
| `+obstacleAvoidance/+planner/stageTiming.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/validatePolynomialTrajectory.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+plotting/createWrappedSpatialPath.m` | Retain | Shared periodic display transform for sandbox, diagnostics, and animation; preserves unwrapped planner output. |
| `+obstacleAvoidance/+plotting/plotTrajectory.m` | Retain, consolidated | Reusable result-only success/failure visualization owner, including periodic and continuous-azimuth views. |
| `+obstacleAvoidance/+search/boundedTimeLayers.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/certifySeedCorridor.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/clusterSeedShape.m` | Removed | Default-zero clustering had no maintained caller and conservatively erased passages. |
| `+obstacleAvoidance/+search/createRouteCandidates.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/createSeedCorridor.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/denseSweptEnvelope.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/seedCorridorInequality.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/seedEnvelopeContainsObstacles.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/timeExpandedVisibilitySearch.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/planMovingTargetIntercept.m` | Defer consolidation | Public wrapper used by four maintained moving-target examples; behavior overlaps the main planner request model. |
| `+obstacleAvoidance/planTrajectory.m` | Retain | Maintained public planner or validator entry point. |
| `+obstacleAvoidance/validateTrajectory.m` | Retain | Maintained public planner or validator entry point. |
| `benchmarks/benchmarkHs3SharedReference.m` | Retain | Active reproducible engine, scaling, or stress benchmark. |
| `benchmarks/benchmarkRandomMovingPolygonStress.m` | Retain | Active reproducible engine, scaling, or stress benchmark. |
| `benchmarks/benchmarkStandaloneHs3Scaling.m` | Retain | Active reproducible engine, scaling, or stress benchmark. |
| `benchmarks/benchmarkTrajectoryEngines.m` | Retain | Active reproducible engine, scaling, or stress benchmark. |
| `benchmarks/compareHs3ExtractionBaseline.m` | Retain | Frozen behavior/runtime parity gate used during extraction and consolidation. |
| `benchmarks/createHs3ReferenceCases.m` | Retain, consolidated | Single deterministic owner of the 21-case trajectory-engine benchmark corpus. |
| `benchmarks/createRepeatedTurnBenchmarkScenario.m` | Retain | Shared deterministic benchmark fixture. |
| `benchmarks/probeHs3SharedReferenceTimes.m` | Removed | No callers; one-off published-arrival feasibility investigation. |
| `benchmarks/reference/testSlewTrajectoriesHS3.m` | Removed | Legacy script mixed fixtures, analytical comparison, moving-target solving, plotting, and workspace-variable outputs; active benchmarks now use the neutral fixture. |
| `examples/exampleAlternatingSlalom.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleDenseConcaveObstacle.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleFourAcceleratingCircles.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleInterceptMovingTargetAtSetTime.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleInterceptMovingTargetEarliest.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleMovingBarrierWait.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleMovingCircleNoAzimuthWrap.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleMovingDeformingUSOutlineVisibility.m` | Defer consolidation | Mapping Toolbox stress coverage; heavyweight and partially overlapping, but exercises full-resolution geometry. |
| `examples/exampleNoPath.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleObstacleAvoidance.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleObstacleFree.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleOpeningUShapedObstacle.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleStaticUShapedObstacle.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleStraightTargetAlternatingOcclusion.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleTargetExitsObstacle.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleTwoOpposingUVisibilityGraph.m` | Retain | Distinct executable scenario contract in the 17-example serial gate. |
| `examples/exampleUSOutlineExtremeVisibility.m` | Defer consolidation | Mapping Toolbox stress coverage; heavyweight and partially overlapping, but exercises full-resolution geometry. |
| `examples/private/createContiguousUSObstacle.m` | Defer consolidation | Mapping Toolbox stress coverage; heavyweight and partially overlapping, but exercises full-resolution geometry. |
| `examples/private/createGeographicRegionObstacle.m` | Defer consolidation | Mapping Toolbox stress coverage; heavyweight and partially overlapping, but exercises full-resolution geometry. |
| `examples/resolveExampleOptions.m` | Retain | Shared maintained-example option or independent-validation infrastructure. |
| `examples/validateExampleResult.m` | Retain | Shared maintained-example option or independent-validation infrastructure. |
| `sandbox/createSandboxPolygonMotionHistory.m` | Retain | Goal Mode moving-polygon profile constructor with direct tests. |
| `sandbox/exportSandboxDiagnosis.m` | Retain, consolidated | Maintained Goal Mode UI or diagnosis export path. |
| `sandbox/obstacleAvoidanceSandbox.m` | Retain, consolidated | Maintained Goal Mode UI with public planner-option controls and reproducible diagnosis export. |
| `tests/+testSupport/plannerFixtures.m` | Retain | Shared deterministic planner fixture/requirement support. |
| `tests/+testSupport/verifySharedPlannerRequirement.m` | Retain | Shared deterministic planner fixture/requirement support. |
| `tests/testArchitectureBoundaries.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testAzimuthWrappingPlotting.m` | Retain | Covers both seam directions, wrapped animation, and the continuous-azimuth companion figure. |
| `tests/testExampleInvariants.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testHs3Planner.m` | Retain, consolidated | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testHs3PolynomialOperations.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testHybridActivityMesh.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `tests/testNonuniformHs3Mesh.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `tests/testObstacleAvoidanceSandboxDiagnosis.m` | Retain, consolidated | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testObstacleInfrastructure.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testPassThroughSearchClassification.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `tests/testPassThroughWaypointWarmStart.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `tests/testPlannerOptions.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testPlannerStageTiming.m` | Retain, consolidated | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testRuckigEngine.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
| `tests/testRuckigWaypointFallback.m` | Retain | Independent earliest- and fixed-arrival coverage for exact multi-edge Ruckig composition. |
| `tests/testStandaloneHs3Kernel.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
| `trajectory/+hs3Engine/+constraints/createFixedConstraintMatrices.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+constraints/evaluateConstraints.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+constraints/evaluatePolynomialConstraints.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+polynomial/convertPowerToBernstein.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+polynomial/createAffineSensitivityModel.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+polynomial/createSubintervalBernsteinMap.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+polynomial/createTrajectoryPolynomial.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+polynomial/evaluateIntegratedSquaredJerk.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+polynomial/evaluateTrajectoryPolynomial.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/+polynomial/resolveSegmentMesh.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/defaultOptions.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/optimize.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/solve.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/solveFixedTime.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/solveFreeTime.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+hs3Engine/validate.m` | Retain | Dimension-neutral HS3 kernel, polynomial, constraint, solve, or validation owner. |
| `trajectory/+ruckigEngine/+internal/createEmptyResult.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/+internal/evaluatePolynomial.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/+internal/evaluatePolynomialConstraints.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/+internal/normalizeRequest.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/+internal/validateResult.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/checkEligibility.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/createFixedTimeAxisProfile.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/createMinimumTimeAxisProfile.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/createRestToRestJerkProfile.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/createSynchronizedJerkProfile.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/defaultOptions.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/evaluateAxisSwitchingProfile.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/+ruckigEngine/solve.m` | Retain | Independent exact switching engine primitive; direct-motion tests cover it. |
| `trajectory/planTrajHs3.m` | Retain | Named public trajectory-engine facade protected by architecture tests. |
| `trajectory/planTrajRuckig.m` | Retain | Named public trajectory-engine facade protected by architecture tests. |

## Verification attached to this audit

- Code Analyzer: zero findings in all changed MATLAB files.
- Unit/integration tests: 163/163 passed in 57.965065 aggregate test seconds.
- Maintained examples: all 17 run in separate MATLAB processes; 16 validated successes and one expected `noValidatedSeed` failure.
- Visible success: `exampleObstacleFree` created three figures.
- Failure diagnostics: `exampleNoPath` created two diagnostic figures.
- Retained hybrid score: Single-U pass-through 21.2540287320 s arrival, 40.5204361036 deg motion, 15.542280 s fresh wall.
- Known incomplete gate: the frozen three-repeat scaling comparator was interrupted after its 10-turn case took 137.6659266 s and later work flooded existing near-singular solver warnings.

## 2026-08-29 additions

The audit now includes the exact Ruckig waypoint fallback and its focused test.
They are retained because the saved Rogue case becomes independently valid
without replacing the preferred HS3 path or adding a scenario-specific route.
The sandbox remains one Goal Mode workflow; its new planner-options panel
exposes three existing public choices and preserves them in both Run and Export.
The complete test tree passes 166/166, the focused fixed-arrival fallback check
passes, and the visible sandbox layout was inspected.

## Optional Ruckig warm-start boundary

The main planner now reaches pass-through classification, exact waypoint-state
search, and activity-mesh refinement only through
`+obstacleAvoidance/+planner/ruckigWarmStart.m`. Option resolution checks for
that exact file before planner execution. If it is absent, a selected or future
default `"passThrough"` mode becomes resolved mode `"none"`; an explicit
request receives one warning and the stable result echoes the fallback.

A copied-repository deletion gate removed only that file and then ran the
maintained Single-U example with an explicit pass-through request. Ordinary
HS3 returned success, independent validation and collision checks passed,
arrival was 22.630887102 seconds, selected polyline length was
34.9425880405 degrees, and smoothed length was 41.5367249083 degrees. Direct
Ruckig motion, the trajectory engine, and the exact rest-to-rest waypoint
fallback remain deliberately outside this optional boundary.

## Planner option ownership audit - 2026-09-01

The current public default record contains 14 fields after removing the unread
planner `Verbose` field and internalizing BMTP plane-reuse and trajectory-solver
controls, including warm-route segmentation. Every
remaining field has a production consumer.

| Classification | Fields | Decision |
| --- | --- | --- |
| Meaningful request/search/validation choices | `GoalTimeMode`, `MinimumTravelSavingsRate_deg_s`, `SampleTime_s`, `TrajectoryMethod`, `UnsupportedTimedTopologyPolicy`, `AllowAzimuthWrapping`, `MaximumSeedCount`, `MaximumTimeLayerCount`, `MaximumWaitRefinementIterations`, `ArrivalTimeTolerance_s`, `ConstraintTolerance`, `CollisionClearanceTolerance_deg`, `CollisionMinimumTimeStep_s`, `CancellationCheckFcn` | Retain. |
| Active implementation controls to evaluate independently | None | The audit found no remaining public field whose only purpose is tuning internal solver construction. |
| Removed or internalized controls | `Verbose`, `EnablePlaneReuse`, `PlaneReuseImprovementTolerance_s`, `MaximumNlpIterations`, `CollocationSegmentCount` | One-release direct-planner warnings and stripping. Caller-owned logging remains; BMTP plane reuse is automatic; the trajectory iteration cap and 20-span warm-route/time-cell cap are internally owned. |
| Compatibility-only inputs | `WaypointWarmStartMode`, `RequestedWaypointWarmStartMode`, `IsWaypointWarmStartAvailable`, `PerSeedWorkBudgetMultiplier`, `SeedClusterDistance_deg` | Warn and strip; none appears in returned defaults. |

This classification is based on production field reads, not names or
documentation. `MaximumNlpIterations` was active rather than dead; its behavior
was internalized only after preserving the former public default and measuring
the nondefault timed and sandbox callers. `CollocationSegmentCount` was
likewise internalized at its former effective default of 20 spans only after
exact static and timed comparisons.

## BMTP restart ownership audit - 2026-09-01

The maintained planner has no caller for externally returned BMTP restart
state. Both static and timed planner adapters call `bmtpEngine.solve` with
seven inputs and two outputs. Only direct tests exercised the former eighth
input and third output through `planTrajBmtp`.

The core engine now owns one seed-derived initialization path and returns only
candidate and diagnostics. The one-release `planTrajBmtp` compatibility facade
still accepts the old arities, warns once when restart input or output is used,
ignores supplied state, and returns a documented empty restart record. This
keeps migration explicit while removing restart validation, alternate initial
best retention, and restart export from the core.

## Plane-reuse diagnostic ownership audit - 2026-09-01

Automatic plane reuse retains two stable summary fields:
`PlaneReuseApplied` and `PlaneReuseCount`. Three iteration-detail arrays were
removed because no production decision, plotter, sandbox, exporter, or public
consumer read them. Their associated pending control and duration snapshots
were diagnostic-only and did not feed a later solve.

The reuse condition, unchanged tagged-pair requirement, plane-preserving
continuation, arrival-tolerance ownership, convergence, retained-best evidence,
collision history, and certificates remain. Complete Target Exits and Extreme
US Outline results matched their saved reuse-triggering baselines exactly after
excluding only runtime and the declared retired fields.

## Dead planner-option compatibility audit - 2026-09-01

Ten retired planner fields had no remaining algorithmic reader and were absent
from returned defaults. Their only production ownership was 54 lines of
special warning/removal logic in `resolvePlannerOptions` and a nine-name
example forwarding allowlist. Planner-level `Verbose` was also dead, while the
separate example display control with that spelling remains live.

The bespoke compatibility paths are now removed. Direct planner callers get
one aggregate `planTrajectory:UnknownOptions` warning for any mix of obsolete
fields, using the same path as every unsupported option. The example resolver
warns once with `resolveExampleOptions:UnknownOptions` and discards obsolete
planner-only inputs at its boundary. No field was renamed or replaced, and no
retired value reaches returned options or a planner decision. This collapses
seven warning owners and the forwarding allowlist into the two already
maintained unknown-field policies.

## Travel-refinement diagnostic ownership audit - 2026-09-01

The balanced and fixed-arrival travel-refinement loops remain behavior-bearing,
but their fifteen `TravelRefinement*` fields had no consumer outside
`bmtpEngine.solve`. Those fields recorded attempted rates, candidate arrays,
counts, exit status, duration, and solver text without influencing a later
solve, selection, validation, plot, or failure decision.

The trace payload and writes are removed. A local
`travelRefinementAccepted` boolean retains the sole control-flow dependency.
All rate construction, SOCP calls, collision checks, plane additions, length
and tradeoff objectives, selected control nets, and certification remain in
execution order. Explicit balanced and fixed refinement-active cases match
their saved physical baselines exactly after removing only the declared schema
and nondeterministic timing fields.

## BMTP facade ownership audit - 2026-09-01

`trajectory/planTrajBmtp.m` had no production caller after restart state moved
out of the engine. Its seven-input path only forwarded to `bmtpEngine.solve`;
its eighth input and third output existed solely to warn and return an empty
restart record. Tests and current appendix text were its only remaining
consumers.

The facade is deleted rather than replaced. `obstacleAvoidance.planTrajectory`
remains the public request-level entry point, and maintained adapters continue
to call the package engine directly. Architecture tests require
`planTrajBmtp.m` to remain absent and `+bmtpEngine/solve.m` to remain present.
This removes a competing public surface without changing the engine contract
used by production planning.

## BMTP final-plane ownership audit - 2026-09-01

Final BMTP certificate construction previously owned two separator algorithms
for unresolved output-span/region pairs: a constant cardinal-axis box shortcut
and the general degree-one maximum-margin conic separator. The shortcut did not
participate in route choice, trajectory optimization, time dilation, or public
continuous validation; it only avoided conic calls when axis-aligned bounding
boxes already proved separation.

The cardinal-axis constructor and its bounds/dispatch branch are removed.
Every unresolved pair now uses `solveMaximumMarginPlane`, while verified
retained parent planes still restrict exactly to each subdivided output span.
The stable certificate retains `AnalyticPairCount` because the independent
orthogonal-cavity motion constructor still produces analytic certificates;
ordinary BMTP certificates now report zero. No public option or new facade was
introduced.

## Uniform BMTP final-certificate ownership audit - 2026-09-01

Verified optimizer planes previously served two roles: they constrained later
BMTP optimization iterations and, through `restrictRetainedPlane`, bypassed the
general solver for some final certificate pairs. Only the first role affects
trajectory generation. The second role created a separate final-certification
algorithm, argument plumbing, re-verification branch, and reuse accounting.

The final-certificate shortcut is removed. Every applicable final output-span
and obstacle-region pair now calls `solveMaximumMarginPlane` and verifies its
result through the same path. Optimizer plane reuse remains unchanged in the
outer solve. The stable certificate schema retains `ReusedPairCount` for other
producers, while ordinary BMTP certificates now report zero and place every
applicable pair in `ConicPairCount`. No option, fallback, or replacement helper
was introduced.

## Moving-obstacle spatial-projection ownership audit - 2026-09-01

Dynamic multi-waypoint seeds previously invoked the static BMTP kernel twice:
first against only invariant obstacles and then, if full-scene validation
rejected that result, against a conservative static projection of every
protected obstacle history. The first attempt was safe but required obstacle
partitioning, its own diagnostic schema, and attachment plumbing around a
duplicate kernel invocation.

The static-only attempt and `StaticProjection` record are removed. Eligible
dynamic seeds now have one spatial approximation,
`createStaticPlanningProjection`, whose provenance and full-scene validation
remain in `SweptProjection`. If that conservative representation cannot return
a valid motion, a time-expanded visibility seed still reaches
`solveTimedBmtpTrajectory`, which models obstacle activity by overlapping time
cells. Direct-wait handling and its continuous wait refinement remain separate
because they represent explicit temporal motion rather than another spatial
projection.
