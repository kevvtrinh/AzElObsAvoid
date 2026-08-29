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
| `+obstacleAvoidance/+planner/solvePassThroughSeedCandidate.m` | Retain | Owns measured Single-U/Two-U quality and runtime wins; covered by focused tests. |
| `+obstacleAvoidance/+planner/solveRouteCandidate.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/stageTiming.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+planner/validatePolynomialTrajectory.m` | Retain | Active planner orchestration, corridor, solve, validation, or result owner. |
| `+obstacleAvoidance/+plotting/plotTrajectory.m` | Retain, consolidated | Reusable result-only success/failure visualization owner. |
| `+obstacleAvoidance/+search/boundedTimeLayers.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/certifySeedCorridor.m` | Retain | Input-derived route/search diagnostics owner. |
| `+obstacleAvoidance/+search/clusterSeedShape.m` | Retain | Input-derived route/search diagnostics owner. |
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
| `sandbox/obstacleAvoidanceSandbox.m` | Retain, consolidated | Maintained Goal Mode UI or diagnosis export path. |
| `tests/+testSupport/plannerFixtures.m` | Retain | Shared deterministic planner fixture/requirement support. |
| `tests/+testSupport/verifySharedPlannerRequirement.m` | Retain | Shared deterministic planner fixture/requirement support. |
| `tests/testArchitectureBoundaries.m` | Retain | Active unit, integration, architecture, or diagnostic regression coverage. |
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
