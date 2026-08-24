# Why short production MATLAB files remain separate

This audit counts nonblank, noncomment lines in production MATLAB files. It
excludes tests, examples, benchmarks, the interactive sandbox, and temporary
verification artifacts. The current tree has 34 production files below 100
code lines:

| Area | Short files | Ownership result |
| --- | ---: | --- |
| Root public APIs | 3 | Stable construction and dispatch boundaries. |
| Neutral `azElInternal` | 20 | Shared input, geometry, topology, corridor, result, polynomial, and comparison invariants. |
| Compact corridor | 5 | Compact-specific motion/adapters plus a validator facade. |
| HS3 | 5 | Standalone option/validation boundaries and focused Hermite-Simpson primitives. |
| Shared planner timing | 1 | One timing schema used independently by both methods. |

The earlier audit described method-local moving-target adapters, topology
generators, corridor certificates, result builders, final validators, and HS3
stop-motion helpers. Those duplicate owners have been removed. One root
intercept adapter, neutral topology/corridor/result helpers, and root
`validateAzElTrajectory` now own those contracts.

## Root public boundaries

| File | Code lines | Why it stays separate |
| --- | ---: | --- |
| `combineAzElObstacles.m` | 71 | Public obstacle-container normalization used before immutable preparation. |
| `makeAzElObstacleData.m` | 28 | Public protected-obstacle construction and safety-margin boundary. |
| `planAzElMotion.m` | 80 | Public selector that dispatches to either separate planner without cross-calls. |

The shared `planAzElMovingTargetIntercept.m` and canonical
`validateAzElTrajectory.m` are no longer short dispatchers; each owns its
complete policy.

## Neutral shared internals

The 20 short neutral files remain separate where they define one mathematical
or data-contract boundary:

- `resolveOptions`, `normalizeLogicalScalar`, and `validatePlannerEndpoints`
  own partial-option, logical, and shared endpoint-validation semantics;
- `prepareDynamic`, `shapeAtTime`, `boundaryShape`, `boundaryToEdges`, and
  `pointPolygonClearance` own immutable obstacle and geometry interpretation;
- `boundedTimeLayers`, `clusterSeedShape`, `convexPolygonRegions`,
  `buildSeedCorridor`, `certifySeedCorridor`, `seedCorridorInequality`, and
  `seedEnvelopeContainsObstacles` own bounded search reduction and independent
  corridor evidence;
- `goalPositionAtTime`, `evaluatePolynomial`, `powerToBernstein`, and
  `integratedSquaredPolynomialJerk` own exact shared motion interpretation;
- `acceptsTrajectoryImprovement` owns the monotone validation/arrival/jerk
  acceptance rule.

The larger neutral `normalizePlannerRequest`, `generateTopologySeeds`,
`denseSweptEnvelope`, `timeExpandedVisibilitySearch`, `emptyPlannerResult`,
and `validatePolynomialTrajectory` files remain separate for the same reason
but are outside this under-100-line table.

## Compact-specific short files

| File or group | Why it stays separate |
| --- | --- |
| `buildFixedDurationAffineModel`, `expandRouteClearance` | Focused affine and clearance algorithms consumed by compact motion construction. |
| `buildEnvelopeBoundary`, `expandDynamicRoute` | Small adapters for compact envelope and time-local route behavior. |
| `validateTrajectory` | Compatibility facade over canonical root validation. |

There are no compact-local topology, convex-decomposition, certificate, result,
or moving-target owners.

## HS3-specific short files

| File or group | Why it stays separate |
| --- | --- |
| `emptyHs3SolverDiagnostics` | Stable flat schema for every nonlinear-solver exit. |
| `integratedSquaredHs3Jerk` | HS3 decision-space objective and gradient. |
| `hs3AffineSensitivity` | Exact Hermite-Simpson coefficient, state, and evaluation sensitivity maps. |
| `resolvePlannerOptions`, `validateTrajectory` | Standalone HS3 option ownership and a compatibility facade over canonical validation. |

The standalone `plan.m` orchestrator and the larger `solveHs3`,
`evaluateHs3TrajectoryConstraints`, and `buildFixedHs3ConstraintMatrices`
files stay separate because they own planning policy, one NLP transcription,
hybrid constraint evaluation, and exact fixed-time matrix assembly,
respectively. The complete HS3 package currently owns 1,602 noncomment lines
under its 2,000-line cap. That count excludes neutral shared dependencies but
does not exclude or rely on a compact baseline: HS3 never calls corridor.

## Shared timing ownership decision

`+azElPlannerMethods/+internal/stageTiming.m` remains separate because its 56
code lines define, reconcile, and synchronize the exclusive timing schema for
both public methods.

No remaining short file should be merged solely to reduce file count. A merge
is justified only when its mathematical invariant, compatibility boundary, or
shared ownership disappears. File length by itself is not evidence that a
boundary is redundant.
