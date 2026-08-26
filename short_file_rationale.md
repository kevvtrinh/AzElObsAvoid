# Why short production MATLAB files remain separate

This audit covers production MATLAB files while excluding tests, examples,
benchmarks, the interactive sandbox, and temporary verification artifacts.
Short files remain only when they own a distinct contract or mathematical
invariant:

| Area | Ownership result |
| --- | --- |
| Root public APIs | Stable obstacle, planner, interception, validation, plotting, and query boundaries. |
| Neutral `azElInternal` | Input, geometry, topology, corridor, result, and polynomial invariants used by HS3. |
| HS3 | Option, validation, transcription, and solver-diagnostic boundaries. |
| Planner timing | One exclusive timing schema for the maintained planner. |

The earlier audit described method-local moving-target adapters, topology
generators, corridor certificates, result builders, final validators, and HS3
stop-motion helpers. Those duplicate owners have been removed. One root
intercept adapter, neutral topology/corridor/result helpers, and root
`validateAzElTrajectory` now own those contracts.

## Root public boundaries

| File | Why it stays separate |
| --- | --- |
| `combineAzElObstacles.m` | Public obstacle-container normalization used before immutable preparation. |
| `planAzElMotion.m` | HS3-only public planner boundary and defaults owner. |

`makeAzElObstacleData.m` owns fresh construction, imported-record
normalization, and absolute reinflation; the separate normalizer and inflater
were removed after all maintained callers migrated.

The shared `planAzElMovingTargetIntercept.m` and canonical
`validateAzElTrajectory.m` are no longer short dispatchers; each owns its
complete policy.

## Neutral shared internals

Short neutral files remain separate where they define one mathematical or
data-contract boundary:

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
and `validatePolynomialTrajectory` remain separate for the same reason.

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
respectively. HS3 never calls an alternate motion planner or fallback.

## Shared timing ownership decision

`+azElPlannerMethods/+internal/stageTiming.m` remains separate because it
defines, reconciles, and synchronizes the exclusive planner timing schema.

No remaining short file should be merged solely to reduce file count. A merge
is justified only when its mathematical invariant, compatibility boundary, or
shared ownership disappears. File length by itself is not evidence that a
boundary is redundant.
