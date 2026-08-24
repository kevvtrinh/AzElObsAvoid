# Why short production MATLAB files remain separate

This audit counts nonblank, noncomment lines in production MATLAB files. It
excludes tests, examples, benchmarks, the interactive sandbox, and temporary
verification artifacts. The current tree has 33 production files below 100
code lines:

| Area | Short files | Ownership result |
| --- | ---: | --- |
| Root public APIs | 3 | Stable construction and dispatch boundaries. |
| Neutral `azElInternal` | 19 | Shared input, geometry, topology, corridor, result, polynomial, and comparison invariants. |
| Compact corridor | 5 | Compact-specific motion/adapters plus a validator facade. |
| HS3 | 5 | Compatibility facades and focused solver primitives. |
| Shared planner timing | 1 | One timing schema used by the composition. |

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
| `planAzElMotion.m` | 84 | Public selector and compact-baseline/HS3 composition owner. |

The shared `planAzElMovingTargetIntercept.m` and canonical
`validateAzElTrajectory.m` are no longer short dispatchers; each owns its
complete policy.

## Neutral shared internals

The 19 short neutral files remain separate where they define one mathematical
or data-contract boundary:

- `resolveOptions` and `normalizeLogicalScalar` own partial-option and logical
  normalization semantics;
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

The larger neutral `generateTopologySeeds`, `denseSweptEnvelope`,
`timeExpandedVisibilitySearch`, `emptyPlannerResult`, and
`validatePolynomialTrajectory` files remain separate for the same reason but
are outside this under-100-line table.

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
| `linearizeHs3Constraints` | Fixed-time affine constraint/Jacobian extraction. |
| `plan`, `validateTrajectory` | Compatibility facades over root composition and canonical validation. |

The former analytic stop-motion, candidate, timing, topology, corridor, result,
and validation helpers are gone. The complete HS3 package—including files above
100 lines—currently owns exactly 1,200 noncomment lines at its cap. That
ownership count excludes the compact baseline and neutral dependencies and is
not a whole-execution-closure claim.

## Shared timing and merge decision

`+azElPlannerMethods/+internal/stageTiming.m` remains separate because its 56
code lines define, reconcile, and synchronize the exclusive timing schema for
both public selections.

No remaining short file should be merged solely to reduce file count. A merge
is justified only when its mathematical invariant, compatibility boundary, or
shared ownership disappears. File length by itself is not evidence that a
boundary is redundant.
