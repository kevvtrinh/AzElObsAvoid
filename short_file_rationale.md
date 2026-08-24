# Why short production MATLAB files remain separate

This audit counts nonblank, noncomment MATLAB lines outside tests, examples,
benchmarks, sandbox, and temporary artifacts. The quintic-only tree has 44
production MATLAB files; 27 contain fewer than 100 code lines.

| Area | Short files | Ownership result |
| --- | ---: | --- |
| Root public APIs | 2 | Stable obstacle construction and normalization contracts. |
| Internal geometry | 3 | Shared boundary and clearance invariants. |
| Internal motion | 2 | Focused affine-model and route-clearance operations. |
| Internal obstacles | 3 | Prepared history, interpolation, and envelope ownership. |
| Internal search | 1 | Time-local route adjustment. |
| Other internal contracts | 16 | Options, schemas, timing, topology reduction, certificates, and polynomial math. |

## Root public boundaries

| File | Code lines | Why it stays separate |
| --- | ---: | --- |
| `combineAzElObstacles.m` | 71 | Public obstacle-container normalization used by planning, validation, and queries. |
| `makeAzElObstacleData.m` | 28 | Public protected-obstacle construction and the safety-margin boundary. |

## Internal boundaries

The following short files each own a reusable invariant or isolate a focused
algorithm from a large orchestrator:

- geometry: `boundaryShape` (8), `boundaryToEdges` (30), and
  `pointPolygonClearance` (44);
- motion: `buildFixedDurationAffineModel` (37) and
  `expandRouteClearance` (34);
- obstacles: `buildEnvelopeBoundary` (28), `prepareDynamic` (75), and
  `shapeAtTime` (94);
- search: `expandDynamicRoute` (44);
- option and schema contracts: `resolveOptions` (17),
  `normalizeLogicalScalar` (8), `emptyPlannerResult` (74), and
  `stageTiming` (56);
- bounded topology and corridor evidence: `boundedTimeLayers` (20),
  `clusterSeedShape` (90), `convexPolygonRegions` (29),
  `buildSeedCorridor` (49), `certifySeedCorridor` (64),
  `seedCorridorInequality` (22), and
  `seedEnvelopeContainsObstacles` (79);
- polynomial and selection math: `goalPositionAtTime` (8),
  `evaluatePolynomial` (62), `powerToBernstein` (17),
  `integratedSquaredPolynomialJerk` (37), and
  `acceptsTrajectoryImprovement` (64).

The direct public planner remains the sole orchestration owner. No
method-selection package or planner facade remains.

## Merge decision

No short production file should be merged solely to reduce file count. These
files either prevent duplicated public contracts, supply independently reused
validation math, or keep focused algorithms out of already large search and
motion owners. A future merge is justified only when its invariant becomes
genuinely caller-local.
