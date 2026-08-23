# Why production MATLAB files below 100 code lines remain separate

This audit counts nonblank, noncomment lines in production MATLAB files. It
excludes tests, examples, benchmarks, the interactive sandbox, temporary
verification files, and imported snapshots. Callers are still counted across
tests, examples, benchmarks, and the sandbox because those calls establish
real ownership.

The combined branch has 49 production files below 100 code lines:

| Area | Short files | Result |
| --- | ---: | --- |
| Root public APIs | 6 | All have multiple callers. |
| Shared root internals | 6 | Four have multiple callers; two are plotter-owned adapters. |
| Corridor method | 21 | All are in the corridor closure; four have one caller. |
| HS3 method | 16 | All are in the HS3 closure; three have one caller. |

No short production file has zero executable callers. A separate reachability
audit did find 22 obsolete root copies of planner search, motion, and validation
code; those files were deleted because their complete replacements now live in
the method folders.

## Root public APIs

| File | Code lines | Why it stays separate |
| --- | ---: | --- |
| `combineAzElObstacles.m` | 71 | Public container-normalization contract used by constructors, examples, benchmarks, and the sandbox. |
| `makeAzElObstacleData.m` | 28 | Public protected-obstacle constructor used by 21 callers. |
| `planAzElMotion.m` | 80 | Stable planner dispatcher used by 23 callers; merging it would duplicate method selection throughout the repository. |
| `planAzElMovingTargetIntercept.m` | 43 | Stable moving-target dispatcher used by five examples and public contract checks. |
| `queryAzElTimeObstacle.m` | 46 | Public method-aware query dispatcher used by examples and regression coverage. |
| `validateAzElTrajectory.m` | 43 | Public method-aware validation dispatcher used by examples, sandbox, and regression coverage. |

These files are small because they are interface boundaries. Their job is to
keep callers stable while selecting one complete implementation, not to contain
the implementation itself.

## Shared root internals

| Files | Why they stay separate |
| --- | --- |
| `resolveOptions.m`, `normalizeLogicalScalar.m` | Shared contract primitives used by constructors, plotting, example setup, and geographic helpers. Merging them would reproduce option semantics in many callers. |
| `+obstacles/prepareDynamic.m`, `+geometry/boundaryShape.m` | Plot preparation and canonical polygon conversion. Preparation is also called by the shape adapter. |
| `+obstacles/shapeAtTime.m` | One direct plotter caller, but it owns the complete sampled-history interpolation and topology-change policy. Folding 94 code lines into the already-large plotter would mix geometry interpretation with rendering. |
| `goalPositionAtTime.m` | One tracked plotter caller, but it owns the fixed-versus-sampled goal adapter used by several plotter sections. Keeping one adapter prevents those sections from drifting. |

`shapeAtTime.m` and `goalPositionAtTime.m` are the only short shared files with
one tracked caller. They remain separate because they own data-interpretation
contracts, while `plotAzElMotion.m` owns rendering.

## Corridor method

Every corridor file below is reachable from
`azElPlannerMethods.corridor.plan` or a method-public query/validation entry.
Most have two to ten direct callers.

| Category | Short files | Separation reason |
| --- | --- | --- |
| Geometry | `boundaryShape`, `boundaryToEdges`, `convexPolygonRegions`, `pointPolygonClearance` | One authoritative polygon representation, traversal, decomposition, and signed-clearance vocabulary shared by search, motion, and certification. |
| Obstacles | `buildEnvelopeBoundary`, `prepareDynamic`, `shapeAtTime` | Immutable obstacle preparation and time interpolation are shared across the complete corridor closure. |
| Motion | `evaluatePolynomial` | Canonical evaluator shared by construction, optimization, and independent validation. |
| Validation | `buildSeedCorridor`, `certifySeedCorridor`, `seedCorridorInequality`, `seedEnvelopeContainsObstacles` | Certificate construction and checking remain separate from the optimizer they independently check. |
| Common | `goalPositionAtTime`, `normalizeLogicalScalar`, `powerToBernstein`, `resolveOptions`, `combineObstacles` | Reused contract and mathematical primitives prevent duplicated semantics inside the isolated method. |

Four corridor files have one direct caller and need an explicit decision:

| File | Code lines | Sole caller | Why it is not merged |
| --- | ---: | --- | --- |
| `+internal/emptyAzElPlannerResult.m` | 60 | `plan.m` | Owns the stable success/failure schema used by every early return. The planner orchestrator should not duplicate that schema across branches. |
| `+internal/+motion/buildStraightJerkProfile.m` | 93 | `buildQuinticSpline.m` | A complete closed-form jerk-limited motion algorithm, distinct from spline assembly. |
| `+internal/+search/clusterSeedShape.m` | 75 | `generateTopologySeeds.m` | A self-contained clustering algorithm extracted from an already-large search orchestrator. |
| `+internal/+search/expandDynamicRoute.m` | 43 | `runCorridorPlanner.m` | Owns time-local route expansion and nearest-boundary decisions; its caller is already the corridor candidate orchestrator. |

## HS3 method

Every HS3 short file is reachable from `azElPlannerMethods.hs3.plan` or its
method-public query/validation entry. The files also preserve the exact
`plan-325` dependency boundary, which is required for source-baseline fidelity
and physical folder removal.

| Category | Short files | Separation reason |
| --- | --- | --- |
| Geometry and obstacles | `pointPolygonClearance`, `prepareDynamic` | Shared geometry used by seed construction, HS3, queries, and validation. |
| Motion | `evaluatePolynomial` | One polynomial evaluator shared by analytic motion, HS3, and independent validation. |
| Validation | `buildSeedCorridor`, `seedCorridorInequality`, `seedEnvelopeContainsObstacles` | Shared exact certificate primitives used by the generator and validator. |
| Common | `goalPositionAtTime`, `normalizeLogicalScalar`, `powerToBernstein`, `resolveOptions`, `combineObstacles` | Reused option, goal, conversion, and obstacle contracts inside the isolated HS3 closure. |

Three HS3 files have one direct caller. The jerk objective and corridor
certificate also have direct regression callers, so they are not included in
this sole-caller table:

| File | Code lines | Sole caller | Why it is not merged |
| --- | ---: | --- | --- |
| `+internal/emptyAzElPlannerResult.m` | 58 | `plan.m` | Owns the stable result schema and keeps every expected failure structurally identical. |
| `+internal/+motion/linearizeHs3Constraints.m` | 17 | `solveHs3.m` | Owns the sparse HS3 constraint/Jacobian mapping; inlining would obscure the nonlinear solver setup. |
| `+internal/+search/clusterSeedShape.m` | 89 | `generateTopologySeeds.m` | A distinct clustering algorithm extracted from the largest HS3 search file. |

## Merge decision

No remaining short production file should be merged solely to reduce file
count. The nine single-caller files own a stable schema, a mathematical or
geometry algorithm, or a data-interpretation boundary extracted from an
already-large orchestrator. The other 40 already prevent duplication across
multiple callers.

A future merge is justified only if the owning algorithm or public boundary is
removed and the code becomes genuinely caller-local. File length alone is not
evidence that a boundary is useless.
