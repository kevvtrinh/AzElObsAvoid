# Why the production MATLAB files below 100 code lines remain separate

This audit counts nonblank, noncomment MATLAB lines in production files. Tests,
examples, benchmarks, and scratch files are excluded from the file set, but
their calls are included when they demonstrate that a production interface is
used. Textual call sites were checked across every MATLAB file in the
repository.

The result is concrete: all 22 short production files are called, 18 have
multiple callers, and four have one caller. None is being kept as an unowned or
unused script.

| File | Code lines | Textual callers | Why it stays separate |
|---|---:|---:|---|
| `combineAzElObstacles.m` | 71 | 13 | Public container-normalization API used by planner, validation, queries, examples, and benchmarks. Merging would duplicate the same input contract. |
| `makeAzElObstacleData.m` | 28 | 20 | Public obstacle constructor with a stable user-facing contract and broad use across fixtures and examples. |
| `+azElInternal/emptyAzElPlannerResult.m` | 60 | 1 | Owns the stable success/failure result schema. Keeping schema initialization outside `planAzElMotion` makes early exits consistent and keeps the public planner smaller. |
| `+azElInternal/goalPositionAtTime.m` | 8 | 5 | Shared fixed-goal/moving-goal adapter used by planning, validation, plotting, and seed generation. |
| `+azElInternal/normalizeLogicalScalar.m` | 8 | 14 | Shared option-contract primitive; merging it would reproduce subtly different logical validation in many public and internal functions. |
| `+azElInternal/powerToBernstein.m` | 16 | 2 | Shared mathematical conversion used by optimization and independent certification. Its degree-specific matrix cache belongs behind one implementation. |
| `+azElInternal/resolveOptions.m` | 17 | 12 | Shared partial-override/default-resolution contract used by public and internal APIs. |
| `+azElInternal/+geometry/boundaryShape.m` | 8 | 2 | One representation boundary for converting repository boundary arrays into `polyshape`; both obstacle preparation and time queries depend on identical behavior. |
| `+azElInternal/+geometry/boundaryToEdges.m` | 30 | 2 | Shared NaN-separated-ring and closure logic used by clearance and topology search. Separate ownership prevents edge-order and tolerance drift. |
| `+azElInternal/+geometry/convexPolygonRegions.m` | 29 | 3 | Shared exact convex decomposition used by motion construction and two independent corridor checks. |
| `+azElInternal/+geometry/pointPolygonClearance.m` | 44 | 9 | Core signed-clearance primitive shared across search, motion, public queries, tests, and independent validation. |
| `+azElInternal/+motion/buildStraightJerkProfile.m` | 93 | 1 | A complete closed-form jerk-limited motion algorithm, including its timing-law helper. Folding it into `buildQuinticSpline` would mix two motion methods and enlarge an already 486-line file. |
| `+azElInternal/+motion/evaluatePolynomial.m` | 54 | 4 | Canonical evaluator for the piecewise-polynomial motion representation, shared by motion construction, retiming, compact optimization, and independent validation. |
| `+azElInternal/+obstacles/buildEnvelopeBoundary.m` | 27 | 2 | Owns conservative time-independent envelope construction used by two different motion paths. |
| `+azElInternal/+obstacles/prepareDynamic.m` | 75 | 7 | Central prepared-obstacle boundary used by planning, plotting, queries, tests, and independent validation; merging would restore repeated canonicalization logic. |
| `+azElInternal/+obstacles/shapeAtTime.m` | 94 | 11 | Authoritative time interpolation and topology-change handling used throughout search, motion, plotting, queries, tests, and validation. |
| `+azElInternal/+search/clusterSeedShape.m` | 75 | 1 | A self-contained clustering algorithm called by the 898-line topology generator. Merging it would make the largest search file harder to navigate without removing any logic. |
| `+azElInternal/+search/expandDynamicRoute.m` | 43 | 1 | Owns time-local route-clearance adjustment and its nearest-boundary helper. Its caller, `runCorridorPlanner`, is already 847 lines and should not absorb another algorithm. |
| `+azElInternal/+validation/buildSeedCorridor.m` | 49 | 2 | Corridor construction is used by production motion solving and directly exercised by regression tests; it remains separate from certification by design. |
| `+azElInternal/+validation/certifySeedCorridor.m` | 63 | 3 | Independent certificate composition used by the solver, public validation, and tests. It must not be hidden inside the optimizer it is meant to check. |
| `+azElInternal/+validation/seedCorridorInequality.m` | 18 | 2 | Shared exact inequality assembly used by both trajectory optimization and independent certification. |
| `+azElInternal/+validation/seedEnvelopeContainsObstacles.m` | 34 | 3 | Shared envelope-containment check used by solver, certificate composition, and focused tests. |

## Merge decision

No short production file should be merged solely to reduce file count. The four
single-caller files are deliberate extractions from large orchestrators or own
a stable result schema. The other 18 already prevent duplication across
multiple callers. A future merge would be justified only if the owning
algorithm disappears or its contract becomes genuinely caller-local—not
because the implementation happens to fit under 100 lines.
