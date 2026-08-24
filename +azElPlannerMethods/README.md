# Az/El planner methods

Users call the public functions at the repository root. This package owns two
separate planners behind the same result contract:

| Folder | `PlannerMethod` | Current responsibility |
| --- | --- | --- |
| `+corridor` | `"corridorQuintic"` | Build validated compact C3/C4 motion from neutral topology and certificate helpers. |
| `+hs3` | `"hs3"` | Build validated Hermite-Simpson motion with an independent collocation and nonlinear-solver path. |

The source histories remain useful provenance: corridor originated at
`325-less-nlp` commit `28526638886b69efdf6d697a942ad2c1207bcc04`,
and HS3 originated at `plan-325` commit
`5a067112a9f880d015f52fb97538a99010871478`. Shared request, topology,
obstacle, result, and validation contracts have since been extracted to neutral
owners without combining the two motion methods.

## Select a method

`corridorQuintic` is the backward-compatible default:

```matlab
options = planAzElMotion();
result = planAzElMotion(obstacles, initialState, goalState, limits, options);
```

Request standalone Hermite-Simpson explicitly:

```matlab
options = planAzElMotion("hs3");
result = planAzElMotion(obstacles, initialState, goalState, limits, options);
```

An HS3 success always contains motion produced by the HS3 transcription and
sets `SelectedMotionSource="hs3"`. It does not execute, seed from, fall back to,
or compare against the compact planner. The compact method likewise contains
no HS3 call.

For moving-target interception, put the selector inside `PlannerOptions`.
The root `planAzElMovingTargetIntercept` owns one bounded chronological
fixed-time policy for both methods; there are no backend intercept adapters.

`PlannerMethod` is the public selector. The compact defaults retain
`MotionMethod="corridorQuintic"` for compatibility; `MotionMethod` never
selects HS3.

## Ownership map

```text
+azElPlannerMethods
|-- +corridor
|   |-- plan.m                         compact planner orchestrator
|   |-- validateTrajectory.m           canonical-validator facade
|   `-- +internal
|       |-- +obstacles                 envelope boundary adapter
|       |-- +search                    dynamic-route adapter
|       `-- +motion                    compact C3/C4 construction
`-- +hs3
    |-- plan.m                         standalone HS3 orchestrator
    |-- resolvePlannerOptions.m         HS3-only option owner
    |-- validateTrajectory.m            canonical-validator facade
    `-- +internal/+motion              HS3 transcription and NLP solver
```

`+azElInternal` owns shared request normalization, endpoint checks, topology
generation, dense envelopes, clustering, convex decomposition, seed-corridor
certification, result schemas, obstacle queries, and polynomial utilities. The
root `validateAzElTrajectory` is the single final validator.

## Dependency and size boundaries

Neither method package depends on the other. Both depend on neutral shared
contracts and the canonical validator, so deleting one motion method does not
silently substitute the remaining method. Removing a method still requires
removing its public selector, tests, examples, and documentation.

HS3-owned production MATLAB is capped at 2,000 nonblank, noncomment lines. The
current standalone package remains below that cap; tests, benchmarks, neutral
shared infrastructure, and the separately selectable compact planner are not
part of the HS3 ownership count.
