# Az/El planner compositions

Users call the public functions at the repository root. This package now owns
one standalone compact planner and one bounded composite improvement layer:

| Folder | `PlannerMethod` | Current responsibility |
| --- | --- | --- |
| `+corridor` | `"corridorQuintic"` | Build the validated compact C3/C4 baseline from neutral topology and certificate helpers. |
| `+hs3` | `"hs3"` | Resolve HS3 controls and optionally improve an immutable compact result with bounded HS3 work. |

The source histories remain useful provenance: corridor originated at
`325-less-nlp` commit `28526638886b69efdf6d697a942ad2c1207bcc04`,
and HS3 originated at `plan-325` commit
`5a067112a9f880d015f52fb97538a99010871478`. The current composition no longer
preserves their former duplicate adapters, search stacks, result builders, or
validators.

## Select a composition

`corridorQuintic` is the backward-compatible default:

```matlab
options = planAzElMotion();
result = planAzElMotion(obstacles, initialState, goalState, limits, options);
```

Request the compact-baseline-plus-HS3 composition explicitly:

```matlab
options = planAzElMotion("hs3");
result = planAzElMotion(obstacles, initialState, goalState, limits, options);
```

`EnableHs3Improvement` defaults to `false`. With those defaults,
`Options.PlannerMethod` is `"hs3"` while `SelectedMotionSource` remains
`"corridorQuintic"`. Set the option true to permit bounded HS3 trials;
`SelectedMotionSource` becomes `"hs3"` only if canonical validation and the
strict monotone comparison accept a candidate.

For moving-target interception, put the selector inside `PlannerOptions`.
The root `planAzElMovingTargetIntercept` owns one bounded chronological
fixed-time policy for both selections; there are no backend intercept adapters.

`PlannerMethod` is the public selector. The compact defaults retain
`MotionMethod="corridorQuintic"` for compatibility. The root dispatcher removes
that compatible field when a copied compact defaults record is switched to
HS3; `MotionMethod` never selects HS3.

## Ownership map

```text
+azElPlannerMethods
|-- +corridor
|   |-- plan.m                         compact baseline orchestrator
|   |-- validateTrajectory.m           canonical-validator facade
|   `-- +internal
|       |-- +obstacles                 envelope boundary adapter
|       |-- +search                    dynamic-route adapter
|       `-- +motion                    compact C3/C4 construction
`-- +hs3
    |-- plan.m                         compatibility facade
    |-- resolvePlannerOptions.m         HS3 options + compact projection
    |-- improve.m                      immutable-baseline improver
    |-- validateTrajectory.m            canonical-validator facade
    `-- +internal/+motion              HS3 NLP implementation
```

`+azElInternal` owns shared topology generation, dense envelopes, clustering,
convex decomposition, seed-corridor construction and certification, result
schemas, obstacle queries, polynomial utilities, and improvement comparison.
The root `validateAzElTrajectory` is the single final validator. Method-qualified
validation functions are compatibility facades only.

## Dependency and removal boundary

The compact method does not depend on HS3. The reverse is intentionally not yet
true: selecting HS3 in `planAzElMotion` first calls the corridor compact planner,
then passes its immutable result to `hs3.improve`. This composition prevents a
failed or inferior nonlinear attempt from regressing a validated compact result,
but it means the current HS3 path transitively depends on `+corridor` pending
neutral extraction of the compact baseline.

The HS3 package is capped at 1,200 noncomment lines and currently owns exactly
1,200. That count excludes the corridor baseline and neutral shared
dependencies, so it is not a whole-executed-closure size claim. Do not describe
HS3 as independently removable or test it by deleting corridor in the current
composition.

Removing HS3 still requires deleting its selector, facade, tests, and
documentation. Removing corridor is unsupported while HS3 composes it. There is
no automatic discovery or silent fallback.
