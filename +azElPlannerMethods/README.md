# Az/El planner methods

This package keeps the two production planning algorithms isolated. Users call
the public functions at the repository root; those functions normalize shared
obstacle inputs once, then select one method folder.

## Source snapshots

| Folder | `PlannerMethod` value | Exact source snapshot | Preserved behavior |
| --- | --- | --- | --- |
| `+corridor` | `"corridorQuintic"` | Originated at `325-less-nlp` `28526638886b69efdf6d697a942ad2c1207bcc04` | Visibility and timed seeds followed by compact C3/C4 quintic motion. Its moving-target adapter performs bounded chronological fixed-arrival trials. |
| `+hs3` | `"hs3"` | `plan-325` at `5a067112a9f880d015f52fb97538a99010871478` | Analytic first motions plus optional HS3 nonlinear improvement. Its moving-target adapter makes one moving-goal earliest-arrival planner call. |

The last planner/evidence commits before those tips were `9dc2530` for the
corridor snapshot and `921b2f7` for HS3. Integration changes include package
qualification, mechanically necessary names, shared behavior-equivalent
contract and obstacle helpers, and the comment and layout style used by
`325-less-nlp`. Numeric constants, solver decisions, candidate order, and
validation rules belong to the method packages. The corridor motion stack has
since been replaced by the compact implementation; the HS3 package retains its
imported solver until its separate replacement gate is satisfied.

## Select a planner

`corridorQuintic` is the backward-compatible default:

```matlab
options = planAzElMotion();
result = planAzElMotion(obstacles, initialState, goalState, limits, options);
```

Request HS3 explicitly:

```matlab
options = planAzElMotion("hs3");
result = planAzElMotion(obstacles, initialState, goalState, limits, options);
```

You can also set the selector on a partial options structure:

```matlab
options = struct("PlannerMethod", "hs3", "MaximumSeedCount", 3);
result = planAzElMotion(obstacles, initialState, goalState, limits, options);
```

For moving-target interception, put the selector inside `PlannerOptions`:

```matlab
interceptOptions = struct( ...
    "InterceptMode", "earliest", ...
    "PlannerOptions", struct("PlannerMethod", "hs3"));

result = planAzElMovingTargetIntercept( ...
    obstacles, initialState, targetMotion, limits, interceptOptions);
```

The returned `Options.PlannerMethod` and
`SearchDiagnostics.PlannerMethod` identify the method that actually ran.

`PlannerMethod` is the two-planner selector. The corridor snapshot's
`MotionMethod="corridorQuintic"` field is retained only for compatibility;
it does not select HS3.

## Folder map

```text
+azElPlannerMethods
|-- +corridor
|   |-- plan.m                         complete corridor planner
|   |-- planMovingTargetIntercept.m    corridor target-time policy
|   |-- validateTrajectory.m           corridor-specific validation
|   `-- +internal
|       |-- +geometry
|       |-- +obstacles
|       |-- +search
|       |-- +motion
|       `-- +validation
`-- +hs3
    |-- plan.m                         complete HS3 planner
    |-- planMovingTargetIntercept.m    HS3 moving-goal policy
    |-- validateTrajectory.m           HS3-specific validation
    `-- +internal
        |-- +search
        |-- +motion
        `-- +validation
```

Canonical obstacle combination and normalization live at the repository root.
Time queries, obstacle preparation and interpolation, boundary traversal,
signed clearance, option merging, logical normalization, goal interpolation,
polynomial evaluation, and power-to-Bernstein conversion live in
`+azElInternal`. A method must not call its sibling's search, solver, validator,
or result builder. This shares input meaning without coupling either planning
algorithm to the other method folder.

## Safely unplug a method

There is no automatic method registry and no silent fallback. Removing a
folder therefore also requires a small explicit dispatcher edit.

To remove HS3:

1. Remove or omit the complete `+hs3` folder.
2. Remove the `"hs3"` normalization and switch cases from both public
   dispatchers.
3. Remove HS3-specific tests and documentation from the deployment.
4. Keep `corridorQuintic` as the default.

To remove the corridor method:

1. Remove or omit the complete `+corridor` folder.
2. Change both public defaults to `"hs3"`.
3. Remove the corridor switch cases and its `MotionMethod` compatibility
   handling.
4. Update the zero-input moving-target defaults, which currently come from
   the corridor adapter.

Do not leave a removed method's selector accepted, and do not catch a missing
method error by running the other planner. An explicit failure is safer than
silently returning a trajectory from a method the user did not request.
