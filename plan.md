# Separated Trajectory Engines

## Decision

The repository has two genuine trajectory engines and no neutral trajectory
dispatcher:

- `ruckigEngine.solve` owns exact jerk-switching state-to-state motion.
- `hs3Engine.solve` owns dimension-neutral Hermite-Simpson collocation.
- `obstacleAvoidance.planTrajectory` owns obstacle-aware engine selection.

Neither engine imports obstacle geometry, Az/El terminology, topology search,
or the other engine. There is no root trajectory `.m` wrapper and no shared
internal package above the engines.

## Production Layout

```text
+obstacleAvoidance/
|-- planTrajectory.m
|-- planMovingTargetIntercept.m
|-- validateTrajectory.m
|-- +input/
|-- +obstacles/
|-- +geometry/
|-- +search/
|-- +planner/
`-- +plotting/

trajectory/
|-- +ruckigEngine/
|   |-- solve.m
|   |-- defaultOptions.m
|   |-- checkEligibility.m
|   |-- exact switching-profile functions
|   `-- +internal/
|       |-- normalizeRequest.m
|       |-- createEmptyResult.m
|       |-- evaluatePolynomial.m
|       |-- evaluatePolynomialConstraints.m
|       `-- validateResult.m
|-- +hs3Engine/
|   |-- solve.m
|   |-- defaultOptions.m
|   |-- optimize.m
|   |-- solveFixedTime.m
|   |-- solveFreeTime.m
|   |-- validate.m
|   |-- +constraints/
|   `-- +polynomial/
`-- THIRD_PARTY_NOTICES.txt
```

## Direct Engine Calls

```matlab
ruckigResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, ruckigOptions);

hs3Result = hs3Engine.solve( ...
    initialState, terminalState, limits, hs3Options, pathConstraints);
```

Each engine resolves its own options, normalizes its own inputs, validates its
own result, and reports expected infeasibility as a failure result. Direct
results do not contain dispatcher provenance or fallback fields.

## Obstacle-Planner Routing

The obstacle planner makes its choice only after normalizing the Az/El request
and rejecting invalid or occupied endpoints.

Ruckig is attempted when all of the following are true:

1. The obstacle field is empty.
2. The target is a fixed position rather than a sampled moving target.
3. The request uses symmetric derivative limits supported by Ruckig.
4. No affine path constraints are needed.

The public Az/El goal-time policy maps as follows:

| Obstacle option | Ruckig option |
| --- | --- |
| `earliestArrival` | `earliestArrival` |
| `fixedArrival` | `fixed` with `FinalTime = goalState.time_s` |

If Ruckig reports `unsupportedSwitchingFamily`, the obstacle planner continues
through topology creation and HS3. An identified physical infeasibility or an
independent-validation failure remains visible and is not hidden by another
solve.

Every nonempty obstacle field and every moving target uses topology search and
HS3. Ruckig never sees obstacle data.

## Result Translation

The engines keep their dimension-neutral field names. The obstacle planner
translates a successful Ruckig polynomial locally into the existing Az/El
fields, then runs `obstacleAvoidance.validateTrajectory`. This local translation
is not a reusable trajectory wrapper and is not exposed as another public API.

The obstacle planner returns its existing fields on every exit, including:

- status, message, and termination reason;
- normalized inputs and resolved options;
- route seeds and per-seed engine diagnostics;
- sampled position, velocity, acceleration, and jerk;
- polynomial data in Az/El units;
- independent collision and kinematic validation;
- exclusive stage timing and total elapsed time.

## Performance Evidence

Before removal, the warm neutral dispatcher added an approximately constant
0.4--0.6 ms per Ruckig call in controlled tests:

| Request | Direct Ruckig median | Dispatcher median | Added time |
| --- | ---: | ---: | ---: |
| short 1-D | 3.212 ms | 3.807 ms | 0.595 ms |
| moderate 2-D | 1.730 ms | 2.288 ms | 0.558 ms |
| synchronized 6-D | 27.554 ms | 27.945 ms | 0.391 ms |
| synchronized 12-D | 59.219 ms | 59.629 ms | 0.411 ms |

The constant cost was material for millisecond-scale Ruckig calls, which is why
the dispatcher and its duplicate normalization were removed.

## Verification Gates

The architecture is complete only when all of these remain true:

- trajectory root contains only the two engine packages and notices;
- Ruckig source contains no HS3, optimizer, obstacle, or planner dependency;
- HS3 source contains no exact switching functions or Az/El dependency;
- obstacle-free fixed-target requests route through Ruckig and pass canonical
  Az/El validation;
- obstacle and moving-target requests continue through search and HS3;
- direct Ruckig and direct HS3 tests pass;
- full repository tests pass;
- maintained examples pass serial headless verification;
- `benchmark.csv`, `verification.md`, and `branch_assessment.md` record the
  measured result and retain unfavorable limitations.
