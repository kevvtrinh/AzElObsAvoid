# HS3 planner snapshot

This folder is the complete HS3 backend imported from `plan-325` commit
`5a067112a9f880d015f52fb97538a99010871478`. The last HS3
algorithm-and-evidence commit on that history is `921b2f7`.

The backend builds bounded topology proposals, constructs deterministic
analytic first motions when applicable, and may use HS3 nonlinear optimization
to improve or recover a continuous finite-jerk trajectory. It does not call
anything inside the sibling `+corridor` folder. HS3 optimization requires
MATLAB Optimization Toolbox because it uses `fmincon`.

## Entry points

- `plan.m` owns HS3 defaults, endpoint checks, seed generation, analytic
  fallback policy, nonlinear attempts, validation, ranking, and result data.
- `planMovingTargetIntercept.m` preserves the Plan-325 one-call moving-goal
  earliest-arrival behavior.
- `combineObstacles.m`, `normalizeTimeObstacleData.m`, and
  `queryTimeObstacle.m` keep obstacle representation and queries local to this
  snapshot.
- `validateTrajectory.m` is this method's independent complete-trajectory
  validator.

These are backend integration points. Application code should normally call
`planAzElMotion` or `planAzElMovingTargetIntercept` at the repository root.

## Internal flow

```text
canonical inputs
    -> prepared obstacle histories
    -> bounded topology and timing seeds
    -> analytic first motion and/or HS3 solve
    -> independent validation and deterministic selection
```

The `+internal` subpackages separate geometry, obstacle preparation, search,
motion construction, and reusable first-motion certificates. Similar files in
the corridor folder are intentionally not shared.

## Selection and removal

Select this method with `PlannerMethod="hs3"`. Use
`planAzElMotion("hs3")` when method-specific defaults such as collocation and
nonlinear-solver limits are needed.

Setting `EnableHs3Improvement=false` changes HS3's internal attempt policy; it
does not select the corridor planner and does not unplug this backend. To omit
HS3 from a deployment, remove the entire folder and remove the HS3 cases from
both root dispatchers. No automatic fallback is provided.
