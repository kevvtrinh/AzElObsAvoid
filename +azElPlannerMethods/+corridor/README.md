# Corridor-quintic planner snapshot

This folder contains the `corridorQuintic` backend imported from
`325-less-nlp` commit
`28526638886b69efdf6d697a942ad2c1207bcc04`. Its latest planner baseline was
recorded with the implementation and evidence committed at `9dc2530`.

The backend builds bounded spatial and timed route proposals, constructs a
continuous corridor around a selected route, and generates independently
validated quintic motion. It contains no HS3 solver and does not call anything
inside the sibling `+hs3` folder.

## Entry points

- `plan.m` owns defaults, endpoint checks, seed generation, candidate
  selection, and the stable result record.
- `planMovingTargetIntercept.m` preserves the corridor branch's bounded
  chronological search over fixed-arrival planner calls.
- `validateTrajectory.m` is this method's independent complete-trajectory
  validator.

Canonical combination, normalization, option handling, obstacle preparation,
geometry-at-time queries, and time queries are shared through the root
functions and `azElInternal`. The corridor backend does not depend on the HS3
folder.

These are backend integration points. Application code should normally call
`planAzElMotion` or `planAzElMovingTargetIntercept` at the repository root.

## Internal flow

```text
canonical inputs
    -> prepared obstacle histories
    -> bounded topology and timing seeds
    -> corridor-constrained continuous motion
    -> independent validation and deterministic selection
```

The `+internal` subpackages separate geometry, obstacle preparation, search,
motion construction, and reusable certificates. Read their local README files
before moving a helper across a boundary.

## Selection and removal

Select this method with `PlannerMethod="corridorQuintic"`; it is also the
current public default. The method-local `MotionMethod` field must remain
`"corridorQuintic"` and is not the public method selector.

The complete folder can be omitted from a deployment only after the root
dispatchers stop accepting its selector and choose a different default. No
automatic fallback is provided.
