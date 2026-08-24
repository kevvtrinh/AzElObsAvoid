# Compact corridor-quintic baseline

This folder owns the `corridorQuintic` compact baseline. It originated from
`325-less-nlp` commit `28526638886b69efdf6d697a942ad2c1207bcc04`; its former
multi-path motion stack has been replaced by compact C3/C4 candidate generation
plus a small exact direct quintic.

`plan.m` resolves compact options, consumes neutral bounded topology seeds,
constructs compact motion, calls the canonical validator, and selects the
earliest independently valid candidate. `validateTrajectory.m` is only a
compatibility facade over root `validateAzElTrajectory`.

There is no method-local moving-target adapter, topology generator, result
builder, convex decomposition, seed-corridor certificate, or final validator.
Those responsibilities are owned by root `planAzElMovingTargetIntercept`,
`azElInternal`, and root `validateAzElTrajectory`.

The remaining internal code owns compact-specific motion construction, one
dynamic-route expansion adapter, and one obstacle-envelope boundary adapter:

```text
canonical inputs
    -> neutral topology and corridor helpers
    -> compact C3/C4 continuous motion
    -> canonical independent validation and deterministic selection
```

Select this method with `PlannerMethod="corridorQuintic"`; it is also the
public default. `MotionMethod="corridorQuintic"` is a compatibility field, not
the public method selector. The compact package does not depend on HS3.
