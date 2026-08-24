# HS3 internal modules

`azElPlannerMethods.hs3.internal` now contains only the nonlinear HS3 motion
implementation and its solver diagnostics. Stable user entry points and the
compact-baseline composition remain at the repository root:

```text
immutable compact result -> optional HS3 solve -> canonical validation
```

- `motion`: HS3 propagation, constraints, objective evaluation, cooperative
  timeout callback, reconstruction, and nonlinear solving.
- `azElInternal`: neutral topology, corridor certificates, result schemas,
  geometry, obstacle queries, polynomial utilities, and improvement comparison.
- `validateAzElTrajectory`: the one final independent validator.

No method-local stop-motion, search, certificate, result, or validation
implementation remains. The HS3 internals do not call corridor directly, but
the root HS3 composition does because it obtains the immutable compact baseline
before calling `hs3.improve`.
