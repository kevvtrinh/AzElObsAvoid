# HS3 internal modules

`azElPlannerMethods.hs3.internal` contains the nonlinear Hermite-Simpson motion
implementation and its solver diagnostics. The standalone flow is:

```text
neutral topology proposal -> HS3 transcription and solve -> canonical validation
```

- `motion`: HS3 propagation, exact fixed-time affine constraints, nonlinear
  trajectory constraints, objective derivatives, cooperative timeout,
  reconstruction, and solver diagnostics.
- `azElInternal`: neutral request, topology, corridor-certificate, result,
  geometry, obstacle, and polynomial contracts.
- `validateAzElTrajectory`: the one final independent validator.

`hs3.plan` owns proposal ordering, collision relinearization, mesh refinement,
validation, and candidate selection.
