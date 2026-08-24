# Corridor internal modules

`azElPlannerMethods.corridor.internal` contains only compact-specific motion,
dynamic-route expansion, and obstacle-envelope boundary code. Stable user
entry points and shared ownership remain outside this package:

```text
neutral topology and certificates -> compact motion -> canonical validation
```

- `azElInternal`: neutral topology generation, clustering, dense envelopes,
  convex decomposition, corridor certificates, result schemas, and shared
  input, obstacle, geometry, goal, and polynomial contracts.
- `obstacles`: the remaining compact envelope-boundary adapter.
- `search`: the remaining dynamic-route expansion adapter; topology generation
  itself is neutral.
- `motion`: compact C3/C4 spline construction, bounded duration search, and
  candidate selection.
- `validateAzElTrajectory`: the canonical independent validator.

No method-local result builder, convex decomposition, seed-corridor
certificate, topology generator, or validator remains. The compact package
does not call the sibling HS3 package.
