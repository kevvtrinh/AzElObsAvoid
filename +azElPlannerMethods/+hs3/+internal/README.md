# HS3 internal modules

`azElPlannerMethods.hs3.internal` belongs only to the Plan-325 HS3 snapshot.
Stable user entry points remain at the repository root. The subpackages keep
the main responsibilities separate:

```text
obstacles -> search -> analytic/HS3 motion -> final validation
     \          \              /
      +----------geometry and certificates
```

- `geometry`: signed polygon-clearance primitives used by HS3 operations.
- `obstacles`: immutable dynamic-obstacle preparation and shape queries.
- `search`: bounded visibility, homology, timed-search, and seed generation.
- `motion`: analytic stop-at-waypoint construction, HS3 propagation,
  constraints, objective evaluation, and nonlinear solving.
- `validation`: seed-corridor and obstacle-envelope certificates used by the
  analytic first-motion path.

Cross-cutting result, option, target, logical, and Bernstein helpers remain in
this directory. Nothing here may call the corridor package; deliberate
duplication preserves source behavior and keeps either planner removable.
