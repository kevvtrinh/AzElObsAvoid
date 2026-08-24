# HS3 internal modules

`azElPlannerMethods.hs3.internal` belongs only to the Plan-325 HS3 snapshot.
Stable user entry points remain at the repository root. The subpackages keep
the main responsibilities separate:

```text
shared obstacles -> search -> analytic/HS3 motion -> final validation
       \              \              /
        +--------------geometry and certificates
```

- `azElInternal`: shared option, goal, dynamic-obstacle, and signed-clearance
  contracts used by both planners and root utilities.
- `search`: bounded visibility, homology, timed-search, and seed generation.
- `motion`: analytic stop-at-waypoint construction, HS3 propagation,
  constraints, objective evaluation, and nonlinear solving.
- `validation`: seed-corridor and obstacle-envelope certificates used by the
  analytic first-motion path.

Cross-cutting result, candidate, and timing helpers remain in this directory.
Nothing here may call the corridor package. Method-specific search, motion
construction, certification, and validation remain isolated so either planner
stays removable.
