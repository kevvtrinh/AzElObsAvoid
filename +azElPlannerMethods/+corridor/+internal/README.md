# Corridor internal modules

`azElPlannerMethods.corridor.internal` belongs only to the corridor snapshot.
Stable user entry points remain at the repository root. The subpackages make
the pipeline boundaries visible:

```text
shared obstacles -> search -> motion -> validation
       \              \        /
        +--------------geometry
```

- `azElInternal`: shared option, goal, dynamic-obstacle, and signed-clearance
  contracts used by both planners and root utilities.
- `geometry`: corridor-specific convex decomposition.
- `obstacles`: corridor-specific obstacle-envelope construction.
- `search`: bounded visibility, homology, timed-search, and seed generation.
- `motion`: compact C3/C4 spline construction, bounded duration search, and
  candidate selection.
- `validation`: corridor and envelope certificates used independently of
  candidate generation.

Small method-specific helpers remain in this directory so the modules do not
depend on an artificial utility hierarchy. Nothing here may call the sibling
HS3 package; method-specific duplication keeps either planner removable.
