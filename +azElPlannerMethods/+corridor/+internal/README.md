# Corridor internal modules

`azElPlannerMethods.corridor.internal` belongs only to the corridor snapshot.
Stable user entry points remain at the repository root. The subpackages make
the pipeline boundaries visible:

```text
obstacles -> search -> motion -> validation
     \          \        /
      +----------geometry
```

- `geometry`: shared polygon construction, edge extraction, convex
  decomposition, and signed-clearance primitives.
- `obstacles`: immutable preparation and time-dependent obstacle geometry.
- `search`: bounded visibility, homology, timed-search, and seed generation.
- `motion`: corridor-constrained spline generation, retiming, and candidate
  selection.
- `validation`: corridor and envelope certificates used independently of
  candidate generation.

Small cross-cutting helpers remain in this directory so the modules do not
depend on an artificial utility hierarchy. Nothing here may call the sibling
HS3 package; deliberate duplication keeps either planner removable.
