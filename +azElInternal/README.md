# Internal planner modules

`azElInternal` is implementation-only. Stable user entry points remain at the
repository root. The subpackages enforce the pipeline boundaries:

```text
obstacles -> search -> motion -> validation
     \          \        /
      +----------geometry
```

- `geometry`: shared polygon construction, edge extraction, convex
  decomposition, and signed-clearance primitives.
- `obstacles`: immutable preparation and time-dependent obstacle geometry.
- `search`: bounded visibility, homology, timed-search, and seed generation.
- `motion`: corridor construction, spline generation, retiming, and candidate
  selection.
- `validation`: corridor and envelope certificates used independently of
  candidate generation.

Small cross-cutting helpers remain in this directory so the modules do not
depend on an artificial utility hierarchy.
