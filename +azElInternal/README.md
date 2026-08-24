# Internal planner modules

`azElInternal` is implementation-only. Stable user entry points remain at the
repository root. The subpackages enforce the pipeline boundaries:

```text
obstacles -> search -> motion
     \          \        /
      +----------geometry
```

- `geometry`: shared polygon construction, edge extraction, convex
  decomposition, and signed-clearance primitives.
- `obstacles`: immutable preparation and time-dependent obstacle geometry.
- `search`: bounded visibility, homology, timed-search, and seed generation.
- `motion`: spline generation, retiming, and candidate selection.

Small cross-cutting helpers, including corridor and envelope certificates used
by independent validation, remain in this directory so the modules do not
depend on an artificial utility hierarchy.
