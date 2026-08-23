# Shared public-utility internals

`azElInternal` now contains only implementation details shared by root public
utilities such as obstacle construction, plotting, time queries, and the
compatibility trajectory validator. Neither planner backend calls this package.
Each complete planner closure instead lives under `+azElPlannerMethods` so one
method folder can be removed without breaking the other.

The remaining geometry is intentionally narrow:

- `geometry`: canonical boundary-to-`polyshape` conversion used while plotting
  prepared histories;
- `obstacles`: immutable preparation and shape-at-time interpolation used by
  plotting.

Small option, logical, and goal-position helpers remain at this level because
several public utilities share them. Planner-specific search, motion,
validation, candidate ranking, and result construction were removed from this
package after their callers moved into the method folders.
