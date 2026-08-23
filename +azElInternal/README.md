# Shared public-utility internals

`azElInternal` contains implementation details shared by root public utilities
and both planner backends. It owns canonical obstacle interpolation, boundary
traversal, signed clearance, and small contract helpers. Either method folder
can still be removed because both depend only on this neutral layer, never on
their sibling.

The remaining geometry is intentionally narrow:

- `geometry`: canonical boundary conversion, deterministic edge traversal, and
  vectorized signed polygon clearance;
- `obstacles`: immutable preparation and shape-at-time interpolation used by
  planning, querying, validation, and plotting.

Small option, logical, and goal-position helpers remain at this level because
several public utilities share them. Planner-specific search, motion,
validation, candidate ranking, and result construction were removed from this
package after their callers moved into the method folders.
