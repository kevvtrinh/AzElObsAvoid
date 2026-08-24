# Shared public-utility internals

`azElInternal` contains implementation details shared by root public utilities
and both planner backends. It is the single owner of canonical obstacle
preparation and interpolation, boundary traversal, signed clearance, option
merging, logical normalization, goal interpolation, polynomial evaluation, and
power-to-Bernstein conversion. Either method folder can still be removed
because both depend only on this neutral layer, never on their sibling.

The remaining geometry is intentionally narrow:

- `geometry`: canonical boundary conversion, deterministic edge traversal, and
  vectorized signed polygon clearance;
- `obstacles`: immutable preparation and shape-at-time interpolation used by
  planning, querying, validation, and plotting.

Small option, logical, goal-position, and polynomial helpers remain at this
level because public utilities and both planner methods share them.
Planner-specific search, motion construction, validation, candidate ranking,
and result construction stay in the method folders.
