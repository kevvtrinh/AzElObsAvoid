# HS3 geometry

This module owns the polygon-clearance primitive used by the HS3 snapshot. It
does not select seeds, solve a trajectory, or decide planner success.

The similar corridor helper is intentionally separate. Each method must keep
the geometry semantics of its source snapshot and remain removable without
leaving a dependency on the other planner.
