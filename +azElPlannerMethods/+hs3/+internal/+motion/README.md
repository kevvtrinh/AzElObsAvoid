# HS3 motion

This module owns the remaining HS3 collocation, jerk propagation, constraints,
objective evaluation, reconstruction, diagnostics, and `fmincon` solve. The
former deterministic stop-at-waypoint family has been removed; the immutable
first motion now comes from the compact baseline.

`hs3.improve` decides attempt order and candidate acceptance. The solver
consumes prepared obstacles and compact seeds; it does not construct scenario
geometry or relax canonical validation. Its deadline is cooperative: setup and
output-callback checks stop future work, but an active solver evaluation can
finish after the requested time and cause measured overrun. Mesh refinement is
not implemented by the current improver.
