# Corridor motion

This module turns geometric and timed seeds into continuous C3 quintic motion,
retimes candidates, and selects the earliest independently valid result. It
consumes prepared obstacles and validation certificates; it does not construct
scenario geometry.

Straight obstacle-free requests may use the exact jerk-switching profile.
Multi-segment requests use the corridor-constrained spline path. Neither path
invokes HS3 or `fmincon`.
