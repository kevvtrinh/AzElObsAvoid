# HS3 motion

This module owns both motion families used by Plan-325:

- deterministic stop-at-waypoint motion used as an analytic first candidate;
- HS3 collocation, jerk propagation, constraints, objective evaluation, and
  `fmincon` solving used for bounded nonlinear improvement or recovery.

The method-level planner decides attempt order and candidate ranking. This
module consumes prepared obstacles and seeds; it does not construct scenario
geometry or relax final validation.
