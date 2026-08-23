# HS3 first-motion certificates

This module owns the corridor and obstacle-envelope checks used to determine
whether an analytic first motion has sufficient independent support. Keeping
these checks outside search and motion construction prevents a candidate from
certifying itself solely with its generating assumptions.

The method-level `validateTrajectory.m` remains the final authority for every
returned timed trajectory, including successful HS3 solutions.
