# Corridor certificates

This module owns reusable corridor and obstacle-envelope certificates. These
checks remain separate from search and spline generation so a candidate cannot
validate itself by repeating only the assumptions used to construct it.

The method-level `validateTrajectory.m` still performs final, complete timed
trajectory validation. These helpers provide the additional corridor and
envelope facts used while building and screening candidates.
