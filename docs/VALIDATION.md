# Independent Validation

Command construction does not certify itself. `validateAzElCommand` rebuilds
the continuous motion from public knot states and independently checks each
acceptance gate.

## Exact motion-limit checks

For each axis and quintic segment, the validator finds real polynomial roots
inside the segment to evaluate:

- position at endpoints and all roots of velocity;
- velocity at endpoints and all roots of acceleration; and
- acceleration at endpoints and all roots of jerk.

This establishes continuous position, velocity, and acceleration extrema up to
floating-point root tolerance. It is not a waypoint-only or plotted-sample
check.

## Continuous collision certificate

Canonical polygons are evaluated at the command interval endpoints and
midpoint. The validator computes signed Euclidean distance to every polygon
edge, including interiors, vertices, disconnected regions, safety margin,
azimuth copies when wrapping, and temporal padding.

For a subinterval of duration `dt`, every instant is at most `dt/4` from an
endpoint-or-midpoint sample. The validator bounds possible unseen clearance
loss by

```text
(maximum command speed + polygon vertex speed) * dt / 4.
```

If sampled clearance minus that bound exceeds the safety margin and numerical
tolerance, the full subinterval is certified. Otherwise it is recursively
subdivided. A checked point below the margin is a collision. Exhausting the
private numerical depth without either result is unresolved and therefore
cannot produce planner success.

Canonical obstacle sample times and temporal-padding offsets split validation
intervals so piecewise time semantics are never crossed silently.

## Other gates

- finite strictly increasing command timestamps;
- complete initial and terminal state error;
- wrapped-display equivalence to continuous azimuth;
- explicit stationary holds with zero rate and acceleration;
- first complete-state arrival excluding trailing; and
- deterministic command equality regressions.

## Claim boundary

Continuous safety and motion validation proves feasibility under the documented
polygon interpolation and quintic command models. It does not prove that an
obstructed command is globally minimum-arrival. Those results remain `Best
found` unless a future independent global certificate closes the lower-bound
gap.
