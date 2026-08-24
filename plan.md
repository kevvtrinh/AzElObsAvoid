# Organic-obstacle planner robustness

## Objective

Keep compact quintic and standalone HS3 separate while improving deterministic
convex, concave, rough U-shaped, and moving-obstacle behavior. Preserve complete
validation, nonzero endpoint support, the 2,000-line HS3 limit, and the two-minute
hairpin gate.

## Retained changes

- Compact tries every bounded topology and includes the full feasible duration
  bracket, removing the seed-4108 false infeasibility.
- Two-point seeds use the same exact analytic motion in static and moving scenes.
- Timed topology generation retains one graph from the route-nearest obstacle's
  history as well as the combined graph. All edges and final trajectories still
  check the complete obstacle set.
- Dense fallback geometry builds and unions one conservative support envelope per
  obstacle, so unrelated histories cannot bridge free space.
- HS3 ranks detours by estimated effort, shares remaining work across detours,
  performs one topology-preserving refinement, and defaults to two mesh passes.
- The organic benchmark and the reversed-order two-mover regression preserve the
  motivating evidence.

## Verification evidence

- Organic benchmark, seeds 4101:4108: 32/32 compact/HS3 method-runs independently
  valid; zero compact false infeasibilities; zero HS3 obstacle-addition reversals.
- Randomized similar-size two-mover campaign, seeds 5201:5208: 8/8 base and added
  motions valid; maximum route deviation after adding the independently clear
  mover was 3.52e-11 deg, reduced from 0.430 deg.
- HS3 hairpin passed twice under two minutes: 112.845 s and 112.882 s wall time,
  about 138.611 s arrival versus compact's frozen 140.561 s arrival.
- Full test suite: 141 passed, 0 failed, 0 incomplete, 591.091 s wall time.

## Known limitation

- Organic seed 4108 still shows a 0.588 s compact arrival improvement when a new
  obstacle nearly touches the original path. The original path remains valid with
  only 0.00293 deg clearance; the extra obstacle changes the local sampled-barrier
  QP basin. Obstacle-independent support nodes, exhaustive visibility, waypoint
  pruning, and broad static clearance expansion were tested and reverted because
  they regressed other deterministic cases. This is not the independently clear
  moving-obstacle regression fixed above.

## Working tree scope

- Preserve unrelated untracked `docs/` and `tmp/`.
