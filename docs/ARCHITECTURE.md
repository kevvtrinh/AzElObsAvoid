# Architecture

## Public boundary

`planAzElAvoidance(request)` is the only planning entry point. It accepts
canonical obstacles, one complete initial state, a fixed or sampled moving
complete-state goal, physical limits, and mission options. Search machinery is
private.

The returned command is a sequence of shared azimuth/elevation knot states.
Each adjacent pair uniquely defines a quintic Hermite segment from endpoint
position, velocity, and acceleration. Position and velocity are continuous;
the current construction is also acceleration-continuous at internal knots.

## Planning flow

1. Normalize canonical inputs without changing their public schema; collapse
   fully repeated static histories to equivalent endpoint samples internally.
2. Establish endpoint facts and an obstacle-free synchronized lower bound.
3. Measure mission extent, obstacle feature size and cadence, safety margin,
   seam proximity, dynamics, and horizon.
4. Derive a deterministic coarse-to-fine spatial and temporal schedule.
5. Generate route seeds from adaptive geometry extrema, offset polygon
   boundaries, scene envelopes, and distinct directional topologies.
6. Build a sparse provisional visibility graph directly from canonical
   geometry.
7. Search routes, arrival times, motion durations, and feasible departure
   holds with bounded per-route work so one blocked topology cannot starve
   alternatives.
8. Construct a complete command: a constant-jerk S-curve for direct
   rest-to-rest motion, or quintic Hermite segments with shared interior turn
   velocity for general routes.
9. Reject obvious motion-limit failures, then invoke the independent validator.
10. Retain the best validated incumbent across every refinement level.

Provisional graph checks can reject an obviously blocked seed, but they never
certify the returned command. Only `validateAzElCommand` can do that.

## Adaptation

The caller never chooses a grid, sample count, refinement depth, candidate
density, collision sampling, or smoothing control. The planner derives private
work from:

- start-to-goal and domain extent;
- smallest observed polygon edge;
- requested safety margin;
- independent obstacle sample cadence and detected geometry motion;
- velocity and acceleration limits;
- goal motion and mission horizon; and
- the remaining single wall-time budget and current incumbent.

Diagnostics record selected spatial/temporal scales, graph size, routes,
candidates, validation failures, incumbent changes, elapsed work, and stopping
reason. Diagnostics are outputs, never reusable scenario hints.

## Time semantics

Compatible obstacle samples use linear corresponding-vertex interpolation.
Incompatible topology uses the conservative union of both adjacent samples.
Geometry is held before the first and after the last sample. Temporal padding
unions the center and both padded-time geometries.

Moving-goal complete states define quintic Hermite motion between supplied
samples. Capture is the first time position, velocity, and acceleration all
match. Optional trailing follows the same goal segments and is excluded from
arrival metrics.

## Failure semantics

- Invalid contracts return `status="invalid"`.
- An occupied initial state, permanently occupied static terminal state, or an
  obstacle-free lower bound beyond the deadline can be proved infeasible.
- Search exhaustion, insufficient refinement, and wall-time exhaustion return
  `status="unknown"`.
