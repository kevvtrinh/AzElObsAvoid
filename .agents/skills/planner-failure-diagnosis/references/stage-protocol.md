# Planner Stage Diagnosis Protocol

Apply only the sections on the failing execution path.

## Failure Classes

Use one primary class:

```text
INPUT, OBSTACLE, SEED, TOPOLOGY, GEOMETRY, MOTION_CONSTRUCTION,
TIMING, KINEMATICS, COLLISION, ENDPOINT, VALIDATION, RANKING,
PERFORMANCE_LIMIT, UNSUPPORTED_CONTRACT, INTERNAL_ERROR, UNKNOWN
```
## Boundary Evidence

- **Input:** fields, shapes, units, time order, workspace, goal policy, terminal
  state, wrapping, and safety-margin policy.
- **Obstacle:** original versus protected geometry, time history, interpolation,
  topology, provenance, and exactly-once inflation.
- **Seed/topology:** generated and rejected states or edges, truncation, route
  candidates, graph scale, and whether a plausible topology was representable.
- **Geometry:** every transformation's input and output route, vertex count,
  length, minimum clearance, and first invalid edge.
- **Motion/timing:** representation, continuity, endpoint conditions, duration
  guesses, positive segment times, waits, goal-time semantics, and moving-data
  alignment.
- **Kinematics/collision:** continuous extrema or certificates, normalized limit
  ratios, collision interval, obstacle identity, and unresolved intervals.
- **Endpoint/ranking:** terminal state and time, each candidate's validity and
  quality, deterministic ordering, fallback use, and selected source.

Use the independent validator rather than planner success, optimizer status,
sampled plots, or penalties. Temporary instrumentation must be behavior-neutral
and removed after evidence collection unless it belongs in the stable result
contract and passes an overhead review.

Before editing, record:

```text
Observed failure:
Failure class:
Earliest failing stage:
Evidence before and after that boundary:
Why later symptoms are not the cause:
General invariant violated:
Proposed fix and structurally different check:
```
