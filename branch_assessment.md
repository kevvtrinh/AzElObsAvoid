# Plan 325 branch assessment

## Evidence scope

This assessment applies to the uncommitted Plan 325 implementation based on
`cc6eec3`. Static analysis found no messages in 62 MATLAB files. The three
maintained test files passed 49 of 49 tests. The full maintained example set
has not yet been rerun. The old Plan 502 rows in `benchmark.csv` are retained
only as historical evidence for their recorded contracts.

The duration header in `benchmark.csv` is now `MotionDuration_s`. A
fixed-arrival duration is the configured horizon. It is not a measured minimum.

## Largest strength

Plan 325 has one public planning path for static, moving, and deforming
obstacles; fixed and earliest arrival; waiting; and moving goals. It first
builds a finite-jerk stop-at-waypoint motion when that motion family supports
the request. It accepts that motion only after independent polynomial,
kinematic, endpoint, and continuous collision validation. A bounded HS3 stage
can improve the result or handle requests that the first motion does not
support.

The validator checks event endpoints, complete polynomial segments, sampled
history agreement, state continuity, physical limits, safety-margin
provenance, and original protected obstacle histories. Planning failures keep
stable diagnostics. The result does not claim global completeness or global
optimality.

## Largest weaknesses

### 1. Bounded proposal coverage

Spatial and time-layer searches use finite samples. Dense-envelope and cluster
reductions can remove a useful topology. Final validation prevents false
success, but it cannot make the proposal set complete. A reduced failure is an
incomplete bounded search, not proof that no physical path exists.

### 2. Conservative first-motion family

The fast motion stops at each geometric waypoint and assigns one common edge
duration. It can be slower than a through-velocity motion. It can also reject a
feasible nonuniform edge schedule. Timed waits, nonzero endpoint derivatives,
and earliest moving-goal intercepts require HS3.

### 3. Local optimization and cooperative deadlines

HS3 uses a frozen local corridor and a local nonlinear solve. Setup and
certificate loops check an absolute deadline between bounded operations, but a
single operation is not preempted. `PlanningDeadlineOverrun_s` reports an
observed overrun. Optimization Toolbox `fmincon` is required for HS3.

### 4. Restricted azimuth wrapping

Periodic obstacle images are not implemented. Azimuth wrapping is therefore
supported only for an obstacle-free fixed-position goal. Other wrapped
requests return an identified unsupported-configuration error.

### 5. Size target

The implementation has 26 production MATLAB files and 6,988 physical
production lines. The largest files have 900, 900, and 869 lines. The complete
MATLAB tree has 11,546 physical lines. The production and complete hard limits
pass. The 10,500-line complete-tree target does not pass.

## Current judgment

The code and focused tests support a compact, correctness-first Plan 325
candidate. Runtime, route quality, visible graphics, and all maintained example
contracts still require serial execution before this branch is complete.
