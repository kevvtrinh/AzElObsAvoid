# Plan: Make Waypoint Fallback Explicit and Support Smooth Moving-Obstacle Detours

## Problem statement

The current planner has a hidden behavioral change for moving-obstacle routes.
When a request contains any changing obstacle, the static BMTP kernel is disabled
for the entire request. The compact timed-motion path currently supports only a
direct wait followed by direct motion. Therefore, a moving-obstacle scenario
whose collision-free route contains interior waypoints returns
`unsupportedTimedTopology` or `invalidDirectWaitSeed`.

The planner currently catches those outcomes and falls back to
`createRuckigWaypointMotion`. That fallback solves every route edge as a
separate state-to-state motion with zero velocity and zero acceleration at each
interior waypoint. It can return a valid collision-free trajectory, but changes
the motion from a smooth pass-through trajectory into stop-and-go motion.

This fallback occurs in cases such as:

- a static hand-drawn U-shaped obstacle requires a multi-waypoint detour while
  any other obstacle in the request moves;
- a moving obstacle itself requires a spatial detour rather than a wait and a
  direct edge;
- a timed visibility seed contains intermediate obstacle-boundary vertices;
- a timed seed is not represented exactly as `wait at start -> direct goal`;
- the user explicitly selects the `ruckigWaypoint` trajectory method.

Removing the fallback exposes the underlying limitation: the planner has no
supported global, smooth multi-waypoint trajectory constructor for a request
containing moving obstacles. The fallback should have been exposed as an option,
and moving-obstacle support should be expanded rather than relying on it as the
only successful path.

## Objectives

1. Make the Ruckig stop-at-waypoint fallback an explicit, documented choice.
2. Default to behavior that never silently substitutes stop-and-go motion for a
   requested smooth global trajectory.
3. Preserve clear diagnostics when timed topology is unsupported.
4. Expand the primary trajectory method to support smooth multi-waypoint
   detours with moving obstacles.
5. Retain Ruckig composition as an optional, honestly labeled recovery method.
6. Independently validate every returned trajectory against continuous moving
   geometry and all velocity, acceleration, and jerk limits.

## Non-goals

- Do not hide unsupported timed topology by changing result messages.
- Do not treat sampled collision checks as continuous safety certificates.
- Do not solve the problem by smoothing only the plotted or sampled history.
- Do not add scenario-specific logic for U-shapes or particular moving
  obstacles.
- Do not declare the Ruckig fallback a pass-through method while its interior
  waypoint states remain zero.
- Do not regress static BMTP behavior while adding dynamic support.

## Phase 1: Expose and document fallback policy

### 1.1 Add a public fallback option

Add a resolved planner option with an explicit policy, preferably:

```matlab
options.UnsupportedTimedTopologyPolicy = "fail";
```

Supported values:

- `"fail"`: retain the original timed-kernel failure and return no trajectory;
- `"ruckigStopAtWaypoints"`: permit the current rest-to-rest Ruckig fallback.

Prefer a policy-valued option over a boolean because future recovery methods,
such as a pass-through Ruckig state refinement, may be added without changing
the public interface again.

Default the option to `"fail"`. A planner should not silently replace smooth
global motion with stop-and-go motion.

### 1.2 Guard the fallback

In `+obstacleAvoidance/+planner/planCorridorQuintic.m`, invoke
`createRuckigWaypointMotion` after `unsupportedTimedTopology` or
`invalidDirectWaitSeed` only when the resolved policy is
`"ruckigStopAtWaypoints"`.

When the policy is `"fail"`:

- retain the original candidate and termination reason;
- retain the timed seed identity, source, route, and timing data;
- record that fallback was available but disabled by policy;
- do not replace the original failure with a generic validation failure.

When fallback is enabled:

- record `FallbackAttempted=true`;
- record `FallbackMethod="ruckigStopAtWaypoints"`;
- record the original termination reason;
- record all interior waypoint times and states;
- record whether every interior waypoint was constrained to rest;
- state the stop-at-waypoint behavior in the result message.

### 1.3 Correct sandbox controls and labels

Replace ambiguous controls such as `Ruckig pass-through` with a control that
describes the actual behavior:

```text
Unsupported timed route:
  Fail and diagnose
  Ruckig stop at waypoints
```

Do not present `WaypointWarmStartMode="passThrough"` as available when the
optional implementation file is absent. The UI should show the resolved
availability before planning rather than accepting a selection and downgrading
it later.

### 1.4 Preserve API compatibility

If compatibility with existing callers is required:

- recognize the old option temporarily;
- map it to the new policy with one deprecation warning;
- echo only the resolved new policy in new diagnostics;
- remove the compatibility mapping after a documented transition period.

## Phase 2: Improve failure diagnostics before expanding the solver

For every timed route seed, retain:

- stable seed index and source;
- spatial waypoint sequence;
- assigned waypoint times or normalized `tau`;
- whether the route contains waits;
- first unsupported transition or topology feature;
- whether each moving obstacle can interact with the route over time;
- original timed-kernel termination reason;
- fallback policy and fallback outcome;
- independent validation outcome for any returned candidate.

Use distinct termination reasons:

- `unsupportedTimedMultiWaypointRoute`;
- `invalidDirectWaitSeed`;
- `fallbackDisabledByPolicy` only as diagnostic metadata, not as a replacement
  for the earliest failure;
- `ruckigWaypointFallbackFailed` when the explicitly enabled fallback fails.

The final result should explain that a geometric route existed but no supported
smooth timed motion constructor could realize it. Do not report this as `no
path` or imply geometric infeasibility.

## Phase 3: Separate static and dynamic obstacle relevance

The current all-or-nothing `useStaticKernel` decision disables static BMTP when
any obstacle changes. Replace this request-wide classification with
route-specific relevance analysis.

For each seed:

1. Determine which obstacles can intersect the seed's conservative swept
   trajectory envelope during the request interval.
2. Partition obstacles into:
   - static obstacles relevant to the route;
   - moving obstacles relevant to the route;
   - obstacles proven irrelevant over the complete physical interval.
3. If all relevant obstacles are static, allow the existing BMTP kernel to
   optimize the route.
4. Validate the resulting trajectory against the complete original obstacle
   set, including obstacles classified as irrelevant.
5. Reject the candidate if independent validation contradicts the relevance
   certificate.

This is an optimization and compatibility path, not the complete dynamic
solution. It will handle cases where a harmless moving obstacle currently
disables an otherwise valid static U-shape solve.

The relevance test must be conservative and continuous in time. If relevance
cannot be proved absent, treat the obstacle as relevant.

## Phase 4: Add global smooth timed-route optimization

Implement a trajectory constructor that accepts an entire timed multi-waypoint
route and optimizes it as one continuous motion. Do not solve its edges as
independent rest-to-rest problems.

### 4.1 Required mathematical behavior

The new solver must:

- optimize the complete route in azimuth, elevation, and physical time;
- support nonzero velocity and acceleration through interior route regions;
- enforce at least C2 state continuity and the repository's required jerk
  representation across mesh intervals;
- enforce start and terminal states exactly;
- enforce strictly increasing physical time;
- support waiting as an explicit stationary interval rather than as repeated
  ambiguous waypoints;
- enforce axis velocity, acceleration, and jerk limits continuously;
- constrain the trajectory against moving obstacle geometry over every
  relevant time interval;
- support earliest-arrival and fixed-arrival modes;
- retain the last independently valid candidate if later iterations fail.

### 4.2 Route semantics

Treat visibility-graph vertices as topology guidance, not mandatory stop
states. The optimized trajectory may pass near or through an admissible
corridor around a seed vertex without attaining the vertex exactly, unless an
exact waypoint is explicitly part of the user request.

Separate these concepts in data structures:

- route guide point;
- exact required waypoint;
- wait event;
- obstacle-side or homology constraint;
- moving-goal capture constraint.

### 4.3 Dynamic obstacle representation

Build time-indexed obstacle constraints from the authoritative moving geometry.
Implement this as an extension of the branch's existing BMTP representation.
Candidate BMTP approaches may include:

- time-cell separating planes coupled to trajectory intervals;
- moving convex-piece constraints with adaptive temporal refinement;
- a timed BMTP extension whose separating planes depend on physical time;
- space-time BMTP cells that jointly constrain trajectory control points and
  moving convex obstacle pieces;
- adaptive temporal subdivision of BMTP segments around obstacle events.

Whichever formulation is chosen, the independent validator remains
authoritative. Sampling alone is not sufficient evidence of collision freedom.

### 4.4 Warm start

Construct a dynamically plausible timed warm start from the complete seed:

1. assign increasing initial waypoint times using route distance, derivative
   limits, obstacle events, and waits;
2. estimate shared nonzero interior velocities from adjacent route directions;
3. reduce speed where curvature, clearance, or an obstacle timing event
   requires it;
4. estimate shared accelerations consistently;
5. generate one continuous warm trajectory;
6. validate the warm start before optimization and record its first failure.

If Ruckig is used to help build the warm start, it must estimate and preserve
shared pass-through waypoint states. It must not force every interior waypoint
to rest.

## Phase 5: Retain an improved optional Ruckig recovery method

Keep the existing stop-at-waypoint implementation under the explicit name
`ruckigStopAtWaypoints` for users who value robustness over traversal quality.

Optionally add a separate future method:

```text
ruckigPassThroughWaypoints
```

That method must:

- estimate feasible shared interior velocities and accelerations;
- use the identical terminal state of one segment as the initial state of the
  next;
- refine waypoint states when a segment violates limits or collision
  constraints;
- fall back to rest only at individually identified waypoints when explicitly
  allowed;
- report every forced stop rather than hiding it.

Do not use a pass-through label until tests demonstrate nonzero speed at
eligible interior waypoints.

## Phase 6: Tests

### 6.1 Fallback-policy tests

Add tests proving that:

- the default policy is `"fail"`;
- unsupported timed topology does not call Ruckig under the default policy;
- enabling `"ruckigStopAtWaypoints"` reproduces the current fallback;
- fallback diagnostics preserve the original failure reason;
- result messages explicitly state when interior stops were imposed;
- unknown policy values are rejected;
- resolved options are echoed consistently on success and failure.

### 6.2 Regression scenarios

At minimum include:

1. Static U-shaped obstacle with no moving obstacles: existing BMTP behavior
   remains smooth and valid.
2. Static U-shaped obstacle plus a distant moving obstacle: route-specific
   relevance permits smooth BMTP motion and full validation passes.
3. Static U-shaped obstacle plus a moving obstacle crossing the direct route:
   the new dynamic solver produces a smooth detour or returns a precise
   supported failure.
4. Moving obstacle requiring a wait and direct motion: existing direct-wait
   behavior remains valid.
5. Moving obstacle requiring both waiting and a spatial detour: global timed
   optimization supports both behaviors in one trajectory.
6. Explicit fallback enabled: motion stops at waypoints and reports those
   stops.
7. Sharp route corner with adequate clearance: optimized motion passes through
   with nonzero speed while respecting acceleration and jerk limits.
8. Corner where a stop is physically necessary: the solver may stop, but the
   outcome is caused by feasibility or optimality rather than a hard-coded
   waypoint state.

### 6.3 Assertions

For every successful test, assert:

- exact start and terminal states;
- strictly increasing time;
- continuous position, velocity, and acceleration;
- velocity, acceleration, and jerk limits over complete polynomial intervals;
- continuous collision freedom against every moving obstacle;
- nonzero interior speed where the scenario is designed to permit
  pass-through;
- no unreported fallback or forced stop;
- deterministic seed ordering and selection;
- stable result and diagnostic schemas.

## Phase 7: Performance and acceptance gates

Measure separately:

- route generation time;
- timed warm-start construction time;
- dynamic optimization time;
- independent validation time;
- total time to first valid candidate;
- final arrival time and motion length;
- minimum clearance;
- peak velocity, acceleration, and jerk;
- minimum speed at interior guide points;
- number and duration of waits;
- number of forced or optimized stops.

Acceptance criteria:

- no hidden Ruckig fallback remains;
- the fallback policy is visible in defaults, sandbox controls, results, and
  documentation;
- static planner benchmarks do not regress outside agreed tolerances;
- a distant moving obstacle no longer disables an otherwise valid static
  multi-waypoint solve;
- at least one moving-obstacle multi-waypoint detour succeeds through the new
  global timed method;
- eligible route corners have demonstrably nonzero pass-through speed;
- all successful trajectories pass independent continuous validation;
- unsupported cases retain the earliest accurate failure reason.

## Recommended implementation order

1. Add and test `UnsupportedTimedTopologyPolicy` with default `"fail"`.
2. Correct sandbox labels and expose resolved fallback behavior.
3. Expand diagnostics so every affected seed and fallback decision is visible.
4. Add conservative route-specific moving-obstacle relevance analysis.
5. Establish regression baselines for static and direct-wait cases.
6. Implement the global smooth timed-route solver behind an explicit
   experimental option.
7. Validate it on moving-detour, wait-plus-detour, and irrelevant-moving-
   obstacle scenarios.
8. Promote it to the primary dynamic route constructor only after continuous
   validation and regression gates pass.
9. Keep `ruckigStopAtWaypoints` as an explicit recovery policy rather than an
   automatic behavioral substitution.

## Completion definition

This work is complete when a moving obstacle no longer causes an undocumented
switch to stop-at-waypoint motion, users can explicitly choose whether that
recovery method is allowed, and the primary planner can optimize and validate a
smooth multi-waypoint trajectory in the presence of genuinely relevant moving
obstacles.
