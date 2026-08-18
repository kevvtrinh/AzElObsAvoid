# HS3 Planner Refactor Plan

## Objective

Replace the current multi-planner architecture with one small, maintainable
HS3 trajectory-planning pipeline. Preserve the useful public obstacle and
planner interfaces, but remove the SIPP, snapshot, and competing trajectory
generation machinery that expanded the repository to roughly 20,000 lines.

The production pipeline shall be:

```text
canonical protected obstacles
    -> a small bounded set of topology seeds
    -> separated HS3 trajectory optimization
    -> independent continuous validation
    -> earliest validated trajectory
```

The implementation must follow the repository's `AGENTS.md`, including its
MATLAB headers, naming, units, example structure, failure behavior, and
verification requirements.

## Branch Creation

Create the refactor as a new branch from the latest `main` branch. Git branch
names cannot contain spaces, so the requested branch name `hs3 refactor` is
represented as `hs3-refactor`.

```text
git status --short
git switch main
git pull --ff-only
git switch -c hs3-refactor
```

Do not overwrite an existing branch. If `hs3-refactor` already exists, stop
and inspect it before deciding whether to resume it or create a differently
named branch.

## Non-Negotiable Size Limits

Treat code size as an acceptance requirement, not a future cleanup task.

- Production MATLAB code, excluding examples, tests, and benchmarks: no more
  than 5,000 lines.
- Entire maintained MATLAB repository, including examples and tests: target
  no more than 9,000 lines.
- No production MATLAB file should exceed 900 lines without a documented
  reason and explicit approval.
- Target 8 to 12 production files with substantial responsibilities. Do not
  replace a few large files with dozens of one-function fragments.
- Do not count comments or headers as an excuse to exceed the limits; they are
  part of the maintained code.
- At the end of every phase, report production and total MATLAB line counts.
- If a feature would violate these limits, first simplify the feature or defer
  it. Do not silently enlarge the architecture.

## Required Planner Contract

Retain one public entry point:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

Retain a zero-input defaults call:

```matlab
options = planAzElMotion();
```

The planner must support:

- static and known time-varying polygon obstacles;
- fixed-arrival and earliest-arrival goal policies;
- initial and terminal position, velocity, and acceleration;
- per-axis velocity, acceleration, and jerk limits;
- optional azimuth wrapping;
- waiting when it is dynamically feasible and useful;
- safety margins applied exactly once by obstacle construction;
- deterministic behavior for identical inputs and options;
- success and expected failure through one stable result schema.

The result should contain only information that callers, validators, or plots
actually use:

- `Success`, `Message`, and `TerminationReason`;
- resolved inputs, limits, and options;
- original and protected obstacle geometry;
- attempted seed summaries and selected seed index;
- selected geometric seed;
- `time_s`, `position_deg`, `velocity_deg_s`, `acceleration_deg_s2`, and
  `jerk_deg_s3`;
- independent collision and kinematic validation;
- elapsed time and concise solver diagnostics.

Remove compatibility fields that no maintained example or public workflow
needs. Do not retain two representations of the same trajectory.

## Honest Optimality Statement

The planner shall make this precise claim:

> The result is the earliest independently validated local HS3 solution found
> from the finite deterministic seed set that was attempted.

It shall not claim global time optimality, global path completeness, or proof
that no feasible trajectory exists. If all seeds fail, return a reason such as
`noValidatedSeed` and preserve the diagnostic outcome for each attempted seed.

## Mathematical Formulation

Use the separated third-order Hermite-Simpson formulation described by
Moreno-Martin, Ros, and Celaya:

https://arxiv.org/abs/2302.09056

Use the jerk-controlled state:

```text
q = [azimuth, elevation]
v = dq/dt
a = dv/dt
j = da/dt
```

with dynamics:

```text
dq/dt = v
dv/dt = a
da/dt = j
```

For earliest arrival, use final time and the HS3 jerk ordinates as decision
variables. Reconstruct position, velocity, and acceleration from the shared
third-order chain rather than maintaining inconsistent independent splines.

Use a lexicographic objective implemented in two stages:

1. Minimize final time.
2. Holding arrival time within a documented tolerance, minimize integrated
   squared jerk.

This prevents a jerk weight from secretly trading away meaningful arrival
time while still selecting the smoother solution among time-equivalent
candidates.

Enforce:

- exact initial state;
- terminal position, velocity, and acceleration;
- fixed or bounded final time according to the goal policy;
- continuous workspace, velocity, acceleration, and jerk limits;
- obstacle separation at optimization constraint points;
- independent post-solve collision and dynamics validation between points.

## Minimal Production Architecture

The final implementation should converge toward the following responsibility
layout. Existing public filenames may be retained where doing so reduces
migration work, but responsibilities must not be duplicated.

| Responsibility | Target size |
| --- | ---: |
| `planAzElMotion.m`: validation, orchestration, selection, result | 300-500 |
| Obstacle construction, packing, interpolation, and query | 700-1,000 total |
| One bounded topology-seed generator | 400-700 |
| HS3 transcription and solve | 800-1,400 total |
| Trajectory reconstruction and adaptive sampling | 250-450 |
| Independent collision and kinematic validation | 400-700 |
| Plotting and animation | 500-800 total |

Small shared normalization utilities are allowed, but every new file must own
a distinct reusable invariant.

## Topology Seeds

HS3 is a local nonlinear optimizer, so retain one deliberately small seed
generator. It exists only to propose different sides and timings around
obstacles; it must not become a second motion planner.

Generate seeds in this deterministic order:

1. Direct start-to-goal route.
2. A small number of shortest distinct routes from one coarse topology graph.
3. Optional waiting variants only when moving geometry blocks a spatially
   useful route during part of the horizon.

Requirements:

- Cap the default seed count at five and expose at most one option to increase
  it to a documented hard maximum.
- Keep only geometrically distinct seeds.
- Never impose seed vertices as trajectory equalities. They initialize HS3
  and select a local obstacle corridor only.
- Do not rank a seed as dynamically feasible before HS3 validates it.
- Do not create snapshot visibility graphs, SIPP safe intervals, event
  detectors, or a second optimality claim.
- Prefer a coarse time-expanded graph for moving obstacles over separate
  spatial and temporal planning systems. It may include waiting edges and
  conservative reachability bounds, but its output is only an initialization.
- Record the graph resolution and each returned seed so failures are
  reproducible.

If the direct seed succeeds and an option such as `DirectSeedOnly` is enabled,
the graph may be skipped. The default should still attempt the bounded seed set
needed for basic obstacle-side diversity.

## Obstacle Constraints

Continue using the protected geometry stored by `makeAzElObstacleData` as the
single source of truth. Do not add another planner-level safety margin.

Use a simple local obstacle-corridor strategy:

- Associate each seed sample with a separating polygon edge or convex piece.
- Freeze the association during one `fmincon` solve so the nonlinear
  constraint remains differentiable.
- Permit at most two input-driven corridor relinearizations when independent
  collision validation identifies a failed segment.
- Do not introduce a complex general corridor-repair framework.
- For moving obstacles, evaluate the associated geometry at the trajectory
  time represented by the HS3 point.
- For concave geometry, preserve the obstacle boundary faithfully or use a
  documented convex decomposition; never replace a concave protected obstacle
  by an unsafe inward approximation.

Optimization-point clearance is not sufficient evidence of collision freedom.
The final result must pass the independent continuous validation below.

## Continuous Validation

Implement one independent validator used by the planner, tests, and examples.
It must not trust `fmincon` success or the planner's `Success` flag.

Validate:

- exact start and terminal states within documented tolerances;
- finite, strictly increasing time;
- continuous position bounds;
- continuous velocity, acceleration, and jerk bounds;
- HS3 dynamics consistency;
- collision freedom against the protected moving geometry;
- azimuth wrapping policy;
- safety-margin provenance.

Use polynomial bounds where they are simple and reliable. Use adaptive
subdivision for moving-obstacle collision validation, refining a segment until
the relative trajectory/obstacle motion cannot cross the measured clearance
or until a documented resolution limit is reached. An unresolved segment must
fail validation; it must never be silently accepted.

## Explicit Legacy Removal

After the replacement pipeline passes focused tests, remove the superseded
production paths rather than leaving them disabled behind options.

Delete or fully replace the responsibilities currently provided by:

- snapshot visibility-graph construction;
- visibility-snapshot selection and change detection;
- safe-interval roadmap construction;
- SIPP and SIPP-IP search;
- the separate space-time visibility-graph forwarding layer;
- large candidate-reduction and seed-budget allocation systems;
- parallel multi-seed planning infrastructure;
- specialized fixed-arrival LP/QP/projection pipelines that duplicate the HS3
  feasibility solve;
- repeated empty schemas and compatibility trajectory conversions;
- benchmark baselines tied only to deleted algorithms;
- options, tests, and documentation for deleted behavior.

Do not retain a legacy planner mode on this replacement branch. Keep only
shared obstacle, query, visualization, and validation code that the new HS3
pipeline genuinely calls.

## Moving-Target Simplification

Do not maintain another thousand-line planner for moving targets. Represent a
known moving goal as a time-indexed goal position function or sampled goal
history consumed by the same HS3 planner.

- Fixed intercept time: constrain the terminal position to the goal position
  at the requested time.
- Earliest intercept: make final time a decision variable and constrain the
  terminal position to the interpolated goal position at that time.
- Terminal velocity and acceleration policy must be explicit.

Any public intercept helper should be a thin input-adaptation wrapper around
`planAzElMotion`, not an independent planning implementation.

## Implementation Phases

### Phase 1: Record the Baseline

- Create `hs3-refactor` from updated `main`.
- Record the starting commit hash and working-tree status.
- Count MATLAB lines separately for production, examples, tests, and
  benchmarks.
- Run the smallest representative baseline cases headlessly:
  - obstacle-free motion;
  - one static obstacle;
  - one moving obstacle;
  - expected no-path case.
- Record behavior and runtime. Do not preserve a bug merely because it is in
  the baseline.

Exit criterion: the branch and reproducible baseline report exist.

### Phase 2: Extract the HS3 Mathematical Kernel

- Identify the current separated HS3 decision layout, integration equations,
  endpoint constraints, objective, and polynomial reconstruction.
- Rewrite only that kernel into the small target architecture.
- Remove warm-start projections, solver cascades, repair loops, and
  compatibility schema assembly from the mathematical kernel.
- Add focused tests against analytic one-axis and two-axis jerk-controlled
  motions.
- Verify fixed-duration feasibility before enabling final-time optimization.

Exit criterion: obstacle-free fixed-arrival and earliest-arrival problems pass
independent state and limit validation.

### Phase 3: Add Minimal Obstacle-Constrained HS3

- Connect canonical protected obstacle queries to the HS3 constraint points.
- Build a local separating corridor from a supplied geometric seed.
- Add at most two corridor relinearization attempts.
- Add adaptive continuous collision validation.
- Test convex, concave, static, translating, and deforming obstacles.

Exit criterion: supplied seeds can produce independently collision-free HS3
trajectories or concise, reproducible failures.

### Phase 4: Build the Single Bounded Seed Generator

- Attempt the direct route.
- Build one coarse topology graph only when obstacle-side alternatives are
  needed.
- Return at most five distinct default seeds.
- Support known moving obstacles with time layers and waiting edges without
  introducing SIPP.
- Keep graph diagnostics bounded and separate from HS3 solver diagnostics.

Exit criterion: the static rectangle, U-shaped obstacle, moving barrier, and
alternating-slot scenarios each generate the expected route diversity without
scenario-specific waypoints.

### Phase 5: Integrate the Public Planner

- Make `planAzElMotion` validate inputs and resolve defaults once.
- Generate the bounded seed set.
- Solve HS3 independently for each seed.
- Validate every returned trajectory with the shared validator.
- Select earliest arrival, then lowest integrated squared jerk within the
  arrival-time tolerance, then lowest deterministic seed index.
- Return one compact stable result on success and failure.

Exit criterion: all maintained examples call the same production planner and
no example performs planning or retiming internally.

### Phase 6: Remove the Superseded Stack

- Use repository-wide call-site searches to prove that legacy SIPP, snapshot,
  and alternate planner functions are no longer called.
- Delete those implementations and their exclusive options, tests,
  benchmarks, wrappers, and documentation.
- Remove stale compatibility fields from examples and plotting.
- Recount lines and simplify further if the production total exceeds 5,000.

Exit criterion: there is exactly one production trajectory-planning path and
the line-count limits are satisfied.

### Phase 7: Migrate and Reduce Examples

Keep a small scenario suite that exercises structurally different behavior:

- Obstacle-free analytic motion.
- Static rectangle requiring a side choice.
- Static U-shaped obstacle.
- Moving barrier where waiting is best.
- Moving barrier where immediate detour is best.
- Alternating slalom or alternating slots.
- Moving-target earliest intercept.
- Dense concave polygon after canonical simplification.
- Expected no-path result.

Each example must only construct inputs, call `planAzElMotion`, independently
validate, and plot. Delete redundant examples that exercise the same algorithm
path without adding a distinct invariant.

Exit criterion: every retained example follows the `AGENTS.md` template and
runs headlessly without local planning logic.

### Phase 8: Final Verification and Size Audit

Run, one process at a time:

- MATLAB syntax and Code Analyzer checks where available;
- focused HS3 unit tests;
- obstacle-construction and interpolation tests;
- successful static and moving-obstacle tests;
- fixed-arrival and earliest-arrival tests;
- nonzero endpoint state tests;
- azimuth wrapping enabled and disabled tests;
- expected no-path and time-limit tests;
- every retained example headlessly;
- at least one visible animation when graphics are available.

For each example, report:

- planner and independent-validation success;
- selected seed and seed count;
- arrival time;
- path length;
- maximum velocity, acceleration, and jerk;
- collision result;
- runtime and termination reason.

Finally report:

- production MATLAB line count;
- total MATLAB line count;
- number of production files;
- every production file exceeding 700 lines;
- deleted files and removed options;
- untested MATLAB/toolbox limitations;
- exact claim of optimality and completeness.

## Required Tests

At minimum, automated tests must cover:

- HS3 integration agrees with known constant-jerk motion;
- reconstructed knot and midpoint states satisfy the HS3 equations;
- terminal state equalities are enforced;
- continuous velocity, acceleration, and jerk limits detect between-knot
  violations;
- direct obstacle-free earliest arrival reaches the analytic or independently
  computed minimum within tolerance;
- different seeds can converge to different obstacle-side local solutions;
- the selected solution is the earliest validated candidate;
- moving obstacles are evaluated at trajectory time, not a frozen snapshot;
- waiting can occur without forcing zero velocity at geometric corners;
- collision at a point between collocation nodes fails validation;
- safety margin is applied once;
- deterministic repeated runs return the same seed ordering and result;
- failed seeds do not erase their diagnostics;
- no-path returns `Success=false` without throwing an expected-planning error.

## Completion Criteria

The `hs3-refactor` branch is complete only when all of the following are true:

- One public planner owns geometry and timing optimization.
- The planner uses separated HS3 with jerk as the motion control.
- All retained examples use that planner.
- Static obstacles, moving obstacles, moving targets, waiting, and jerk limits
  use the same production path.
- The SIPP, snapshot, and competing planner implementations are removed.
- Every successful result passes independent continuous collision and
  kinematic validation.
- Expected failures return stable results and useful diagnostics.
- Production MATLAB code is at or below 5,000 lines.
- Total maintained MATLAB code is at or below the agreed repository ceiling,
  with any exception explicitly approved before completion.
- Documentation states that the result is the best validated local solution
  among attempted seeds, not a globally optimality-certified path.
- The final diff contains no scenario-specific route, waypoint, timing, or
  obstacle-name logic.

## Stop Conditions

Stop and report instead of expanding the design when:

- satisfying one example appears to require a scenario-specific heuristic;
- a proposed feature creates a second planner or retimer;
- the production code would exceed 5,000 lines;
- continuous collision validation cannot resolve a segment safely;
- a solver failure is being hidden by clipping, fallback, or altered inputs;
- a deleted legacy dependency is still required by a maintained public
  workflow;
- MATLAB or Optimization Toolbox is unavailable for required verification.

The purpose of this branch is not to preserve every experiment. It is to leave
one understandable HS3 planner that can be trusted, tested, and maintained.
