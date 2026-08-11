# Greenfield Maintainer Instructions

These instructions apply to this directory and all descendants.

## Assignment

Build a new azimuth/elevation obstacle-avoidance planner from scratch. For this
assignment, no planner is assumed. Files outside the starting design sources
listed below are out of scope during architecture design.

Use only these starting design sources:

1. [`README.md`](README.md), which is the greenfield landing page.
2. [`PLAN.md`](PLAN.md), which defines the problem and acceptance criteria.
3. [`STYLE.md`](STYLE.md), which defines MATLAB and documentation conventions.
4. [`makeAzElObstacleData.m`](makeAzElObstacleData.m), which fixes the canonical
   obstacle input format.

Choose the new public API, architecture, internal representation, validators,
tests, and implementation from first principles. Evidence decides whether a
design choice remains.

## Non-negotiable outcome

The new planner must return the earliest physically achievable validated
arrival it can establish for a fixed or moving goal while:

- avoiding all az/el obstacle regions and the requested safety margin;
- obeying position, velocity, and acceleration constraints throughout motion;
- matching complete initial and terminal states;
- maintaining continuous position and velocity;
- carrying velocity through safe turns instead of stopping at artificial
  intermediate points;
- representing necessary waits explicitly and feasibly;
- handling static and time-varying obstacles with independent time grids; and
- reporting unknown honestly when it cannot prove success or infeasibility.

Safety and kinodynamic validity come before arrival. Arrival comes before
distance, visual smoothness, compute runtime, and memory.

## Preserve the input

Do not redesign the canonical record:

```matlab
azElData = struct( ...
    "targetName", targetName, ...
    "time_s", time_s, ...
    "az_deg", {azimuthSlices_deg}, ...
    "el_deg", {elevationSlices_deg}, ...
    "status", statusBySample);
```

Preserve field names, units, cell orientation, independent obstacle time grids,
paired nonfinite region separators, and empty-region representation.

Keep `makeAzElObstacleData.m` as the supported builder. Numeric boundaries are
static and repeated over `time_s`; cell boundaries are sampled time-varying
geometry. The planner must accept this canonical format directly. Private
packing must remain private.

## Examples are pure mission inputs and validation

Example files are acceptance cases, not planner configuration files. Every
example must contain only this visible flow:

1. build or load canonical `azElData`;
2. define the initial state and fixed or moving goal;
3. define physical limits and genuine mission options;
4. call the one public planner with its ordinary defaults;
5. independently validate and assert the returned command; and
6. optionally report or visualize the already validated result.

Genuine mission options include safety margin, wrap policy, uncertainty
padding, mission deadline, and capture or trailing semantics. Display options
may affect only reporting or visualization and must never flow back into
planning.

An example must never provide or derive:

- coarse or fine grid spacing;
- spatial or temporal resolution;
- refinement levels, schedules, or iteration counts;
- branching, neighborhood, or candidate-generation controls;
- internal cost weights or stopping thresholds;
- collision-sampling or smoothing controls;
- a planner mode or implementation selector;
- per-scenario compute tuning;
- guide paths, corridors, waypoints, or stored commands; or
- hints chosen because the case is known to be difficult.

Scenario support functions may construct obstacle geometry only. They must not
return planner options, internal hints, or partial solutions. All numbered
examples use the same planner defaults. Add a source-level regression test that
scans example and support files and fails if a forbidden internal option or
route hint appears.

## The planner owns adaptive coarse-to-fine work

The planner must select all internal resolution and refinement behavior by
examining the request. Callers and examples do not choose a grid. A single
fixed resolution is not an acceptable product strategy across the supported
problem family.

The planner must autonomously reason about at least:

- total angular extent and start-to-goal separation;
- obstacle size, boundary detail, separation, and requested safety margin;
- narrow passages and clearance relative to command dynamics;
- obstacle sample cadence, apparent motion, and short safe time windows;
- initial and terminal velocity and acceleration;
- physical rate and acceleration limits;
- azimuth-wrap policy and seam proximity;
- mission horizon and deadline; and
- remaining global planning budget and incumbent quality.

Use that information to choose an initial coarse representation, then move
through finer spatial and temporal resolutions when needed. Refinement should
concentrate around plausible passages, obstacle boundaries, turns, tight time
windows, unresolved collision checks, and regions that may improve arrival.
Broaden or refine when a coarse attempt is inconclusive; do not mistake coarse
failure for infeasibility.

Adaptive behavior also owns internal work allocation, candidate density,
validation sampling needed to certify a result, and the decision to stop
refining. It must:

- operate under one shared default planning budget rather than per-example
  budgets;
- retain the best independently validated incumbent across all resolutions;
- compare incumbents by feasibility first and physical arrival second;
- validate against canonical polygon geometry, never only the selected grid;
- continue refinement while evidence predicts a meaningful arrival or
  feasibility improvement and budget remains;
- return the best validated incumbent if refinement ends; and
- return `unknown`, not `infeasible`, when resolution or budget is insufficient
  to prove a result.

For deterministic inputs, adaptive choices must be deterministic. Diagnostics
must record the selected spatial and temporal resolutions, refinement sequence,
work spent at each level, incumbent changes, validation failures, and stopping
reason. These records are outputs for explanation and testing, never inputs
copied into an example.

## Build order

1. Freeze request and result schemas, units, tolerances, and failure categories.
2. Build independent collision, clearance, kinodynamic, endpoint, and arrival
   validators.
3. Build obstacle-free minimum-time oracles for solvable boundary states.
4. Implement unobstructed synchronized two-axis motion.
5. Add static obstacle avoidance with internally selected coarse-to-fine
   spatial refinement and velocity carry.
6. Add time-varying obstacles with adaptive temporal resolution, feasible
   waits, and moving goals.
7. Improve arrival while preserving every independent validation gate and the
   best validated incumbent across refinement levels.
8. Add deterministic, randomized, source-purity, and headless visual evidence.

Do not begin with performance tuning. Establish a correct end-to-end result and
independent validation first.

## Development rules

- Keep one clear public planning entry point.
- Keep mission options separate from private implementation controls.
- Make all grid, resolution, and refinement choices inside the planner.
- Do not require callers to provide routes, waypoints, corridors, templates,
  scenario identifiers, or hidden tuning.
- Do not key behavior to obstacle names, filenames, example names, or seeds.
- Do not store answers for demonstrations.
- Never weaken collision or kinodynamic checks to gain speed.
- Keep the best independently validated incumbent during bounded planning.
- Never discard or regenerate a seeded case because it is difficult.
- Use the same canonical geometry and time semantics for validation and
  visualization.
- Update command time, position, velocity, and acceleration coherently.
- Preserve unrelated user changes in the worktree.

## Required evidence

A successful regression asserts:

- finite, strictly increasing command time;
- exact initial state within tolerance;
- complete terminal state within tolerance;
- no obstacle or safety-margin violation over full intervals;
- no position, velocity, or acceleration limit violation;
- continuous unwrapped azimuth when seam wrapping is enabled;
- correct first-arrival measurement excluding trailing holds;
- positive velocity carry through turns intended to remain in motion; and
- deterministic output for identical deterministic inputs.

Moving-scene tests also cover time interpolation, uncertainty padding, feasible
waits, capture state, and optional trailing. Failure tests distinguish proved
infeasibility from a time-limited unknown result.

For every benchmark, record the input or seed, validation status, arrival,
execution duration, terminal errors, clearance, limit usage, angular travel,
velocity carry, waits, planning wall time, memory when available, and guarantee
class.

## Claims

Use the guarantee vocabulary in `PLAN.md`. A validated command is not
automatically minimum-arrival. A global claim requires an independent
continuous lower bound or an equivalent proof, including its model and
tolerance.

Visualizations support understanding but never certify safety, dynamics, or
optimality.

## Handoff discipline

Keep the core documentation limited to the greenfield assignment and the work
produced from it.

At each handoff, leave:

- the exact behavior implemented or still failing;
- reproduction commands and seeds;
- arrival and validation evidence;
- tests and static checks that passed;
- any generated or uncommitted files identified explicitly; and
- the next smallest evidence-backed task.
