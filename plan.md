# Plan 325 Planner Rebuild Plan

## Objective

Build one compact and general azimuth/elevation planner from the useful parts
of the inspected branches. Do not combine old planners as independent layers.
Keep one public planner, one seed generator, two bounded motion families, one
independent validator, and one result contract.

The production pipeline is:

```text
canonical original and protected obstacles
    -> bounded direct, sampled spatial, reduced, and timed seed proposals
    -> deterministic finite-jerk first motion when supported
    -> optional bounded separated HS3 improvement
    -> independent continuous validation
    -> deterministic candidate selection
```

The deterministic first motion is a real trajectory. It is not a waypoint
hint. It uses rest-to-rest quintic motion on each edge and stops at each
waypoint. It supports fixed-position goals with zero initial and terminal
velocity and acceleration. Every first motion needs independent validation.

HS3 remains the general local trajectory optimizer. It is required when the
first-motion family cannot represent the request. It is also an optional
improvement stage for a valid first motion. Improvement is enabled by default
with a 15 second budget.

The implementation must follow `AGENTS.md`. This includes the planner
contract, stable failure results, units, examples, diagnostics, validation,
MATLAB headers, and verification reports.

## Branch

This implementation uses the Git branch `plan-325`. It uses an isolated
worktree so that work on Plan 502 is unchanged.

## Non-Negotiable Size Limits

Treat code size as an acceptance requirement, not a future cleanup task.

- Production MATLAB code, excluding examples, tests, and benchmarks: target
  no more than 6,000 lines and hard limit 7,000 lines.
- Entire maintained MATLAB repository, including all 14 main examples: target
  no more than 10,500 lines and hard limit 12,000 lines.
- No production MATLAB file should exceed 900 lines without a documented
  reason and explicit approval.
- Target 8 to 16 production files with substantial responsibilities. Do not
  replace a few large files with dozens of one-function fragments.
- Do not count comments or headers as an excuse to exceed the limits; they are
  part of the maintained code.
- At the end of every phase, report production and total MATLAB line counts.
- If a feature would violate these limits, first simplify the feature or defer
  it. Do not silently enlarge the architecture.

## Required Planner Contract

Keep one public entry point:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

Keep the zero-input defaults call:

```matlab
options = planAzElMotion();
```

The planner supports these input families through this interface:

- static and known time-varying polygon obstacles;
- fixed-arrival and earliest-arrival goal policies;
- fixed-position and sampled moving goals;
- initial and terminal position, velocity, and acceleration;
- per-axis velocity, acceleration, and jerk limits;
- waiting in timed seed proposals;
- safety margins applied exactly once by obstacle construction;
- deterministic behavior for identical inputs and options;
- success and expected failure through one stable result schema.

Azimuth wrapping is a restricted input family. It is currently supported only
for an obstacle-free request with a fixed-position goal. Reject wrapping with
obstacles or a moving goal as an unsupported configuration. This restriction
protects physical collision and target correctness at the coordinate seam.

The stable result includes:

- status, message, and termination reason;
- resolved inputs, limits, and options;
- original and protected obstacle data;
- bounded search diagnostics and every attempted seed summary;
- the selected seed and selected motion source;
- sampled position, velocity, acceleration, and jerk histories;
- the exact segment polynomial;
- independent collision and constraint validation;
- arrival time, trajectory duration, and goal horizon;
- elapsed time, first validated motion time, and deadline overrun.

Expected planning outcomes return a result. Invalid inputs and unsupported
configurations throw identified errors.

## Honest Result Claim

Use this claim:

> The result is the earliest independently validated candidate from the finite
> deterministic seed and motion families that the planner attempted.

For arrival times within `ArrivalTimeTolerance_s`, select the candidate with
the lowest exact integrated squared jerk. Use the deterministic seed index for
the final tie.

Do not claim global time optimality, global path completeness, or proof that
no feasible trajectory exists. A `noValidatedSeed` result means only that the
bounded attempted families did not produce a valid result.

## Bounded Seed Generator

Use one deterministic seed generator. It supplies proposals and diagnostics.
It does not make a global planning claim.

Generate a bounded mixture of:

1. the direct seed;
2. spatial visibility seeds from distinct bounded 2-D homology signatures;
3. time-layer seeds with motion and wait edges for changing obstacles when
   their estimated query work is inside the seed-search work limit.

The default maximum seed count is five. The hard public maximum is nine.
Moving histories use bounded time layers, nodes, and collision samples.
Retain the complete generated and rejected counts in diagnostics.

Augment each spatial graph state with principal-angle path integrals about one
interior representative of each connected sampled obstacle region. Keep only
signature components from -1 through 1. Stop at the public route count, 4,000
augmented states, or the seed deadline. Record representatives, discovered
signatures, state count, and truncation state. This is a bounded 2-D homology
proposal search. It is not a continuous Az/El/time homotopy certificate.

Spatial proposal construction can use a conservative dense-history support
region or an optional clustered convex region. Mark each affected seed with
`UsesReducedGeometry`. Use these regions only for spatial proposals and for a
certified seed corridor. Verify that the reduced region contains the original
protected geometry before a corridor can support a first motion.

Timed search queries the original protected obstacle histories at a bounded
set of edge samples. It must not use a cluster or dense-history envelope in
place of an obstacle. The dense-history gate suppresses timed search and
reports `timedQueryWorkLimit`; this makes coverage incomplete. The generator
keeps a base spatial route before optional timed work starts. HS3 and
independent validation use the original protected histories. Only final
adaptive validation certifies continuous collision freedom.

`EstimatedDuration_s` is an HS3 initial guess. It is not a causal constraint
and is not a lower bound on final time. The goal-time policy, endpoint state,
and physical limits define the permitted time range.

## Deterministic First Motion

Construct supported piecewise quintic trajectories in seed order. Stop this
stage at the first independently validated motion. Each trajectory has these
properties:

- position follows each seed edge;
- velocity and acceleration are zero at every edge boundary;
- position, velocity, and acceleration are continuous;
- each edge duration satisfies analytic velocity, acceleration, and jerk
  limits;
- the result contains exact polynomial coefficients and sampled histories;
- the public validator checks the polynomial, histories, endpoints, limits,
  and continuous collision state.

This motion family supports zero initial and terminal derivatives. It supports
a fixed-position goal, a moving goal with a fixed arrival time, and stationary
wait edges with a finite seed duration. It rejects an earliest-arrival moving
goal and nonzero endpoint derivatives. HS3 handles these cases when the
general optimizer finds a valid solution.

The first motion can use an exact seed corridor. It can use a reduced spatial
corridor only after an independent containment and separation certificate.
The certificate and final trajectory validation both use the exact protected
obstacles.

## Bounded HS3 Stage

Use the separated third-order Hermite-Simpson formulation with jerk as the
control. The state is position, velocity, and acceleration. Reconstruct the
state from one consistent third-order chain.

Use HS3 in two cases:

1. Run it as an improvement when `EnableHs3Improvement` is true.
2. Run it as the required general motion stage when no valid first motion
   exists, even if optional improvement is disabled.

`EnableHs3Improvement` is true by default.
`MaximumHs3ImprovementTime_s` is 15 seconds by default. Divide the remaining
improvement budget across the remaining seed attempts. A failed, invalid,
later, or higher-jerk HS3 result must not replace a better valid motion.

For earliest arrival, use final time and jerk ordinates as decision variables.
Use two objective stages:

1. minimize final time;
2. hold time within its tolerance and minimize exact integrated squared jerk.

Compute the squared-jerk integral from the polynomial. Do not use a quadrature
rule that is not exact for the jerk polynomial. Mesh refinement can replace a
candidate only when it does not make arrival time or exact jerk cost worse.

HS3 obstacle constraints use exact protected obstacle histories. Reduced
geometry can initialize a spatial corridor. It cannot replace exact geometry
in optimization or final validation.

## Cooperative Deadlines

Use `MaximumPlanningTime_s` for the complete planner call. Use
`MaximumHs3ImprovementTime_s` for optional improvement. Pass the remaining
time to HS3 and to independent validation.

Deadline checks are cooperative. Check them:

- before and after seed construction units;
- before each first-motion and HS3 attempt;
- inside solver callbacks;
- during adaptive collision validation;
- before optional mesh refinement.

A bounded operation can complete after the last check. Report this as
`PlanningDeadlineOverrun_s`. Do not hide an overrun. Preserve a valid first
motion if the optional improvement deadline ends.

## Independent Continuous Validation

Use one public validator for planner selection, tests, and examples. It must
not trust a solver status or the planner success flag.

Validate:

- finite and strictly increasing sampled time;
- exact polynomial schema and segment time base;
- polynomial continuity at every segment boundary;
- initial and terminal position, velocity, and acceleration;
- agreement between the polynomial and sampled histories;
- continuous workspace, velocity, acceleration, and jerk limits;
- continuous collision clearance against exact protected moving geometry;
- safety-margin provenance and the wrapping policy;
- deadline state and unresolved adaptive intervals.

Use analytic polynomial bounds when practical. Use adaptive subdivision for
moving-obstacle collision checks. Fail an interval when its relative motion
bound cannot prove separation at the minimum time step.

Do not accept a trajectory by clipping it. Do not reconstruct missing
polynomials from sampled output. Do not infer a passed check from a solver
success code.

## Minimal Production Architecture

Keep these responsibilities separate without adding another planner:

| Responsibility | Target size |
| --- | ---: |
| `planAzElMotion.m`: validation, orchestration, selection, result | 300-500 |
| Obstacle construction, packing, interpolation, and query | 700-1,000 total |
| One bounded topology-seed generator | 400-700 |
| First-motion construction | 350-650 |
| HS3 transcription and solve | 800-1,400 total |
| Independent collision and kinematic validation | 400-700 |
| Plotting and animation | 500-800 total |

Keep obstacle construction, seed proposal, motion construction, validation,
and visualization separate. Do not add SIPP, snapshot planners, parallel
planner modes, scenario-specific waypoints, or another result schema.

## Implementation Phases

### Phase 1: Preserve the Useful Baseline

- Start Plan 325 from the inspected Plan 502 revision.
- Use an isolated worktree.
- Record the branch, source revision, repository size, and baseline behavior.
- Preserve Plan 502 and all unrelated user changes.

Exit criterion: the isolated Plan 325 branch and baseline evidence exist.

### Phase 2: Restore the Required Example Contracts

- Restore the original single U geometry and request.
- Restore the 40-circle moving field and goal-time policy.
- Restore the native changing U.S. outline and goal-time policy.
- Keep compact controls as runtime overrides only.

Exit criterion: tests protect the physical geometry and request of each case.

### Phase 3: Add the Deterministic First Motion

- Implement the rest-to-rest quintic edge invariant once.
- Produce the full polynomial and sampled result schema.
- Validate each candidate independently.
- Reject unsupported endpoint and moving-goal cases clearly.

Exit criterion: analytic tests prove endpoint, continuity, limit, and
collision behavior.

### Phase 4: Rebuild Bounded Seed Coverage

- Keep direct and original-geometry sampled timed seed search.
- Replace side restrictions and edge-removal retries with bounded
  homology-signature spatial search.
- Permit reduced geometry only for spatial proposal work.
- Add containment certificates for reduced corridors.
- Record unreduced sampled, reduced, timed, and completeness diagnostics.

Exit criterion: the single U, 40-circle, and changing U.S. families receive
bounded deterministic proposals without scenario-specific logic.

### Phase 5: Bound and Correct HS3 Improvement

- Use the first valid motion as the retained baseline.
- Attempt all bounded seeds in deterministic order.
- Treat timed seed duration as an initial guess.
- Use exact jerk integration and safe final-time bounds.
- Keep a refined or improved candidate only when selection quality does not
  become worse.

Exit criterion: a failed optional solve cannot erase a valid candidate, and a
request outside the first-motion family can still use HS3.

### Phase 6: Strengthen Validation and Deadlines

- Validate the polynomial schema, time base, continuity, endpoints, and
  sampled-history agreement.
- Use exact protected obstacle histories for continuous validation.
- Pass remaining deadline values through all expensive stages.
- Report first-valid time and any deadline overrun.

Exit criterion: an unrelated, shifted, discontinuous, or falsely sampled
polynomial cannot pass.

### Phase 7: Verify the Complete Repository

Run one MATLAB process at a time:

- Code Analyzer and syntax checks;
- focused first-motion tests;
- HS3, seed, moving-target, wrapping, and validator tests;
- default and override tests;
- a successful plan and an expected no-path result;
- every maintained example headlessly;
- at least one visible example when graphics are available.

For each example, report the required path, duration, collision, kinematic,
validation, runtime, and termination metrics. Do not omit an unfavorable
result.

Exit criterion: reports contain the exact commands, results, runtimes, and
untested items.

### Phase 8: Audit Size and Publish

- Count production and total MATLAB lines.
- Inspect per-file additions and deletions.
- Explain every existing source file with more than 50 added lines.
- Update the benchmark and assessment from the new evidence.
- Commit and push branch `plan-325`.

Exit criterion: the pushed branch contains the tested implementation and its
evidence.

## Required Tests

At minimum, tests must cover:

- exact first-motion endpoint states and polynomial continuity;
- continuous velocity, acceleration, and jerk limits;
- collision checks between returned samples;
- unsupported first-motion states route to HS3 without false success;
- direct, unreduced sampled spatial, reduced spatial, and timed seeds;
- timed duration as an initial guess, not a lower bound;
- moving and deforming obstacles evaluated at trajectory time;
- reduced corridor containment against exact protected geometry;
- earliest and fixed arrival;
- fixed and moving goals;
- nonzero endpoint derivatives through HS3;
- deterministic candidate ordering and selection;
- optional HS3 failure with retained valid first motion;
- exact jerk cost and no-worse mesh refinement;
- deadline termination and reported overrun;
- obstacle-free azimuth wrapping;
- rejected wrapping with obstacles and with moving goals;
- expected no-path with stable search diagnostics;
- the three restored large-scenario input contracts.

## Completion Criteria

Plan 325 is complete only when all these statements are true:

- One public planner owns selection and returns one stable schema.
- One bounded seed generator produces direct, unreduced sampled spatial,
  reduced spatial, and original-geometry sampled timed proposals. Spatial
  routes retain distinct bounded 2-D homology signatures.
- A supported seed can produce a deterministic independently validated first
  motion.
- HS3 is bounded, optional for improvement, and required for unsupported
  first-motion requests.
- Timed edge samples, HS3, and validation use original protected histories.
- Reduced geometry is limited to spatial proposals and certified corridors.
- Every successful result passes independent continuous validation.
- Expected failures retain useful diagnostics.
- Deadline overrun is measured and reported.
- Azimuth wrapping restrictions are explicit and enforced.
- Production MATLAB code is at or below 6,000 lines and below the 7,000-line
  hard limit.
- The complete MATLAB repository is at or below the 10,500-line target and
  below the 12,000-line hard limit.
- The final diff contains no scenario-specific planner behavior.
- Documentation makes no global completeness or optimality claim.

## Stop Conditions

Stop and report before adding more architecture when:

- an example appears to need a scenario-specific heuristic;
- a feature creates a second planner, retimer, or validator;
- production code would exceed 7,000 lines;
- exact validation cannot resolve collision or constraint safety;
- a deadline, reduction, fallback, or failure would be hidden;
- required verification cannot run in the available MATLAB environment.

The result must be small enough to understand and strict enough to trust. It
must report the limits of the bounded search and local optimization.
