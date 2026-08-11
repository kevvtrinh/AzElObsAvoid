# Greenfield Azimuth/Elevation Obstacle-Avoidance Plan

## 1. Greenfield mandate

Design and implement a new planner from first principles. For this assignment,
no planner is assumed. The work begins with only:

- the problem and acceptance criteria in this document;
- the MATLAB conventions in [`STYLE.md`](STYLE.md); and
- the canonical `azElData` format produced by
  [`makeAzElObstacleData.m`](makeAzElObstacleData.m).

Files outside this starting set are out of scope during architecture design.
Derive the architecture from the physical problem, prove each important
invariant independently, and let measured evidence determine what is retained.

## 2. Mission

Create a dependable, self-contained MATLAB planner that turns time-indexed
azimuth/elevation exclusion regions into an executable pointing command.

The command must:

1. avoid every obstacle and its requested safety margin;
2. obey position, velocity, and acceleration limits throughout the motion;
3. honor the complete initial and terminal states;
4. reach the goal at the minimum possible physical arrival time;
5. move smoothly and carry velocity through safe turns rather than stopping at
   artificial intermediate points; and
6. provide independent evidence that the returned command is valid.

The initial scope begins at `azElData`. Orbit propagation, sensor projection,
and the upstream production of measured az/el boundaries are outside the
planner.

## 3. Fixed input contract

The input format is already decided and must not be redesigned. One obstacle is
one scalar `azElData` structure:

```matlab
azElData = struct( ...
    "targetName", obstacleName, ...
    "time_s", time_s, ...
    "az_deg", {azimuthBoundaryByTime_deg}, ...
    "el_deg", {elevationBoundaryByTime_deg}, ...
    "status", statusByTime);
```

The fields mean:

- `targetName`: nonempty scalar text used for display and diagnostics.
- `time_s`: nonempty, finite, strictly increasing sample times in seconds.
- `az_deg`: column cell array containing one azimuth boundary vector per time.
- `el_deg`: column cell array containing the matching elevation boundary
  vector per time.
- `status`: scalar text or one status string per time sample.

At each time sample, paired azimuth and elevation vectors have equal length.
Paired nonfinite rows may separate disconnected polygon regions. Empty geometry
is represented by `zeros(0, 1)` in both coordinate cells.

Multiple obstacles may be supplied as a struct array or a cell collection.
Each obstacle owns its time grid; the planner must not require all obstacles to
share sample times.

### Required az/el data builder

Keep `makeAzElObstacleData.m` intact as the supported construction path:

```matlab
azElData = makeAzElObstacleData( ...
    obstacleName, time_s, azimuthBoundary_deg, elevationBoundary_deg);
```

- Numeric boundary vectors describe a static polygon repeated over the given
  time base.
- Cell-array boundaries describe independently sampled time-varying polygons.
- The output must preserve the exact canonical field names and units above.

The new planner must consume this format directly. It may create private packed
or indexed representations internally, but callers must never be required to
construct them.

## 4. Planning request

Use one ordinary request contract for all supported scenes. The design should
accept canonical obstacle data, an initial state, a fixed or moving goal,
physical limits, and mission options.

A fixed boundary state has:

```matlab
state = struct( ...
    "time_s", stateTime_s, ...
    "position_deg", [azimuth_deg, elevation_deg], ...
    "velocity_deg_s", [azimuthRate_deg_s, elevationRate_deg_s], ...
    "acceleration_deg_s2", [azimuthAcceleration_deg_s2, ...
        elevationAcceleration_deg_s2]);
```

Physical limits have:

```matlab
limits = struct( ...
    "azimuth_deg", [minimumAzimuth_deg, maximumAzimuth_deg], ...
    "elevation_deg", [minimumElevation_deg, maximumElevation_deg], ...
    "maxVelocity_deg_s", [maximumAzimuthRate_deg_s, ...
        maximumElevationRate_deg_s], ...
    "maxAcceleration_deg_s2", [maximumAzimuthAcceleration_deg_s2, ...
        maximumElevationAcceleration_deg_s2]);
```

Mission options own only real mission choices, including:

- obstacle safety margin;
- whether azimuth wraps across its seam;
- temporal uncertainty or padding;
- fixed-goal deadline;
- moving-target capture and trailing requirements;
- tolerances used for validation; and
- one total planning-work or wall-time limit.

The caller must not supply intermediate waypoints, corridors, motion templates,
scenario identifiers, or tuning knowledge needed to make ordinary cases work.

## 5. Required output

Choose one clear public MATLAB entry point during the first implementation
phase. Its result structure must have the same public fields on success and
failure and make these facts easy to inspect:

- success or failure;
- a plain-language message and structured reason;
- command timestamps;
- wrapped azimuth/elevation positions;
- continuous unwrapped positions when azimuth wrapping is enabled;
- velocity and acceleration histories;
- first valid arrival or capture time;
- intentional waiting intervals;
- resolved mission options and physical limits;
- collision and kinodynamic validation results;
- planning wall time and physical execution duration; and
- the precise strength of any minimum-arrival claim.

Failure is a legitimate result. Distinguish invalid input, blocked endpoints,
an impossible obstacle-free boundary-value request, a proven infeasible
mission, and a search that ended without proving either success or
infeasibility.

## 6. Primary objective

The planner's primary trajectory objective is minimum physical arrival time.
Candidate results are compared in this order:

1. independently validated collision safety and kinodynamic feasibility;
2. earliest physical arrival;
3. smoother velocity-carrying motion for arrivals equal within tolerance;
4. shorter angular travel for otherwise equivalent results; and
5. lower planning runtime and memory use.

Keep these quantities distinct:

- planning wall time;
- physical command start time;
- physical first-arrival time;
- execution duration from command start to arrival; and
- optional operational delay when planning blocks motion.

A shorter geometric path is worse if it arrives later. A quick computation is
not a substitute for a quicker executable motion. A hold after first arrival
does not count toward the arrival metric.

## 7. Kinodynamic motion requirements

The pointing system begins and ends with a state, not only a coordinate. The
planner must use and preserve nonzero incoming velocity and acceleration and
must meet the requested terminal velocity and acceleration.

Every returned command must:

- remain inside azimuth and elevation position bounds;
- remain inside both axes' velocity bounds;
- remain inside both axes' acceleration bounds;
- use finite, strictly increasing timestamps;
- maintain continuous position and velocity;
- contain no numerical chatter or seam-induced derivative spikes;
- coordinate both axes on one realizable time history;
- enter and leave any intentional wait within the same physical limits; and
- carry useful velocity through safe changes of direction.

Do not force zero velocity at internal points merely because the internal path
description has segments. A full stop is appropriate only when required for
safety, boundary conditions, or the minimum-arrival motion.

"Smooth" means at minimum continuous position and velocity with bounded
acceleration. Prefer continuous acceleration and reasonable jerk, but never
claim a jerk constraint unless it is requested and independently validated.

## 8. Obstacle and time requirements

- Treat occupied space as the union of every obstacle region active at the
  queried time.
- Apply the safety margin to polygon interiors, edges, and vertices.
- Handle disconnected regions separated by nonfinite rows.
- Preserve independent obstacle time grids.
- State and test the interpolation rule between supplied time samples.
- State and test behavior before an obstacle's first time and after its last.
- Apply temporal uncertainty consistently to endpoint and trajectory checks.
- Treat azimuth wrapping as explicit mission policy.
- Retain an unwrapped continuous command even when display positions wrap.
- Use the same geometry and time semantics for validation and visualization.

Checking only returned waypoints is insufficient. The full motion between
samples must be certified collision-free at a justified resolution or by a
continuous check.

## 9. Independent verification

Build acceptance checks separately from command construction. A successful
result is not accepted until independent code verifies:

- input and timestamp validity;
- exact initial-state agreement;
- terminal position, velocity, and acceleration tolerance;
- position, velocity, and acceleration limits over the full command;
- collision clearance over the full command and safety margin;
- seam continuity and correct wrap semantics;
- feasibility of every wait; and
- correct first-arrival measurement.

Maintain obstacle-free minimum-time calculations for boundary states that
admit an independent solution. These are lower bounds for obstructed scenes and
exact arrival oracles when the lower-bound motion is collision-free.

Use these claim labels:

- **Validated feasible:** all independent safety and motion checks pass.
- **Minimum arrival under a stated model:** a reproducible bound or certificate
  supports the claim within that model and tolerance.
- **Globally minimum arrival:** an independent continuous lower bound is met or
  an equivalent proof is provided.
- **Best found:** a validated command is available without a global proof.
- **Unknown:** no validated command was found, but infeasibility was not proved.

Never infer global optimality from visual quality, dense sampling, or strong
benchmark performance alone.

## 10. Acceptance cases and metrics

Build a deterministic acceptance suite containing:

- unobstructed boundary-value cases with independent minimum-time answers;
- static detours with several turns and measurable velocity carry;
- narrow-clearance and near-tangent safety-margin cases;
- moving obstacles where departure time or waiting matters;
- wrapped and non-wrapped azimuth-seam cases;
- nonzero initial and terminal velocity and acceleration;
- multiple obstacles with different time grids;
- disconnected regions within one time sample;
- fixed and moving goals;
- infeasible and time-limited requests with honest status; and
- seeded randomized holdouts that are never discarded for being difficult.

For each case, record:

- input or seed identity;
- success and independent-validation status;
- arrival time and execution duration;
- terminal state errors;
- maximum position, velocity, and acceleration violations;
- minimum obstacle clearance;
- angular distance traveled before arrival;
- velocity carried through non-waiting interior turns;
- wait count and total wait duration;
- planning wall time and peak memory when available; and
- the guarantee label from Section 9.

Judge results by feasibility first and arrival second. Use runtime and memory as
practical constraints and smoothness metrics as evidence of motion quality.

## 11. Build phases

### Phase A: Specify the public boundary

- Freeze the input, state, limits, options, and output schemas.
- Choose one public entry-point name and one defaults mechanism.
- Define units and tolerances beside every public field.
- Create input-contract and empty-result tests before implementing planning.

### Phase B: Build independent authorities

- Implement collision and clearance queries directly from canonical data.
- Implement full-command kinodynamic validation.
- Implement terminal-state and first-arrival measurement.
- Implement obstacle-free minimum-time oracles for supported boundary states.

### Phase C: Establish simple end-to-end motion

- Solve unobstructed requests first.
- Produce synchronized two-axis position, velocity, and acceleration histories.
- Verify nonzero boundary velocities and accelerations.
- Verify wrapped and non-wrapped azimuth behavior.

### Phase D: Add obstacle avoidance

- Generate safe motion around static obstacle regions.
- Preserve velocity through safe turns.
- Validate full intervals, not only internal path points.
- Return the earliest validated command found within the work limit.

### Phase E: Add time-varying behavior

- Account for moving obstacle geometry and independent time grids.
- Add physically feasible departure choices and intentional waits.
- Add moving-target capture and optional continued tracking.
- Verify time interpolation and uncertainty behavior independently.

### Phase F: Improve minimum arrival

- Compare every improvement against independent lower bounds where available.
- Prefer earlier validated arrival over shorter distance.
- Remove artificial interior stops and unnecessary waits.
- Keep the best validated incumbent throughout bounded planning.

### Phase G: Harden and hand off

- Run deterministic unit, integration, randomized, and headless visual tests.
- Run MATLAB Code Analyzer on every new or changed function.
- Record seeds and exact reproduction commands for every unresolved failure.
- Keep `PLAN.md`, `STYLE.md`, `AGENTS.md`, and public help synchronized.

## 12. Guardrails

- Do not weaken validation to gain speed.
- Do not make behavior depend on example names, obstacle names, filenames, or
  seeds.
- Do not store routes or waypoints for known demonstrations.
- Do not expose internal tuning as required mission input.
- Do not silently change units, field names, time semantics, or wrap policy.
- Do not report position-only arrival when terminal velocity or acceleration is
  wrong.
- Do not report a time-limited unknown result as proved infeasible.
- Do not force a stop at an interior turn unless physics or safety requires it.

## 13. Definition of done

The greenfield planner is complete when one ordinary caller contract consumes
the canonical `azElData` format and reliably returns independently validated,
minimum-arrival-seeking, kinodynamically feasible commands across the required
static, time-varying, wrapping, multi-obstacle, and moving-target cases.

Motion is smooth and carries velocity through safe turns. Failure results are
honest and actionable. Reproducible tests support every public guarantee. A new
maintainer can understand the inputs, physics, objective, evidence, and next
steps using this brief alone.
