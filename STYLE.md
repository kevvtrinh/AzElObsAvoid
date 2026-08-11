# Greenfield Az/El Planner Style

This style applies to the new MATLAB planner, validators, examples, and tests.
Write the implementation from the problem contract in [`PLAN.md`](PLAN.md).

## 1. Design posture

- Prefer small functions with explicit data flow.
- Use value structures for requests, obstacle data, options, and results.
- Avoid handle classes, hidden mutable state, and global configuration.
- Keep public mission inputs separate from private implementation controls.
- Make deterministic behavior the default.
- Make units and physical meaning visible in names.
- Keep command construction separate from independent validation.

## 2. Function headers

Every function begins immediately after its declaration with:

```matlab
%% Section 0: Header & Readme
```

The help block lists every supported call under `SYNTAX`, then uses these
sections in order: `PURPOSE`, `INPUTS`, `OUTPUTS`, and `UNITS`. Use `%` followed
by 74 asterisks as the separator.

Purpose entries use `%   -` bullets. Arguments use `%   - name (type)`, with
descriptions on the next line and structure fields nested one level deeper.
Document defaults, tolerances, dimensions, and failure behavior beside the
field they govern.

## 3. Executable sections

Use numbered, title-cased sections in execution order:

```matlab
%% Section 1: Validate Inputs & Apply Defaults
%% Section 2: Build The Internal Representation
%% Section 3: Construct Candidate Commands
%% Section 4: Independently Validate Candidates
%% Section 5: Assemble The Result
%% Section 6: Local Functions
```

Adapt titles to the function. Do not place `%%` sections inside loops or
conditionals. Use a dashed comment for meaningful internal stages:

```matlab
% --- Validate Continuous Clearance --------------------------------
```

A short helper that expresses one idea needs no executable sections.

## 4. Naming and units

Spell words out. Use `maximumVertices`, not `maxVerts`, and
`candidateArrivalTime_s`, not `candT`.

Use these suffixes:

| Suffix | Meaning |
| --- | --- |
| `_deg` | degrees |
| `_s` | seconds |
| `_deg_s` | degrees per second |
| `_deg_s2` | degrees per second squared |
| `_deg_s3` | degrees per second cubed |
| `_deg2` | degrees squared |
| `_1_deg` | per degree |
| `_rad` | radians |
| `_rad_s` | radians per second |

Dimensionless values have no suffix. Boolean names read as assertions, such as
`isTimeOrdered`, `commandIsCollisionFree`, and
`terminalStateIsInsideTolerance`.

Use lower camel case for public structure fields. Use Pascal case only for
private packed records when that distinction materially improves readability.

Error and warning identifiers use the emitting function and a Pascal-case
condition:

```matlab
error("validatePlannerRequest:InvalidBoundaryState", ...)
warning("buildObstacleIndex:RegionDropped", ...)
```

## 5. Canonical obstacle data

Do not change the public `azElData` fields or units:

- `targetName`
- `time_s`
- `az_deg`
- `el_deg`
- `status`

Use [`makeAzElObstacleData.m`](makeAzElObstacleData.m) to build synthetic or
example obstacles. Numeric boundary inputs are static; cell boundaries vary by
time. Generated and externally supplied records must pass the same validation
rules.

Private packing is allowed for speed, but no private field or storage layout
may become required caller input. Visualization and independent collision
validation must interpret canonical geometry identically.

## 6. Options

One function owns the complete default options structure. Partial option
structures remain valid: omitted or empty fields receive defaults. Unknown
fields warn once and are ignored unless accepting them would be unsafe.

Echo resolved public options in the result. Keep internal tuning private. A
caller-facing option represents a physical limit, mission choice, validation
tolerance, or one overall resource bound.

## 7. Validation

Validate failures that would otherwise be silent or appear far downstream:

- required fields and structural types;
- finite, strictly increasing time;
- paired polygon coordinate lengths;
- state dimensions and units;
- physical limit consistency;
- boundary states inside position and motion limits; and
- options whose bounds would silently discard data.

Use `validateattributes` for ordinary numeric checks and identified errors for
structural or semantic failures. Include the affected obstacle, sample, region,
or command index and the actual value when useful.

Candidate construction must not certify itself. Call a separate validator that
checks the complete returned command against canonical obstacles, safety
margin, time semantics, physical limits, and terminal-state tolerance.

## 8. Motion calculations

Treat timestamp, position, velocity, and acceleration histories as one coherent
command. Any function that changes the path or timing must update all state
histories consistently.

- Never smooth only plotted positions.
- Never hide a wait with repeated timestamps.
- Never compute derivatives across a wrapped azimuth discontinuity.
- Preserve a continuous unwrapped azimuth history.
- Never declare arrival from position alone when a terminal rate or
  acceleration is requested.
- Do not impose zero speed at internal points without a physical reason.
- Add explicit assertions for velocity carry through turns intended to remain
  in motion.

Name stationary intervals `wait` or `hold`. Reserve `arrival` for the first
time the complete terminal state is satisfied.

## 9. Time language

Use these terms precisely:

- `planningElapsed_s`: wall-clock computation time.
- `arrivalTime_s`: absolute first-valid-arrival time.
- `executionDuration_s`: command start to first valid arrival.
- `operationalArrivalDelay_s`: delay including blocking planning, when modeled.

Do not use the bare word `time` when two meanings could be confused. Exclude
post-arrival holds from arrival and distance metrics.

## 10. Comments

Comments explain why, the invariant being protected, or the consequence of a
choice. Do not narrate syntax. Explain every non-obvious tolerance, numerical
guard, and deliberate asymmetry at the point of use.

A shared helper states the invariant that would diverge if its logic were
duplicated. Mathematical comments name the modeled quantity and units before
giving the formula.

## 11. Warnings and failures

Warn when input geometry, samples, or requested behavior are reduced, dropped,
or ignored and the result alone would not reveal it. Accumulate loop warnings
and emit one useful summary per obstacle or result.

Return structured failure records for valid but unsolved requests. Reserve
errors for invalid calls, broken invariants, corrupt internal state, or unsafe
conditions that prevent a meaningful result.

Never label resource exhaustion as proved infeasible.

## 12. Result schemas

Every exit path returns the same public fields. Construct results from one
template helper. Preallocate record arrays from a shared template. When the
final size is unknown, grow private storage geometrically and trim once.

Keep diagnostics factual and stable. Separate result fields for feasibility,
arrival, collision evidence, motion-limit evidence, compute usage, and the
strength of any optimality claim.

## 13. Layout

Target about 78 characters per line. Continue long expressions with `...` and
indent continuations four spaces. Replace complex multi-line conditions with
named intermediate assertions when that improves debugging.

Do not end a line with an assignment or comparison operator followed only by
`...`. Put the first meaningful term on that line or name an intermediate
quantity.

Prefer readable vector operations. Otherwise use explicit loops with diagnostic
indices such as `sampleIndex`, `obstacleIndex`, `regionIndex`, and
`commandIntervalIndex`. Do not use `cellfun` or `arrayfun` in planner code.

## 14. Examples

Each example follows one visible progression:

1. build canonical obstacle data;
2. define initial and terminal states;
3. define physical limits and mission options;
4. call the one public entry point;
5. independently validate the returned command; and
6. report and optionally visualize the result.

Examples are mission descriptions, not tuning files. Do not include hidden
routes, waypoints, scenario-specific settings, or stored answers.

## 15. Tests and benchmarks

Write function-based tests with descriptive names. A regression test records
the physical condition it protects, not an internal mechanism.

Randomized generators use explicit seeds and preserve draw order during code
changes. Never regenerate a case because it fails. Benchmarks record input or
seed identity, environment, validation, arrival, terminal error, clearance,
limit use, velocity carry, waits, planning runtime, and memory where available.

Compare feasibility first and arrival second. Distance and visual smoothness
are supporting measures, not substitutes for arrival.

## 16. Documentation

Public help and Markdown lead with the problem, units, contracts, observable
behavior, and evidence. Keep the handoff limited to the greenfield assignment
and the results produced from it.

Links use relative repository paths. Code snippets are runnable MATLAB. When a
public field, unit, tolerance, or guarantee changes, update the help, plan,
tests, and maintainer instructions in the same change.
