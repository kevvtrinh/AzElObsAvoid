# Plan 325 Repository Consistency Audit

**Repository:** `kevvtrinh/AzElObsAvoid`
**Branch audited:** `plan-325`
**Branch HEAD observed:** `b845880` (`Record bounded minimum-time experiments`, 2026-08-20)
**Audit date:** 2026-08-20
**Scope:** static cross-file consistency audit of the current `plan-325` branch, with `AGENTS.md`, `plan.md`, `README.md`, the public MATLAB APIs, examples, plotter, internal planner stages, and tests treated as contracts.

> This is a consistency audit, not a claim that the branch is broken. The branch's own assessment reports 52 passing tests, 18 maintained examples, 17 validated successes, and the expected no-path failure. Several inconsistencies below are specifically gaps that can coexist with a green test suite.

## Executive summary

The branch is substantially more coherent than the older planner stacks, but it still has several contract drifts. The most important ones are:

1. `MaxJerk_deg_s3` is advertised as a uniform example override, but only 10 of 18 examples actually apply it to `limits`; 8 silently keep a hard-coded jerk limit.
2. `RandomSeed` is a public `planAzElMotion` option even though the production planner is deterministic and does not consume it. One example reuses the same field name for scenario randomness and explicitly removes it before calling the planner.
3. `AGENTS.md` says `limits` contains physical **and workspace** limits, but `AzimuthInterval_deg` and `ElevationInterval_deg` live in `options` while only velocity/acceleration/jerk live in `limits`.
4. Verbose behavior is not uniform: 9 examples default it on and 9 inherit the planner default off; additionally, `solveAzElHs3` always sets `fmincon` display to `none`, so verbose mode cannot show optimizer iteration progress.
5. The formal plan says there is one public entry point, but the repository exposes `planAzElMovingTargetIntercept` as a second public callable API with its own option schema.
6. The README understates the deterministic first-motion capability: it says fixed-position goals only, while the plan and implementation support a moving goal at a fixed arrival time.
7. The plotting/example option system has two sources of defaults and two implementations of the same compatibility aliases, contrary to `AGENTS.md`'s single-source/centralized-alias rules.
8. The branch meets hard size limits but misses its own minimal-architecture targets: 26 production MATLAB files versus a target of 8–16, `planAzElMotion.m` is 883 lines versus a 300–500 target, and the seed generator is 900 lines versus a 400–700 target.

---

# 1. Confirmed high-priority inconsistencies

## INC-001 — `MaxJerk_deg_s3` does not mean the same thing in every example

**Severity:** High
**Type:** Functional example-contract inconsistency

`examples/resolveAzElExampleOptions.m` explicitly accepts `MaxJerk_deg_s3`, validates it, and returns it as part of the shared example controls:

- [`resolveAzElExampleOptions.m` lines 71–85](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m#L71-L85)
- [`resolveAzElExampleOptions.m` lines 142–145](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m#L142-L145)

However, the examples split into two behaviors.

### Examples that honor the shared override

These assign `limits.maxJerk_deg_s3` from the resolved shared value:

- `exampleFortyMovingCircleGrid`
- `exampleFourAcceleratingCircles`
- `exampleInterceptMovingTargetAtSetTime`
- `exampleMovingDeformingUSOutlineVisibility`
- `exampleOpeningUShapedAzElTimeSpace`
- `exampleStraightTargetAlternatingOcclusion`
- `exampleTargetExitsObstacle`
- `exampleTwoOpposingUVisibilityGraph`
- `exampleUSOutlineExtremeVisibility`
- `exampleUShapedAzElTimeSpace`

For example, `exampleFourAcceleratingCircles` uses:

```matlab
"maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3
```

See [`exampleFourAcceleratingCircles.m` lines 123–126](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/exampleFourAcceleratingCircles.m#L123-L126).

### Examples that ignore the same override

These still hard-code `[2 2]`:

- `exampleAlternatingSlalom`
- `exampleAzElPlanning`
- `exampleDenseConcaveAzElMotion`
- `exampleInterceptMovingTargetEarliest`
- `exampleMovingBarrierWait`
- `exampleMovingCircleNoAzimuthWrap`
- `exampleNoPathAzElMotion`
- `exampleObstacleFreeAzElMotion`

For example, `exampleAzElPlanning` accepts the shared overrides but then uses:

```matlab
"maxJerk_deg_s3", [2 2]
```

See [`exampleAzElPlanning.m` lines 26–29 and 40–46](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/exampleAzElPlanning.m#L26-L46).

### Why this is inconsistent

A caller can pass the same documented field to two maintained examples and get different physical planner limits. The override is accepted without an error in both cases, which makes the ignored case especially misleading.

### Recommended fix

Make jerk a physical input, not a display control. At minimum, every example should set:

```matlab
limits.maxJerk_deg_s3 = exampleControls.MaxJerk_deg_s3;
```

A cleaner design would stop placing `MaxJerk_deg_s3` inside `displayOptions` entirely and return separate scenario/physical overrides.

### Test to add

Parameterize all 18 examples and verify that an override such as:

```matlab
struct("MaxJerk_deg_s3", [1.23 1.45], "PlotOutputs", false)
```

is reflected exactly in:

```matlab
result.Inputs.limits.maxJerk_deg_s3
```

---

## INC-002 — `RandomSeed` is a public planner option with no production-planner effect

**Severity:** High
**Type:** Dead/stale public API + name collision

`planAzElMotion` exposes, validates, and echoes:

```matlab
"RandomSeed", 0
```

See:

- [`planAzElMotion.m` lines 392–419](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L392-L419)
- [`planAzElMotion.m` lines 475–478](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L475-L478)
- [`planAzElMotion.m` result template](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L803-L845)

But the production topology generator and HS3 solver do not consume `RandomSeed`; the planner is intentionally deterministic.

This conflicts directly with `AGENTS.md`:

> Add an option only when it represents a meaningful user choice.

See [`AGENTS.md` lines 46–50](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L46-L50).

The collision becomes visible in `exampleTargetExitsObstacle`: that example uses `RandomSeed` for **scenario generation**, then explicitly removes the field before resolving planner options:

```matlab
randomSeed = exampleOverrides.RandomSeed;
...
plannerOverrides = rmfield(plannerOverrides, "RandomSeed");
```

See [`exampleTargetExitsObstacle.m` around the example-control block](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/exampleTargetExitsObstacle.m#L38-L60).

### Why this is inconsistent

The same field name has two meanings:

- at the planner boundary: apparently a planner random seed;
- in one example: scenario/target-path randomness.

Yet the production planner itself does not appear to use randomness at all.

### Recommended fix

Choose one:

1. **Preferred:** remove `RandomSeed` from `planAzElMotion` until the planner actually uses randomness, and rename the scenario field to `ScenarioRandomSeed` or `TargetPathRandomSeed`; or
2. deliberately introduce randomized planner behavior and use/return the seed consistently.

Given Plan 325's deterministic design, option 1 is much cleaner.

---

## INC-003 — Workspace limits are in `options`, contrary to the repository's own public contract

**Severity:** High
**Type:** Public API responsibility mismatch

`AGENTS.md` explicitly defines:

- `limits`: **physical and workspace limits**, with units in field names;
- `options`: algorithm and display choices that do not change input meaning.

See [`AGENTS.md` lines 56–75](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L56-L75).

But `planAzElMotion` places these physical workspace bounds in `options`:

```matlab
"AzimuthInterval_deg", [-180 180], ...
"ElevationInterval_deg", [-90 90], ...
```

See [`planAzElMotion.m` lines 395–403](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L395-L403).

Meanwhile `normalizeLimits` requires only:

```matlab
maxVelocity_deg_s
maxAcceleration_deg_s2
maxJerk_deg_s3
```

See [`planAzElMotion.m` lines 560–575](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L560-L575).

The endpoint-feasibility logic then has to read physical bounds from both structures:

- derivatives from `limits`;
- position bounds from `options`.

See [`planAzElMotion.m` lines 576–628](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L576-L628).

### Recommended fix

Move azimuth/elevation bounds into `limits`, for example:

```matlab
limits.azimuthInterval_deg = [-180 180];
limits.elevationInterval_deg = [-90 90];
limits.maxVelocity_deg_s = [2 2];
limits.maxAcceleration_deg_s2 = [1 1];
limits.maxJerk_deg_s3 = [2 2];
```

Keep algorithmic controls such as wrapping policy, seed count, collocation mesh, and solver tolerances in `options`.

This also makes the public API match `AGENTS.md` without changing the conceptual five-input planner interface.

---

## INC-004 — Verbose behavior is not uniform across the maintained examples

**Severity:** High for usability / Medium for correctness
**Type:** Example-default + diagnostics inconsistency

The planner default is:

```matlab
Verbose = false
```

See [`planAzElMotion.m` lines 410–419](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L410-L419).

The 18 maintained examples are split exactly 9/9:

| Example | Default `Verbose` | `MaxJerk_deg_s3` override reaches `limits` |
| --- | ---: | ---: |
| `exampleAlternatingSlalom` | false/inherited | **No** |
| `exampleAzElPlanning` | false/inherited | **No** |
| `exampleDenseConcaveAzElMotion` | false/inherited | **No** |
| `exampleFortyMovingCircleGrid` | **true** | Yes |
| `exampleFourAcceleratingCircles` | **true** | Yes |
| `exampleInterceptMovingTargetAtSetTime` | **true** | Yes |
| `exampleInterceptMovingTargetEarliest` | false/inherited | **No** |
| `exampleMovingBarrierWait` | false/inherited | **No** |
| `exampleMovingCircleNoAzimuthWrap` | false/inherited | **No** |
| `exampleMovingDeformingUSOutlineVisibility` | **true** | Yes |
| `exampleNoPathAzElMotion` | false/inherited | **No** |
| `exampleObstacleFreeAzElMotion` | false/inherited | **No** |
| `exampleOpeningUShapedAzElTimeSpace` | **true** | Yes |
| `exampleStraightTargetAlternatingOcclusion` | **true** | Yes |
| `exampleTargetExitsObstacle` | **true** | Yes |
| `exampleTwoOpposingUVisibilityGraph` | **true** | Yes |
| `exampleUSOutlineExtremeVisibility` | **true** | Yes |
| `exampleUShapedAzElTimeSpace` | false/inherited | Yes |

The branch advertises a uniform example override path, so default diagnostics should not be an accidental per-scenario property.

### Additional verbose inconsistency inside HS3

`solveAzElHs3` hard-codes:

```matlab
"Display", "none"
```

and does not read `Verbose`.

See [`+azElInternal/solveAzElHs3.m` lines 130–143](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/%2BazElInternal/solveAzElHs3.m#L130-L143).

That means a long optimizer call can remain silent even when the top-level planner has `Verbose=true`.

### Recommended fix

- Set one shared example default for `Verbose` in `resolveAzElExampleOptions`.
- Remove per-example `Verbose=true` unless a scenario has a clearly documented reason.
- Have HS3 consume the resolved verbose setting.
- Prefer a throttled custom progress line from the existing `OutputFcn` over raw `fmincon` `Display="iter"`, e.g. iteration, elapsed time, current arrival objective, feasibility/constraint violation, and best validated candidate.

---

## INC-005 — `plan.md` says one public entry point, but a second public planner façade exists

**Severity:** High/Medium
**Type:** Public API contract drift

`plan.md` is explicit:

```text
Keep one public entry point:
result = planAzElMotion(...)
```

See [`plan.md` lines 49–73](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plan.md#L49-L73).

`AGENTS.md` also says to keep one public planner entry point when supported behavior can be expressed through inputs/options:

[`AGENTS.md` lines 45–50](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L45-L50).

But the root of the repository also exposes:

```matlab
planAzElMovingTargetIntercept(...)
```

with its own zero-input defaults call and option schema:

- `InterceptMode`
- `SpecifiedInterceptTime_s`
- `MaximumSearchDuration_s`
- `MatchTargetVelocity`
- `MatchTargetAcceleration`
- `PlannerOptions`

See [`planAzElMovingTargetIntercept.m` lines 1–41](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMovingTargetIntercept.m#L1-L41).

The function does correctly delegate to `planAzElMotion` rather than implementing a second planning algorithm:

[`planAzElMovingTargetIntercept.m` lines 119–163](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMovingTargetIntercept.m#L119-L163).

### Why this is inconsistent

There is one planning **engine**, but not literally one public **entry point**.

### Recommended fix

Choose and document one of these contracts:

- **One engine, multiple public adapters:** change `plan.md`, README, and assessment language from “one public entry point/planner” to “one production planning engine”; or
- **One public entry point literally:** make the moving-target conversion an internal/helper adapter and have examples build `goalState` then call `planAzElMotion` directly.

The second choice is more faithful to the current `plan.md` and `AGENTS.md`.

---

# 2. Documentation and terminology inconsistencies

## INC-006 — README says first motion is fixed-goal only; implementation supports fixed-time moving goals

**Severity:** Medium
**Type:** Documentation-versus-code drift

The README says:

> This family is available for a fixed-position goal...

and repeats under Known Limits that the deterministic first-motion family requires a fixed-position goal.

See [`README.md` first-motion description and Known Limits](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/README.md#L11-L17) and [`README.md` near the Known Limits section](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/README.md#L107-L120).

The latest plan instead states that first motion supports:

- fixed-position goals;
- a **moving goal with a fixed arrival time**;
- stationary wait edges with finite seed duration.

See [`plan.md` lines 140–160](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plan.md#L140-L160).

`buildAzElStopWaypointMotion` implements that broader behavior and rejects only an **earliest-arrival moving goal**, not every moving goal.

### Recommended fix

Update the README to say:

> The deterministic first-motion family supports zero endpoint derivatives, fixed-position goals, and sampled moving goals when the arrival time is fixed. Earliest-arrival moving-goal requests require HS3.

---

## INC-007 — Stale “HS3” terminology remains in generic planner paths

**Severity:** Medium
**Type:** Refactor naming drift

Plan 325 is explicitly a two-motion-family planner: deterministic first motion plus optional/general HS3. Nevertheless, generic planner status lines still use `[HS3]`:

- endpoint rejection: `[HS3] ...`
- no validated seed summary: `[HS3] ...`
- final successful planner summary: `[HS3] ...`

See [`planAzElMotion.m` verbose output](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L92-L120) and [`planAzElMotion.m` final assembly](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L304-L388).

The plotter header also says:

> Plot returned HS3 motion

although the selected motion can be `firstMotion`.

See [`plotAzElMotion.m` lines 1–10](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plotAzElMotion.m#L1-L10).

The primary test file is still named `testHs3Planner.m`, even though it now tests the general Plan 325 planner contract.

### Recommended fix

Use stage-specific labels only when the stage is actually HS3:

- `[planner]`
- `[seed graph]`
- `[first motion]`
- `[hs3]`
- `[validator]`

Rename `testHs3Planner.m` to something such as `testAzElPlanner.m` after a compatibility-safe transition if external automation depends on the filename.

---

## INC-008 — `scenarioDefaults` documentation is narrower than its actual behavior

**Severity:** Medium/Low
**Type:** Helper contract documentation drift

`resolveAzElExampleOptions` documents `scenarioDefaults` as:

> scalar partial `planAzElMotion` options struct

See [`resolveAzElExampleOptions.m` lines 14–19](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m#L14-L19).

But the implementation explicitly extracts display fields from `scenarioDefaults`:

[`resolveAzElExampleOptions.m` lines 43–69](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m#L43-L69).

Examples rely on this behavior for values such as `FigureVisible`, `Title`, and `Verbose`.

### Recommended fix

Document `scenarioDefaults` as a combined partial example-control structure, or better, stop combining planner and display defaults in one struct.

---

# 3. Options, limits, time semantics, and schema inconsistencies

## INC-009 — Three “time limits” use different clocks/semantics

**Severity:** Medium
**Type:** API naming ambiguity

The codebase exposes:

- `MaximumPlanningTime_s`: wall-clock budget for the planner;
- `MaximumHs3ImprovementTime_s`: wall-clock budget for optional local improvement;
- `MaximumSearchDuration_s`: **mission/intercept horizon** in `planAzElMovingTargetIntercept`, not wall-clock compute time.

`MaximumSearchDuration_s` is used as:

```matlab
initialTime_s + options.MaximumSearchDuration_s
```

See [`planAzElMovingTargetIntercept.m` lines 35–41 and 121–139](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMovingTargetIntercept.m#L35-L41).

### Why this matters

Two fields sound like computation/search budgets but one is actually physical trajectory time. This is easy to misconfigure.

### Recommended fix

Rename the moving-target field to something unambiguously mission-time based, e.g.:

```matlab
MaximumInterceptHorizon_s
```

or:

```matlab
InterceptSearchHorizon_s
```

If a larger API cleanup is acceptable, use explicit wall-clock naming for compute budgets, e.g. `PlanningWallTimeLimit_s`.

---

## INC-010 — `goalState.time_s` changes meaning with `GoalTimeMode`

**Severity:** Medium/Low
**Type:** Semantic overloading

For `fixedArrival`, `goalState.time_s` is the required terminal time.

For `earliestArrival`, it serves as the latest allowed horizon rather than the achieved arrival time.

The result then separately exposes:

- `ArrivalTime_s`
- `TrajectoryDuration_s`
- `GoalHorizon_s`

See [`planAzElMotion.m` result assembly](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L349-L388).

### Recommended fix

At minimum, document the mode-dependent meaning immediately beside `goalState.time_s` in the public header. A future breaking cleanup could split this into explicit arrival/horizon fields.

---

## INC-011 — Motion-time vocabulary changes across result layers

**Severity:** Medium/Low
**Type:** Result/diagnostic naming drift

The same conceptual quantities use multiple names:

| Meaning | Names currently used |
| --- | --- |
| achieved terminal time | `FinalTime_s`, `ArrivalTime_s`, `Intercept.Time_s` |
| trajectory duration | `MotionDuration_s`, `TrajectoryDuration_s` |

For example, the public result maps:

```matlab
result.ArrivalTime_s = selectedCandidate.FinalTime_s;
result.TrajectoryDuration_s = selectedCandidate.MotionDuration_s;
```

See [`planAzElMotion.m` lines 377–380](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L377-L380).

Seed summaries use `MotionDuration_s`:

[`planAzElMotion.m` lines 712–730](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L712-L730).

### Recommended fix

Use the public result terminology (`ArrivalTime_s`, `TrajectoryDuration_s`) across public diagnostic records. Internal solver-local names can remain different if they never escape.

---

## INC-012 — Public field casing is mixed inside the same scientific payload structures

**Severity:** Low
**Type:** Naming convention inconsistency

Moving-goal structures use scientific lower-camel/unit-bearing fields:

```matlab
time_s
targetTime_s
targetPosition_deg
```

but interpolation uses PascalCase:

```matlab
InterpolationMethod
```

See [`planAzElMotion.m` moving-goal normalization](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L509-L547) and [`planAzElMovingTargetIntercept.m` target normalization](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMovingTargetIntercept.m#L93-L118).

`AGENTS.md` distinguishes PascalCase status/control fields from scientific payload fields, so `InterpolationMethod` sits awkwardly between the conventions.

### Recommended fix

Do not break compatibility just for style. If a migration is otherwise justified, introduce `interpolationMethod` as the canonical field and retain `InterpolationMethod` as one documented deprecated alias for one release.

---

## INC-013 — Production search policy is split between public options and hidden hard-coded caps

**Severity:** Medium
**Type:** Configuration-source fragmentation

`plannerDefaults` says it defines the complete public planner option source of truth:

[`planAzElMotion.m` lines 392–419](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m#L392-L419).

But `generateAzElTopologySeeds` contains important bounded-work constants outside that source, including:

- dense envelope query-work gate: `10e3`;
- visibility work budget: `1e6`;
- visibility candidate range: `24` to `96`;
- extended timed node count: `24`;
- time-layer cap: `17`;
- retained accepted/rejected edge traces: `2000` each;
- homology-state cap: `4000`.

Representative locations:

- [`generateAzElTopologySeeds.m` lines 54–67](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/%2BazElInternal/generateAzElTopologySeeds.m#L54-L67)
- [`generateAzElTopologySeeds.m` lines 154–170](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/%2BazElInternal/generateAzElTopologySeeds.m#L154-L170)
- [`generateAzElTopologySeeds.m` lines 207–242](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/%2BazElInternal/generateAzElTopologySeeds.m#L207-L242)

### Important nuance

Not every internal cap should become a public user option. In fact, `AGENTS.md` says not to expose internal constants without demonstrated need.

### Recommended fix

Centralize them as named **internal** defaults/constants instead of scattering literal values. Then document which are intentionally non-public. This gives one place to audit the bounded-search policy without bloating the public API.

---

# 4. Plotting and example-control inconsistencies

## INC-014 — Example plot defaults and direct plotter defaults disagree

**Severity:** Medium
**Type:** Duplicate default source

`resolveAzElExampleOptions` defaults:

```matlab
Title       = "Azimuth/elevation motion plan"
FrameStride = 4
Pause_s     = 0.01
```

See [`resolveAzElExampleOptions.m` lines 43–55](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m#L43-L55).

`plotAzElMotion` defaults:

```matlab
Title       = "Az/El motion plan"
FrameStride = 10
Pause_s     = 0.001
```

See [`plotAzElMotion.m` lines 24–36](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plotAzElMotion.m#L24-L36).

This conflicts with `AGENTS.md`'s options rule:

> One defaults structure or local defaults function is the only source of truth. Do not repeat the same default in validation, examples, and plotting.

See [`AGENTS.md` lines 436–454](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L436-L454).

### Recommended fix

Create one canonical plot-default provider, e.g. an internal helper used by both:

```matlab
plotDefaults = azElInternal.defaultPlotOptions();
```

Scenario-specific titles can still override the common defaults.

---

## INC-015 — Display compatibility aliases are implemented twice

**Severity:** Medium
**Type:** Duplicated compatibility logic

Both:

- `resolveAzElExampleOptions`
- `plotAzElMotion`

map the same legacy names:

- `ShowKinematicPlot` -> `ShowKinematics`
- `AnimationFrameStride` -> `FrameStride`
- `AnimationPause_s` -> `Pause_s`

See:

- [`resolveAzElExampleOptions.m` lines 147–163](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m#L147-L163)
- [`plotAzElMotion.m` lines 284–311](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plotAzElMotion.m#L284-L311)

This directly conflicts with `AGENTS.md`:

> Centralize forwarding aliases instead of maintaining two implementations.

See [`AGENTS.md` lines 538–549](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L538-L549).

### Recommended fix

Move the mapping into one internal compatibility helper and call it from both boundaries.

---

## INC-016 — `displayOptions` mixes plotting controls with physical/configuration compatibility data

**Severity:** Medium
**Type:** Structure-responsibility mismatch

The helper says its second output is `displayOptions`, but it adds:

```matlab
JerkConstraintEnabled
MaxJerk_deg_s3
ConfiguredFiniteMaxJerk_deg_s3
PlotOptions
```

See [`resolveAzElExampleOptions.m` lines 141–145](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m#L141-L145).

`plotAzElMotion` then explicitly strips those fields as compatibility names:

[`plotAzElMotion.m` lines 305–310](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plotAzElMotion.m#L305-L310).

Examples reveal the ambiguity by calling the same second output either `displayOptions` or `jerkConfiguration`.

### Recommended fix

Separate responsibilities:

```matlab
[plannerOptions, plotOptions, exampleControls] = ...
```

or keep only plot fields in the display structure and pass physical limits separately.

---

## INC-017 — Plot handle schema returns singular and plural aliases for the same objects

**Severity:** Low
**Type:** Output-schema duplication

For the kinematic figure, the plotter assigns both:

```matlab
handles.KinematicFigure
handles.KinematicAxes
handles.KinematicsFigure
handles.KinematicsAxes
```

See [`plotAzElMotion.m` lines 200–243](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plotAzElMotion.m#L200-L243).

### Recommended fix

Choose one canonical pair. If the duplicates exist for compatibility, explicitly mark the older pair deprecated and remove it after the documented compatibility window.

---

# 5. Example-interface and result-metadata inconsistencies

## INC-018 — Example metadata does not have one common schema

**Severity:** Medium
**Type:** Example-result contract drift

`exampleAzElPlanning` returns common fields such as:

```matlab
ExampleName
ExampleMetrics
ExampleControls
ExampleGeometry
```

See [`exampleAzElPlanning.m` lines 64–72](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/exampleAzElPlanning.m#L64-L72).

`exampleFourAcceleratingCircles` instead uses:

```matlab
ExampleConfiguration
ExampleInputs
CircleMotionValidation
MovingTargetValidation
```

and does not establish the same common `ExampleName`/`ExampleControls` vocabulary.

See [`exampleFourAcceleratingCircles.m` lines 213–236](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/exampleFourAcceleratingCircles.m#L213-L236).

Scenario-specific metadata is fine, but the **common** metadata should be stable if examples are supposed to be uniform executable interfaces.

### Recommended fix

Require these common fields from every maintained example:

```matlab
result.ExampleName
result.ExampleControls
result.ExampleMetrics
result.ExampleValidation
```

Put scenario-specific material below one nested record, e.g. `result.ExampleData`.

---

## INC-019 — One maintained example breaks the documented one-override-struct signature pattern

**Severity:** Medium/Low
**Type:** Uniform-example interface drift

`AGENTS.md` prescribes:

```matlab
function result = exampleScenario(exampleOverrides)
```

and says every example should accept one optional scalar override structure.

See [`AGENTS.md` lines 143–195 and 212–224](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L143-L224).

Most maintained examples follow that shape. `exampleInterceptMovingTargetAtSetTime` instead declares:

```matlab
function result = exampleInterceptMovingTargetAtSetTime(interceptTime_s, options)
```

and supports four call forms.

See [`exampleInterceptMovingTargetAtSetTime.m` lines 1–19](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/exampleInterceptMovingTargetAtSetTime.m#L1-L19).

### Recommended fix

Keep one override struct and make intercept time a named example field, e.g.:

```matlab
exampleOverrides.InterceptTime_s = 12;
```

This makes the example callable by the same runner/harness as every other example.

---

# 6. Architecture and maintainability inconsistencies

## INC-020 — The implemented architecture misses its own preferred compactness targets

**Severity:** Medium
**Type:** Plan-versus-implementation drift

`plan.md` targets:

- 8–16 production files;
- `planAzElMotion.m`: 300–500 lines;
- topology seed generator: 400–700 lines;
- complete maintained repository: target <= 10,500 lines;
- hard production cap: 7,000 lines;
- no production MATLAB file over 900 lines without approval.

See [`plan.md` size limits](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plan.md#L33-L48) and [`plan.md` minimal architecture table](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plan.md#L228-L242).

The branch assessment reports:

- **26** production MATLAB files;
- **7,000** production physical lines — exactly the hard limit;
- **52** MATLAB files total;
- **11,653** physical lines total;
- preferred 10,500-line target does not pass.

See [`branch_assessment.md` lines 1–15](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/branch_assessment.md#L1-L15).

Current large files include approximately:

| File | Current lines | Plan target |
| --- | ---: | ---: |
| `planAzElMotion.m` | 883 | 300–500 |
| `+azElInternal/generateAzElTopologySeeds.m` | 900 | 400–700 |
| `+azElInternal/solveAzElHs3.m` | 886 | part of 800–1,400 HS3 total |
| `plotAzElMotion.m` | 499 | 500–800 |

### Interpretation

This is not a hard-limit violation. It is a documented failure of the branch's preferred/minimal architecture goals and leaves zero production-line margin for additional features.

### Recommended fix

Reduce and consolidate before adding features. In particular:

- shrink orchestration/local-helper burden in `planAzElMotion`;
- split or simplify the 900-line seed generator by **responsibility**, not one-function fragments;
- remove dead compatibility/API code such as the unused planner `RandomSeed`;
- centralize repeated option/alias/default code rather than adding helpers that duplicate policy.

---

## INC-021 — `generateAzElTopologySeeds` section numbering skips Section 5

**Severity:** Low
**Type:** Coding-style inconsistency

The file goes from:

```matlab
%% Section 4: Search The Time-Expanded Visibility Graph
```

to:

```matlab
%% Section 6: Local Functions
```

with no Section 5.

See [`generateAzElTopologySeeds.m` around lines 133–180](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/%2BazElInternal/generateAzElTopologySeeds.m#L133-L180).

`AGENTS.md` explicitly says to renumber sections when execution order changes:

[`AGENTS.md` lines 371–393](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L371-L393).

### Recommended fix

Rename the local-functions section to Section 5.

---

## INC-022 — Local helper headers do not consistently follow the mandatory `AGENTS.md` rule

**Severity:** Low/Medium
**Type:** Repository-style contract drift

`AGENTS.md` says every function, public **or local**, begins immediately after its declaration with:

```matlab
%% Section 0: Header & Readme
```

and explicitly states that short local helpers still receive the Section 0 contract.

See [`AGENTS.md` lines 303–370](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md#L303-L370).

Many local functions instead begin only with a one-line `% PURPOSE` comment. Examples include:

- `plannerDefaults` and other local functions in `planAzElMotion.m`;
- `normalizePlotAliases` in `plotAzElMotion.m`;
- local helpers in `validateAzElExampleResult.m`.

### Recommended fix

Decide whether the rule is still desired. If yes, enforce it consistently. If not, relax `AGENTS.md` to match the actual compact local-helper style. Given the repository's tight line-count constraints, relaxing the rule for short local helpers is probably the better choice.

---

# 7. Test-suite consistency gaps

## INC-023 — The example contract tests do not test the shared example-control contract

**Severity:** High/Medium
**Type:** Regression-coverage gap

`tests/testExampleContracts.m` currently protects:

- source hashes for three large examples and the U.S. helper;
- fixed-goal arrival policy;
- slalom elevation bounds;
- duration metric naming.

See [`testExampleContracts.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/tests/testExampleContracts.m).

It contains no checks for:

- `MaxJerk_deg_s3` propagation;
- common `Verbose` behavior;
- common `ExampleName` / `ExampleControls` / `ExampleValidation` metadata;
- shared display alias behavior;
- uniform example call signatures.

This is why INC-001, INC-004, INC-018, and INC-019 can coexist with all tests passing.

### Recommended fix

Add one parameterized `testUniformExampleControls` test that loops over all 18 maintained example functions in headless mode and verifies at least:

1. `example()` and `example([])` are valid;
2. `PlotOutputs=false` creates no figures;
3. a shared `MaxJerk_deg_s3` override reaches `result.Inputs.limits`;
4. all examples expose the common metadata fields;
5. the chosen common `Verbose` default is consistent;
6. compatibility aliases resolve through the one central alias map.

For the set-time intercept example, this test will force a decision about whether to keep its exceptional numeric argument or move that value into the override struct.

---

# 8. Additional non-uniformities worth cleaning up

These are lower-risk than the items above but still contribute to API drift.

## INC-024 — `ExampleControls` versus `ExampleConfiguration` vocabulary

Some examples call the shared resolved structure `displayOptions`, some call it `jerkConfiguration`, and result fields alternate between `ExampleControls` and `ExampleConfiguration`.

**Fix:** choose `ExampleControls` for common controls and reserve `ExampleData`/`ScenarioData` for scenario-specific information.

## INC-025 — Planner result `Inputs` casing differs from outer result-control casing

The outer result uses PascalCase control/status fields (`Success`, `Options`, `Validation`, `SearchDiagnostics`) while `Inputs` contains lower-camel payload names (`obstacles`, `initialState`, `goalState`, `limits`). This is defensible and mostly consistent with `AGENTS.md`, but it should be treated as an explicit convention rather than mixed ad hoc.

**Fix:** document this exact convention in the README/API section; do not rename fields solely for aesthetics.

## INC-026 — Compatibility fields are silently discarded by the plotter

`plotAzElMotion` removes `JerkConstraintEnabled`, `MaxJerk_deg_s3`, `ConfiguredFiniteMaxJerk_deg_s3`, and `PlotOptions` before normal option resolution.

See [`plotAzElMotion.m` lines 305–310](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plotAzElMotion.m#L305-L310).

Because these fields are produced by the example resolver itself, this is evidence that the example-control structure is serving too many roles.

**Fix:** do not pass a mixed compatibility/configuration structure to the plotter. Pass canonical plot options only.

---

# 9. Items I would *not* count as inconsistencies

The following are limitations, but they are explicitly documented and should not be mislabeled as accidental inconsistency:

- finite seed coverage is incomplete;
- the planner does not claim global optimality;
- homology signatures are 2-D sampled spatial classes, not full continuous Az/El/time homotopy classes;
- cooperative deadlines can overrun because active operations are not preempted;
- HS3 is a local nonlinear optimizer and may have numerical conditioning issues;
- deterministic first motion stops at geometric waypoints and is conservative;
- azimuth wrapping is restricted to obstacle-free fixed-position goals;
- dense reductions may remove a useful proposal but cannot directly validate an invalid trajectory.

These are stated in the README, `plan.md`, and/or `branch_assessment.md` and are therefore known design limitations rather than hidden contract drift.

---

# 10. Recommended repair order

## Priority 0 — Make the public contract mean one thing everywhere

1. Fix `MaxJerk_deg_s3` propagation in all 18 examples.
2. Move azimuth/elevation workspace bounds from `options` into `limits`.
3. Remove the unused planner `RandomSeed`; rename scenario randomness to `ScenarioRandomSeed`/`TargetPathRandomSeed`.
4. Standardize the example `Verbose` default.
5. Make HS3 verbose mode report throttled optimization progress.
6. Add tests for all of the above before changing other behavior.

## Priority 1 — Remove API/documentation ambiguity

7. Decide whether `planAzElMovingTargetIntercept` is a public adapter or whether there is literally one public entry point; update implementation/docs consistently.
8. Correct the README first-motion moving-goal support statement.
9. Rename `MaximumSearchDuration_s` to an intercept-horizon name.
10. Replace stale generic `[HS3]` labels with stage-correct terminology.
11. Standardize common example metadata.

## Priority 2 — Consolidate duplicated policy

12. Create one canonical plot-default source.
13. Create one centralized display-alias normalizer.
14. Stop mixing physical jerk configuration into `displayOptions`.
15. Centralize internal bounded-search constants.

## Priority 3 — Architecture/style cleanup

16. Reduce the 26-production-file / 7,000-line architecture before adding features.
17. Reduce `planAzElMotion.m` and `generateAzElTopologySeeds.m` toward their stated target ranges.
18. Fix section numbering.
19. Decide whether the strict local-function header rule in `AGENTS.md` should be enforced or relaxed.

---

# 11. Proposed canonical contract after cleanup

A cleaner end state would look approximately like this:

```matlab
limits = struct( ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90], ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);

options = planAzElMotion();
options.GoalTimeMode = "earliestArrival";
options.MaximumSeedCount = 5;
options.EnableHs3Improvement = true;
options.MaximumHs3ImprovementTime_s = 15;
options.Verbose = false;
```

Example calls would all use one shape:

```matlab
exampleOverrides = struct( ...
    "PlotOutputs", false, ...
    "FigureVisible", "off", ...
    "Verbose", true, ...
    "MaxJerk_deg_s3", [2.5 2.5]);

result = exampleScenario(exampleOverrides);
```

and every example would expose at least:

```matlab
result.ExampleName
result.ExampleControls
result.ExampleMetrics
result.ExampleValidation
```

The production planner itself would not expose `RandomSeed` unless production planning becomes randomized.

---

# 12. Audit references

Primary branch documents and code used for this audit:

- [`AGENTS.md`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/AGENTS.md)
- [`plan.md`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plan.md)
- [`README.md`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/README.md)
- [`branch_assessment.md`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/branch_assessment.md)
- [`planAzElMotion.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMotion.m)
- [`planAzElMovingTargetIntercept.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/planAzElMovingTargetIntercept.m)
- [`plotAzElMotion.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/plotAzElMotion.m)
- [`+azElInternal/generateAzElTopologySeeds.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/%2BazElInternal/generateAzElTopologySeeds.m)
- [`+azElInternal/solveAzElHs3.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/%2BazElInternal/solveAzElHs3.m)
- [`examples/resolveAzElExampleOptions.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/resolveAzElExampleOptions.m)
- [`examples/validateAzElExampleResult.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/examples/validateAzElExampleResult.m)
- [`tests/testExampleContracts.m`](https://github.com/kevvtrinh/AzElObsAvoid/blob/plan-325/tests/testExampleContracts.m)
- [all 18 maintained examples](https://github.com/kevvtrinh/AzElObsAvoid/tree/plan-325/examples)

## Bottom line

The branch is not internally chaotic, but the **interfaces and documentation have drifted faster than the underlying solver architecture**. The largest practical risk is that a user thinks a shared option means the same thing everywhere when it does not (`MaxJerk_deg_s3`, `Verbose`, `RandomSeed`, time-budget naming). Fixing those contracts first will make later solver/refactor work much safer and easier to test.
