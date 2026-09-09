# Reusable C3 profile library

The library stores normalized, independently validated quintic motions and compact
initialization clocks. BMTP transfers several compatible profiles to a new route,
compares their achievable timing against the current obstacles, and refines the
best proposal. Every returned motion must pass the existing continuous engine
certificate and public independent validator.

This implementation accelerates **static, rest-to-rest, earliest-arrival detours**.
Direct chords, fixed-arrival requests, moving geometry, non-rest endpoints, and
library misses use the ordinary solver. The code is independent of obstacle names
and example IDs; the demonstration bank has limited coverage of route shapes.

## Build and use

```matlab
addpath(pwd, fullfile(pwd,'trajectory'), fullfile(pwd,'examples'));
mkdir('output');
[library, trainingReport] = buildExampleC3ProfileLibrary('output/c3_profiles.mat');

options = struct('GoalTimeMode','earliestArrival','C3ProfileLibrary',library);
result = planner(obstacles, initialState, goalState, limits, options);
```

`C3ProfileLibrary` also accepts a MAT filename. For repeated calls, load it once
with `bmtpEngine.loadC3ProfileLibrary(filename)` and pass the struct. The public
planner still checks its data on each call. Leaving the option empty retains
ordinary planning. `planner(...)` remains the single public planning entry point.

Build a bank for your application from successful public planner results:

```matlab
library = bmtpEngine.buildC3ProfileLibrary({trainingResult1, trainingResult2}, ...
    'output/application_profiles.mat');
```

Every supplied result is independently revalidated. Invalid results raise an
error. The example builder records all eight training outcomes and retains only
valid motions. Six training cases succeeded in the recorded run, covering cavity,
L, and convex-block detours; two ordinary training solves failed and were excluded.
Generated MAT files and benchmark outputs stay outside source control.

## How lookup reduces computation

For endpoint distance `L`, chord frame `R`, and duration `T`, positions are stored
as `(q-q0)*R/L` and span durations as `h/T`. The bank contains the actual C3
polynomial, original phase clock, compact initialization clock, normalized route,
and a dimensionless velocity/acceleration/jerk signature. Export subdivision does
not change the original optimizer clock; older uniformly bisected outputs are
recognized too.

1. Lookup compares 17 arclength samples, matches route vertex counts, considers
   reflections, and screens large changes in route shape or derivative regime.
   It retrieves at most four compatible profiles.
2. The stored curve is transferred into the current chord frame and adapted to
   the current protected visibility route. The compact clock aggregates phases
   shorter than 1% of the stored duration. This interpolation is only a solver
   guess; the full stored polynomial remains C3.
3. Each proposal receives a fresh conic initialization against current obstacles
   and limits. The proposal with the shortest feasible initialized duration wins.
   Shape proximity alone does not choose the final profile.
4. Default `C3ProfileMode='repair'` uses up to 40 timing-refinement iterations on
   the smaller phase model. Exact redundant-plane removal and direct factorization
   keep this model better conditioned. The final conic solve generates the path
   using length and jerk-variation objectives.
5. The existing `PathLengthTimeAllowance_s` is shared between timing regularization
   and the final length/variation solve: half for each, with no additional time
   budget. Setting it to zero removes both allowances.

`C3ProfileMode='warmStart'` retains the ordinary phase grid and full 200-iteration
timing budget. It still uses the profile shortlist. It is available for cases
where a longer local optimization is useful.

The current-world axis limits are always enforced anew. Arbitrary rotation is not
assumed to preserve feasibility under anisotropic limits. Obstacle margins are
applied only by ordinary obstacle preparation. No filter is applied after motion
generation, and no old collision certificate is reused.

## Explicit quality caps

Profile assistance is a local heuristic, not a proof of global time optimality.
The recorded half-second arrival comparisons are empirical. When a reference
solution for the **same current request** is available, enforce the budget directly:

```matlab
options.C3ProfileMaxArrival_s = reference.ArrivalTime_s + 0.49;
options.C3ProfileMaxLength_units = reference.MotionLength_units * 1.005;
result = planner(obstacles, initialState, goalState, limits, options);
```

These caps screen the library proposal. A violation retries the ordinary solver;
they do not replace the mission horizon or constrain the fallback's objective.
Failed engine certification or public independent validation also triggers
recovery with the original geometry and tolerances. Computing a new reference
first costs time and must be included if it is part of the online workflow.

## Reproduce the benchmark

```matlab
addpath(fullfile(pwd,'benchmarks'));
[summary, trials, motions] = benchmarkC3ProfileLibrary( ...
    library, 3, 'output/c3_profile_benchmark');
```

The eight scenarios are separate from training and cover changed proportions,
start positions, anisotropic limits, 30-degree rotation, spatial scaling, and an
asymmetric cavity. Each has one warmup followed by three measured runs of each
planner on identical inputs. Wall time includes option/library checks, geometry,
lookup, conic trials, nonlinear refinement, final construction, validation, and
fallback. Arrival, length, and exact normalized jerk variation are reported
separately; failed solves remain in the results.

## Diagnostics and remaining limits

`result.SolverDiagnostics.ProfileLibrary` reports the shortlist, selected entry,
route/limit error, reflection, lookup time, every conic initialization outcome,
attempted/accepted status, proposal arrival/length, solver reason, and fallback
time. `Accepted` means the returned motion used the library; a match alone does
not. Misses and unsupported requests record ordinary-solver work as fallback time.
A native analytic/corridor solution can succeed without consuming a matched profile.

The search currently scans a small bank and requires matching route vertex counts.
A larger bank needs broader training coverage and indexed retrieval. Unrepresented
route families use the ordinary solver. The library does not yet accelerate
moving-obstacle event clocks or non-rest endpoint states. All physical validation
requirements remain unchanged for those requests.

## Recorded result (MATLAB R2024b, 2026-09-09)

Three measured runs per method per case, after warmup. All 24 measured library
runs passed independent validation and used a profile; none invoked the ordinary
solver fallback. The full regression suite passed all 137 tests.

| Case | Ordinary / library wall (s) | Speedup | Arrival change (s) | Path change | Jerk variation change |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 8.22 / 3.52 | 2.33x | +0.199 | -3.60% | -28.89% |
| 2 | 8.26 / 3.16 | 2.61x | +0.221 | -1.42% | -45.32% |
| 3 | 9.81 / 2.65 | 3.70x | +0.280 | -2.67% | -28.91% |
| 4 | 7.93 / 2.12 | 3.74x | +0.305 | -1.85% | -25.76% |
| 5 | 8.43 / 2.05 | 4.12x | +0.218 | -0.78% | -39.92% |
| 6 | 7.78 / 3.62 | 2.15x | +0.438 | -0.88% | -22.92% |
| 7 | 7.55 / 1.64 | 4.59x | +0.310 | -0.53% | -27.61% |
| 8 | 8.21 / 1.58 | 5.21x | +0.245 | -1.00% | -35.55% |

Cases 1-4 change proportions and initial position; case 5 changes per-axis limits;
case 6 rotates/translates the scene; case 7 increases spatial scale with unchanged
derivative limits; case 8 changes wall proportions asymmetrically.

The ready-to-use local bank is `output/c3_profiles.mat`. The complete trial data,
summary, and final plot are `output/c3_profile_benchmark.mat`, `.json`,
`_summary.csv`, and `.png`. These generated artifacts are ignored by Git.

Earlier approaches were rejected: transferring a whole curve rigidly often
needed fallback; timing-only reuse delayed arrival by about six seconds; nearest
shape selection alone could lengthen the path. Compact clocks, current-scene conic
selection, and bounded refinement were retained only after the held-out checks
above. These measurements do not guarantee the same gains on every new scene.

The final moving-US fallback check preserved arrival at 8.5 s and path length
40.298297432 units. It showed no acceleration: the two warmed runs averaged
23.21 s without a library and 23.62 s with one. This is
consistent with the stated static-detour scope and includes lookup/check overhead.
An explicit tight arrival cap correctly rejected a library proposal and recovered
the ordinary result; zero allowance and the graphical U example also passed.
