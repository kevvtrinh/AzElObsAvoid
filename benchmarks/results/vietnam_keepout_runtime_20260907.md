# Vietnam keepout runtime investigation

The working planner now has a **25.8342754 s measured median**, versus
**198.1828721 s** for the frozen original: **86.9644% less runtime (7.6713x)**.
The final three repeats were **35.1461561, 25.8342754, and 23.2860196 s**.
The preliminary run was 39.468021 s. The median meets the requested 30 s target;
individual runs are not guaranteed to stay below 30 s.

## Baseline and scope

The exact saved input is `Rogue Examples/vietnam_keepout_slew_input.mat`, SHA256
`DBD475086DA1398E4735887B78D3A5466A6C22DEB67174FB889540DE392C7CAC`.
The reference is revision `5872d456d1fd6f31925b04a4491d3479ad4cb3e5` plus the
user's existing working changes, frozen before this experiment. The original
search file hash is `F310B0A32591DB87CC7327CA9F217DBB2E9CB9F972CD3ED0F67F11AAF4E1F667`.
All comparisons use MATLAB R2024b Update 4 on the same Windows machine, identical
inputs/options, and sequential MATLAB processes. No protected geometry,
interpolation model, tolerance, seed budget, time layer, or physical arrival time
is changed. In particular, the closed-ring issue from the
[input diagnosis](vietnam_keepout_diagnosis_20260907.md) is not normalized here.

Timings measure the complete two-output public planner call, including its own
validation and diagnostic assembly. A fresh independent public validation runs
after each timed call. MATLAB startup, loading the saved input, and that extra
independent validation are outside the timing. Each implementation receives a
preliminary run followed by three measured repetitions. Paths are isolated and
functions are cleared between implementations/repetitions. Cache contents are
rebuilt inside every search invocation; no answer is reused across planner calls.

## Measured progression

| Implementation | Measured repeats (s) | Median (s) |
|---|---|---:|
| Original working baseline | 194.158968, 198.182872, 202.462712 | 198.182872 |
| Midpoint-first short circuit | 78.847082, 82.108733, 87.248511 | 82.108733 |
| Reuse identical point/geometry occupancy | 51.388036, 51.424562, 55.476737 | 51.424562 |
| Populate stationary-interval occupancy in batches | 37.387071, 37.989650, 37.875174 | 37.875174 |
| Combined layer construction and exact endpoint reuse | 29.287439, 28.373739, 28.038419 | 28.373739 |
| Final source, including nonfinite-clock cache guard | 35.146156, 25.834275, 23.286020 | 25.834275 |

The final integrated gate passed and the implementation is applied to
`+obstacleAvoidance/+search/timeExpandedVisibilitySearch.m`. It constructs
candidate masks once per reached time layer and reuses previously checked
endpoint occupancy only when both coordinates and physical query time are
exactly equal. Nonfinite sampled clocks bypass caching. The sole production
change is this search implementation; regression coverage is in
`tests/testTimedVisibilityScreening.m`.

This is measured wall-clock improvement, not a real-time deadline guarantee.
Timing variation is visible in the recorded repetitions, including unfavorable
runs. The additional independent validator is outside the timed planner call.

## Exactness and memory

For every saved-input comparison, require `isequaln` on the route, polynomial,
motion histories, resolved inputs/options, success/termination, complete search
record excluding elapsed time, and independent validation excluding its two
runtime fields. The reference motion is:

- Success and fresh independent validation: true; termination: `goalReached`.
- Polyline length: 17.297991315813945 units.
- Sampled smoothed length: 17.305374620918947 units.
- Physical motion duration: 30 s.
- Collision-free, resolved continuous collision checks, and kinematic checks: pass.
- Minimum protected clearance: 0.000061818786833534034 units.
- Optional plane certificate: not accepted; adaptive continuous validation passes.
- Search: 56,571 accepted motion edges, 5,618 waits, 4,818,010 rejected transitions.

The cache stores only authoritative Boolean occupancy answers. Original obstacle
sample times have distinct keys from open history intervals. Any interval with
nonzero corresponding-vertex motion bypasses interval reuse. A 300 MiB private
cache allowance, authorized by the user, bounds stored lookup data; eviction and
memory-based bypass repeat the original query without changing search decisions.
This input's 111 nodes and 31 distinct obstacle times imply at most 10,090,899
bytes of occupancy payload before lookup arrays. Raising the cap alone was not
the speedup: larger batches and reuse removed repeated work.

## Controls and rejected experiments

Focused controls cover collision samples before/at/after the midpoint, an
all-clear moving case, authoritative sample geometry distinct from stationary
interval hulls, and floating-point cancellation in endpoint coordinates and
timestamps, plus a nonfinite-clock cache-contamination control. The unchanged
saved input remains a regression requiring planner
success and fresh public validation.

All 18 maintained examples were executed on the frozen original and the
integrated candidate, with complete result, search, and independent-validation
equality excluding runtime fields. They also matched the earlier short-circuit
comparison. All 17 successful examples remained successful; `exampleNoPath`
retained its expected `noValidatedSeed` failure. The default finite jerk modes
were used; their full metrics and actual runtimes are in `benchmark.csv` and
were reported in chat. Historical sentinel values that already differed in the
user's frozen working tree were not treated as changes caused by this work.

The affected suite passed **126/126 tests**, with zero failures or incomplete
tests. The final overflow control then passed on both original and final source,
bringing the distinct covered checks to **127**. All six small screening controls
were rerun on both implementations after the guard. The exact saved request was
then replayed four times on the final source, with full physical/search/validation
equality each time. MATLAB Code Analyzer reported zero issues in production and
the new test file. The promoted files match the tested isolated copies exactly.

Verification covers 15 existing test files plus the new regression file. The
unrelated convergence-summary/travel-refinement case was excluded; this is not a
claim that the entire repository test suite ran. User-deleted legacy MAT fixtures
were not recreated. The existing user changes and supplied MAT input were preserved.

Rejected isolated experiments are not production paths or current benchmark
entries: convex Boolean distance shortcuts took 77.3054 s; boundary-record
caching took 87.0715 s; scalar transition tables took 35.3382 s; per-node
vectorization had median 30.8418 s; full-layer construction alone took 34.4016 s.
These missed their declared incremental benefit gates. The per-node version
reached 29.1789 s once but did not establish consistently sub-30-second runtime.
The integrated endpoint version initially measured median 28.8381 s; review then
added the exact timestamp guard and reran correctness and timing checks.

The initial baseline logging harness saved the physical result before failing
to append its first timing row; its planner wall time is unavailable. Two
isolated baseline tests initially lacked copied sandbox dependencies; their
dependencies were restored and the focused reruns passed. A MATLAB launch
quoting error occurred before a cache experiment entered the planner. These are
harness/environment errors, not planner failures.

Raw evidence is under ignored `tmp/vietnam-runtime-20260907/`. The benchmark
append contains 96 measured accepted planner/example calls plus one actual
initial baseline call whose timing was lost by the logging harness. Rejected
experimental timings are retained in this short account rather than in the
current benchmark rows. The initial diagnosis replay is recorded separately.

## Reproduce the request

From the repository root in MATLAB:

```matlab
addpath(pwd, fullfile(pwd, 'trajectory'));
source = load(fullfile('Rogue Examples', 'vietnam_keepout_slew_input.mat'));
timer = tic;
[result, diagnosis] = obstacleAvoidance.planTrajectory(source.protectedObstacles, source.initialState, source.goalState, source.limits, source.options);
elapsed_s = toc(timer);
validation = obstacleAvoidance.validateTrajectory(result);
assert(result.Success && validation.Passed);
```
