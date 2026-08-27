# Plan 325 verification

Current worktree evidence is summarized in
[Standalone Hermite-Simpson restoration — 2026-08-24](#standalone-hermite-simpson-restoration--2026-08-24);
the latest example evidence is in
[Extreme deforming U.S. and moving-sun example — 2026-08-24](#extreme-deforming-us-and-moving-sun-example--2026-08-24).
Earlier sections are retained as historical checkpoints.

## Fixed-arrival geometric lower-bound proof — 2026-08-26

- Environment: `HS3-planner` at
  `855a569+fixed-lower-bound-worktree`, MATLAB R2024b Update 4, AMD64 Family
  23 Model 113, no Parallel Computing Toolbox or worker pool.
- Baseline diagnosis: the 42-second four-circle history contained 421 slices.
  With plots disabled, wall time was 62.021637 seconds and planner time was
  60.0989 seconds. Stage timing was 34.9519 seconds corridor construction,
  21.6022 collision checking, 2.1221 motion solving, 0.4585 topology, 0.2055
  final validation, and 0.7587 unattributed.
- Profiler: 41 `polyshape.union` calls consumed 48.2364 inclusive seconds;
  `seedEnvelopeContainsObstacles` consumed 54.4786 seconds. The selected
  direct seed's HS3 call used only 0.4489 seconds.
- Retained changes: the example supplies obstacle samples only through its
  requested 22-second planning horizon while independently checking the full
  conceptual 42-second profile endpoints. Dynamic fixed-arrival seed order is
  shortest-geometric-first, and search stops after an independently validated
  motion reaches the Euclidean start-goal lower bound.
- Focused result: clipping history alone reduced wall time to 26.148349
  seconds. The complete change reached 5.722992 seconds in the focused run and
  5.634472 seconds in the final serial matrix. The result remained exactly 20
  degrees at exactly 22 seconds with collision, velocity, acceleration, jerk,
  dynamics, endpoint, and example shortest-route validation passing.
- Structural regression: a distinct far-moving-obstacle fixed-arrival test
  exposes multiple seeds, attempts one, reaches its computed geometric lower
  bound, and passes. The moving-barrier waiting regression also passes.
- Static analysis and tests: Code Analyzer reported zero findings in the
  changed files. The complete suite passed 106/106 in 69.471882 seconds.
- Examples: 18 fresh serial `PlotOutputs=false` processes completed in
  297.286362 seconds. Seventeen successes passed independent collision and
  kinematic certificates; the expected no-path case independently validated
  `noValidatedSeed`. Exact rows are in `benchmark.csv` under
  `855a569+fixed-lower-bound-worktree`.
- Graphics: the visible four-circle success produced two figures and retained
  the shortest-route certificate. The hidden expected failure produced two
  diagnostic figures and retained nine rejected edges.
- Unfavorable evidence: `exampleMovingCircleNoAzimuthWrap` independently
  validated at 8.707031 seconds, 0.0609997 seconds later than its preceding
  row. The changed ordering applies only to fixed arrival, so no causal
  improvement or regression claim is made for this timing-sensitive case.

## Fixed-arrival length-first candidate quality — 2026-08-26

- Baseline: clean `855a569`, `HS3-planner`, identical scenario inputs and
  deterministic focused controls.
- Retained rule: fixed-arrival candidates keep the requested terminal time and
  rank by independently validated sampled motion length, then integrated jerk
  and stable seed index. Earliest-arrival behavior is unchanged.
- Focused benefit: four accelerating circles shortened from 27.8702009821 to
  25.9348981999 degrees (6.943986%) at exactly 22 seconds.
- Structural fixed cases: alternating occlusion shortened 3.809973%; target
  exits obstacle shortened 4.772009%; the already-straight specified-time
  intercept remained unchanged at 9.53894054682 degrees and 12 seconds.
- Physical validity: every improved result retained independent collision,
  velocity, acceleration, jerk, dynamics, endpoint, and fixed-time checks.
- Center-line diagnosis: the four protected circles block elevation zero from
  7.3664844164 to 12.6335155836 seconds. Both early and late direct passages
  violate the time/dynamics bounds; the direct seed was attempted and returned
  `optimizerInfeasible`.
- Tests: the focused planner suite passes 54/54, including stable early-failure
  diagnostics and static multi-route shortest-selection coverage.
- Examples: all 18 maintained examples ran serially in fresh headless
  processes in 254.1345943 seconds; exact rows are in `benchmark.csv` under
  `855a569+fixed-length-worktree`.
- Graphics: the improved fixed-arrival four-circle result independently
  validated and created three visible figures.
- Runtime tradeoff: static fixed-arrival planning may now attempt every retained
  seed, bounded by `MaximumSeedCount` and `MaximumPlanningTime_s`, rather than
  stopping after the first validated seed.

## Flat architecture and frozen HS3 boundary — 2026-08-26

- Environment: `HS3-planner` at
  `ad3139c+flat-architecture-worktree`, MATLAB R2024b Update 4.
- Architecture: six flat Az/El packages plus frozen `+hs3`; no nested
  production package directories, duplicate production MATLAB basenames, or
  legacy `+azElInternal` / `+azElPlannerMethods` trees remain.
- Dependency boundary: Az/El seed solving delegates optimization,
  reconstruction, and evaluation to `hs3`; numerical optimizer calls occur
  only in `+hs3/optimize.m`; HS3 source remains domain-neutral.
- Static checks: Code Analyzer reported zero findings. `git diff --check`
  passed, and `git diff -- +hs3` was empty.
- Tests: 104/104 passed, zero failed or incomplete, in 52.560 seconds.
- Examples: all 18 maintained examples ran serially and headlessly in fresh
  processes in 241.822 seconds. Seventeen independently validated successes
  passed collision and kinematic certificates; the expected no-path example
  independently validated `noValidatedSeed`. Exact rows are in
  `benchmark.csv`.
- Graphics: a visible obstacle-free success created three figures. The
  expected no-path case created two hidden diagnostic figures and retained its
  search grid.
- Known weakness: moving-barrier and opening-U optimization emitted repeated
  near-singular or singular working-precision warnings. The returned motions
  independently validated, but the conditioning issue remains visible.

## Severe-static fixed-time quality search — 2026-08-26

- Environment: `HS3-planner` at `7661321+fixed-quality-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes while two
  user-owned MATLAB processes remained untouched.
- Profile: wide U spent 12.87 of 14.23 planner seconds in motion solving and
  11.98 seconds in `fmincon`; topology generation used 0.48 seconds. This
  localized the gap to the motion transcription rather than route discovery.
- Retained rule: on the first quality decision, a non-timed earliest-arrival
  static route with relative sampled-motion inflation above
  `2.5 / segmentCount` receives a maximum-mesh fixed-arrival feasibility solve.
  Existing timed bisection then shortens the horizon while preserving the
  original topology seed. Other route families retain their prior flow.
- Wide-U evidence: the final matrix reaches 22.6308876389 seconds with a
  34.9425880405-degree polyline, 41.5363500661-degree sampled motion,
  64 segments, one mesh pass, passing collision and kinematic certificates,
  and 17.688485 seconds wall. Arrival improves by 0.4075710153 seconds and wall
  by 1.429006 seconds versus the preceding worktree, leaving a
  0.7981520967-second arrival gap to 325.
- Negative controls: two opposing U remains exactly 21.9090824092 seconds;
  forty moving circles remains 61.2011842765 seconds; extreme U.S. remains
  6.3679977362 seconds; moving/deforming U.S. remains 8.75061035156 seconds;
  fixed-arrival and causal-timing examples remain valid.
- Rejected probes: global static fixed-time search slowed two opposing U and
  did not match the retained wide-U result. Interior-point-convex restored
  feasibility but introduced 80--87-second probes and a run exceeding
  12 minutes. Finer extreme-U.S. meshes regressed arrival, so none were kept.
- Rogue replays: `failure.mat` validates at 88.2939404925 seconds in
  7.219560 seconds wall; `successwhenincreasehorizon.mat` validates at
  88.2939359679 seconds in 7.831260 seconds wall. Both use 20 segments and one
  mesh pass; their arrival difference is 4.525 microseconds.
- Full tests: 82/82 passed, zero failed or incomplete, in 50.675781 seconds.
- Maintained examples: all 18 ran serially in fresh MATLAB processes. Exact
  rows are appended under `7661321+fixed-quality-worktree` in `benchmark.csv`;
  17 successes and the expected failure independently pass.
- Graphics: visible basic planning produced two figures. Expected no path
  produced two diagnostic figures and retained `noValidatedSeed`.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. Nine HS3 files contain exactly 2,000 nonblank, noncomment lines.
- 325 comparator: remaining arrival gaps are wide U +0.7981520967 seconds,
  forty moving circles +0.8393466878 seconds, and extreme U.S.
  +0.3610623598 seconds. Two opposing U is 0.2018852770 seconds earlier.
  Moving/deforming U.S. is 0.3892445513 seconds earlier but 38.0431317 seconds
  slower wall. No global optimality or uniform runtime claim is made.

## Derivative-slack continuation quality pass — 2026-08-26

- Environment: `HS3-planner` at `7661321+slack-quality-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes while two
  user-owned MATLAB processes remained untouched.
- Retained rule: after one valid same-mesh relinearization, an
  earliest-arrival non-timed spatial candidate with acceleration and jerk
  peaks each below 75% of their limits may receive one 2x mesh pass initialized
  from the validated motion. Length-inflation quality passes retain their
  original-seed initialization.
- Two-opposing-U evidence: two focused repeats and the final matrix reproduce
  21.9090824092-second arrival, 24.5077116377-degree polyline,
  24.4201122273-degree sampled motion, 20 segments, one mesh pass, passing
  collision and kinematic certificates. This improves the preceding worktree
  by 0.9660319268 seconds and beats the 325 row by 0.2018852770 seconds.
  Final wall is 13.972852 seconds, so the improvement carries an explicit
  9.3646144-second wall increase versus the preceding row.
- Negative controls: forty moving circles remains exactly at
  61.2011842765 seconds and 125.185941203 degrees; wide U remains
  23.0384586542 seconds; extreme U.S. remains 6.3679977362 seconds; moving
  circle remains 8.64603156476 seconds. Basic and alternating-slalom results
  also remain numerically unchanged.
- Rogue replays: `failure.mat` validates at 88.2939404925 seconds in
  7.295837 seconds wall; `successwhenincreasehorizon.mat` validates at
  88.2939359679 seconds in 7.919359 seconds wall. Both use 20 segments and one
  mesh pass, and their arrival difference is 4.525 microseconds.
- Full tests: 82/82 passed, zero failed or incomplete, in 50.245827 seconds.
  An earlier 81/82 invocation intentionally does not count because its harness
  suppressed the warning required by the sole failing warning-contract test.
- Maintained examples: all 18 ran serially in fresh MATLAB processes after the
  final ordering fix. Exact rows are appended under
  `7661321+slack-quality-worktree` in `benchmark.csv`; 17 successes and the
  expected failure independently pass.
- Graphics: visible basic planning produced three figures. Expected no path
  produced two diagnostic figures and retained `noValidatedSeed`.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. Nine HS3 files contain exactly 2,000 nonblank, noncomment lines.
- 325 comparator: remaining arrival gaps are wide U +1.2057231120 seconds,
  forty moving circles +0.8393466878 seconds, and extreme U.S.
  +0.3610623598 seconds. Moving/deforming U.S. is 0.3892445513 seconds earlier
  but measured 55.917228 seconds wall versus 18.2106663 seconds on 325. No
  global optimality or uniform runtime claim is made.

## Dynamic spatial quality pass — 2026-08-26

- Environment: `HS3-planner` at `7661321+dynamic-quality-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes.
- Forty-circle localization: the 10-, 20-, and 30-segment transcriptions use
  the identical 110.807922148-degree topology. They independently validate at
  64.5557730468, 61.2011842765, and 60.1588345587 seconds respectively.
  Starting at 20 and 30 segments costs 23.9281868 and 31.5052284 seconds wall.
- Retained bounded rule: after a valid earliest-arrival spatial candidate,
  sampled-motion inflation above one mesh interval permits one quality pass.
  Dynamic geometry uses 2x segments; static severe inflation retains the
  existing 3x choice. Fixed-arrival and timed-topology candidates are excluded.
- Final forty-circle evidence: 20 segments, 61.2011842765-second arrival,
  125.185941203-degree sampled motion, passing collision and kinematic
  certificates, and 18.6109941 seconds wall. This improves the immediately
  preceding worktree by 3.3545887703 seconds arrival for 2.1703734 seconds wall
  and leaves a 0.8393466878-second gap to the 325 row.
- Negative controls: moving circle remains at 10 segments, zero passes, and
  8.64603156476 seconds. Moving/deforming U.S. remains a timed topology at
  8.75061035156 seconds and zero mesh passes. Four accelerating circles retains
  fixed 22-second behavior. Three dense single-obstacle moving probes under
  timed-search suppression remained below the inflation threshold and did not
  refine.
- Tests: 82/82 passed, zero failed or incomplete, in 50.3745157 seconds.
- Maintained examples: all 18 ran serially in fresh MATLAB processes. Exact
  final rows are appended under `7661321+dynamic-quality-worktree` in
  `benchmark.csv`; 17 successes and the expected failure independently pass.
- Graphics: visible basic planning produced three figures and 526 objects.
  Expected no path produced two figures and 341 objects, 15 rejected
  transitions, and 9 retained rejected edges.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. `git diff --check` reports no whitespace errors beyond line-ending
  notices. Nine HS3 files contain 1,999 nonblank, noncomment lines.
- 325 comparator: remaining arrival gaps are wide U +1.205723112 seconds,
  forty moving circles +0.8393466878 seconds, two opposing U
  +0.7641466498 seconds, and extreme U.S. +0.36106235982 seconds.
  Moving/deforming U.S. is 0.3892445513 seconds earlier but 31.5077561 seconds
  slower wall. No global optimality or uniform runtime claim is made.

## Ordered-boundary and route-quality Pareto — 2026-08-26

- Environment: `HS3-planner` at
  `7661321+geometry-fastpath-worktree`, MATLAB R2024b Update 4,
  Optimization Toolbox 24.2, serial fresh processes.
- Geometry fast path: `shapeAtTime` now reports ordered-single-region,
  convexity, and outward-orientation evidence directly from canonical vertices.
  HS3 consumes that record without constructing a transient `polyshape`.
  Multi-region or degenerate geometry still constructs and queries the exact
  shape. `testShapeQueryReportsOrderedBoundaryProperties` compares the
  lightweight orientation against `isinterior` and checks convex, concave, and
  multi-region classifications.
- Forty-circle profile before the change: 22,920 `shapeAtTime` calls and
  19.270743 profiled seconds in `buildCorridor`, dominated by repeated
  `polyshape`, `area`, and `isinterior` calls. Final matrix evidence preserves
  every physical metric and reduces wall from 20.6046246 to 16.4406207 seconds.
- Static ordering: detour proposals are ranked by geometric length because
  candidate work is already separately bounded. Extreme U.S. now attempts the
  22.2394635087-degree route before the 22.3733117302-degree route and validates
  at 6.3679977362 seconds, 20 segments, and 24.6064786878 degrees of sampled
  motion. Final wall is 64.0234264 seconds.
- Bounded mesh rule: route inflation above one coarse mesh interval triggers
  one 2x pass; severe inflation above 2.5 intervals triggers one 3x pass.
  Wide U retains 30 segments and 23.0384586542 seconds. Extreme U uses 20
  segments. The neutral-circle regression expects 20 segments and the complete
  HS3 suite passes.
- Rogue replays: `failure.mat` independently validates on seed 2 at
  88.2939404925 seconds, 20 segments, 204.669079083-degree sampled motion,
  0.0061099373-degree clearance, and 6.9218305 seconds wall.
  `successwhenincreasehorizon.mat` validates on seed 2 at 88.2939359679
  seconds, 20 segments, 204.667439115-degree sampled motion,
  0.0061101734-degree clearance, and 7.5901807 seconds wall. Both collision and
  kinematic certificates pass; their arrival difference is 4.52 microseconds.
- Full tests: 82/82 passed, zero failed or incomplete, in 49.6429588 seconds.
  Focused HS3 plus obstacle-infrastructure tests passed 57/57 in
  44.1194061 seconds.
- Full examples: all 18 maintained examples ran serially in fresh processes.
  Seventeen independently validated successes and the expected validated
  failure are recorded under `7661321+geometry-fastpath-worktree` in
  `benchmark.csv` and were reported directly in chat.
- Graphics: visible basic planning produced three figures and 526 graphics
  objects. Expected no path produced two figures and 341 objects, including
  15 rejected transitions and 9 retained rejected edges.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. `git diff --check` reports no whitespace errors beyond line-ending
  notices. Nine HS3 files contain 1,998 nonblank, noncomment lines.
- 325 comparator: versus `da52da8+quintic-root-recovery-worktree` on
  `325-full-suite`, remaining arrival gaps are forty moving circles
  +4.1939354581 seconds, wide U +1.205723112 seconds, two opposing U
  +0.7641466498 seconds, and extreme U.S. +0.36106235982 seconds.
  Moving/deforming U.S. is 0.3892445513 seconds earlier but 31.1939185 seconds
  slower wall. No global optimality or uniform runtime claim is made.

## Static quality and time-expanded retiming — 2026-08-26

- Environment: `HS3-planner` at `7661321+timed-retiming-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes, plots and
  animation disabled for benchmark timing.
- Static rogue replay: `failure.mat` validates on seed 2 with 30 segments at
  87.1503426168 seconds, 200.592721765-degree sampled motion, and
  0.0027516996-degree clearance. `successwhenincreasehorizon.mat` validates on
  the same seed and mesh at 87.1503401418 seconds, 200.134547736-degree sampled
  motion, and 0.0028127474-degree clearance. The 180/360-second horizons differ
  by about 2.5 microseconds. `straightline.mat` and `skeptic.mat` remain valid
  at 57.5394882088 and 57.5394875671 seconds.
- Mesh evidence on the rogue topology: 20 segments validate at
  88.2939404925 seconds, 30 at 87.1503426168 seconds, and 40 at
  86.5467293065 seconds. The retained policy permits one 3x quality pass and
  therefore chooses the measured 30-segment runtime/quality point. The saved
  historical 40-segment 86.5088536619-second trajectory still passes the
  current continuous validator; the remaining local-quality gap is explicit.
- Structurally different regression:
  `testStaticLengthInflationTriggersOneQualityMeshPass` uses a neutral
  48-vertex circle, verifies the one-pass 30-segment mesh, and requires an
  arrival below 82 seconds. The earlier tall-detour horizon regression remains
  in the suite.
- Timed-topology repair: the moving/deforming U.S. example improves from
  30.1605224609 to 8.75061035156 seconds by testing one arc-length timing law
  on the same `timeExpandedVisibilityGraph` seed. Its final polyline is
  41.5785140688 degrees, sampled motion is 40.7424283094 degrees, and wall time
  is 52.9070181 seconds. Collision and all kinematic certificates pass.
- Direct-wait guard: arc-length retiming is ineligible for `directWait` seeds,
  because removing their repeated position destroys the causal law they
  encode. Final serial runs preserve moving barrier at 10.2314453125 seconds
  and opening-U at 11.8560791016 seconds; both select direct-wait seeds and pass
  independent validation.
- Full maintained matrix: 18/18 example outcomes match their contracts in
  separate serial MATLAB processes: 17 validated successes plus the validated
  expected no-path result. Exact per-example geometry, duration, certificates,
  termination, and wall time are appended to `benchmark.csv` under
  `7661321+timed-retiming-worktree` and were reported directly in chat.
- Test command: `runtests('tests')` with only MATLAB's singular-matrix warning
  IDs suppressed. Result: 81/81 passed, zero failed or incomplete, in
  52.4952677 seconds. A prior run with all warnings disabled produced one
  expected test-harness failure because the unknown-option warning contract
  was intentionally hidden; that run is not counted as code evidence.
- Static command: recursive `checkcode(..., '-id')` over all MATLAB sources.
  Result: zero messages across 84 files. The three modified HS3 solver files
  also passed a focused Code Analyzer run.
- Graphics: visible `exampleAzElPlanning` passed and produced three figures
  with 526 objects. Hidden `exampleNoPathAzElMotion` passed independent failure
  validation and produced two diagnostic figures with 15 rejected transitions
  and 9 retained rejected edges.
- Repository checks: `git diff --check` found no whitespace errors beyond
  line-ending notices. Nine HS3 MATLAB files contain exactly 2,000 nonblank,
  noncomment lines, which meets the hard cap with zero spare lines.
- Comparator: the requested branch baseline is the later serial matrix under
  `da52da8+quintic-root-recovery-worktree` on `325-full-suite`, not only the
  earlier commit state. Current moving/deforming U.S. arrival is
  0.3892445513 seconds earlier than that like-for-like row, while wall time is
  34.6963518 seconds slower. Unfavorable current arrival gaps remain visible
  for forty moving circles (+4.1939354581 seconds), extreme U.S. visibility
  (+2.21937577543 seconds), the wide U (+1.205723112 seconds), and two opposing
  U obstacles (+0.7641466498 seconds). No global optimality or uniform runtime
  claim is made.

## Timed-arrival and exhaustive-failure repair — 2026-08-26

- Rogue duration-role diagnosis: identical static geometry and limits differed
  only in the 180/360-second horizon. Topology geometry was identical, while
  seed 2's warm-start duration had changed from 93.4400819229 seconds to the
  74.4431617582-second independent-axis lower bound. Solving from the
  conservative duration succeeds and independently validates under both
  horizons. The retained repair separates reachability/pruning from solver
  initialization.
- Rejected alternatives: fixed-time QP probes at 86.5, 88.3, 90, 91, and
  91.02 seconds were optimizer-infeasible under the current affine corridor,
  even though the nonlinear formulation validates at 91.019 seconds; extending
  that QP to static spatial seeds would therefore lose feasible motions. A
  nonlinear 85-second warm start converged to the same 91.018899717-second
  local solution, while the 74.4431617582-second physical-bound start failed
  before optimization with a nonfinite initial constraint. Neither alternative
  improved arrival without unacceptable robustness loss, so neither was
  retained.
- Serial rogue replays after the repair:
  `failure.mat` passes on seed 2 at 91.0188996291 seconds, 186.880163846-degree
  polyline, 217.919941498-degree sampled motion length, and 5.544338 seconds
  wall; `successwhenincreasehorizon.mat` passes on seed 2 at
  91.0189002025 seconds, the same polyline, 217.830384396-degree sampled
  motion length, and 5.280859 seconds wall. Both collision and kinematic
  certificates pass. `straightline.mat` remains valid at 57.5394882088 seconds
  in 2.759510 seconds wall; `skeptic.mat` remains valid at 57.5394875671
  seconds in 3.419472 seconds wall.
- Focused regression:
  `testDetourWarmStartDoesNotDefineReachability` constructs a tall multi-axis
  detour, measures its physical lower bound and conservative warm duration,
  then verifies that an identical request with a horizon between those values
  retains the route and clamps only the warm start. It passes in 0.79775
  seconds.
- Fresh maintained controls ran serially and headlessly. `exampleAzElPlanning`
  passes at 7.57952069664 seconds in 4.706205 seconds wall;
  `exampleMovingCircleNoAzimuthWrap` passes at 8.64603156476 seconds in
  10.553267 seconds wall; `exampleNoPathAzElMotion` returns the expected,
  independently validated `noValidatedSeed` with zero HS3 attempts in
  1.150478 seconds wall. Exact rows were appended to `benchmark.csv`.
- Final verification: 79/79 repository tests passed in 51.045611 seconds;
  Code Analyzer reported zero messages across 84 MATLAB files; `git diff
  --check` reported no whitespace errors beyond line-ending notices.

- Environment: `HS3-planner` at `7661321+worktree`, MATLAB R2024b Update 4,
  Optimization Toolbox 24.2, no Parallel Computing Toolbox, figures and
  animation disabled for timing.
- Explicit comparator: `325-full-suite` at `67bc087`, whose maintained
  examples select `corridorQuintic` by default.
- Profiled no-path baseline: 55.6428 seconds planner total, including
  54.2559 seconds motion solving, 0.3598 seconds topology, and 0.1177 seconds
  final validation. Five `fmincon` calls across three meshes spent 32.3781
  seconds in augmented-matrix factorization. The retained exact exhaustive
  static certificate returns the same validated failure in 1.3769188 seconds
  headless with zero HS3 attempts.
- Timed repair: fixed-time feasibility bisection preserves absolute event
  timing. Opening-U reaches 11.8560791016 seconds in 12.9626115 seconds wall;
  moving barrier reaches 10.2314453125 seconds in 12.6557865 seconds wall.
  Both pass independent collision and kinematic validation.
- Arrival regression guard: moving circle retains 8.64603156476 seconds after
  rejecting a faster fixed-QP-only incumbent probe that arrived at
  8.70703125 seconds. Its final 10.7064233-second wall remains slower than the
  9.0140762-second pre-repair measurement.
- Earlier verification: full repository tests passed 78/78 in 44.1178 seconds. Visible
  basic planning created three figures and 527 objects. Expected no path
  created two diagnostic figures with 15 rejected transitions.
- Size: nine HS3 MATLAB files contain 1,927 nonblank, noncomment lines;
  `plan.m` contains 575 physical lines.
- Untested: the user stopped the fresh 18-example serial comparison matrix
  after the focused regressions were isolated. No global optimality,
  completeness, or uniform runtime claim is made.

## Evidence scope

- Branch: `plan-325`.
- Baseline commit for the prepared-obstacle experiment: `4f59472`.
- Verified state: the current uncommitted Plan 325 implementation.
- Runtime: MATLAB R2024b Update 4 with Optimization Toolbox.
- Date: 2026-08-20.
- Every maintained example used a finite jerk limit.

Each maintained example ran in its own MATLAB process. Runs were serial.
Headless controls disabled plots, animation, and pauses.

## Implemented contract changes

- Workspace bounds moved from planner options to
  `limits.azimuthInterval_deg` and `limits.elevationInterval_deg`.
- `MaximumPlanningTime_s` and the whole-planner deadline were removed.
- Required work uses deterministic seed, graph, iteration, evaluation,
  collocation, and refinement limits.
- Optional HS3 improvement retains its separate 15-second default limit.
- Stable verbose messages use the `[AzEl]` prefix and stage prefixes.
- Fixed-goal examples state earliest arrival. Moving-target examples state
  their target-time policy.
- The slalom elevation interval is `[-5 5]` degrees.
- All maintained examples route `MaxJerk_deg_s3` into physical limits.
- The plotter retains the `main` branch visual style.
- A planning-runtime non-regression gate now applies to later changes.
- Dynamic obstacle slices and interval data are prepared once per planning
  call. The data remains time-dependent and shape-dependent.

## Size checks

| Scope | Files | Physical lines | Limit | Result |
| --- | ---: | ---: | ---: | --- |
| Core production, without plotting | 27 | 6,559 | 7,000 hard limit | pass |
| Plotting | 1 | 499 | separate report | pass |
| Production MATLAB | 28 | 7,058 | 7,000 target plus allowance | pass |
| Complete MATLAB tree | 54 | 11,769 | 12,000 hard limit | pass by 231 |
| Complete MATLAB tree | 54 | 11,769 | 10,500 target | fail by 1,269 |

No production MATLAB file is longer than 900 lines. The preferred complete
tree target does not pass.

## Final headless example results

`P/V` means planner success and independent example-validation pass. `C/K`
means collision and kinematic certificate pass. `NaN` means unavailable after
an expected failure. Fixed target durations are required target times, not
minimum-time results.

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635 | 16.758281983 | 12.180917402 | 1/1 | 30.2005183 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519 | 11.303432110 | 7.817268021 | 1/1 | 20.0718797 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721560 | 12.807761070 | 8.817608547 | 1/1 | 36.8415733 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 122.962176120 | 64.556766026 | 1/1 | 22.4426104 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed target | 1/1 | 24.363303007 | 27.712518684 | 22 | 1/1 | 56.3132192 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed target | 1/1 | 9.538940547 | 9.538940547 | 12 | 1/1 | 3.3125197 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest target | 1/1 | 10.097524449 | 7.342215833 | 6.275807672 | 1/1 | 4.8222865 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.139859112 | 10.544227894 | 1/1 | 29.9058765 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593185 | 12.113593185 | 12.293137410 | 1/1 | 16.7853734 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805147 | 71.508173805 | 12.986426213 | 1/1 | 36.3672785 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 15.5711128 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955 | 4.472860956 | 4.613406127 | 1/1 | 6.2329199 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1 | 16.7397940 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed target | 1/1 | 13.341664064 | 19.229413228 | 20.869565217 | 1/1 | 23.9341472 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed target | 1/1 | 19.824386759 | 22.879930804 | 24 | 1/1 | 12.5645525 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720884 | 24.302835532 | 22.876124561 | 1/1 | 66.0249043 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040 | 42.580115766 | 26.492875600 | 1/1 | 40.7620833 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463509 | 25.132264157 | 8.902682125 | 1/1 | 132.5539808 | `goalReached` |

The wide single-U request is unchanged. Its validated arrival duration is
26.493 seconds, which is below the requested 38-second threshold.

## Sparse visibility-graph experiment

The graph now tests deterministic Delaunay pairs plus all start and goal
connections. It does not change collision validation. The 40-circle case
tested 62 of 153 possible pairs and retained all 28 visible edges, the same
route, and the same arrival time. The wide U tested 55 of 120 possible pairs
and retained both homology classes and the same 26.493-second arrival. The
pair reduction is 59.5% and 54.2%, respectively. No wall-time gain was
confirmed, so this is a graph-work and memory improvement only.

## Runtime non-regression check

Prepared dynamic obstacle data passed the two required runtime gates before the
complete example run. The final serial 40-circle run decreased from
23.0063697 to 22.4426104 seconds, which is a 2.45% decrease. The final serial
moving-U.S. run decreased from 86.5107311 to 36.3672785 seconds, which is a
57.96% decrease. Both runs kept the same arrival duration within numerical
solver variation and passed independent validation. The prepared data stores
each source slice and each interpolation interval. It does not use one static
shape for a complete history.

The shared-jerk correction preserves the old `[2 2]` defaults. Seven changed
examples stayed within 2.7% of their recorded runs. The basic example received
an explicit A/B check:

- old source: 18.842, 18.979, and 19.518 seconds;
- corrected source: 19.090, 19.239, and 19.279 seconds.

The ranges overlap. The 1.37% median difference is inside observed process
noise. The returned path and duration were identical. No confirmed runtime
increase was accepted.

## Display and verbose checks

- Visible success: obstacle-free planning passed and created three figures.
- Visible failure: the no-path example passed failure validation and created
  two diagnostic figures with reason `noValidatedSeed`.
- Verbose success: the obstacle-free example printed setup, seed generation,
  first motion, ten-iteration HS3 updates, selection, and completion lines.
- Quiet runs produced no planner progress text.

## Automated checks

- Full tests: 56 passed, 0 failed, and 0 incomplete.
- Full test process time: 34.0101058 seconds.
- Code Analyzer: 54 MATLAB files and 0 messages.
- `git diff --check`: passed.
- MATLAB source lines longer than 100 characters: 0.
- Focused jerk-contract tests: 4 passed, 0 failed.

## Superseded audit artifacts and cleanup decisions

The one-time `repo_inconsistencies_plan_325.md` and
`repo_cleanup_audit_plan_325.md` reports described old commit `b845880` and
were removed after their resolved decisions were preserved here and in
`branch_assessment.md`. Workspace ownership, verbose behavior, timeout removal,
production size, jerk routing, prepared obstacle reuse, and package ownership
now reflect the maintained implementation rather than that historical audit.

`certifySeedCorridor` remains because the production validator calls it.
`RandomSeed` remains for public compatibility because removing it would break
the result schema. Polynomial sampling remains a measurement-first cleanup
candidate. The repository-owned `repository-cleanup` skill remains under the
parent workspace guidance directory rather than inside this project tree.

## Known limits and claim

- Spatial and timed proposals use finite samples and can miss a feasible
  topology.
- Reduced seed geometry can reject a useful proposal. Final validation uses
  the original protected obstacle history.
- The analytic motion stops at geometric waypoints. HS3 is local and can
  return a poor local result.
- Solver matrix-conditioning warnings can occur. Accepted motions still pass
  independent validation.
- Periodic obstacle images are not implemented.
- Optimization Toolbox is required for HS3.

A successful result is an independently validated motion from a finite,
deterministic proposal set. The planner does not claim global route
completeness or global time optimality.

## Adaptive Early-HS3 Verification — 2026-08-20

The planner now constructs the analytic fallback without immediately running
its continuous certificate for eligible spatial visibility seeds. It first
tries a denser HS3 motion and validates that motion independently. If HS3
fails, it validates the unchanged analytic fallback. Timed and wait seeds keep
the existing causal workflow. Reduced-geometry seeds do not receive a second
clearance expansion.

| Example | Prior duration (s) | New duration (s) | Prior wall (s) | New wall (s) | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| Wide single U | 26.492875600 | 22.828232905 | 40.7620833 | 16.8495066 | pass |
| 40 moving circles | 64.556766026 | 64.556780044 | 22.4426104 | 9.0772133 | equivalent within 0.001 s |
| Moving and deforming U.S. | 12.986426213 | 12.987386290 | 36.3672785 | 27.4422293 | equivalent within 0.001 s |

All 18 maintained examples ran serially in separate MATLAB processes. There
were 17 validated successes and one expected validated no-path result. The
visible success check created three figures. The visible no-path check created
two diagnostic figures and reported two rejected transitions. The final test
run passed 56 tests with no failures or incomplete tests. Code Analyzer checked
54 MATLAB files and returned zero messages. No MATLAB line exceeds 100
characters.

The complete MATLAB tree passes its 12,000-line hard limit. Production has
7,058 lines. The starting commit contained the same production line count
after prepared dynamic-obstacle data was added.

The performance allowance uses the declared wide-U, 40-circle, and moving-U.S.
benchmark set. A 58-line overage requires a 17.4 percent reduction because
`0.30 * 58 / 100 = 0.174`. Their wall-time reductions are 58.66, 59.55, and
24.54 percent. The minimum is 24.54 percent, so the allowance passes. The
wide-U arrival improves by 13.83 percent. The other two arrivals remain within
the configured 0.001-second equivalence tolerance. All three motions pass
independent collision and kinematic validation.

## Plan 325 continuation verification — 2026-08-21

This section supersedes earlier final-state counts and timing tables above. It
applies to commit `a023f1c` plus the current uncommitted optimized worktree.
Historical measurements remain visible for comparison.

### Accepted behavior

- Exact multi-obstacle `visibilityGraph` seeds can attempt HS3 before the
  analytic stop-at-waypoint fallback is accepted. The decision is based on
  obstacle count and seed provenance, not scenario geometry or route names.
- Early and later HS3 attempts share `MaximumHs3ImprovementTime_s`. Later work
  skips seeds already attempted early and uses only the remaining budget.
- Batched polynomial evaluation replaces repeated per-sample helper calls.
- Bernstein conversion accepts multiple columns. HS3 continuous-bound
  conversion is batched by segment and axis and then restored to the exact
  legacy inequality order.
- Seed and seed-summary templates are owned by the existing stable-result
  constructor so `planAzElMotion.m` remains below 900 physical lines.

The analytic motion remains a recoverable fallback and uses the same public
independent validation. No constraint tolerance, collision margin, obstacle
geometry, or iteration limit was weakened.

### Exact-equivalence checks

- Batched polynomial evaluation matched the scalar calculation bit for bit for
  uniform and nonuniform segment durations.
- Matrix Bernstein conversion matched a scalar column loop with maximum error
  zero.
- Complete continuous-bound vectors matched the prior segment/axis loop bit
  for bit with azimuth wrapping disabled and enabled.

### Declared runtime and size proof

Maintained production has 28 files and 7,139 physical lines. The 139-line
overage requires `0.30 * 139 / 100 = 0.417`, or a 41.7 percent wall-time
reduction. Before final A/B measurement, the declared representative set was
the extreme outline, dense concave, and U-shaped time-space examples.

Both sides ran serially in separate headless MATLAB processes. The baseline
was clean `a023f1c`; `Verbose=false` was supplied to both sides because the
baseline example resolver otherwise leaves that accessed field unset.

| Example | Baseline success/validation | Candidate success/validation | Baseline wall (s) | Candidate wall (s) | Reduction | Baseline arrival (s) | Candidate arrival (s) |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `exampleUSOutlineExtremeVisibility` | 1/1 | 1/1 | 83.8056819 | 48.2212733 | 42.46% | 6.684968340018 | 6.684968340018 |
| `exampleDenseConcaveAzElMotion` | 1/1 | 1/1 | 43.6252843 | 16.8686791 | 61.34% | 8.817608547166 | 8.817608547166 |
| `exampleUShapedAzElTimeSpace` | 1/1 | 1/1 | 89.9305427 | 17.1115690 | 80.97% | 38.549593103900 | 22.819550649779 |

The minimum measured reduction is 42.46 percent, so the production allowance
passes by 0.76 percentage points. The narrow margin is recorded explicitly.

### Final serial headless examples

Every row used finite jerk (`JerkConstrained = 1`). `P/V` is planner success
and independent example-validation pass. `C/K` is collision and applicable
kinematic-certificate pass. The no-path row uses `NaN` for unavailable motion
metrics and reports its stable termination reason.

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635036 | 16.260374075985 | 12.181917401593 | 1/1 | 20.9589414 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519024 | 11.303432110211 | 7.817268020971 | 1/1 | 11.6422250 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721559528 | 12.807761070221 | 8.817608547166 | 1/1 | 16.6448439 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685255 | 122.955558287524 | 64.556780043561 | 1/1 | 7.9373138 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.363303007331 | 27.712518684341 | 22 | 1/1 | 41.7615641 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.538940546821 | 9.538940546821 | 12 | 1/1 | 3.5031323 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 10.097524449091 | 7.342215833181 | 6.275807672232 | 1/1 | 4.7489639 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.211853153746 | 10.545227890750 | 1/1 | 26.3832859 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593184851 | 12.113593184851 | 12.293137410146 | 1/1 | 16.9825396 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805146555 | 69.402637353295 | 12.987386289935 | 1/1 | 28.8491449 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 10.8599145 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955000 | 4.472860955593 | 4.613406126529 | 1/1 | 5.9277416 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1 | 16.9304397 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 13.341664064126 | 19.229413227596 | 20.869565217391 | 1/1 | 22.9307595 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 19.824386758954 | 22.879930804015 | 24 | 1/1 | 8.3465627 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720883753 | 24.302835531542 | 22.876124561206 | 1/1 | 34.8855048 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040466 | 42.753271369061 | 22.819550649779 | 1/1 | 16.8144250 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463508699 | 26.617094587006 | 6.684968340018 | 1/1 | 47.9449594 | `goalReached` |

### Interactive and graphics checks

- A two-polygon interactive case previously accepted an analytic
  boundary-hugging route at about 20.718 seconds. The accepted multi-obstacle
  early-HS3 source selected a wider smooth route at about 8.859 seconds and
  passed independent validation.
- Default visible obstacle-free success passed at 4.613406126529 seconds,
  created four figures, and produced valid workspace, kinematic, and animation
  graphics. Wall time was 14.8765553 seconds.
- Visible expected no-path planning passed failure validation, created two
  figures, preserved two rejected transitions, and reported
  `noValidatedSeed`. Wall time was 14.6133705 seconds.

### Rejected experiments

- Returning after the first early multi-obstacle HS3 result regressed the
  two-U arrival to 23.9675706 seconds. It was removed; unattempted topologies
  now share the remaining HS3 budget.
- Applying continuation to broader reduced-obstacle cases increased the
  40-circle wall time to 24.217 seconds. Continuation was restricted to
  multiple exact obstacles. The final 40-circle result is 7.9373138 seconds.
- Template consolidation initially produced 14 focused-test errors because a
  stale local constructor call remained. The defect was corrected before any
  runtime result was accepted.

### Final automated and size checks

- Focused planner tests after recovery: 43 passed, 0 failed, 0 incomplete.
- Full tests after the final internal-interface cleanup: 56 passed, 0 failed,
  0 incomplete in 38.9511717 seconds.
- Code Analyzer: 55 MATLAB files, 0 messages.
- Production MATLAB: 28 files, 7,139 physical lines; allowance passes.
- Longest production files: `solveAzElHs3.m` 900 lines and
  `planAzElMotion.m` 888 lines.
- Maintained tracked MATLAB tree: 54 files and 11,873 physical lines; the
  12,000-line hard cap passes by 127 lines.
- Untracked interactive sandbox: 694 lines; excluded from maintained counts
  and not suitable to add without cleanup.

The current planner remains bounded and local. These results establish measured
improvement and independent feasibility for the exercised scenario families;
they do not establish global time optimality or complete reachability.

## Consolidated Plan 325 worktree verification — 2026-08-21

This section consolidates superseded worktree tables. Every executed historical
row remains in `benchmark.csv`, and chronological proof/recovery checkpoints
remain in `plan.md`. The current complete table follows in the next section.

### Accepted measured changes before the final checkpoint

- Exact multi-obstacle visibility seeds gained an early HS3 opportunity under
  one shared bounded budget. The manually drawn two-polygon case changed from
  about 20.718 to 8.859 seconds and selected a wider smooth motion. Opposing-U
  retained its independently valid route during that gate.
- Loop-free polynomial record evaluation was bit exact and reduced its isolated
  benchmark from 0.326166 to 0.103552 seconds over 15,000 repetitions.
- Batched polynomial reconstruction preserved coefficient and terminal-state
  bits for 1, 2, 7, and 19 segments.
- Lazy requested-output evaluation preserved two- through five-output calls bit
  for bit and reduced the position-only helper path by 54.84 percent.
- Batched seed-corridor conversion preserved the complete inequality vector bit
  for bit and reduced its isolated helper time by 78.22 percent.
- Frozen corridor times are now computed once per HS3 setup; the profiled
  extreme case had recomputed the invariant 34,203 times.
- Fixed-arrival speed-aware initialization and CG reduced the alternating-
  occlusion motion from 19.229413227596 to 15.324880519000 degrees and reduced
  the four-accelerating-circle wall from 43.0900 to about 29.2 seconds.
- Geometry-conditioned CG improved dense-concave arrival from
  8.817608547166 to 8.798638844754 seconds while exact multi-obstacle and timed
  families retained factorization.

### Removed experiments

- Stopping after the first early exact multi-obstacle result regressed opposing-
  U arrival from 22.8761246 to 23.9675706 seconds.
- Broad continuation increased the 40-circle wall to 24.217 seconds.
- Static-corridor vectorization improved dense locally but repeatedly regressed
  extreme serial pairs; cached Bernstein matrices also regressed dense.
- SQP exceeded a 60-second basic-case proof window. Global CG regressed opposing
  U shapes from 30.74 to 38.35 seconds, and timed CG regressed moving barrier.
- Earliest-arrival average-speed initialization regressed dense arrival to
  8.9027893 seconds. Five-percent seed expansion regressed wide-U arrival to
  24.3270 seconds.
- All removed forms were restored before subsequent measurements.

The user-approved production target is 7,500 physical lines. The separate
900-line production-file and 12,000-line tracked-MATLAB limits remain in force.

## Constraint-feasibility recovery follow-up — 2026-08-21

This section supersedes the preceding geometry-conditioned table and counts.

Earliest-arrival HS3 now retains a constraint-feasible primary minimum-time
solution instead of always running a second nonlinear solve that may trade up
to `ArrivalTimeTolerance_s` of arrival for lower integrated jerk. The second
solve remains available only when the primary nonlinear residual exceeds
`ConstraintTolerance`; it is therefore a feasibility recovery, not a routine
arrival relaxation. Fixed-arrival HS3 is unchanged because its primary
objective is already integrated squared jerk.

### Bounded experiment outcome

- Setting `ArrivalTimeTolerance_s` to `1e-5` recovered 0.99 milliseconds on
  basic, dense, and opposing-U examples, but the opposing-U wall increased to
  34.3097261 seconds. The default was not changed.
- Removing the second solve globally improved basic, dense, opposing-U,
  wide-U, and extreme runtime, but alternating slalom returned
  `noValidatedSeed` because its primary equality residual exceeded tolerance.
  That broad form was removed.
- The accepted condition runs recovery only when the primary residual exceeds
  `ConstraintTolerance`. Alternating slalom returns validated success, while
  already feasible primaries avoid the second solve.
- Limited-memory BFGS increased dense wall from the 13.09-second reference to
  13.5923721 seconds. PCG tolerances `0.01` and `0.2` took 13.2316361 and
  13.1500434 seconds without changing outer iteration counts. Both forms were
  removed.
- Scaling final-time `TypicalX` from the feasible guess produced only a 0.35
  percent extreme serial-pair improvement (40.4166606 to 40.2756680 seconds),
  below the proof threshold. It was removed.
- Nargout-sized allocation in the evaluator preserved every requested output
  but took 1.1225357 seconds versus 1.0924350 seconds for 20,000 position-only
  calls. It was removed.
- Accepted constraint-array reuse removed a duplicate selected-decision
  callback. Feasible basic and recovery-dependent alternating results remained
  bit for bit, 43 focused tests passed, and the solver decreased to 883 lines.
- Repository cleanup consolidated 506 superseded verification lines. Complete
  historical rows remain in `benchmark.csv`; checkpoints remain in `plan.md`.

### Final serial headless examples

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635036 | 16.758281982866 | 12.180917402175 | 1/1 | 13.5232923 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519024 | 11.411616815005 | 7.816267856881 | 1/1 | 8.2728629 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721559528 | 13.431299536656 | 8.797638855700 | 1/1 | 12.3737232 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685255 | 126.114009817632 | 64.555779916429 | 1/1 | 4.4305056 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.363303011158 | 27.712517413842 | 22 | 1/1 | 29.0747122 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.538940546821 | 9.538940546821 | 12 | 1/1 | 3.4576386 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 10.097524449091 | 7.342498781519 | 6.274806792200 | 1/1 | 3.2941048 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.134483812277 | 10.544227895142 | 1/1 | 25.3593433 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593184851 | 12.113593184851 | 12.293137410146 | 1/1 | 17.2145099 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805146555 | 71.436692325459 | 12.986386910606 | 1/1 | 27.7708022 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 11.1774461 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955000 | 4.484905564719 | 4.612405963436 | 1/1 | 3.3335129 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1 | 16.3750253 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 13.341664064126 | 15.324880518989 | 20.869565217391 | 1/1 | 22.8903990 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 19.824386758954 | 22.879843740594 | 24 | 1/1 | 6.9458763 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720883753 | 24.370904895056 | 22.875124576026 | 1/1 | 21.4478266 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040466 | 43.259235381251 | 22.818548735851 | 1/1 | 7.7077780 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463508699 | 26.617251754385 | 6.683971648809 | 1/1 | 36.7603481 | `goalReached` |

Every successful row passed collision and applicable kinematic certificates.
The no-path row retained its independently validated failure schema and two
rejected transitions. Earliest selected HS3 solutions are about one
millisecond earlier than the preceding jerk-relaxed results. Some are wider:
dense motion increases 4.86 percent, 40 circles 2.57 percent, wide U 1.18
percent, and opposing U shapes 0.28 percent. Those tradeoffs are visible and
retain hard jerk limits.

### Final checks and size

- Focused HS3 tests: 43 passed in 30.8298656 seconds; solver Code Analyzer
  messages: 0.
- Full tests: 56 passed, 0 failed, and 0 incomplete in 33.8326055 seconds.
- Code Analyzer: 55 MATLAB files, 0 messages.
- Visible success: three figures and 490 graphics objects with independently
  valid 4.612405963436-second duration.
- Visible failure: two diagnostic figures, two rejected transitions, and an
  independently valid `noValidatedSeed` result.
- Production: 28 files and 7,123 physical lines, 377 below the user-approved
  7,500-line target.
- Maintained tracked MATLAB tree: 54 files and 11,857 physical lines, 143
  below the 12,000-line hard cap.
- Core files: `solveAzElHs3.m` is 883 lines and `planAzElMotion.m` is 888.
- The 694-line interactive sandbox remains untracked.

The interactive two-polygon sandbox remains in the exact multi-obstacle early-
HS3 family. Its accepted wider-arc eligibility is unchanged, but the geometry
was not manually redrawn during this headless gate.

No global optimality, complete reachability, or machine-independent runtime
claim is made.

## Affine fixed-time constraint verification

Fixed-arrival HS3 constraints are affine in jerk because final time and every
obstacle query time are fixed. The final source evaluates that affine basis
once and passes it through fmincon's linear-constraint interface; earliest-
arrival requests retain the nonlinear time-decision callback. Diagnostics
report `linearFixedTime` or `nonlinearTimeDecision`, and focused tests exercise
both representations.

A seeded independent 30-variable affine system with 41 inequalities and six
equalities reproduced 100 random decisions with maximum residual error
7.11e-15, including the conversion from raw `c(x) <= 0` values to fmincon
matrix bounds.

| Fixed-arrival example | Previous wall (s) | Final wall (s) | Previous/final motion (deg) |
| --- | ---: | ---: | ---: |
| `exampleFourAcceleratingCircles` | 29.0747122 | 25.2320069 | 27.712517413842 / 20.372411016257 |
| `exampleInterceptMovingTargetAtSetTime` | 3.4576386 | 2.9311400 | 9.538940546821 / 9.538940546821 |
| `exampleStraightTargetAlternatingOcclusion` | 22.8903990 | 20.9311553 | 15.324880518989 / 14.220153980999 |
| `exampleTargetExitsObstacle` | 6.9458763 | 4.5878764 | 22.879843740594 / 22.879867003467 |

The target-exit change is 0.0000233 degrees; its integrated squared jerk
decreased from 0.146912667008831 on pushed checkpoint `2074c14` to
0.146912596533341, and its maximum solver violation decreased from
1.82e-14 to 2.00e-15. The accelerating-circle and alternating-occlusion cases
selected different, independently validated motions. Fixed arrival time and
all hard certificates were preserved.

All 18 maintained examples then ran serially and headlessly. Seventeen were
independently validated successes; `exampleNoPathAzElMotion` retained the
independently validated `noValidatedSeed` failure. Every earliest-arrival
metric was bit exact to the preceding feasibility-recovery sweep. The full
rows are appended to `benchmark.csv` under source tag
`2074c14+linear-fixed-constraints-worktree`.

The sweep initially exposed a pre-planning example-contract failure:
`exampleMovingDeformingUSOutlineVisibility` read `Verbose` from a partial
planner-options structure. The shared example resolver now materializes the
single public planner default structure before applying scenario and user
overrides. A dedicated default/override test passes, the original headless
case passes, and the structurally different extreme-outline case also passes.

- Full tests: 57 passed, 0 failed, and 0 incomplete in 29.1130938 seconds.
- Code Analyzer: 55 maintained MATLAB files, 0 messages.
- Visible success: four figures and 643 graphics objects.
- Visible failure: two figures, 341 graphics objects, and two rejected
  transitions with `noValidatedSeed`.
- Production: 29 files and 7,185 physical lines, 315 below the approved 7,500.
- Maintained MATLAB tree: 55 files and 11,940 physical lines, 60 below the
  12,000 hard cap.
- Core files: `solveAzElHs3.m` is exactly 900 lines and `planAzElMotion.m` is
  888 lines. The 694-line interactive sandbox remains untracked.

The interior-point feasibility-mode recovery experiment was removed after it
retained the same eight iterations and 326 evaluations while slightly
increasing alternating-slalom wall time.

## Exact fixed-time objective gradient

After fixed-time constraints became linear, fmincon still estimated the exact
quadratic jerk-objective gradient by repeated objective calls. The retained
helper returns the closed-form gradient for fixed-arrival solves. A seeded
30-variable central-difference proof measured maximum absolute error
2.58e-9 and maximum relative error 5.33e-10. The set-time example's reported
objective function count decreased from 1,032 to 24.

The complete 18-example serial headless gate was repeated on the extracted
helper source. Seventeen successes and the expected no-path failure passed
independent validation. All earliest-arrival metrics were bit exact. The
largest measured fixed-arrival wall improvement was accelerating circles,
25.2320069 to 23.6112598 seconds; set-time, occlusion, and target-exit walls
were 2.9573416, 20.7929114, and 4.5272330 seconds. Their motion changes versus
the no-gradient affine source were zero, 1.41e-6, 5.81e-7, and 1.07e-6 degrees.
All collision and kinematic certificates passed.

An additional seeded central-difference matrix covered 1, 2, 5, and 9
segments in both fixed- and variable-time layouts. Across all eight cases, the
maximum absolute gradient error was 2.83e-9 and the maximum relative error was
1.14e-9.

- Full tests: 57 passed in 29.2256310 seconds.
- Code Analyzer: 56 maintained MATLAB files, 0 messages.
- Visible success/failure: four/two figures; failure retained two rejected
  transitions and `noValidatedSeed`.
- Gradient-checkpoint production: 30 files and 7,231 lines.
- Gradient-checkpoint maintained MATLAB tree after removing 29 redundant test-only blank
  lines: 56 files and 11,957 lines.
- Core files: solver 885 lines, planner 888 lines.

## Buffered convex-envelope membership

The final profile showed 3.74 seconds in repeated seed-envelope
`polyshape.isinterior` calls. The helper already rejects nonconvex regions and
buffers each accepted convex region by the same tolerance. The retained change
uses vectorized `inpolygon` on those exact buffered vertices. Focused inside,
outside, and concave-envelope cases pass.

Two serial accelerating-circle runs measured 22.5436799 and 23.2924850
seconds, a 22.9181-second median, versus 23.5150876 and 23.6112598 seconds
before the substitution, a 23.5632-second median. The median improvement is
2.7 percent. The final full sweep run was 23.2993549 seconds. Moving/deforming
U.S. wall decreased from 27.5321294 to 26.9813917 seconds in the serial sweep.
Small cases remain startup-noise dominated and no universal runtime claim is
made.

The complete 18-example headless gate was repeated. All metrics were exact to
the analytic-gradient source; 17 successes and the expected no-path failure
passed independent validation. The final suite passed 58 tests in 29.0953213
seconds, Code Analyzer found zero messages across 56 maintained files, visible
success produced four figures, and visible failure produced two figures with
two rejected transitions.

Final size is 7,231 production lines and 11,974 maintained MATLAB lines. The
solver is 885 lines and planner 888. The interactive sandbox remains untracked.

### Batched complete-history containment

Concatenating every canonical obstacle-history vertex reduces convex-envelope
membership from one polygon query per slice to one query per obstacle/region.
The strengthened regression rejects a history that starts inside and ends
outside, as well as outside and concave cases. All final trajectory metrics
remained exact.

Accelerating-circle serial pairs changed from 22.5436799/23.2924850 seconds
before history batching to 21.5364519/22.3062824 seconds after it, improving
the median from 22.9181 to 21.9214 seconds (4.35 percent). The final sweep run
was 22.5887965 seconds. Moving/deforming U.S. improved from 26.9813917 to
26.8349249 seconds in the final serial sweeps. The complete 18-example gate,
58 tests in 29.3425297 seconds, zero-message analysis across 56 files, and
visible success/failure checks all passed.

The final profile confirms the mechanism: envelope-related `inpolygon` calls
fell from 6,452 to 292 (95.5 percent), and envelope-helper time fell from
3.05102 to 1.92402 profiled seconds. `buildCorridor` fell from 4.73690 to
4.00813 profiled seconds. Profiler wall time is not compared to no-profile
benchmark rows.

After the resolver removed its duplicate defaults call, the exact frozen source
again passed all 58 tests in 29.1248773 seconds and Code Analyzer again found
zero messages across 56 maintained MATLAB files.

Against the complete feasibility-recovery sweep on the same branch, summed
18-example wall time decreased from 271.4097073 to 252.0683835 seconds (7.13
percent) and the median per-case reduction was 4.11 percent. Seventeen rows
were faster. Opposing-U increased from 21.4478266 to 21.7702201 seconds (1.50
percent) despite bit-exact trajectory metrics; this unfavorable single-pair
timing remains visible and no universal speedup is claimed.

A durable full-decision directional-gradient regression was then added while
removing the same number of redundant test separators. The absolute final
suite passed 59 tests in 29.2410235 seconds; Code Analyzer remained at zero
messages across 56 maintained MATLAB files. Production and maintained-tree
line counts did not increase.

The public obstacle constructor was also exercised with a row-oriented first
history slice and column-oriented second slice. Both coordinate histories were
canonicalized to columns, and the batched complete-history envelope query
returned true for a containing convex envelope.

A seeded randomized comparison across 100 convex buffered envelopes found zero
membership differences between the previous `polyshape.isinterior` predicate
and buffered-vertex `inpolygon` for 100,000 random points and all boundary
vertices.

A seeded nonuniform evaluator stress covered 5,000 random samples across 37
segments and position through jerk outputs. The maximum absolute difference
from an independent scalar loop was 1.75e-13.

A post-audit rerun of the complete test suite passed 59 tests with zero failed
or incomplete results in 26.0991124 seconds.

Relative to pushed `2074c14`, production changed from 7,139 to 7,231 lines
(+92) and the maintained planner/test tree excluding examples changed from
7,953 to 8,748 (+795).
Production is 269 lines below the user-approved 7,500-line target, so no
performance-based overage allowance is used. The solver shrank from 900 to
885 lines while the two focused internal helpers hold 111 lines.

The user then explicitly confirmed that example files have no repository line
cap and authorized tracking the 694-line interactive sandbox. The 24 example
files total 3,920 lines; the combined MATLAB tree is 12,668 lines, while the
12,000-line cap applies only to the maintained planner/test tree excluding
examples. Example files remain excluded from planner-growth claims.

Current policy: the 7,500-line production target remains unchanged, and any
production overage must earn at least a 25 percent wall-time reduction per
additional 100 production lines, using the smallest reduction in the declared
representative benchmark set. Historical checkpoints below may quote the
earlier 30-percent formula; those measurements are preserved as historical
evidence and are not the current acceptance rule.

## 2026-08-21 — 325-less-nlp evidence-gated implementation

### Scope and environment

Work ran in the isolated `325-less-nlp-implementation` worktree on branch
`325-less-nlp`, based on exact commit
`5a067112a9f880d015f52fb97538a99010871478`. MATLAB was R2024b Update 4 on
PCWIN64 with six reported cores. Optimization Toolbox 24.2 was available;
Spline/Curve Fitting functions and a parallel pool were unavailable. No commit
or push was requested or performed.

Three long inline MATLAB launches reported a transient startup `File system
inconsistency` before executing test code. A one-line runtime probe recovered
the runtime; these startup failures produced no planner or prototype result.

### Phase A — frozen HS3 scaling baseline

`benchmarkRepeatedTurnHs3([1 2 5 10 20])` ran serially with seed 325,
earliest-arrival mode, three maximum seeds, eight HS3 segments, no mesh
refinement, and a 60-second per-improvement allowance. The tracked final rows
are in `benchmarks/repeated_turn_hs3_phase_a.csv`.

| Turns | HS3 seed attempts / solves | Route vertices | Variables | Inequalities | HS3 stage (s) | Total (s) | Planner / validation | Reason |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | :---: | --- |
| 1 | 2 / 2 | 4 | 35 | 641 | 7.5631 | 9.1771 | 1 / 1 | `goalReached` |
| 2 | 2 / 2 | 7 | 35 | 706 | 13.2254 | 13.6974 | 1 / 1 | `goalReached` |
| 5 | 2 / 2 | 17 | 35 | 901 | 13.1546 | 13.3591 | 1 / 1 | `goalReached` |
| 10 | 2 / 6 | 31 | 35 | 1226 | 75.1263 | 75.3596 | 0 / 0 | `noValidatedSeed` |
| 20 | 2 / 6 | 61 | 35 | 1876 | 193.3010 | 193.8603 | 0 / 0 | `noValidatedSeed` |

The first raw five-case run was also retained in the work log rather than
silently replaced: its total walls were 10.590, 14.052, 16.965, 75.783, and
190.380 seconds. Repeat count is one because the 20-turn case alone costs
about three minutes; no median or variance claim is made.

Exact failure diagnosis found no search truncation. At 10 turns the direct
seed collides; the visibility seed is analytically time-window infeasible and
its optimized motion has -0.159396-degree minimum clearance. At 20 turns the
visibility seed is time-window infeasible and both HS3 candidates remain
nonlinear-constraint infeasible after two relinearizations each. The failures
are classified as motion construction/collision, not topology generation.

A five-turn MATLAB profile measured 22.620 seconds in HS3, 19.411 seconds in
`fmincon`, 15.363 seconds in finite-difference gradient/Jacobian work, and
14.530 seconds over 4,769 trajectory-constraint callbacks. Corridor
constraints were the largest callback component.

### Phase B — representation comparison

Research-only prototypes covered straight, 45-degree, 90-degree, S-turn,
horseshoe, and five-alternation routes. Both were exactly C3 in their scoped
checks. The quintic B-spline passed the maintained polynomial validator. The
fixed-stop septic Bezier interpolated every route vertex but was incompatible
with that quintic schema and forced zero velocity, acceleration, and jerk at
every vertex.

Across five warm repeats, quintic motion durations ranged from 8.944 to 18.257
seconds on multi-turn cases; septic durations ranged from 28 to 84 seconds.
For the five-alternation route, quintic exposed 15 relative shape/timing
parameters, a flexible C3 septic would expose 35, and the restrictive
fixed-stop septic used five. Exact rows remain in
`benchmarks/spline_representation_phase_b.csv`. The rejected septic code and
candidate-only tests were removed under the bounded-experiment recovery rule.

### Phase C — deterministic low-dimensional optimizer

The retained research optimizer reduces the route by input arc length and
goal-horizon evidence, uses one normal-offset decision per interior control,
retimes analytically from continuous Bernstein derivative bounds, and uses
sampled clearance only as a search signal. Success is set only by an unchanged
`validateAzElTrajectory` call.

Final serial results from `benchmarkLowDimensionalSpline([1 2 5])` are in
`benchmarks/low_dimensional_spline_phase_c.csv`:

| Turns | Route vertices | Decisions / evaluations | Optimizer (s) | Total (s) | Motion (s) | Minimum clearance (deg) | Validation |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | :---: |
| 1 | 4 to 3 | 1 / 3 | 1.3008 | 1.8750 | 8.9443 | 0.0150470 | pass |
| 2 | 7 to 4 | 2 / 4 | 0.8039 | 1.1165 | 10.9851 | 0.0004261 | pass |
| 5 | 17 to 8 | 6 / 27 | 9.4410 | 10.5180 | 23.7683 | 0.0002560 | pass |

Relative to HS3-stage time, these are 82.80, 93.92, and 28.23 percent faster.
Their motion durations are respectively 34.96, 16.53, and 14.34 percent
longer than HS3. Those tradeoffs and the small clearance reserve prevent a
production replacement claim.

The 10-turn prerequisite failed in every bounded objective experiment. The
retained mean-penalty form took 131.70 seconds and remained colliding; a
worst-clearance form took 59.81 seconds with -0.14462-degree sampled clearance;
a per-obstacle form took 63.35 seconds with -0.10607-degree sampled clearance.
The two unsuccessful objective variants were removed. The 20-turn spline was
not run after the 10-turn gate failed.

Therefore supervised imitation, reinforcement learning, production planner
integration, and HS3 removal were skipped. No learned safety, completeness,
or optimality claim is made.

### Focused checks and size

- Code Analyzer returned zero messages for both benchmark harnesses, the
  shared scenario constructor, the quintic constructor and optimizer, and all
  retained research tests.
- The focused HS3 diagnostic schema test passed 1 of 1 in 5.4625 seconds.
- The retained optimizer tests passed 3 of 3 in 3.9494 seconds after one test
  exposed and caused correction of a non-monotone route-size assumption.
- A maintained alternating-slalom headless run passed planner and independent
  validation with jerk enabled, 16.06044-degree selected polyline,
  16.75828-degree smoothed path, 12.18092-second motion, collision and
  kinematic certificates true, and 25.31959-second wall time.
- Production is 30 MATLAB files and 7,117 physical lines. The non-example
  MATLAB tree is 40 files and 11,153 lines, passing the 12,000-line limit by
  847. The solver is 894 lines and planner 888, both below 900.

### Final maintained tests and example sweep

Code Analyzer checked all 64 MATLAB files and returned zero messages. No
MATLAB line exceeded 100 characters. The complete `tests` suite passed 59 of
59 with zero failed or incomplete tests in 51.488825 seconds. The combined
focused HS3 and retained research suite separately passed 52 of 52 in
52.784031 seconds.

All 18 maintained examples ran headlessly and serially in separate successful
MATLAB processes. Jerk constraints were enabled in every row. `P/V` is planner
success / independent example validation; `C/K` is collision freedom /
kinematic certificate.

| Example | Goal mode | P/V | Polyline (deg) | Smoothed (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | :---: | ---: | ---: | ---: | :---: | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635036 | 16.758281982866 | 12.180917402175 | 1/1 | 21.5868513 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519024 | 11.411616815005 | 7.816267856881 | 1/1 | 17.4983344 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721559528 | 13.431299536656 | 8.797638855700 | 1/1 | 23.9031665 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685255 | 126.114009817632 | 64.555779916429 | 1/1 | 8.7493230 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 20.000000000000 | 20.372412421524 | 22.000000000000 | 1/1 | 38.5619066 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.538940546821 | 9.538940546821 | 12.000000000000 | 1/1 | 7.0938467 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 10.097524449092 | 7.342498781519 | 6.274806792200 | 1/1 | 6.5559756 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10.000000000000 | 10.134483812277 | 10.544227895142 | 1/1 | 35.7830259 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593184851 | 12.113593184851 | 12.293137410146 | 1/1 | 20.2116419 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805146555 | 71.436692325459 | 12.986386910606 | 1/1 | 44.6775511 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 20.1083406 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955000 | 4.484905564719 | 4.612405963436 | 1/1 | 5.5753072 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10.000000000000 | 10.000000000000 | 15.000000000000 | 1/1 | 20.4914773 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 21.403170279055 | 14.220154562476 | 20.869565217391 | 1/1 | 27.1701093 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 19.824386758954 | 22.879868072534 | 24.000000000000 | 1/1 | 8.4301465 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720883753 | 24.370904895056 | 22.875124576026 | 1/1 | 47.3626836 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040466 | 43.259235381251 | 22.818548735851 | 1/1 | 16.2122849 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463508699 | 26.617251754385 | 6.683971648809 | 1/1 | 63.6908664 | `goalReached` |

The visible success rerun of `exampleObstacleFreeAzElMotion` passed and created
three visible figures in 14.3139553 seconds. The hidden failure rerun of
`exampleNoPathAzElMotion` passed its expected-failure validation and created
two diagnostic figures with one expanded state and two rejected transitions
in 33.3814964 seconds.

Before these serial invocations, an attempted PowerShell loop launched 18
MATLAB processes too rapidly; every process failed during MATLAB startup with
`File system inconsistency` before executing example code. Two later reporting
wrappers also used stale result/diagnostic field names after the example ran;
the affected forty-circle and no-path cases were rerun successfully with the
current schema. These environment/reporting failures are not counted as
example passes or planner failures.

`exampleFourAcceleratingCircles` emitted extensive `Matrix is close to
singular or badly scaled` warnings from the interior-point linear systems.
Planner and independent validation still passed, but the warning is retained
as a numerical-conditioning issue rather than suppressed from the record.
## Bounded route-interpolation recovery — 2026-08-21

Frozen source and command:

```text
41f2f92fc15b7522360ee7044c670308aaa1bf44
report=benchmarkLowDimensionalSpline(10,struct('PrintProgress',true));
```

The pre-edit baseline returned `Success=false`, `maximumSweeps`, 31 original
and 14 reduced route vertices, 12 decisions, 145 evaluations, 43.8440379
seconds motion duration, -0.011725083 degrees minimum clearance, and 90.3091
seconds total wall time.

The single candidate made the interior B-spline knot targets interpolate the
input route. MATLAB Code Analyzer reported zero messages for the three touched
research files, and all five focused representation tests passed in 1.8834
seconds. The equivalent 10-turn run nevertheless returned `Success=false`,
`maximumSweeps`, 31 original and 8 reduced route vertices, 6 decisions, 70
evaluations, 44.7352610 seconds motion duration, -0.544570324 degrees minimum
clearance, and 55.7710 seconds total wall time. Runtime improved, but physical
feasibility regressed materially, so the candidate failed the primary gate.

The bounded-experiment recovery removed the interpolation option, linear
solve, diagnostics, and focused test. `git diff` then showed no source or test
change. The exact baseline command was rerun and reproduced 14 reduced route
vertices, 12 decisions, 145 evaluations, 43.8440379 seconds motion duration,
-0.011725083 degrees clearance, and `maximumSweeps`; total wall time was
95.4536 seconds. The deterministic result recovered exactly, while the timing
difference is treated as ordinary process variation. No candidate algorithm
or benchmark artifact remains.

## Five bounded spline-option experiments — 2026-08-21

Every measured run used commit `41f2f92`, seed 325, the frozen 10-turn static
scenario, a 60.5-second horizon, unchanged code, and maintained independent
validation. Each candidate changed one optimizer option in a fresh MATLAB
process. The complete gate required exact validation, at least 0.02 degrees
continuous clearance, and less than 95.4536 seconds total wall time.

| Candidate | Reduced vertices | Decisions | Evaluations | Wall (s) | Clearance (deg) | Exact validation | Decision |
| --- | ---: | ---: | ---: | ---: | ---: | :---: | --- |
| `TimingReserveFraction=1.0` | 24 | 22 | 156 | 140.7407 | 0.000144580 | pass | reject: reserve and runtime |
| `DurationWeight=0.01` | 14 | 12 | 145 | 92.4190 | -0.011725083 | fail | reject: baseline-equivalent |
| `CollisionPenaltyWeight=5000` | 14 | 12 | 145 | 93.5085 | -0.011725083 | fail | reject: baseline-equivalent |
| `InitialStepFraction=0.1` | 14 | 12 | 145 | 93.0052 | -0.393003257 | fail | reject: worse clearance |
| `MaximumNormalOffset_deg=5` | 14 | 12 | 145 | 98.0729 | -0.353568844 | fail | reject: worse clearance/runtime |

The initially proposed `DurationWeight=0` call was rejected by input
validation before planning because the option contract requires a positive
weight. It was corrected once to the contract-valid 0.01 value; no parameter
sweep followed.

The exact default recovery command then reproduced 14 reduced vertices,
12 decisions, 145 evaluations, 43.8440379-second motion duration,
-0.011725083-degree clearance, and `maximumSweeps`. Its 109.0203-second total
wall time is reported but not interpreted as a deterministic regression.
No candidate met the complete gate, so no 1/2/5 non-regression runs, default
changes, source changes, production integration, or learned-policy work were
performed.

## Feasibility-first corridor spline experiment — 2026-08-21

The bounded candidate reused the maintained seed-corridor builder, Bernstein
half-space inequalities, and independent corridor certificate. Its search
objective ranked maximum normalized corridor violation ahead of duration,
jerk, and offset quality, and required at least 0.02 degrees clearance.

MATLAB Code Analyzer initially reported zero messages for the three candidate
files. A missing single-obstacle envelope was corrected mechanically by
assembling canonical protected obstacle slices; no search parameter was tuned.
The unchanged focused one-turn proof then returned `Success=false`, validation
false, no corridor certificate, and `NaN` continuous clearance. Because the
candidate failed the smallest feasibility proof, the frozen 10-turn candidate
gate was not run.

All candidate MATLAB and test edits were removed. `git diff --name-only --
'*.m'` was empty and `git diff --check` passed. The exact default recovery run
reproduced 31-to-14 route reduction, 12 decisions, 145 evaluations,
43.844037904 seconds motion duration, -0.011725083225 degrees clearance, and
`maximumSweeps`. Reported total wall time was 106.9760 seconds. A post-run
reporting expression then used invalid one-subscript table indexing, but only
after the benchmark had printed and completed; it did not alter the measured
planner result.

## Worst-clearance-first spline objective — 2026-08-22

The retained 10-turn trace showed a ranking weakness: its selected candidate
had -0.746881006 degrees sampled clearance at 16 seconds against barrier 4,
while the zero-offset evaluation was less deeply colliding at -0.102462142
degrees. A bounded research-only candidate therefore separated infeasible
clearance/horizon ranking from duration and jerk quality and ranked the worst
normalized clearance deficit ahead of its mean squared tie-break.

Code Analyzer returned zero messages and all three focused optimizer tests
passed. The identical 10-turn benchmark nevertheless failed exact validation
with -0.075958561 degrees continuous clearance, a 43.740088788-second motion,
145 evaluations, `stepTolerance`, and 100.5287 seconds total wall time. The
candidate was rejected because it failed the 0.02-degree reserve and 95-second
runtime gates.

All candidate MATLAB edits were removed. The exact default recovery reproduced
14 reduced vertices, 12 decisions, 145 evaluations, 43.844037904 seconds
motion, -0.011725083225 degrees clearance, `maximumSweeps`, and 85.4054 seconds
total wall time. One preceding MATLAB launch failed at startup with `File
system inconsistency` and produced no planner result.

### Earliest-stage and low-D performance diagnosis

At 0.005-degree edge resolution, the original 31-vertex visibility route had
no occupied samples and 0.000999997 degrees minimum clearance. Uniform
arc-length reduction to 14 vertices created 3,111 occupied samples and
-0.093841740 degrees minimum clearance at barrier 8. The retained reducer thus
supplies a geometrically invalid route before spline construction; later
collision and ranking failures are symptoms rather than the earliest cause.

A MATLAB profile of the retained 5-turn case reported a valid 8.6189-second
total run. Low-D optimization used 7.5682 seconds, candidate evaluation used
7.0074 seconds, and sampled clearance used 6.0315 seconds across 28 calls.
Those calls caused approximately 22,000 obstacle-shape and point-polygon
evaluations. Static clearance batching is therefore the measured runtime
mechanism relevant to the next representation experiment.

## Static batching and protected-route experiments — 2026-08-22

The retained research-only optimization batches sampled clearance for exactly
static protected obstacle histories. Four focused optimizer tests passed,
including equality with the public query for static selected samples and the
unchanged public-query path for a moving triangle. Code Analyzer reported zero
messages for the optimizer and test. The 5-turn candidate reproduced the exact
prior decision, 23.768318698-second duration, 1.201967415 integrated jerk, and
0.000256039918-degree independent clearance; optimizer wall time was 1.4484
seconds versus the prior recorded 9.4410 seconds. No broad scaling claim is
made from this focused first-run comparison.

Three bounded 10-turn route/reserve variants were then rejected:

| Candidate | Route | Evaluations | Motion (s) | Clearance (deg) | Wall (s) | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| topology-preserving subsequence | 31->22 | 77 | 60.449031 | 0.000912303 | 5.22298 | validation pass; reserve fail |
| expanded protected subsequence | 31->20 | 71 | 54.502822 | 0.000154950 | 8.45032 | validation pass; reserve fail |
| reserve-aware mean search | 31->20 | 217 | 54.374347 | 0.000785581 | 78.35975 | reserve fail; maximum sweeps |
| reserve-aware worst deficit | 31->20 | 217 | 54.342181 | 0.000342955 | 107.35371 | reserve and wall fail |

The topology reducer passed a structurally different protected-rectangle test,
and the expanded reducer preserved at least 0.02 degrees sampled polyline
clearance in that test. The unchanged continuous validator nevertheless showed
that the resulting noninterpolating quintic spans approached the benchmark
obstacles much more closely than their geometric routes. This localizes the
remaining defect to motion construction/corridor enforcement rather than route
topology.

All topology, expansion, reserve-acceptance, worst-deficit, diagnostic, helper,
and associated test edits were removed. Code Analyzer returned to zero messages.
The exact 10-turn recovery reproduced 14 reduced vertices, 12 decisions, 145
evaluations, 43.844037904-second motion, -0.011725083225-degree clearance,
`maximumSweeps`, and 2.359616 seconds total. Only the independently verified
static-clearance batching and its static/moving equivalence tests remain.

## Affine full-span corridor prototype — 2026-08-22

The existing `seedCorridorInequality` and `certifySeedCorridor` functions were
reused rather than duplicated. The missing inputs were complete convex
protected-history envelopes and correct ordered-edge-to-span correspondence.
With arc-length correspondence, the 31-vertex two-axis system was infeasible;
uniform ordered-edge correspondence made all 300 records feasible. That full
route certified 0.02 degrees clearance but required 65.9380 seconds.

Topology-preserving route trials then established the representation boundary:
20 and 21 expanded vertices met timing potential but their corridor systems
were infeasible, while 22 vertices were feasible. Edge-length timing weights
gave 60.7119 seconds. The general span-length allocation
`edgeLength_deg.^1.05` reduced the certified motion to the final retained gate:

```text
corridor10 validation=1 certificate=1
clearance_deg=0.0200000000000009
duration_s=60.2576877911092
wall_s=4.0949002
```

The retained `planCorridorConstrainedQuinticPrototype` uses 40 bounded two-axis
control-point variables and 210 complete Bernstein corridor records for that
request. The quadratic solve itself took about 0.18 to 0.20 seconds in repeated
proof runs; end-to-end method time includes route proposals, basis construction,
certificate, and independent validation.

MATLAB Code Analyzer returned zero messages for the new function and test.
Four focused tests passed: the frozen 10-turn gate, a structurally different
rectangle, a stable blocked-shortcut failure, and a moving-triangle history
envelope. The retained constructor and coordinate-optimizer suites also passed,
for 12 research tests total. The moving test was rerun warning-free after exact
history-coordinate deduplication and trusted convex-hull construction.

This is a research result, not production replacement evidence. No maintained
example or production planner calls the new method, no broad scaling matrix was
run, and the explicit 22-vertex benchmark override and 1.05 exponent are not
claimed optimal or complete for unseen route families.

## Repeated-hairpin feasibility and topology fallback — 2026-08-22

The declared stress input is a parameterized alternating-end maze with 12
horizontal protected walls. Consecutive openings alternate between workspace
edges, forcing eleven lateral reversals approaching 180 degrees. The initial
command was:

```matlab
benchmarkHairpinCorridorQuintic(12, struct( ...
    'FigureVisible', 'off', 'RouteVertexCount', 22));
```

Before the topology change, the request produced no visibility seed: 50 nodes,
47 accepted edges, five homology states, five expanded states, 166 rejected
transitions, and no truncation. One wall succeeded, while every case from two
through twelve walls failed at the same graph-connectivity boundary.

The retained topology change preserves the original Delaunay-sparse graph when
it connects start to goal. Only when that graph is disconnected does it use all
candidate pairs, and only when `candidatePairCount * boundaryEdgeCount` fits
the existing one-million-work budget. This leaves successful sparse seed order
unchanged. Two focused tests passed: the new two-wall reversal route and the
existing two-obstacle homology-diversity case. The complete planner test file
then passed 46/46 tests in 18.4028 seconds.

The first post-topology 12-wall route contained 50 protected-clear vertices.
The route-count target grew deterministically from 22 to the first clear
26-vertex subsequence, but that affine full-span corridor was infeasible. The
retained feasibility-first recovery retries once with the complete input route
when compressed corridor feasibility fails. The final result was:

```text
hairpins=12 route=50->50 success=1 validation=1 certificate=1
duration_s=369.337421187 clearance_deg=0.02
candidate_s=16.0761214 reason=corridorPrototypeValidated
compression target=22 first clear=26 fallback=50 C3=1
maximum velocity ratio=0.446277235527
maximum acceleration ratio=0.189227125012
maximum jerk ratio=0.0677886650004
```

The six focused corridor-prototype tests passed. The frozen 10-turn gate stayed
independently valid and corridor-certified at 60.2576877911092 seconds,
0.0200000000000009 degrees clearance, and 4.048403 seconds wall time. The
structurally different rectangle, moving-history envelope, adaptive target
growth, and unclear-source-route failure checks also passed.

The 20-turn scaling case remains unfavorable. Recovery reaches a certified
61-vertex spline with 0.02 degrees clearance, but its 122.474368665-second
arrival exceeds the 115.5-second horizon. It returns
`trajectoryValidationFailed` after 38.3098911 seconds rather than hiding the
deadline violation. This is a timing-quality limitation, not a collision or
span-stop failure.

Eighteen noninteractive maintained examples were run headlessly in separate
MATLAB processes. Seventeen planner successes passed independent validation,
collision checks, and all modeled kinematic certificates. The expected no-path
example returned `noValidatedSeed`, passed its example-level failure check, and
created two hidden diagnostic figures without a selected trajectory. A
separate visible `exampleAzElPlanning` run passed and created two figures. Full
per-example metrics and wall times are appended to `benchmark.csv` under source
`41f2f92+hairpin-worktree`. `exampleAzElInteractiveSandbox` was not executed
because it blocks for live mouse-drawn inputs. `exampleFourAcceleratingCircles`,
`exampleOpeningUShapedAzElTimeSpace`, and
`exampleStraightTargetAlternatingOcclusion` emitted extensive near-singular
fmincon warnings despite valid final results; `exampleMovingBarrierWait`
emitted one such warning.

MATLAB Code Analyzer reported zero messages for the changed topology generator,
planner test, corridor prototype/tests, and both new benchmarks. The topology
generator was reduced from 924 to 888 lines after replacing a local traversal
with MATLAB graph connectivity and dropping redundant diagnostics.

Repository size remains a blocking compliance issue for production replacement:
current production MATLAB is 9,206 lines versus the 7,000 target, and the
complete MATLAB tree is 16,483 versus the 12,000 hard cap. `HEAD` was already
8,389 and 14,690 lines respectively. No performance claim is used to waive the
hard tree cap. Generated MAT/CSV/PNG evidence is untracked, and no commit or
push was performed.

## Corridor-only final verification — 2026-08-22

- Static checks: MATLAB Code Analyzer returned zero messages for the public
  planner and touched corridor/topology helpers. `git diff --check` passed.
- Automated suite: `runtests('tests','IncludeSubfolders',true)` passed 53/53
  after the legacy implementation and its direct tests were removed.
- Maintained examples: all 18 noninteractive examples ran in separate fresh
  MATLAB processes. Seventeen successes passed independent validation and
  selected `corridorQuintic`; `exampleNoPathAzElMotion` returned the expected
  validated `noValidatedSeed`. All 18 passed the zero-HS3-attempt gate. Exact
  metrics are in `benchmark.csv` under `working-tree-corridor-only`.
- Generality guards: disconnected fixed-time target, expected no path,
  connected slalom, static/dynamic timing, obstacle-free exact jerk switching,
  and the 10-hairpin corridor passed. The target arrived at exactly 24 seconds;
  the hairpin motion retained 0.02-degree certified clearance without stops.
- Graphics: a visible slalom run produced three valid figures. A headless
  no-path run produced two diagnostic figures from returned diagnostics.
- Size: core production excluding plotting is 6,954 lines; plotting is 565;
  examples are 3,910; tests are 1,360; benchmarks are 830; and the maintained
  tree excluding examples/scratch is 9,709. The largest production file is
  `generateAzElTopologySeeds.m` at 900 lines. No allowance is required.
- Limitations: finite topology/time work bounds mean no completeness or global
  optimum claim. Direct zero-derivative obstacle-free motion uses an exact C2
  jerk-switching profile; multi-turn spline motion remains C3 continuous. The
  interactive sandbox was not run.

## Span-demand timing controller — 2026-08-22

The retained timing controller converts each quintic span's measured velocity,
acceleration, and jerk utilization to a common local time demand and applies
bounded proportional feedback to normalized log span durations. Every trial is
rebuilt by the existing affine corridor solver and must pass the unchanged
independent validator. The controller permits at most 16 trials and stops after
one probe when the initial arc-length allocation does not improve arrival.

On the same headless single-U request, the prior 180-trial coordinate search
returned 24.740511444152 seconds in 32.890 seconds wall time. The controller
returned 24.973219952131 seconds in 5.991363 seconds with 11 trials: 5.49 times
faster for a 0.94 percent arrival penalty. The motion remained independently
valid and collision-free with approximately 0.0001 degrees clearance. Exact
peaks were `[1.978202183578 1.983598081965] deg/s`,
`[0.749909506789 0.745360921560] deg/s^2`, and
`[0.976970345530 0.975378311450] deg/s^3`. Three repeated controller motions
were bit exact; warm wall times were 3.440425 and 3.168312 seconds.

The full automated tree passed 53/53 in 17.9147 seconds. All 18 maintained
examples then ran serially in fresh MATLAB processes: 17 independently valid,
collision-free successes and the independently valid expected no-path result,
all with zero HS3 time. The single U was 24.9732199521 seconds in 6.053188
seconds, opposing U was 31.9439273474 seconds in 13.236511 seconds, and
alternating slalom was 13.2008531355 seconds in 6.327214 seconds. Dense
concavity remained exactly 12.1408011078 seconds but took an unfavorable
49.841631 seconds; its selected result used zero controller trials, localizing
that cost to exact concave-corridor construction rather than controller work.
A visible success created four figures and a hidden expected failure created
two returned-diagnostic figures. The interactive sandbox was not run.

Final recount is 7,171 core production lines excluding 565 plotting lines and
9,928 maintained lines excluding examples/scratch. The largest production file
is 900 lines, and the maintained-tree 12,000-line cap passes. Core production is
171 lines above the 7,000-line target. A 51.3-percent minimum wall reduction
would be required for a size allowance, but the dense-concavity regression
prevents such a branch-wide claim; size therefore remains a completion blocker.

## Monotone controller and batched affine corridor evidence — 2026-08-22

The fixed-gain controller trace decreased certified U duration through its
fifth update and then increased it. A retained objective-monotone stop removes
all later trials. An immediate isolated-process A/B pair used identical inputs
and validation: the original 11-trial controller took `11.7948769 s`; the
six-trial controller took `10.3352666 s`. Both selected exactly
`24.973219952131 s` motion. Gain-only, route-expansion, smoothing-QP,
geometric-jerk-QP, and Bernstein peak/scale trials were unfavorable and were
fully removed.

Profiling the frozen 12-wall hairpin localized `19.4845` of `22.7422` profiled
solver seconds to protected-route visibility sampling, principally repeated
obstacle normalization in 1,225 edge queries. Visibility samples now retain
the same coordinates and occupancy policy but are queried once per start
vertex. After batching, repeated unit-offset spline construction became the
dominant stage. For zero endpoint velocity and acceleration, one exact affine
control-point-to-polynomial map now supplies the corridor Jacobian. Nonzero
endpoint derivatives retain the original full path; empty corridors skip the
absent Jacobian.

The frozen hairpin candidate baseline was `17.2736906 s`. Three final 12-wall
candidate times were `8.3679933`, `8.1033994`, and `8.5242998 s`; median
`8.3679933 s` is a `51.56%` reduction (`2.064x`). Corresponding total walls
were `9.7478098`, `9.5022615`, and `10.0311034 s`, median `9.7478098 s`.
Every result independently validated, remained collision-free and corridor-
certified, retained `0.02 deg` clearance, and returned bit-identical
`164.828287993221 s` motion. The final recorded run was:

```text
HAIRPIN_FINAL success=1 validation=1 totalWall=9.9147208
candidate=8.4738678 duration=164.828287993221
polyline=174.460973313305 smooth=190.315066870802
collision=1 clearance=0.02 cert=1 route=50 fallback=1
```

The final fresh U run used jerk constraints, six controller trials, and zero
HS3 time. It returned planner and independent-validation success, a
`34.942588040466 deg` polyline, `36.518869120532 deg` sampled smoothed length,
`24.973219952159 s` motion duration, collision freedom, and
`0.0000999999983853 deg` minimum clearance in `8.3241566 s` wall time. The
motion remains `9.44%` slower than the frozen main-branch
`22.818548735851 s` reference, so it misses the required `5%` limit of
`23.959476172644 s`. Runtime and hairpin gates pass; arrival quality does not.

Code Analyzer reported zero messages for the four affected MATLAB files. The
complete automated tree passed `53/53` in `27.2821021 s`. A full maintained-
example rerun was not performed after this runtime-only change; the interactive
sandbox also remains untested. The new converter file has 142 physical lines,
but it moves the prior local conversion responsibility out of
`buildAzElQuinticSpline`; exact request-local net growth cannot be separated
reliably from the already-untracked corridor replacement worktree.
Current recount is 7,267 core production lines, 565 plotting lines, and 10,024
maintained lines excluding examples/scratch. The maintained-tree cap passes,
but core production is 267 lines above the 7,000-line target.

## Exact continuous-derivative retimer — 2026-08-22

This section supersedes the immediately preceding U-gate result. The retained
method is control-theoretic rather than a gain sweep. For a bounded small
static system, it represents each continuous velocity, acceleration, and jerk
polynomial as an affine function of corridor offsets. LP bisection minimizes a
common derivative time scale; exact stationary points of the returned
polynomials are exchanged back as constraints. Subsequent span-time feedback
uses a bounded secant gain. Every retained trial still passes the independent
continuous collision and exact polynomial kinematic validator.

The input-driven eligibility rule requires earliest arrival, static geometry,
zero endpoint derivatives, and at most 100 decision-span work units. A bounded
experiment increased this cap to 200. Slalom duration remained exactly
`13.200853135488 s`, exact exchange was not retained by the selected result,
and wall increased to `6.2049921 s`; the cap change was fully reverted.

Focused final evidence:

- U: `23.958753333083 s`, `5.9070831 s` focused wall, four timing trials, 22
  LP solves, exact exchange accepted, collision-free with approximately
  `1e-4 deg` clearance. The duration passes the frozen five-percent limit
  `23.959476172644 s` by `0.000722839561 s`.
- Alternating slalom: `13.200853135488 s`, `4.8009601 s` focused wall,
  independently valid and path-identical to the retained legacy controller.
- Opposing U: `28.575922291343 s`, `4.9860068 s` focused wall, independently
  valid; the prior controller result was `31.9439273474 s`.
- 12-wall hairpin: `164.828287993221 s` duration, `6.3611859 s` candidate,
  `6.9003699 s` total wall, `0.02 deg` clearance, corridor certificate and
  independent validation passed. Exact exchange correctly remained outside
  its bounded work envelope.

All 18 maintained examples then ran headlessly, serially, and in separate
MATLAB processes. Jerk constraints were enabled in every row. `P/V` is planner
success / independent validation; `C/K` is collision / kinematic certificate.

| Example | Mode | P/V | Polyline (deg) | Smoothed (deg) | Duration (s) | C/K | Wall (s) |
| --- | --- | :---: | ---: | ---: | ---: | :---: | ---: |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.0604396350 | 16.0939038492 | 13.2008531355 | 1/1 | 5.0060698 |
| `exampleAzElPlanning` | earliest | 1/1 | 11.1521195190 | 11.3711961911 | 7.87966799510 | 1/1 | 3.7547719 |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.7007215595 | 12.7825547226 | 12.1408011078 | 1/1 | 63.7774602 |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 110.720320521 | 105.698249574 | 1/1 | 8.4310602 |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.3633030073 | 24.1898172734 | 22 | 1/1 | 7.4556229 |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.53894054682 | 9.53894054682 | 12 | 1/1 | 1.9469955 |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 7.47189347130 | 7.47189347130 | 6.92326129150 | 1/1 | 2.5638684 |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10 | 10.9963769559 | 1/1 | 4.1996987 |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12 | 12 | 13.6192201977 | 1/1 | 4.8828633 |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 85.2741687071 | 86.3554199630 | 21.1566336854 | 1/1 | 38.1883850 |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 1.4665151 |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.47213595500 | 4.47213595500 | 4.53112887415 | 1/1 | 1.7182018 |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 12.1926012246 | 1/1 | 5.2892453 |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 21.4031702791 | 13.7887424080 | 20.8695652174 | 1/1 | 7.0890508 |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 20.1815464898 | 21.7103196601 | 24 | 1/1 | 6.4284400 |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 24.5077116377 | 24.5056885160 | 28.5759222913 | 1/1 | 8.2111642 |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.9425880405 | 35.8847577549 | 23.9587533331 | 1/1 | 5.3970581 |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.3733117302 | 22.4077742797 | 14.9628027709 | 1/1 | 36.1710905 |

All durations are preserved or improved relative to the preceding
corridor-only matrix. The frozen main/HS3 durations remain materially better
for several difficult static and moving cases, so an all-example
meet-or-beat claim is not made. The dense-concavity wall result is unfavorable
and retained as a separate performance problem.

Verification commands and results:

- `runtests('tests','IncludeSubfolders',true)`: 54/54 passed in 23.762344 s.
- Focused exact-retimer test file: 4/4 passed in 9.834277 s.
- Code Analyzer: 65 maintained MATLAB files, zero messages.
- `git diff --check`: no whitespace errors; only existing LF/CRLF notices.
- Visible obstacle-free success: four figures, 643 graphics objects, valid
  `4.531128874149 s` motion.
- Hidden expected no-path: two diagnostic figures, 341 graphics objects, two
  rejected transitions, independently valid `noValidatedSeed` result.

Final size is 7,312 core production lines excluding 565 plotting lines and
10,013 maintained MATLAB lines excluding examples/scratch. Each production
file is at most 900 lines. The 12,000-line maintained cap passes; core remains
312 lines over its 7,000-line target. The new 184-line
`optimizeAzElExactTraversal.m` owns exact derivative exchange so the main
corridor solver remains 745 lines; this is a new production responsibility,
not hidden inside an oversized solver. The interactive sandbox remains
untested.

## Active-set QP and ten-level exchange — 2026-08-22

This section supersedes the preceding exact-retimer timings. The frozen
performance environment was MATLAB R2024b Update 4 on an AMD64 Family 23
Model 113 CPU, with no Parallel Computing Toolbox/pool, deterministic seed
325, hidden figures, and unchanged example inputs and validation.

The declared performance cases were dense static concavity and the
moving/deforming U.S. outline. Profiling the dense baseline measured
`63.6346615 s` wall, including `60.4469126 s` in two dense interior-point
`quadprog` calls, `1.0777094 s` in spline construction, `0.5621513 s` in
validation, and `0.4179040 s` in seed generation. The driver was the affine
corridor inequality system, not topology or collision queries.

The retained solver first obtains an LP-feasible bounded point. Proven LP
infeasibility now skips the unnecessary QP; feasible systems use active-set
`quadprog` from that point. Dense concavity returned the bit-identical
`12.140801107795 s` duration with independent validation. Candidate walls
were `3.5709276`, `3.8215486`, and `3.6560574 s`; the final isolated matrix
recorded `3.7330015 s`, a 94.15-percent reduction from the prior
`63.7774602 s` isolated row. No topology, geometry, safety margin, limit, or
arrival policy changed.

Increasing exact derivative exchange from eight to ten bisection levels
improved U from `23.958753331534 s` to `23.801121658178 s`. A focused wall was
`5.9427083 s`, and the final isolated wall was `6.0206867 s`. Twelve levels
returned an unfavorable `24.464044542719 s` controller basin and were
immediately reverted; ten levels then reproduced bit exactly. Opposing-U
retained `28.575922287848 s`.

The 12-wall hairpin remained independently valid and corridor-certified at
`164.828287993153 s`, `0.02 deg` clearance, `6.6162314 s` candidate time, and
`7.1631349 s` total wall.

### Rejected dynamic and static experiments

- Applying the full timing controller to every validated dynamic candidate
  selected the historical `63.0848051466 deg` seed and improved duration from
  `21.1566336854` to `17.8606111859 s`, but wall increased to
  `123.0861456 s`; the change was removed and the baseline reproduced.
- A path-parameterized unit-gain fixed-point prototype avoided repeated
  obstacle queries and produced a valid `17.8505455378 s` motion in
  `3.1016398 s` of retimer work. It still missed the frozen
  `12.986386910606 s` reference and was not added to production.
- Unconstrained exact geometry exchange on the same dynamic seed accepted a
  60.55-percent derivative-scale reduction but crossed moving protected
  geometry; it was removed.
- Re-evaluating timing eligibility after the dense static fallback and exact
  geometry exchange on its 40-work system produced no duration improvement;
  both experiments were removed.
- The existing `codex/paper-retimer` branch contains a 918-line Debrouwere
  sequential-convex retimer, but its small-grid path invokes `fmincon`; it was
  not imported into this non-NLP, size-constrained branch.

### Final isolated maintained-example matrix

Every example used jerk constraints and zero HS3/NLP time. `P/V` is planner /
independent validation and `C/K` is collision / kinematic certificate.

| Example | Mode | P/V | Polyline | Smoothed | Duration | C/K | Wall |
| --- | --- | :---: | ---: | ---: | ---: | :---: | ---: |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.0604396350 | 16.0939038492 | 13.2008531355 | 1/1 | 5.6057875 |
| `exampleAzElPlanning` | earliest | 1/1 | 11.1521195190 | 11.3621319718 | 7.87638446302 | 1/1 | 3.7984968 |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.7007215595 | 12.7825547226 | 12.1408011078 | 1/1 | 3.7330015 |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 110.720320521 | 105.698249574 | 1/1 | 8.1444993 |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.3633030073 | 24.1898172734 | 22 | 1/1 | 6.9149601 |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.53894054682 | 9.53894054682 | 12 | 1/1 | 1.9762920 |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 7.47189347130 | 7.47189347130 | 6.92326129150 | 1/1 | 2.9338914 |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10 | 10.9963769559 | 1/1 | 4.1621376 |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12 | 12 | 13.6192201977 | 1/1 | 5.0249601 |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 85.2741687071 | 86.3554199630 | 21.1566336854 | 1/1 | 37.3056387 |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 1.6125348 |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.47213595500 | 4.47213595500 | 4.53112887415 | 1/1 | 1.8623578 |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 12.1926012246 | 1/1 | 5.2572565 |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 21.4031702791 | 13.7887424080 | 20.8695652174 | 1/1 | 7.1288445 |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 20.1815464898 | 21.7103196588 | 24 | 1/1 | 6.6824366 |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 24.5077116377 | 24.5056885156 | 28.5759222878 | 1/1 | 8.2537591 |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.9425880405 | 35.8186005827 | 23.8011216582 | 1/1 | 6.0206867 |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.3733117302 | 22.4077742797 | 14.9628027709 | 1/1 | 34.8486747 |

The isolated wall sum decreased from `211.9775617` to `151.2662157 s`, a
28.64-percent aggregate reduction dominated by dense concavity. The smallest
per-example reduction was negative because fresh-process timing varied; no
uniform speedup or production-size allowance is claimed. Durations were
preserved within numerical roundoff except improvements to U and
`exampleAzElPlanning`.

Final checks: 54/54 tests passed in `24.886636 s`; Code Analyzer returned zero
messages for the three affected production files; `git diff --check` found no
whitespace errors beyond existing line-ending notices. Core production is
7,333 lines excluding 565 plotting lines, and maintained MATLAB excluding
examples/scratch is 10,034 lines. The 12,000-line and 900-line caps pass; core
remains 333 lines above target. The all-example historical duration goal and
core-size target remain open.

## Projection consolidation and control-law boundary — 2026-08-22

Repeated nearest-boundary projection was consolidated through the maintained
`pointPolygonClearance` invariant in static corridor construction, dynamic
seed expansion, and route-clearance expansion. One-use maximum, empty-record,
best-partial, and static-geometry helpers were inlined without changing their
contracts. Focused U, hairpin, moving/deforming, obstacle-free, and no-path
runs remained independently valid; U recovered at
`23.801121658982 s`, and the hairpin remained
`164.828287993153 s` with `0.02 deg` certified clearance.

The complete automated tree then passed 54/54 in `25.0606 s`, and Code
Analyzer returned zero messages for all 65 maintained MATLAB files. Core
production is now 7,186 physical lines excluding 565 plotting lines;
maintained MATLAB excluding examples/scratch is 9,887 lines. The 900-line
per-file and 12,000-line maintained caps pass, while core remains 186 lines
above target.

A bounded controller experiment tested the control-theoretic unit log-time
gain implied by derivative scaling. It improved U from `23.801121658982` to
`23.746859860594 s`, but planning regressed from `7.876384463025` to
`7.879667995135 s` and slalom from `13.200853135545` to
`13.233991799907 s`; it was removed. A smooth exact-exchange gain schedule
improved U to `23.752030563984 s` but regressed planning to
`7.877654207014 s`. A two-band schedule then entered a worse
`24.277638906889 s` U basin. Both schedules were removed. The retained
controller reproduced U at `23.801121658982 s`, planning at
`7.876384463025 s`, slalom at `13.200853135545 s`, and opposing U at
`28.575922287848 s` with independent validation.

The diagnostic secant trace explains the boundary. U began with log-demand
error norm `0.39193` and subsequently accepted gains `0.72343`, `0.42026`,
`1.0`, and `0.71226`; planning began near balance at `0.00807`; slalom did
not accept exact exchange and converged under the fixed gain. Because each
corridor QP may change geometry, this is a switched, nonsmooth response—not a
fixed plant whose optimal gain depends only on derivative order or error
magnitude. A future approach must estimate geometry response or enforce
trust-region acceptance; further scalar gain guesses are excluded.

## Retained one-step trust-region controller — 2026-08-22

The retained controller resolves the switched-response issue without another
gain formula. On the first accepted exact-exchange update only, it evaluates
the theoretically ideal unit log-time step and the established `0.7` damped
step, keeps the shorter successful response, and then resumes the existing
secant controller. The comparison costs at most one additional bounded
small-system solve and is selected from observed validated duration; it does
not branch on scenario identity.

Focused results preserved planning at `7.876384463025 s`, slalom at
`13.200853135545 s`, and opposing U at `28.575922287848 s`. U selected the
unit response and improved from `23.801121658982` to
`23.746859860594 s`, 4.07 percent above frozen main and below the five-percent
gate. Its fresh-process no-plot wall was `5.9768360 s`, including
`5.2146382 s` reported planner time. The 12-wall hairpin remained successful,
independently valid, and corridor-certified at `164.828287993153 s` and
`0.02 deg` clearance; candidate time was `8.7038268 s` under the current
machine load.

All 18 maintained examples ran in fresh processes with plots disabled. The
17 successes passed independent example validation, the expected no-path
case returned `noValidatedSeed` and passed its failure contract, and every
row recorded zero HS3/NLP execution. Durations were unchanged from the prior
matrix except the U improvement.

| Example | Success/valid | Duration (s) | Wall (s) |
| --- | :---: | ---: | ---: |
| `exampleAlternatingSlalom` | 1/1 | 13.2008531355 | 5.5760827 |
| `exampleAzElPlanning` | 1/1 | 7.87638446302 | 3.9926940 |
| `exampleDenseConcaveAzElMotion` | 1/1 | 12.1408011078 | 3.7691847 |
| `exampleFortyMovingCircleGrid` | 1/1 | 105.698249574 | 8.7373155 |
| `exampleFourAcceleratingCircles` | 1/1 | 22 | 6.9427357 |
| `exampleInterceptMovingTargetAtSetTime` | 1/1 | 12 | 1.9055792 |
| `exampleInterceptMovingTargetEarliest` | 1/1 | 6.92326129150 | 2.5512170 |
| `exampleMovingBarrierWait` | 1/1 | 10.9963769559 | 4.1809937 |
| `exampleMovingCircleNoAzimuthWrap` | 1/1 | 13.6192201977 | 5.0294509 |
| `exampleMovingDeformingUSOutlineVisibility` | 1/1 | 21.1566336854 | 37.5709360 |
| `exampleNoPathAzElMotion` | 0/1 | `NaN` | 1.4948528 |
| `exampleObstacleFreeAzElMotion` | 1/1 | 4.53112887415 | 2.0797329 |
| `exampleOpeningUShapedAzElTimeSpace` | 1/1 | 12.1926012246 | 5.0517531 |
| `exampleStraightTargetAlternatingOcclusion` | 1/1 | 20.8695652174 | 7.4046856 |
| `exampleTargetExitsObstacle` | 1/1 | 24 | 6.8877105 |
| `exampleTwoOpposingUVisibilityGraph` | 1/1 | 28.5759222878 | 8.6294735 |
| `exampleUShapedAzElTimeSpace` | 1/1 | 23.7468598606 | 5.9768360 |
| `exampleUSOutlineExtremeVisibility` | 1/1 | 14.9628027709 | 35.8570521 |

The no-plot wall sum was `153.6382859 s`; fresh-process timing differs from
the prior matrix, so no uniform incremental speed claim is made. The earlier
active-set change still provides the material aggregate wall reduction.
Automated tests passed 54/54 in `24.9829 s`, Code Analyzer reported zero
messages across 65 maintained MATLAB files, and `git diff --check` found no
whitespace errors beyond line-ending notices. Current core is 7,200 lines
excluding 565 plotting lines; maintained MATLAB excluding examples/scratch is
9,901 lines. Core remains 200 lines above target, and the all-example frozen
main duration goal remains open.

## Minimum-jerk dynamic safe-side controller — 2026-08-22

The path-fixed timing controller first recovered the moving/deforming short
visibility seed at `17.8505455488212 s`. A feasibility-only safe-side exchange
improved it to `16.9391537555 s` but stalled with both acceleration and jerk
saturated. Replacing the arbitrary zero LP objective with a convex sampled,
limit-normalized integrated-jerk quadratic produced monotone geometry updates.
Keeping the obstacle activation horizon fixed while backtracking the decision
trust radius corrected a barrier-row loss found during the bounded prototype.

The retained production controller is input-driven: it runs only for earliest-
arrival dynamic routes with at least 12 route vertices, uses a `0.5 deg`
maximum control step, three bounded backtracks, at most 24 accepted iterations,
and validates every proposed motion against the original protected dynamic
geometry. It does not use example names, reference trajectories, hidden
waypoints, or relaxed validation.

Fresh production evidence for `exampleMovingDeformingUSOutlineVisibility`:

- duration `12.873502939647 s` versus frozen reference
  `12.986386910606 s` (improvement `0.112883970959 s`);
- selected polyline `63.0848051465551 deg`, smoothed path
  `64.4440622821245 deg`;
- independent validation, continuous collision, velocity, acceleration, and
  jerk checks all passed; minimum protected clearance
  `0.00176885089852413 deg`;
- planner time `40.6991780 s` and no-plot wall `58.6827768 s` in the focused
  run; the independent matrix row measured `59.3688435 s`. This is slower
  than the prior `33.4187830 s` fixed-point run, so no runtime improvement is
  claimed.

A reference-only representation experiment was used for diagnosis, never in
production: a doubled-knot C3 quintic fit reproduced the removed reference to
`0.0003631 deg` at `12.9864056 s`, proving that geometry smoothness—not scalar
timing gain—was the limiting mechanism. The reference file and all prototype
artifacts were removed after recording aggregate evidence.

All 18 maintained noninteractive examples then ran in separate fresh MATLAB
processes with plots disabled. The 17 expected successes passed independent
validation; `exampleNoPathAzElMotion` returned the expected
`noValidatedSeed`. The full row data is appended to `benchmark.csv` under
`working-tree-minimum-jerk`. Automated tests passed 54/54 and Code Analyzer
reported zero messages. The current size is 7,475 core production lines
excluding 565 plotting lines and 10,167 maintained MATLAB lines excluding
examples/scratch; the 900-line per-file, conditional 7,500-line core, and
12,000-line maintained limits pass.

HS3 execution files were already deleted on this branch. The remaining HS3-
only seed-summary fields, elapsed-time field, live benchmark flag, and current
test assertions were removed. Historical benchmark rows remain as evidence.
Several frozen main-branch example durations still outperform this branch, so
the broader all-example meet-or-beat objective remains open.

## Compact C3 dynamic duration controller — 2026-08-22

Repeated scalar gain changes were stopped after diagnosis showed that every
fixed-duration geometry solve naturally approaches the velocity boundary, so
post-solve derivative utilization does not identify the next feasible duration.
The retained controller instead uses a hybrid bounded law: command 15% shorter
durations until feasibility switches, then recover inside the valid/invalid
bracket. Every duration candidate is a convex minimum-jerk QP over an
eight-span, doubled-knot C3 quintic with exact endpoint conditions, sampled
workspace/velocity/acceleration/jerk inequalities, and time-correct protected
obstacle half-planes. The maintained validator remains authoritative.

The bounded prototype gates were:

- forty moving circles: `62.4777398626363 s` versus frozen
  `64.5557799164289 s`, independent clearance `0.0211388632 deg` in the
  513-point prototype;
- moving circle: `9.00245747230184 s`, below its frozen reference and valid;
- moving/deforming U.S.: the fixed eight-span representation was infeasible at
  `12.873502939647 s`, so the proven variable-route minimum-jerk controller
  was preserved rather than replaced.

Production runs at most one compact candidate for earliest-arrival static or
dynamic inputs with a shortest eligible seed of three through ten vertices.
A zero-length seed is eligible only when its bounded hold-recovery result
passed. A 256-vertex maximum per obstacle slice bounds polygon projection
work. The work gate was added after a dense geographic-outline experiment
improved duration to `6.31442280144023 s` but regressed wall to
`71.7744356 s`; after rejection, the established outline result returned at
`9.08019276709151 s` and `38.5873657 s` wall. This is an input-resolution
rule, not a scenario or obstacle-name branch.

Halving the compact constraint grid from 513 to 257 points survived the strict
proof window:

- forty circles retained `62.4777398626363 s`, passed independent validation,
  had `0.0018431217789 deg` clearance, and final wall `16.3138462 s`;
- moving circle reached `8.75122873615098 s`, passed independent validation,
  had `0.00126607744059 deg` clearance, and wall `6.6309076 s`.

The same unchanged controller was then tested on static compact topology.
Dense concavity reached `8.70379660028619 s` versus frozen
`8.79764398400 s`, with `0.00383572502922 deg` clearance and production wall
`5.4895786 s`. Basic planning reached `7.64965634404255 s` versus frozen
`7.81626785688 s`, with `0.0154824394667 deg` clearance and `5.2756101 s`
wall. Expanding the seed cap to ten improved slalom to `10.8556642583563 s`.
A seventh duration trial improved U to `22.6408601069837 s`, beating main with
`8.1805762 s` wall and narrow positive `0.0000217071124385 deg` clearance.
A recovered direct-wait seed improved moving barrier to `10.3713875624741 s`.

For direct rest-to-rest fixed-arrival trials, the exact jerk-switching profile
is now uniformly stretched to the requested duration. Position is preserved;
velocity, acceleration, and jerk coefficients decrease by the first, second,
and third powers of the time scale. This moved earliest target intercept to
`6.11153430175781 s` versus frozen `6.275807672 s`. Specified intercept, four
accelerating circles, alternating occlusion, and target-exit fixed-time rows
all retained their required arrival and independent validation.

Rejected dense/opposing experiments remain visible: opposing U reached only
`26.6497104563525 s` with 12 spans, alternate homotopies were infeasible, and
a 6-degree offset region regressed it to `29.1180993363482 s`. Dense
nearest-vertex linearization first exposed two indexing failures; the
source-correct form remained at `9.08019276709151 s` and was removed because
it produced no verified improvement. A direct interpolated-edge projection
also accepted no shorter motion (`37.3217196 s` wall without occupancy sign,
`40.2701656 s` with sign) and was removed. Exact original geometry remains in
both generation and validation.

The retained fresh-process matrix is recorded in `benchmark.csv` under
`working-tree-compact-c3`:

| Example | Duration (s) | Wall (s) |
| --- | ---: | ---: |
| `exampleAlternatingSlalom` | 10.8556642584 | 9.4585237 |
| `exampleAzElPlanning` | 7.64965634404 | 5.2756101 |
| `exampleDenseConcaveAzElMotion` | 8.70379660029 | 5.4895786 |
| `exampleFortyMovingCircleGrid` | 62.4777398626 | 16.3138462 |
| `exampleFourAcceleratingCircles` | 22 | 7.0245682 |
| `exampleInterceptMovingTargetAtSetTime` | 12 | 1.7462867 |
| `exampleInterceptMovingTargetEarliest` | 6.11153430176 | 2.2133061 |
| `exampleMovingBarrierWait` | 10.3713875625 | 5.3361420 |
| `exampleMovingCircleNoAzimuthWrap` | 8.75122873615 | 6.6309076 |
| `exampleMovingDeformingUSOutlineVisibility` | 12.8735029396 | 63.6398616 |
| `exampleNoPathAzElMotion` | `NaN` | 1.4197695 |
| `exampleObstacleFreeAzElMotion` | 4.53112887415 | 1.7442480 |
| `exampleOpeningUShapedAzElTimeSpace` | 12.1926012246 | 5.3723642 |
| `exampleStraightTargetAlternatingOcclusion` | 20.8695652174 | 6.9819562 |
| `exampleTargetExitsObstacle` | 24 | 6.3354136 |
| `exampleTwoOpposingUVisibilityGraph` | 27.5872457615 | 12.6033494 |
| `exampleUShapedAzElTimeSpace` | 22.6408601070 | 8.1805762 |
| `exampleUSOutlineExtremeVisibility` | 9.08019276709 | 38.5873657 |

The final wall sum is `204.3536736 s` versus frozen `214.3340749 s`, a `4.66%`
reduction (`1.049x`). All 17 expected successes and the expected no-path row
passed. Fifteen of 17 success durations meet or beat the frozen main matrix.
Remaining gaps are extreme outline `+35.85%` and opposing U `+20.60%`.

The 12-wall hairpin remained independently valid at
`164.828287993152 s`, `0.0199999999878 deg` clearance, `9.4377117 s`
candidate time, and `10.8024597 s` total benchmark wall in the final run.
The prior faster timing was not substituted for this unfavorable evidence.

Final recovery verification produced 54/54 passing tests in `32.3868484 s`, zero Code
Analyzer messages across 76 maintained MATLAB files, and no `git diff --check`
errors beyond line-ending notices. Literal core production is 7,500 lines
excluding 565 plotting lines; maintained MATLAB excluding examples/scratch is
10,267 lines; the largest production file is 881 lines. All C3 prototype MAT
files and scratch harnesses were removed after integration.

## Batched stationary geometry and feasibility-switched C3 — 2026-08-22

The declared focused baseline used fresh MATLAB R2024b Update 4 processes,
`PlotOutputs=false`, deterministic example defaults, no parallel toolbox, and
unchanged protected geometry, margins, limits, arrival policy, and independent
validation. Before this change, opposing U was `27.5872457615295 s` path and
`13.290439 s` wall; extreme outline was `9.08019276709151 s` path and
`36.6313948 s` wall. The retained single-U reference was
`22.6408601069837 s` with `8.1805762 s` wall.

Profiling the headless extreme sequence measured `44.6659108 s` profiler wall.
`validateAzElTrajectory` accounted for `30.927561 s`, compact/corridor planning
for `29.613187 s`, `obstacleShapeAtTime` for `14.954427 s`, and
`pointPolygonClearance` for `14.373534 s`. The driving input was up to 5,352
boundary vertices. Identical static history slices were rebuilding the same
polyshape at every interior query. Reusing the prepared shape reduced an
otherwise unchanged extreme run from `36.6313948 s` to `26.7460725 s`; its
path and clearance were unchanged. Opposing U decreased from `13.290439 s` to
`12.2319648 s`, also with unchanged path and clearance.

The exact point-to-edge kernel was then extended from scalar to batched points.
Against the former scalar implementation, 490 trajectory/random dense queries
had maximum clearance error `6.8834e-15 deg`, zero nearest-point error, zero
edge-index mismatches, and `4.0734x` speedup. All three geographic regions
measured `4.04x` to `5.88x`; after matching scalar projection arithmetic at
vertex ties, both U obstacles had zero error and zero edge-index mismatches.
A maintained matrix/scalar equivalence test now covers signs, nearest points,
and deterministic edge order. Dynamic and topology-changing histories retain
the scalar time-local path; dense eligibility is opened only when preparation
proves stationary intervals.

Compact C3 previously spent its one solve on the shortest geometric seed even
when that seed had no valid motion. Requiring a validated eligible topology
selected the valid dense seed and produced `6.22216662414646 s` extreme motion
versus frozen main `6.684968340018 s`. The final focused run took
`29.0349372 s`, passed independent collision/kinematic validation, and had
`0.00709851977447 deg` clearance. This is a `39.44%` wall reduction relative
to the `47.9449594 s` frozen-main row and a `20.74%` reduction relative to the
fresh non-NLP baseline.

Opposing-U profiling showed a `24.8681467553 deg` smooth path versus main
`24.3709048951 deg`, but only `0.331` acceleration and `0.095` jerk utilization;
the long-duration minimum-jerk basin, not route length, caused the gap. A
probe at 10 percent from the physical jerk-limited lower bound toward the
validated upper bound now selects the controller mode. If feasible, the solver
bisects the lower failed/upper feasible bracket; if infeasible, it switches to
the established high-to-low continuation. Eight trials include the probe plus
the former seven-trial budget. This invariant is derived from feasibility and
contains no example, obstacle, route, or benchmark identity.

Focused fresh-process results after the switch were:

| Example | Frozen main duration (s) | Current duration (s) | Wall (s) | Clearance (deg) |
| --- | ---: | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 12.1809174022 | 10.8556642584 | 4.8657687 | 0.000502959179063 |
| `exampleUShapedAzElTimeSpace` | 22.8185487359 | 22.6408601070 | 6.1954661 | 0.0000217071124385 |
| `exampleTwoOpposingUVisibilityGraph` | 22.8751245760 | 22.1609457614 | 9.1406562 | 0.0143650783404 |
| `exampleUSOutlineExtremeVisibility` | 6.68496834002 | 6.22216662415 | 29.0349372 | 0.00709851977447 |

All four passed the maintained independent validator with continuous collision
and exact kinematic limits; the three compact rows selected compact C3 and no
production HS3 implementation exists. The single U remains below ten seconds
wall and beats main. Opposing U now beats main path by `3.12%` and reduces wall
`58.01%` versus the `21.7702201 s` frozen reference. Slalom is unchanged in
path and improves in measured wall.

Focused tests pass 6/6: four dynamic-timing tests plus stationary-shape reuse
and batched/scalar projection equivalence. Code Analyzer reports zero messages
for the four affected production files. Core production is 7,498 literal lines
excluding 565 plotting lines; maintained MATLAB excluding examples/scratch is
10,296 lines; maximum production file size is 881 lines. The temporary oracle
harness and task-specific temporary directory were removed.

The required current full 18-example matrix, full suite, and hairpin rerun are
not yet claimed. After the focused runs, new MATLAB processes began failing
before initialization with `Fatal Startup Error: System Error: File system
inconsistency`, including a zero-work batch and a separate non-batch startup.
One delayed `disp('STARTUP_OK')` launch later succeeded, but every subsequent
fresh benchmark launch failed again despite 6/30-second cooldowns and verified
service-host restarts. No maintained example ran in those failed matrix
attempts. The last complete 18-row matrix above remains historical evidence
only until fresh-process MATLAB execution recovers reliably.

## Final controlled fresh-process proof — 2026-08-22

The user authorized a reset of the MathWorks coordination state. With MATLAB
stopped, the exact `ServiceHost` and `MATLABConnector` directories were moved
to timestamped sibling backups rather than deleted. MATLAB regenerated them
and passed two zero-work startup checks. Rapid consecutive launches still
reproduced the pre-initialization file-system error, so those six failed
launches executed no example code and are not benchmark rows. Stopping only
the regenerated MathWorks Service Host and allowing a ten-second settle before
each launch then produced 18 consecutive fresh, serial MATLAB processes.

All 17 expected successes passed the examples' independent validation,
continuous collision checks, and kinematic certificates. Every success
selected `corridorQuintic`; no production HS3 implementation exists. The
expected no-path case returned `noValidatedSeed` and passed its stable failure
contract. Exact CSV rows are under source
`working-tree-batched-c3-final`.

| Example | Frozen main duration (s) | Current duration (s) | Wall (s) |
| --- | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 12.180917402175 | 10.855664258356 | 4.4654387 |
| `exampleAzElPlanning` | 7.816267856881 | 7.649656344043 | 4.0561583 |
| `exampleDenseConcaveAzElMotion` | 8.797638855700 | 8.690573182986 | 3.9323402 |
| `exampleFortyMovingCircleGrid` | 64.555779916429 | 62.477739862636 | 15.3927512 |
| `exampleFourAcceleratingCircles` | 22 | 22 | 7.1808018 |
| `exampleInterceptMovingTargetAtSetTime` | 12 | 12 | 1.6243784 |
| `exampleInterceptMovingTargetEarliest` | 6.274806792200 | 6.111534301758 | 2.0817962 |
| `exampleMovingBarrierWait` | 10.544227895142 | 10.371387562474 | 5.0897298 |
| `exampleMovingCircleNoAzimuthWrap` | 12.293137410146 | 8.751228736151 | 6.9248772 |
| `exampleMovingDeformingUSOutlineVisibility` | 12.986386910606 | 12.873502939647 | 53.0425551 |
| `exampleNoPathAzElMotion` | `NaN` | `NaN` | 1.1949310 |
| `exampleObstacleFreeAzElMotion` | 4.612405963436 | 4.531128874149 | 1.5160743 |
| `exampleOpeningUShapedAzElTimeSpace` | 15 | 11.735378678642 | 5.5235209 |
| `exampleStraightTargetAlternatingOcclusion` | 20.869565217391 | 20.869565217391 | 6.7002168 |
| `exampleTargetExitsObstacle` | 24 | 24 | 6.2389747 |
| `exampleTwoOpposingUVisibilityGraph` | 22.875124576026 | 22.160945761398 | 8.5208632 |
| `exampleUShapedAzElTimeSpace` | 22.818548735851 | 22.640860106984 | 5.5963246 |
| `exampleUSOutlineExtremeVisibility` | 6.683971648809 | 6.222166624146 | 27.9310043 |

Thus all 17 success durations meet or beat the frozen optimized main rows;
the only positive difference is `3.0e-13 s` fixed-arrival roundoff. The
current example-wall sum is `167.0127367 s`, a `33.74%` reduction from the
`252.0683835 s` optimized-main matrix and an `18.27%` reduction from the prior
`204.3536736 s` complete corridor-only matrix. This is an aggregate result,
not a uniform per-example speedup: forty moving circles, moving/deforming U.S.,
and target-exits-obstacle are slower in wall time than optimized main. The
moving/deforming row is the material outlier at `53.0425551 s` versus
`26.8349249 s`; its path nevertheless improves by `0.112883970959 s` and
remains independently valid.

The acceptance cases pass directly. Single U is `22.640860106984 s` with
`5.5963246 s` wall and `2.17071124385404e-05 deg` minimum clearance. Opposing
U is `22.160945761398 s` with `8.5208632 s` wall. Extreme outline is
`6.222166624146 s` with `27.9310043 s` wall. A fresh 12-wall hairpin run
returned the exact `164.828287993152 s` motion, `0.0199999999878 deg`
clearance, corridor certificate, and independent validation in
`8.0691334 s` candidate / `9.3474237 s` total wall.

The complete automated suite passed `56/56` in `28.7732939 s`. The combined
test/analyzer reporting command then used an invalid character/string path
expression after the tests had finished; a corrected analyzer-only rerun
checked 66 nonscratch MATLAB files with zero messages. A visible successful
planning run passed and created three figures with 529 graphics objects. The
expected failure passed and created two diagnostic figures with 343 objects,
one expanded state, and two rejected transitions. No graphics check reran the
planner to fabricate diagnostics.

Final repository audit found 7,498 core production lines excluding the
565-line plotting module, 10,296 maintained nonscratch/nonexample MATLAB lines,
and an 881-line largest production file. The final CSV source contains exactly
18 rows, 17 successes, 18 validation passes, and the recorded
`167.0127367 s` wall sum. Production search found no live `solveAzElHs3` or
`fmincon` call. `git diff --check` found no whitespace errors; its output was
limited to existing LF-to-CRLF notices. The temporary matrix wrapper was
removed. No commit or push was performed.

## Dynamic seed-slot coverage experiment — 2026-08-22

The bounded baseline used eight deterministic moving-circle fields with master
seed `3252026`, three allowed seeds, unchanged protected geometry and limits,
and public independent validation. The command was
`probeDynamicTopologyCoverage()`. Baseline planner success and independent
validation were `2/8`; total measured wall was `51.710458 s`. In two failed
cases the exact timed search produced a direct wait, the extended timed search
produced no unique additional seed, and the reserved third slot remained
unused even though a spatial visibility route existed.

The retained change computes the extended timed proposal before allocating
spatial search capacity. A slot is reserved only when that proposal is
nonempty and distinct from the already generated exact-time seeds; spatial
seeds are still appended before the timed proposal, preserving established
seed precedence. No public option, work cap, obstacle tolerance, or validation
rule changed. The final identical probe passed `3/8` in `60.5390732 s`. The new
six-circle success is independently valid at `21.3080494189 s` duration and
`0.00264401481202 deg` minimum clearance. The other five no-path results remain
visible; this is a focused coverage gain, not a completeness claim.

The complete 18-example headless gate passed: 17 independently validated,
collision-free and kinematically certified successes plus the expected
`noValidatedSeed` contract. All 17 motion durations exactly match the frozen
`working-tree-batched-c3-final` rows within `1e-6 s`; the measured wall sum was
`124.039315 s` in one warmed MATLAB process and is not compared with the prior
fresh-process wall sum. Exact rows are recorded in `benchmark.csv` under
`working-tree-topology-slot-proof`. A visible success created three figures
with 487 graphics objects; the expected failure created two diagnostic figures
with 341 objects without rerunning search. The final automated suite passes
`57/57`, including the new deterministic seed-slot regression test, and Code
Analyzer reports zero messages for the changed MATLAB files.

Route-size experiments were unfavorable and removed. On the same moving
10-wall request, the original 42-vertex dynamic fallback returned an
independently valid solution in `15.5401058 s`. Skipping expansion finished in
`5.9110476 s` but lost the solution. Densifying a sampled-clear 22-vertex
subsequence finished in `8.5416433 s`, and preserving its original temporal
coordinates finished in `9.1188355 s`; both returned `noValidatedSeed`.
Therefore the known route-dimensional runtime cliff remains unresolved rather
than trading correctness for speed.

Final size is unchanged at 7,498 core production lines excluding the 565-line
plotter. The maintained nonscratch/nonexample tree is 10,348 lines, and the
largest production file is 887 lines. The production diff is net zero lines;
the dynamic-timing test file adds 52 lines for one deterministic moving-circle
fixture and its focused assertion. Temporary stress and benchmark harnesses
were removed. No commit or push was performed.

## Shallow collision-residual feedback experiment — 2026-08-22

The collision-stage baseline was the pushed `034a6a4` topology-slot commit.
Fixed moving-circle case 1 constructed kinematically valid motions but all
three seeds failed authoritative collision validation; the visibility seed's
reported clearance was `-0.0790211794215 deg`. Reconstructing its 13-vertex
retimed spline localized the actual feedback starting residual to
`-0.00108381917352 deg` at `28.4660205183 s` duration.

The retained controller linearizes signed protected-obstacle clearance against
interior spline controls. For a violated row, the single-row minimum required
decision norm is `-bound / norm(row)`. The applied feedback gain is the lesser
of one and the trust radius divided by the largest required norm. A bounded
minimum-norm QP applies that partial residual correction. For an interior
point, the closest-boundary vector is reversed so the gradient points toward
increasing signed clearance. Intermediate steps must remain kinematically
valid and strictly improve independently measured minimum clearance; only a
fully validated trajectory can become a planner success.

The first prototype took one accepted step and reached
`0.00612677738555 deg` clearance with unchanged duration. Integrated production
case 1 passes at `0.00570897255047 deg` selected clearance. The final identical
eight-case moving-circle sweep passes and independently validates `4/8`, versus
the pushed `3/8` baseline. Cases 3, 4, 6, and 8 remain reproducible
`noValidatedSeed` outcomes; no redraw, tolerance change, or topology special
case was used.

An initial all-residual implementation failed the benchmark gate. It recovered
three extreme-outline candidates starting at `-0.00662790746078`,
`-0.0112188785752`, and `-0.0102058874618 deg`, changing the final-region path
from `6.22216662414646 s` to `8.39529809634767 s`. Disabling recovery restored
the frozen path exactly. The retained input-driven local-linearization rule
therefore limits recovery to penetrations no deeper than `0.005 deg`, one tenth
of the controller's `0.05 deg` clearance target. Case 1 still passes, while a
fresh extreme-outline rerun restores the exact baseline metrics.

The final 18-example gate used one fresh serial MATLAB process per example.
All 17 expected successes passed independent validation, collision checks, and
kinematic certificates; the expected no-path result passed its stable failure
contract. Every successful polyline length, smoothed length, and duration is
exact to the pushed topology-slot evidence. The fresh-process wall sum is
`172.6919951 s`; the moving/deforming outline remains the unfavorable wall
outlier at `53.2656011 s`. Exact rows are in `benchmark.csv` under
`working-tree-residual-feedback-final`.

The focused controller regression passes in `13.3148997 s`. The final complete
suite passes `58/58` in `48.4440443 s`, and Code Analyzer reports zero messages
across all 66 nonscratch MATLAB files. A visible success produced three figures and
487 graphics objects. The expected failure produced two diagnostic figures and
341 objects from its original search record, including one expanded state and
two rejected transitions. Final size is 7,500 core production lines excluding
the 565-line plotter, 10,367 maintained nonscratch/nonexample MATLAB lines, and
an 887-line largest production file. The two temporary proof harnesses were
removed; the pre-existing untracked benchmark artifacts were left untouched.

## Disconnected visibility boundary support — 2026-08-22

The baseline was pushed commit `0c8bf66`. On the fixed moving-circle family
with master seed `3252026`, three allowed seeds, unchanged geometry, and public
independent validation, cases 3, 4, 6, and 8 returned `noValidatedSeed`; the
baseline therefore passed 4/8. Stage diagnosis classified cases 3, 4, and 8 as
bounded-topology failures. Case 6 had a sampled-clear timed polyline with
`0.199951692541 deg` clearance, but its smoothed motion collided at
`-0.522166959136 deg` and no spatial fallback was available.

The retained invariant is limited to a disconnected bounded visibility graph.
After the existing offset/exhaustive retry ladder reaches retry three, the four
input workspace corners become ordinary visibility-tested support nodes and a
three-seed portfolio preserves one spatial-diversity opportunity. Connected
retry-zero graphs retain their established temporal seed portfolio. The
corners are not hidden waypoints: occupied or blocked corners receive no usable
edge, and the unchanged workspace-spanning wall remains `noValidatedSeed`.

Two broader prototypes were rejected. Adding corners to every graph changed
the shallow collision-controller densification factor and failed its focused
regression. Replacing the third temporal seed on every three-seed request
changed `exampleMovingCircleNoAzimuthWrap` from the frozen
`8.75122873615098 s` motion to `8.77956166926098 s`. Restricting both behaviors
to graphs that exhausted retry three restored the exact frozen moving-circle
polyline, smoothed length, and duration.

The final identical eight-case sweep passed and independently validated 8/8 in
`232.089424 s`:

| Case | Duration (s) | Minimum clearance (deg) |
| --- | ---: | ---: |
| 1 | 20.9069577259 | 0.00570897255047 |
| 2 | 21.1343850031 | 0.00537827059448 |
| 3 | 21.1728041688 | 0.00268927771046 |
| 4 | 21.2089164576 | 0.0104420223329 |
| 5 | 20.5473740302 | 0.000355731152304 |
| 6 | 21.1748286081 | 0.00165612365924 |
| 7 | 20.5780642844 | 0.0012584101104 |
| 8 | 20.9089388561 | 0.000739339048806 |

The focused dynamic suite, including the four-case replay and static wall,
passed 7/7 in `95.3418714 s`. The final complete suite passed 59/59 in
`134.081193 s`; Code Analyzer reported zero messages across 66 nonscratch
MATLAB files. The new deterministic regression requires retry-three topology,
a selected `visibilityGraph` seed, positive homology coverage, and positive
independent continuous clearance for all four formerly failing cases.

The definitive 18-example gate used one fresh serial MATLAB process per
example. All 17 expected successes passed independent collision and kinematic
validation, and the expected no-path result passed its stable failure contract.
Every successful polyline length, smoothed length, and duration is exact to the
pushed `working-tree-residual-feedback-final` rows within `1e-6`; exact new
rows are appended to `benchmark.csv` under
`working-tree-boundary-support-final`. Measured wall sum was an unfavorable
`205.6452420 s` versus `172.6919951 s` for the pushed evidence, so no runtime
improvement is claimed. The moving/deforming outline and extreme outline were
the largest walls at `61.1979723 s` and `39.7610225 s`.

A visible U-shaped success passed and created three figures with 522 graphics
objects. The visible expected failure passed its example contract and created
two returned-diagnostic figures with 342 objects, three expanded states, and
15 rejected transitions. MATLAB then returned Windows graphics teardown code
`1073807364` after printing those results; a hidden retry failed before startup
with `-1073741205`. This GUI-host fault is retained as unfavorable environment
evidence rather than reported as a planner or figure-generation pass with a
clean process exit.

Production remains exactly 7,500 core physical lines excluding the 565-line
plotter. The maintained nonscratch/nonexample MATLAB tree is 10,392 lines, and
the largest production file remains `generateAzElTopologySeeds.m` at 887
lines. The production diff is net zero lines through local compaction; the
dynamic-timing test adds 25 lines. Temporary diagnostic and example-gate
harnesses were removed, and the twelve pre-existing untracked benchmark
artifacts were left untouched. No commit or push was performed.

## Repository cleanup and module refactor — 2026-08-22

The repository cleanup retained every production algorithm and removed only
resolved generated outputs. Twenty-three internal implementation files moved
from one flat package into `geometry`, `obstacles`, `search`, `motion`, and
`validation` subpackages. Internal names now rely on package context, for
example `azElInternal.search.generateTopologySeeds` and
`azElInternal.motion.solveCorridorQuintic`. The public planner, obstacle,
validation, plotting, and example entry points did not change.

Source cleanup consolidated protected/original boundary-history
normalization, polyshape construction, and boundary-edge extraction. Dense
history vertices are collected through preallocated cells. Corridor solves
reuse immutable prepared obstacle caches passed by the planner; raw benchmark
calls still normalize and prepare once. No planner tolerance, work limit,
seed order, geometry, validation rule, or result field changed.

Seven tracked `scratch/` MATLAB/CSV outputs were removed and remain
recoverable from Git. Ignore rules now cover reproducible scratch, MAT, PNG,
run-CSV, and summary-CSV outputs. The twelve pre-existing untracked benchmark
artifacts remain on disk and are now ignored.

Verification evidence:

- Pre-change baseline: Code Analyzer `0`; tests `59/59` in `139.173458 s`.
- Focused normalization/provenance checks: `5/5` in `2.306672 s`.
- Post-refactor Code Analyzer: `0` across all nonscratch MATLAB files.
- Post-refactor tests: `59/59` in `150.384070 s`.
- Protected and original boundary-size errors retained their exact identifiers.
- Fresh-process examples: 17 independently validated successes and one
  validated expected `noValidatedSeed` failure. Every successful polyline,
  smoothed length, and duration exactly matches the
  `working-tree-boundary-support-final` evidence.
- Fresh-process example wall sum: an unfavorable `222.7331866 s` versus the
  prior `205.6452420 s`; no runtime improvement is claimed. The largest walls
  remain moving/deforming outline `60.1933815 s` and extreme outline
  `41.5000613 s`.
- Four-wall hairpin raw-solver benchmark smoke: planner/validation/corridor
  certificate `1/1/1`, duration `54.6635704076809 s`, candidate solve
  `2.6237182 s`, and `corridorPrototypeValidated` termination.
- Exact example rows are appended to `benchmark.csv` under
  `working-tree-repository-cleanup`.

Core MATLAB changes from 35 files / 7,500 physical lines / 6,468 executable
lines / 898 comment lines to 37 files / 7,773 physical lines / 6,450
executable lines / 1,200 comment lines. Executable code decreased by 18 lines.
The 273-line physical overage is documentation/comment growth explicitly
authorized by the user after regression. The largest production file is
`search/generateTopologySeeds.m` at 886 lines; all production files remain
below 900 lines.

The Windows MATLAB launcher returned a pre-startup `File system
inconsistency` error for long inline commands and one nested shell loop.
Short harness-based fresh processes completed all reported checks. The
temporary harness is removed before handoff. No commit or push was requested.

## Readability and short-file audit — 2026-08-23

The follow-up readability pass is comments and formatting only. Section 0 is
now reserved for each file's primary function: all 86 local Section 0 headers
were removed, all 229 local functions begin with a direct purpose sentence,
and duplicate local `PURPOSE` blocks were removed. The 131 internal loops retain
immediately preceding explanations, and additional decision comments describe
fallbacks, work limits, candidate acceptance, retiming, and early exits in the
four largest motion/search files.

All 22 production MATLAB files below 100 nonblank, noncomment code lines were
audited for textual callers. None is uncalled. Eighteen have multiple callers;
the four single-caller files own a stable result schema or a distinct algorithm
extracted from an already-large orchestrator. The complete rationale is in
`short_file_rationale.md`.

Text-only checks found zero local Section 0 headers, missing local-function
purpose comments, unexplained internal loops, bare assignment continuations,
code continuations at or below 120 characters, trailing whitespace, or
physical-contract hash mismatches. `git diff --check` passed. Per the user's
explicit instruction for this session, MATLAB tests, Code Analyzer, and
examples were not rerun after this comments-only pass. The earlier 59/59 and
18-example results above predate the latest comment changes and are not claimed
as post-pass execution evidence.

## Persistent interactive sandbox — 2026-08-23

The supplied two-tab UI is now `sandbox/azElInteractiveSandbox.m`. Goal Mode
automatically requests the start, then the goal, then obstacle strokes. Free
Mode requests the start, then its first endpoint, then obstacle strokes;
`Add Segment` remains for optional later endpoints. Separate start and goal
buttons are not present.

Both tabs now expose **Add Obstacle** for additional strokes. The initial
start-to-goal sequence still enters the first obstacle automatically, then
returns to idle after retaining it. The axes use an outer-position constraint
so the azimuth ticks and label remain inside their reserved area instead of
overlapping the action-button row.

The planning controls were subsequently condensed into three labeled groups.
Workspace and kinematic values share headings and use one row per setting;
timing and obstacle values also use one row each. The verbose checkbox moved
to the unused strip beneath the panel, removing the final-row overlap visible
under Windows display scaling.

The planning-control rows were moved below the panel title so the workspace
azimuth label is visible under common Windows display scaling. Canvas redraw
deletes all axes children, including graphics with hidden handles, before
resetting and reconstructing the axes; this removes the stale obstacle outline
that previously survived Reset.

Text-only checks found one primary Section 0 header, direct opening comments on
all 61 local functions, explanations immediately before all 20 loops, no bare
assignment continuations, no continuation blocks at or below 120 characters,
and no trailing whitespace. `git diff --check` passed. MATLAB, Code Analyzer,
and regression tests were not run because the user explicitly prohibited test
execution in this session.

Obstacle layers now use hidden legend handles for raw traces, simplified line
centerlines, polygon boundaries, original obstacle fills, and protected safety
outlines. Start, goal, requested routes, solved motion, and failure-route
entries retain their legend labels. Obstacle rendering and planner data are
unchanged.

## Combined corridor/HS3 method suite — 2026-08-23

The `325-full-suite` worktree combines two complete, physically isolated
planner snapshots behind `PlannerMethod`:

- corridor source: `325-less-nlp` at `2852663`;
- HS3 source: `plan-325` at `5a06711`.

Each branch's moving-target adapter is also isolated because their earliest
intercept policies differ. Corridor retains its chronological fixed-arrival
search and bracket refinement. HS3 retains its one-call moving-goal
earliest-arrival solve. Neither public dispatcher falls back to the other
method.

### Fresh-process maintained-example comparison

All 18 noninteractive maintained examples were run in a fresh MATLAB process
for each method. Each call disabled plots, animation, kinematics, search edges,
visibility graphs, swept surfaces, and verbose output. HS3 runs reapplied only
the collocation and improvement-time settings recorded by the Plan-325 source
examples.

The comparison gate required exact matches for goal-time policy, jerk policy,
planner success, independent example validation, collision/certificate fields,
and termination reason. Selected polyline length, returned-motion length, and
motion duration had to match within `1e-6`. Wall time was recorded but was not
an equality gate.

Results:

| Method | Baseline source | Cases | Validated successes | Validated expected failures | Gated differences | Wall sum (s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `corridorQuintic` | `working-tree-repository-cleanup` | 18 | 17 | 1 | 0 | 163.9501049 |
| `hs3` | `2074c14+batched-envelope-history-worktree` | 18 | 17 | 1 | 0 | 238.5162172 |

Both expected failures were `exampleNoPathAzElMotion` with planner
`Success=false`, example validation passed, and termination reason
`noValidatedSeed`. Exact fresh rows are appended to `benchmark.csv` under
`2852663+325-full-suite-corridor-worktree` and
`5a06711+325-full-suite-hs3-worktree`.

An initial PowerShell-variable loop triggered MATLAB's pre-startup Windows
`File system inconsistency` launcher error and executed no cases. The final
literal-command fresh-process runs completed every recorded case on its first
attempt. This launcher issue is not hidden or counted as planner evidence.

### Physical unplugging proof

Two task-owned temporary copies were created. The HS3 folder was deleted from
one and the corridor folder from the other. In the corridor-only copy,
`exampleObstacleFreeAzElMotion` returned success, passed independent example
validation, and echoed `corridorQuintic`. In the HS3-only copy, the same checks
passed and the result echoed `hs3`. The temporary copies were deleted after
the proof.

### Current verification boundary

MATLAB Code Analyzer checked all 109 intended MATLAB files and returned zero
messages. The scan excluded only the unrelated untracked
`report_evidence_tmp.m` and `single_u_report_tmp.m` scripts. Text audits also
found explanations immediately before all 368 loops, exactly one primary
Section 0 in every function file, no local-function Section/PURPOSE boilerplate,
and no production MATLAB file without an executable caller. Of the 49
production files below 100 code lines, nine have one caller and are justified
individually in `short_file_rationale.md`.

Canonical source comparisons found no executable divergence in the 34-file
corridor package or the 26-file HS3 package after accounting for the approved
namespace and public-entry renames. Backend dependency scans found no call to
the root planner or shared root internals and no reference to the sibling
method. Only the four public dispatchers reference both method packages.

The repository regression test suite was not run because the user explicitly
prohibited tests unless requested in this session. The example matrix above is
fresh execution evidence, not a substitute claim for unrun automated tests.
Visible interactive sandbox verification also remains unrun; its planner
selector received static and Code Analyzer checks only.

## Compact stage-timing checkpoint — 2026-08-23

This checkpoint compares branch `325-full-suite` with the frozen
`27070ac5fac6f90624731a753d4b029e7ecea8e5` (`27070ac`) baseline. The public
`SearchDiagnostics.StageTiming` contract contains exactly these seven fields:

1. `TopologyElapsedTime_s`
2. `CorridorConstructionElapsedTime_s`
3. `MotionSolvingElapsedTime_s`
4. `CollisionCheckingElapsedTime_s`
5. `FinalValidationElapsedTime_s`
6. `UnattributedElapsedTime_s`
7. `TotalElapsedTime_s`

The five named stages are exclusive: nested work is charged once, to its
owning stage. Repeated candidates and attempts are additive, including work
later discarded. `TotalElapsedTime_s` is measured independently and
`UnattributedElapsedTime_s` reconciles it against the five named stages;
over-attribution is rejected instead of being hidden.

### Maintained-example and focused verification

Each maintained example gate used fresh serial MATLAB processes.

| Method | Cases | Validated outcome | Direct process wall sum (s) | Harness example wall sum (s) | Planner sum (s) |
| --- | ---: | --- | ---: | ---: | ---: |
| `hs3` | 18/18 | 17 successes + expected validated `noPath` | 560.7800023 | 336.8454606 | 260.8179100 |
| `corridorQuintic` | 18/18 | 17 successes + expected validated `noPath` | 413.7346972 | 220.3806093 | 161.7237387 |

All 36 returned `StageTiming` records reconciled. A visible corridor smoke
run returned planner success `1`, independent validation `1`, and created two
figures. The focused final-source tests passed 23/23.

The earlier full-suite run passed 125/125 tests in 210.6225 seconds. That run
predates the final focused test additions and is recorded only as earlier
evidence. On the final source, the complete suite passed 127/127 tests with
zero failures or incomplete tests in 174.7697065 seconds. MATLAB Code Analyzer
then checked all 93 maintained production and test files and reported zero
findings.

### Frozen-baseline A/B timing

The A/B run comprised 48 successful invocations: two methods, four
representative examples, three repetitions, and two source versions. Every
observation used a fresh headless serial MATLAB process. Pair order was
balanced as baseline-candidate, candidate-baseline, baseline-candidate. The
table reports the exact median percentage changes from the recorded summary;
negative values favor the candidate.

| Method | Example | Direct-process wall | Harness wall | Planner time |
| --- | --- | ---: | ---: | ---: |
| `hs3` | `exampleObstacleFreeAzElMotion` | +1.308% | -9.959% | -4.938% |
| `hs3` | `exampleUShapedAzElTimeSpace` | +2.650% | +0.111% | +3.327% |
| `hs3` | `exampleFourAcceleratingCircles` | -0.198% | -3.193% | -3.377% |
| `hs3` | `exampleNoPathAzElMotion` | -4.035% | -4.995% | -4.884% |
| `corridorQuintic` | `exampleObstacleFreeAzElMotion` | -5.652% | -0.090% | +6.899% |
| `corridorQuintic` | `exampleUShapedAzElTimeSpace` | +0.848% | -1.667% | -0.160% |
| `corridorQuintic` | `exampleFourAcceleratingCircles` | -3.841% | -6.203% | -5.011% |
| `corridorQuintic` | `exampleNoPathAzElMotion` | +2.899% | +5.907% | +14.443% |

All 48 runs preserved status, validation, termination, policy, certificate,
and selected-seed outputs. The largest baseline/candidate physical-value
difference was `1.3056e-13`, below the `1e-6` audit threshold. The unfavorable
`corridorQuintic` no-path planner median increased by 14.443%; this is retained
as a regression signal, not averaged away or presented as a speedup.

### Exact size and current limits

Physical-line counts were recomputed from the frozen baseline after the timing
cleanup and canonical obstacle-infrastructure consolidation.

| Scope | Baseline | Current | Delta |
| --- | ---: | ---: | ---: |
| Production MATLAB | 76 files / 15,634 lines | 76 files / 15,140 lines | 0 files / -494 lines |
| Maintained MATLAB (production and tests) | 84 files / 19,057 lines | 84 files / 18,609 lines | 0 files / -448 lines |

The maintained tree still exceeds the former 12,000-line hard cap. The A/B
sample has only three observations per cell and includes fresh-process launch
noise, so it supports behavioral equivalence and identifies possible runtime
regressions; it does not establish a general speedup.

`+azElInternal` is now the neutral shared obstacle layer. The two method-local
combine, normalize, query, boundary-traversal, and signed-clearance copies were
removed. A pre-deletion equivalence gate passed 5/5 across all three copies and
the new shared query. After deletion, the permanent infrastructure tests passed
5/5 and both planner unit files passed 92/92. The broader suite was started but
stopped when the user requested no additional tests before pushing, so no
post-consolidation full-suite result is claimed.

## Corridor helper consolidation — 2026-08-23

This behavior-preserving cleanup was measured against task baseline `3280bb0`.

| Scope | Baseline | Current | Delta |
| --- | ---: | ---: | ---: |
| Fixed-goal corridor runtime closure | 42 files / 7,713 lines | 36 files / 7,366 lines | -6 files / -347 lines |
| Production MATLAB | 76 files / 15,133 lines | 70 files / 14,787 lines | -6 files / -346 lines |
| Production and tests | 84 files / 18,602 lines | 78 files / 18,256 lines | -6 files / -346 lines |

Five method-local copies were replaced with their executable-equivalent
`azElInternal` implementations. `buildStraightJerkProfile` became a local
function of its only caller, `buildQuinticSpline`. Search, candidate selection,
result construction, and independent final validation remain separate.

Verification produced:

- pre-change focused tests: 52/52 passed;
- post-change focused tests: 52/52 passed;
- MATLAB Code Analyzer: 0 messages across 14 changed MATLAB files;
- dependency audit: 36/36 runtime files reachable, 0 orphaned;
- stale deleted-helper references: 0;
- `git diff --check`: passed.

The complete maintained-example, visible-graphics, and full regression matrices
were not rerun. No `benchmark.csv` row was added because this checkpoint
executed tests and static audits, not a maintained-example benchmark.

## Input-history collision broad phase and ungrouped feasibility — 2026-08-23

This checkpoint modifies only the corridor validator in production. The
representative case is `exampleFortyMovingCircleGrid`, headless and serial in
fresh MATLAB R2024b Update 4 processes, seed 325, with plots and animation
disabled. Obstacle histories, limits, safety margin, validation, and route
quality were identical between baseline and candidate.

This is the only maintained example setting `SeedClusterDistance_deg` above
zero. Its swept input has `SourceRegionCount=1`, so the requested 2-degree
clustering creates `ClusterGroupCount=0`. A zero-distance run generated the
same 18 nodes, 62 candidate pairs, 28 visibility edges, 80 rejected
transitions, 21 expanded states, two seeds, 110.807929685255-degree selected
polyline, and 62.4777398626363-second validated motion. Ungrouped operation
was already feasible; clustering was not providing the runtime bound.

Profiling recorded 47,793 calls each to `shapeAtTime` and
`pointPolygonClearance`, with collision checking dominant. The retained broad
phase derives a lower bound from every incoming obstacle's supplied
`InternalPreparation.HistoryBounds_deg` and the caller's velocity limits. It
assumes no constant speed, rigid shape, fixed size, or named scenario. Only an
input-derived proof skips exact geometry; near obstacles keep the existing
shape-at-time query and adaptive certificate.

Three alternating fresh-process pairs produced:

| Metric | Baseline runs | Candidate runs | Median change |
| --- | --- | --- | ---: |
| Planner time (s) | 14.3362922 / 14.5753369 / 14.5689666 | 7.1588191 / 7.0040011 / 6.9326244 | -51.925% |
| Collision stage (s) | 8.2384 / 8.2169 / 8.3139 | 0.5109 / 0.5553 / 0.5085 | -93.799% |
| Direct example wall (s) | 16.1544479 / 16.5913308 / 16.4316327 | 7.9557515 / 7.7722001 / 7.6849762 | -52.703% |
| Selected collision checks | 3440 / 3440 / 3440 | 56 / 56 / 56 | -98.372% |

Every pair preserved success, independent validation, termination, graph
counts, route, sampled path length, duration, minimum clearance, collision
state, and kinematic certificate. The smallest paired planner reduction was
50.067%; no averaged result hides a slower pair. The retained default grouped
option also formed zero groups and completed in 7.3898134 seconds with
identical physical output.

The user also requested a materially stronger input history for the
moving/deforming U.S. example. Its transform now starts at the native outline,
accumulates 12 degrees of rotation, grows nominally by 18% azimuth and 14%
elevation, and supplies larger nonrigid ripples, shear, and translation at
every five-second slice. Measured protected extents changed from
57.9658931-by-24.4877162 degrees to 67.7072812-by-30.1003469 degrees (+16.805%
and +22.920% after rotation). The changed example returned a
78.0069578875528-degree polyline, 76.5702550667827-degree sampled motion,
19.2943308306518-second duration, collision freedom, kinematic validity, and
independent validation.

Verification on the retained source produced:

- 59/59 focused tests passed in 110.6526933 seconds;
- all 18 corridor examples passed in literal fresh processes: 17 validated
  successes and the expected validated `noValidatedSeed` failure;
- the visible deforming-U.S. smoke succeeded with three visible figures;
- the complete repository suite passed 132/132 in 133.3061309 seconds;
- MATLAB Code Analyzer reported zero messages across 106 MATLAB files;
- `benchmark.csv` retained its original 17-column schema, and `git diff
  --check` reported only line-ending conversion warnings.

The task adds 35 production MATLAB lines to a 13,978-line production baseline.
Applying the growth formula to the 35 task-added lines requires a 10.5%
representative reduction; the smallest paired reduction is 50.067%. The
14,013-line production tree remains above the 7,000-line target, which is an
existing repository limitation rather than something hidden by this local
speedup. Evidence covers the 40-obstacle family plus structurally different
deforming, static, and no-path checks; it proves no global scaling or
completeness claim.

Two PowerShell-variable matrix launches and one earlier static-check launch
hit MATLAB's pre-startup Windows `File system inconsistency` error and executed
no governed case. Literal commands completed every recorded example. One early
report used a stale field after a planner run; that incomplete log was excluded
from benchmark evidence.

## Shared option, goal, and obstacle helper consolidation — 2026-08-23

Task baseline: local `325-full-suite` at `a51f6e9`. The existing untracked
`docs/` directory was preserved and excluded from the change.

Three corridor helpers and seven HS3-local helpers were executable copies or
behavior-equivalent variants of neutral `azElInternal` contracts. All callers
were redirected to shared option, logical, goal, obstacle, polynomial, and
Bernstein implementations before the private files were removed. The shared
shape-at-time implementation retains the same interpolation and conservative
topology-change policy and additionally reuses a prepared shape for a
stationary matching-topology interval. No search, motion-construction,
candidate-selection, solver, validation, or public-dispatch algorithm changed.

Verification produced:

- focused pre-change baseline: 33/33 passed across shared infrastructure and
  both planners' option, moving-goal, dynamic-obstacle, and interpolation paths;
- post-change unit gate: 101/101 passed across
  `testObstacleInfrastructure`, `testFixedDurationAffineModel`,
  `testHs3Planner`, and `testAzElPlanner`;
- MATLAB Code Analyzer: zero messages across 20 modified MATLAB files;
- deleted-helper reference audit: zero matches;
- corridor-to-HS3 and HS3-to-corridor call audit: zero matches;
- `git diff --check`: passed with Windows line-ending conversion warnings only.

Physical MATLAB counts, using the same PowerShell `Get-Content` method on the
task baseline and worktree:

| Scope | Baseline | Current | Delta |
| --- | ---: | ---: | ---: |
| Production MATLAB | 70 files / 14,822 lines | 62 files / 14,256 lines | -8 files / -566 lines |
| Production and tests | 78 files / 18,291 lines | 70 files / 17,725 lines | -8 files / -566 lines |

The maintained examples, visible graphics, and complete repository suite were
not rerun because this cleanup changed ownership without changing planner
behavior. No `benchmark.csv` row was added because no maintained example
benchmark was executed. The first sandboxed MATLAB baseline launch failed
before startup with Windows `File system inconsistency`; the approved literal
launch completed the recorded 33-test baseline.

## Shared validation, timed-search, and test-contract cleanup — 2026-08-23

Task baseline: pushed `325-full-suite` commit `625b243`. The unrelated
untracked `docs/`, `sandbox/explainAzElPlannerWalkthrough.m`, and
`sandbox/explainSingleUQuinticWalkthrough.m` paths were preserved and excluded
from the change and static-analysis count.

This cleanup made `+azElInternal` the single owner of three additional exact
or parameterized invariants: seed-corridor Bernstein inequalities, polynomial
schema/dynamics/history validation, and time-expanded visibility search. The
polynomial validator accepts the method's range certificate as a callback, so
corridor retains exact stationary-point extrema while HS3 retains conservative
Bernstein bounds. The seed generators retain method-specific graph creation,
candidate ordering, diagnostics, and top-level policy. Twenty-four
behavior-identical planner contracts and seven fixture builders moved into
shared test support; method-specific tests remain in their original suites.

| Scope | Baseline physical / code | Current physical / code | Delta |
| --- | ---: | ---: | ---: |
| Production | 14,256 / 10,756 | 13,926 / 10,416 | -330 / -340 |
| Tests | 3,469 / 2,955 | 3,360 / 2,785 | -109 / -170 |
| Production and tests | 17,725 / 13,711 | 17,286 / 13,201 | -439 / -510 |

Verification on the retained worktree produced:

- focused post-extraction planner gates: 92/92 after each shared production
  or contract-suite change;
- complete regression: 132/132 passed in 245.517 seconds, compared with the
  pre-change 132/132 baseline in 174.860 seconds. This single wall-time increase
  is unfavorable but is not attributed to production behavior without a
  repeated controlled timing comparison;
- MATLAB Code Analyzer: zero messages across 102 intended MATLAB files;
- corridor maintained examples: 18/18 in 119.882 seconds, comprising 17
  independently validated successes and the expected validated
  `noValidatedSeed` failure;
- HS3 maintained examples: 18/18 in 304.914 seconds with the same 17-success,
  one-expected-failure split;
- every successful example reported collision freedom and passing velocity,
  acceleration, jerk, and dynamics certificates;
- visible obstacle-free smoke: success, independent validation, and three
  visible figures;
- hidden expected-failure smoke: independently validated `noValidatedSeed`,
  zero selected seed, and two diagnostic figures without rerunning planning;
- `git diff --check`: passed with Windows line-ending conversion warnings.

HS3 emitted extensive existing `fmincon` near-singular or singular-matrix
warnings during several maintained cases, notably the accelerating-circles,
moving-barrier, and no-path examples. Their returned outcomes still passed the
independent gates, but the warnings remain adverse numerical-robustness
evidence and are not suppressed. Two long inline MATLAB launches and one
earlier nested-Git static launch failed before executing governed work with the
recorded Windows `File system inconsistency` startup error; short literal
commands and the temporary serial runner completed. The temporary runner was
removed after the matrix.

The 36 fresh example rows were appended to `benchmark.csv` under
`625b243+dedup-worktree`. This checkpoint is a maintainability and deployment-
size improvement; no planner-runtime, completeness, optimality, or trajectory-
quality improvement is claimed.

## Compact corridor cutover — 2026-08-24

This checkpoint replaces the legacy corridor motion stack on branch
`325-full-suite` from worktree source `8111d0f+compact-cutover-worktree`.
Static straight requests retain the exact direct endpoint quintic. Every
obstacle-path request is assembled through `solveCompactC3Candidate` and uses
compact C3/C4 motion; no HS3 or nonlinear solve is invoked by the corridor
method.

### Size and dependency evidence

Final obstacle-path ownership is 1,023 nonblank, noncomment MATLAB lines, below
the user-authorized 1,200-line limit:

| File | Noncomment lines |
| --- | ---: |
| `solveCompactC3.m` | 563 |
| `solveCompactC3Candidate.m` | 129 |
| `runCorridorPlanner.m` | 260 |
| `buildFixedDurationAffineModel.m` | 37 |
| `expandRouteClearance.m` | 34 |

The final count is 33 lines above the earlier 990-line checkpoint because the
two duration brackets now share an explicit preparation cache instead of
rebuilding the same affine basis. Against committed `8111d0f`, the four deleted
legacy files contain 1,309 physical and 1,011 noncomment lines:
`solveCorridorQuintic`, `retimeDynamicRoute`, `optimizeExactTraversal`, and
`spanTimeDemand`. Repository-wide text inspection found no executable caller
of those names. Raw `solveCompactC3` has one production caller, the shared
candidate adapter used by the planner and both scaling benchmarks.

### Correctness and regression evidence

The first complete suite run found two failures: the U-case timing and stage-
accounting tests expected one affine-basis build but observed two. Diagnostics
showed that an infeasible 0.5 route bracket performed two trials, then a 0.8
bracket rebuilt the identical basis and succeeded. Removing the first bracket
changed a dynamic selected topology, so that experiment was reverted. The
retained fix caches preparation across both brackets. Both affected test files
then passed 16/16, and a fresh complete run passed 133/133.

The final maintained-example capture ran all cases serially in one MATLAB
process. It produced 17 independently validated successes and the expected
validated `noValidatedSeed` failure. Every successful duration met or beat the
frozen legacy duration:

| Example | Compact duration (s) | Frozen legacy (s) | Wall (s) |
| --- | ---: | ---: | ---: |
| Alternating slalom | 10.7822098011 | 10.8556642584 | 3.3593887 |
| Az/El planning | 7.64390306784 | 7.64965634404 | 0.9348455 |
| Dense concave | 8.68804103025 | 8.69057318299 | 0.6536946 |
| Forty moving circles | 60.3618388755 | 62.4777398626 | 5.9253491 |
| Four accelerating circles | 22 | 22 | 4.1871735 |
| Intercept at set time | 12 | 12 | 0.0741688 |
| Intercept earliest | 6.11153430176 | 6.11153430176 | 0.1952022 |
| Moving barrier wait | 10.2149013519 | 10.3713875625 | 3.2450860 |
| Moving circle, no wrap | 8.68792447418 | 8.75122873615 | 6.3141087 |
| Moving/deforming U.S. | 14.1949781492 | 19.2943308307 | 21.3253900 |
| Expected no path | `NaN` | `NaN` | 0.0834465 |
| Obstacle free | 4.53112887415 | 4.53112887415 | 0.0501164 |
| Opening U | 11.7332839966 | 11.7353786786 | 5.7116175 |
| Straight moving target | 20.8695652174 | 20.8695652174 | 2.0964864 |
| Target exits obstacle | 24 | 24 | 1.9774286 |
| Two opposing Us | 22.1109676862 | 22.1609457614 | 0.8391154 |
| Single U | 21.8327355422 | 22.640860107 | 0.8751825 |
| Extreme U.S. outline | 6.10504292559 | 6.22216662415 | 11.9142326 |

Complete polyline, smoothed-length, collision, kinematic-certificate, and wall-
time fields are appended to `benchmark.csv`; no metrics were copied from an
earlier run. The no-path row records planner success false and independent
example validation true.

A structurally different obstacle detour with nonzero initial and terminal
velocity and acceleration passed independent validation at 7.49992177607 s.
Endpoint errors were `7.68e-16` and `3.79e-14 deg/s` for velocity and
`1.22e-14` and `1.37e-12 deg/s^2` for acceleration. The maintained regression
is `testCompactDetourPreservesNonzeroEndpointStates`.

### Scaling, graphics, and static evidence

| Case | Compact duration (s) | Frozen legacy (s) | Candidate wall (s) |
| --- | ---: | ---: | ---: |
| 1 turn | 6.60420575985 | 7.36692262286 | 1.7638058 |
| 5 turns | 19.8905274829 | 29.9538760389 | 0.9634780 |
| 10 turns | 36.3238796555 | 41.8816850826 | 0.8148864 |
| 20 turns | 68.3588042743 | 83.4675614946 | 2.9578101 |
| 12 hairpins | 140.56091613 | 164.828287993 | 3.7061584 |

All five scaling results passed independent validation. The first combined
scaling command used the wrong hairpin report-field name after the four turn
cases had passed; the hairpin was rerun with its documented
`IndependentValidation` field and passed. MATLAB Code Analyzer checked 99
intended production, benchmark, example, and test files with zero messages.
A visible U-case smoke produced three figures and passed; a hidden expected-
failure smoke produced two diagnostic figures and no selected motion.

The compact duration and topology searches remain finite. This evidence proves
the maintained cases and synthetic scales only; it is not a completeness,
global-optimality, or uniform wall-time-speedup claim.

## Lean HS3 bounded composite — 2026-08-24

The standalone 3,868-noncomment-line HS3 planner was replaced by a bounded
composite with exactly 1,200 HS3-owned nonblank, noncomment MATLAB lines. The
count includes the HS3 facade, option resolver, improvement controller, and
remaining solver kernel. It excludes shared/public infrastructure and the
compact baseline that the composite intentionally invokes; it is not a claim
that the complete transitive execution closure fits in 1,200 lines.

The compact result is immutable unless an opt-in HS3 attempt independently
validates and is no later, has no greater integrated squared polynomial jerk,
and strictly improves at least one measure. The default is
`EnableHs3Improvement=false`, so normal HS3 calls return the validated compact
motion with composition diagnostics and without nonlinear-solver work. A
failed compact result may recover to any independently valid HS3 candidate.
Rejected candidates and cooperative time-limit overruns remain reported.
Requested mesh refinement is retained in diagnostics but explicitly reports
`RefinementSupported=false`.

### Final verification matrix

- MATLAB Code Analyzer checked 94 live `.m` files with zero findings.
- The complete test suite passed 138/138 in 205.211 seconds. New coverage
  includes exact default-off compact success/failure parity, nonzero start and
  terminal velocity/acceleration, moving-target terminal derivatives, timed
  waits, direct-facade recursion safety, optional-improver timing, and
  compact-failure recovery diagnostics.
- All 18 maintained examples ran serially in separate MATLAB processes for
  both `corridorQuintic` and `hs3`. All 36 example contracts passed. The 17
  successful pairs had identical arrival times and physical trajectories; the
  no-path pair returned the same `noValidatedSeed` failure and passed its
  expected-failure contract. The HS3 runs selected `corridorQuintic`, as
  required by the default-off bounded policy.
- A second serial HS3 capture produced the complete fresh rows appended to
  `benchmark.csv` under `hs3-compact-composite-worktree`; no compact metrics
  were copied into those rows.
- Graphics gates produced four valid U-shaped success figures with 680
  graphics objects and two expected-failure diagnostic figures with 342
  objects. The failure retained `noValidatedSeed` and selected seed zero.

### Scaling and hairpin comparison

Each public-method timing comparison used identical canonical inputs, seed
325, three interleaved repetitions, and exact comparison of the selected seed,
sampled position, velocity, acceleration, jerk, and arrival time.

| Case | Duration (s) | Compact median (s) | HS3 median (s) | HS3/compact | Compact max (s) | HS3 max (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 turn | 6.60420575985 | 0.5845021 | 0.4958138 | 0.848267 | 2.3496011 | 0.6624357 |
| 5 turns | 19.8905274829 | 0.3807402 | 0.3773811 | 0.991177 | 1.0238277 | 0.4704385 |
| 10 turns | 36.3238796555 | 0.7929697 | 0.7951018 | 1.002689 | 0.8257864 | 0.8282841 |
| 20 turns | 68.3588042743 | 3.1064024 | 3.0717906 | 0.988858 | 3.1310520 | 3.1094026 |
| 12 hairpins | 140.56091613 | 1.9999662 | 2.0402493 | 1.020142 | 2.5180664 | 2.2851960 |

All five cases succeeded, passed independent validation, and produced exact
compact/HS3 physical parity. The 10-turn median was 0.27 percent slower and the
hairpin median was 2.01 percent slower, both within the predeclared 5 percent
median allowance; these unfavorable observations are retained. The hairpin
HS3 maximum was lower in this three-repeat sample. Timing noise and finite
benchmark topology prevent a claim of uniform speedup, completeness, or global
optimality.

### Diff-growth disclosure

The existing `solveCompactC3.m` changes by +382/-58 lines against `8111d0f`.
Its added responsibilities are the production C3/C4 representations, endpoint-
derivative mapping, exact jerk accounting, safe-side QPs, and reusable affine
preparation needed to replace four solver paths. Keeping the compact solver as
an optional sidecar was rejected because it preserved duplicate production;
removing the first duration bracket was tested and reverted after it changed a
dynamic selected topology. The existing `runCorridorPlanner.m` changes by
+125/-491 lines: additions normalize compact/direct candidates and preserve
stable diagnostics while 491 lines of legacy recovery orchestration disappear.
Both files are covered by the 133-test suite, 18-example matrix, scaling gate,
nonzero-endpoint regression, Code Analyzer, and graphics smokes.

`verification.md` itself changes by +121/-1 lines to retain the evidence above,
while `plan.md` changes by +51/-61 and therefore shrinks overall. New source is
the 139-physical-line shared compact candidate adapter and the 41-physical-line
clearance helper; they centralize maintained planner/benchmark behavior rather
than moving legacy code between files.

## Standalone Hermite-Simpson restoration — 2026-08-24

This section supersedes the compact-composition checkpoint above for the
current worktree. `PlannerMethod="hs3"` dispatches directly to a standalone
Hermite-Simpson planner. HS3 does not call the compact/corridor planner, use a
compact result as a seed or fallback, merge method results, or emit composition
diagnostics. Compact remains a separate public method and appears below only as
an external frozen comparator.

The HS3 production package contains 1,602 nonblank, noncomment MATLAB lines
across nine files, below the 2,000-line limit. Exact affine sensitivity maps
replace decision-variable finite differences for fixed-time constraints and
most earliest-arrival derivatives. The public planner preserves nonzero
initial and final velocity and acceleration, uses neutral topology seeds,
bounded collision relinearization and mesh refinement, and accepts only motion
that passes the canonical independent validator.

### Final verification matrix

- Complete automated suite: 137 passed, 0 failed, 0 incomplete in 360.861356 s.
- Maintained examples: 18/18 scenario outcomes passed. This includes 17
  independently validated successes and the expected diagnosable no-path
  result. Every success reported `SelectedMotionSource="hs3"`.
- Focused regression examples: static U 29.484334 s, timed-opening U 20.101672
  s, and moving-barrier wait 53.121601 s; each passed its scenario validator.
- Exact-sensitivity tests: 6/6 passed, including nonzero endpoint derivatives,
  static and deforming obstacles, and event-knot timing.
- Code Analyzer reported zero messages for the changed dispatcher, HS3 sources,
  and neutral request helpers. `git diff --check` passed.

### Standalone scaling and hairpin gate

All measurements used the public HS3 dispatch with deterministic seed 325 and
were independently revalidated. The benchmark rejects compact composition.

| Case | Total wall (s) | HS3 arrival (s) | Frozen compact arrival (s) | Outcome |
| --- | ---: | ---: | ---: | --- |
| 1 turn | 3.776525 | 6.61105444584 | 6.60420575985 | pass; HS3 0.006849 s later |
| 5 turns | 7.563524 | 18.62266933 | 19.8905274829 | pass; HS3 earlier |
| 10 turns | 63.799021 | 33.5725360484 | 36.3238796555 | pass; HS3 earlier |
| 20 turns | 64.116174 | 114.561539749 | 68.3588042743 | pass; HS3 materially later |
| 12 hairpins | 93.339104 | 128.179676226 | 140.56091613 | pass; under 120 s and earlier |

The hairpin run produced a 50-vertex route, returned HS3-owned motion, and
passed a separately rerun validator. The 20-turn arrival is the clearest
remaining quality regression; the results do not establish global optimality,
completeness, or uniform superiority over compact.

### Remaining limitations

Static scenes stop after the first independently valid HS3 topology to protect
the wall-time budget, so another untried static topology may arrive earlier or
use less jerk. Timed topology proposals are solved at their input-derived fixed
arrival before comparison, which preserves their wait law but may miss a faster
solution on the same topology. The planning deadline is cooperative and can be
slightly overrun by solver setup or final validation. Several difficult
earliest-arrival solves still emit near-singular `fmincon` warnings even when
independent validation passes.

## Extreme deforming U.S. and moving-sun example — 2026-08-24

`exampleMovingDeformingUSOutlineVisibility` now uses two independently moving
obstacles. The U.S. outline is active from 0 through 240 s, grows smoothly from
8% to 135% scale, deforms during the interior of the history, completes an
actual 180-degree endpoint reversal, and is inactive at the 300 s mission end.
A 32-vertex starburst sun traverses the lower scene for the complete mission.
Scenario validation measured an initial/peak U.S. area ratio of
0.00351165980796, endpoint-geometry cosine -1, maximum scale 1.35, inactive
final U.S. geometry, 34.6708365607 degrees of sun-centroid travel, and maximum
sun boundary elevation 16.6 degrees, below the 18-degree route start.

Both public planner methods passed independent and scenario validation with
jerk constraints enabled. Compact used a 41.6645269891-degree polyline,
41.9569827325-degree smoothed motion, 9.14130766846 s duration, and 40.659661 s
wall time. Standalone HS3 used the same polyline, a 42.4287030058-degree
smoothed motion, 75 s duration, and 108.428247 s wall time. The longer HS3
duration and wall time are retained as unfavorable evidence. Code Analyzer
reported zero messages for both modified source files, the maintained
example-contract suite passed 5/5, and a plot-enabled run created three figures
with six axes while scenario validation remained passing.

The existing example changes by +109/-12 lines. Its net growth owns the second
obstacle, explicit scenario-history assertions, preserved result metadata, and
the local sun transform; the earlier script had no sun and could only validate
the planned trajectory, not the requested growth/rotation/disappearance
history. Keeping those assertions in a test-only duplicate was rejected because
the maintained example result must remain self-verifying. The private U.S.
helper changes by +39/-17 lines to centralize one transformation profile shared
by geometry generation and diagnostics. Both paths are covered by Code Analyzer,
the 5/5 contract suite, compact and HS3 headless runs, and the graphics smoke.

## Interactive export and randomized moving-polygon stress — 2026-08-24

Both interactive tabs now enable `Export Bundle` after a planner call. The MAT
file stores the versioned `azElSandboxDiagnosis-v1` bundle: raw and canonical
scene geometry, exact planner inputs and resolved options, retained segment
results, latest success or failure result, independent validation, sandbox log,
environment metadata, and reproduction commands. Graphics handles and
callbacks are deliberately excluded. `testAzElSandboxDiagnosisExport` passed
3/3 cases: successful round-trip and reproduction, preservation of an
`endpointBlocked` failure, and hidden-UI button state on both tabs. Code
Analyzer reported zero messages for the exporter, sandbox, and focused test.

`benchmarkRandomMovingPolygonStress` generated deterministic three-obstacle
scenes with 5-to-12-vertex polygons, source radii from 6.5 to 8 degrees,
35-degree cross-frame translations, and 180-to-360-degree rotations. Each case
also retained an independently calculated lower-bound clearance for a boundary
witness route. Compact passed 11/12 seeds (`1001:1012`); seed 1011 returned
`noValidatedSeed` despite a 2.40166-degree witness lower bound. Standalone HS3
passed the seven exercised seeds (`1001:1006` and `1011`) with independent
validation, including seed 1011 in 30.061148 seconds.

Seed 1011 is a compact motion-construction failure, not an input, obstacle,
topology, or physical-feasibility failure. The primary 10-point visibility
seed exists, HS3 solves the identical normalized request, and successful-exit
compact QP trials become collision-free when validated with only the workspace
bounds widened. The compact QP constrains workspace position at 257 samples
while minimizing jerk; its seed-anchored barrier drives the curve to the
-20-degree elevation boundary. Exact continuous validation finds between-
sample undershoot of 0.0000017 to 0.000642 degrees and rejects the motions
before collision certification. The reported collision-resolution and
azimuth-wrap failures are therefore downstream prerequisite symptoms. No
planner behavior was changed or tolerance weakened in this checkpoint.

Final-source verification passed 140/140 automated tests with zero failures or
incomplete tests. The maintained example matrix passed 18/18 for compact in
91.618892 seconds and 18/18 for standalone HS3 in 551.197924 seconds; each
method produced 17 independently validated successes plus the expected
validated no-path outcome. The changed/new MATLAB files had zero Code Analyzer
messages. The modified U.S. example's plot-enabled smoke created three figures
and six axes while retaining passing scenario validation, and
`git diff --check` passed.

## Unified obstacle construction owner — 2026-08-24

`makeAzElObstacleData` now owns all three canonical construction operations:
fresh static or sampled construction, normalization of one imported canonical
record, and absolute reinflation of canonical arrays or nested cells. The
separate `normalizeAzElTimeObstacleData.m` and `inflateAzElObstacleData.m`
implementations were removed. `combineAzElObstacles`, focused tests, public
documentation, and the safety-margin idempotence contract now call the single
owner. Established normalization and inflation error/warning identifiers were
preserved so malformed-input diagnostics did not change silently.

This is an ownership and file-count consolidation, not a source-size claim.
The three former owners contained 533 physical / 339 nonblank, noncomment
lines; the unified owner contains 576 / 464. Including the one-line caller
expansion in `combineAzElObstacles`, production changes by +44 physical / +126
noncomment lines while removing two public files. The added code is the
input-type dispatch and explicit local contracts needed to expose three
unambiguous call forms in one public function. Thin compatibility wrappers
were considered but rejected because the requested outcome was one owner and
all repository callers are migrated. The file remains below the 900-line
per-file limit.

Verification on the final source produced zero Code Analyzer messages for the
unified owner, combiner, and changed tests. The focused obstacle suite matched
its 5/5 baseline; the complete suite passed 140/140 in 337.935477 seconds.
The maintained examples passed 18/18 for compact in 91.419464 seconds and
18/18 for standalone HS3 in 549.982944 seconds. Each method retained 17
independently validated successes and the expected validated
`noValidatedSeed` outcome. Plot-enabled success and failure smokes produced
four figures/seven axes and two figures/two axes respectively, with both
scenario gates passing. No planner algorithm, obstacle geometry, margin,
tolerance, seed, or expected result changed, so no new benchmark row or
performance improvement is claimed.

## Corridor-quintic quality and runtime recovery — 2026-08-24

Two exported Rogue bundles isolated three general regressions in the compact
motion owner. First, refined exact-C3 routes doubled every interior knot and
could cross a dense-QP dimension cliff: one unselected 22-point route created
80 decisions and 2,142 barrier rows, then spent 65.0545 s in its motion solve.
Second, route refinement replaced span weights with equal weights even though
the resulting `173vs131` spans ranged from 2.801 to 28.599 degrees. That made
the equal-time peak velocity demand 629.189 versus 419.656 under geometric
allocation, a 49.93% inflation. Third, six earliest-arrival trials could leave
a multi-second feasible/infeasible bracket despite the public 0.001 s arrival
tolerance.

The retained input-driven rules keep doubled-knot exact C3 only through 48
decision variables, use continuous-C4 exact motion above that bound, allocate
refined span time from actual edge lengths, and use 14 trials for exact
earliest-arrival searches. Obstacle geometry, safety margin, limits, topology
opportunities, public options, independent validation, and HS3 were unchanged.
The saved `az_el_sandbox_goal_20260824_174716.mat` trajectory is bit-for-bit
unchanged in time, position, velocity, acceleration, and jerk at 37.845175 s,
while wall time fell from 77.9230141 to 9.2077994 s (88.18%). The formerly
dominant long seed now uses 40-decision C4, completes its motion work in 2.697 s,
and remains correctly unselected.

For `173vs131.mat`, the saved compact result arrived at 173.25 s with a
370.353-degree smoothed path. Fresh standalone HS3 arrived at 131.642423799 s
with a 319.454-degree path and 0.46758 jerk-squared. The retained compact rules
arrived at 136.042437744 s in 8.1911 s wall time, with a 311.101948-degree path,
0.02-degree clearance, and 0.0957002 jerk-squared. They recover 37.2076 s of
the 41.6076 s arrival regression, but compact remains 4.400014 s (3.34%) later
than HS3 on this case. A larger C3 map, pure length weighting, and axis-demand
weighting were measured and rejected because they either retained the timing
defect or worsened path length, jerk, or wall time.

The distinct 12-hairpin scale case passed independent validation with 96 C4
decisions, 138.455023011 s arrival, 8.2178 s wall time, and 0.02-degree
clearance; its prior frozen duration was 140.560916 s. All 18 maintained
compact example contracts passed serially: 17 independently valid successes
and the expected independently valid `noValidatedSeed` outcome. Plot-enabled
success produced three figures/six axes; the expected failure produced two
figures/two axes without rerunning planning. The complete automated suite
passed 144/144 in 643.151742 s wall time (632.433713 s summed test duration).
Code Analyzer reported zero messages for the changed production owner and its
focused regression test, and `git diff --check` found no whitespace errors.
HS3 moving-target tests still emit existing near-singular `fmincon` warnings;
those unfavorable diagnostics did not fail validation and were not suppressed.

Before push, the six reviewed task files were applied by themselves to a
detached `da52da8` worktree so unrelated dirty corridor, HS3, sandbox, and test
changes could not influence the gate. Code Analyzer again reported zero
messages, the focused planner suites passed 59/59, and the complete isolated
suite passed 142/142 with zero failures or incomplete tests in 709.815981 s
wall time (697.739404 s summed duration). The isolated count is two below the
dirty-worktree count because preexisting uncommitted tests were deliberately
excluded. The same expected HS3 near-singular warnings remained visible.

The cumulative dirty-worktree diff for `solveCompactC3.m` is +53/-12 lines;
reliable history cannot separate all earlier user edits from this task. The
681-line file now also owns the representation bound, geometric span
allocation, and bounded duration-search count because all three govern the
same compact exact-motion construction. Splitting those constants into a new
owner would add an interface without removing responsibility. The retained
alternatives and the two supplied bundles, focused regression, hairpin scale
case, full example matrix, graphics smokes, Code Analyzer, and full suite cover
that growth; the file remains below the 900-line limit.

## HS3-only production cutover — 2026-08-25

At source commit `67bc087` on branch `HS3-planner`, the public dispatcher was
reduced to HS3 and the complete corridor-quintic implementation and its
method-specific tests, benchmarks, and spline artifacts were removed. Active
source and documentation contain no references to `corridorQuintic`,
`azElPlannerMethods.corridor`, `solveCompactC3`, `compact planner`, or `quintic
planner`; historical benchmark and verification records remain unchanged.

Code Analyzer reported zero messages across all 83 remaining MATLAB files. A
direct HS3 planning request succeeded, passed independent validation, returned
`SelectedMotionSource = "hs3"` and `PlannerMethod = "hs3"`, and produced a
five-second trajectory. The focused suites passed 67/67 in 388.369877 seconds:
`testHs3OptionOwner`, `testHs3AffineSensitivity`, `testHs3Planner`,
`testPlannerStageTiming`, and `testExampleContracts`. Existing near-singular
`fmincon` warnings were visible in moving-barrier and moving-target coverage.

The maintained examples were then launched serially in separate MATLAB
processes, but all attempts failed during MATLAB startup before example code
ran with `System Error: File system inconsistency`. A single-example retry
failed identically. Consequently no fresh example metrics or benchmark rows
were recorded, and the visible-success and expected-failure diagnostic-figure
checks remain untested in this environment.

A later single MATLAB process successfully ran the complete post-cutover test
suite: 75/75 passed, with zero failures or incomplete tests and 366.849286
seconds summed test duration. The same near-singular HS3 `fmincon` warnings were
visible and were not suppressed.

## Sandbox export recovery — 2026-08-25

The bundle writer successfully created and reloaded a 318,976-byte MAT file
from a real guidata-backed sandbox state, establishing that bundle assembly and
the core save operation were healthy. Two subsequent visible user runs showed
that converting only the success notification was insufficient. The identical
`Cell elements must be character arrays` error occurs at the earlier
`uiputfile` filter boundary because its cell elements were MATLAB strings.

The public sandbox snapshot now exposes
`ExportBundle(filePath, modeName)`, the UI button calls the same explicit-path
owner after its dialog returns, and the writer verifies both a nonempty file
and the required `diagnosisBundle` MAT variable. UI failures display their
actual message, identifier, and first source location in an error dialog. The
file-dialog filter, success notification, `save`, `whos -file`, `version`, and
`datetime` calls now receive character arguments for cross-version
compatibility. The focused export suite passed 4/4 and Code Analyzer reported
zero messages before this visible-only compatibility correction; concurrent
MATLAB startup failed before a final focused or visible callback rerun.

Pre-run export now prepares a copy of the live mode state by reading current
controls and rebuilding canonical obstacle geometry without calling the
planner. Complete Goal Mode scenes retain exact replayable planner inputs;
partial Goal Mode and Free Mode scenes retain their explicit request geometry
and controls. Pre-run bundles use `PlanningState = "notRun"`,
`HasPlannerResult = false`, an empty `Result`, and no validation claim. A new
focused test exercises the public no-dialog export before any planner call.
The expanded export suite passed 5/5 with zero failures or incomplete tests,
and Code Analyzer reported zero messages across the two production files and
focused test.

## Dynamics-timescale mesh verification — 2026-08-26

Changed `+azElPlannerMethods/+hs3/plan.m` so an untimed spatial detour with
more than two route legs starts at twice the configured HS3 mesh only when its
estimated duration per base segment exceeds twice the supplied
`maxVelocity_deg_s ./ maxAcceleration_deg_s2` time scale. Direct and timed
seeds retain the configured mesh, and the existing maximum segment count still
bounds the result. Code Analyzer reported zero findings and the complete HS3
package remains exactly 2,000 noncomment production lines.

Focused A/B evidence:

- 40 moving circles: 61.2011842765 -> 58.6189853057 s arrival;
  smoothed length 125.185941203 -> 123.380530717 degrees; current wall
  18.873958 -> 24.875451 s; independent validation and certificates pass.
- Rogue 180/360 horizons: 86.5467293065 and 86.5467226767 s, both valid,
  differing by 0.000006629817 s. The preceding commit reported
  88.2939404925 and 88.2939359679 s.
- Static U control: unchanged at 22.6308876389 s. Timed moving-circle
  control: unchanged at 8.64603156476 s.
- Rejected blanket-20 diagnostics: static U regressed to 22.6623174130 s;
  moving circle improved to 8.560546875 s but wall rose to 16.976155 s.

Every maintained example was then run headlessly in its own serial MATLAB
process. All 18 expected outcomes passed: 17 independently validated successes
and the independently validated `noValidatedSeed` case. Exact metrics and wall
times are appended to `benchmark.csv`. A visible `exampleAzElPlanning` run
created three figures and 526 graphics objects; a hidden plotted no-path run
created two diagnostic figures with nine rejected edges and no trajectory.

`testHs3Planner` passed 51/51 in 43.414460 seconds. The authoritative complete
suite passed 82/82 in 50.011338 seconds with warnings enabled. An earlier
81/82 diagnostic run is invalid as a suite result because its harness disabled
all warnings, preventing the required unknown-option warning from reaching
`verifyWarning`; no repository assertion failed in that run. `git diff
--check` reports only existing LF-to-CRLF conversion notices.

## Direct dynamics-mesh jump — 2026-08-26

The pushed dynamics-timescale rule reached its best long-detour results by
starting at 20 segments and then refining to 40. The retained follow-up keeps
the ordinary configured 10-segment first solve and, only when the same
input-derived long untimed multi-leg predicate is true, makes its single
quality pass jump directly by 4x. Other candidates retain the ordinary 2x
refinement. This removes a redundant intermediate transcription without a new
option, scenario identifier, obstacle property, seed, tolerance, or extra
production line; the complete HS3 package remains exactly 2,000 noncomment
lines.

Focused evidence preserves the 40-circle result exactly at 58.6189853057
seconds arrival, 110.807922148-degree polyline, and 123.380530717-degree
smoothed motion. Final serial wall time was 22.727164 seconds versus 24.875451
seconds in the pushed 20-to-40 verification. A focused repeat took 22.398936
seconds. The structurally distinct neutral-circle regression improved from
80.2105179472 seconds at 20 segments to 78.7444420156 seconds at 40 segments;
its wall time rose from 4.140629 to 7.139318 seconds. The regression now
requires one 40-segment quality pass and arrival below 79 seconds.

A direct 10-to-30 alternative was measured and rejected despite its lower
19.113692-second 40-circle wall time: arrival regressed to 60.1588345587
seconds and smoothed motion grew to 124.541423742 degrees. The retained 40-
segment result therefore does not trade away the arrival improvement for the
lower runtime. No uniform speedup is claimed.

Both rogue horizons reproduce on seed 2 and 40 segments. `failure.mat` at the
180-second horizon reaches 86.5467293065 seconds in 21.522385 seconds wall;
`successwhenincreasehorizon.mat` at 360 seconds reaches 86.5467226767 seconds
in 20.076234 seconds wall. Their 6.630-microsecond arrival difference preserves
the repaired horizon invariance, and both pass independent collision and all
derivative certificates.

All 18 maintained examples ran serially in fresh MATLAB processes. Seventeen
successes and the expected `noValidatedSeed` outcome independently validate;
their exact metrics are appended to `benchmark.csv`. The moving/deforming U.S.
case remains the dominant wall-time weakness at 49.573514 seconds. A visible
success produced three figures and 526 objects. A corrected failure plot probe
produced two figures, 341 objects, and nine rejected edges; an earlier reporter
queried the wrong diagnostics nesting after the valid example and is retained
as a `NaN`-wall benchmark row rather than hidden.

The focused long-detour regression passed in 7.1857 seconds,
`testHs3Planner` passed 51/51 in 42.668797 seconds, and the warnings-enabled
complete suite passed 82/82 in 49.490930 seconds. Code Analyzer reported zero
findings across 84 MATLAB files. `git diff --check` reports only existing
LF-to-CRLF notices. Two user-owned MATLAB processes, PIDs 8516 and 31968,
remained alive and were never signaled.

## Deforming-outline stage diagnosis — 2026-08-26

An end-to-end profile at local commit `6427ce9` separated the 51.619861-second
moving/deforming U.S. example into 20.669104 seconds of scenario construction
and 26.1176 seconds of planning. Construction spent 16.556995 seconds in 60
`polybuffer` calls. Planning reported 8.3759 seconds topology, 3.4312 seconds
corridor construction, 11.0734 seconds motion solving, 1.0686 seconds
collision checking, and 0.3503 seconds final validation. This establishes that
the largest remaining runtime is shared between exact obstacle construction
and planning rather than being solely an HS3 solver issue.

A boundary-classification opt-out reduced an isolated 14,000-vertex geometry
microbenchmark from 0.898543 to 0.386355 seconds with identical coordinates,
but failed the end-to-end retention gate. Its profiled example took 52.626923
seconds versus 51.619861 seconds, and a fresh run's 49.326170 seconds was only
0.247344 seconds below the preceding 49.573514-second matrix row. The extra
interface was reverted. The focused obstacle suite remained 6/6, and no
production code from this diagnostic is retained.

Exact buffer reuse is already active for the translated sun. It cannot be
applied to the U.S. history because every interior slice independently changes
scale and applies coordinate-coupled nonlinear deformation before rotation.
Exact duplicate and collinear removal was previously measured as negligible,
and vectorized buffering improved only about 1.5%. Geometry reduction,
coarser history, parallel-toolbox dependence, and affine reuse were therefore
not presented as safe runtime improvements. The 49-second unfavorable runtime
remains explicit.

## Obstacle-free bounded fixed-time search — 2026-08-26

### Retention gate and implementation

The baseline was local commit `6427ce9` on `HS3-planner`, with only the
documented runtime-profile records dirty. The primary metric was independently
validated obstacle-free earliest arrival; collision, workspace, endpoint,
velocity, acceleration, jerk, stable API, diagnostic schema, package size, and
representative runtime were hard invariants. The retained change reuses the
existing bounded fixed-arrival feasibility search for an obstacle-free
earliest-arrival request and begins that search at twice the configured
collocation mesh, still capped by `MaximumCollocationSegmentCount`. The
condition is determined only by empty obstacle input and goal-time policy.
Nonempty-obstacle and fixed-arrival paths do not enter it.

The maintained example improves from 4.60777936881 to 4.5458984375 seconds,
0.06188093131 seconds or 1.343% earlier. Final wall time improves from
3.882538 to 2.956477 seconds, a 23.85% reduction. A distinct direct request
from `[-3, 1]` to `[5, -2]` with asymmetric velocity, acceleration, and jerk
limits independently validates at 5.70751953125 seconds on 20 segments using
the linear fixed-time constraint representation. The former local
`topologyAlignedSegmentCount` helper was removed and the bounded expression
was kept in the owning execution sequence. The complete nine-file HS3 package
remains exactly 2,000 nonblank, noncomment lines.

An isolated detached worktree at `67bc087` supplied the requested
325-full-suite comparison. Its obstacle-free example reached 4.53112887415
seconds in 4.588076 seconds wall; current remains 0.01476956335 seconds later
but runs 1.631599 seconds faster. Its identical wide-U input reached
21.8327355422 seconds in 6.057014 seconds, versus current 22.6308876389 seconds.
Current fixed-time attempts at the 325 duration remained collision-unresolved,
including a solve seeded from the 325 trajectory. An 80-segment current solve
reached only 22.5797733 seconds and raised wall time from 13.019699 to
18.815125 seconds. Warm-start, doubled corridor sampling, and doubled reserve
experiments were neutral or regressive and were fully reverted. These results
do not support broadening the obstacle-free rule to obstacle cases.

### Final serial maintained matrix

Every maintained example ran in its own fresh MATLAB process with figures and
animation disabled. Jerk was enabled in every row.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.060439635 | 16.7100287566 | 11.1855739606 | 1 / 1 | 7.340792 | `goalReached` |
| `exampleAzElPlanning` | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 4.887374 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.618792 | `goalReached` |
| `exampleFortyMovingCircleGrid` | 1 / 1 | 110.807922148 | 123.380530717 | 58.6189853057 | 1 / 1 | 22.624950 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 24.3633026735 | 27.8702009821 | 22 | 1 / 1 | 25.939702 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 2.334870 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 5.975042 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 13.101632 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 9.562227 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 41.5785140688 | 40.7424283094 | 8.75061035156 | 1 / 1 | 50.142665 | `goalReached` |
| `exampleNoPathAzElMotion` | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.434626 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | 1 / 1 | 4.472135955 | 4.47285938216 | 4.5458984375 | 1 / 1 | 2.956477 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 | 16.487221 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 21.4031702791 | 14.2200520815 | 20.8695652174 | 1 / 1 | 9.071585 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1815464898 | 21.3509241121 | 24 | 1 / 1 | 8.791014 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.5077116377 | 24.4201122273 | 21.9090824092 | 1 / 1 | 12.949268 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 16.447739 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 48.172937 | `goalReached` |

The 40-circle, static-U, moving-circle, moving/deforming-U.S., and extreme-U.S.
arrival values match the preceding committed behavior. The final matrix has 17
independently validated successes plus the independently validated expected
`noValidatedSeed` result. Existing near-singular solver warnings remained
visible on timed cases.

### Rogue, graphics, tests, and static gates

With seed 2 and a 40-segment cap, `failure.mat` at a 180-second horizon reaches
86.5467293065 seconds in 20.844898 seconds wall. The 360-second
`successwhenincreasehorizon.mat` reaches 86.5467226767 seconds in 20.803370
seconds wall. Both independently pass collision and every derivative
certificate; their arrival difference is 6.630 microseconds.

A visible `exampleAzElPlanning` run produced three figures, six axes, and 530
graphics objects. A hidden plotted expected failure produced two diagnostic
figures, two axes, 344 objects, and no trajectory. The warnings-enabled
complete suite passed 83/83 with zero failures or incomplete tests in
51.100108 seconds wall and 44.390953 seconds summed duration. Code Analyzer
reported zero findings across all 84 MATLAB files. `git diff --check` reported
only the existing LF-to-CRLF notices. User-owned MATLAB PIDs 8516 and 31968
remained alive and were never signaled. A final two-file Code Analyzer retry
passed with zero findings after one preceding MATLAB process failed during
startup with `System Error: File system inconsistency`; that failed process
did not execute repository code.
