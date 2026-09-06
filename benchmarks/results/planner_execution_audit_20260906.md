# Planner execution and data-flow cleanup — 2026-09-06

Baseline: `3877017`, branch `bmtp-cleanup-codex`; implementation is the current
uncommitted cleanup. MATLAB R2024b. The audit froze source, resolved requests,
and results before editing. User-owned MAT/ASV files were preserved.

## Decisions and fixes

| Finding | Disposition and evidence |
| --- | --- |
| Confusing stage names and timing aliases | Separate empty records, route conversion, solving, and recovery: `createEmptyPathGuess`, `createPathGuesses`, `createRoutePathGuesses`, `solvePathGuess`, `tryAdditionalPathGuesses`. Name direct/fixed-time attempts, stationary enclosures, fixed solver endpoints, and output assembly explicitly. Split timing creation, reconciliation, and attachment. Candidates and summaries use `ArrivalTime_s`/`TrajectoryDuration_s`; polynomial segment metadata remains separate. |
| Empty polling blocks, unused helper/assignment, discarded direct candidate and recovery guess | Removed. Recovery converts only searched routes. Default diagnostics initialize once on each executed path. MATLAB analysis reports no findings in obstacle avoidance or BMTP. |
| Duplicated validation and unused returned geometry | Remove the separate candidate check-results array and discarded refinement planes/pairs. Keep validation with its motion and summary. Refinement still uses those planes internally and retains active-pair counts. |
| Repeated request-invariant geometry | Reuse static regions and moving-history enclosures across guesses. Static U and Philippines each reduced solver decomposition/grouping from two calls to one. Rebuild only stale obstacle preparations; editing one of two obstacles rebuilt one and exactly matched fresh preparation. Independent validation still checks original protected histories. |
| Optional work without a consumer | Occupancy computes blocking IDs/clearance metadata only when requested; polynomial evaluation omits unrequested derivatives. All requested outputs match full evaluation. Profiled polynomial-record evaluations fell 252→223 (static U), 416→360 (Philippines), and 2288→815 (moving barrier). |
| Misleading dynamic eligibility | Dispatch recognizes wait-then-move guesses before calling that constructor. A two-point dynamic guess is reported as `unsupportedDynamicDirectGuess`, rather than a multi-waypoint failure. Seed limits, ordering, and explicit fallback policy are unchanged. |
| Discarded useful diagnostics | Keep actual candidate ranking, visibility-attempt rejection/connectivity evidence, both timed and spatial partial routes, timed certificate rejection reasons, and only executed timed trials. Pair trial validation with the correct candidate. Remove constant/duplicate fields such as the unused corridor timer and `LayerLimitApplied`. |
| Rejected motion overwritten or dropped | Retain a constructed rejected motion instead of replacing it with an empty eligibility record. On planning failure, expose the retained motion and its failed validation in the existing result fields; keep `Success=false` and `Route_deg` empty. Cases with no constructed motion remain empty. Explicit fallback regression verifies this distinction. |
| Scalar repetition in envelope certificates | Fix row/column expansion that returned vector clearance or failed to assemble segment/region indices. New regressions cover one/two regions and linear/cubic polynomials. |
| Redundant intercept records and ambiguous path parameters | Stop constructing per-trial intercept records that callers discarded. `ParameterBasis` distinguishes normalized time from normalized distance; `ObstacleEnvelope_deg` names the guess's obstacle outline. |

All above findings were observed through source/data-flow inspection, execution,
or regressions. No collision tolerance, margin, search horizon, seed quota,
physical limit, or optimization objective was weakened.

## Cold paths retained

The 18 default examples alone never entered timed BMTP, the stationary-enclosure
adapter, or the waypoint fallback. That does **not** make those paths dead:
focused tests exercised them, including accepted motions/construction and failed
validation. Tests also exercised later-guess recovery, deferred timed search, and
exact-region retry after conservative grouping failed. Supplemental probes
exercised growing graph storage (69 states, 12 routes), caller-supplied envelope
certificates, specified-time target derivative extraction, and hidden graphics.
Nonzero derivative feasibility and exhaustive branch coverage are not established.

The direct physical motion and direct optimizer guess also remain distinct:
one constructs a timed motion immediately; the other initializes an obstacle-aware
curve solve. A lack of default-example hits is now explicitly separated from
unreachable code in the repository-health skill.

## Matched measurements

Saved requests were identical. One first call plus three warmed calls per case
ran in separate before/after MATLAB processes; profiling ran separately. All
routes, polynomial coefficients, sampled positions, and arrival times matched
exactly; independent validation passed every comparison.

| Saved request | First call before / after (s) | Warm median before / after (s) |
| --- | ---: | ---: |
| Static U | 6.645 / 6.746 | 4.196 / 4.223 |
| Philippines | 5.331 / 5.248 | 4.350 / 4.255 |
| Moving barrier | 0.917 / 0.937 | 0.242 / 0.244 |

Only the first case starts a fresh MATLAB process. These timings do not establish
a speedup; retain the cleanup for removed duplicate work and corrected ownership.
The earlier instrumented audit measured only about 0.73 s in solver decomposition
across its example run set, so it was not the dominant overall bottleneck.

## Verification

Follow-up consolidation moved timing attachment into `planTrajectory` and fixed
endpoint adaptation into the static solver adapter. Concise help text retains
the required interface documentation. Two standalone helper files were removed:
this pass is +79/-218 production MATLAB lines (net -139); the complete cleanup
versus `3877017` is +646/-677 (net -31), including comments and blank lines.
All 49 focused tests passed. A fixed-target fixture exercised the static adapter
and matched the equivalent fixed endpoint motion exactly; a saved static-U request
also matched its prior motion exactly. Both passed independent validation.
No maintained examples were rerun in this consolidation pass.

- Initial audit: 18 profiled default finite-jerk examples and 67 focused MATLAB
  tests, plus supplemental coverage probes.
- Cleanup: 143 distinct MATLAB tests passed across the broad run and focused
  corrections; 24 Node sandbox tests passed. Three tests that directly invoke
  maintained examples were omitted from the broad runner; the separate complete
  example matrix covers their default scenarios, but not their exact assertions
  or the convergence test's alternate clearance option.
- All 18 maintained default finite-jerk examples matched frozen inputs, routes,
  polynomials, positions, and arrivals exactly: 17 independently valid motions
  and the expected `noValidatedSeed` example. The final failure-output change was
  followed by 14 focused output/fallback/graphics checks, two empty-motion failure
  contracts, and an additional unchanged no-path example run.
- Successful example rows passed collision, kinematic, and applicable certificate
  checks. No-path numeric metrics remain NaN. Full precision and all earlier
  benchmark history remain in `benchmark.csv`.
- Initial gates exposed certificate dimension defects and stale owner/schema
  assertions; these were corrected. A stronger failure test exposed final output
  dropping rejected motion; the explicit-fallback case now verifies retention.
- MATLAB code analysis, diff whitespace checks, and skill validation pass.
  Documentation exporters were updated for renamed fields; PDFs were not rebuilt.

Raw frozen sources, profiles, comparison runs, unsuccessful runner logs, and
scratch probes remain under ignored `tmp/planner-audit-20260906/`. No production
profiling hooks were added. Manual browser interaction was not repeated. This is
bounded test evidence, not a claim of exhaustive reachability or optimality.
