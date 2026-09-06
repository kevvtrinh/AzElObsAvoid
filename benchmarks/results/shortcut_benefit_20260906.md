# Fixed-time detour shortcut: on/off comparison

Measured 2026-09-06 in MATLAB R2024b Update 4. Frozen source commit `63217438676e5c83fe5dde3ef5f5b4e5425c6799` plus the recorded working changes. No production planner changes.

## Method

22 newly generated cases (seeds 6101–6120 plus empty and sealed controls), and exact input replays of two maintained examples. Five cases each cover static rectangles, translating/rotating rectangles, equal-axis diagonal travel, and alternating barriers. This deliberately mixed small suite is not a random sample of real workloads; its percentages are specific to this suite.

The off copy bypasses only the early fixed-time lateral detour. The direct-motion check, later route search, solver, physical limits, protected geometry, and safety tolerances remain enabled and identical. The off package is mechanically renamed to keep both versions loaded without clearing MATLAB caches. Both share the same frozen motion engine. All completed outputs receive a fresh check through the unchanged public validator.

One warm-up and three timed repeats per case/mode, alternating mode order. Reported time is the median full planner-call wall time; input generation, MATLAB startup, source loading/path setup, and the extra independent check are excluded. The planner’s own validation is included. First-run timings and all repeats are retained in the CSV. A material timing change must exceed both 20% and 0.05 seconds. Movement-time differences use 0.001 seconds. Neither timing nor path length substitutes for safety.

Each planner call has an external 60-second limit. Timeout stops only the benchmark-owned MATLAB process; later repeats for that case/mode are skipped and the worker restarts. Timeout means unresolved within this budget, not proven impossible. No failed cases are regenerated. New runs after a restart may be cold; their warm-up results are not used for median comparisons.

## Finding

On this suite, the shortcut improves the outcome in 10/22 new cases: nine shorter validated movements and one additional validated solution. It materially reduces calculation time in 2/22 and increases it in 8/22 (including the additional-solution case). Seven of the nine shorter movements improve by only about 0.05–0.17 seconds; two improve from 24 to 7.708 seconds. No movement becomes longer and no previously successful case is lost. The five static cases and five moving cases account for all ten new-case outcome gains; diagonal, weaving, and control cases show no outcome gain.

Accept the hypothesis of usefulness for some new simple-detour cases. Reject the broad claim that it generally speeds up planning. Leave production behavior unchanged: the user requested measurement, and this tradeoff does not justify a blanket removal or an unmeasured new rule. The current earliest-arrival objective makes movement time relevant even when computing the movement takes longer.

All 192 calls completed, with no exceptions, timeouts, or worker restarts. Every claimed success passed fresh independent validation; physical outcomes and metrics matched exactly across each group's four runs. Both versions failed the sealed-wall control; the off version additionally returned `noValidatedSeed` for moving_6110. Search failure is not a proof of infeasibility.

## Per-case results

All times are seconds. “On/off valid” means planner success AND fresh independent validation. Timing improvements/regressions are compared only where three timed repeats completed on both sides.

| Case | On/off valid | Planning on | Planning off | Motion on | Motion off | Shortcut selected |
|---|---|---:|---:|---:|---:|---|
| static_6101 | True/True | 3.473 | 1.160 | 7.708 | 7.757 | True |
| static_6102 | True/True | 2.556 | 0.682 | 7.708 | 7.793 | True |
| static_6103 | True/True | 2.048 | 0.493 | 7.708 | 7.793 | True |
| static_6104 | True/True | 2.117 | 0.663 | 7.708 | 7.797 | True |
| static_6105 | True/True | 3.724 | 1.148 | 7.708 | 7.757 | True |
| moving_6106 | True/True | 2.308 | 0.917 | 7.708 | 7.880 | True |
| moving_6107 | True/True | 4.360 | 0.923 | 7.708 | 7.879 | True |
| moving_6108 | True/True | 2.278 | 3.540 | 7.708 | 24.000 | True |
| moving_6109 | True/True | 2.414 | 4.354 | 7.708 | 24.000 | True |
| moving_6110 | True/False | 6.933 | 4.899 | 7.708 | — | True |
| diagonal_6111 | True/True | 0.654 | 0.579 | 6.826 | 6.826 | False |
| diagonal_6112 | True/True | 0.586 | 0.553 | 7.095 | 7.095 | False |
| diagonal_6113 | True/True | 0.699 | 0.661 | 7.174 | 7.174 | False |
| diagonal_6114 | True/True | 0.578 | 0.501 | 7.094 | 7.094 | False |
| diagonal_6115 | True/True | 0.562 | 0.613 | 7.269 | 7.269 | False |
| weave_6116 | True/True | 4.060 | 3.857 | 10.232 | 10.232 | False |
| weave_6117 | True/True | 3.134 | 3.917 | 9.924 | 9.924 | False |
| weave_6118 | True/True | 4.095 | 3.921 | 10.045 | 10.045 | False |
| weave_6119 | True/True | 3.231 | 3.142 | 10.127 | 10.127 | False |
| weave_6120 | True/True | 3.640 | 3.285 | 10.131 | 10.131 | False |
| empty_control | True/True | 0.022 | 0.021 | 7.708 | 7.708 | False |
| sealed_control | False/False | 0.189 | 0.145 | — | — | False |
| exampleMovingRotatingObstacleField | True/True | 2.148 | 1.109 | 9.042 | 9.261 | True |
| exampleMovingBarrierWait | True/True | 0.426 | 0.325 | 10.090 | 10.090 | False |

## Counts

- New cases (22): independently valid with shortcut 21/22; without 20/22.
- New cases (22): shortcut selected in 10; materially faster planning in 2; materially slower planning in 8; shorter validated movement in 9; longer in 0; gained valid outcomes 1; lost valid outcomes 0.
- All cases (24): independently valid with shortcut 23/24; without 22/24.
- All cases (24): shortcut selected in 11; materially faster planning in 2; materially slower planning in 10; shorter validated movement in 10; longer in 0; gained valid outcomes 1; lost valid outcomes 0.

## Movement and validation details

| Case | Mode | Polyline (deg) | Smoothed (deg) | Duration (s) | Planner | Independent | Collision | Kinematic | Certificate | Termination |
|---|---|---:|---:|---:|---|---|---|---|---|---|
| static_6101 | on | 16.213 | 16.213 | 7.708 | True | True | True | True | True | goalReached |
| static_6101 | off | 16.207 | 16.862 | 7.757 | True | True | True | True | True | goalReached |
| static_6102 | on | 16.584 | 16.584 | 7.708 | True | True | True | True | True | goalReached |
| static_6102 | off | 16.549 | 17.264 | 7.793 | True | True | True | True | True | goalReached |
| static_6103 | on | 16.230 | 16.230 | 7.708 | True | True | True | True | True | goalReached |
| static_6103 | off | 16.222 | 16.546 | 7.793 | True | True | True | True | True | goalReached |
| static_6104 | on | 16.277 | 16.277 | 7.708 | True | True | True | True | True | goalReached |
| static_6104 | off | 16.269 | 16.777 | 7.797 | True | True | True | True | True | goalReached |
| static_6105 | on | 16.312 | 16.312 | 7.708 | True | True | True | True | True | goalReached |
| static_6105 | off | 16.301 | 16.967 | 7.757 | True | True | True | True | True | goalReached |
| moving_6106 | on | 16.498 | 16.498 | 7.708 | True | True | True | True | True | goalReached |
| moving_6106 | off | 17.120 | 17.965 | 7.880 | True | True | True | True | True | goalReached |
| moving_6107 | on | 16.143 | 16.143 | 7.708 | True | True | True | True | True | goalReached |
| moving_6107 | off | 16.856 | 17.932 | 7.879 | True | True | True | True | True | goalReached |
| moving_6108 | on | 16.121 | 16.121 | 7.708 | True | True | True | True | True | goalReached |
| moving_6108 | off | 16.272 | 16.293 | 24.000 | True | True | True | True | True | goalReached |
| moving_6109 | on | 16.115 | 16.115 | 7.708 | True | True | True | True | True | goalReached |
| moving_6109 | off | 16.295 | 16.892 | 24.000 | True | True | True | True | True | goalReached |
| moving_6110 | on | 16.311 | 16.311 | 7.708 | True | True | True | True | True | goalReached |
| moving_6110 | off | — | — | — | False | False | False | False | False | noValidatedSeed |
| diagonal_6111 | on | 17.050 | 17.107 | 6.826 | True | True | True | True | True | goalReached |
| diagonal_6111 | off | 17.050 | 17.107 | 6.826 | True | True | True | True | True | goalReached |
| diagonal_6112 | on | 17.193 | 17.473 | 7.095 | True | True | True | True | True | goalReached |
| diagonal_6112 | off | 17.193 | 17.473 | 7.095 | True | True | True | True | True | goalReached |
| diagonal_6113 | on | 17.250 | 17.562 | 7.174 | True | True | True | True | True | goalReached |
| diagonal_6113 | off | 17.250 | 17.562 | 7.174 | True | True | True | True | True | goalReached |
| diagonal_6114 | on | 17.193 | 17.590 | 7.094 | True | True | True | True | True | goalReached |
| diagonal_6114 | off | 17.193 | 17.590 | 7.094 | True | True | True | True | True | goalReached |
| diagonal_6115 | on | 17.323 | 17.750 | 7.269 | True | True | True | True | True | goalReached |
| diagonal_6115 | off | 17.323 | 17.750 | 7.269 | True | True | True | True | True | goalReached |
| weave_6116 | on | 18.563 | 19.343 | 10.232 | True | True | True | True | True | goalReached |
| weave_6116 | off | 18.563 | 19.343 | 10.232 | True | True | True | True | True | goalReached |
| weave_6117 | on | 18.201 | 18.961 | 9.924 | True | True | True | True | True | goalReached |
| weave_6117 | off | 18.201 | 18.961 | 9.924 | True | True | True | True | True | goalReached |
| weave_6118 | on | 18.372 | 19.110 | 10.045 | True | True | True | True | True | goalReached |
| weave_6118 | off | 18.372 | 19.110 | 10.045 | True | True | True | True | True | goalReached |
| weave_6119 | on | 18.488 | 19.240 | 10.127 | True | True | True | True | True | goalReached |
| weave_6119 | off | 18.488 | 19.240 | 10.127 | True | True | True | True | True | goalReached |
| weave_6120 | on | 18.508 | 19.278 | 10.131 | True | True | True | True | True | goalReached |
| weave_6120 | off | 18.508 | 19.278 | 10.131 | True | True | True | True | True | goalReached |
| empty_control | on | 16.000 | 16.000 | 7.708 | True | True | True | True | True | goalReached |
| empty_control | off | 16.000 | 16.000 | 7.708 | True | True | True | True | True | goalReached |
| sealed_control | on | — | — | — | False | False | False | False | False | noValidatedSeed |
| sealed_control | off | — | — | — | False | False | False | False | False | noValidatedSeed |
| exampleMovingRotatingObstacleField | on | 20.716 | 20.716 | 9.042 | True | True | True | True | True | goalReached |
| exampleMovingRotatingObstacleField | off | 22.477 | 23.412 | 9.261 | True | True | True | True | True | goalReached |
| exampleMovingBarrierWait | on | 10.000 | 10.000 | 10.090 | True | True | True | True | True | goalReached |
| exampleMovingBarrierWait | off | 10.000 | 10.000 | 10.090 | True | True | True | True | True | goalReached |

## Evidence and limits

The supplied example functions were not executed: their unchanged setup sections were captured once and their exact planner requests replayed. The new cases use finite jerk [4 4]; maintained requests preserve their example defaults. Only earliest-arrival requests were measured. This measures the current complete planner under this budget, not exhaustive feasibility or shortest-path optimality.

Raw per-run data, captured requests, complete results/diagnoses, public validation outputs, source hashes, frozen source, predeclared protocol, harness, and watchdog logs remain in `tmp/shortcut-benefit-20260906/`. The archived CSV includes warm-ups, unfavorable runs, and timeouts. No production options or algorithm changes were made.

Lengths above are measured from the returned route and sampled motion histories; they are not analytic curve-length certificates. Across the 22 new cases, the sum of per-case median planning times is 53.662 s on versus 39.975 s off; this aggregate includes failed attempts and is specific to this chosen suite.

[All 192 raw run records](shortcut_benefit_20260906.csv). Exact generated input definitions are retained in [the generator](../reference/shortcut_benefit_20260906/buildShortcutCases.m); its two companion capture functions reproduce the maintained requests without running their plotting wrappers. The MAT files under the local scratch directory preserve the resolved requests and complete returned records.
