# Plan 325 branch assessment

## Evidence scope

This assessment covers commit `a023f1c` plus the current uncommitted Plan 325
worktree. The worktree also contains user changes that predate this
continuation; they were preserved.

- All 18 maintained examples ran serially and headlessly on the final
  batched-bound planner source. Seventeen returned independently validated
  success and the no-path example returned its expected validated failure.
- The final full suite passed 56 of 56 tests. Code Analyzer checked all 55
  MATLAB files present in the worktree and reported zero messages.
- A visible default success created four valid graphics objects. A visible
  expected failure created two diagnostic figures and retained two rejected
  transitions.
- Maintained production has 28 files and 7,139 physical lines. The maintained
  tracked MATLAB tree has 54 files and 11,873 physical lines.
- The untracked interactive sandbox has 694 lines. It is useful for manual
  experiments but is not included in maintained-tree counts or benchmark
  claims; adding it to the repository would exceed the 12,000-line hard cap.

## Current judgment

Plan 325 remains a compact, physically validated planner with materially
better runtime and earlier arrival on the affected static-visibility cases.
The most important change is input-driven: an exact multi-obstacle visibility
seed may now receive the same early HS3 opportunity as the prior single- or
reduced-obstacle cases. The analytic stop-at-waypoint motion remains the
validated fallback, and all early and later HS3 work shares one bounded time
budget.

The planner does not claim global optimality or search completeness. It returns
the fastest independently validated candidate found within deterministic
proposal limits and bounded local optimization work.

## Largest strengths

### 1. The wall-hugging failure class now receives an early smooth-motion test

The old eligibility condition suppressed early HS3 whenever more than one
exact obstacle was present. In the two-polygon interactive case, this allowed
the analytic boundary-following fallback to win before a wider smooth arc was
tested. The general fix depends only on seed provenance and obstacle count; it
contains no scenario names, route directions, hidden waypoints, or fixture
geometry.

The sandbox case decreased arrival from about 20.718 to 8.859 seconds while
selecting a wider smooth route and passing independent validation. The
maintained two-opposing-U case retained its exact 22.876124561206-second
arrival and 24.302835531542-degree motion while final wall time decreased to
34.8855048 seconds.

### 2. Polynomial and continuous-bound evaluation are batched exactly

Polynomial histories are evaluated by sample batches instead of repeated
per-sample helper calls. Bernstein conversion accepts multiple polynomial
columns, and HS3 converts segment/axis bounds in batches while reconstructing
the legacy inequality ordering exactly.

Uniform- and nonuniform-duration polynomial checks were bit-for-bit equal to
the scalar calculation. Matrix Bernstein conversion and complete bound vectors
were also bit-for-bit equal, including azimuth-wrapping mode. The optimization
therefore changes evaluation cost, not constraint meaning or tolerance.

### 3. Arrival and runtime improved without weakening validation

On the final 18-example sweep:

- extreme outlines reached 6.684968340018 seconds in 47.9449594 seconds wall;
- the wide U reached 22.819550649779 seconds in 16.8144250 seconds wall;
- 40 moving circles retained 64.556780043561 seconds and ran in 7.9373138
  seconds wall;
- two opposing U shapes retained 22.876124561206 seconds and ran in
  34.8855048 seconds wall;
- obstacle-free planning retained 4.613406126529 seconds and ran in
  5.9277416 seconds wall.

Every successful motion passed collision and applicable kinematic certificate
checks. The expected no-path result remained a stable failure with diagnostics.

### 4. The size allowance is supported by a declared A/B set

At 7,139 production lines, the 139-line excess requires a 41.7 percent
wall-time reduction. Before final measurement, the declared representative set
was the extreme outline, dense concave, and U-shaped time-space examples.

Against clean `a023f1c` in separate headless MATLAB processes:

| Example | Baseline wall (s) | Candidate wall (s) | Reduction | Arrival result |
| --- | ---: | ---: | ---: | --- |
| Extreme outlines | 83.8056819 | 48.2212733 | 42.46% | identical |
| Dense concave | 43.6252843 | 16.8686791 | 61.34% | identical |
| U-shaped time-space | 89.9305427 | 17.1115690 | 80.97% | improved |

The minimum measured reduction is 42.46 percent, so the proportional allowance
passes by 0.76 percentage points. This margin is narrow and should not be
presented as a broad machine-independent speed guarantee.

## Main weaknesses

### 1. Search and optimality remain bounded

Spatial and timed proposals use finite samples and deterministic caps. HS3 is
local and time bounded. A missed topology or poor local basin can still prevent
the globally fastest motion. Final validation prevents false success but does
not prove global infeasibility or global minimum arrival.

### 2. Conditioning warnings remain

The basic example can emit MATLAB matrix-conditioning warnings. Returned
motions pass independent polynomial, collision, endpoint, and kinematic
validation, but variable scaling remains the best next numerical target.

### 3. One route-quality weakness remains visible

`exampleStraightTargetAlternatingOcclusion` retains a 19.229413227596-degree
motion for a 13.341664064126-degree polyline. It is valid and meets its fixed
arrival, but the excess length should be improved without accepting a runtime
regression or scenario-specific guide.

### 4. Repository size has little headroom

Production passes only through the proportional performance allowance. The
maintained tracked MATLAB tree is 127 lines below the 12,000-line hard cap and
1,373 lines above the preferred 10,500-line target. The untracked interactive
sandbox cannot be added as-is without cleanup.

## Rejected experiments and recovery

- Stopping after the first early multi-obstacle HS3 result regressed the
  two-opposing-U arrival from 22.8761246 to 23.9675706 seconds. That form was
  removed; remaining unattempted exact topologies now share the residual HS3
  budget.
- Continuing early HS3 too broadly increased the 40-circle wall time from its
  roughly 9-second reference to 24.217 seconds. Continuation was narrowed to
  multiple exact obstacles; the final 40-circle run is 7.9373138 seconds.
- A template-consolidation edit initially left one stale local constructor
  call. The focused suite exposed 14 errors. The call was fixed and all 43
  focused tests then passed before the full 56-test run.

Unfavorable evidence remains part of the assessment rather than being replaced
by the accepted measurements.

## Recommended next work

1. Profile and improve HS3 variable scaling, keeping exact constraint and
   final-validation equivalence tests.
2. Improve the alternating-occlusion route length through a general
   through-velocity or topology-ranking improvement.
3. Split or shrink the interactive sandbox before considering it maintained
   repository content.
4. Keep runtime changes under the same serial A/B recovery rule and avoid
   growing either 900-line core file.

## Final claim

Plan 325 now tests wider smooth motion earlier for exact multi-obstacle
visibility routes and evaluates its polynomial constraints substantially
faster. It supports static, moving, and deforming obstacles, target
interception, timed waits, finite jerk, and stable no-path diagnostics.

It is a bounded, independently validated planner—not a complete or globally
optimal solver.
