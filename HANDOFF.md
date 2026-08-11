# Handoff

## Implemented behavior

The repository contains a from-scratch MATLAB azimuth/elevation planner with:

- one public planning call and a stable result schema;
- canonical static, moving, multi-obstacle, and disconnected polygon input;
- fixed and moving complete-state goals;
- internal deterministic coarse-to-fine boundary and timing work;
- C2 quintic motion with exact boundary state matching and turn velocity carry;
- feasible explicit departure holds and optional moving-goal trailing;
- independent continuous dynamics and collision validation;
- wrap-safe continuous azimuth; and
- honest invalid, infeasible, and unknown failure categories.

The planner deliberately labels obstructed trajectories `Best found`. No global
detour-optimality claim is made.

## Reproduction

From the repository root in MATLAB R2024b:

```matlab
results = runtests("tests", "IncludeSubfolders", true);
assertSuccess(results);

files = [dir("*.m"); dir(fullfile("private", "*.m")); ...
    dir(fullfile("examples", "*.m")); dir(fullfile("tests", "*.m"))];
for fileIndex = 1:numel(files)
    assert(isempty(checkcode(fullfile( ...
        files(fileIndex).folder, files(fileIndex).name), "-id")));
end
```

## Evidence at this handoff

- Obstacle-free command: validated, deterministic, exact complete states.
- Nonzero boundary velocity/acceleration: validated.
- Static blocking polygon: validated detour with positive interior velocity.
- Delayed-opening moving wall: validated explicit stationary wait.
- Wrapped seam: continuous 170 deg to 190 deg, displayed terminal -170 deg.
- Moving goal: validated complete-state capture plus one-second trailing.
- Independent time grids and disconnected regions: accepted canonically.
- Between-sample moving collision: independently detected.
- Blocked endpoints and impossible deadline: proved infeasible.
- Artificially exhausted work budget: correctly reported unknown.
- Headless visualization: exercised only after validation.
- Example source-purity scan: passes.

The final MATLAB R2024b run on 2026-08-11 produced:

| Preserved input | Arrival (s) | Clearance (deg) | Velocity use | Acceleration use | Interior carry (deg/s) | Wait (s) | Planning (s) | Claim |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `example01Unobstructed` | 4.8094 | Inf | 0.7797 | 0.9984 | N/A | 0 | 0.452 | Minimum under stated model |
| `example02StaticDetour` | 6.6789 | 2.4752 | 0.7958 | 0.9998 | 4.2375 | 0 | 10.996 | Best found |
| `example03TimedWallWait` | 11.5008 | 1.3794 | 0.9999 | 0.7388 | 0 | 5.25 | 20.007 | Best found |

All 15 function-based tests passed in 49.5 seconds. Code Analyzer reported
zero messages across 28 MATLAB files. Runtime and memory vary with hardware;
the benchmark function returns the full evidence table, including terminal
errors, travel, wait count, memory after each case when available, and the
guarantee label. Deterministic command content does not depend on wall-clock
measurements.

## Known boundaries

- Global minimum arrival is not proved for obstructed scenes.
- Wait search currently introduces stationary holds at the initial safe state;
  it does not yet optimize arbitrary intermediate loiter locations.
- Topology-changing moving polygons use a conservative adjacent-slice union,
  which may reject a feasible narrow time window.
- First-arrival exclusion before the claimed endpoint uses a dense independent
  complete-state scan; a future analytic root certificate would be stronger.
- The route generator is boundary-derived and deterministic, not
  probabilistically complete.

## Next smallest evidence-backed tasks

1. Add analytic complete-state first-arrival root isolation.
2. Add internally selected safe intermediate loiter states for moving scenes.
3. Add an independent continuous lower bound for a restricted static polygon
   class and promote claims only when the bound is met.
4. Expand preserved seeded holdouts without scenario-specific tuning.

There are no required generated files. MATLAB temporary figures and test output
are ignored by Git.
