# Plan 325 branch assessment

## Evidence scope

This assessment applies to the current Plan 325 worktree based on commit
`e6c3200` plus the verified sparse visibility-graph change.

- All 18 maintained examples ran in separate MATLAB processes.
- Seventeen examples returned independently validated success.
- The no-path example returned the expected validated failure.
- Visible success and failure plot checks passed.
- All 56 tests passed.
- Code Analyzer checked 53 MATLAB files and returned 0 messages.
- Production has 27 files and 7,000 physical lines.
- The complete MATLAB tree has 53 files and 11,711 physical lines.

The production and complete-tree hard limits pass. The preferred 10,500-line
complete-tree target does not pass.

## Current judgment

Plan 325 remains the best compact rebuild candidate in this repository. It
uses one planning engine, deterministic finite proposal work, a certified
finite-jerk first motion, bounded local HS3 work, and one independent final
validator. It does not join complete planner stacks from other branches.

The current API is clearer. Workspace intervals are physical limits. The
whole-planner wall-clock limit is removed. Required work uses deterministic
algorithm limits. Verbose progress identifies each planner stage.

## Main strengths

### 1. Physical validation remains authoritative

Every successful run passed collision and kinematic checks. The validator
checks polynomial consistency, endpoints, continuity, physical limits,
safety-margin provenance, and moving collision intervals.

The expected no-path case returned a stable failure result and diagnostic
figures. It did not claim that the finite search proved global infeasibility.

### 2. Minimum-arrival results improved

The current verified results include:

- single U: 26.493-second arrival;
- two opposing U shapes: 22.876-second arrival;
- 40 moving circles: 64.557-second arrival;
- moving and deforming U.S.: 12.986-second arrival;
- extreme U.S.: 8.903-second arrival.

The single-U geometry remains the reviewed wide U. Its duration is below the
requested 38-second threshold.

### 3. Dense-case planning time is materially lower

The final 40-circle run took 23.006 seconds. The moving-U.S. run took 86.511
seconds. The extreme-U.S. run took 130.892 seconds. These are lower than the
older final report values of 22.635, 127.870, and 210.499 seconds except for
the small 40-circle run difference, which is inside normal process noise.

The four-accelerating-circle run took 54.110 seconds instead of 65.062
seconds. The moving-barrier run took 30.089 seconds instead of 40.570 seconds.

### 4. Sparse visibility work is explicit

The seed graph tests Delaunay candidates plus all start and goal connections.
The 40-circle case tested 62 of 153 pairs without losing a visible edge or
changing the selected route. The wide U tested 55 of 120 pairs and kept both
route classes and the same arrival time. No wall-time gain was confirmed.

### 5. Runtime regressions now have an explicit rejection rule

An accepted change must not produce a confirmed planning-time increase on
its affected representative examples. Apparent increases receive repeated
warm measurements. A change is rejected when the increase exceeds observed
run noise and the user did not explicitly accept the tradeoff.

The shared-jerk correction received an A/B check. Old and new basic-example
timing ranges overlapped. The returned trajectory was identical.

### 6. Example controls now have one physical meaning

Every maintained example routes `MaxJerk_deg_s3` into
`limits.maxJerk_deg_s3`. The eight corrected examples preserve their old
`[2 2]` defaults. An explicit `[1.23 1.45]` override reached the planner and
passed independent validation.

### 7. Plot and diagnostic behavior is stable

The plotter uses the `main` branch visual language while consuming the Plan
325 result schema. The visible success case created three figures. The
visible no-path case created two diagnostic figures. Verbose mode reports
setup, seed generation, first motion, HS3 progress, selection, and completion.

## Main weaknesses

### 1. Proposal coverage remains incomplete

Spatial and time-layer searches use finite samples. Dense-envelope and cluster
reductions can remove a useful topology. Final validation prevents false
success, but it cannot make proposal search complete.

The 2-D homology signature separates sampled spatial classes. It does not
classify all continuous Az/El/time paths.

### 2. Some dense cases remain slow

The moving-U.S. case still takes 86.651 seconds. The extreme-U.S. case takes
120.153 seconds. The two-U case takes 64.003 seconds. These times are better
than older results but remain large for interactive planning.

### 3. HS3 still reports conditioning warnings

The basic example can produce MATLAB matrix-conditioning warnings. Accepted
motions pass independent validation, but solver scaling remains a useful
future target.

### 4. The first motion is conservative

The quintic first motion stops at each geometric waypoint. This gives a clear
finite-jerk certificate but can leave velocity capacity unused. Earliest
moving-target interception and nonzero endpoint derivatives still require
HS3.

### 5. One route-quality regression remains

The straight-target alternating-occlusion example has the same fixed target
duration, but its final motion length increased from about 15.299 degrees to
19.229 degrees. It remains collision-free and kinematically valid. This is a
route-quality issue, not a false success, and it should be improved without a
planning-time increase.

### 6. The preferred size target still fails

Production is at the 7,000-line hard limit. The complete tree is 1,211 lines
above the preferred target. New production work must remove at least as many
lines as it adds.

## Cleanup audit decisions

The two downloaded audit reports apply to the baseline commit and contain
some stale findings.

- Workspace ownership, timeout removal, verbose output, size, and jerk routing
  are now corrected.
- `certifySeedCorridor` remains because the production validator calls it.
- `RandomSeed` remains because immediate removal would break the public result
  schema. A later compatibility migration can deprecate it.
- Independent validation remains separate from solver constraints.
- Polynomial sampling, repeated obstacle preparation, and occupancy-query
  preparation remain valid measurement-first cleanup candidates.

No numerical cleanup was accepted without profiler evidence and a runtime
comparison.

## Recommended next work

1. Prepare immutable obstacle-query data once per planning call, then profile
   the moving-U.S. and extreme-U.S. cases.
2. Improve HS3 variable scaling and verify that warnings and runtime decrease.
3. Improve the straight-target route length without increasing planning time.
4. Test a through-velocity quintic first motion under the existing recovery
   rule and independent validator.
5. Consolidate exact duplicate sampling helpers only if measurement shows no
   runtime cost.
6. Keep every later change under the planning-time non-regression gate.

## Final claim

Plan 325 is a compact and useful planner candidate. It supports static,
moving, and deforming obstacles, target interception, timed waits, bounded
route diversity, and stable failure diagnostics.

It is not complete or globally optimal. Dense cases can still be slow. Future
work must improve general scaling or route quality without scenario-specific
routes, hidden substitutions, or confirmed planning-time regressions.
