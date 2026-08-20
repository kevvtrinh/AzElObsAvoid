# Plan 325 branch assessment

## Evidence scope

This assessment applies to the current Plan 325 worktree based on commit
`4f59472` plus the verified prepared-obstacle change.

- All 18 maintained examples ran in separate MATLAB processes.
- Seventeen examples returned independently validated success.
- The no-path example returned the expected validated failure.
- Visible success and failure plot checks passed.
- All 56 tests passed.
- Code Analyzer checked 54 MATLAB files and returned 0 messages.
- Production has 28 files and 6,937 physical lines.
- The complete MATLAB tree has 54 files and 11,231 physical lines.

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

The final 40-circle run took 22.443 seconds. Its prepared-data baseline was
23.006 seconds. This is a 2.45% decrease. The moving-U.S. run took 36.367
seconds. Its baseline was 86.511 seconds. This is a 57.96% decrease. Both
motions passed independent validation and kept the same arrival duration
within numerical solver variation.

The four-accelerating-circle run took 54.110 seconds instead of 65.062
seconds. The moving-barrier run took 30.089 seconds instead of 40.570 seconds.

### 4. Dynamic data preparation is interval-aware

The planner creates source-slice shapes and interval interpolation data once
per planning call. Matching-topology intervals keep vertex deltas and speed
bounds. Topology-changing intervals keep a conservative union for that
interval. The planner does not replace the complete history with a static
shape. Public results keep the canonical obstacle format.

### 5. Sparse visibility work is explicit

The seed graph tests Delaunay candidates plus all start and goal connections.
The 40-circle case tested 62 of 153 pairs without losing a visible edge or
changing the selected route. The wide U tested 55 of 120 pairs and kept both
route classes and the same arrival time. No wall-time gain was confirmed.

### 6. Runtime regressions now have an explicit rejection rule

An accepted change must not produce a confirmed planning-time increase on
its affected representative examples. Apparent increases receive repeated
warm measurements. A change is rejected when the increase exceeds observed
run noise and the user did not explicitly accept the tradeoff.

The shared-jerk correction received an A/B check. Old and new basic-example
timing ranges overlapped. The returned trajectory was identical.

### 7. Example controls now have one physical meaning

Every maintained example routes `MaxJerk_deg_s3` into
`limits.maxJerk_deg_s3`. The eight corrected examples preserve their old
`[2 2]` defaults. An explicit `[1.23 1.45]` override reached the planner and
passed independent validation.

### 8. Plot and diagnostic behavior is stable

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

The extreme-U.S. case takes 132.554 seconds. The two-U case takes 66.025
seconds. These times remain large for interactive planning. The moving-U.S.
case decreased to 36.367 seconds after prepared dynamic data was added.

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

Production is 63 lines below the 7,000-line hard limit. The complete tree is
731 lines above the preferred target. New production work must preserve the
hard limit.

## Cleanup audit decisions

The two downloaded audit reports apply to the baseline commit and contain
some stale findings.

- Workspace ownership, timeout removal, verbose output, size, and jerk routing
  are now corrected.
- `certifySeedCorridor` remains because the production validator calls it.
- `RandomSeed` remains because immediate removal would break the public result
  schema. A later compatibility migration can deprecate it.
- Independent validation remains separate from solver constraints.
- Polynomial sampling remains a valid measurement-first cleanup candidate.
- Repeated obstacle preparation is removed from the measured planning path.

No numerical cleanup was accepted without profiler evidence and a runtime
comparison.

## Recommended next work

1. Improve HS3 variable scaling and verify that warnings and runtime decrease.
2. Improve the straight-target route length without increasing planning time.
3. Profile the extreme-U.S. case with the prepared dynamic data.
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
