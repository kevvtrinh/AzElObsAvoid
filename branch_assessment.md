# Plan 325 branch assessment

## Evidence scope

This assessment applies to planner commit `c5fb6e9` and geographic example
budget commit `f1014e7` on branch `plan-325`.

- All 18 maintained headless examples completed in separate MATLAB processes.
- Seventeen examples returned validated success.
- The no-path example returned the expected validated failure.
- The zero-input visible example passed and created four figures.
- The visible no-path example passed failure validation and created one search
  diagnostic figure.
- The focused test result is 50 passed, 0 failed, and 0 incomplete. The summed
  MATLAB test duration is 56.414948200 seconds.
- Code Analyzer checked 52 MATLAB files and returned 0 messages.

The implementation has 26 production MATLAB files and 6,997 physical
production lines. The complete MATLAB tree has 52 files and 11,588 physical
lines. The production and complete hard limits pass. The 10,500-line
complete-tree target does not pass.

## Recommended design

I would use the Plan 325 structure again. I would not combine complete planner
stacks from the other branches.

The useful structure is:

1. Keep one public planner and one stable result.
2. Generate a small deterministic set of geometry and timed proposals.
3. Build a certified finite-jerk motion as soon as one proposal supports it.
4. Validate that motion with the independent validator.
5. Use bounded HS3 only when it is required or when it can improve a valid
   motion.
6. Keep original protected obstacle histories as the final collision
   authority.

This structure gave one path for static, moving, and deforming obstacles;
fixed and earliest arrival; timed waiting; and moving goals. It also kept
expected failure results and search diagnostics stable.

## Largest strengths

### 1. Correctness before local optimization

The analytic path can return a valid finite-jerk motion before HS3 completes.
The planner does not accept it until the independent validator checks the
polynomial coefficients, event endpoints, sampled state history, continuity,
physical limits, safety provenance, and moving collision intervals.

The final matrix had 17 validated successes. Every successful result passed
collision, kinematic, and applicable certificate checks. The no-path case
returned `noValidatedSeed` and a valid diagnostic record.

### 2. Compact public structure

One public planner owns input resolution, stage control, selection, and the
stable result. Internal functions own seed generation, analytic motion, HS3,
polynomial evaluation, and validation. No production file exceeds 900 lines.

The implementation is much smaller than the large branch families that kept
snapshot graphs, SIPP, several retimers, and competing planner paths.

### 3. Useful results on the stress cases

- The 40-circle case returned a 64.5557855485-second motion in 25.6657512
  seconds. The spatial reference returned a 77.022740-second motion in
  40.793195 seconds. Plan 325 used a longer motion path.
- The moving-U.S. case returned a 25.614496552-second motion. The spatial
  reference returned 29.667632 seconds. Plan 325 took 138.9406737 seconds of
  wall time. The spatial reference took 50.403850 seconds.
- The large-U case returned a 34.9425880405-degree motion in
  38.5495931039 seconds. The spatial reference returned a 35.012080-degree
  motion in 43.976516 seconds. Plan 325 wall time was 62.3235920 seconds,
  compared with 1.546111 seconds.
- The two-U case took 62.7020007 seconds, compared with 91.218125 seconds for
  Plan 502. The returned motion lengths and durations were almost equal.

These values show that no one planner wins every metric. The Plan 325
architecture is useful, but its expensive cases still need work.

## Largest weaknesses

### 1. Proposal coverage is incomplete

Spatial and time-layer searches use finite samples. Dense-envelope and cluster
reductions can remove a useful topology. Dense moving histories can exceed the
timed-query work limit. The planner then omits the timed family and reports
incomplete coverage.

Final validation prevents false success. It cannot make the proposal search
complete. A bounded failure is not proof that no feasible path exists.

### 2. The first motion is conservative

The analytic motion stops at each geometric waypoint. It assigns a certified
duration to each edge. This gives a fast correctness path, but it can be much
slower than a through-velocity motion. Nonzero endpoint derivatives and
earliest moving-goal intercepts require HS3.

The next motion improvement should be one general through-velocity primitive
family with a continuous derivative certificate. It should not add another
public planner.

### 3. Complex geometry can still be slow

The moving-U.S. run took 138.9406737 seconds. The full geographic sequence
took 463.8727127 seconds. The large-U result took about 40 times the spatial
reference wall time. These are material runtime limits.

The final results reported zero deadline overrun. However, deadlines remain
cooperative. One active solver or geometry operation is not preempted.

### 4. HS3 remains local and toolbox-dependent

HS3 uses a frozen local corridor and local nonlinear optimization. It can miss
a feasible solution or select an unfavorable local solution. It requires
Optimization Toolbox `fmincon`.

### 5. Azimuth wrapping is restricted

Periodic obstacle images are not implemented. Wrapping is supported only for
an obstacle-free fixed-position goal. Other wrapped requests return an
identified unsupported-configuration error.

### 6. The preferred size target failed

The 7,000-line production hard limit passes by three lines. The 12,000-line
complete hard limit passes by 412 lines. The preferred 10,500-line complete
target fails by 1,088 lines. The branch is compact relative to the other full
planner branches, but it did not reach that target.

## Lessons from failed runs

- An early moving-U.S. attempt spent the complete 360-second budget in seed
  generation. Expensive proposal stages now have bounded work and preserve
  time for motion validation.
- An early dense 40-circle attempt reserved a timed-search slot that it could
  not use. The final dense work gate keeps the available spatial proposal.
- An early opening-U result failed independent example validation. The timed
  wait invariant corrected it and produced the final 10-degree motion.
- Hawaii failed twice with a 120-second geographic-region budget. It passed in
  an isolated 180-second run. The final complete sequence uses an explicit
  full-region budget, and all three regions pass.
- A numerical-equivalence test was too strict at `1e-10`. The observed
  difference was `6.96e-9`. The final `1e-8` tolerance remains below the
  planner constraint tolerance and does not change physical results.

These failures support stage budgets, independent validation, and honest
coverage reports. They do not support scenario-specific routes or hidden
waypoints.

## Final judgment

Plan 325 is a good compact planner candidate. It has a clear public contract,
a fast certified feasibility path, a bounded optimizer, strong independent
validation, and useful search diagnostics. The complete maintained matrix
supports this judgment.

It is not a complete or globally optimal planner. It also does not meet the
10,500-line complete-tree target. The next work should improve the general
through-velocity motion family and reduce dense-geometry query cost. It should
keep the current public interface and independent validator.
