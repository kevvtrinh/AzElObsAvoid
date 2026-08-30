# Plan: warm-start BMTP from a Ruckig motion

Status: not started. Written on `novel-rep` for a future attempt.

## Why

`bmtpEngine.solve` alternates time-power and separating-plane SOCPs. Its
expensive step is reaching a **first collision-free iterate**, because separating
planes cannot be established until one exists. Commit `41c4311` exists solely
because of this: at a 180 s horizon the solver produced six colliding iterates,
reached 179.203882993999 s, hit an infeasible seventh and failed, while at 360 s
it could occupy a collision-free 191.541694821082 s iterate, establish planes,
and then descend to about 100.676 s.

Ruckig produces a validated collision-free motion for the same seed in
**0.027 to 0.083 s**. If that motion can be handed to BMTP as its starting
iterate, the solver skips the step it is worst at.

## What was already tried and rejected

A different use of the same speed advantage - running a cheap Ruckig composition
for every seed and reordering the BMTP portfolio best-first - was implemented and
measured against this branch. It preserved arrival and path length bit-identically
and cost about 14 percent wall overall:

    warmed, min of 3 repetitions in one session
    case                        base      screened   delta
    exampleObstacleAvoidance    2.472  ->  2.380     -3.7%
    exampleStaticUShapedObstacle 1.912 ->  1.579    -17.4%
    exampleStraightTarget...     5.488 ->  6.013     +9.6%
    exampleTargetExitsObstacle  18.073 -> 20.731    +14.7%
    rogueBundle                  7.575 ->  9.888    +30.5%
    total                       35.52  -> 40.59      +14%

The screen runs a composition for *every* seed up front, so it only pays where
reordering finds a good incumbent early, and it costs most where there are most
seeds. Do not re-implement it. The warm start is the other half of the idea and
was never built.

## The interface

Read from `trajectory/+bmtpEngine/solve.m`:

```matlab
solve(seed, obstacles, initialState, goalState, limits, regions_deg, ...
      options, warmStart)

warmStart.ControlPoint_deg   % segmentCount x (degree+1) x 2
warmStart.SegmentTime_s      % positive SCALAR
```

- `degree` is 7 for the compact representation, 16 otherwise (`solve.m:60-62`).
- `segmentCount` derives from the seed route, capped by
  `maximumWarmSegmentCount` (`solve.m:70-76`). A longer route is resampled and
  `diagnostics.WarmRouteResampled` records it.
- Validation (`solve.m:113-125`) requires an exact size match, all-finite control
  points, and a finite positive scalar `SegmentTime_s`. A malformed struct raises
  `bmtpEngine:InvalidWarmStart`. An empty struct is the normal cold start.

`SegmentTime_s` being **scalar** is the crux: every Bezier span must have the
same duration.

## The actual difficulty

It is not format conversion. The two representations partition time differently
and one of them cannot be re-partitioned exactly.

A Ruckig profile is piecewise-**constant-jerk**, so each phase is a cubic in
time, and phase durations are **unequal** - they fall wherever the switching law
puts them. BMTP requires N spans of **equal** duration.

A uniform span will therefore generally straddle two or more jerk phases. Across
such a span the motion is piecewise-cubic with interior breaks, which is not a
single polynomial. A degree-7 Bezier represents a cubic exactly but cannot
exactly represent a piecewise-cubic with an interior break.

Exact conversion is impossible in general. Three ways out, in preference order:

1. **Accept approximation.** A warm start is an initial iterate, not an answer.
   BMTP refines it and `obstacleAvoidance.validateTrajectory` still gates
   acceptance. This framing makes the problem tractable; take it.
2. **Use degree 16**, which has far more freedom than degree 7 for a
   multi-break span.
3. **Raise the segment count** so spans are short enough to contain at most one
   break, subject to `maximumWarmSegmentCount`.

Do not chase exactness. Measure the approximation error and decide on evidence.

## Steps

### 1. Conversion, standalone and tested before any planner wiring

Write a converter from a Ruckig polynomial record to
`{ControlPoint_deg, SegmentTime_s}`:

- Take total duration `T` and target `segmentCount N`; set
  `SegmentTime_s = T / N`.
- For each uniform span, evaluate the Ruckig position polynomial at
  Chebyshev-spaced sample times and fit degree-`d` Bernstein control points by
  least squares.
- Return the maximum position deviation between the fitted curve and the true
  motion, per span and overall.

Test in isolation with no planner involved. **A span lying inside a single jerk
phase must reproduce to ~1e-12** - assert that explicitly, it is the correctness
anchor. Report the error for multi-break spans rather than asserting a bound you
have not measured.

Reuse rather than rewrite: `powerToBernstein` already exists inside
`+obstacleAvoidance/validateTrajectory.m` for the plane certificate, and
`bmtpEngine.evaluatePolynomial` evaluates the source motion.

### 2. Certify the converted curve before handing it over

A warm start that collides is worse than none: BMTP would still have to find a
feasible point, having paid conversion cost for nothing.

Check the converted control net against the protected obstacles using the same
Bernstein convex-hull separation the validator performs in
`certifyStaticPlaneCertificate`. If the converted curve is not collision-free,
**do not pass it** - fall through to the cold start and record why.

Report how often conversion succeeds and how often the result survives this
check. If it rarely survives, the approach is dead and that is the finding.

### 3. Wire in behind an option; measure iterations, not just wall

In `planCorridorQuintic`, where a seed already has a validated Ruckig
composition, convert it and pass it to `solveBmtpTrajectory`. Gate on a new
documented planner option defaulting to off until measured.

The primary metric is **outer SOCP iterations to the first collision-free
iterate**, with and without the warm start. If iterations do not drop, the
mechanism does not work regardless of what the clock says.

### 4. Measure honestly

Unwarmed wall-clock variance on this machine has exceeded **30 to 48 percent per
case**, larger than the effect being measured. Two successive single-run A/B
tests of the pre-screening gave "12 percent slower" and then "within 0.8
percent"; both were noise and either would have been reported as fact.

Discard a warm-up pass, take at least three repetitions inside one MATLAB
session, and report min and median. Use `MATLAB_PREFDIR=<scratch dir>` on every
invocation, or concurrent MATLAB processes collide on the shared preference
directory and die with `System Error: File system inconsistency` before loading
any code.

## Acceptance

- Arrival and path length unchanged within 1e-9 on every maintained example.
  This is a speed change; if an answer moves, it has failed.
- Suite stays green (89/89 on this branch).
- `exampleNoPath` still fails as `noValidatedSeed`.
- Reported: conversion error per case, fraction of converted curves passing the
  collision check, and outer iterations to first feasibility with and without.

## Kill criteria

Stop and report rather than pushing through if any hold:

- Converted curves rarely pass the collision check. Conversion error is then too
  large for the geometry and tuning will not fix it.
- Iterations to first feasibility do not drop. The premise - that finding a
  feasible point is BMTP's bottleneck - would be wrong for these scenes.
- Any arrival or path length moves. That is a correctness failure, not a tuning
  problem.

## Scope

This makes the existing solver reach its first feasible point faster. It does
not make a Ruckig kernel a replacement for BMTP.

That replacement was attempted separately across thirteen mechanisms and does
not work, for two independent reasons. Static U demands a 0.165685 deg maneuver
against a 1.333333 deg kinematic minimum, so a trajectory composed of
individually traversable segments cannot take the turn at speed. And the global
formulation that would dissolve that limit - optimising phase times against
separating constraints from an infeasible warm start - is what `bmtpEngine.solve`
already is. A full 38-row corpus comparison found the kernel had zero arrival
wins and no case it validated that BMTP did not.
