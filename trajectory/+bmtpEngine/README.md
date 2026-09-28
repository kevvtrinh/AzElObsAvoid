# BMTP engine

The public planning call is `planner(...)`. It calls `bmtpEngine.solve` to turn
one proposed path into motion with position, velocity, acceleration and jerk.
The engine returns a candidate and its checks. The planner's independent
validator must accept that candidate before the planner reports success.

## Read the engine in this order

1. `solve.m` chooses direct motion or one of the three BMTP optimizers and
   returns a stable success or failure result.
2. `prepareRequest.m` checks the supplied route, states, limits, options, and
   prepared obstacle regions. `createStartingCurve.m` gives BMTP controls and
   initial segment times for that route.
3. `evaluateCandidate.m` builds one motion from selected controls and times,
   then keeps its matching check. Direct candidates may stop at the first
   unproved obstacle pair. Optimizer candidates get a complete check.
4. `createMotionOutput.m` samples and formats the selected motion. The planner
   then runs its separate public independent validator before reporting success.

## Where the supporting math lives

| Package | Responsibility |
| --- | --- |
| Engine root | Own the request, starting curve, candidate handoff, output, and top-level decision flow. |
| `+motion` | Construct and evaluate curves, apply endpoint states, split curves into smaller pieces, and calculate motion timing. |
| `+optimization` | Build solver constraints, adjust the curve and segment durations, and optionally shorten travel at a retained clock. |
| `+separation` | Find and check lines that keep the curve and each obstacle on opposite sides. |
| `+validation` | Calculate rounding reserves and check the full prepared curve against obstacle, workspace, motion-rate and continuity requirements. This is an engine check, separate from the planner's public validator. |

Folder names begin with `+` because they are MATLAB packages. An internal call
uses the full name, for example `bmtpEngine.motion.evaluatePolynomial`.

## How `solve` chooses its work

1. Try direct motion when the request permits it. For a direct earliest-arrival
   path with both ends at rest, moving obstacles may also allow waiting before
   departure. A failed departure stage can return a failure for the planner to
   handle; it does not always continue to optimization.
2. If direct motion has not passed and planning continues, select the optimizer:
   - Static obstacles and earliest arrival: `solveActivePairTrajectory` adds
     separation constraints for curve/obstacle pairs as they are needed.
   - Variable segment durations: `solveTimedAlternatingTrajectory` recalculates
     which moving regions overlap each segment as the times change.
   - Assigned segment durations: `solveAlternatingTrajectory` adjusts the curve
     with those durations fixed.
3. Each optimizer path gives `solve` its selected controls and clock with the
   prepared motion and its matching check. `createMotion` sets endpoint states, corrects
   small join differences, assigns durations, and retains the complete
   polynomial. Larger corrections or an impossible deadline fail preparation.
   `checkMotionWithSubdivision` checks that polynomial with `checkFinalMotion`.
   When one line cannot separate a whole segment from an obstacle, it uses
   `subdivideMotion` to check smaller pieces without changing the physical motion.
   Each piece retains its original `SourceSegmentIndex` for optimizer constraints.
4. `solve` consumes the selected motion and check without reconstructing them.

Expected failure returns a candidate with `Success = false` and a reason. A
usable solver vector alone does not establish that the motion is valid.

## Terms used in the code

- A **control point** helps define a Bezier curve; it is not generally a point
  the vehicle passes through. A starting curve still needs validation.
- A **segment** is one polynomial piece with its own duration. Its time fraction
  runs from 0 to 1: on a segment from 2 to 6 seconds, fraction 0.5 means 4 seconds.
- A **separating plane** is the stored name for a separating line in this 2D
  planner. Its normal and offset define `normal x position + offset = 0`.
  The curve and obstacle must remain on opposite sides with the required gap.
- **Active** means a line is available as a constraint. **Verified** means its
  separation checks passed for the specified curve, obstacle and interval.
- A **continuous check** covers the time between output samples as well as the
  samples themselves. Changing plot sample spacing does not replace that check.

Use `ApplyCommentAndStyle.md` and `MATLAB_STYLE_PREFERENCES.md` at the repository
root when updating names, comments or layout. Review one file at a time and keep
the equations, validation requirements and planning method order unchanged
during a style review.
