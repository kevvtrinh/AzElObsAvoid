# BMTP engine

The public planning call is `planner(...)`. It calls `bmtpEngine.solve` to turn
one proposed path into motion with position, velocity, acceleration and jerk.
The engine returns a candidate and its checks. The planner's independent
validator must accept that candidate before the planner reports success.

## Where each stage lives

| Package | Responsibility |
| --- | --- |
| `+pipeline` | Collect solver inputs, build a starting curve, prepare the returned motion, and assemble output arrays. Also try to shorten timed motion while preserving its selected arrival time. |
| `+motion` | Construct and evaluate curves, apply endpoint states, split curves into smaller pieces, and calculate motion timing. |
| `+optimization` | Build the solver constraints and adjust the curve and segment durations. Record the solver result and diagnostics together. |
| `+separation` | Find and check lines that keep the curve and each obstacle on opposite sides. |
| `+validation` | Calculate rounding reserves and check the full prepared curve against obstacle, workspace, motion-rate and continuity requirements. |

Folder names begin with `+` because they are MATLAB packages. An internal call
uses the full name, for example `bmtpEngine.motion.evaluatePolynomial`.

## How `solve` chooses its work

1. `createSolveRequest` collects the checked states, limits, options and obstacle
   regions. `createWarmStart` turns the proposed route into an initial curve.
2. Try direct motion when the request permits it. For a direct earliest-arrival
   path with both ends at rest, moving obstacles may also allow waiting before
   departure. A failed departure stage can return a failure for the planner to
   handle; it does not always continue to optimization.
3. If direct motion has not passed and planning continues, select the optimizer:
   - Static obstacles and earliest arrival: `solveActivePairTrajectory` adds
     separation constraints for curve/obstacle pairs as they are needed.
   - Variable segment durations: `solveTimedAlternatingTrajectory` recalculates
     which moving regions overlap each segment as the times change.
   - Assigned segment durations: `solveAlternatingTrajectory` adjusts the curve
     with those durations fixed.
4. Prepare and check the selected motion. `prepareFinalMotion` sets endpoint
   states, corrects joins, and assigns durations once, then retains the complete
   polynomial. `refineMotionSeparation` checks that motion with `checkFinalMotion`.
   When one line cannot separate a whole segment from an obstacle, it uses
   `subdivideMotion` to check smaller pieces without changing the physical motion.
   Each piece retains its original `SourceSegmentIndex` for optimizer constraints.
   A solver may return that prepared motion and its matching checks for reuse.
5. `createMotionOutput` assembles the accepted curve and sampled output arrays.
   The planner then applies its public independent validation.

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
