# Static U lessons

The source lessons and their visible companions live in
`walkthrough/example-static/` at the repository root.

Open the lessons in MATLAB R2024b and run their sections in order.
Each file runs independently using the inputs from `examples/exampleStaticUShapedObstacle.m`.
Update `projectFolder` inside the script if the repository moves.

Step 1 explains checked inputs, the original wall, the miter safety margin,
the separate zigzag diagnostics, and the protected polygon and its eight edges.
Plots are created directly by the script. It stops before route search or motion
generation.

Step 2 expands the convex partition: test the boundary turns, triangulate the
wall, order shared edges, and merge neighboring regions only when their union
stays convex. Plots show the triangles, final pieces, and actual accepted and
rejected merge candidates. Every partition calculation is inline.

Step 3 expands the verified zero-translation interval, speed bounds, active
history, snapshots, point occupancy, and outward edge directions. It reuses the
convex partition already taught in Step 2 and stops before visibility search.

Steps 5–8 keep the requested numbering and run independently; they do not
require a Step 4 file. Each expands its newly taught calculations inline and
uses named production calls to repeat earlier stages.

Step 5 expands occupied edge sides, incoming/outgoing edge matching, corner
turns and the exact reduction from eight boundary vertices to six route corners.
Plots show why the two inward corners are omitted without removing any wall.

Step 6 expands endpoint-entry tests, supporting-line tangency, boundary
intersections, contacts and strict-interior midpoint checks for all fifteen
corner pairs. A separate teaching diagram shows the rejected direct chord
crossing the protected wall at fractions 0.38 and 0.72.

Step 7 expands endpoint clearance, all start/goal connections, saved-pair index
mapping, length costs and construction of the full eight-node graph. MATLAB's
`shortestpath` selects the spatial route. Plots show that route, nearest-edge
clearance and accumulated distance/progress. This is not a timed motion.

Step 8 follows the actual static earliest-arrival seed: degree 8, SplitCount 3,
and `UsesVariableClock = false`. It expands redundant-point removal, integer
length allocation, subdivision, Bézier controls, derivative-based starting
times and endpoint-state controls. Plots show twelve starting segments, repeated
controls, duration bounds and initial jerk jumps. It stops at `startingCurve`,
before BMTP optimization and independent validation.

Steps 9–16 continue that same Static U execution path:

- Step 9 builds an actual maximum-margin separating-line problem and expands
  the independent side, gap, normal-length and physical-clearance checks.
- Step 10 builds every workspace bound, endpoint equation, C3 join equation
  and velocity/acceleration/jerk constraint row.
- Step 11 expands obstacle-line rows, relaxed time-power cones, fixed-value
  elimination, the actual conic solve and decoding of its proposed motion.
- Step 12 evaluates all nine actual arrival proposals, discovers sampled
  overlaps, follows retention/stopping decisions, and expands fixed-clock
  edge-length objectives, slack, three line-loading rounds and acceptance.
  **Run Section 3 to open the iteration slider in a MATLAB figure window.**
  It shows the curve, sampled overlap map, proposed duration and retention
  status for iterations 1–9, alongside actual minimized arrival `p3`, all
  four clock variables, monitored polygon length and every separating-line
  `q` labeled by segment/region. The shortening section prints all three
  rounds' edge-bound sums, slack sums, weighted penalties and total costs,
  and plots the objective components. Text explains why newly loaded wall
  constraints can increase costs and which quantities are monitored rather
  than minimized. Saved Live Editor output is a static snapshot.
- Step 13 expands exact curve subdivision, Bernstein-to-power conversion,
  degree-eight endpoint matching, polynomial peak rates and duration dilation.
- Step 14 expands timing, stored derivative consistency, C3 joins, endpoints
  and whole-interval bounds using Bernstein controls, subdivision and extrema.
- Step 15 expands proof subdivision from 24 to 31 pieces without changing
  the physical motion, rebuilds original obstacle geometry, and checks every
  final curve segment against all three regions: 93 independently verified pairs.
- Step 16 expands sampled histories, numerical curve-length integration,
  integrated squared jerk, metadata comparisons and final public validation.

These lessons use the visible companions in `reference/` to record actual
solver intermediates. `recordStaticUSolve.m` runs a diagnostic copy and the
public planner on identical checked inputs, requires exact equality of their
returned polynomials, and reuses a recording only for identical inputs and
source text. `buildStaticUTraceRuntime.py` copies the BMTP package into ignored
output and inserts diagnostic records; it does not edit production source.
The recorder uses the bundled Python runtime when available, otherwise `python`
on PATH. MATLAB R2024b and Optimization Toolbox are required for the solves.
`drawStaticUIteration.m` only repaints arrays calculated inline in Step 12.

Earlier taught stages appear as explicit setup calls. Newly taught calculations
are expanded top to bottom, including degree-eight polynomial adjustments.
Polynomial extrema use the production numerical root calculations; the
lessons do not claim exact-arithmetic certification. The public independent
validator remains the acceptance authority.

All plots are created directly in MATLAB. Lessons use explanatory text blocks,
no tables or teaching assertions, and at most two vertically stacked tiles per
figure. These cover the Static U execution path; unexecuted empty, timed and
fixed-arrival branches are identified where they differ.

These Live Scripts are teaching source, kept without embedded execution outputs.
Executed local copies and verification notes are in the ignored
`output/static-u` folder. After editing a local copy, copy its source back here
without saved outputs before committing it.

The ignored output folder also retains `buildLaterLessons.py`,
`verifyLaterLessons.m`, `inspectLaterLessonOutputs.py`, and
`publishLessonSources.py` for reproducing Steps 5–8 and their checks. Build first,
run the MATLAB verifier, then inspect native saved text/code/figures and publish
source copies. Verification assertions are confined to the verifier. The
`verification-steps05-08.txt` summary and `steps05-08-output-checks.json` record
the completed comparisons; per-step `verification-step05.log` through
`verification-step08.log` retain MATLAB execution results.
Disposable extracted code and figure previews are
removed after review; the builder recreates them when needed.

For Steps 9–16, the corresponding retained reproduction tools are
`buildFinalLessons.py`, `verifyFinalLessons.m`, and
`inspectFinalLessonOutputs.py`. The ignored `staticU_solverTrace.mat` retains
the diagnostic recording used for comparisons; lessons obtain fresh records
from the recorder rather than loading this file. `verification-final.log`,
`verification-steps09-16.txt`, and `steps09-16-output-checks.json` retain the
completed numerical, slider and native-output checks. Temporary diagnostic
runtime copies, extracted scripts and plot previews are removed after review.
`verification-proof-export.log` retains the expanded three-round proof
subdivision and final export comparisons; `verification-layout.log` retains
the last native matrix-width and title-layout pass.
`verification-objectives.log` retains the Step 12 objective/readout check
against nine actual trajectory vectors, 72 line vectors and all three
fixed-clock solves. Its diagnostic records include the raw line vectors
before verification adjusts offsets.

`relocation-checks.json` checks all 15 source Live Scripts after the folder
move: valid document structure, updated paths, unchanged non-document archive
entries and no saved outputs. `verification-relocation.log` records MATLAB
execution of Step 1 and Step 12 from outside the project, including the
recorder, iteration viewer and independent public validation.
