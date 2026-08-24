# Completed ungrouped-obstacle performance checkpoint

## Objective and scope

On `325-full-suite`, make the corridor-quintic engine practical for the
maintained moving-circle case without seed-obstacle grouping, using only
input-derived obstacle information. Also make the rotating U.S. example's
supplied deformation visibly increase in size.

## Completed evidence

- The only maintained example that requests clustering is
  `exampleFortyMovingCircleGrid` (`SeedClusterDistance_deg=2`). Its supplied
  swept geometry already has one region, so clustering creates zero groups.
- Equivalent headless seed-325 grouped and ungrouped baselines both succeeded,
  passed example/grid/continuous validation, and returned the same graph,
  route, and 62.4777398626363 s motion.
- Ungrouped profiling localized 47,793 repeated shape/clearance queries;
  collision checking was 8.238 s of a 14.569 s median planner run.
- The retained corridor-only broad phase uses each obstacle's supplied
  `InternalPreparation.HistoryBounds_deg`. It assumes no fixed translation,
  speed, rigidity, or size and falls through to exact geometry near the path.
- Three alternating fresh-process ungrouped pairs preserved all reported
  physics. Median planner time fell from 14.569 s to 7.004 s and median
  collision time from 8.238 s to 0.511 s; selected collision checks fell from
  3,440 to 56.
- The U.S. history now starts at the native outline and ends after 12 degrees
  of input-specified rotation with 18%/14% nominal scale growth. Measured
  protected extents grew 16.8% azimuth and 22.9% elevation. The example still
  succeeded and independently validated.
- MATLAB Code Analyzer found zero messages in the three touched MATLAB files.
  Focused corridor, dynamic-timing, and example-contract tests passed 59/59 in
  110.6526933 s.
- All 18 corridor examples passed in fresh literal processes, the visible U.S.
  smoke produced three figures, the complete suite passed 132/132, and Code
  Analyzer finished with zero messages across 106 MATLAB files.
- Detailed measurements and every final example row are retained in
  `verification.md`, `branch_assessment.md`, and `benchmark.csv`.

## Files changed

- `+azElPlannerMethods/+corridor/validateTrajectory.m`
- `examples/private/createContiguousUSObstacle.m`
- `tests/testExampleContracts.m`
- `plan.md`
- Root-local `.agents/skills/planner-performance-diagnosis/SKILL.md` records
  the user requirement that obstacle behavior come from supplied inputs.

The unrelated untracked `docs/` directory remains untouched.

## Remaining limits

The broad phase is conservative only because its box is constructed from the
complete obstacle history supplied by the caller; absent evolution must remain
fail-safe through updated inputs, caller bounds, or replanning. Current proof
is strongest for the 40-circle and deforming-U.S. families and does not support
a global scaling claim.
Corridor construction is now the largest measured stage at roughly 4.6 of 7.0
seconds in the 40-circle case. The production tree remains above its line-count
target. No files are staged, committed, or pushed, and the unrelated untracked
`docs/` directory remains preserved.
