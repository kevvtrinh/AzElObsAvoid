# Supplied Vietnam boundary benchmark

The CSV retains all 842 supplied coordinate pairs and decimal text: 280 rows
at 2770 s and 281 at each of 2910 and 3000 s, including two explicit closing
copies. The fixture removes those copies before interpolating the original
280 vertex indices at 0.25 s spacing. The source file is unchanged.

## Declared normalization and exact motion

The shared constructor's strict proper-crossing rule removes five zigzag
vertices at every one of the 921 samples, producing 275 vertices in both
original and protected histories. These changes are declared repairs, not
convex-hull substitution. Relative to MATLAB's simplified original fill,
the repair **adds** 0.34087064 to 0.79924373 square units and removes zero
area. The three anchor additions are 0.79924373, 0.48124398, and 0.34087064.
The previous description of these quantities as removed area was incorrect.
Per-sample counts and both added/removed areas remain visible in
`NormalizationDiagnostics` through canonical rebuilds.

All 920 intervals have the exact `linearCorrespondingConvexPartition` model.
Preparation preserves the complete supplied time and coordinate history but
proves two spans, [2770,2910] and [2910,3000] s. Each span has one conforming
273-triangle partition. Its restrictions give the same proven face indices
on all source intervals. The 918 redundant keyframe boundaries disappear
from time cells, so the cell count is 546; every supplied keyframe remains
a search time layer, exactly as for unmerged histories.

The alignment at every original short interval preserves the supplied indices.
Re-running circular alignment between 2770 and 2910 s instead selects a
shift with maximum coordinate difference 11.901921783 units and fails both
partition and simplicity proof. The canonicalizer therefore carries
the existing source-interval alignment into its span proof. It never
substitutes a newly aligned long-span motion.

```matlab
addpath(pwd,fullfile(pwd,'examples'),fullfile(pwd,'trajectory'));
[obstacle,initialState,goalState,limits] = createVietnamBoundaryScenario();
prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle,[2770,3000]);
assert(all(prepared.InternalPreparation.IntervalGeometryModel == ...
    "linearCorrespondingConvexPartition"));
assert(isequal(prepared.InternalPreparation.MergedSpanTime_s, ...
    [2770,2910;2910,3000]));
result = planner(prepared,initialState,goalState,limits, ...
    struct('GoalTimeMode','fixedArrival','WrapX',false));
assert(result.Success);
validation = obstacleAvoidance.validateTrajectory(result);
assert(validation.Passed);
```

## Measurements and verification

Measurements for Brief D are recorded below after the final verification run.
The supplied pre-change measurements were unsupported interpolation before
repair, and 226.8 s preparation plus 349.4 s planning in the user's repaired
experiment. Those baseline timings were supplied by the user, not rerun here.

An intermediate measured candidate run returned arrival 3000 s, length
113.137085047164 units, and passed independent validation. Preparation took
3.417355 s, planning 17.973679 s, and separate validation 3.720484 s (25.111518 s
total, excluding 5.855201 s fixture construction and MATLAB startup).

## Deforming U.S. outline

The first moving-cell construction used axis-aligned margin squares of half-width
`safetyMargin_units`. That square contains the disc buffer but not the
constructor's square-join protection: for the first US interval [0,5] s,
exact Boolean subtraction found 0.002117077092612 and 0.001818174066173
square units of the authoritative endpoint samples outside the candidate
moving-cell union, and a rotated unit square with 0.1-unit protection reproduces
0.000655552753962 square units missing from its margin-square hull. A square
join of distance `d` reaches at most `d*sqrt(2)` from the source, so the
moving cells now use half-width `sqrt(2)*safetyMargin_units`. With that
margin the explicit endpoint containment check passes on every US interval
(uncovered area at roundoff), and `testBoundedCorrespondence` keeps both the
counterexample for half-width `d` and the proof for `d*sqrt(2)`.

Two further changes were required for the US history. Circular correlation
undoes a rotation by a cyclic index shift, so `createMovingObstacle` declares
`vertexCorrespondence` as `sourceIndex` and the moving cells follow the
caller's transform. The moving-cell union of a coastline outline is then covered by
grid squares at the interval's displacement scale (at most 256 squares per
interval), each clipped piece replaced by its convex hull, so cell counts do
not follow coastline detail: without the cover the 14613-vertex outline
produced 78198 cells and the solver ran out of memory.

The full 14613-vertex outline is supported and slow: one run planned it in
1005 s wall with 5142 cells, arrival 29.333 s, length 40.4556, and passed
independent validation. The maintained example therefore caps the supplied
outline at 100 vertices with `MaximumOutlineVertices`, a deterministic
Douglas-Peucker reduction in the example helper that keeps the smallest
tolerance meeting the cap and splits any chord that crosses the far shore
(the raw reduction folded the outline at the Chesapeake Bay mouth). The
reduced outline is the supplied obstacle; the planner treats it exactly.

Two computation-only changes then removed most of the remaining time
without changing any enclosure. The cover pieces are replaced by the exact
longest-shared-edge-first convex repartition of their union (same enclosed
set; 5524 cells became 2680 for the whole scene, 1582 of them U.S. moving-cell
cells across the 48 intervals), and the timed search tests every candidate
edge against a moving convex cell in one vectorized batch with the same
probes and tolerance as the former point-by-point loop. The node set of the
search follows the cell vertices, so the repartition changed the discrete
route proposal: arrival moved from 24.7124 s to 25.8355 s at the same
length, both independently valid.

Measured on the 100-vertex example (`runDeformExamples`, one cold run, this
branch): wall 83.03 s, planner 61.93 s, arrival 25.8355 s, length
40.5138437, independent validation passed, every U.S. interval
`movingCellCorrespondingConvexCells`. The same run planned the Vietnam slew in
20.16 s (arrival 3000 s, length 113.137085, 546 cells; example wall
30.66 s). `testDeformingUSOutline` keeps the U.S. case supported and exact.
