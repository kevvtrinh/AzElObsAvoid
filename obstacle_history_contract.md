# Obstacle history contract

Each timestamp has paired x/y coordinate columns. Paired nonfinite rows separate
boundary rings; unpaired nonfinite coordinates are invalid input. A single
timestamp denotes a static obstacle. A history with multiple timestamps is
inactive outside its first and last timestamps, including when all samples are
empty. Status strings do not override geometry.

The shared constructor removes exact consecutive coordinate duplicates and
redundant closing copies. A run with fewer than three distinct coordinate pairs
cannot enclose polygonal area and is removed independently of the other runs.
If no runs remain, both coordinate columns are empty and the timestamp stays.
Separators alone also normalize to empty columns.

No distance or area threshold classifies distinct coordinates as duplicates.
In particular, nearby binary64 coordinates may bound a thin positive-area
region and are retained. After duplicate removal, each ring is checked for
proper crossings of non-adjacent edges `(i,i+1)` and `(j,j+1)`, with `i<j`,
using strictly opposite orientation signs and no inflated tolerance. The
interior run `i+1..j` is removed, joining vertex `i` to vertex `j+1` (cyclic
at the last edge). Crossings are recomputed iteratively, shortest interior
run first; equal-length runs use the smaller first index. If that leaves
fewer than three distinct vertices, the whole run is removed and recorded.
The rule uses the supplied ring order, identically for both geometry roles.

This is a declared repair of the supplied ring: a self-crossing ring has no
unique supplied fill. Removed vertex counts and both removed and added area
are reported per sample and role, relative to MATLAB's simplified fill of
the pre-repair ring. Added area must not be described as removed area. The
repair does not replace a concave ring with its convex hull. Equal zigzags
at every sample preserve equal counts and index correspondence; different
repairs can destroy correspondence. Three distinct vertices alone still do
not certify a valid polygon.

Original and protected histories follow the same normalization rules and stay
distinct. An absolute safety margin is rebuilt from original geometry, never
added repeatedly to protected geometry. `NormalizationDiagnostics` records
removed-run and duplicate counts by role, `RemovedZigzagVertexCountBySample`,
`RemovedZigzagAreaBySample_units2`, `AddedZigzagAreaBySample_units2`, the
`selfCrossingZigzagRemoved` reason, and affected
sample indices/times. Lists contain at most one entry per history sample.
Canonical rebuilds preserve these diagnostics for the same time history;
diagnostics are informational and never determine obstacle occupancy.

At a sample time, its normalized protected geometry is authoritative. An empty
sample does not clear either neighboring interval. Between samples, the current
preparer interpolates verified corresponding vertices. A moving concave ring is
represented by one conforming convex partition whose vertices use that same
linear correspondence. Every face must remain strictly convex and the outer
boundary must remain simple over the complete interval. The partition union is
therefore the interpolated polygon, not its convex hull. Geometrically equivalent
endpoint samples may use an exact static model.

When an exact continuous face certificate is unavailable, single original
rings with equal counts and a source-vertex triangulation may use
`sweptCorrespondingConvexCells`. Protected buffered rings need not correspond.
The original rings use the declared vertex correspondence (below). Every
triangle of the lower original sample's constrained triangulation is carried
to the upper sample by that correspondence, and the convex hull of its lower
and upper vertices is formed. The margin is applied exactly once as the four
corners of an axis-aligned square of half-width `sqrt(2)*safetyMargin_units`
at every hull vertex: the constructor protects samples with
`polybuffer(...,'JointType','square')`, and a square join of distance `d`
reaches at most `d*sqrt(2)` from any source point, so that square contains
it. Every triangle is kept. A balanced Boolean union of the hulls is the
interval's swept union.

For any triangle point carried by its barycentric map, `(1-tau)*a+tau*b`
lies in `conv(startTriangle union endTriangle)`. The complete conforming map
covers the interpolated polygon whenever its boundary is simple, and covers
the map's image even if it folds. Every boundary source vertex must
participate; an omitted vertex that later moves would invalidate that
argument. This is an explicit conservative interval model, with
over-approximation determined by vertex displacement and by the
square-versus-buffer margin. It is not an exact moving polygon and is never
reported as interpolated topology.

The swept union is then covered at the interval's own resolution so that
coastline-scale detail does not multiply solver cells: the union is clipped
to grid squares of side equal to the interval's largest vertex displacement
plus the margin square, and each clipped piece is replaced by the convex
hull of its vertices. Each hull contains its piece and lies inside its
square, so the cover contains the union and fills only concavities narrower
than a displacement the sweep has already blurred. The side is also never
smaller than the union's extent divided by a declared budget of 256 grid
squares per interval; like the visibility search's pair-work budget, this
bounds solver work explicitly, and a coarser cover is only more conservative.
The interval's enclosure is the union of the cover pieces. Its cells are the
exact longest-shared-edge-first convex repartition of that union (the same
rule as exact intervals), so point queries and cells agree exactly and the
cell count follows the enclosure's shape rather than the grid.

Preparation still independently checks both authoritative protected sample
shapes against the enclosure, using the existing endpoint area-certificate
roundoff tolerance. Failure discards the candidate geometry and records
`sweptEnvelopeExcludesProtectedSample`, along with
`IntervalSweptUncoveredProtectedArea_units2`. Such intervals remain
`unsupportedContinuousDeformation`; the planner returns the existing stable
`unsupportedObstacleInterpolation`. It does not enlarge the prescribed cells,
shrink protection, or replace the samples. Unequal original counts or an
unavailable single-ring source map also remain unsupported.

Vertex correspondence between samples is reported per obstacle in
`vertexCorrespondence`. The generic constructor normalizes that declaration
into the logical `UsesSourceIndex` field consumed by preparation. It defaults to
reported `circularCorrelation`, which recovers a cyclic shift and orientation by
centered, scaled circular correlation. `createMovingObstacle` passes reported
`sourceIndex` into the generic constructor before normalization because every
sample is one source ring under the caller's transform. Its returned public
record therefore reports `sourceIndex` and has `UsesSourceIndex` true. This
prevents correlation from undoing a supplied rotation by a cyclic index shift.
The retained name reports provenance and does not select later behavior.

For a certified swept interval, cells are static over its absolute active
interval, interior point queries return their union and union-boundary edges,
`TopologyIsInterpolated` is false, and the interior speed bound is zero.
Sample-time queries retain the normalized protected sample, with an infinite
speed bound at a possible enclosure discontinuity. Closed-interval cells
conservatively include those samples; sample shapes are not overwritten to
make queries equal. This explicitly rejects D4's premise. Temporal visibility
checks the stationary union over the traversed part of its lifetime. Plots,
snapshots, occupancy queries, time cells, and independent source-rebuilt
validation all use this same interpretation. Preparation records candidate
face counts before/after union and partition/hull/union/repartition timings
in `IntervalSweptCellCount` and `IntervalSweptTiming_s`, including rejected
candidates; timing and provenance never select planner behavior.

Preparation also canonicalizes redundant corresponding keyframes without
editing `time_s`, `x_units`, or `y_units`. Consecutive velocities must agree
componentwise within `64*eps(coordinateScale)` coordinate units per second,
with unchanged per-source-interval alignment. The current reduction requires
that alignment to preserve the supplied index order. Crucially, it does not
realign the distant span endpoints: that can select a different motion.
One exact certificate and shared face-index partition must hold over the
complete span; its restrictions certify every constituent interval with the
same partition. A span without that certificate is not merged. The supplied
sample geometry remains authoritative at each retained sample time.

`IntervalGeometryModel` retains the reported model name. Preparation also emits
`IntervalHasExactPartition`, `IntervalIsStationary`, `IntervalUsesSweptCells`,
and `IntervalIsUnsupported`; geometry, search, and planner branches consume
those logical facts rather than the reported name. Exact partition consumers
read `IntervalStartRegions_units` and `IntervalEndRegions_units`.

`MergedSpanTime_s` and `MergedIntervalCount` expose the preparation-only
reduction. `RejectedMergeSpanSampleIndex` records spans
without a shared exact partition. Cells use certified span boundaries and
interior queries use the merged affine motion; search time layers keep every
supplied keyframe, exactly as for unmerged histories. Source
interval model and coverage arrays remain indexed by the supplied history.
Any query window touching a candidate span prepares its complete span so that
independent rebuilds clip identical cells. This can extend preparation beyond
the requested window, but never changes obstacle activity or source samples.

Preparation accepts an optional closed time window. It retains the complete
normalized source history and prepares only in-window samples and the two
endpoints of each overlapping source interval. Reusing a source-checked cache
extends this coverage without discarding earlier prepared entries. Exact point
queries prepare that sample, or both bracketing samples and their interval.
Single-sample static obstacles remain active at every time.

`SamplePrepared` and `IntervalPrepared` distinguish cached entries from entries
not yet requested. Unprepared neighboring intervals give a sample an infinite
speed bound unless the complete history is known to be static. Public queries
prepare their requested times; internal prepared queries reject missing
coverage. Planning, validation, and plotting request their physical window;
independent validation rebuilds preparation from source geometry.

Generic single-ring correspondence (reported as `circularCorrelation`) uses
centered, scaled circular correlation in both
orientations, with O(N log N) alignment work; a declared `sourceIndex`
correspondence skips alignment. Numerically tied shifts
are selected at a fixed physical anchor so cyclic starting indices do not
choose a different interpolation. There is no exhaustive-shift fallback.
Correspondence still needs a continuous convex or conforming-partition
certificate. Unsupported deformation is reported explicitly. Correlation alone
is not a motion or occupancy certificate.
