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
region and are retained. This is coordinate normalization, not simplification
or a repair of self-intersections. Three or more distinct vertices alone are
not proof that a polygon is valid.

Original and protected histories follow the same normalization rules and stay
distinct. An absolute safety margin is rebuilt from original geometry, never
added repeatedly to protected geometry. `NormalizationDiagnostics` records
removed-run and duplicate counts by role, fixed reason codes, and affected
sample indices/times. Lists contain at most one entry per history sample.
Canonical rebuilds preserve these diagnostics for the same time history;
diagnostics are informational and never determine obstacle occupancy.

At a sample time, its normalized protected geometry is authoritative. An empty
sample does not clear either neighboring interval. Between samples, the current
preparer interpolates verified corresponding vertices for supported motion.
Other intervals use explicitly labeled static-equivalent, nested endpoint-union,
or endpoint-hull models. These models describe the declared sampled history;
they are not a continuous physical coverage guarantee for arbitrary unsampled
projected motion. Preparation, time cells, queries, and independent validation
must retain the same interval interpretation.

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

Generic single-ring correspondence uses centered, scaled circular correlation
in both orientations, with O(N log N) alignment work. Numerically tied shifts
are selected at a fixed physical anchor so cyclic starting indices do not
choose a different interpolation. There is no exhaustive-shift fallback.
Correspondence still needs the existing translation or convex-interpolation
verification; unsupported deformations retain an explicitly named conservative
interval model. Correlation alone is not a motion or occupancy certificate.
