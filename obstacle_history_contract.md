# Obstacle History Contract

This document defines how every public planner, query, validator, and plotting
function interprets a canonical obstacle history. Geometry is conservative:
when the supplied samples do not support the direct interpolation model, the
planner uses an enclosing occupied set rather than guessing a less restrictive
motion.

## Canonical Samples

Each obstacle contains strictly increasing `time_s` samples. Matching
`az_deg` and `el_deg` cells contain protected polygon boundaries in degrees.
Paired nonfinite rows separate rings. A ring is represented by its finite
vertices without a repeated closing vertex.

At an exact sample time, the supplied protected sample is authoritative.
Geometric nesting determines filled regions and holes; ring orientation and
the choice of first vertex are representation details, not occupancy changes.
Safety margins are applied to original sample geometry exactly once before
history evaluation.

A history with two or more samples is inactive before its first sample and
after its last sample. A one-sample history represents permanent static
geometry. The `status` field is retained metadata. Values such as `"hidden"`
do not deactivate supplied geometry because sensor visibility is not evidence
that the physical obstacle disappeared.

## Between-Sample Motion

The supported direct model is linear corresponding-vertex motion. It does not
claim rigid translation or rigid angular motion. For example, vertices of a
sampled rotating rectangle follow straight chords between sample positions;
they do not follow circular arcs.

Before computing vertex deltas, preparation may remove representation-only
differences from one simple ring:

- cyclically shifting the first vertex; and
- reversing traversal orientation.

The alignment with the smallest total squared vertex displacement defines the
repository correspondence. Equal vertex counts alone do not establish that
correspondence. Direct interpolation is used only when the two samples each
contain one finite, nondegenerate ring with the same vertex count and the
aligned intermediate boundary remains a supported simple occupied polygon.

Multiple rings, holes, changed vertex counts, degenerate rings, and any
unverified alignment use the conservative fallback. That fallback is the
convex hull of all finite endpoint vertices, occupied for the complete source
interval. It may fill holes or bridge disconnected regions. This loss of free
space is deliberate and is reported as an enclosure rather than mislabeled as
exact topology interpolation. It also covers the gap between separated
endpoint shapes; a union of endpoints alone is not a valid swept enclosure.

`TopologyIsInterpolated` is true only for direct linear interpolation.
Fallback geometry is a time-invariant conservative occupied set within its
source interval, so its geometric speed bound is zero. The source model is not
being declared stationary: the enclosure already contains every admitted
between-sample position.

## Request-Horizon Geometry

Proposal and broad-phase geometry for `[startTime_s, endTime_s]` contains only:

- shapes at the exact request endpoints when the history is active there;
- stored samples inside the closed request horizon; and
- conservative interval geometry for every source interval intersecting the
  request horizon.

Stored samples and intervals completely outside the request horizon do not
enlarge its projection or bounds. Horizon restriction changes proposal work,
not authoritative validation: the validator continues to query the original
protected history at physical trajectory times.

## Unsupported Intent

The canonical format does not encode material vertex identifiers, a rigid-body
pose, birth/death semantics, or visibility-driven activity. Callers needing a
different physical model must sample a conservative protected boundary or add
an explicit future model field. The planner does not infer those meanings from
array shape, filename, target name, or status text.
