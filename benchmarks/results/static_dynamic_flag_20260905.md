# Keep automatic static-scene detection

Decision, 2026-09-05: do not force static scenes through dynamic search.
The flag changes route search and candidate construction, not just runtime.

MATLAB R2024b; 14 static planner scenes from 12 maintained examples; one warmup
per mode and three interleaved measured repeats. Original geometry, finite jerk
limits, and arrival policies were retained. Timings cover planner calls only.
Baseline was `4344795` plus frozen working edits; production was not changed.

| Scene | Normal → forced-dynamic median seconds | Motion effect |
| --- | ---: | --- |
| Philippines | 13.437 → 115.169 | Length 23.354 → 23.953 units; duration 5.796 → 6.216 s. |
| Croatia | 3.009 → 13.715 | Same returned motion. |
| Alternating target occlusion | 2.862 → 11.087 | Slightly longer travel at the same arrival. |
| Target exits obstacle | 7.746 → 13.293 | Length 20.685 → 21.940 units at 24 s. |
| Static U | 9.344 → 9.022 | Longer path and later arrival; timing ranges overlap. |

All 78 successful timed calls passed independent collision, kinematic, and
applicable certificate checks. The six timed NoPath calls returned failures:
normal `noValidatedSeed`, forced `unsupportedTimedMultiWaypointRoute`.
The Philippines forced warmup exhausted its shared example budget; subsequent
measured calls succeeded in 98.229–121.865 s. This is not an infeasibility result.

Exact-motion exits remained enabled, and successful dynamic seed solves used
static-projection BMTP; the experiment did not force the timed-cell kernel.
Small timings on shared early exits do not measure a solver advantage.

All 112 comparison calls plus an initial warmup remain in
[benchmark.csv](../../benchmark.csv). Certificate reporting was corrected from
retained validation records without weakening checks. Older detailed tables and
experiment notes remain in Git history.
