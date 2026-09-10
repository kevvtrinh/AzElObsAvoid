# Saved moving-detour diagnosis

The preserved request is `tests/fixtures/savedMovingDetour.json`, copied from
`RogueCasses/x-y-request.json`. It contains a translating rectangle with 21
keyframes, a rotating 38-vertex concave obstacle, a 180-second horizon, and the
original per-axis motion limits and safety margins.

## Retained dense-history path

Dense, fixed-position, rest-to-rest earliest-arrival requests first build a
time-expanded route proposal. The spatial nodes come from the sampled protected
obstacle union. The time layers contain each source time, each source-interval
midpoint, and nine uniform horizon samples. Forward search enumerates motion and
wait transitions and selects the first reachable goal layer. It does not compute
homology or enumerate homotopy classes.

The selected route is only a BMTP seed. A 16-span, degree-eight timed BMTP solve
discovers colliding span-region pairs by sampling and activates a maximum-margin
separating line for each newly discovered pair. It then refines travel at the
selected arrival clock. Success still requires a new certificate against the
exact affine time cells followed by the public independent validator. The
proposal's sampled visibility checks cannot approve a motion.

The timed path is used when the source history contains at least 16 intervals,
so its fixed mesh compresses the input history. Sparse histories retain the
existing chronological fixed-arrival search and validated delayed-chord
incumbent. If the timed proposal or BMTP solve fails, the same fallback runs.

## MATLAB R2024b measurements

All runs replayed the identical saved JSON with plots and profiling disabled.
The full public `planner` call includes route search, BMTP, exact recertification,
independent validation, and result assembly.

| Implementation | Full planner runtime (s) | Arrival (s) | Motion length (units) |
| --- | ---: | ---: | ---: |
| Prior chronological branch | 236.291 | 134 | 240.730295011 |
| `bmtp-cleanup-codex` (`c04f3b2`) | 55.443810, 49.511337, 48.159892 | 117 | 229.949945793 |
| Retained timed path | 16.987219, 14.909188, 14.412398 | 117 | 229.959020399 |

The retained median is 14.909188 seconds: 15.85 times faster than the prior
chronological run and 3.32 times faster than the cleanup branch median. A final
post-audit replay took 14.843730 seconds. Relative to cleanup, arrival is
identical and motion length increases by 0.009074606 units (0.00395%).

The solver considered 188 applicable span-region pairs, tagged 13, performed 11
trajectory SOCPs and 78 separating-line SOCPs, and exported a degree-eight C3
motion. The exact final certificate and public validator both passed.

The earlier roughly seven-second observation was a fixed-arrival active-set
prototype, not the full earliest-arrival planner. That generic prototype was not
retained because it changed the maintained 220-vertex path by 0.00134342 units
and changed its plane-update behavior. The dedicated timed path preserves the
220-vertex fixture exactly.

## Limits of the earliest-arrival claim

Arrival 117 seconds is the earliest reachable layer in this deterministic
time-expanded graph. `GlobalEarliestProven` remains false because the finite
layer set and sampled proposal edges do not prove a continuous-time global
minimum. Collision freedom and motion limits are exact for the returned
polynomial certificate; optimality between unsearched times is not claimed.

## Batched occupancy milestone

Profiling the committed timed planner attributed 12.26 seconds to route search,
including 8,702 public occupancy calls. Rechecking obstacle preparation inside
those calls consumed 4.12 seconds. The moving-scene search now batches each
edge's original thirteen samples into one public query, bounded to 262,144
sample positions per batch. Sample positions, times, boundary policy, candidate
transitions, and final certification are unchanged.

Fully batching stationary histories was rejected: a 49-layer crossing-barrier
search increased from a 0.282-second median to 1.466 seconds. The retained
change keeps the original occupancy cache whenever the prepared history has a
stationary interval, using batching only for continuously changing geometry.

Three paired search-only runs on the saved request had medians of 10.873656
seconds before and 7.871029 seconds after. Routes, clocks, and every search
record field matched exactly. Two structurally different stationary-history
comparisons (13 and 49 uniform layers, plus obstacle events) also matched
exactly and retained cache performance.

Full planner runs took 13.833663, 11.554937, and 11.078550 seconds, a median of
11.554937 seconds (22.5% below the previous 14.909188-second milestone). Each
returned exactly the previous polynomial, arrival 117 seconds, motion length
229.959020399 units, and a passing independent validator. The production change
adds only 16 lines to the existing search function.

## Regression coverage and code size

The suite now contains 30 MATLAB tests. The saved-request regression checks arrival 117,
independent validation, timed-route selection, and active-pair reduction.
Structurally different regressions retain the nine-second moving-circle detour,
the 82.5-second long request, the validated waiting incumbent, and the exact
220-vertex length of 121.503236303671 units. A crossing-barrier regression covers
both continuously moving and stationary source intervals, preserving the
reference graph's departure and arrival times.

The retained implementation adds 1,749 lines in new production files and 81 net
lines in existing production files, for a net production increase of 1,830
lines. Reusing the existing boundary-only exact graph was tested as a smaller
alternative, but the saved request did not complete within one minute; the
measured 14.9-second route implementation was retained.
