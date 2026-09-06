# Historical fastcone measurements

Decision: fastcone was removed. Production uses degree-eight curves and MATLAB
coneprog. These CSVs describe older research, not the current engine.

The warm full-planner study used MATLAB R2024b, one warmup and three interleaved
repeats. Summed medians improved 3.774x across the recorded maintained/saved cases,
but recovery was included and motion quality was not identical. All 96 returned
motions independently validated; eight NoPath runs remained expected failures.
This does not establish current cold performance or universal acceleration.

Fixed-program conic replay measured 9.508x summed-median improvement, with 852
recoveries in 3,084 measured calls. Some programs slowed down; 41 programs had
positive flags but failed independent primal tolerances on at least one repeat.
Those outcomes remain in the CSVs and are not native acceptance.

Keep adjacent CSV measurements and source manifests in full. Raw research inputs
were deleted at the user's request; exact replay requires recapturing them.
See [current decisions](../../../branch_assessment.md) and
[verification](../../../verification.md). Earlier narrative remains in Git history.
