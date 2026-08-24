# Standalone Hermite-Simpson restoration

## Objective and hard gates

Keep `corridorQuintic` and `hs3` as separate motion methods. HS3 must use an
actual Hermite-Simpson transcription and must not call, seed from, fall back to,
merge with, or return the compact planner. HS3-owned production must remain at
or below 2,000 nonblank, noncomment MATLAB lines. The public standalone HS3 path
must solve and independently validate the 12-hairpin benchmark in under 120
seconds while preserving nonzero endpoint velocity and acceleration, moving
targets, timed waits, and stable expected no-path results.

## Final implementation

- `planAzElMotion` dispatches directly to either compact or standalone HS3.
- HS3 generates neutral input-derived topology seeds, solves third-order
  Hermite-Simpson NLPs, and accepts only canonical independently validated HS3
  motion.
- Exact affine sensitivity and constraint matrices remove decision-variable
  finite differencing for fixed-time solves and most earliest-arrival columns.
- Collision-only failures receive at most two relinearizations per mesh and one
  bounded mesh refinement. Static scenes stop at the first validated topology;
  changing scenes compare completed timed candidates inside one cooperative
  planning budget.
- Nonzero initial and final position, velocity, and acceleration remain part of
  the transcription and derivative tests.
- Deleted composition-only improver, old options, and duplicate request and
  endpoint normalization.

## Final evidence

- HS3 production: 1,602 noncomment lines across nine MATLAB files; zero compact
  planner calls, warm starts, fallbacks, merged results, or composition fields.
- 12 hairpins: actual HS3 success and independent validation in 93.339104 s
  total, below the 120 s gate; arrival 128.179676226 s versus frozen compact
  arrival 140.56091613 s.
- Repeated turns, total wall / arrival: 1 turn 3.776525 / 6.611054 s; 5 turns
  7.563524 / 18.622669 s; 10 turns 63.799021 / 33.572536 s; 20 turns 64.116174
  / 114.561540 s. All four independently validated.
- Maintained examples: 18/18 scenario outcomes passed, including 17 successful
  HS3 motions and the expected diagnosable no-path result.
- Automated tests: 137 passed, 0 failed, 0 incomplete in 360.861356 s.
- Exact HS3 sensitivity tests: 6/6. Changed-source Code Analyzer: zero messages.
  `git diff --check`: clean.

## Honest limitations

- HS3 is not uniformly better than compact: it is 0.006849 s later on one turn
  and 46.202735 s later on 20 turns using the frozen compact arrivals.
- First-valid static selection and fixed-time treatment of timed topology seeds
  can miss a better local topology or arrival.
- The planning deadline is cooperative, and difficult earliest-arrival NLPs can
  still emit conditioning warnings. Independent validation, not optimizer status,
  remains authoritative.
- Finite topology search and local nonlinear optimization provide neither a
  completeness nor a global optimality certificate.
