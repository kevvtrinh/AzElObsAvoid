# Less-NLP spline research status

This directory is research-only. `planAzElMotion` still uses the maintained
HS3 path, and no result from this directory is a learned safety certificate.
The unchanged `validateAzElTrajectory` function is the acceptance authority.

## Evidence-gated outcome

- Phase A froze the HS3 1/2/5/10/20-turn baseline in
  `benchmarks/repeated_turn_hs3_phase_a.csv`.
- Phase B compared an open quintic B-spline with a fixed-stop septic Bezier.
  Both passed their scoped representation checks, but septic required
  28–84 seconds of motion on multi-turn routes versus 8.94–18.26 seconds for
  quintic and was incompatible with the maintained quintic validator. Its
  implementation and candidate-only tests were removed after rejection; the
  measured rows remain in `benchmarks/spline_representation_phase_b.csv`.
- Phase C retained the bounded quintic prototype in this directory. The
  deterministic optimizer passed maintained validation for 1, 2, and 5 turns
  with 1, 2, and 6 decision variables. The accepted rows are in
  `benchmarks/low_dimensional_spline_phase_c.csv`.
- The 10-turn gate failed. The best bounded retry remained colliding and took
  63.35 seconds; an earlier mean-penalty form took 131.70 seconds and also
  failed. The 20-turn candidate was not run because the prerequisite gate had
  already failed.
- Supervised imitation, reinforcement learning, production planner
  integration, and HS3 deletion were therefore not authorized by evidence and
  were not performed.

## Files

- `buildQuinticBsplinePrototype.m` converts an open degree-five B-spline into
  the maintained piecewise-polynomial schema and analytically retimes it from
  continuous Bernstein derivative bounds.
- `optimizeQuinticBsplinePrototype.m` uses bounded normal-offset coordinate
  search. Sampled clearance guides search only; continuous public validation
  decides success.
- `testQuinticBsplinePrototype.m` covers straight, 45-degree, 90-degree,
  S-turn, horseshoe, and five-alternation routes.
- `testQuinticBsplineOptimizerPrototype.m` covers exact static-obstacle
  success, determinism, and an expected impossible-horizon failure.

Run the retained research checks from the repository root:

```matlab
addpath(genpath(pwd));
runtests({ ...
    'scratch/learnedSplinePolicy/testQuinticBsplinePrototype.m', ...
    'scratch/learnedSplinePolicy/testQuinticBsplineOptimizerPrototype.m'});
report = benchmarkLowDimensionalSpline([1 2 5]);
```
