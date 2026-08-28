# References

## Homology-signature graph search

- Bhattacharya, S., Likhachev, M., and Kumar, V. (2012). "Search-Based Path
  Planning with Homotopy Class Constraints in 3D." *Proceedings of the
  Twenty-Sixth AAAI Conference on Artificial Intelligence*, 2097-2099.
  https://doi.org/10.1609/aaai.v26i1.8435

  The spatial visibility search augments each graph node with a bounded 2-D
  homology signature. The signature integrates principal angle changes about
  one interior representative of each connected sampled obstacle region. This
  preserves distinct route classes during Dijkstra search. It is a 2-D
  adaptation. It is not the paper's 3-D electromagnetic construction and is
  not a continuous Az/El/time homotopy certificate.

## Third-order direct collocation

- Moreno-Martin, S., Ros, L., and Celaya, E. (2024). "Collocation Methods
  for Second and Higher Order Systems." *Autonomous Robots*, 48, Article 2.
  https://doi.org/10.1007/s10514-023-10155-z

  The HS3 optimizer uses a separated third-order chain. Quadratic jerk
  ordinates are integrated once to produce acceleration, velocity, and
  position polynomials. This prevents independent state splines from
  violating the shared dynamics.

## Non-stopping waypoint-state refinement

- Koskela, P. `rsruckig`, MIT-licensed Rust motion-planning library.
  https://github.com/petrikosk/rsruckig

  The optional `passThrough` waypoint warm start is a MATLAB adaptation of
  the local waypoint-state search in `calculator_waypoints.rs`. It estimates
  nonzero interior velocities, chains exact jerk-limited state-to-state
  sections, and boundedly refines shared waypoint velocity and acceleration.
  This repository does not embed or call the Rust implementation. HS3 and the
  independent obstacle validator remain authoritative for feasibility.

## Scenario smoothstep motion

- Perlin, K. (2002). "Improving Noise." *Proceedings of SIGGRAPH 2002*,
  681-682. https://doi.org/10.1145/566570.566636

  Maintained moving-obstacle examples and stress benchmarks use the smoothstep
  blend `10*u^3 - 15*u^4 + 6*u^5` to create deterministic scenario motion
  with zero endpoint velocity and acceleration. This is input construction,
  not a planner motion method.

## Bernstein polynomial bounds

- Farouki, R. T. (2012). "The Bernstein Polynomial Basis: A Centennial
  Retrospective." *Computer Aided Geometric Design*, 29(6), 379-419.
  https://doi.org/10.1016/j.cagd.2012.03.001

  The solver and independent validator convert segment-local power
  coefficients to Bernstein form. They use the convex-hull property to
  bound complete polynomial intervals instead of checking samples only.
