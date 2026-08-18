# Azimuth-Elevation RP Seed Model

`azElRpRetimerAgent.mat` is the required production seed policy. The planner
loads this model for every request. There is no deterministic radius fallback.

The model is a DDPG policy. It receives one 10-element observation for each
interior route corner. It returns a radius fraction in `[0, 1]`. The planner
uses the fraction to create a G3 Bernstein seed. The HS-3 optimizer can change
the seed geometry and timing, but independent collision and kinematic checks
have final authority.

The observation order is:

1. Turn deflection divided by pi.
2. Minimum adjacent length divided by maximum adjacent length.
3. Bounded logarithm of the minimum adjacent length.
4. Requested turn radius divided by the minimum adjacent length.
5. Incoming azimuth direction component.
6. Incoming elevation direction component.
7. Outgoing azimuth direction component.
8. Outgoing elevation direction component.
9. Acceleration time scale divided by the reference time scale.
10. Jerk time scale divided by the reference time scale.

The checked-in metadata identifies the model format, MATLAB release, training
seed, and validation results. The branch does not retain a second production
retimer or an obsolete training pipeline.
