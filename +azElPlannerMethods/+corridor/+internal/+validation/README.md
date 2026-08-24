# Corridor validation ownership

No corridor-local certificate or final-validation implementation remains.
`azElInternal.buildSeedCorridor`, `certifySeedCorridor`, and
`seedEnvelopeContainsObstacles` own reusable corridor evidence outside motion
construction.

Root `validateAzElTrajectory` performs final complete timed-trajectory
validation. `azElPlannerMethods.corridor.validateTrajectory` is a compatibility
facade only.
