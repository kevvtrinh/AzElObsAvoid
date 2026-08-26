# HS3 validation ownership

No HS3-local validation or first-motion certificate implementation remains.
Seed-corridor construction, certification, and obstacle-envelope checks are
owned by neutral `azElInternal` helpers.

Root `validateAzElTrajectory` is the final authority for every returned timed
trajectory. `azElPlannerMethods.hs3.validateTrajectory` is a compatibility
facade only.
