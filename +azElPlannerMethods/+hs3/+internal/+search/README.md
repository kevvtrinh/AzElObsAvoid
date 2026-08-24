# HS3 search

No HS3-local search implementation remains. The compact baseline produces the
bounded visibility, homology, and timed-search seed portfolio through neutral
`azElInternal.generateTopologySeeds`. Seeds are proposals and never establish
planner success.

When improvement is enabled, `hs3.improve` reuses the immutable baseline's
seeds in deterministic order. HS3 retains only nonlinear motion work;
candidate validation and strict improvement comparison use canonical owners.
