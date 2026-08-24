# HS3 search ownership

No HS3-local graph-search implementation remains. Standalone `hs3.plan`
obtains bounded visibility, homology, and timed-search proposals directly from
the neutral `azElInternal.generateTopologySeeds` owner.

Seeds are proposals and never establish planner success. HS3 assigns its own
collocation mesh, solves each attempted proposal with its Hermite-Simpson
transcription, and requires canonical validation before selection. The compact
planner is not called and contributes no seed, result, warm start, fallback, or
acceptance decision.
