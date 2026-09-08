# Production consolidation

The public API is now `planner(obstacles, initialState, goalState, limits, options)`.
Target histories enter through `goalState.targetMotion`; goal time and the ordinary
timing mode select a fixed meeting or the latest search time. Chronological later
interception, specified derivative matching, explicit terminal derivatives, and
independent validation are preserved. Examples, tests, sandbox callers, and
benchmark defaults use the new entry point. Removed APIs have no forwarding shims.

Optional diagnostics retain complete structured records. Recursive table flattening
and its per-attempt concatenation are deleted; test-only adapters inspect the same
leaf evidence. The public validator still rebuilds authoritative obstacle geometry.
The independent raw-record assembly test includes nested certificates, arrays,
failure reasons, and multiple attempts.

## First consolidation checkpoint

Production has 113 files and 15,512 physical lines: 11,231 code, 3,292 comment,
and 989 blank lines. This includes the root planner and every production package.
The 15,528-line physical baseline is a measurement rule, not an executable-code
claim. Removing 621 content-free comments and shortening repeated help text explains
much of this checkpoint's physical reduction. The user requested more aggressive
deletion of redundant implementations, so this checkpoint is not completion.

The first full migration run passed 213 of 215 tests. The two failed assertions
were old source hashes (only the planner call name changed) and an old MATLAB
error label. After those corrections, 216 of 217 tests passed; the new synthetic
diagnostics fixture omitted the orchestrator's SelectionPolicy field. Fixing that
fixture passed all five output tests. No behavioral or physical acceptance gate
was weakened. A complete final suite follows the deeper consolidation.

Both pinned cleanup and this checkpoint received the same 20 frozen physical
requests, three untimed calls requesting two outputs, and three timed calls.
All 60 comparisons pass the original outcome, input, option, independent-validation,
arrival, and adaptive-length gates. A second comparison with the visibility-stage
capture confirms every arrival and adaptive arc is unchanged by consolidation.
Raw captures are `scratch/final_matched_before.mat`, `final_matched_after.mat`,
and their quality reports. These filenames identify a checkpoint, not completion
of the user's subsequent request for deeper deletion. All repetitions, including
unfavorable timings, remain in the benchmark history.

Next: share the numerical motion law, event integration, polynomial format and
evaluation, and independent scalar range checking across their existing callers.
Retain the general switching equations for nonzero endpoints and the broader
obstacle formulations where reduced clocks do not cover the request.
