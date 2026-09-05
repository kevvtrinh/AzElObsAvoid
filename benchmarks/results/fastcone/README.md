# Accepted fastcone runtime measurements

These are the completed research measurements accepted before branch
integration, not a new repeated benchmark of the integration checkout.
MATLAB R2024b ran a source-hashed snapshot of the then-current HS working
folder, with one warmup per method and three interleaved measured repeats.
The selected engine is the MATLAB/MEX blocks adapter. The adjacent manifests
retain the measured source identities. Raw MAT inputs, returned trajectories,
dispatch traces, and MATLAB metadata remain in the research workspace's
`output/hs-examples-blocks/`, `output/hs-examples-blocks-saved/`, and
`output/blocks-corpus.mat`; large raw artifacts are not vendored here.

Full-planner times include internal validation, failed attempts, and coneprog
recovery. They exclude example construction, plotting, saving, and the
harness's additional independent validation. Maintained examples use their
default physical inputs and options; saved requests use the recorded inputs
and options. Only cases with measured coneprog calls are reported. The
geographic example includes all three regional planner calls in its total.

| Example | coneprog planner (s) | fastcone planner (s) | Speedup |
|---|---:|---:|---:|
| exampleAlternatingSlalom | 4.258529 | 1.183775 | 3.597x |
| exampleNoPath | 0.292639 | 0.213145 | 1.373x |
| exampleObstacleAvoidance | 1.886139 | 0.432482 | 4.361x |
| exampleStaticUShapedObstacle | 16.942329 | 2.402593 | 7.052x |
| exampleStraightTargetAlternatingOcclusion | 3.257662 | 1.544471 | 2.109x |
| exampleTargetExitsObstacle | 8.441004 | 3.360929 | 2.512x |
| exampleTwoOpposingUVisibilityGraph | 2.455841 | 1.313832 | 1.869x |
| exampleUSOutlineExtremeVisibility | 10.562226 | 6.612997 | 1.597x |
| Rogue Examples/failed.mat | 31.403219 | 4.692790 | 6.692x |
| Rogue Examples/inefficientroute.mat | 4.653802 | 0.878943 | 5.295x |
| Rogue Examples/inefficientroute_2.mat | 4.440025 | 0.837377 | 5.302x |

Summed medians are **88.593414 to 23.473334 seconds: 3.774x, 73.50% less
full-planner runtime**. Maintained and saved subsets are 2.819x and 6.319x.
Within each case, the slowest measured candidate repeat was faster than the
fastest measured baseline repeat. The adjacent `maintained-runtime.csv` and
`saved-runtime.csv` retain ranges and recovery counts.

All 96 returned trajectories across warmups and repeats passed independent
validation. Eight NoPath runs preserved the expected `noValidatedSeed`
outcome; they measure failed search, not a successful motion or an
infeasibility proof. All 104 result records matched baseline inputs/options.
The Hawaii subplan passes continuous collision validation without a plane
certificate in both methods.

Quality is not identical. Alternating target occlusion travels 0.018561612
degrees farther at the same 20.869565217-second arrival. The saved `failed.mat`
balanced objective worsens by 0.021893742 degrees. The two inefficientroute
requests improve their configured balanced objectives by 1.153389 and
6.092522 degrees while arriving later and traveling less. These differences
were disclosed before the user accepted integration. Feasibility alone is
not evidence of equivalent trajectory quality or global optimality.

## Identical-program conic replay

A separate replay holds all 1,028 conic programs fixed. Summed per-program
medians were **80.832102 to 8.501165 seconds: 9.50836x**, including recovery.
There were 852 recoveries among 3,084 measured candidate calls.

| Profile family | Speedup |
|---|---:|
| Earliest arrival | 22.200x |
| Balanced arrival | 18.693x |
| Separating planes | 1.723x |
| Fixed arrival | 1.158x |

`conic-cases.csv` retains every case, including the degree-7, eight-span
fixed-arrival slowdown to 0.656x. `conic-profiles.csv` includes flags and
independent primal-check failures. Forty-one programs had a positive candidate
flag but exceeded the separate original-unit primal tolerance on at least one
repeat; every such result came through original coneprog recovery. Four
programs had nonpositive candidate flags. They are retained in
`conic-validation.csv` and are not counted as native acceptance.

Neither replay nor planner measurement establishes universal acceleration,
quality equivalence, a fully analytical trajectory engine, or completion of
the initial 10x overall target. Integration checks are recorded separately in
the repository's `verification.md` and `benchmark.csv`.
