# Prepare obstacles once — retained

Decision, 2026-09-05: prepare original histories at the public planning boundary;
reuse them in internal geometry and validation calls. Public calls still detect
changed data, and derived static projections are prepared when constructed.

MATLAB R2024b; four deterministic cases, warmup, two timed repeats, and separate
profiling. Baseline was `4344795` plus working changes, not a clean commit.

| Case | Preparation calls before → after | Median wall seconds before → after |
| --- | ---: | ---: |
| Direct motion | 3 → 1 | 0.250301 → 0.255162 |
| Static detour | 5575 → 1 | 1.81801 → 1.33100 |
| Moving barrier / wait | 961 → 1 | 0.476463 → 0.381705 |
| No path | 39 → 1 | 0.290346 → 0.287972 |

All retained motion and independent-validation fields matched exactly except
elapsed time. Geometry, tolerances, margins, and search budgets were unchanged.
The benefit is eliminated repeated preparation; two repeats do not establish a
general runtime improvement. The direct case was slightly slower.

The accompanying refactor separates `[result, diagnosis]`, removes unused stage
arguments and duplicate records, and updates consumers. See
[verification](../../verification.md) for the completed test/example checks.
Detailed historical prose is in Git; recorded example metrics remain in
[benchmark.csv](../../benchmark.csv).
