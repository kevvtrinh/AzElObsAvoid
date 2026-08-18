# HS3 refactor baseline

- Branch: `hs3-refactor`
- Starting commit: `e46ccae6343a8127d303211a0d1134754a847bc2`
- MATLAB: R2024b Update 4
- Optimization Toolbox: `fmincon` and `intlinprog` available

## Starting size

| Category | Files | Lines |
| --- | ---: | ---: |
| Production | 27 | 18,090 |
| Examples | 17 | 2,989 |
| Tests | 3 | 2,521 |
| Benchmarks | 2 | 699 |
| Scratch | 1 | 137 |

## Headless baseline cases

| Case | Planner | Validation | Termination | Runtime (s) |
| --- | --- | --- | --- | ---: |
| Obstacle-free specified-time intercept | success | pass | `goalReached` | 3.020331 |
| Static circular obstacle | success | pass | `goalReached` | 14.920845 |
| Translating circular obstacle | success | pass | `goalReached` | 5.945178 |
| Full-height static wall | failure | stable failure | `noFeasibleCandidate` | 1.396432 |

The first obstacle-free run also passed but the reporting expression used a
field name that does not exist. The corrected run above supplied the recorded
metrics. No planner result changed because of the reporting error.
