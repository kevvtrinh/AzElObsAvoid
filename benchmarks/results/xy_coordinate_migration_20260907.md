# Generic x/y coordinates and independent periodic axes

Date: 2026-09-07. MATLAB R2024b on the same Windows host. Baseline: 3fcd62c plus the existing working changes; comparison used an isolated copy of every existing tracked file and the supplied untracked bundle. No user fixture deletion was restored. Full numeric histories and the starting patch remain under ignored tmp storage.

## Contract and acceptance

The public planner, engine boundary, obstacles, validation, examples, both sandboxes, saved bundles, documentation, and plot labels now use [x y] and caller-consistent coordinate units. Physical fields use position_units, velocity_units_s, acceleration_units_s2, and jerk_units_s3. Rotation angles still use degrees. Both axes must use the same coordinate unit; no numerical rescaling was performed.

WrapX and WrapY default to false. Each enabled axis uses its own workspace interval width as the period. Normalization selects the nearest equivalent endpoint once; exact half-period ties select positive displacement and stay stable on revalidation. Returned histories remain continuous, while plots split every crossed x or y seam. Disabled axes retain continuous workspace-bound checks. Periodic obstacles and moving goals remain explicitly unsupported. Combined scalar derivative limits still allocate L/sqrt(2) to each axis, and mixing scalar and vector limit forms remains invalid.

Acceptance required exact preservation of unwrapped example physics and validation; independent x-only, y-only, both-axis, shifted unequal-period, half-period, and plotting checks; and no new unexplained test failures. No global winding or trajectory optimality is claimed.

## Maintained examples

All 18 were executed with finite jerk and headless controls before and after migration. Every route, time sample, position, velocity, acceleration, jerk, duration, arrival time, success flag, and termination reason matched exactly. Each success also passed fresh independent validation. Status below combines planner, independent validator, collision, kinematic, and applicable certificate status. NoPath is the expected failure and all five flags are false.

| Example | Polyline (units) | Smoothed (units) | Duration (s) | Status / termination | Baseline wall (s) | Candidate wall (s) |
|---|---:|---:|---:|---|---:|---:|
| exampleAlternatingSlalom | 16.019320 | 16.034754 | 10.550094 | all pass / goalReached | 5.652840 | 5.226113 |
| exampleDenseConcaveObstacle | 12.761105 | 12.761105 | 8.500000 | all pass / goalReached | 1.921521 | 1.975376 |
| exampleFourAcceleratingCircles | 20.000000 | 20.000000 | 22.000000 | all pass / goalReached | 2.638768 | 2.539993 |
| exampleInterceptMovingTargetAtSetTime | 9.538941 | 9.538941 | 12.000000 | all pass / goalReached | 0.103392 | 0.093730 |
| exampleInterceptMovingTargetEarliest | 7.308890 | 7.308890 | 6.111111 | all pass / goalReached | 0.194695 | 0.163569 |
| exampleMovingBarrierWait | 10.000000 | 10.000000 | 10.090302 | all pass / goalReached | 0.621773 | 0.534441 |
| exampleMovingCircleNoWrap | 12.453788 | 12.453788 | 8.500000 | all pass / goalReached | 1.141130 | 1.001221 |
| exampleMovingDeformingUSOutlineVisibility | 40.248219 | 40.248219 | 7.916667 | all pass / goalReached | 17.354295 | 17.701651 |
| exampleMovingRotatingObstacleField | 20.716209 | 20.716209 | 9.041667 | all pass / goalReached | 1.508375 | 1.778911 |
| exampleNoPath | NaN | NaN | NaN | all false / noValidatedSeed | 0.437757 | 0.425448 |
| exampleObstacleAvoidance | 11.152120 | 11.220422 | 7.572045 | all pass / goalReached | 1.019072 | 1.090926 |
| exampleObstacleFree | 4.472136 | 4.472136 | 4.531129 | all pass / goalReached | 0.027380 | 0.023135 |
| exampleOpeningUShapedObstacle | 10.000000 | 10.000000 | 13.617522 | all pass / goalReached | 1.230239 | 1.339093 |
| exampleStaticUShapedObstacle | 34.942588 | 38.678082 | 20.872548 | all pass / goalReached | 5.035029 | 4.683036 |
| exampleStraightTargetAlternatingOcclusion | 13.341664 | 13.610416 | 20.869565 | all pass / goalReached | 1.439815 | 1.457131 |
| exampleTargetExitsObstacle | 20.135789 | 20.685147 | 24.000000 | all pass / goalReached | 2.819171 | 2.868673 |
| exampleTwoOpposingUVisibilityGraph | 24.035785 | 24.205764 | 22.100628 | all pass / goalReached | 1.368238 | 1.338283 |
| exampleUSOutlineExtremeVisibility | 22.070647 | 23.257993 | 5.827605 | all pass / goalReached | 15.435104 | 15.689407 |

These are single example passes, not a speedup study. The arbitrary-period x wrapping case was warmed up and repeated three times in each revision: period 10, initial [4 0], raw goal [-4 0]. It changed from an 8-unit displacement and 6.5 s motion to a 2-unit displacement and 3.372281323269 s motion. Both validated. Median planning wall time was 0.004211 s before and 0.004167 s after; that small difference is not evidence of a runtime improvement.

The contract suite separately exercised exampleTargetExitsObstacle with clearance tolerance 1e-4 and MaximumSeedCount=2. Its original test run passed canonical validation; numeric submetrics were not retained. A full-metric repeat passed planner, independent validation, collision, kinematics, and certificates, terminating goalReached: polyline 21.742547 units; smoothed 21.932157 units; duration 24.000000 s; wall 5.334572 s. All actual runs, including the original test run's unavailable numeric fields, were appended to benchmark.csv.

## Tests and boundaries

- 172 distinct MATLAB tests have a final passing result, including eight wrapping regressions, scalar/separate derivative limits, both native sandbox controls, and continuous polynomial validation.
- 27 Node tests pass against the actual standalone page functions, including independent periods, simultaneous seams, stable half-period ties, and unchanged obstacle translation/rotation/stretch behavior.
- The existing testTimedRouteArrivalHandoff/testSavedGrowingObstacleUsesEarlierTurn fails with the same route-source and missing-diagnostic assertions in both the candidate and the untouched baseline. Motion succeeds and validates in both. Its supplied fixture was already modified before this task; assertions were preserved.
- Three fixture-dependent cases could not pass because failed.mat, pathtoolong.mat, and pathtoolong2.mat were already deleted. The first was attempted and errored on its missing file; the other two were excluded. Two duplicate maintained-example test wrappers were excluded while all 18 examples were independently measured above.
- Two migration regressions found in the first suite pass (unused argument and stale JavaScript constant assertion) were corrected and passed their reruns. A new half-period test initially passed unresolved options to an interface that requires resolved options; its corrected form passes without changing that interface.
- MATLAB Code Analyzer examined 179 MATLAB files. No syntax errors were found. The new plot helper's expected growing-output annotations were corrected; existing test-only advisories remain.
- Browser inspection verified generic unit labels, shifted x [-5,5] and y [100,120] bounds, both checked wrap controls, and numeric endpoint application. Native graphics/export/run tests passed headlessly.

## Data and documentation

### Commit preparation

The local fixture deletions, modified numeric payload in inefficientroute.mat, untracked bumpyroad.mat, and pre-existing assessment paragraph were excluded from the migration commit. Copies of the five affected committed fixtures were migrated to the generic schema with unchanged numeric payload digests, then staged without replacing the working files. All four formerly blocked or failing fixture regressions passed in an isolated checkout of that staged tree (28.5705 s total): both saved long-path cases, supplied failure-bundle replay, and saved growing-obstacle arrival handoff. The failures reported above describe the pre-existing working fixture set, not the committed fixture set. Together with the earlier checks, 176 distinct MATLAB tests have a passing result.

All three present MAT bundles were migrated recursively; numeric payload digests (type, shape, and values) remained unchanged. Existing missing fixtures were left missing. Text and MAT field/string audits found no former frame terminology in current source or saved bundles. The three tracked PDFs were rebuilt and visually inspected. Both archived PDF filenames now derive from the common historical source and explicitly disclose the terminology edition; historical algorithms and measured figures are not presented as current planner behavior. Historical benchmark numbers were preserved while coordinate headers were renamed. Git history and ignored temporary baseline artifacts were not rewritten.
