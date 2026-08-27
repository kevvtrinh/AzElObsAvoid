function tests = testHs3Planner
%% Section 0: Header & Readme
% SYNTAX
%   tests = testHs3Planner
%**************************************************************************
% PURPOSE
%   - Verify the standalone Hermite-Simpson planner, geometry interpolation,
%     validation, deterministic seed behavior, moving targets, and failures.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test position is degrees, time is seconds, and derivatives use
%     deg/s, deg/s^2, and deg/s^3.
%**************************************************************************
tests = functiontests(localfunctions);
end
function setupOnce(testCase)
% Add the repository root for path-based test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "hs3"));
testCase.TestData.RepositoryRoot = repositoryRoot;
testCase.TestData.Fixtures = testSupport.plannerFixtures();
end
function testDefaultsExposeStandaloneHs3Planner(testCase)
% Verify that zero-input defaults expose only standalone HS3 controls.
options = obstacleAvoidance.planTrajectory("hs3");
verifyEqual(testCase, options.MaximumSeedCount, 5);
verifyEqual(testCase, options.SeedClusterDistance_deg, 0);
verifyEqual(testCase, options.GoalTimeMode, "earliestArrival");
verifyEqual(testCase, options.MaximumPlanningTime_s, 115);
verifyEqual(testCase, options.PlannerMethod, "hs3");
verifyFalse(testCase, isfield(options, "MotionMethod"));
verifyFalse(testCase, isfield(options, "UseParallel"));
verifyFalse(testCase, isfield(options, "MaximumVisibilitySnapshotsPerObstacle"));
verifyFalse(testCase, isfield(options, "AzimuthInterval_deg"));
verifyFalse(testCase, isfield(options, "ElevationInterval_deg"));
end

function testRootAndDirectPackageCallsRunStandaloneHs3(testCase)
% Verify the public dispatcher and its one package implementation agree.
initialState = testCase.TestData.Fixtures.State( ...
    0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    4, [1 0.5], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
options = fixedOptions();
directOptions = rmfield(options, "PlannerMethod");
directResult = obstacleAvoidance.planner.plan( ...
    [], initialState, goalState, limits, directOptions);
publicResult = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
verifyTrue(testCase, directResult.Success, directResult.Message);
verifyTrue(testCase, directResult.Validation.Passed, ...
    directResult.Validation.Message);
verifyTrue(testCase, publicResult.Success, publicResult.Message);
verifyTrue(testCase, publicResult.Validation.Passed, ...
    publicResult.Validation.Message);
verifyEqual(testCase, directResult.SelectedMotionSource, "hs3");
verifyEqual(testCase, publicResult.SelectedMotionSource, "hs3");
verifyFalse(testCase, isfield(directResult, "CompositionDiagnostics"));
verifyFalse(testCase, isfield(publicResult, "CompositionDiagnostics"));
verifyEqual(testCase, directResult.position_deg, ...
    publicResult.position_deg, "AbsTol", 1e-10);
verifyEqual(testCase, publicResult.Options.PlannerMethod, "hs3");
verifyEqual(testCase, publicResult.SearchDiagnostics.PlannerMethod, "hs3");
end

function testStandaloneSuccessAndFailureHaveNoCompactComposition(testCase)
% Preserve a stable HS3 result schema on success and endpoint failure.
initialState = testCase.TestData.Fixtures.State( ...
    0, [0 0], [0.1 0.05], [0.02 0]);
goalState = testCase.TestData.Fixtures.State( ...
    8, [4 2], [0.15 -0.05], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 3], [1 1.5], [2 3]);
options = fixedOptions();
success = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
blockingObstacle = testCase.TestData.Fixtures.RectangleObstacle( ...
    [0 8], [-1 1 -1 1], 0);
failure = obstacleAvoidance.planTrajectory( ...
    blockingObstacle, initialState, goalState, limits, options);

verifyTrue(testCase, success.Success, success.Message);
verifyTrue(testCase, success.Validation.Passed, success.Validation.Message);
verifyEqual(testCase, success.SelectedMotionSource, "hs3");
verifyFalse(testCase, failure.Success);
verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
verifyFalse(testCase, isfield(success, "CompositionDiagnostics"));
verifyFalse(testCase, isfield(failure, "CompositionDiagnostics"));
verifyEqual(testCase, fieldnames(success), fieldnames(failure));
end
function testObstacleFreeEarliestUsesBoundedFixedSearch(testCase)
% Verify a distinct direct request uses the finer convex arrival bracket.
initialState = testCase.TestData.Fixtures.State( ...
    0, [-3 1], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    12, [5 -2], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [3 2], [1.2 0.8], [2.4 1.6]);
options = obstacleAvoidance.planTrajectory("hs3");
options.MaximumSeedCount = 1;
options.DirectSeedOnly = true;
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, ...
    result.Validation.Message);
verifyEqual(testCase, result.Polynomial.SegmentCount, ...
    2 * options.CollocationSegmentCount);
summary = result.SeedSummaries(result.SelectedSeedIndex);
verifyEqual(testCase, ...
    summary.Hs3SolverDiagnostics.ConstraintRepresentation, ...
    "linearFixedTime");
verifyLessThanOrEqual(testCase, summary.RelinearizationCount, 14);
verifyLessThan(testCase, result.ArrivalTime_s, 5.72);
end
function testObstacleFreeDirectMotionPreservesShortestLine(testCase)
% Reproduce the saved diagonal direct case without accepting axis-phase bend.
initialState = testCase.TestData.Fixtures.State( ...
    0, [-18.6803279 0.553278689], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    180, [52.8770492 -61.045082], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [0.75 0.75], [2.5 2.5]);
options = obstacleAvoidance.planTrajectory("hs3");
options.MaximumSeedCount = 1;
options.DirectSeedOnly = true;
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
displacement_deg = goalState.position_deg - initialState.position_deg;
direction = displacement_deg / norm(displacement_deg);
normal = [-direction(2), direction(1)];
deviation_deg = abs( ...
    (result.position_deg - initialState.position_deg) * normal.');
motionLength_deg = sum(vecnorm(diff(result.position_deg, 1, 1), 2, 2));
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyLessThanOrEqual(testCase, max(deviation_deg), 1e-8);
verifyEqual(testCase, motionLength_deg, norm(displacement_deg), ...
    "AbsTol", 1e-8);
verifyLessThanOrEqual(testCase, result.ArrivalTime_s, 39.54);
end
function testCertifiedDirectObstacleMotionPreservesShortestLine(testCase)
% Exercise the same invariant when irrelevant protected geometry is present.
obstacle = testCase.TestData.Fixtures.RectangleObstacle( ...
    [0 45], [-120 -110 70 80], 0.2);
initialState = testCase.TestData.Fixtures.State( ...
    0, [-18.6803279 0.553278689], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    45, [52.8770492 -61.045082], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [0.75 0.75], [2.5 2.5]);
options = fixedOptions();
options.DirectSeedOnly = false;
options.MaximumSeedCount = 2;
options.MaximumPlanningTime_s = 15;
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
displacement_deg = goalState.position_deg - initialState.position_deg;
direction = displacement_deg / norm(displacement_deg);
normal = [-direction(2), direction(1)];
deviation_deg = abs( ...
    (result.position_deg - initialState.position_deg) * normal.');
motionLength_deg = sum(vecnorm(diff(result.position_deg, 1, 1), 2, 2));
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyLessThanOrEqual(testCase, max(deviation_deg), 1e-8);
verifyEqual(testCase, motionLength_deg, norm(displacement_deg), ...
    "AbsTol", 1e-8);
end
function testNonparallelEndpointDerivativeDoesNotForceDirectLine(testCase)
% Preserve feasible lateral endpoint motion when collinearity is incompatible.
initialState = testCase.TestData.Fixtures.State( ...
    0, [0 0], [0 0.2], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    8, [4 0], [0 0.2], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
options = fixedOptions();
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyGreaterThan(testCase, max(abs(result.position_deg(:, 2))), 1e-4);
end
function testWorkspaceIntervalsBelongToLimits(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testWorkspaceIntervalsBelongToLimits");
end

function testOldWorkspaceOptionGivesMigrationError(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testOldWorkspaceOptionGivesMigrationError");
end

function testVerboseOutputUsesOnePrefixFamily(testCase)
% Verify quiet mode is quiet and verbose mode uses only the public prefix.
% checkcode cannot see that evalc reads these three local variables.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]); %#ok<NASGU>
goalState = testCase.TestData.Fixtures.State(6, [2 0], [0 0], [0 0]); %#ok<NASGU>
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]); %#ok<NASGU>
options = fixedOptions();
quietText = evalc("obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);");
verifyEmpty(testCase, strtrim(quietText));
options.Verbose = true;
verboseText = evalc("obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);");
verifyNotEmpty(testCase, ...
    regexp(verboseText, '\[ObstacleAvoidance\]', 'once'));
verifyEmpty(testCase, regexp(verboseText, ...
    '(?m)^\[HS3\]|^\[first motion', 'once'));
end
function testNearbyObstacleClusteringChangesOnlySeedGeometry(testCase)
% Verify that three nearby regions form one seed hull without physical edits.
movingSource_deg = [-1 -4; 1 -4; 1 -2; -1 -2];
movingTarget_deg = movingSource_deg + [0 0.25];
movingObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "moving rectangle", [0; 20], ...
    {movingSource_deg(:, 1); movingTarget_deg(:, 1)}, ...
    {movingSource_deg(:, 2); movingTarget_deg(:, 2)}, 0);
obstacles = [ ...
    movingObstacle; ...
    testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 -1.5 0.5], 0); ...
    testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 1 3], 0)];
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.DirectSeedOnly = false;
options.MaximumSeedCount = 3;
options.SeedClusterDistance_deg = 0.6;
[seeds, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacles, initialState, goalState, limits, options);
verifyEqual(testCase, diagnostics.SeedCluster.SourceRegionCount, 3);
verifyEqual(testCase, diagnostics.SeedCluster.ClusterGroupCount, 1);
verifyEqual(testCase, diagnostics.SeedCluster.ClusteredRegionCount, 3);
verifyNotEmpty(testCase, diagnostics.SeedCluster.ClusterBoundary_deg);
verifyGreaterThanOrEqual(testCase, numel(seeds), 2);
verifyFalse(testCase, diagnostics.Coverage.ExactSpatialProposalUsed);
verifyTrue(testCase, diagnostics.Coverage.ReducedSpatialProposalUsed);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyTrue(testCase, diagnostics.Coverage.ExtendedTimedSearchAttempted);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchUsesExactObstacles);
verifyEqual(testCase, ...
    diagnostics.Coverage.TimedSearchSuppressionReason, "");
verifyTrue(testCase, diagnostics.Coverage.CompletenessLost);
verifyGreaterThanOrEqual(testCase, diagnostics.CandidateOffsetRetryCount, 0);
verifyGreaterThan(testCase, diagnostics.VisibilityWorkBudget, 0);
verifyGreaterThanOrEqual(testCase, ...
    diagnostics.EstimatedExhaustiveVisibilityWork, 0);
verifyTrue(testCase, islogical(diagnostics.ExhaustiveVisibilityUsed));
verifyTrue(testCase, ...
    islogical(diagnostics.ExhaustiveVisibilityFallbackUsed));
visibilitySeeds = seeds([seeds.Source] == "visibilityGraph");
verifyTrue(testCase, all([visibilitySeeds.UsesReducedGeometry]));
isOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacles, 0, 0.75, 10, struct("PlannerMethod", "hs3"));
verifyFalse(testCase, isOccupied);
end
function testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints");
end

function testSeedCorridorCertificateChecksCompletePolynomial(testCase)
% Verify independent containment and continuous Bernstein separation checks.
obstacle = testCase.TestData.Fixtures.RectangleObstacle([0 10], [-1 1 -1 1], 0);
boundary_deg = [-1 -1; 1 -1; 1 1; -1 1];
seed = struct( ...
    "tau", [0; 1], "position_deg", [-2 2; 2 2], ...
    "CorridorBoundary_deg", boundary_deg);
corridor = obstacleAvoidance.search.createSeedCorridor(seed, 1);
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, 1, 1:2) = [-2 4];
positionPower_deg(1, 2, 1) = 2;
polynomial = struct( ...
    "SegmentCount", 1, "positionPower_deg", positionPower_deg);
trajectory = struct( ...
    "Polynomial", polynomial, ...
    "SeedCorridorBoundary_deg", boundary_deg, ...
    "SeedCorridor", corridor);
[certified, clearance_deg] = obstacleAvoidance.search.certifySeedCorridor( ...
    trajectory, obstacle, 1e-7);
verifyTrue(testCase, certified);
verifyGreaterThan(testCase, clearance_deg, 0.5);
trajectory.Polynomial.positionPower_deg(1, 2, 1) = 0.5;
[certifiedAfterEntry, ~] = obstacleAvoidance.search.certifySeedCorridor( ...
    trajectory, obstacle, 1e-7);
verifyFalse(testCase, certifiedAfterEntry);
end
function testSeedEnvelopeRequiresCompleteContinuousContainment(testCase)
% Verify complete continuous histories stay inside one envelope region.
boundary_deg = [-2 -2; 2 -2; 2 2; -2 2];
inside = testCase.TestData.Fixtures.RectangleObstacle([0 2], [-1 1 -1 1], 0);
outside = testCase.TestData.Fixtures.RectangleObstacle([0 2], [2.5 3.5 -1 1], 0);
verifyTrue(testCase, obstacleAvoidance.search.seedEnvelopeContainsObstacles( ...
    boundary_deg, inside, 0));
verifyFalse(testCase, obstacleAvoidance.search.seedEnvelopeContainsObstacles( ...
    boundary_deg, outside, 0));
mixedHistory = inside;
mixedHistory.az_deg{2} = mixedHistory.az_deg{2} + 3;
verifyFalse(testCase, obstacleAvoidance.search.seedEnvelopeContainsObstacles( ...
    boundary_deg, mixedHistory, 0));
concaveBoundary_deg = [-2 -2; 2 -2; 0 0; 2 2; -2 2];
verifyFalse(testCase, obstacleAvoidance.search.seedEnvelopeContainsObstacles( ...
    concaveBoundary_deg, inside, 0));
end
function testConstantJerkPolynomialPassesIndependentDynamics(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testConstantJerkPolynomialPassesIndependentDynamics");
end

function testUnrelatedPolynomialCannotValidateSampledHistory(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testUnrelatedPolynomialCannotValidateSampledHistory");
end

function testShiftedPolynomialTimeCannotHideInitialTime(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testShiftedPolynomialTimeCannotHideInitialTime");
end

function testFixedArrivalEnforcesTerminalState(testCase)
% Verify fixed time and nonzero initial and terminal state equalities.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0.1 0], [0.05 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 2], [0.2 -0.1], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
result = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyEqual(testCase, result.velocity_deg_s(1, :), ...
    initialState.velocity_deg_s, "AbsTol", 1e-10);
verifyEqual(testCase, result.acceleration_deg_s2(1, :), ...
    initialState.acceleration_deg_s2, "AbsTol", 1e-10);
verifyEqual(testCase, result.time_s(end), 8, "AbsTol", 1e-7);
verifyEqual(testCase, result.velocity_deg_s(end, :), ...
    goalState.velocity_deg_s, "AbsTol", 1e-6);
verifyHs3Solved(testCase, result, "linearFixedTime");
end
function testIntegratedJerkGradientMatchesDirectionalDifference(testCase)
% Verify the exact variable-time gradient and fixed-time Hessian.
segmentCount = 3;
decision = [linspace(-1, 1, 2 * (2 * segmentCount + 1)).'; 8];
direction = cos((1:numel(decision)).');
[~, gradient] = hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
    decision, true, 8, segmentCount, 1, 2);
step = 1e-6;
forward = hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
    decision + step * direction, true, 8, segmentCount, 1, 2);
backward = hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
    decision - step * direction, true, 8, segmentCount, 1, 2);
verifyEqual(testCase, gradient.' * direction, ...
    (forward - backward) / (2 * step), "RelTol", 1e-8);

fixedDecision = decision(1:end - 1);
fixedDirection = direction(1:end - 1);
[fixedValue, fixedGradient, fixedHessian] = ...
    hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
    fixedDecision, false, 8, segmentCount, 1, 2);
[~, forwardGradient] = ...
    hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
    fixedDecision + step * fixedDirection, false, 8, segmentCount, 1, 2);
[~, backwardGradient] = ...
    hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
    fixedDecision - step * fixedDirection, false, 8, segmentCount, 1, 2);
verifyEqual(testCase, fixedValue, ...
    0.5 * fixedDecision.' * fixedHessian * fixedDecision, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, fixedGradient, fixedHessian * fixedDecision, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, (forwardGradient - backwardGradient) / (2 * step), ...
    fixedHessian * fixedDirection, "RelTol", 1e-8);
end
function testEarliestArrivalIsInsideHorizon(testCase)
% Verify the bounded convex arrival bracket and independent validation.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.GoalTimeMode = "earliestArrival";
result = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyLessThan(testCase, result.time_s(end), goalState.time_s);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyHs3Solved(testCase, result, "linearFixedTime");
jerkRampTime_s = limits.maxAcceleration_deg_s2(1) / ...
    limits.maxJerk_deg_s3(1);
constantAccelerationTime_s = (-1.5 + sqrt(16.25)) / 2;
independentMinimumTime_s = 2 * ...
    (constantAccelerationTime_s + 2 * jerkRampTime_s);
verifyEqual(testCase, result.time_s(end), independentMinimumTime_s, ...
    "AbsTol", 0.25);
end
function testEarliestGoalIsNotRejectedByHorizonOccupancy(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testEarliestGoalIsNotRejectedByHorizonOccupancy");
end

function testIntegerWorkLimitsRejectFractionalValues(testCase)
% Verify invalid optimizer work limits fail at the public boundary.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumNlpIterations = 1.5;
didThrow = false;
try
    obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
catch exception
    didThrow = true;
    verifyTrue(testCase, contains(exception.message, "integer"));
end
verifyTrue(testCase, didThrow);
end
function testMovingGoalInterpolationMethodMustBeScalar(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testMovingGoalInterpolationMethodMustBeScalar");
end

function testMovingGoalHistoryRequiresTwoSamples(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testMovingGoalHistoryRequiresTwoSamples");
end

function testInterceptWrapperTextOptionsMustBeScalar(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testInterceptWrapperTextOptionsMustBeScalar");
end

function testInterceptWrapperRequiresTwoTargetSamples(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testInterceptWrapperRequiresTwoTargetSamples");
end

function testMeshOptionsAreHonestlyReflected(testCase)
% Report the selected HS3 mesh and never claim unperformed refinement.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0.1 0], [0.05 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 2], [0.2 -0.1], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.CollocationSegmentCount = 4;
options.MaximumCollocationSegmentCount = 8;
options.MaximumMeshRefinementPasses = 1;
result = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyGreaterThanOrEqual(testCase, result.Polynomial.SegmentCount, 4);
verifyLessThanOrEqual(testCase, result.Polynomial.SegmentCount, 8);
verifyEqual(testCase, result.Polynomial.SegmentCount, ...
    4 * 2 ^ result.SearchDiagnostics.MeshRefinementPassCount);
verifyFalse(testCase, isfield(result, "CompositionDiagnostics"));
end
function testStaticObstacleProducesOppositeSideSeeds(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testStaticObstacleProducesOppositeSideSeeds");
end

function testFixedArrivalSelectsShortestValidatedStaticMotion(testCase)
% Verify fixed arrival evaluates retained static routes and selects by length.
obstacle = testCase.TestData.Fixtures.RectangleObstacle( ...
    [0 20], [-1 1 -3 3], 0.1);
initialState = testCase.TestData.Fixtures.State( ...
    0, [-6 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    20, [6 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
options = fixedOptions();
options.DirectSeedOnly = false;
options.MaximumSeedCount = 3;
options.DeterministicWorkBudget = true;
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
validated = find([result.SeedSummaries.ValidationPassed]);
motionLength_deg = [result.SeedSummaries(validated).MotionLength_deg];
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyGreaterThanOrEqual(testCase, numel(validated), 2);
verifyEqual(testCase, result.SearchDiagnostics.AttemptedSeedCount, ...
    numel(result.Seeds));
verifyEqual(testCase, ...
    result.SeedSummaries(result.SelectedSeedIndex).MotionLength_deg, ...
    min(motionLength_deg), "AbsTol", 1e-9);
end

function testTwoObstacleHomologySearchFindsDistinctClasses(testCase)
% Verify signature diversity for a structurally different obstacle field.
obstacles = [ ...
    testCase.TestData.Fixtures.RectangleObstacle([0 20], [-3 -1 -1 1], 0.1); ...
    testCase.TestData.Fixtures.RectangleObstacle([0 20], [1 3 -1 1], 0.1)];
initialState = testCase.TestData.Fixtures.State(0, [-6 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [6 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSeedCount = 5;
options.DirectSeedOnly = false;
[seeds, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacles, initialState, goalState, limits, options);
verifySize(testCase, diagnostics.HomologyRepresentative_deg, [2 2]);
verifyEqual(testCase, size(diagnostics.HomologyClassSignatures, 2), 2);
verifyGreaterThanOrEqual(testCase, diagnostics.HomologyClassCount, 2);
verifyLessThan(testCase, diagnostics.VisibilityCandidatePairCount, ...
    diagnostics.NodeCount * (diagnostics.NodeCount - 1) / 2);
verifyEqual(testCase, size(unique( ...
    diagnostics.HomologyClassSignatures, "rows"), 1), ...
    diagnostics.HomologyClassCount);
verifyGreaterThanOrEqual(testCase, ...
    sum([seeds.Source] == "visibilityGraph"), 2);
verifyFalse(testCase, diagnostics.HomologySearchTruncated);
verifyEqual(testCase, diagnostics.RouteCleanupAcceptedCount, 0);
verifyEqual(testCase, diagnostics.RouteCleanupLengthReduction_deg, 0);
end

function testSpatialVisibilityCleanupShortensSameClassRectangleRoutes(testCase)
% Shorten sparse-graph routes without changing their searched classes.
boxBounds_deg = [ ...
    3.273967634778897, 4.759475100110320, ...
    -1.969302583911333, 0.257700798227361; ...
    -7.682661487978057, -5.689347269083195, ...
    -2.237331933064558, -1.182829800806282; ...
    1.113286443533166, 4.319563635550832, ...
    -0.413471964608378, 1.614712841485502; ...
    0.168516682050331, 1.597232839656645, ...
    -0.588545920496329, 3.198791926371235; ...
    2.921431145758944, 5.447043764515808, ...
    -2.967092353311103, -1.589963711268966; ...
    -7.835366233841265, -4.538291473771393, ...
    1.343538998087866, 2.511566406533022];
obstacles = rectangleObstacles(testCase, boxBounds_deg);
[seeds, diagnostics] = spatialCleanupSeeds(testCase, obstacles);
verifyGreaterThanOrEqual(testCase, diagnostics.RouteCleanupAcceptedCount, 1);
verifyGreaterThan(testCase, ...
    diagnostics.RouteCleanupLengthReduction_deg, 1e-3);
verifyCleanedSpatialSeeds(testCase, seeds, diagnostics, obstacles);
end

function testSpatialVisibilityCleanupRejectsCircleHomologyChanges(testCase)
% Exercise accepted and rejected shortcuts on curved protected regions.
circleSpecification_deg = [ ...
    -2.271964379198359, 0.072286638808674, 1.098394723514003; ...
    -5.081942780381125, -1.713565819393793, 0.601134406621426; ...
    -6.074659165988352, 0.924717840817177, 1.637047106355157; ...
    2.752016845588681, -2.118818969422467, 1.738329825222016];
angle_rad = (0:15).' * (2 * pi / 16);
obstacleTemplate = obstacleAvoidance.obstacles.createObstacle( ...
    "circle", [0; 40], cos(angle_rad), sin(angle_rad), 0.05);
obstacles = repmat(obstacleTemplate, size(circleSpecification_deg, 1), 1);
for obstacleIndex = 1:numel(obstacles)
    center_deg = circleSpecification_deg(obstacleIndex, 1:2);
    radius_deg = circleSpecification_deg(obstacleIndex, 3);
    obstacles(obstacleIndex) = obstacleAvoidance.obstacles.createObstacle( ...
        "circle", [0; 40], ...
        center_deg(1) + radius_deg * cos(angle_rad), ...
        center_deg(2) + radius_deg * sin(angle_rad), 0.05);
end
[seeds, diagnostics] = spatialCleanupSeeds(testCase, obstacles);
verifyGreaterThanOrEqual(testCase, diagnostics.RouteCleanupAcceptedCount, 1);
verifyGreaterThan(testCase, ...
    diagnostics.RouteCleanupLengthReduction_deg, 10);
verifyGreaterThan(testCase, ...
    diagnostics.RouteCleanupHomologyRejectedCount, 0);
verifyCleanedSpatialSeeds(testCase, seeds, diagnostics, obstacles);
end

function testDetourWarmStartDoesNotDefineReachability(testCase)
% Keep a reachable multi-axis detour when its conservative warm start is long.
obstacle = testCase.TestData.Fixtures.RectangleObstacle( ...
    [0 20], [-1 1 -4 4], 0.1);
initialState = testCase.TestData.Fixtures.State( ...
    0, [-6 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    20, [6 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
options = fixedOptions();
options.DirectSeedOnly = false;
options.MaximumSeedCount = 3;
[longHorizonSeeds, ~] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacle, initialState, goalState, limits, options);
longDetourSeeds = longHorizonSeeds( ...
    [longHorizonSeeds.Source] == "visibilityGraph");
verifyNotEmpty(testCase, longDetourSeeds);
route_deg = longDetourSeeds(1).position_deg;
axisTravel_deg = sum(abs(diff(route_deg, 1, 1)), 1);
minimumDuration_s = max(axisTravel_deg ./ limits.maxVelocity_deg_s);
warmDuration_s = longDetourSeeds(1).EstimatedDuration_s;
verifyGreaterThan(testCase, warmDuration_s, ...
    minimumDuration_s);

shortHorizon_s = 0.5 * (minimumDuration_s + warmDuration_s);
shortGoalState = goalState;
shortGoalState.time_s = shortHorizon_s;
[shortHorizonSeeds, ~] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacle, initialState, shortGoalState, limits, options);
shortDetourSeeds = shortHorizonSeeds( ...
    [shortHorizonSeeds.Source] == "visibilityGraph");
verifyNotEmpty(testCase, shortDetourSeeds);
verifyEqual(testCase, shortDetourSeeds(1).EstimatedDuration_s, ...
    shortHorizon_s, ...
    "AbsTol", 1e-12);
end
function testLongStaticDetourUsesDynamicsScaledQualityMesh(testCase)
% Verify a long detour receives one bounded, dynamics-scaled quality pass.
circleAngle_rad = (0:47).' * (2 * pi / 48);
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "neutral circle", [0; 360], ...
    20 * cos(circleAngle_rad), 20 * sin(circleAngle_rad), 0.2);
initialState = testCase.TestData.Fixtures.State( ...
    0, [-50 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    360, [100 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [0.75 0.75], [2.5 2.5]);
options = obstacleAvoidance.planTrajectory("hs3");
options.MaximumSeedCount = 3;
options.MaximumPlanningTime_s = 30;
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, ...
    result.Validation.Message);
verifyEqual(testCase, result.Polynomial.SegmentCount, 40);
verifyEqual(testCase, ...
    result.SearchDiagnostics.MeshRefinementPassCount, 1);
verifyLessThan(testCase, result.ArrivalTime_s, 79);
end
function testMovingObstacleUsesTrajectoryTime(testCase)
% Verify that the same point changes occupancy as protected geometry moves.
time_s = [0; 4];
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
azimuth_deg = {source_deg(:, 1); source_deg(:, 1)};
elevation_deg = {source_deg(:, 2); source_deg(:, 2) + 4};
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "moving", time_s, azimuth_deg, elevation_deg, 0);
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, [0; 0], [0; 0], [0; 4], ...
    struct("PlannerMethod", "hs3"));
verifyEqual(testCase, occupied, [true; false]);
end
function testOccupancyOnlyMatchesDetailedQuery(testCase)
% Verify the fast occupancy path preserves moving and boundary decisions.
time_s = [0; 2];
first_deg = [-1 -1; 1 -1; 1 1; -1 1];
second_deg = first_deg + [2 0];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "moving", time_s, ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0);
queryAzimuth_deg = [0; 1; 3];
queryElevation_deg = [0; 0; 0];
queryTime_s = [0; 0; 2];
queryOptions = struct("PlannerMethod", "hs3");
fastOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, queryAzimuth_deg, queryElevation_deg, ...
    queryTime_s, queryOptions);
[detailedOccupied, ~, details] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, queryAzimuth_deg, queryElevation_deg, ...
    queryTime_s, queryOptions);
verifyEqual(testCase, fastOccupied, detailedOccupied);
verifyEqual(testCase, fastOccupied, [true; true; true]);
verifyLessThanOrEqual(testCase, min(details.MinimumClearance_deg), 0);
end
function testDeformingObstacleInterpolatesAtTrajectoryTime(testCase)
% Verify vertex interpolation changes occupancy for a deforming polygon.
time_s = [0; 2];
first_deg = [-1 -1; 1 -1; 1 1; -1 1];
second_deg = [-2 -0.5; 2 -0.5; 2 0.5; -2 0.5];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "deforming", time_s, ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0);
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, [1.5; 1.5], [0; 0], [0; 2], ...
    struct("PlannerMethod", "hs3"));
verifyEqual(testCase, occupied, [false; true]);
end
function testDeformingObstacleUsesThePlannerPath(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testDeformingObstacleUsesThePlannerPath");
end

function testTopologyChangeUsesAStationaryConservativeUnion(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testTopologyChangeUsesAStationaryConservativeUnion");
end

function testMovingBarrierGeneratesWaitingSeed(testCase)
% Verify a requested but unused cluster does not suppress timed search.
time_s = [0; 6; 8; 12];
source_deg = [-0.5 -2; 0.5 -2; 0.5 2; -0.5 2];
centerElevation_deg = [0; 0; 6; 6];
azimuth_deg = cell(4, 1);
elevation_deg = cell(4, 1);
% Translate the same boundary at each time sample to define the moving barrier.
for sampleIndex = 1:4
    translated_deg = source_deg + [0 centerElevation_deg(sampleIndex)];
    azimuth_deg{sampleIndex} = translated_deg(:, 1);
    elevation_deg{sampleIndex} = translated_deg(:, 2);
end
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "barrier", time_s, azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = obstacleAvoidance.planTrajectory("hs3");
options.MaximumSeedCount = 5;
options.SeedClusterDistance_deg = 0.6;
[seeds, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacle, initialState, goalState, limits, options);
verifyEqual(testCase, diagnostics.SeedCluster.ClusterGroupCount, 0);
verifyTrue(testCase, any([seeds.Source] == "directWait"));
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchUsesExactObstacles);
verifyEqual(testCase, ...
    diagnostics.Coverage.TimedSearchSuppressionReason, "");
verifyGreaterThan(testCase, diagnostics.ExpandedCount, 0);
verifyEqual(testCase, diagnostics.GraphType, ...
    "timeExpandedVisibilityGraph");
verifyGreaterThan(testCase, diagnostics.TemporalLayerCount, 1);
verifyGreaterThan(testCase, diagnostics.WaitEdgeCount, 0);

% The standalone HS3 planner must retain the validated timed wait. Another
% independently valid topology may be selected when it has lower jerk at
% the same fixed arrival.
options.GoalTimeMode = "fixedArrival";
options.MaximumPlanningTime_s = 15;
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, ...
    result.Validation.Message);
waitIndex = find([result.Seeds.Source] == "directWait", 1);
verifyNotEmpty(testCase, waitIndex);
verifyTrue(testCase, result.SeedSummaries(waitIndex).Hs3Attempted);
verifyTrue(testCase, result.SeedSummaries(waitIndex).ValidationPassed);
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyFalse(testCase, isfield(result, "CompositionDiagnostics"));
end
function testObstacleActivationSpanEnablesTimedSearch(testCase)
% Verify equal geometry can still change occupancy through its active span.
obstacle = testCase.TestData.Fixtures.RectangleObstacle([3 6], [-0.5 0.5 -2 2], 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = obstacleAvoidance.planTrajectory("hs3");
[~, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchUsesExactObstacles);
end
function testTimedSeedKeepsDistinctAbsoluteDuration(testCase)
% Verify equal spatial routes keep distinct moving-obstacle time guesses.
azimuth_deg = repmat({[-1; 1; 1; -1]}, 2, 1);
elevation_deg = {[9; 9; 11; 11]; [11; 11; 13; 13]};
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "far moving obstacle", [0; 20], azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = obstacleAvoidance.planTrajectory("hs3");
options.MaximumSeedCount = 5;
[seeds, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacle, initialState, goalState, limits, options);
directSeed = seeds([seeds.Source] == "directVisibilityEdge");
timedSeed = seeds([seeds.Source] == "timeExpandedVisibilityGraph");
verifyNotEmpty(testCase, directSeed);
verifyNotEmpty(testCase, timedSeed);
verifyEqual(testCase, directSeed(1).position_deg, timedSeed(1).position_deg);
verifyGreaterThan(testCase, timedSeed(1).EstimatedDuration_s, ...
    directSeed(1).EstimatedDuration_s);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
options.GoalTimeMode = "fixedArrival";
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, any([result.SeedSummaries.Hs3ValidationPassed]));
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyGreaterThan(testCase, numel(result.Seeds), 1);
verifyEqual(testCase, result.SearchDiagnostics.AttemptedSeedCount, 1);
verifyEqual(testCase, ...
    sum(vecnorm(diff(result.position_deg, 1, 1), 2, 2)), ...
    norm(goalState.position_deg - initialState.position_deg), ...
    "AbsTol", 1e-9);
verifyEqual(testCase, ...
    result.SearchDiagnostics.ValidatedCandidateCount, ...
    nnz([result.SeedSummaries.ValidationPassed]));
verifyEqual(testCase, ...
    result.SearchDiagnostics.BestPartialSeedIndex, ...
    result.SelectedSeedIndex);
verifyTrue(testCase, isfinite( ...
    result.SearchDiagnostics.FirstValidatedMotionTime_s));
end
function testDenseEnvelopeReportsTimedSearchWorkLimit(testCase)
% Verify dense timed work is suppressed and reported as incomplete.
time_s = [0; 6; 8; 12];
angle_rad = (0:1199).' * (2 * pi / 1200);
source_deg = [0.5 * cos(angle_rad), 2 * sin(angle_rad)];
centerElevation_deg = [0; 0; 6; 6];
azimuth_deg = repmat({source_deg(:, 1)}, 4, 1);
elevation_deg = cell(4, 1);
% Shift every elevation sample while preserving the dense boundary topology.
for sampleIndex = 1:4
    elevation_deg{sampleIndex} = source_deg(:, 2) + centerElevation_deg(sampleIndex);
end
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "dense moving barrier", time_s, azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = obstacleAvoidance.planTrajectory("hs3");
options.MaximumSeedCount = 2;
[seeds, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, diagnostics.DenseSeedEnvelopeUsed);
verifyTrue(testCase, diagnostics.Coverage.ReducedSpatialProposalUsed);
verifyFalse(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyEqual(testCase, diagnostics.Coverage.TimedSearchSuppressionReason, ...
    "timedQueryWorkLimit");
directWaitSeeds = seeds([seeds.Source] == "directWait");
verifyEmpty(testCase, directWaitSeeds);
spatialSeeds = seeds([seeds.Source] == "visibilityGraph");
verifyNotEmpty(testCase, spatialSeeds);
verifyTrue(testCase, all([spatialSeeds.UsesReducedGeometry]));
verifyTrue(testCase, diagnostics.Coverage.CompletenessLost);
end
function testWaitingSeedDoesNotImposeCornerState(testCase)
% Verify a repeated seed vertex does not become a zero-velocity equality.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSolverTime_s = Inf;
seed = struct( ...
    "Index", 1, "Source", "directWait", ...
    "position_deg", [0 0; 0 0; 4 0], ...
    "tau", [0; 0.4; 1], "EstimatedDuration_s", 8, ...
    "Length_deg", 4, "UsesReducedGeometry", false);
candidate = obstacleAvoidance.planner.solveRouteCandidate( ...
    [], initialState, goalState, limits, options, seed);
cornerTime_s = 0.4 * goalState.time_s;
[~, sampleIndex] = min(abs(candidate.time_s - cornerTime_s));
verifyTrue(testCase, candidate.OptimizerFeasible, candidate.Message);
verifyGreaterThan(testCase, ...
    norm(candidate.velocity_deg_s(sampleIndex, :)), 0.1);
verifyGreaterThan(testCase, ...
    norm(candidate.position_deg(sampleIndex, :) - ...
    seed.position_deg(2, :)), 0.1);
end

function testEarlySolverFailureRetainsMotionLengthSchema(testCase)
% Verify fixed-quality diagnostics keep one schema before optimization.
initialState = testCase.TestData.Fixtures.State( ...
    0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    8, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSolverTime_s = 0;
seed = struct( ...
    "Index", 1, "Source", "directVisibilityEdge", ...
    "position_deg", [0 0; 4 0], "tau", [0; 1], ...
    "EstimatedDuration_s", 8, "Length_deg", 4, ...
    "UsesReducedGeometry", false);
candidate = obstacleAvoidance.planner.solveRouteCandidate( ...
    [], initialState, goalState, limits, options, seed);
verifyTrue(testCase, isfield(candidate, "MotionLength_deg"));
verifyEqual(testCase, candidate.MotionLength_deg, Inf);
end

function testAzimuthWrappingChangesThePhysicalRequest(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testAzimuthWrappingChangesThePhysicalRequest");
end

function testAzimuthWrappingRejectsUnmodeledPeriodicGeometry(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testAzimuthWrappingRejectsUnmodeledPeriodicGeometry");
end

function testPlanningTimeOptionIsResolved(testCase)
% Verify the standalone planner owns one positive end-to-end wall-time budget.
options = obstacleAvoidance.planTrajectory("hs3");
options.MaximumPlanningTime_s = 7;
resolved = obstacleAvoidance.input.resolveHs3Options( ...
    rmfield(options, "PlannerMethod"));
verifyEqual(testCase, resolved.MaximumPlanningTime_s, 7);
end

function testEarlyPlannerFailureKeepsValidationFieldOrder(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testEarlyPlannerFailureKeepsValidationFieldOrder");
end

function testBetweenNodeCollisionFailsValidation(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testBetweenNodeCollisionFailsValidation");
end

function testObstacleActivationAtTerminalTimeFailsValidation(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testObstacleActivationAtTerminalTimeFailsValidation");
end

function testBetweenNodeVelocityViolationFailsValidation(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testBetweenNodeVelocityViolationFailsValidation");
end

function testSafetyMarginIsAppliedExactlyOnce(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testSafetyMarginIsAppliedExactlyOnce");
end

function testTranslatedHistoryReusesExactProtectedShape(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testTranslatedHistoryReusesExactProtectedShape");
end

function testDeterministicRepeatedRun(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testDeterministicRepeatedRun");
end

function testNoPathReturnsStableDiagnostics(testCase)
% Verify expected no-path failure returns diagnostics instead of an error.
wall = testCase.TestData.Fixtures.RectangleObstacle([0 12], [-0.5 0.5 -90 90], 0.2);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSeedCount = 3;
options.MaximumNlpIterations = 40;
limits.elevationInterval_deg = [-10 10];
result = obstacleAvoidance.planTrajectory(wall, initialState, goalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyEqual(testCase, numel(result.SeedSummaries), numel(result.Seeds));
verifyGreaterThanOrEqual(testCase, ...
    result.SearchDiagnostics.Grid.ExpandedCount, 0);
verifyEqual(testCase, ...
    size(result.SearchDiagnostics.Grid.FrontierNodes_deg, 2), 2);
plotOptions = struct( ...
    "FigureVisible", "off", ...
    "ShowWorkspace", true, ...
    "ShowVisibilityGraphs", true, ...
    "ShowKinematics", false, ...
    "ShowAnimation", false);
handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
figureCleanup = onCleanup(@() closeTestFigures(handles));
verifyTrue(testCase, isgraphics(handles.WorkspaceFigure, "figure"));
verifyTrue(testCase, isgraphics(handles.VisibilityFigure, "figure"));
verifyNotEqual(testCase, ...
    numel(wall.originalAz_deg{1}), numel(wall.az_deg{1}));
verifyNotEmpty(testCase, findobj( ...
    handles.WorkspaceAxes, "DisplayName", "Original obstacle"));
surfaceHandles = findobj(handles.VisibilityAxes, "Type", "surface");
if ~isempty(surfaceHandles)
    verifyTrue(testCase, isnumeric(surfaceHandles(1).EdgeColor));
end
end

function testExactStaticNoPathSkipsMotionSolve(testCase)
% Verify an exact exhaustive topology failure does not enter HS3.
wall = obstacleAvoidance.obstacles.createObstacle( ...
    "full-height wall", [0; 12], [-0.5; 0.5; 0.5; -0.5], ...
    [-90; -90; 90; 90], 0);
initialState = testCase.TestData.Fixtures.State( ...
    0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
limits.elevationInterval_deg = [-10 10];
options = fixedOptions();
options.MaximumSeedCount = 3;
options.DirectSeedOnly = false;
result = obstacleAvoidance.planTrajectory(wall, initialState, goalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyTrue(testCase, ...
    result.SearchDiagnostics.Grid.StaticNoPathCertified);
verifyEqual(testCase, result.SearchDiagnostics.AttemptedSeedCount, 0);
end
function testMovingTargetUsesSamePlanner(testCase)
% Verify the intercept wrapper only adapts target inputs to planTrajectory.
targetTime_s = (0:5:20).';
targetPosition_deg = [6 + 0.1 * targetTime_s, ...
    ones(size(targetTime_s))];
targetMotion = struct( ...
    "time_s", targetTime_s, ...
    "position_deg", targetPosition_deg, ...
    "InterpolationMethod", "linear");
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
plannerOptions = fixedOptions();
plannerOptions.GoalTimeMode = "earliestArrival";
interceptOptions = struct( ...
    "InterceptMode", "earliest", ...
    "MaximumSearchDuration_s", 20, ...
    "PlannerOptions", plannerOptions);
result = obstacleAvoidance.planMovingTargetIntercept( ...
    initialState, targetMotion, limits, interceptOptions);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.Intercept.Mode, "earliest");
verifyEqual(testCase, result.Options.PlannerMethod, "hs3");
verifyEqual(testCase, result.SearchDiagnostics.PlannerMethod, "hs3");
verifyEqual(testCase, ...
    result.Intercept.Options.PlannerOptions.PlannerMethod, "hs3");
verifyEqual(testCase, result.Intercept.Search.Method, ...
    "boundedChronologicalFixedTime");
verifyGreaterThan(testCase, result.Intercept.Search.TrialCount, 1);
verifyEqual(testCase, result.position_deg(end, :), ...
    result.Intercept.TargetPosition_deg, "AbsTol", 1e-6);
plotOptions = struct( ...
    "FigureVisible", "off", "ShowWorkspace", true, ...
    "ShowVisibilityGraphs", true, "ShowKinematics", false, ...
    "ShowAnimation", true, "FrameStride", numel(result.time_s));
handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
figureCleanup = onCleanup(@() closeTestFigures(handles));
verifyEqual(testCase, numel(handles.AnimationKinematicAxes), 4);
verifyTrue(testCase, all(isgraphics(handles.AnimationKinematicAxes, "axes")));
verifyTrue(testCase, isgraphics(handles.AnimationLegend));
axesHandles = [handles.WorkspaceAxes, handles.VisibilityAxes, ...
    handles.AnimationAxes];
% Check each rendered view so none can silently omit the moving-target trace.
for axesIndex = 1:numel(axesHandles)
    targetHandles = findobj(axesHandles(axesIndex), ...
        "DisplayName", "Moving target");
    verifyNotEmpty(testCase, targetHandles);
end
end

function testMovingTargetSpecifiedTimeMatchesNonzeroDerivatives(testCase)
% Verify HS3 receives the shared adapter's nonzero terminal target state.
targetTime_s = (0:2:20).';
targetPosition_deg = [0.04 * targetTime_s .^ 2, ...
    0.02 * targetTime_s .^ 2];
targetMotion = struct( ...
    "time_s", targetTime_s, ...
    "position_deg", targetPosition_deg, ...
    "InterpolationMethod", "pchip");
initialState = testCase.TestData.Fixtures.State( ...
    0, [0 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
interceptTime_s = 10;
interceptOptions = struct( ...
    "InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", interceptTime_s, ...
    "MatchTargetVelocity", true, ...
    "MatchTargetAcceleration", true, ...
    "PlannerOptions", fixedOptions());
result = obstacleAvoidance.planMovingTargetIntercept( ...
    initialState, targetMotion, limits, interceptOptions);

derivativeStep_s = 0.01;
lowerPosition_deg = interp1(targetTime_s, targetPosition_deg, ...
    interceptTime_s - derivativeStep_s, "pchip");
centerPosition_deg = interp1(targetTime_s, targetPosition_deg, ...
    interceptTime_s, "pchip");
upperPosition_deg = interp1(targetTime_s, targetPosition_deg, ...
    interceptTime_s + derivativeStep_s, "pchip");
expectedVelocity_deg_s = (upperPosition_deg - lowerPosition_deg) ./ ...
    (2 * derivativeStep_s);
expectedAcceleration_deg_s2 = (upperPosition_deg - ...
    2 * centerPosition_deg + lowerPosition_deg) ./ derivativeStep_s ^ 2;

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.Options.PlannerMethod, "hs3");
verifyEqual(testCase, result.SearchDiagnostics.PlannerMethod, "hs3");
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyEqual(testCase, result.Intercept.Mode, "specifiedTime");
verifyEqual(testCase, result.Intercept.Search.Method, "specifiedFixedTime");
verifyEqual(testCase, result.Intercept.Search.TrialCount, 1);
verifyEqual(testCase, result.Intercept.TerminalVelocityPolicy, "target");
verifyEqual(testCase, result.Intercept.TerminalAccelerationPolicy, "target");
verifyEqual(testCase, result.velocity_deg_s(end, :), ...
    expectedVelocity_deg_s, "AbsTol", 1e-6);
verifyEqual(testCase, result.acceleration_deg_s2(end, :), ...
    expectedAcceleration_deg_s2, "AbsTol", 1e-6);
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyFalse(testCase, isfield(result, "CompositionDiagnostics"));
end

function options = fixedOptions()
% Return fast deterministic settings for focused tests.
options = obstacleAvoidance.planTrajectory("hs3");
options.GoalTimeMode = "fixedArrival";
options.DirectSeedOnly = true;
options.MaximumSeedCount = 1;
options.CollocationSegmentCount = 5;
options.MaximumNlpIterations = 150;
options.MaximumNlpFunctionEvaluations = 15000;
options.MaximumPlanningTime_s = 8;
options.SampleTime_s = 0.05;
options.Verbose = false;
end

function obstacles = rectangleObstacles(testCase, boxBounds_deg)
% Construct static protected rectangles for route-cleanup tests.
obstacleCount = size(boxBounds_deg, 1);
obstacleTemplate = testCase.TestData.Fixtures.RectangleObstacle( ...
    [0 40], boxBounds_deg(1, :), 0.05);
obstacles = repmat(obstacleTemplate, obstacleCount, 1);
for obstacleIndex = 1:obstacleCount
    obstacles(obstacleIndex) = ...
        testCase.TestData.Fixtures.RectangleObstacle( ...
        [0 40], boxBounds_deg(obstacleIndex, :), 0.05);
end
end

function [seeds, diagnostics] = spatialCleanupSeeds(testCase, obstacles)
% Run deterministic spatial visibility search without invoking HS3.
initialState = testCase.TestData.Fixtures.State( ...
    0, [-12 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    40, [12 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
options = obstacleAvoidance.planTrajectory("hs3");
options.GoalTimeMode = "fixedArrival";
options.DirectSeedOnly = false;
options.MaximumSeedCount = 5;
[seeds, diagnostics] = obstacleAvoidance.search.createRouteCandidates( ...
    obstacles, initialState, goalState, limits, options);
end

function verifyCleanedSpatialSeeds(testCase, seeds, diagnostics, obstacles)
% Independently check length, sampled clearance, and homology signatures.
spatialSeeds = seeds([seeds.Source] == "visibilityGraph");
verifyEqual(testCase, numel(spatialSeeds), diagnostics.HomologyClassCount);
verifyEqual(testCase, diagnostics.RouteCleanupAttemptedCount, ...
    diagnostics.HomologyClassCount);
protectedShape = polyshape();
for obstacleIndex = 1:numel(obstacles)
    protectedShape = union(protectedShape, ...
        obstacleAvoidance.obstacles.shapeAtTime(obstacles(obstacleIndex), 0));
end
for seedIndex = 1:numel(spatialSeeds)
    route_deg = spatialSeeds(seedIndex).position_deg;
    measuredLength_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
    verifyEqual(testCase, spatialSeeds(seedIndex).Length_deg, ...
        measuredLength_deg, "AbsTol", 1e-12);
    signature = independentRouteSignature( ...
        route_deg, diagnostics.HomologyRepresentative_deg);
    verifyEqual(testCase, signature, ...
        diagnostics.HomologyClassSignatures(seedIndex, :));
    for edgeIndex = 1:size(route_deg, 1) - 1
        fraction = linspace(0, 1, 1001).';
        sample_deg = route_deg(edgeIndex, :) + fraction .* ...
            (route_deg(edgeIndex + 1, :) - route_deg(edgeIndex, :));
        clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance( ...
            protectedShape, sample_deg);
        verifyGreaterThan(testCase, min(clearance_deg), 0);
    end
end
end

function signature = independentRouteSignature(route_deg, representative_deg)
% Recompute open-route winding classes independently from search state.
representativeCount = size(representative_deg, 1);
signature = zeros(1, representativeCount, "int8");
for representativeIndex = 1:representativeCount
    offset_deg = route_deg - representative_deg(representativeIndex, :);
    phase_rad = atan2(offset_deg(:, 2), offset_deg(:, 1));
    wrappedStep_rad = atan2(sin(diff(phase_rad)), cos(diff(phase_rad)));
    endpointStep_rad = atan2( ...
        sin(phase_rad(end) - phase_rad(1)), ...
        cos(phase_rad(end) - phase_rad(1)));
    signature(representativeIndex) = int8(round( ...
        (sum(wrappedStep_rad) - endpointStep_rad) / (2 * pi)));
end
end

function verifyHs3Solved(testCase, result, representation)
% Require selected standalone solver evidence for the requested formulation.
verifyEqual(testCase, result.SelectedMotionSource, "hs3");
verifyFalse(testCase, isfield(result, "CompositionDiagnostics"));
summary = result.SeedSummaries(result.SelectedSeedIndex);
verifyTrue(testCase, summary.Hs3Attempted);
verifyTrue(testCase, summary.Hs3OptimizerFeasible);
verifyTrue(testCase, summary.Hs3ValidationPassed);
verifyEqual(testCase, ...
    summary.Hs3SolverDiagnostics.ConstraintRepresentation, representation);
end

function closeTestFigures(handles)
% Close figures created by a plot test even when its verification fails.
figureHandles = [handles.WorkspaceFigure; handles.VisibilityFigure];
close(figureHandles(isgraphics(figureHandles, "figure")));
end
