function tests = testHs3Planner
%% Section 0: Header & Readme
% SYNTAX
%   tests = testHs3Planner
%**************************************************************************
% PURPOSE
%   - Verify the compact HS3 planner, geometry interpolation, validation,
%     deterministic seed behavior, moving targets, and stable failures.
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
testCase.TestData.RepositoryRoot = repositoryRoot;
testCase.TestData.Fixtures = testSupport.plannerFixtures();
end
function testDefaultsExposeOneSmallPlanner(testCase)
% Verify that zero-input defaults expose only the maintained HS3 controls.
options = planAzElMotion("hs3");
verifyEqual(testCase, options.MaximumSeedCount, 5);
verifyEqual(testCase, options.SeedClusterDistance_deg, 0);
verifyEqual(testCase, options.GoalTimeMode, "earliestArrival");
verifyFalse(testCase, options.EnableHs3Improvement);
verifyFalse(testCase, isfield(options, "UseParallel"));
verifyFalse(testCase, isfield(options, "MaximumVisibilitySnapshotsPerObstacle"));
verifyFalse(testCase, isfield(options, "MaximumPlanningTime_s"));
verifyFalse(testCase, isfield(options, "AzimuthInterval_deg"));
verifyFalse(testCase, isfield(options, "ElevationInterval_deg"));
end

function testDirectBackendFacadeUsesCompositeWithoutRecursion(testCase)
% Verify the method-qualified facade terminates through the public composite.
initialState = testCase.TestData.Fixtures.State( ...
    0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State( ...
    8, [4 2], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 2], [1 1], [2 2]);
options = fixedOptions();
directResult = azElPlannerMethods.hs3.plan( ...
    [], initialState, goalState, limits, options);
publicResult = planAzElMotion( ...
    [], initialState, goalState, limits, options);

verifyTrue(testCase, directResult.Success, directResult.Message);
verifyTrue(testCase, directResult.Validation.Passed, ...
    directResult.Validation.Message);
verifyEqual(testCase, directResult.Options.PlannerMethod, "hs3");
verifyEqual(testCase, ...
    directResult.SearchDiagnostics.PlannerMethod, "hs3");
verifyFalse(testCase, directResult.CompositionDiagnostics.Hs3.Enabled);
verifyFalse(testCase, directResult.CompositionDiagnostics.Hs3.Attempted);
verifyExactCompactMotion(testCase, directResult, publicResult);
end

function testDefaultOffCompositeExactlyMatchesCompactSuccessAndFailure(testCase)
% Preserve the immutable compact result for valid and infeasible requests.
initialState = testCase.TestData.Fixtures.State( ...
    0, [0 0], [0.1 0.05], [0.02 0]);
goalState = testCase.TestData.Fixtures.State( ...
    8, [4 2], [0.15 -0.05], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [2 3], [1 1.5], [2 3]);
hs3Options = fixedOptions();
compactOptions = compactOptionsFromHs3(hs3Options);
compactSuccess = planAzElMotion( ...
    [], initialState, goalState, limits, compactOptions);
compositeSuccess = planAzElMotion( ...
    [], initialState, goalState, limits, hs3Options);

verifyTrue(testCase, compactSuccess.Success, compactSuccess.Message);
verifyTrue(testCase, compositeSuccess.Validation.Passed, ...
    compositeSuccess.Validation.Message);
verifyFalse(testCase, ...
    compositeSuccess.CompositionDiagnostics.Hs3.Attempted);
verifyExactCompactMotion( ...
    testCase, compositeSuccess, compactSuccess);

shortGoalState = testCase.TestData.Fixtures.State( ...
    1, [4 0], [0 0], [0 0]);
slowLimits = testCase.TestData.Fixtures.PhysicalLimits( ...
    [1 1], [1 1], [1 1]);
compactFailure = planAzElMotion( ...
    [], initialState, shortGoalState, slowLimits, compactOptions);
compositeFailure = planAzElMotion( ...
    [], initialState, shortGoalState, slowLimits, hs3Options);

verifyFalse(testCase, compactFailure.Success);
verifyFalse(testCase, compositeFailure.Success);
verifyFalse(testCase, ...
    compositeFailure.CompositionDiagnostics.Hs3.Attempted);
verifyExactCompactMotion( ...
    testCase, compositeFailure, compactFailure);
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
quietText = evalc("planAzElMotion([], initialState, goalState, limits, options);");
verifyEmpty(testCase, strtrim(quietText));
options.Verbose = true;
verboseText = evalc("planAzElMotion([], initialState, goalState, limits, options);");
verifyNotEmpty(testCase, regexp(verboseText, '\[AzEl\]', 'once'));
verifyEmpty(testCase, regexp(verboseText, ...
    '(?m)^\[HS3\]|^\[first motion', 'once'));
end
function testNearbyObstacleClusteringChangesOnlySeedGeometry(testCase)
% Verify that three nearby regions form one seed hull without physical edits.
movingSource_deg = [-1 -4; 1 -4; 1 -2; -1 -2];
movingTarget_deg = movingSource_deg + [0 0.25];
movingObstacle = makeAzElObstacleData( ...
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
[seeds, diagnostics] = azElInternal.generateTopologySeeds( ...
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
isOccupied = queryAzElTimeObstacle( ...
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
corridor = azElInternal.buildSeedCorridor(seed, 1);
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, 1, 1:2) = [-2 4];
positionPower_deg(1, 2, 1) = 2;
polynomial = struct( ...
    "SegmentCount", 1, "positionPower_deg", positionPower_deg);
trajectory = struct( ...
    "Polynomial", polynomial, ...
    "SeedCorridorBoundary_deg", boundary_deg, ...
    "SeedCorridor", corridor);
[certified, clearance_deg] = azElInternal.certifySeedCorridor( ...
    trajectory, obstacle, 1e-7);
verifyTrue(testCase, certified);
verifyGreaterThan(testCase, clearance_deg, 0.5);
trajectory.Polynomial.positionPower_deg(1, 2, 1) = 0.5;
[certifiedAfterEntry, ~] = azElInternal.certifySeedCorridor( ...
    trajectory, obstacle, 1e-7);
verifyFalse(testCase, certifiedAfterEntry);
end
function testSeedEnvelopeRequiresCompleteContinuousContainment(testCase)
% Verify complete continuous histories stay inside one envelope region.
boundary_deg = [-2 -2; 2 -2; 2 2; -2 2];
inside = testCase.TestData.Fixtures.RectangleObstacle([0 2], [-1 1 -1 1], 0);
outside = testCase.TestData.Fixtures.RectangleObstacle([0 2], [2.5 3.5 -1 1], 0);
verifyTrue(testCase, azElInternal.seedEnvelopeContainsObstacles( ...
    boundary_deg, inside, 0));
verifyFalse(testCase, azElInternal.seedEnvelopeContainsObstacles( ...
    boundary_deg, outside, 0));
mixedHistory = inside;
mixedHistory.az_deg{2} = mixedHistory.az_deg{2} + 3;
verifyFalse(testCase, azElInternal.seedEnvelopeContainsObstacles( ...
    boundary_deg, mixedHistory, 0));
concaveBoundary_deg = [-2 -2; 2 -2; 0 0; 2 2; -2 2];
verifyFalse(testCase, azElInternal.seedEnvelopeContainsObstacles( ...
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
options.EnableHs3Improvement = true;
options.MaximumHs3ImprovementTime_s = 3;
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.velocity_deg_s(1, :), ...
    initialState.velocity_deg_s, "AbsTol", 1e-10);
verifyEqual(testCase, result.acceleration_deg_s2(1, :), ...
    initialState.acceleration_deg_s2, "AbsTol", 1e-10);
verifyEqual(testCase, result.time_s(end), 8, "AbsTol", 1e-7);
verifyEqual(testCase, result.velocity_deg_s(end, :), ...
    goalState.velocity_deg_s, "AbsTol", 1e-6);
verifyHs3Attempted(testCase, result, "linearFixedTime");
end
function testIntegratedJerkGradientMatchesDirectionalDifference(testCase)
% Verify the exact gradient includes both jerk and final-time decisions.
segmentCount = 3;
decision = [linspace(-1, 1, 2 * (2 * segmentCount + 1)).'; 8];
direction = cos((1:numel(decision)).');
[~, gradient] = azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
    decision, true, 8, segmentCount, 1);
step = 1e-6;
forward = azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
    decision + step * direction, true, 8, segmentCount, 1);
backward = azElPlannerMethods.hs3.internal.motion.integratedSquaredHs3Jerk( ...
    decision - step * direction, true, 8, segmentCount, 1);
verifyEqual(testCase, gradient.' * direction, ...
    (forward - backward) / (2 * step), "RelTol", 1e-8);
end
function testEarliestArrivalIsInsideHorizon(testCase)
% Verify the two-stage earliest-arrival solve and independent validation.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.GoalTimeMode = "earliestArrival";
options.EnableHs3Improvement = true;
options.MaximumHs3ImprovementTime_s = 3;
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyLessThan(testCase, result.time_s(end), goalState.time_s);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyHs3Attempted(testCase, result, "nonlinearTimeDecision");
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
    planAzElMotion([], initialState, goalState, limits, options);
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

function testCompositeDeclaresMeshRefinementUnsupported(testCase)
% Report the current improver limitation instead of claiming a refined result.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0.1 0], [0.05 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 2], [0.2 -0.1], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.CollocationSegmentCount = 4;
options.MaximumCollocationSegmentCount = 8;
options.MaximumMeshRefinementPasses = 1;
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyFalse(testCase, ...
    result.CompositionDiagnostics.Hs3.RefinementSupported);
verifyEqual(testCase, ...
    result.CompositionDiagnostics.Hs3.RequestedRefinementPasses, 1);
verifyFalse(testCase, result.CompositionDiagnostics.Hs3.Attempted);
verifyEqual(testCase, result.SearchDiagnostics.MeshRefinementPassCount, 0);
end
function testStaticObstacleProducesOppositeSideSeeds(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testStaticObstacleProducesOppositeSideSeeds");
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
[seeds, diagnostics] = azElInternal.generateTopologySeeds( ...
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
end
function testMovingObstacleUsesTrajectoryTime(testCase)
% Verify that the same point changes occupancy as protected geometry moves.
time_s = [0; 4];
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
azimuth_deg = {source_deg(:, 1); source_deg(:, 1)};
elevation_deg = {source_deg(:, 2); source_deg(:, 2) + 4};
obstacle = makeAzElObstacleData( ...
    "moving", time_s, azimuth_deg, elevation_deg, 0);
occupied = queryAzElTimeObstacle( ...
    obstacle, [0; 0], [0; 0], [0; 4], ...
    struct("PlannerMethod", "hs3"));
verifyEqual(testCase, occupied, [true; false]);
end
function testOccupancyOnlyMatchesDetailedQuery(testCase)
% Verify the fast occupancy path preserves moving and boundary decisions.
time_s = [0; 2];
first_deg = [-1 -1; 1 -1; 1 1; -1 1];
second_deg = first_deg + [2 0];
obstacle = makeAzElObstacleData( ...
    "moving", time_s, ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0);
queryAzimuth_deg = [0; 1; 3];
queryElevation_deg = [0; 0; 0];
queryTime_s = [0; 0; 2];
queryOptions = struct("PlannerMethod", "hs3");
fastOccupied = queryAzElTimeObstacle( ...
    obstacle, queryAzimuth_deg, queryElevation_deg, ...
    queryTime_s, queryOptions);
[detailedOccupied, ~, details] = queryAzElTimeObstacle( ...
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
obstacle = makeAzElObstacleData( ...
    "deforming", time_s, ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0);
occupied = queryAzElTimeObstacle( ...
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
obstacle = makeAzElObstacleData( ...
    "barrier", time_s, azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion("hs3");
options.MaximumSeedCount = 5;
options.SeedClusterDistance_deg = 0.6;
[seeds, diagnostics] = azElInternal.generateTopologySeeds( ...
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

% The default-off HS3 composite must retain the same validated timed wait.
options.GoalTimeMode = "fixedArrival";
options.EnableHs3Improvement = false;
compositeResult = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
compactResult = planAzElMotion( ...
    obstacle, initialState, goalState, limits, ...
    compactOptionsFromHs3(options));
verifyTrue(testCase, compositeResult.Success, compositeResult.Message);
verifyTrue(testCase, compositeResult.Validation.Passed, ...
    compositeResult.Validation.Message);
verifyEqual(testCase, ...
    compositeResult.Seeds(compositeResult.SelectedSeedIndex).Source, ...
    "directWait");
verifyExactCompactMotion(testCase, compositeResult, compactResult);
end
function testObstacleActivationSpanEnablesTimedSearch(testCase)
% Verify equal geometry can still change occupancy through its active span.
obstacle = testCase.TestData.Fixtures.RectangleObstacle([3 6], [-0.5 0.5 -2 2], 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion("hs3");
[~, diagnostics] = azElInternal.generateTopologySeeds( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchUsesExactObstacles);
end
function testTimedSeedKeepsDistinctAbsoluteDuration(testCase)
% Verify equal spatial routes keep distinct moving-obstacle time guesses.
azimuth_deg = repmat({[-1; 1; 1; -1]}, 2, 1);
elevation_deg = {[9; 9; 11; 11]; [11; 11; 13; 13]};
obstacle = makeAzElObstacleData( ...
    "far moving obstacle", [0; 20], azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion("hs3");
options.MaximumSeedCount = 5;
[seeds, diagnostics] = azElInternal.generateTopologySeeds( ...
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
options.EnableHs3Improvement = false;
result = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.SeedSummaries(1).FirstMotionValidationPassed);
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
obstacle = makeAzElObstacleData( ...
    "dense moving barrier", time_s, azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion("hs3");
options.MaximumSeedCount = 2;
[seeds, diagnostics] = azElInternal.generateTopologySeeds( ...
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
candidate = azElPlannerMethods.hs3.internal.motion.solveHs3( ...
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
function testAzimuthWrappingChangesThePhysicalRequest(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testAzimuthWrappingChangesThePhysicalRequest");
end

function testAzimuthWrappingRejectsUnmodeledPeriodicGeometry(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testAzimuthWrappingRejectsUnmodeledPeriodicGeometry");
end

function testRemovedPlanningTimeOptionGivesMigrationError(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "hs3", "testRemovedPlanningTimeOptionGivesMigrationError");
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
result = planAzElMotion(wall, initialState, goalState, limits, options);
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
handles = plotAzElMotion(result, plotOptions);
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
function testMovingTargetUsesSamePlanner(testCase)
% Verify the intercept wrapper only adapts target inputs to planAzElMotion.
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
result = planAzElMovingTargetIntercept( ...
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
handles = plotAzElMotion(result, plotOptions);
figureCleanup = onCleanup(@() closeTestFigures(handles));
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
result = planAzElMovingTargetIntercept( ...
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
verifyEqual(testCase, result.Intercept.Mode, "specifiedTime");
verifyEqual(testCase, result.Intercept.Search.Method, "specifiedFixedTime");
verifyEqual(testCase, result.Intercept.Search.TrialCount, 1);
verifyEqual(testCase, result.Intercept.TerminalVelocityPolicy, "target");
verifyEqual(testCase, result.Intercept.TerminalAccelerationPolicy, "target");
verifyEqual(testCase, result.velocity_deg_s(end, :), ...
    expectedVelocity_deg_s, "AbsTol", 1e-6);
verifyEqual(testCase, result.acceleration_deg_s2(end, :), ...
    expectedAcceleration_deg_s2, "AbsTol", 1e-6);

% The moving-target adapter must feed the identical request to compact mode.
compactInterceptOptions = interceptOptions;
compactInterceptOptions.PlannerOptions = ...
    compactOptionsFromHs3(interceptOptions.PlannerOptions);
compactResult = planAzElMovingTargetIntercept( ...
    initialState, targetMotion, limits, compactInterceptOptions);
verifyExactCompactMotion(testCase, result, compactResult);
end

function options = fixedOptions()
% Return fast deterministic settings for focused tests.
options = planAzElMotion("hs3");
options.GoalTimeMode = "fixedArrival";
options.DirectSeedOnly = true;
options.MaximumSeedCount = 1;
options.CollocationSegmentCount = 5;
options.MaximumNlpIterations = 150;
options.MaximumNlpFunctionEvaluations = 15000;
options.SampleTime_s = 0.05;
options.Verbose = false;
end

function compactOptions = compactOptionsFromHs3(hs3Options)
% Project the same public inputs onto the immutable compact baseline.
if isfield(hs3Options, "PlannerMethod")
    hs3Options = rmfield(hs3Options, "PlannerMethod");
end
[~, compactOptions] = ...
    azElPlannerMethods.hs3.resolvePlannerOptions(hs3Options);
compactOptions.PlannerMethod = "corridorQuintic";
end

function verifyExactCompactMotion(testCase, composite, compact)
% Compare every deterministic status, seed, and physical trajectory field.
exactFields = { ...
    'Success', 'Message', 'TerminationReason', 'Seeds', ...
    'SelectedSeedIndex', 'SelectedMotionSource', 'SelectedSeed_deg', ...
    'time_s', 'position_deg', 'velocity_deg_s', ...
    'acceleration_deg_s2', 'jerk_deg_s3', 'Polynomial', ...
    'SeedCorridorBoundary_deg', 'SeedCorridor', 'ArrivalTime_s', ...
    'TrajectoryDuration_s', 'GoalHorizon_s', 'RandomSeed', ...
    'OptimalityStatement'};
for fieldIndex = 1:numel(exactFields)
    field = exactFields{fieldIndex};
    verifyEqual(testCase, composite.(field), compact.(field));
end
compositeValidation = rmfield(composite.Validation, ...
    {'CollisionCheckingElapsedTime_s', 'ElapsedTime_s'});
compactValidation = rmfield(compact.Validation, ...
    {'CollisionCheckingElapsedTime_s', 'ElapsedTime_s'});
verifyEqual(testCase, compositeValidation, compactValidation);
verifyEqual(testCase, composite.CompositionDiagnostics.Baseline.Success, ...
    compact.Success);
verifyEqual(testCase, ...
    composite.CompositionDiagnostics.Baseline.ValidationPassed, ...
    compact.Success && compact.Validation.Passed);
verifyEqual(testCase, ...
    composite.CompositionDiagnostics.Baseline.SelectedSeedIndex, ...
    compact.SelectedSeedIndex);
verifyEqual(testCase, ...
    composite.CompositionDiagnostics.Baseline.ArrivalTime_s, ...
    compact.ArrivalTime_s);
end

function verifyHs3Attempted(testCase, result, representation)
% Require solver evidence without requiring the optional trial to be selected.
composition = result.CompositionDiagnostics.Hs3;
verifyTrue(testCase, composition.Enabled);
verifyTrue(testCase, composition.Attempted);
verifyNotEmpty(testCase, composition.Attempts);
verifyTrue(testCase, any([result.SeedSummaries.Hs3Attempted]));
representationFound = false;
for attemptIndex = 1:numel(composition.Attempts)
    diagnostics = composition.Attempts(attemptIndex).SolverDiagnostics;
    if isfield(diagnostics, "ConstraintRepresentation") && ...
            diagnostics.ConstraintRepresentation == representation
        representationFound = true;
        break;
    end
end
verifyTrue(testCase, representationFound);
verifyTrue(testCase, any(result.SelectedMotionSource == ...
    ["corridorQuintic", "hs3"]));
end

function closeTestFigures(handles)
% Close figures created by a plot test even when its verification fails.
figureHandles = [handles.WorkspaceFigure; handles.VisibilityFigure];
close(figureHandles(isgraphics(figureHandles, "figure")));
end
