function tests = testAzElPlanner
%% Section 0: Header & Readme
% SYNTAX
%   tests = testAzElPlanner
%**************************************************************************
% PURPOSE
%   - Verify the corridor-quintic planner, geometry interpolation, validation,
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
% Verify that zero-input defaults expose only maintained planner controls.
options = planAzElMotion();
verifyEqual(testCase, options.MaximumSeedCount, 5);
verifyEqual(testCase, options.SeedClusterDistance_deg, 0);
verifyEqual(testCase, options.GoalTimeMode, "earliestArrival");
verifyFalse(testCase, isfield(options, "UseParallel"));
verifyFalse(testCase, isfield(options, "MaximumVisibilitySnapshotsPerObstacle"));
verifyFalse(testCase, isfield(options, "MaximumPlanningTime_s"));
verifyFalse(testCase, isfield(options, "AzimuthInterval_deg"));
verifyFalse(testCase, isfield(options, "ElevationInterval_deg"));
end

function testWorkspaceIntervalsBelongToLimits(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testWorkspaceIntervalsBelongToLimits");
end

function testOldWorkspaceOptionGivesMigrationError(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testOldWorkspaceOptionGivesMigrationError");
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
verifyEmpty(testCase, regexp(verboseText, '(?m)^\[first motion', 'once'));
end

function testNearbyObstacleClusteringChangesOnlySeedGeometry(testCase)
% Verify that three nearby regions form one seed hull without physical edits.
movingSource_deg = [-1 -4; 1 -4; 1 -2; -1 -2];
movingTarget_deg = movingSource_deg + [0 0.25];
movingObstacle = makeAzElObstacleData( ...
    "moving rectangle", [0; 20], ...
    {movingSource_deg(:, 1); movingTarget_deg(:, 1)}, {movingSource_deg(:, 2); movingTarget_deg(:, 2)}, 0);
obstacles = [ movingObstacle; testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 -1.5 0.5], 0); testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 1 3], 0)];
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.DirectSeedOnly = false;
options.MaximumSeedCount = 3;
options.SeedClusterDistance_deg = 0.6;
[seeds, diagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( obstacles, initialState, goalState, limits, options);
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
verifyEqual(testCase, diagnostics.Coverage.TimedSearchSuppressionReason, "");
verifyTrue(testCase, diagnostics.Coverage.CompletenessLost);
visibilitySeeds = seeds([seeds.Source] == "visibilityGraph");
verifyTrue(testCase, all([visibilitySeeds.UsesReducedGeometry]));
isOccupied = queryAzElTimeObstacle(obstacles, 0, 0.75, 10);
verifyFalse(testCase, isOccupied);
end

function testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints");
end

function testSeedCorridorCertificateChecksCompletePolynomial(testCase)
% Verify independent containment and continuous Bernstein separation checks.
obstacle = testCase.TestData.Fixtures.RectangleObstacle([0 10], [-1 1 -1 1], 0);
boundary_deg = [-1 -1; 1 -1; 1 1; -1 1];
seed = struct( "tau", [0; 1], "position_deg", [-2 2; 2 2], "CorridorBoundary_deg", boundary_deg);
corridor = azElPlannerMethods.corridor.internal.validation.buildSeedCorridor(seed, 1);
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, 1, 1:2) = [-2 4];
positionPower_deg(1, 2, 1) = 2;
polynomial = struct( "SegmentCount", 1, "positionPower_deg", positionPower_deg);
trajectory = struct( "Polynomial", polynomial, "SeedCorridorBoundary_deg", boundary_deg, "SeedCorridor", corridor);
[certified, clearance_deg] = azElPlannerMethods.corridor.internal.validation.certifySeedCorridor( trajectory, obstacle, 1e-7);
verifyTrue(testCase, certified);
verifyGreaterThan(testCase, clearance_deg, 0.5);
trajectory.Polynomial.positionPower_deg(1, 2, 1) = 0.5;
[certifiedAfterEntry, ~] = azElPlannerMethods.corridor.internal.validation.certifySeedCorridor( trajectory, obstacle, 1e-7);
verifyFalse(testCase, certifiedAfterEntry);
end

function testSeedEnvelopeRequiresCompletePolygonContainment(testCase)
% Verify complete polygons and histories must lie in the envelope union.
boundary_deg = [-2 -2; 2 -2; 2 2; -2 2];
inside = testCase.TestData.Fixtures.RectangleObstacle([0 2], [-1 1 -1 1], 0);
outside = testCase.TestData.Fixtures.RectangleObstacle([0 2], [2.5 3.5 -1 1], 0);
verifyTrue(testCase, azElPlannerMethods.corridor.internal.validation.seedEnvelopeContainsObstacles( boundary_deg, inside, 0));
verifyFalse(testCase, azElPlannerMethods.corridor.internal.validation.seedEnvelopeContainsObstacles( boundary_deg, outside, 0));
mixedHistory = inside;
mixedHistory.az_deg{2} = mixedHistory.az_deg{2} + 3;
verifyFalse(testCase, azElPlannerMethods.corridor.internal.validation.seedEnvelopeContainsObstacles( boundary_deg, mixedHistory, 0));
concaveBoundary_deg = [-2 -2; 2 -2; 0 0; 2 2; -2 2];
verifyFalse(testCase, azElPlannerMethods.corridor.internal.validation.seedEnvelopeContainsObstacles( concaveBoundary_deg, inside, 0));
insideConcaveRegion = testCase.TestData.Fixtures.RectangleObstacle([0 2], [-1.5 -0.5 -1 1], 0);
verifyTrue(testCase, azElPlannerMethods.corridor.internal.validation.seedEnvelopeContainsObstacles( ...
    concaveBoundary_deg, insideConcaveRegion, 0));
end

function testConstantJerkPolynomialPassesIndependentDynamics(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testConstantJerkPolynomialPassesIndependentDynamics");
end

function testUnrelatedPolynomialCannotValidateSampledHistory(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testUnrelatedPolynomialCannotValidateSampledHistory");
end

function testShiftedPolynomialTimeCannotHideInitialTime(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testShiftedPolynomialTimeCannotHideInitialTime");
end

function testFixedArrivalEnforcesTerminalState(testCase)
% Verify fixed time and nonzero initial and terminal state equalities.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0.1 0], [0.05 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 2], [0.2 -0.1], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.velocity_deg_s(1, :), initialState.velocity_deg_s, "AbsTol", 1e-10);
verifyEqual(testCase, result.acceleration_deg_s2(1, :), initialState.acceleration_deg_s2, "AbsTol", 1e-10);
verifyEqual(testCase, result.time_s(end), 8, "AbsTol", 1e-7);
verifyEqual(testCase, result.velocity_deg_s(end, :), goalState.velocity_deg_s, "AbsTol", 1e-6);
verifyEqual(testCase, result.SelectedMotionSource, "corridorQuintic");
verifyEqual(testCase, result.SearchDiagnostics.PlannerMethod, "corridorQuintic");
end

function testEarliestArrivalIsInsideHorizon(testCase)
% Verify the two-stage earliest-arrival solve and independent validation.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.GoalTimeMode = "earliestArrival";
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyLessThan(testCase, result.time_s(end), goalState.time_s);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.SelectedMotionSource, "corridorQuintic");
verifyEqual(testCase, size(result.Polynomial.positionPower_deg, 3), 6);
jerkRampTime_s = limits.maxAcceleration_deg_s2(1) / limits.maxJerk_deg_s3(1);
constantAccelerationTime_s = (-1.5 + sqrt(16.25)) / 2;
independentMinimumTime_s = 2 * (constantAccelerationTime_s + 2 * jerkRampTime_s);
verifyEqual(testCase, result.time_s(end), independentMinimumTime_s, "AbsTol", 0.25);
end

function testEarliestGoalIsNotRejectedByHorizonOccupancy(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testEarliestGoalIsNotRejectedByHorizonOccupancy");
end

function testIntegerWorkLimitsRejectFractionalValues(testCase)
% Verify an invalid bounded seed-count limit fails at the public boundary.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSeedCount = 1.5;
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
    testCase, "corridor", "testMovingGoalInterpolationMethodMustBeScalar");
end

function testMovingGoalHistoryRequiresTwoSamples(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testMovingGoalHistoryRequiresTwoSamples");
end

function testInterceptWrapperTextOptionsMustBeScalar(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testInterceptWrapperTextOptionsMustBeScalar");
end

function testInterceptWrapperRequiresTwoTargetSamples(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testInterceptWrapperRequiresTwoTargetSamples");
end

function testEndpointStateDoesNotInvokeLegacyMeshRefinement(testCase)
% Verify endpoint-state support stays on the experimental polynomial path.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0.1 0], [0.05 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 2], [0.2 -0.1], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
result = planAzElMotion([], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.SelectedMotionSource, "corridorQuintic");
verifyEqual(testCase, result.SearchDiagnostics.MeshRefinementPassCount, 0);
end

function testStaticObstacleProducesOppositeSideSeeds(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testStaticObstacleProducesOppositeSideSeeds");
end

function testTwoObstacleHomologySearchFindsDistinctClasses(testCase)
% Verify signature diversity for a structurally different obstacle field.
obstacles = [ testCase.TestData.Fixtures.RectangleObstacle([0 20], [-3 -1 -1 1], 0.1); testCase.TestData.Fixtures.RectangleObstacle([0 20], [1 3 -1 1], 0.1)];
initialState = testCase.TestData.Fixtures.State(0, [-6 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [6 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSeedCount = 5;
options.DirectSeedOnly = false;
[seeds, diagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( obstacles, initialState, goalState, limits, options);
verifySize(testCase, diagnostics.HomologyRepresentative_deg, [2 2]);
verifyEqual(testCase, size(diagnostics.HomologyClassSignatures, 2), 2);
verifyGreaterThanOrEqual(testCase, diagnostics.HomologyClassCount, 2);
verifyFalse(testCase, diagnostics.ExhaustiveVisibilityFallbackUsed);
verifyLessThan(testCase, diagnostics.VisibilityCandidatePairCount, ...
    diagnostics.NodeCount * (diagnostics.NodeCount - 1) / 2);
verifyEqual(testCase, size(unique( diagnostics.HomologyClassSignatures, "rows"), 1), diagnostics.HomologyClassCount);
verifyGreaterThanOrEqual(testCase, sum([seeds.Source] == "visibilityGraph"), 2);
verifyFalse(testCase, diagnostics.HomologySearchTruncated);
end

function testAffordableExhaustiveVisibilityFindsHairpinRoute(testCase)
% Verify affordable complete visibility preserves a two-wall reversal path.
firstWall = makeAzElObstacleData( "first wall", 0, [-10; 6.5; 6.5; -10], [3.65; 3.65; 4.35; 4.35], 0.15);
secondWall = makeAzElObstacleData( "second wall", 0, [-6.5; 10; 10; -6.5], [7.65; 7.65; 8.35; 8.35], 0.15);
obstacles = [firstWall; secondWall];
initialState = testCase.TestData.Fixtures.State(0, [0 2], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(120, [0 10], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
limits.azimuthInterval_deg = [-10 10];
limits.elevationInterval_deg = [0 12];
options = fixedOptions();
options.MaximumSeedCount = 3;
options.DirectSeedOnly = false;
[seeds, diagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( obstacles, initialState, goalState, limits, options);
visibilitySeeds = seeds([seeds.Source] == "visibilityGraph");

verifyNotEmpty(testCase, visibilitySeeds);
verifyTrue(testCase, diagnostics.ExhaustiveVisibilityUsed);
verifyTrue(testCase, diagnostics.ExhaustiveVisibilityFallbackUsed);
verifyFalse(testCase, diagnostics.HomologySearchTruncated);
route_deg = visibilitySeeds(1).position_deg;

% Sample every route edge densely to prove the complete polyline stays outside the obstacle.
for edgeIndex = 1:size(route_deg, 1) - 1
    edgeLength_deg = norm(route_deg(edgeIndex + 1, :) - route_deg(edgeIndex, :));
    sampleCount = ceil(edgeLength_deg / 0.01) + 1;
    edgeFraction = linspace(0, 1, sampleCount).';
    edgeSamples_deg = route_deg(edgeIndex, :) + edgeFraction .* (route_deg(edgeIndex + 1, :) - route_deg(edgeIndex, :));
    occupied = queryAzElTimeObstacle( obstacles, edgeSamples_deg(:, 1), edgeSamples_deg(:, 2), zeros(sampleCount, 1));
    verifyFalse(testCase, any(occupied));
end
end

function testMovingObstacleUsesTrajectoryTime(testCase)
% Verify that the same point changes occupancy as protected geometry moves.
time_s = [0; 4];
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
azimuth_deg = {source_deg(:, 1); source_deg(:, 1)};
elevation_deg = {source_deg(:, 2); source_deg(:, 2) + 4};
obstacle = makeAzElObstacleData( "moving", time_s, azimuth_deg, elevation_deg, 0);
occupied = queryAzElTimeObstacle( obstacle, [0; 0], [0; 0], [0; 4]);
verifyEqual(testCase, occupied, [true; false]);
end

function testOccupancyOnlyMatchesDetailedQuery(testCase)
% Verify the fast occupancy path preserves moving and boundary decisions.
time_s = [0; 2];
first_deg = [-1 -1; 1 -1; 1 1; -1 1];
second_deg = first_deg + [2 0];
obstacle = makeAzElObstacleData( ...
    "moving", time_s, {first_deg(:, 1); second_deg(:, 1)}, {first_deg(:, 2); second_deg(:, 2)}, 0);
queryAzimuth_deg = [0; 1; 3];
queryElevation_deg = [0; 0; 0];
queryTime_s = [0; 0; 2];
fastOccupied = queryAzElTimeObstacle( obstacle, queryAzimuth_deg, queryElevation_deg, queryTime_s);
[detailedOccupied, ~, details] = queryAzElTimeObstacle( obstacle, queryAzimuth_deg, queryElevation_deg, queryTime_s);
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
    "deforming", time_s, {first_deg(:, 1); second_deg(:, 1)}, {first_deg(:, 2); second_deg(:, 2)}, 0);
occupied = queryAzElTimeObstacle( obstacle, [1.5; 1.5], [0; 0], [0; 2]);
verifyEqual(testCase, occupied, [false; true]);
end

function testDeformingObstacleUsesThePlannerPath(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testDeformingObstacleUsesThePlannerPath");
end

function testTopologyChangeUsesAStationaryConservativeUnion(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testTopologyChangeUsesAStationaryConservativeUnion");
end

function testStationaryMatchingTopologyReusesExactShape(testCase)
% Verify an unchanged interval returns its prepared source geometry exactly.
boundary_deg = [-2 -1; 2 -1; 2 1; -2 1];
obstacle = makeAzElObstacleData( "stationary", [0; 2], boundary_deg(:, 1), boundary_deg(:, 2), 0);
preparedObstacle = azElInternal.obstacles.prepareDynamic(obstacle);
[shape, geometry] = azElInternal.obstacles.shapeAtTime(preparedObstacle, 1);
verifyEqual(testCase, shape.Vertices, preparedObstacle. InternalPreparation.SampleShapes{1}.Vertices);
verifyEqual(testCase, geometry.azimuth_deg, obstacle.az_deg{1});
verifyEqual(testCase, geometry.elevation_deg, obstacle.el_deg{1});
verifyEqual(testCase, geometry.VertexSpeedBound_deg_s, 0);
verifyTrue(testCase, geometry.TopologyIsInterpolated);
end

function testBatchedPolygonClearanceMatchesScalarQueries(testCase)
% Verify batched projection preserves scalar signs, points, and edge order.
shape = polyshape([0 3 3 1 1 0], [0 0 3 3 1 1], "Simplify", false, "KeepCollinearPoints", true);
queryPosition_deg = [-1 0.5; 0.5 0.5; 2 2; 1 1; 3.5 2.5];
[batchClearance_deg, batchNearest_deg, batchEdgeIndex] = azElInternal.geometry.pointPolygonClearance(shape, queryPosition_deg);

% Compare each scalar clearance query with the corresponding batched result.
for queryIndex = 1:size(queryPosition_deg, 1)
    [clearance_deg, nearest_deg, edgeIndex] = azElInternal.geometry.pointPolygonClearance( ...
        shape, queryPosition_deg(queryIndex, :));
    verifyEqual(testCase, batchClearance_deg(queryIndex), clearance_deg);
    verifyEqual(testCase, batchNearest_deg(queryIndex, :), nearest_deg);
    verifyEqual(testCase, batchEdgeIndex(queryIndex), edgeIndex);
end
end

function testMovingBarrierGeneratesWaitingSeed(testCase)
% Verify a requested but unused cluster does not suppress timed search.
time_s = [0; 6; 8; 12];
source_deg = [-0.5 -2; 0.5 -2; 0.5 2; -0.5 2];
centerElevation_deg = [0; 0; 6; 6];
azimuth_deg = cell(4, 1);
elevation_deg = cell(4, 1);

% Translate the source boundary to each sampled center elevation in the history.
for sampleIndex = 1:4
    translated_deg = source_deg + [0 centerElevation_deg(sampleIndex)];
    azimuth_deg{sampleIndex} = translated_deg(:, 1);
    elevation_deg{sampleIndex} = translated_deg(:, 2);
end
obstacle = makeAzElObstacleData( "barrier", time_s, azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion();
options.MaximumSeedCount = 5;
options.SeedClusterDistance_deg = 0.6;
[seeds, diagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( obstacle, initialState, goalState, limits, options);
verifyEqual(testCase, diagnostics.SeedCluster.ClusterGroupCount, 0);
verifyTrue(testCase, any([seeds.Source] == "directWait"));
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchUsesExactObstacles);
verifyEqual(testCase, diagnostics.Coverage.TimedSearchSuppressionReason, "");
verifyGreaterThan(testCase, diagnostics.ExpandedCount, 0);
verifyEqual(testCase, diagnostics.GraphType, "timeExpandedVisibilityGraph");
verifyGreaterThan(testCase, diagnostics.TemporalLayerCount, 1);
verifyGreaterThan(testCase, diagnostics.WaitEdgeCount, 0);
end

function testObstacleActivationSpanEnablesTimedSearch(testCase)
% Verify equal geometry can still change occupancy through its active span.
obstacle = testCase.TestData.Fixtures.RectangleObstacle([3 6], [-0.5 0.5 -2 2], 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion();
[~, diagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchUsesExactObstacles);
end

function testTimedSeedKeepsDistinctAbsoluteDuration(testCase)
% Verify equal spatial routes keep distinct moving-obstacle time guesses.
azimuth_deg = repmat({[-1; 1; 1; -1]}, 2, 1);
elevation_deg = {[9; 9; 11; 11]; [11; 11; 13; 13]};
obstacle = makeAzElObstacleData( "far moving obstacle", [0; 20], azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(20, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion();
options.MaximumSeedCount = 5;
[seeds, diagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( obstacle, initialState, goalState, limits, options);
directSeed = seeds([seeds.Source] == "directVisibilityEdge");
timedSeed = seeds([seeds.Source] == "timeExpandedVisibilityGraph");
verifyNotEmpty(testCase, directSeed);
verifyNotEmpty(testCase, timedSeed);
verifyEqual(testCase, directSeed(1).position_deg, timedSeed(1).position_deg);
verifyGreaterThan(testCase, timedSeed(1).EstimatedDuration_s, directSeed(1).EstimatedDuration_s);
verifyTrue(testCase, diagnostics.Coverage.TimedSearchAttempted);
options.GoalTimeMode = "fixedArrival";
result = planAzElMotion( obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.SeedSummaries(1).FirstMotionValidationPassed);
verifyTrue(testCase, all([result.SeedSummaries.FirstMotionAttempted]));
verifyEqual(testCase, result.SearchDiagnostics.AttemptedSeedCount, numel(result.Seeds));
verifyEqual(testCase, result.SelectedMotionSource, "corridorQuintic");
end

function testDenseEnvelopeReportsTimedSearchWorkLimit(testCase)
% Verify dense timed work is suppressed and reported as incomplete.
time_s = [0; 6; 8; 12];
angle_rad = (0:1199).' * (2 * pi / 1200);
source_deg = [0.5 * cos(angle_rad), 2 * sin(angle_rad)];
centerElevation_deg = [0; 0; 6; 6];
azimuth_deg = repmat({source_deg(:, 1)}, 4, 1);
elevation_deg = cell(4, 1);

% Build every elevation slice of the dense moving barrier from the same source boundary.
for sampleIndex = 1:4
    elevation_deg{sampleIndex} = source_deg(:, 2) + centerElevation_deg(sampleIndex);
end
obstacle = makeAzElObstacleData( "dense moving barrier", time_s, azimuth_deg, elevation_deg, 0);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = planAzElMotion();
options.MaximumSeedCount = 2;
[seeds, diagnostics] = azElPlannerMethods.corridor.internal.search.generateTopologySeeds( obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, diagnostics.DenseSeedEnvelopeUsed);
verifyTrue(testCase, diagnostics.Coverage.ReducedSpatialProposalUsed);
verifyFalse(testCase, diagnostics.Coverage.TimedSearchAttempted);
verifyEqual(testCase, diagnostics.Coverage.TimedSearchSuppressionReason, "timedQueryWorkLimit");
directWaitSeeds = seeds([seeds.Source] == "directWait");
verifyEmpty(testCase, directWaitSeeds);
spatialSeeds = seeds([seeds.Source] == "visibilityGraph");
verifyNotEmpty(testCase, spatialSeeds);
verifyTrue(testCase, all([spatialSeeds.UsesReducedGeometry]));
verifyTrue(testCase, diagnostics.Coverage.CompletenessLost);
end

function testAzimuthWrappingChangesThePhysicalRequest(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testAzimuthWrappingChangesThePhysicalRequest");
end

function testAzimuthWrappingRejectsUnmodeledPeriodicGeometry(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testAzimuthWrappingRejectsUnmodeledPeriodicGeometry");
end

function testRemovedPlanningTimeOptionGivesMigrationError(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testRemovedPlanningTimeOptionGivesMigrationError");
end

function testEarlyPlannerFailureKeepsValidationFieldOrder(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testEarlyPlannerFailureKeepsValidationFieldOrder");
end

function testBetweenNodeCollisionFailsValidation(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testBetweenNodeCollisionFailsValidation");
end

function testObstacleActivationAtTerminalTimeFailsValidation(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testObstacleActivationAtTerminalTimeFailsValidation");
end

function testBetweenNodeVelocityViolationFailsValidation(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testBetweenNodeVelocityViolationFailsValidation");
end

function testExactInteriorVelocityPeakAtLimitPassesValidation(testCase)
% Verify a conservative coefficient hull does not reject a feasible curve.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [4 0]);
goalState = testCase.TestData.Fixtures.State(1, [2 / 3 0], [0 0], [-4 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([1 1], [5 5], [9 9]);
trajectory = testCase.TestData.Fixtures.InteriorVelocityPeakTrajectory();
validation = validateAzElTrajectory( trajectory, [], initialState, goalState, limits, fixedOptions());
verifyTrue(testCase, validation.Passed, validation.Message);
verifyTrue(testCase, validation.VelocityWithinLimits);
end

function testSafetyMarginIsAppliedExactlyOnce(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testSafetyMarginIsAppliedExactlyOnce");
end

function testTranslatedHistoryReusesExactProtectedShape(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testTranslatedHistoryReusesExactProtectedShape");
end

function testDeterministicRepeatedRun(testCase)
testSupport.verifySharedPlannerContract( ...
    testCase, "corridor", "testDeterministicRepeatedRun");
end

function testNoPathReturnsStableDiagnostics(testCase)
% Verify expected no-path failure returns diagnostics instead of an error.
wall = testCase.TestData.Fixtures.RectangleObstacle([0 12], [-0.5 0.5 -90 90], 0.2);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = fixedOptions();
options.MaximumSeedCount = 3;
limits.elevationInterval_deg = [-10 10];
result = planAzElMotion(wall, initialState, goalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyEqual(testCase, numel(result.SeedSummaries), numel(result.Seeds));
verifyGreaterThanOrEqual(testCase, result.SearchDiagnostics.Grid.ExpandedCount, 0);
verifyEqual(testCase, size(result.SearchDiagnostics.Grid.FrontierNodes_deg, 2), 2);
plotOptions = struct( ...
    "FigureVisible", "off", ...
    "ShowWorkspace", true, "ShowVisibilityGraphs", true, "ShowKinematics", false, "ShowAnimation", false);
handles = plotAzElMotion(result, plotOptions);
figureCleanup = onCleanup(@() closeTestFigures(handles));
verifyTrue(testCase, isgraphics(handles.WorkspaceFigure, "figure"));
verifyTrue(testCase, isgraphics(handles.VisibilityFigure, "figure"));
verifyNotEqual(testCase, numel(wall.originalAz_deg{1}), numel(wall.az_deg{1}));
verifyNotEmpty(testCase, findobj( handles.WorkspaceAxes, "DisplayName", "Original obstacle"));
surfaceHandles = findobj(handles.VisibilityAxes, "Type", "surface");
if ~isempty(surfaceHandles)
    verifyTrue(testCase, isnumeric(surfaceHandles(1).EdgeColor));
end
end

function testMovingTargetUsesSamePlanner(testCase)
% Verify the intercept wrapper only adapts target inputs to planAzElMotion.
targetTime_s = (0:5:20).';
targetPosition_deg = [6 + 0.1 * targetTime_s, ones(size(targetTime_s))];
targetMotion = struct( "time_s", targetTime_s, "position_deg", targetPosition_deg, "InterpolationMethod", "linear");
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
plannerOptions = fixedOptions();
plannerOptions.GoalTimeMode = "earliestArrival";
interceptOptions = struct( ...
    "InterceptMode", "earliest", "MaximumSearchDuration_s", 20, "PlannerOptions", plannerOptions);
result = planAzElMovingTargetIntercept( initialState, targetMotion, limits, interceptOptions);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.Intercept.Mode, "earliest");
verifyEqual(testCase, result.SelectedMotionSource, "corridorQuintic");
verifyGreaterThan(testCase, result.Intercept.Search.TrialCount, 1);
verifyEqual(testCase, result.Intercept.Search.Method, "boundedChronologicalFixedTime");
verifyEqual(testCase, result.position_deg(end, :), result.Intercept.TargetPosition_deg, "AbsTol", 1e-6);
plotOptions = struct( ...
    "FigureVisible", "off", "ShowWorkspace", true, ...
    "ShowVisibilityGraphs", true, "ShowKinematics", false, "ShowAnimation", true, "FrameStride", numel(result.time_s));
handles = plotAzElMotion(result, plotOptions);
figureCleanup = onCleanup(@() closeTestFigures(handles));
axesHandles = [handles.WorkspaceAxes, handles.VisibilityAxes, handles.AnimationAxes];

% Confirm the moving target is rendered on every axes that should display it.
for axesIndex = 1:numel(axesHandles)
    targetHandles = findobj(axesHandles(axesIndex), "DisplayName", "Moving target");
    verifyNotEmpty(testCase, targetHandles);
end
end

function options = fixedOptions()
% Return fast deterministic settings for focused tests.
options = planAzElMotion();
options.GoalTimeMode = "fixedArrival";
options.DirectSeedOnly = true;
options.MaximumSeedCount = 1;
options.SampleTime_s = 0.05;
options.Verbose = false;
end

function closeTestFigures(handles)
% Close figures created by a plot test even when its verification fails.
figureHandles = [handles.WorkspaceFigure; handles.VisibilityFigure];
close(figureHandles(isgraphics(figureHandles, "figure")));
end
