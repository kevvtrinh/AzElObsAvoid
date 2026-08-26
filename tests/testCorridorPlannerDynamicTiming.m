function tests = testCorridorPlannerDynamicTiming
%% Section 0: Header & Readme
% SYNTAX
%   tests = testCorridorPlannerDynamicTiming
%**************************************************************************
% PURPOSE
%   - Collect deterministic regression tests for corridor-planner timing,
%     dynamic-obstacle handling, and bounded recovery behavior.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (MATLAB function-test array)
%       Local test functions discovered by functiontests.
%**************************************************************************
% UNITS
%   - Test fixtures use degrees, seconds, and their time derivatives.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add planner folders once and remember the caller's path so the suite leaves MATLAB unchanged.
projectRoot = fileparts(fileparts(mfilename("fullpath")));
testCase.TestData.OriginalPath = path;
addpath(projectRoot);
addpath(fullfile(projectRoot, "examples"));
end

function teardownOnce(testCase)
% Restore the exact MATLAB path captured by setupOnce.
path(testCase.TestData.OriginalPath);
end

function testLateOpeningUsesInputDrivenTimingBracket(testCase)
% Verify the planner discovers the delayed opening by adjusting an explicit hold rather than using scenario knowledge.
[obstacles, initialState, goalState, limits] = lateOpeningRequest();
result = planAzElMotion( obstacles, initialState, goalState, limits, plannerOptions(5));

verifyExperimentalSuccess(testCase, result);
selected = result.SeedSummaries(result.SelectedSeedIndex).SolverDiagnostics;
verifyTrue(testCase, isfield(selected, "CompactC3"));
verifyTrue(testCase, selected.CompactC3.Accepted);
verifyGreaterThan(testCase, selected.CompactC3.TrialCount, 1);
end

function testTranslatingCurvedObstacleRetainsValidatedTiming(testCase)
% Verify a curved obstacle translating through time still produces an independently valid trajectory.
[obstacles, initialState, goalState, limits] = translatingCircleRequest();
result = planAzElMotion( obstacles, initialState, goalState, limits, plannerOptions(3));

verifyExperimentalSuccess(testCase, result);
end

function testUnusedTimedSlotAdmitsSpatialDiversity(testCase)
% Verify an unused timed-seed slot can retain a geometrically different route around moving obstacles.
[obstacles, initialState, goalState, limits] = movingCircleRequest(3267864, 6);
result = planAzElMotion( obstacles, initialState, goalState, limits, plannerOptions(3));

verifyExperimentalSuccess(testCase, result);
verifyEqual(testCase, numel(result.Seeds), 3);
verifyTrue(testCase, any([result.Seeds.Source] == "directWait"));
verifyTrue(testCase, any([result.Seeds.Source] == "visibilityGraph"));
end

function testShallowCollisionResidualUsesBoundedFeedback(testCase)
% Verify bounded geometry feedback resolves a shallow dynamic collision without replacing the selected topology.
[obstacles, initialState, goalState, limits] = movingCircleRequest(3259945, 4);
result = planAzElMotion( obstacles, initialState, goalState, limits, plannerOptions(3, 3259945));

verifyExperimentalSuccess(testCase, result);
summary = result.SeedSummaries(result.SelectedSeedIndex);
verifyEqual(testCase, summary.SeedSource, "visibilityGraph");
verifyTrue(testCase, summary.SolverDiagnostics.CompactC3.Accepted);
verifyGreaterThan(testCase, summary.SolverDiagnostics.CompactC3.QpCount, 0);
verifyGreaterThan(testCase, result.Validation.MinimumClearance_deg, 0);
end

function testOffRouteMoverDoesNotEvictRelevantTimedSeed(testCase)
% Verify an independently clear mover does not displace the route-shaping timed proposal.
[nearObstacle, farObstacle, initialState, target, limits, options] = ...
    twoMoverInterceptRequest(5202);
base = planAzElMovingTargetIntercept( ...
    nearObstacle, initialState, target, limits, options);
added = planAzElMovingTargetIntercept( ...
    combineAzElObstacles(farObstacle, nearObstacle), ...
    initialState, target, limits, options);

verifyExperimentalSuccess(testCase, base);
verifyExperimentalSuccess(testCase, added);
farValidation = validateAzElTrajectory( ...
    base, farObstacle, base.Inputs.initialState, ...
    base.Inputs.goalState, base.Inputs.limits, base.Options);
verifyTrue(testCase, farValidation.Passed, farValidation.Message);
verifyGreaterThan(testCase, farValidation.MinimumClearance_deg, 4);
verifyEqual(testCase, added.position_deg, base.position_deg, "AbsTol", 1e-8);
end

function testDisconnectedSparseGraphUsesWorkspaceBoundarySupport(testCase)
% Verify four deterministic moving fields that need spatial detour support.
caseSeeds = [3275783 3283702 3299540 3315378];
obstacleCounts = [8 10 14 18];

% Replay each fixed random case to preserve reproducible dynamic-planning evidence.
for caseIndex = 1:numel(caseSeeds)
    [obstacles, initialState, goalState, limits] = movingCircleRequest(caseSeeds(caseIndex), obstacleCounts(caseIndex));
    result = planAzElMotion( obstacles, initialState, goalState, limits, plannerOptions(3, caseSeeds(caseIndex)));

    verifyExperimentalSuccess(testCase, result);
    if ~result.Success || ~result.Validation.Passed
        continue;
    end
    summary = result.SeedSummaries(result.SelectedSeedIndex);
    grid = result.SearchDiagnostics.Grid;
    verifyEqual(testCase, summary.SeedSource, "visibilityGraph");
    verifyGreaterThanOrEqual(testCase, grid.CandidateOffsetRetryCount, 3);
    verifyGreaterThan(testCase, grid.HomologyClassCount, 0);
    verifyGreaterThan(testCase, result.Validation.MinimumClearance_deg, 0);
end
end

function testStaticWorkspaceWallRemainsExpectedFailure(testCase)
% Verify a wall spanning the workspace remains a stable, diagnosable no-path result.
time_s = [0; 20];
boundary_deg = [-0.4 -4; 0.4 -4; 0.4 4; -0.4 4];
obstacles = makeAzElObstacleData( ...
    "workspace-spanning wall", time_s, ...
    {boundary_deg(:, 1); boundary_deg(:, 1)}, {boundary_deg(:, 2); boundary_deg(:, 2)}, 0.1);
initialState = restState(0, [-3 0]);
goalState = restState(20, [3 0]);
limits = motionLimits([-3.5 3.5], [-3.5 3.5]);

result = planAzElMotion( obstacles, initialState, goalState, limits, plannerOptions(3));

verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyEqual(testCase, result.SearchDiagnostics.PlannerMethod, "corridorQuintic");
end

function testSmallStaticCorridorUsesValidatedTimingController(testCase)
% Verify the compact controller reuses one affine basis across duration trials.
result = exampleUShapedAzElTimeSpace(struct("PlotOutputs", false));

verifyExperimentalSuccess(testCase, result);
diagnostics = result.SeedSummaries( result.SelectedSeedIndex).SolverDiagnostics;
verifyTrue(testCase, isfield(diagnostics, "CompactC3"));
verifyTrue(testCase, diagnostics.CompactC3.Accepted);
verifyGreaterThan(testCase, diagnostics.CompactC3.QpCount, 0);
verifyEqual(testCase, diagnostics.CompactC3.AffineBasisBuildCount, 1);
verifyEqual(testCase, diagnostics.CompactC3.AffineBasisReuseCount, diagnostics.CompactC3.TrialCount);
verifyLessThanOrEqual(testCase, result.TrajectoryDuration_s, 22.818548735851);
end

function testLongStaticRouteBoundsExactCorridorDecisionGrowth(testCase)
% Bound QP growth while retaining geometric time allocation after refinement.
azimuth_deg = [(0:10), 20].';
route_deg = [azimuth_deg, zeros(size(azimuth_deg))];
seed = struct("position_deg", route_deg, ...
    "tau", linspace(0, 1, size(route_deg, 1)).', ...
    "Source", "visibilityGraph");
initialState = restState(0, route_deg(1, :));
goalState = restState(60, route_deg(end, :));
limits = motionLimits([-2 22], [-2 8]);
obstacle = makeAzElObstacleData( ...
    "far static obstacle", 0, [4; 6; 6; 4], [4; 4; 6; 6], 0);
obstacle = azElInternal.obstacles.prepareDynamic(obstacle);
options = planAzElMotion("corridorQuintic");

[candidate, diagnostics] = ...
    azElPlannerMethods.corridor.internal.motion.solveCompactC3Candidate( ...
    seed, obstacle, initialState, goalState, limits, options);

expectedDecisionCount = 2 * (diagnostics.SpanCount - 1);
spanDurationRatio = max(candidate.Polynomial.SegmentDuration_s) / ...
    min(candidate.Polynomial.SegmentDuration_s);
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyEqual(testCase, diagnostics.Representation, "C4ExactCorridor");
verifyEqual(testCase, diagnostics.DecisionCount, expectedDecisionCount);
verifyGreaterThan(testCase, diagnostics.TrialCount, 6);
verifyLessThanOrEqual(testCase, diagnostics.TrialCount, 14);
verifyGreaterThan(testCase, spanDurationRatio, 2);
end

function testStaticArrivalWindowRetainsFinalDurationBracket(testCase)
% Verify a static request whose route-derived brackets all exceed the allowed
% arrival still searches the window the request itself permits.
windowEnd_s = 40;
[obstacles, initialState, goalState, limits] = staticArcRequest(windowEnd_s);
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions(5));

verifyExperimentalSuccess(testCase, result);
verifyEqual(testCase, ...
    result.SearchDiagnostics.Grid.Coverage.TimedSearchSuppressionReason, ...
    "staticObstacleHistory");
verifyLessThanOrEqual(testCase, result.ArrivalTime_s, windowEnd_s);
compact = result.SeedSummaries(result.SelectedSeedIndex).SolverDiagnostics.CompactC3;
verifyTrue(testCase, compact.Accepted);

% Every route-length bracket must have been exhausted, leaving the request's
% own arrival window as the bracket that produced the accepted motion.
verifyGreaterThan(testCase, numel(compact.BracketAttempts), 1);
verifyTrue(testCase, isnan(compact.BracketFraction));
verifyEqual(testCase, compact.InitialDuration_s, ...
    goalState.time_s - initialState.time_s, "AbsTol", 1e-9);
end

function testStaticWideArrivalWindowKeepsRouteDerivedBracket(testCase)
% Verify the added window bracket stays unused when a route-derived bracket
% already produces an accepted motion.
[obstacles, initialState, goalState, limits] = staticArcRequest(50);
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions(5));

verifyExperimentalSuccess(testCase, result);
compact = result.SeedSummaries(result.SelectedSeedIndex).SolverDiagnostics.CompactC3;
verifyTrue(testCase, compact.Accepted);
verifyFalse(testCase, isnan(compact.BracketFraction));
end

function [obstacles, initialState, goalState, limits] = lateOpeningRequest()
% Build a concave barrier that changes from closed to open at a known physical time.
openingTime_s = 12;
transitionHalfWidth_s = 1e-3;
closedBoundary_deg = [ -9 8; -6 8; -6 -5; 6 -5; 6 8; 9 8; 9 -8; -9 -8];
leftOpen_deg = [-9 8; -6 8; -6 -5; -2 -5; -2 -8; -9 -8];
rightOpen_deg = [6 8; 9 8; 9 -8; 2 -8; 2 -5; 6 -5];
openBoundary_deg = [leftOpen_deg; NaN NaN; rightOpen_deg];
time_s = [0; openingTime_s - transitionHalfWidth_s; openingTime_s + transitionHalfWidth_s; 100];
obstacles = makeAzElObstacleData( ...
    "late-opening concavity", time_s, ...
    {closedBoundary_deg(:, 1); closedBoundary_deg(:, 1); ...
    openBoundary_deg(:, 1); openBoundary_deg(:, 1)}, ...
    {closedBoundary_deg(:, 2); closedBoundary_deg(:, 2); openBoundary_deg(:, 2); openBoundary_deg(:, 2)}, 0.2);
initialState = restState(0, [0 1]);
goalState = restState(100, [0 -11]);
limits = motionLimits([-12 12], [-12 12]);
end

function [obstacles, initialState, goalState, limits] = translatingCircleRequest()
% Build a circular obstacle whose center translates across the requested time interval.
time_s = [0; 16];
angle_rad = (0:31).' * (2 * pi / 32);
radius_deg = 1.35;
centerElevation_deg = [-0.5; 3.2];
azimuth_deg = cell(2, 1);
elevation_deg = cell(2, 1);

% Build the two source slices of the translating curved obstacle history.
for index = 1:2
    azimuth_deg{index} = radius_deg * cos(angle_rad);
    elevation_deg{index} = centerElevation_deg(index) + radius_deg * sin(angle_rad);
end
obstacles = makeAzElObstacleData( "translating curved body", time_s, azimuth_deg, elevation_deg, 0.1);
initialState = restState(0, [-6.5 -0.5]);
goalState = restState(16, [6.5 -0.5]);
limits = motionLimits([-8 8], [-5 6]);
end

function [obstacles, initialState, goalState, limits] = movingCircleRequest(randomSeed, obstacleCount)
% Build a deterministic moving-circle field with a clear upper witness lane.
stream = RandStream("mt19937ar", "Seed", randomSeed);
missionEndTime_s = 80;
obstacleTime_s = [0; 20; 40; 60; missionEndTime_s];
angle_rad = (0:15).' * (2 * pi / 16);
obstacleByIndex = cell(obstacleCount, 1);
centerAzimuth_deg = linspace(-13, 13, obstacleCount).' + 0.8 * (rand(stream, obstacleCount, 1) - 0.5);
baseElevation_deg = 2.5 * (rand(stream, obstacleCount, 1) - 0.5);
radius_deg = 0.7 + 0.45 * rand(stream, obstacleCount, 1);
amplitude_deg = 1.5 + 1.5 * rand(stream, obstacleCount, 1);
phase_rad = 2 * pi * rand(stream, obstacleCount, 1);

% Construct every randomized moving circle while preserving the seeded draw order.
for obstacleIndex = 1:obstacleCount
    azimuthByTime_deg = cell(numel(obstacleTime_s), 1);
    elevationByTime_deg = cell(numel(obstacleTime_s), 1);

    % Sample this obstacle's sinusoidal center motion at every history time.
    for timeIndex = 1:numel(obstacleTime_s)
        normalizedTime = obstacleTime_s(timeIndex) / missionEndTime_s;
        centerElevation_deg = baseElevation_deg(obstacleIndex) + ...
            amplitude_deg(obstacleIndex) * sin( 2 * pi * normalizedTime + phase_rad(obstacleIndex));
        centerAzimuthAtTime_deg = centerAzimuth_deg(obstacleIndex) + ...
            0.5 * cos(pi * normalizedTime + phase_rad(obstacleIndex));
        azimuthByTime_deg{timeIndex} = centerAzimuthAtTime_deg + radius_deg(obstacleIndex) * cos(angle_rad);
        elevationByTime_deg{timeIndex} = centerElevation_deg + radius_deg(obstacleIndex) * sin(angle_rad);
    end
    obstacleByIndex{obstacleIndex} = makeAzElObstacleData( ...
        "moving coverage circle " + obstacleIndex, obstacleTime_s, azimuthByTime_deg, elevationByTime_deg, 0.1);
end

obstacles = combineAzElObstacles(obstacleByIndex{:});
initialState = restState(0, [-17 0]);
goalState = restState(missionEndTime_s, [17 0]);
limits = motionLimits([-19 19], [-12 12]);
end

function [nearObstacle, farObstacle, initialState, target, limits, options] = ...
        twoMoverInterceptRequest(randomSeed)
% Reproduce two similarly sized rotating movers with only one shaping the route.
stream = RandStream("mt19937ar", "Seed", randomSeed);
obstacleTime_s = [0; 5; 10; 15; 20];
unitBoundary_deg = [ ...
    1 0; 0.4 0.4; 0 1; -0.4 0.4; ...
    -1 0; -0.4 -0.4; 0 -1; 0.4 -0.4];
localBoundary_deg = (1.35 + 0.35 * rand(stream)) * unitBoundary_deg;
nearAzimuth_deg = cell(numel(obstacleTime_s), 1);
nearElevation_deg = cell(numel(obstacleTime_s), 1);
farAzimuth_deg = cell(numel(obstacleTime_s), 1);
farElevation_deg = cell(numel(obstacleTime_s), 1);

for timeIndex = 1:numel(obstacleTime_s)
    fraction = (timeIndex - 1) / (numel(obstacleTime_s) - 1);
    angle_rad = deg2rad(180 * fraction + 30 * rand(stream));
    rotation = [cos(angle_rad) -sin(angle_rad); ...
        sin(angle_rad) cos(angle_rad)];
    nearCenter_deg = [3.4 + 1.2 * rand(stream), ...
        -0.8 + 1.6 * sin(pi * fraction + 2 * pi * rand(stream))];
    farCenter_deg = [-4 + 16 * fraction, -(7.5 + rand(stream))];
    nearBoundary_deg = localBoundary_deg * rotation.' + nearCenter_deg;
    farBoundary_deg = localBoundary_deg * rotation.' + farCenter_deg;
    nearAzimuth_deg{timeIndex} = nearBoundary_deg(:, 1);
    nearElevation_deg{timeIndex} = nearBoundary_deg(:, 2);
    farAzimuth_deg{timeIndex} = farBoundary_deg(:, 1);
    farElevation_deg{timeIndex} = farBoundary_deg(:, 2);
end
nearObstacle = makeAzElObstacleData( ...
    "route-shaping mover", obstacleTime_s, ...
    nearAzimuth_deg, nearElevation_deg, 0.2);
farObstacle = makeAzElObstacleData( ...
    "off-route similar mover", obstacleTime_s, ...
    farAzimuth_deg, farElevation_deg, 0.2);
initialState = restState(0, [0 0]);
targetTime_s = (0:2:20).';
target = struct("time_s", targetTime_s, ...
    "position_deg", [6 + 0.2 * targetTime_s, 1 + 0.02 * targetTime_s], ...
    "InterpolationMethod", "linear");
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", [2 2]);
options = struct( ...
    "InterceptMode", "specifiedTime", "SpecifiedInterceptTime_s", 10, ...
    "MatchTargetVelocity", false, "MatchTargetAcceleration", false, ...
    "PlannerOptions", struct( ...
    "PlannerMethod", "corridorQuintic", "MaximumSeedCount", 5));
end

function [obstacles, initialState, goalState, limits] = staticArcRequest(windowEnd_s)
% Build a thick many-vertex static arc with one narrow opening. The dense
% boundary makes the visibility route stop at many vertices, so every
% route-length duration bracket lands above a tight requested arrival while a
% smooth compact traversal of the same topology stays inside it.
openingHalfAngle_rad = deg2rad(16);
arcAngle_rad = linspace(openingHalfAngle_rad, ...
    2 * pi - openingHalfAngle_rad, 60).';
innerRadius_deg = 7.5;
outerRadius_deg = 10.5;
boundary_deg = [ ...
    innerRadius_deg * [cos(arcAngle_rad), sin(arcAngle_rad)]; ...
    outerRadius_deg * [cos(flipud(arcAngle_rad)), sin(flipud(arcAngle_rad))]];
obstacles = makeAzElObstacleData("thick static arc", 0, ...
    boundary_deg(:, 1), boundary_deg(:, 2), 0.2);
initialState = restState(0, [0 0]);
goalState = restState(windowEnd_s, [-14 0]);
limits = motionLimits([-16 16], [-16 16]);
end

function options = plannerOptions(maximumSeedCount, randomSeed)
% Return the small deterministic option set shared by these focused timing tests.
if nargin < 2
    randomSeed = 325;
end
options = struct( ...
    "MotionMethod", "corridorQuintic", ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", maximumSeedCount, "SampleTime_s", 0.05, "RandomSeed", randomSeed, "Verbose", false);
end

function state = restState(time_s, position_deg)
% Construct an endpoint state at rest at the requested time and angular position.
state = struct( "time_s", time_s, "position_deg", position_deg, "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function limits = motionLimits(azimuthInterval_deg, elevationInterval_deg)
% Construct the common kinematic limits while allowing each fixture to choose its workspace.
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.8 0.8], ...
    "maxJerk_deg_s3", [2.5 2.5], ...
    "azimuthInterval_deg", azimuthInterval_deg, "elevationInterval_deg", elevationInterval_deg);
end

function verifyExperimentalSuccess(testCase, result)
% Check both the public success contract and the expected corridor-quintic provenance.
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.SelectedMotionSource, "corridorQuintic");
verifyEqual(testCase, result.SearchDiagnostics.PlannerMethod, "corridorQuintic");
end
