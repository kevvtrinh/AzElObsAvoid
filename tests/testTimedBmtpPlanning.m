function tests = testTimedBmtpPlanning
%% Section 0: Header & Readme
% SYNTAX
%   tests = testTimedBmtpPlanning
%**************************************************************************
% PURPOSE
%   - Verify smooth timed multi-waypoint planning around a moving circle and
%     a static concave obstacle without forced interior rest states.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-test array)
%       Deterministic public-planner dynamic-topology regression cases.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Run the motivating general input family once for all assertions.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
[obstacles, initialState, goalState, limits, options] = createScenario();
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
testCase.TestData.Result = result;
testCase.TestData.Validation = obstacleAvoidance.validateTrajectory(result);
end

function testMovingCircleAndStaticUSucceeds(testCase)
% Require full dynamic collision and kinematic validation.
result = testCase.TestData.Result;
validation = testCase.TestData.Validation;
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, validation.Passed, validation.Message);
verifyTrue(testCase, validation.CollisionFree);
verifyTrue(testCase, validation.VelocityWithinLimits);
verifyTrue(testCase, validation.AccelerationWithinLimits);
verifyTrue(testCase, validation.JerkWithinLimits);
verifyEqual(testCase, ...
    result.SearchDiagnostics.StaticProjectionCreationCount, 1);
verifyEqual(testCase, ...
    result.SearchDiagnostics.StaticRepresentationCreationCount, 1);
verifyEqual(testCase, result.Seeds(result.SelectedSeedIndex).Source, ...
    "timeExpandedVisibilityGraph");
end

function testInteriorWaypointsAreNotForcedToRest(testCase)
% Check the smooth polynomial state at every interior timed-seed knot.
result = testCase.TestData.Result;
seed = result.Seeds(result.SelectedSeedIndex);
interiorTime_s = result.time_s(1) + seed.tau(2:end - 1) * ...
    (result.time_s(end) - result.time_s(1));
[~, ~, velocity_deg_s] = bmtpEngine.evaluatePolynomial( ...
    result.Polynomial, interiorTime_s);
verifyGreaterThan(testCase, min(vecnorm(velocity_deg_s, 2, 2)), 1e-3);
acceptedIndex = find([result.SeedSummaries.ValidationPassed], 1, "first");
diagnostics = result.SeedSummaries(acceptedIndex).SolverDiagnostics;
verifyEqual(testCase, diagnostics.Identifier, "bmtpTimedCell");
verifyEqual(testCase, diagnostics.TimedBmtp.Outcome, ...
    "acceptedAfterFullValidation");
end

function testTimedCellsRespectSearchLayerBudget(testCase)
% Keep the continuous optimizer no finer than the search clock it receives.
result = testCase.TestData.Result;
acceptedIndex = find([result.SeedSummaries.ValidationPassed], 1, "first");
diagnostics = result.SeedSummaries(acceptedIndex).SolverDiagnostics;
maximumTimedSegmentCount = result.Options.MaximumTimeLayerCount - 1;
verifyLessThanOrEqual(testCase, ...
    diagnostics.Coverage.TimedSegmentCount, maximumTimedSegmentCount);
end

function [obstacles, initialState, goalState, limits, options] = createScenario()
% Create input-driven static-concave and translating-convex geometry.
missionEndTime_s = 40;
obstacleTime_s = [0; missionEndTime_s];
uPosition_deg = [ ...
    -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
staticObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "static concave polygon", obstacleTime_s, uPosition_deg(:, 1), ...
    uPosition_deg(:, 2), 0.20);
angle_rad = linspace(0, 2 * pi, 33).';
angle_rad(end) = [];
startCircle_deg = [-10 + 2 * cos(angle_rad), -7 + 2 * sin(angle_rad)];
finishCircle_deg = [10 + 2 * cos(angle_rad), -7 + 2 * sin(angle_rad)];
movingObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "translating convex polygon", [0; 10; missionEndTime_s], ...
    {startCircle_deg(:, 1); startCircle_deg(:, 1); finishCircle_deg(:, 1)}, ...
    {startCircle_deg(:, 2); startCircle_deg(:, 2); finishCircle_deg(:, 2)}, ...
    0.10);
obstacles = obstacleAvoidance.obstacles.combineObstacles( ...
    staticObstacle, movingObstacle);
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct("time_s", missionEndTime_s, "position_deg", [0 -10]);
limits = struct( ...
    "azimuthInterval_deg", [-14 14], ...
    "elevationInterval_deg", [-12 10], ...
    "maxVelocity_deg_s", [3 3], ...
    "maxAcceleration_deg_s2", [1.5 1.5], ...
    "maxJerk_deg_s3", [3 3]);
options = struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "MaximumSeedCount", 2, ...
    "MaximumTimeLayerCount", 17, ...
    "SampleTime_s", 0.05, ...
    "UnsupportedTimedTopologyPolicy", "fail");
end
