function tests = testRuckigWaypointMotion
%% Section 0: Header & Readme
% SYNTAX
%   tests = testRuckigWaypointMotion
%**************************************************************************
% PURPOSE
%   - Verify exact Ruckig route composition on a static obstacle detour.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the product and independent trajectory-engine entry points.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
end

function testStaticDetourSupportsEarliestAndFixedArrival(testCase)
% Exercise a structurally different four-vertex route around a static box.
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "static box", 0, [-1 1 1 -1], [-1 -1 1 1], 0.2);
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacle);
initialState = struct( ...
    "time_s", 0, "position_deg", [-4 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 30, "position_deg", [4 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "azimuthInterval_deg", [-10 10], ...
    "elevationInterval_deg", [-10 10]);
options = obstacleAvoidance.planTrajectory();
options.TrajectoryMethod = "ruckigWaypoint";
seed = struct( ...
    "Index", 1, "Source", "visibilityGraph", ...
    "position_deg", [-4 0; -1.5 -1.5; 1.5 -1.5; 4 0]);

[candidate, diagnostics] = ...
    obstacleAvoidance.planner.createRuckigWaypointMotion( ...
    seed, initialState, goalState, limits, options);
validation = obstacleAvoidance.validateTrajectory( ...
    candidate, obstacles, initialState, goalState, limits, options);

verifyTrue(testCase, candidate.OptimizerFeasible);
verifyTrue(testCase, diagnostics.Accepted);
verifyEqual(testCase, diagnostics.CompletedPartCount, 3);
verifyTrue(testCase, validation.Passed, validation.Message);
verifyTrue(testCase, validation.CollisionFree);
verifyGreaterThan(testCase, validation.MinimumClearance_deg, ...
    options.CollisionClearanceTolerance_deg);
verifyEqual(testCase, candidate.position_deg(1, :), ...
    initialState.position_deg, "AbsTol", 1e-10);
verifyEqual(testCase, candidate.position_deg(end, :), ...
    goalState.position_deg, "AbsTol", 1e-10);

options.GoalTimeMode = "fixedArrival";
[fixedCandidate, fixedDiagnostics] = ...
    obstacleAvoidance.planner.createRuckigWaypointMotion( ...
    seed, initialState, goalState, limits, options);
fixedValidation = obstacleAvoidance.validateTrajectory( ...
    fixedCandidate, obstacles, initialState, goalState, limits, options);
verifyTrue(testCase, fixedCandidate.OptimizerFeasible);
verifyTrue(testCase, fixedDiagnostics.Accepted);
verifyTrue(testCase, fixedValidation.Passed, fixedValidation.Message);
verifyTrue(testCase, fixedValidation.CollisionFree);
verifyEqual(testCase, fixedCandidate.FinalTime_s, goalState.time_s, ...
    "AbsTol", 1e-10);
end
