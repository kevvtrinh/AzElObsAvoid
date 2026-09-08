function tests = testRuckigWaypointMotion
%% Section 0: Header & Readme
% SYNTAX
%   tests = testRuckigWaypointMotion
%**************************************************************************
% PURPOSE
%   - Verify exact Ruckig composition for at most two route segments.
%   - Verify that longer routes fail explicitly before the engine runs.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
    % Add the product and independent trajectory-engine entry points.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
end

function testTwoSegmentDetourSupportsEarliestAndFixedArrival(testCase)
    % Exercise the maximum supported route size around a static box.
    obstacle     = obstacleAvoidance.obstacles.createObstacle("static box", 0, [-1 1 1 -1], [-1 -1 1 1], 0.2);
    obstacles    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    initialState = struct();
    initialState.time_s              = 0;
    initialState.position_units        = [-4 0];
    initialState.velocity_units_s      = [0 0];
    initialState.acceleration_units_s2 = [0 0];
    goalState = struct();
    goalState.time_s              = 30;
    goalState.position_units        = [4 0];
    goalState.velocity_units_s      = [0 0];
    goalState.acceleration_units_s2 = [0 0];
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3         = [2 2];
    limits.xInterval_units    = [-10 10];
    limits.yInterval_units  = [-10 10];
    options = planner();
    seed    = struct();
    seed.Index        = 1;
    seed.Source       = "visibilityGraph";
    seed.position_units = [-4 0; 0 -2.5; 4 0];

    [candidate, diagnostics] = obstacleAvoidance.planner.createRuckigWaypointMotion(seed, initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(candidate, obstacles, initialState, goalState, limits, options);

    verifyTrue(testCase, candidate.OptimizerFeasible);
    verifyTrue(testCase, diagnostics.Accepted);
    verifyEqual(testCase, diagnostics.CompletedPartCount, 2);
    verifyEqual(testCase, diagnostics.MaximumSupportedPartCount, 2);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, validation.CollisionFree);
    verifyGreaterThan(testCase, validation.MinimumClearance_units, options.CollisionClearanceTolerance_units);
    verifyEqual(testCase, candidate.position_units(1, :), initialState.position_units, "AbsTol", 1e-10);
    verifyEqual(testCase, candidate.position_units(end, :), goalState.position_units, "AbsTol", 1e-10);

    options.GoalTimeMode = "fixedArrival";
    [fixedCandidate, fixedDiagnostics] = obstacleAvoidance.planner.createRuckigWaypointMotion(seed, initialState, goalState, limits, options);
    fixedValidation = obstacleAvoidance.validateTrajectory(fixedCandidate, obstacles, initialState, goalState, limits, options);
    verifyTrue(testCase, fixedCandidate.OptimizerFeasible);
    verifyTrue(testCase, fixedDiagnostics.Accepted);
    verifyTrue(testCase, fixedValidation.Passed, fixedValidation.Message);
    verifyTrue(testCase, fixedValidation.CollisionFree);
    verifyEqual(testCase, fixedCandidate.ArrivalTime_s, goalState.time_s, "AbsTol", 1e-10);
end

function testThreeSegmentRouteReturnsExplicitUnsupportedResult(testCase)
    % Reject a longer route without constructing any partial Ruckig motion.
    initialState = struct();
    initialState.time_s              = 0;
    initialState.position_units        = [-4 0];
    initialState.velocity_units_s      = [0 0];
    initialState.acceleration_units_s2 = [0 0];
    goalState = struct();
    goalState.time_s              = 30;
    goalState.position_units        = [4 0];
    goalState.velocity_units_s      = [0 0];
    goalState.acceleration_units_s2 = [0 0];
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3         = [2 2];
    limits.xInterval_units    = [-10 10];
    limits.yInterval_units  = [-10 10];
    options = planner();
    seed    = struct();
    seed.Index        = 1;
    seed.Source       = "visibilityGraph";
    seed.position_units = [-4 0; -2 -1.5; 2 -1.5; 4 0];

    [candidate, diagnostics] = obstacleAvoidance.planner.createRuckigWaypointMotion(seed, initialState, goalState, limits, options);

    verifyFalse(testCase, candidate.Success);
    verifyFalse(testCase, candidate.OptimizerFeasible);
    verifyEqual(testCase, candidate.TerminationReason, "ruckigWaypointSegmentLimitExceeded");
    verifyThat(testCase, candidate.Message, matlab.unittest.constraints.ContainsSubstring("at most 2 route segments"));
    verifyFalse(testCase, diagnostics.Accepted);
    verifyEqual(testCase, diagnostics.RequestedPartCount, 3);
    verifyEqual(testCase, diagnostics.MaximumSupportedPartCount, 2);
    verifyEqual(testCase, diagnostics.CompletedPartCount, 0);
    verifyEqual(testCase, diagnostics.EngineTerminationReason, "ruckigWaypointSegmentLimitExceeded");
end
