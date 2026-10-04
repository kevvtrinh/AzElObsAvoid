function tests = testAuditObstacleFixes
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testAuditObstacleFixes.m')
% PURPOSE: Prevent sampled overlap checks from promoting a colliding iterate.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based test results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    testCase.TestData.Limits = struct( ...
        'xInterval_units',          [-10 10], ...
        'yInterval_units',          [-10 10], ...
        'maxVelocity_units_s',      [2 2], ...
        'maxAcceleration_units_s2', [2 2], ...
        'maxJerk_units_s3',         [4 4]);
end

function testThinWallCrossingIsNotPromotedBySampling(testCase)
    % A wall 1e-5 wide lies between the fixed samples of the straight first
    % iterate, so the sampled overlap check reports it clear. Continuous
    % proof must decide the best plan so far and separate the pair.
    straightControls_units       = zeros(1, 9, 2);
    straightControls_units(1, :, 1) = (0:8) / 8;
    wall_units = [0.0004 -0.1; 0.0005 -0.1; 0.0005 0.1; 0.0004 0.1];
    sampledPairs = bmtpEngine.separation.findSampledObstacleOverlaps(straightControls_units, {wall_units}, ...
        min(wall_units, [], 1), max(wall_units, [], 1), true);
    verifyFalse(testCase, any(sampledPairs, 'all'));

    % The visibility route seeds the solver around the wall, but its first
    % unconstrained iterate is the straight chord through the wall.
    offset_units = 0.005;
    width_units  = 1e-5;
    wall_units   = [offset_units -0.5; offset_units + width_units -0.5; ...
        offset_units + width_units 0.5; offset_units 0.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle('thin wall', 0, wall_units(:, 1), wall_units(:, 2), 0);
    initial  = struct('time_s', 0, 'position_units', [-4 0]);
    goal     = struct('time_s', 30, 'position_units', [4 0]);
    result   = planner(obstacle, initial, goal, testCase.TestData.Limits, struct('GoalTimeMode', 'earliestArrival'));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, "initialSpatialSnapshot");
    verifyGreaterThan(testCase, result.Diagnostics.SolverDiagnostics.TaggedPairCount, 0);
end
