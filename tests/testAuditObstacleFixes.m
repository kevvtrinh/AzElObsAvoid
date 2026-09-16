function tests = testAuditObstacleFixes
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testAuditObstacleFixes.m')
% PURPOSE: Regress the obstacle-side defects found by the read-only audit:
%          span merging that moves authoritative samples, sampled overlap
%          checks promoting a colliding iterate, and gallery plots of
%          partially prepared failure results.
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

function testSubEpsilonVelocityDriftIsNotMergedAway(testCase)
    % Two intervals with velocities 0 and 1e-14 units/s differ by less than
    % the velocity epsilon, yet the middle sample is displaced by 0.005 from
    % the merged span. The authoritative sample must win.
    square_units = [0 0; 1 0; 1 1; 0 1];
    time_s       = [0; 1e12; 2e12];
    xByTime      = {square_units(:, 1); square_units(:, 1); square_units(:, 1) + 0.01};
    yByTime      = repmat({square_units(:, 2)}, 3, 1);
    obstacle = obstacleAvoidance.obstacles.createObstacle('drifting square', time_s, xByTime, yByTime, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle, [0 2e12]);
    verifyEqual(testCase, prepared.InternalPreparation.SpanEndSampleIndex, [2; 3]);
    verifyEqual(testCase, prepared.InternalPreparation.MergedIntervalCount, 0);

    cells  = obstacleAvoidance.obstacles.createTimeCells(prepared, 1e12, 2e12);
    region = cells.Regions_units{1};
    verifyTrue(testCase, inpolygon(0.002, 0.5, region(:, 1), region(:, 2)));

    affineX  = {square_units(:, 1); square_units(:, 1) + 0.005; square_units(:, 1) + 0.01};
    affine   = obstacleAvoidance.obstacles.createObstacle('affine square', time_s, affineX, yByTime, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(affine, [0 2e12]);
    verifyEqual(testCase, prepared.InternalPreparation.SpanEndSampleIndex, [3; 3]);
    verifyEqual(testCase, prepared.InternalPreparation.MergedIntervalCount, 1);
end

function testThinWallCrossingIsNotPromotedBySampling(testCase)
    % A wall 1e-5 wide lies between the fixed samples of the straight first
    % iterate, so the sampled overlap check reports it clear. Continuous
    % certification must decide the incumbent and separate the pair.
    straightControls_units       = zeros(1, 9, 2);
    straightControls_units(1, :, 1) = (0:8) / 8;
    wall_units = [0.0004 -0.1; 0.0005 -0.1; 0.0005 0.1; 0.0004 0.1];
    sampledPairs = bmtpEngine.findSampledObstacleOverlaps(straightControls_units, {wall_units}, ...
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
    verifyEqual(testCase, result.VisibilityGraph.SearchKind, "initialSpatialSnapshot");
    verifyGreaterThan(testCase, result.SolverDiagnostics.TaggedPairCount, 0);
end

function testGalleryPlotsPartiallyPreparedFailureResult(testCase)
    collinear_units = [0 0; 0.5 0; 1 0];
    smallSquare     = [0 0; 1 0; 1 1; 0 1];
    largeSquare     = [0 0; 2 0; 2 2; 0 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('deforming', [0; 1; 2], ...
        {collinear_units(:, 1); smallSquare(:, 1); largeSquare(:, 1)}, ...
        {collinear_units(:, 2); smallSquare(:, 2); largeSquare(:, 2)}, 0);
    initial = struct('time_s', 0, 'position_units', [-4 0]);
    goal    = struct('time_s', 2, 'position_units', [4 0]);
    result  = planner(obstacle, initial, goal, testCase.TestData.Limits, struct('GoalTimeMode', 'fixedArrival'));
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "unsupportedObstacleInterpolation");
    verifyFalse(testCase, all(result.PreparedObstacles.InternalPreparation.SamplePrepared));

    figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery({result}, "unsupported", 'off');
    cleanup = onCleanup(@() close(figureHandles(isvalid(figureHandles))));
    verifyEqual(testCase, numel(figureHandles), 1);
    axesHandle = findobj(figureHandles(1), 'Type', 'axes');
    verifyTrue(testCase, contains(string(axesHandle(1).Title.String), "unprepared snapshot"));
end
