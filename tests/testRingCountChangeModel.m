function tests = testRingCountChangeModel
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testRingCountChangeModel.m')
%**************************************************************************
% PURPOSE
%   - Regress the conservative interval model for changing obstacle rings.
%**************************************************************************
% INPUTS
%   - MATLAB unit test framework.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.Test array)
%       Function-based tests for preparation, planning, and validation.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'), fullfile(root, 'tests'));
end

function testAppearingObstacleBlocksStraightMotion(testCase)
    square_units = [-1, -1; 1, -1; 1, 1; -1, 1];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'appearing square', [0; 10], {zeros(0, 1); square_units(:, 1)}, ...
        {zeros(0, 1); square_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;

    verifyEndpointHullModel(testCase, preparation);
    verifyTrue(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        prepared, 0, 0, 5));

    [initialState, goalState, limits, options] = planningRequest( ...
        [-3, 0], [3, 0], 0, 10, [-5, 5], [-5, 5]);
    result     = planner(obstacle, initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(result);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyGreaterThan(testCase, max(abs(result.position_units(:, 2))), 1);
end

function testRingSplitOccupiesFragmentsAndGap(testCase)
    leftLower_units  = [-2, -1; -1, -1; -1, 1; -2, 1];
    leftUpper_units  = leftLower_units + [0.1, 0];
    rightUpper_units = [1, -1; 2, -1; 2, 1; 1, 1];
    upper_units = [leftUpper_units; NaN, NaN; rightUpper_units];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'separated fragments', [0; 10], ...
        {leftLower_units(:, 1); upper_units(:, 1)}, ...
        {leftLower_units(:, 2); upper_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;

    verifyEndpointHullModel(testCase, preparation);
    verifyEqual(testCase, numel(preparation.IntervalStartRegions_units{1}), 1);
    verifyEqual(testCase, obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        prepared, [-1.5, 1.5, 0], [0, 0, 0], 5), [true, true, true]);

    [initialState, goalState, limits, options] = planningRequest( ...
        [0, -3], [0, 3], 0, 10, [-4, 4], [-4, 4]);
    result     = planner(obstacle, initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(result);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    % The hull spans x=[-2,2], so a valid crossing must detour past an end.
    verifyGreaterThan(testCase, max(abs(result.position_units(:, 1))), 2);
    addedArea_units2 = preparation.IntervalEndpointHullAddedArea_units2(1);
    verifyGreaterThan(testCase, addedArea_units2, 0);
    endpointUnion = union(preparation.SampleShapes{1}, preparation.SampleShapes{2});
    verifyEqual(testCase, addedArea_units2, ...
        area(subtract(preparation.IntervalUnionShapes{1}, endpointUnion)));
end

function testTwoDeformingRingsAndAddedArea(testCase)
    lowerLeft_units  = [-4, -1; -2, -1; -2, 1; -4, 1];
    lowerRight_units = [2, -1; 4, -1; 4, 1; 2, 1];
    upperLeft_units  = [-4.2, -0.7; -2.1, -1.3; -1.8, 0.8; -3.8, 1.2];
    upperRight_units = [2.1, -1.2; 4.3, -0.8; 3.8, 1.3; 1.9, 0.7];
    lower_units = [lowerLeft_units; NaN, NaN; lowerRight_units];
    upper_units = [upperLeft_units; NaN, NaN; upperRight_units];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'two deforming rings', [0; 4], ...
        {lower_units(:, 1); upper_units(:, 1)}, ...
        {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;

    verifyEndpointHullModel(testCase, preparation);
    regions_units = preparation.IntervalStartRegions_units{1};
    verifyEqual(testCase, numel(regions_units), 1);
    verifyTrue(testCase, groupIsCovered(regions_units, [lowerLeft_units; upperLeft_units]));
    verifyTrue(testCase, groupIsCovered(regions_units, [lowerRight_units; upperRight_units]));

    addedArea_units2 = preparation.IntervalEndpointHullAddedArea_units2(1);
    verifyTrue(testCase, isfinite(addedArea_units2));
    verifyGreaterThanOrEqual(testCase, addedArea_units2, 0);
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared, 0, 4);
    verifyEqual(testCase, cells.Regions_units, cells.EndRegions_units);
end

function testLargeNonoverlappingJumpCoversCrossSampleSegments(testCase)
    square_units = [-1, -1; 1, -1; 1, 1; -1, 1];
    lower_units  = [square_units + [-10, 0]; NaN, NaN; square_units + [-5, 0]];
    upper_units  = [square_units + [5, 0]; NaN, NaN; square_units + [12, 0]];
    verifyCrossSampleSegments(testCase, lower_units, upper_units);
end

function testSwappedRingIdentitiesCoverCrossSampleSegments(testCase)
    square_units = [-1, -1; 1, -1; 1, 1; -1, 1];
    lower_units  = [square_units + [-5, 0]; NaN, NaN; square_units + [5, 0]];
    % Supplied ring identities swap sides. The offset keeps the samples
    % geometrically non-equivalent so this exercises the enclosure model.
    upper_units  = [square_units + [5, 0.5]; NaN, NaN; square_units + [-5, 0.5]];
    verifyCrossSampleSegments(testCase, lower_units, upper_units);
end

function testDegenerateEndpointRemainsUnsupported(testCase)
    lower_units = [0, 0; 1, 0; 2, 0];
    upper_units = [0, 0; 2, 0; 2, 2; 0, 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'degenerate endpoint', [0; 1], ...
        {lower_units(:, 1); upper_units(:, 1)}, ...
        {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    verifyTrue(testCase, prepared.InternalPreparation.IntervalIsUnsupported);
    verifyFalse(testCase, prepared.InternalPreparation.IntervalUsesEndpointHull);
    verifyEqual(testCase, prepared.InternalPreparation.IntervalProofReason, ...
        "degenerateEndpointGeometry");
end

function verifyCrossSampleSegments(testCase, lower_units, upper_units)
    % Every lower-to-upper pair is admissible without declared ring identity.
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'unknown ring identity', [0; 4], ...
        {lower_units(:, 1); upper_units(:, 1)}, ...
        {lower_units(:, 2); upper_units(:, 2)}, 0);
    prepared    = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    preparation = prepared.InternalPreparation;
    verifyEndpointHullModel(testCase, preparation);
    verifyEqual(testCase, numel(preparation.IntervalStartRegions_units{1}), 1);
    lowerFinite_units = lower_units(all(isfinite(lower_units), 2), :);
    upperFinite_units = upper_units(all(isfinite(upper_units), 2), :);
    for lowerIndex = 1:size(lowerFinite_units, 1)
        midpoint_units = (lowerFinite_units(lowerIndex, :) + upperFinite_units) / 2;
        verifyTrue(testCase, all(obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
            prepared, midpoint_units(:, 1), midpoint_units(:, 2), 2)));
        verifyTrue(testCase, groupIsCovered( ...
            preparation.IntervalStartRegions_units{1}, midpoint_units));
    end
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared, 0, 4);
    verifyEqual(testCase, cells.Regions_units, preparation.IntervalStartRegions_units{1});
    verifyEqual(testCase, cells.Regions_units, cells.EndRegions_units);
end

function testStandInRingChangeWindowPlans(testCase)
    root        = fileparts(fileparts(mfilename('fullpath')));
    historyPath = fullfile(root, 'tmp', 'azel', 'azel_history.mat');
    assumeTrue(testCase, isfile(historyPath), ...
        'The optional tmp/azel/azel_history.mat stand-in is unavailable.');

    loaded       = load(historyPath, 'obstacles');
    initialState = struct('time_s', 300, 'position_units', [10, 0]);
    goalState    = struct('time_s', 530, 'position_units', [110, 10]);
    limits = struct( ...
        'xInterval_units',          [-180, 180], ...
        'yInterval_units',          [-90, 90], ...
        'maxVelocity_units_s',      [2, 2], ...
        'maxAcceleration_units_s2', [0.5, 0.5], ...
        'maxJerk_units_s3',         [1, 1]);
    options = struct('GoalTimeMode', 'fixedArrival');

    wallTimer  = tic;
    result     = planner(loaded.obstacles, initialState, goalState, limits, options);
    wallTime_s = toc(wallTimer);
    validation = obstacleAvoidance.validateTrajectory(result);
    fprintf(['[ring-count regression] t=[300,530] wall %.2f s, success %d, ', ...
        '%s, valid %d; %s\n'], wallTime_s, result.Success, ...
        result.TerminationReason, validation.Passed, result.Message);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
end

function verifyEndpointHullModel(testCase, preparation)
    % Verify the typed interval classification without using its report name.
    verifyTrue(testCase, preparation.IntervalUsesEndpointHull(1));
    verifyFalse(testCase, preparation.MatchingTopology(1));
    verifyFalse(testCase, preparation.IntervalHasExactPartition(1));
    verifyFalse(testCase, preparation.IntervalUsesMovingCells(1));
    verifyFalse(testCase, preparation.IntervalIsStationary(1));
    verifyFalse(testCase, preparation.IntervalIsUnsupported(1));
    verifyEqual(testCase, preparation.IntervalGeometryModel(1), "endpointConvexHull");
    verifyEqual(testCase, preparation.IntervalProofReason(1), "");
end

function covered = groupIsCovered(regions_units, vertices_units)
    % All queried vertices must fit wholly inside the returned convex hull.
    covered = false;
    for regionIndex = 1:numel(regions_units)
        region_units = regions_units{regionIndex};
        [inside, onBoundary] = inpolygon(vertices_units(:, 1), vertices_units(:, 2), ...
            region_units(:, 1), region_units(:, 2));
        if all(inside | onBoundary)
            covered = true;
            return;
        end
    end
end

function [initialState, goalState, limits, options] = planningRequest( ...
        start_units, goal_units, initialTime_s, goalTime_s, xInterval_units, yInterval_units)
    % Build one deterministic fixed-arrival request for focused regressions.
    initialState = struct('time_s', initialTime_s, 'position_units', start_units);
    goalState    = struct('time_s', goalTime_s, 'position_units', goal_units);
    limits = struct( ...
        'xInterval_units',          xInterval_units, ...
        'yInterval_units',          yInterval_units, ...
        'maxVelocity_units_s',      [2, 2], ...
        'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3',         [4, 4]);
    options = struct('GoalTimeMode', 'fixedArrival');
end
