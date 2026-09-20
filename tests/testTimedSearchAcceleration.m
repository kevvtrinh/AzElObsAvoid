function tests = testTimedSearchAcceleration
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testTimedSearchAcceleration.m')
%**************************************************************************
% PURPOSE
%   - Verify conservative space-time filtering and proven retry skips.
%   - Preserve exact contact, direction, timing, and search diagnostics.
%**************************************************************************
% INPUTS
%   - MATLAB function-based unit test framework.
%**************************************************************************
% OUTPUTS
%   - Broad-phase and collision-proof regression results.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
end

function testRemoteCellsLeaveRouteAndCountsUnchanged(testCase)
    nearObstacle = createDeformingBar("near bar", 4, 6, 2, 0, 10);
    farObstacle  = createDeformingBar("far bar", 400, 402, 2, 0, 10);
    [route_units, routeTime_s, record] = runSearch(nearObstacle, [0, 0], [10, 0], (0:10).');
    [bothRoute_units, bothRouteTime_s, bothRecord] = runSearch( ...
        [nearObstacle, farObstacle], [0, 0], [10, 0], (0:10).');

    verifyEqual(testCase, bothRoute_units, route_units);
    verifyEqual(testCase, bothRouteTime_s, routeTime_s);
    verifyEqual(testCase, bothRecord.RejectedTransitionCount, record.RejectedTransitionCount);
    verifyEqual(testCase, bothRecord.ExpandedCount, record.ExpandedCount);
    verifyNotEmpty(testCase, routeTime_s);
end

function testTemporalSegmentBoxMismatchStaysClear(testCase)
    earlyObstacle = createDeformingBar("early bar", 4, 6, -0.5, 0, 1);
    alignedObstacle = createDeformingBar("aligned bar", 4, 6, -0.5, 1, 2);
    sampleTimes_s = (0:10).';

    [clearRoute_units, clearRouteTime_s] = runSearch( ...
        earlyObstacle, [0, 0], [10, 0], sampleTimes_s);
    [alignedRoute_units, alignedRouteTime_s] = runSearch( ...
        alignedObstacle, [0, 0], [10, 0], sampleTimes_s);

    verifyEqual(testCase, clearRoute_units(end, :), [10, 0], 'AbsTol', 1e-12);
    verifyNotEmpty(testCase, clearRouteTime_s);
    verifyEqual(testCase, alignedRoute_units(end, :), [10, 0], 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, alignedRouteTime_s(end), clearRouteTime_s(end));
end

function testExactMovingContactStaysBlockedInBothDirections(testCase)
    obstacle = createDeformingBar("contact bar", 4, 6, 0, 0, 10);
    sampleTimes_s = (0:0.25:10).';
    [forwardRoute_units, forwardRouteTime_s, forwardRecord] = runSearch( ...
        obstacle, [0, 0], [10, 0], sampleTimes_s);
    [reverseRoute_units, reverseRouteTime_s, reverseRecord] = runSearch( ...
        obstacle, [10, 0], [0, 0], sampleTimes_s);

    verifyEmpty(testCase, forwardRoute_units);
    verifyEmpty(testCase, forwardRouteTime_s);
    verifyEmpty(testCase, reverseRoute_units);
    verifyEmpty(testCase, reverseRouteTime_s);
    verifyEqual(testCase, forwardRecord.ProvenSkippedTransitionCount, 0);
    verifyEqual(testCase, reverseRecord.ProvenSkippedTransitionCount, 0);
end

function testStrictInteriorCollisionSkipsOnlyBlockedArrivals(testCase)
    obstacle = createDeformingBar("blocking bar", 4, 6, -0.5, 0, 10);
    sampleTimes_s = (0:0.1:10).';
    [route_units, routeTime_s, record] = runSearch( ...
        obstacle, [0, 0], [10, 0], sampleTimes_s);

    verifyEmpty(testCase, route_units);
    verifyEmpty(testCase, routeTime_s);
    verifyGreaterThan(testCase, record.ProvenSkippedTransitionCount, 0);
    verifyGreaterThanOrEqual(testCase, record.RejectedTransitionCount, ...
        record.ProvenSkippedTransitionCount);
end

function testProofStopsAtTheFirstClearArrival(testCase)
    obstacle = createDeformingBar("departing bar", 4, 6, -0.5, 0, 2);
    sampleTimes_s = (0:0.1:10).';
    [route_units, routeTime_s, record] = runSearch( ...
        obstacle, [0, 0], [10, 0], sampleTimes_s);

    verifyEqual(testCase, route_units(end, :), [10, 0], 'AbsTol', 1e-12);
    verifyEqual(testCase, routeTime_s(end), 3.6, 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, record.ProvenSkippedTransitionCount, 0);
end

function testSharpCornerToleranceOvershootStaysBlocked(testCase)
    sliver = createSliver(1e-7);
    [route_units, routeTime_s] = runSearch( ...
        sliver, [-1e-4, 0], [10, 5], (0:10).');

    verifyEmpty(testCase, route_units);
    verifyEmpty(testCase, routeTime_s);
end

function testPointOutsideSharpCornerOvershootStaysVisible(testCase)
    sliver = createSliver(1e-7);
    [route_units, routeTime_s] = runSearch( ...
        sliver, [-1, 0], [10, 5], (0:10).');

    verifyEqual(testCase, route_units(end, :), [10, 5], 'AbsTol', 1e-12);
    verifyNotEmpty(testCase, routeTime_s);
end

function testSubToleranceMotionStartsAtTheNextPhysicalLayer(testCase)
    x_units = [0.4; 0.6; 0.6; 0.4];
    y_units = [-1; -1; 1; 1];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        "early blocking box", [0; 0.1], {x_units; x_units}, {y_units; y_units}, 0);
    nodes_units    = [0, 0; 1, 0];
    edgeCost_units = [0, 1; 1, 0];
    initialState = struct( ...
        'time_s',                0, ...
        'position_units',        [0, 0], ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    goalState = struct( ...
        'time_s',                1, ...
        'position_units',        [1, 0], ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    limits  = struct('maxVelocity_units_s', [1e13, 1e13]);
    options = struct('GoalTimeMode', "earliestArrival");

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, obstacle, initialState, goalState, ...
        limits, [0; 1], options);

    verifyEqual(testCase, route_units, nodes_units, 'AbsTol', 1e-12);
    verifyEqual(testCase, routeTime_s, [0; 1], 'AbsTol', 1e-12);
end

function testBoundedPairCachePreservesExactSearch(testCase)
    obstacles = [ ...
        createDeformingBar("first remote bar", 400, 402, 2, 0, 10), ...
        createDeformingBar("second remote bar", 500, 502, 2, 0, 10)];
    sampleTimes_s = [0; 10];
    [expectedRoute_units, expectedRouteTime_s] = runSearch( ...
        obstacles, [0, 0], [10, 0], sampleTimes_s);

    % With at least two moving cells, 1,450 nodes exceed the eager cache
    % work bound. All extra nodes are disconnected, so only storage strategy
    % changes and the same exact start-to-goal edge remains authoritative.
    nodeCount   = 1450;
    nodes_units = [0, 0; 10, 0; ...
        (1001:1000 + nodeCount - 2).', 50 * ones(nodeCount - 2, 1)];
    edgeCost_units       = Inf(nodeCount);
    edgeCost_units(1, 2) = 10;
    initialState = struct( ...
        'time_s',                0, ...
        'position_units',        [0, 0], ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    goalState = struct( ...
        'time_s',                10, ...
        'position_units',        [10, 0], ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    limits  = struct('maxVelocity_units_s', [4, 4]);
    options = struct('GoalTimeMode', "earliestArrival");
    [route_units, routeTime_s] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, obstacles, initialState, goalState, ...
        limits, sampleTimes_s, options);

    verifyEqual(testCase, route_units, expectedRoute_units);
    verifyEqual(testCase, routeTime_s, expectedRouteTime_s);
end

function [route_units, routeTime_s, record] = runSearch( ...
        obstacles, start_units, goal_units, sampleTimes_s)
    nodes_units = [start_units; goal_units];
    edgeLength_units = norm(goal_units - start_units);
    edgeCost_units = [0, edgeLength_units; edgeLength_units, 0];
    initialState = struct( ...
        'time_s',               sampleTimes_s(1), ...
        'position_units',       start_units, ...
        'velocity_units_s',     [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    goalState = struct( ...
        'time_s',               sampleTimes_s(end), ...
        'position_units',       goal_units, ...
        'velocity_units_s',     [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    limits  = struct('maxVelocity_units_s', [4, 4]);
    options = struct('GoalTimeMode', "earliestArrival");
    [route_units, routeTime_s, record] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, obstacles, initialState, goalState, ...
        limits, sampleTimes_s, options);
end

function bar = createDeformingBar( ...
        name, leftX_units, rightX_units, lowerY_units, initialTime_s, finalTime_s)
    startY_units = [lowerY_units; lowerY_units; lowerY_units + 1; lowerY_units + 1];
    endY_units   = [lowerY_units; lowerY_units; lowerY_units + 1.25; lowerY_units + 1.25];
    x_units      = [leftX_units; rightX_units; rightX_units; leftX_units];
    bar = obstacleAvoidance.obstacles.createObstacle(name, ...
        [initialTime_s; finalTime_s], {x_units; x_units}, ...
        {startY_units; endY_units}, 0);
end

function sliver = createSliver(height_units)
    x_units      = [0; 10; 20; 10];
    startY_units = [0; height_units; 0; -height_units];
    endY_units   = 1.2 * startY_units;
    sliver = obstacleAvoidance.obstacles.createObstacle("thin sliver", ...
        [0; 10], {x_units; x_units}, {startY_units; endY_units}, 0);
end
