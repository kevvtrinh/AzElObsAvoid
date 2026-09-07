function tests = testTimedVisibilityScreening
%% Section 0: Header & Readme
% SYNTAX
%   tests = testTimedVisibilityScreening
% PURPOSE
%   Preserve timed reachability when rejected edges stop collision sampling.
% INPUTS
%   None. Replay the unchanged saved request and small moving-obstacle cases.
% OUTPUTS
%   Deterministic MATLAB function tests and independent validation checks.
% UNITS
%   Coordinate units, seconds, and motion derivatives.

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testBlockedSampleBeforeAtAndAfterMidpoint(testCase)
    % Each narrow moving box meets the edge at one of the retained samples.
    % A clear midpoint must not cause the other samples to be skipped.
    for collisionTime_s = [3 6 9]
        obstacle = crossingBox(collisionTime_s, 0);
        [route, ~, record] = runTwoNodeSearch(obstacle);
        verifyEmpty(testCase, route);
        verifyEqual(testCase, record.MotionEdgeCount, 0);
        verifyEqual(testCase, record.ReachableGoalLayerCount, 0);
    end
end

function testAllClearSamplesPreserveTheTimedRoute(testCase)
    % Move the same box away from the edge; preserve complete reachability.
    obstacle = crossingBox(3, 2);
    [route, time_s, record] = runTwoNodeSearch(obstacle);
    verifyEqual(testCase, route, [0 0; 12 0]);
    verifyEqual(testCase, time_s, [0; 12]);
    verifyEqual(testCase, record.MotionEdgeCount, 1);
    verifyEqual(testCase, record.WaitEdgeCount, 1);
    verifyEqual(testCase, record.SelectedGoalLayerIndex, 2);
end

function testExactVietnamRequestPreservesValidatedMotion(testCase)
    % The saved input is known feasible; require its established physical answer.
    loaded = load(fullfile(testCase.TestData.RepositoryRoot, 'Rogue Examples', 'vietnam_keepout_slew_input.mat'));
    [result, diagnosis] = obstacleAvoidance.planTrajectory(loaded.protectedObstacles, loaded.initialState, loaded.goalState, loaded.limits, loaded.options);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyEqual(testCase, result.TerminationReason, "goalReached");
    verifyEqual(testCase, result.TrajectoryDuration_s, 30, 'AbsTol', 1e-12);
    verifyEqual(testCase, obstacleAvoidance.geometry.routeLength(result.Route_units), 17.297991315813945, 'AbsTol', 1e-9);
    verifyEqual(testCase, obstacleAvoidance.geometry.routeLength(result.position_units), 17.305374620918947, 'AbsTol', 1e-9);
    verifyEqual(testCase, diagnosis.Search.MotionEdgeCount, 56571);
    verifyEqual(testCase, diagnosis.Search.WaitEdgeCount, 5618);
    verifyEqual(testCase, diagnosis.Search.RejectedTransitionCount, 4818010);
end

function testAuthoritativeSampleStaysSeparateFromStationaryHulls(testCase)
    % A narrow hull blocks repeated edge midpoints except at the exact sample
    % time. Reusing an interval answer there would erase the feasible route.
    lower = [-0.05 -0.05; 0.05 -0.05; 0.05 0.05; -0.05 0.05] + [6 -3];
    middle = [-0.05 -0.05; 0.05 -0.05; 0 0.05] + [6 3];
    obstacle = obstacleAvoidance.obstacles.createObstacle('changing ring count', [0; 12; 24], {lower(:,1); middle(:,1); lower(:,1)}, {lower(:,2); middle(:,2); lower(:,2)});
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    verifyFalse(testCase, any(obstacle.InternalPreparation.MatchingTopology));
    initial = struct();
    initial.time_s = 6;
    initial.position_units = [0 0];
    goal = struct();
    goal.time_s = 18;
    goal.position_units = [12 0];
    limits = struct();
    limits.maxVelocity_units_s = [3 3];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct('GoalTimeMode', 'fixedArrival'));
    nodes = [initial.position_units; goal.position_units];
    costs = [0 12; Inf 0];
    [route, time_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodes, costs, obstacle, initial, goal, limits, (6:0.5:18).', options);
    verifyEqual(testCase, route, [0 0; 12 0]);
    verifyEqual(testCase, time_s, [6; 18]);
end

function testRoundedEndpointKeepsItsOwnCollisionQuery(testCase)
    % Cancellation changes this sampled endpoint from x=1 to x=0. The latter
    % collides, so a free goal node must not replace the actual sample check.
    obstacle = obstacleAvoidance.obstacles.createObstacle('endpoint cancellation', 0, [-0.25; 0.25; 0.25; -0.25], [-0.25; -0.25; 0.25; 0.25], 0);
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    initial = struct();
    initial.time_s = 0;
    initial.position_units = [1e16 0];
    goal = struct();
    goal.time_s = 1;
    goal.position_units = [1 0];
    sampledEndpoint = initial.position_units + (goal.position_units - initial.position_units);
    verifyEqual(testCase, sampledEndpoint, [0 0]);
    verifyFalse(testCase, obstacleAvoidance.obstacles.queryPreparedObstacles(obstacle, 1, 0, 1));
    verifyTrue(testCase, obstacleAvoidance.obstacles.queryPreparedObstacles(obstacle, 0, 0, 1));
    limits = struct();
    limits.maxVelocity_units_s = [2e16 1];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct('GoalTimeMode', 'fixedArrival'));
    [route, ~, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch([initial.position_units; goal.position_units], [0 1; Inf 0], obstacle, initial, goal, limits, [0; 1], options);
    verifyEmpty(testCase, route);
    verifyEqual(testCase, record.MotionEdgeCount, 0);
end

function testRoundedEndpointTimeKeepsItsOwnCollisionQuery(testCase)
    % A free goal at t=1 does not clear the actual endpoint sampled at t=0.
    obstacle = obstacleAvoidance.obstacles.createObstacle('clock cancellation', [0; 0.5], 12 + [-0.25; 0.25; 0.25; -0.25], [-0.25; -0.25; 0.25; 0.25], 0);
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    initial = struct();
    initial.time_s = -1e16;
    initial.position_units = [0 0];
    goal = struct();
    goal.time_s = 1;
    goal.position_units = [12 0];
    verifyEqual(testCase, initial.time_s + (goal.time_s - initial.time_s), 0);
    verifyFalse(testCase, obstacleAvoidance.obstacles.queryPreparedObstacles(obstacle, 12, 0, 1));
    verifyTrue(testCase, obstacleAvoidance.obstacles.queryPreparedObstacles(obstacle, 12, 0, 0));
    limits = struct();
    limits.maxVelocity_units_s = [1 1];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct('GoalTimeMode', 'fixedArrival'));
    [route, ~, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch([initial.position_units; goal.position_units], [0 12; Inf 0], obstacle, initial, goal, limits, [initial.time_s; goal.time_s], options);
    verifyEmpty(testCase, route);
    verifyEqual(testCase, record.MotionEdgeCount, 0);
end

function testOverflowedQueryTimeDoesNotPolluteFiniteTimeCache(testCase)
    % Preserve the original sampled-search behavior even when a duration
    % overflows. Invalid query clocks must not supply answers for finite ones.
    obstacle = obstacleAvoidance.obstacles.createObstacle('overflow clock control', 0, 6 + [-0.25; 0.25; 0.25; -0.25], [-0.25; -0.25; 0.25; 0.25], 0);
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    initial = struct();
    initial.time_s = -1e308;
    initial.position_units = [0 0];
    goal = struct();
    goal.time_s = 1e308;
    goal.position_units = [12 0];
    limits = struct();
    limits.maxVelocity_units_s = [1 1];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct('GoalTimeMode', 'fixedArrival'));
    [route, time_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch([initial.position_units; goal.position_units], [0 12; Inf 0], obstacle, initial, goal, limits, [initial.time_s; 0; goal.time_s], options);
    verifyEqual(testCase, route, [0 0; 12 0]);
    verifyEqual(testCase, time_s, [initial.time_s; goal.time_s]);
end

function obstacle = crossingBox(collisionTime_s, yOffset_units)
    % The box translates across the straight edge at one specified time.
    corners = [-0.05 -0.05; 0.05 -0.05; 0.05 0.05; -0.05 0.05];
    first = corners + [collisionTime_s, yOffset_units - 0.1 * collisionTime_s];
    last = first + [0 1.2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('crossing box', [0; 12], {first(:,1); last(:,1)}, {first(:,2); last(:,2)});
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
end

function [route, time_s, record] = runTwoNodeSearch(obstacles)
    % With only the start and goal layers, the single edge has no alternate clock.
    initial = struct();
    initial.time_s = 0;
    initial.position_units = [0 0];
    goal = struct();
    goal.time_s = 12;
    goal.position_units = [12 0];
    limits = struct();
    limits.maxVelocity_units_s = [2 2];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct('GoalTimeMode', 'fixedArrival'));
    nodes = [initial.position_units; goal.position_units];
    costs = [0 12; 12 0];
    [route, time_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodes, costs, obstacles, initial, goal, limits, [0; 12], options);
end
