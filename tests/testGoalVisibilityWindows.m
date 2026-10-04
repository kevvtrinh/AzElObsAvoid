function tests = testGoalVisibilityWindows
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testGoalVisibilityWindows.m')
%**************************************************************************
% PURPOSE
%   - Verify goal wait windows and necessary clock bounds for timed routes.
%**************************************************************************
% INPUTS
%   - MATLAB function-based unit test framework.
%**************************************************************************
% OUTPUTS
%   - Goal-window and route-clock regression results.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************
tests=functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testReachableFirstWindowRetainsItsSafeGoalInterval(testCase)
    blocker=createGoalBlocker(4,3);
    nodes_units=[0,0;4,0];
    costs_units=[0,4;4,0];
    initial=struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goal=struct('time_s',10,'position_units',nodes_units(2,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits=struct('maxVelocity_units_s', [4,4], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    options=struct('GoalTimeMode',"earliestArrival");
    [route_units,routeTime_s,record]=obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,costs_units,blocker,initial,goal,limits,(0:10).',options);
    verifyEqual(testCase,routeTime_s,[0;1]);
    verifyEqual(testCase,route_units,[0,0;4,0],'AbsTol',1e-12);
    verifyEqual(testCase,record.SelectedGoalWindowStartTime_s,0);
    verifyEqual(testCase,record.SelectedGoalWindowEndTime_s,2);
    verifyLessThan(testCase,routeTime_s(end),goal.time_s);
end

function testMissedFirstWindowWaitsAtNearestSafeNode(testCase)
    blocker=createGoalBlocker(10,2);
    nodes_units=[0,0;10,0;8,0];
    costs_units=Inf(3);
    costs_units(1:4:end)=0;
    costs_units(1,3)=8; costs_units(3,1)=8;
    costs_units(2,3)=2; costs_units(3,2)=2;
    initial=struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goal=struct('time_s',10,'position_units',nodes_units(2,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits=struct('maxVelocity_units_s', [8,8], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    options=struct('GoalTimeMode',"earliestArrival");
    [route_units,routeTime_s]=obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,costs_units,blocker,initial,goal,limits,(0:10).',options);
    stagingRows=find(all(abs(route_units-[8,0])<=1e-12,2));
    verifyGreaterThanOrEqual(testCase,numel(stagingRows),2);
    verifyLessThanOrEqual(testCase,routeTime_s(stagingRows(1)),1);
    verifyGreaterThanOrEqual(testCase,routeTime_s(stagingRows(end)),3);
    verifyEqual(testCase,route_units(end,:),goal.position_units,'AbsTol',1e-12);
    verifyEqual(testCase,routeTime_s(end),8,'AbsTol',1e-12);
    verifyLessThan(testCase,routeTime_s(end),goal.time_s);
end

function testRampBoundSkipsPhysicallyImpossibleFirstWindow(testCase)
    blocker=createGoalBlocker(10,3);
    nodes_units=[0,0;10,0];
    costs_units=[0,10;10,0];
    initial=struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal=struct('time_s',12,'position_units',nodes_units(2,:), ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    limits=struct('maxVelocity_units_s',[20,20], ...
        'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
    options=struct('GoalTimeMode',"earliestArrival");
    [route_units,routeTime_s,record]= ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,costs_units,blocker,initial,goal,limits,(0:12).',options);
    verifyGreaterThan(testCase,record.MinimumGoalArrivalTime_s,3);
    verifyLessThan(testCase,record.MinimumGoalArrivalTime_s,7);
    verifyGreaterThan(testCase,routeTime_s(end),7);
    verifyEqual(testCase,route_units(end,:),goal.position_units,'AbsTol',1e-12);
    verifyLessThan(testCase,routeTime_s(end),goal.time_s);
end

function testFixedArrivalBoundsIntermediateNodeFromRest(testCase)
    nodes_units = [0, 0; 30, 0; 20, 3];
    edgeCost_units = hypot(nodes_units(:, 1) - nodes_units(:, 1).', ...
        nodes_units(:, 2) - nodes_units(:, 2).');
    blockerVertices_units = [24, -0.3; 26, -0.3; 26, 0.3; 24, 0.3];
    blocker = obstacleAvoidance.obstacles.createObstacle("direct edge blocker", 0, ...
        {blockerVertices_units(:, 1)}, {blockerVertices_units(:, 2)}, 0);
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 40, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [1, 1], 'maxJerk_units_s3', [4, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, blocker, initialState, goalState, limits, ...
        (0:40).', struct('GoalTimeMode', "fixedArrival"));

    % The direct edge is blocked. Reaching x = 20 from rest takes at least
    % sqrt(2 x 20 / 1) = 6.32 s on any route to the middle node.
    middleRowIndex = find(all(route_units == nodes_units(3, :), 2), 1, "first");
    verifyNotEmpty(testCase, middleRowIndex);
    verifyEqual(testCase, routeTime_s(middleRowIndex), 7);
    verifyEqual(testCase, routeTime_s(end), goalState.time_s);
end

function testSmallSpeedRestBoundDoesNotUnderflow(testCase)
    nodes_units = [0, 0; 1e-201, 0];
    edgeCost_units = [0, 1e-201; 1e-201, 0];
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 0.55, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [2 * 1e-201 / 0.55, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [1e-200, 10], ...
        'maxAcceleration_units_s2', [1e-200, 1], 'maxJerk_units_s3', [1e-198, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        [0; 0.55], struct('GoalTimeMode', "fixedArrival"));

    % Full speed starts after 5e-201 units, so the rest bound is
    % sqrt(2 x 1e-201 / 1e-200) = 0.447 s. Squaring speed first loses
    % that threshold and gives 0.6 s, incorrectly rejecting this layer.
    verifyEqual(testCase, route_units, nodes_units);
    verifyEqual(testCase, routeTime_s, [0; 0.55]);
end

function testSubnormalSpeedRestBoundUsesAccelerationBranch(testCase)
    q = realmin * eps;
    nodes_units = [0, 0; 40 * q, 0; 11 * q, 0];
    edgeCost_units = Inf(3);
    edgeCost_units(1:4:end) = 0;
    edgeCost_units(1, 3) = 11 * q;
    edgeCost_units(3, 1) = 11 * q;
    edgeCost_units(3, 2) = 29 * q;
    edgeCost_units(2, 3) = 29 * q;
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 100, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [q, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [5 * q, 10], ...
        'maxAcceleration_units_s2', [q, 1], 'maxJerk_units_s3', [1000 * q, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        [0; 4.695; 100], struct('GoalTimeMode', "fixedArrival"));

    % Reaching 11 q from rest takes sqrt(22) = 4.6904 s. Halving 5 q
    % rounds to 2 q and chooses a 4.7 s cruising bound, losing the layer.
    verifyEqual(testCase, route_units, nodes_units([1, 3, 2], :));
    verifyEqual(testCase, routeTime_s, [0; 4.695; 100]);
end

function testLargeDistanceRestBoundDoesNotOverflow(testCase)
    nodes_units = [0, 0; 1e150, 0];
    edgeCost_units = [0, 1e150; 1e150, 0];
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 4e155, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [5e-6, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [1, 10], ...
        'maxAcceleration_units_s2', [1e-160, 1], 'maxJerk_units_s3', [1, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        [0; 4e155], struct('GoalTimeMode', "fixedArrival"));

    % Full speed starts after 5e159 units, so the x bound is
    % sqrt(2) x 1e155 s, below the 4e155 s deadline. Computing
    % 2 D / a first overflows and incorrectly removes the clear edge.
    verifyEqual(testCase, route_units, nodes_units);
    verifyEqual(testCase, routeTime_s, [0; 4e155]);
end

function testFixedArrivalRestBoundRoundsDownAtLargeTime(testCase)
    nodes_units = [0 0; 10000003 0];
    edgeCost_units = [0 10000003; 10000003 0];
    layerTimes_s = [0; 141421.37745051135];
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0 0], 'acceleration_units_s2', [0 0]);
    goalState = struct('time_s', 141421.37745051135, ...
        'position_units', nodes_units(2, :), ...
        'velocity_units_s', [141.421377 0], 'acceleration_units_s2', [0 0]);
    limits = struct('maxVelocity_units_s', [200 200], ...
        'maxAcceleration_units_s2', [0.001 1], ...
        'maxJerk_units_s3', [1e10 1e10]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        layerTimes_s, struct('GoalTimeMode', "fixedArrival"));

    % The rounded rest bound sits one ULP above the goal layer without
    % the downward guard, even though that layer is physically reachable.
    verifyEqual(testCase, route_units, nodes_units);
    verifyEqual(testCase, routeTime_s, layerTimes_s);
end

function testFixedArrivalRestBoundAllowsForDisplacementRounding(testCase)
    % The goal displacement is rounded up by the coordinate subtraction, and
    % the deadline equals the true rest bound to within 1e-11 s. Only a guard
    % that also covers that rounding keeps the reachable goal layer.
    nodes_units = [2.2204460492503128e-16 0; 2.020622470302625 0];
    distance_units = nodes_units(2, 1) - nodes_units(1, 1);
    edgeCost_units = [0 distance_units; distance_units 0];
    layerTimes_s = [0; 2087516.2958152567];
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0 0], 'acceleration_units_s2', [0 0]);
    goalState = struct('time_s', layerTimes_s(end), ...
        'position_units', nodes_units(2, :), ...
        'velocity_units_s', [1.9359106075993457e-6 0], 'acceleration_units_s2', [0 0]);
    limits = struct('maxVelocity_units_s', [1e-5 1], ...
        'maxAcceleration_units_s2', [9.273750875526926e-13 1], ...
        'maxJerk_units_s3', [1e10 1e10]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        layerTimes_s, struct('GoalTimeMode', "fixedArrival"));

    verifyEqual(testCase, route_units, nodes_units);
    verifyEqual(testCase, routeTime_s, layerTimes_s);
end

function testEarliestArrivalKeepsItsRelaxedClock(testCase)
    nodes_units = [0, 0; 20, 0];
    edgeCost_units = [0, 20; 20, 0];
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 30, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [1, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [1, 1], 'maxJerk_units_s3', [4, 4]);

    [route_units, routeTime_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        (0:30).', struct('GoalTimeMode', "earliestArrival"));

    % The moving goal makes minimumTravelTime use 20 / 10 = 2 s.
    % Applying the fixed-clock rest bound here would delay arrival to
    % layer 7 because sqrt(2 x 20 / 1) = 6.32 s.
    minimumTravelTime_s = obstacleAvoidance.input.minimumTravelTime( ...
        initialState, goalState, limits);
    verifyEqual(testCase, minimumTravelTime_s, 2);
    verifyEqual(testCase, record.MinimumGoalArrivalTime_s, minimumTravelTime_s);
    verifyEqual(testCase, route_units, nodes_units);
    verifyEqual(testCase, routeTime_s, [0; 2]);
end

function testFixedClockBatchesUnequalAxisSources(testCase)
    nodes_units = [0, 0; 30, 0; -6.9, 6.9; 6.9, 6.9];
    edgeCost_units = Inf(4);
    edgeCost_units(1:5:end) = 0;
    edgeCost_units(1, 3:4) = vecnorm( ...
        nodes_units(3:4, :) - nodes_units(1, :), 2, 2).';
    edgeCost_units(3:4, 2) = vecnorm( ...
        nodes_units(3:4, :) - nodes_units(2, :), 2, 2);
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 10, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [1.5, 1.5], 'maxJerk_units_s3', [4, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        [0; 1; 2; 3; 4; 8; 10], struct('GoalTimeMode', "fixedArrival"));

    % Straight rest-to-rest eligibility is 9.327 s, so layer 10 is first.
    % Rule A reaches both sources at layer 4: sqrt(2 x 6.9 / 1.5)
    % = 3.03 s. Leaving out layer 7 makes both goal edges first reach
    % layer 8, so their stopping bounds share one comparison vector.
    % Node 3 needs 36.9 / 10 + 10 / (2 x 1.5) = 7.02 s to stop,
    % more than the 10 - 4 = 6 s left. Node 4 needs
    % sqrt(2 x 23.1 / 1.5) = 5.55 s, so it can finish.
    % Swapping those source bounds would accept node 3 and reject node 4,
    % changing the selected route.
    verifyEqual(testCase, route_units, nodes_units([1, 4, 2], :));
    verifyEqual(testCase, routeTime_s, [0; 4; 10]);
end

function testFixedArrivalStoppingToleranceUsesRemainingTime(testCase)
    nodes_units = [0, 0; 10, 0];
    edgeCost_units = [0, 10; 10, 0];
    initialState = struct('time_s', 1e16, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [1, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', initialState.time_s + 2, ...
        'position_units', nodes_units(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [1, 1], 'maxJerk_units_s3', [4, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        [initialState.time_s; goalState.time_s], struct('GoalTimeMode', "fixedArrival"));

    % The speed bound tries this edge after 10 / 10 = 1 s, but stopping
    % over 10 units needs sqrt(20) = 4.47 s and only 2 s remain.
    % At a 1e16 s epoch, 256 x eps(epoch) = 512 s would hide the shortage.
    verifyEmpty(testCase, route_units);
    verifyEmpty(testCase, routeTime_s);
end

function testFixedArrivalStoppingToleranceHandlesOverflowedDuration(testCase)
    nodes_units = zeros(2, 2);
    edgeCost_units = zeros(2);
    initialState = struct('time_s', -1e308, 'position_units', [0, 0], ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 1e308, 'position_units', [0, 0], ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [1, 1], ...
        'maxAcceleration_units_s2', [1, 1], 'maxJerk_units_s3', [1, 1]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        [-1e308; 1e308], struct('GoalTimeMode', "fixedArrival"));

    % The finite endpoints have an infinite difference; zero travel still fits.
    verifyEqual(testCase, route_units, nodes_units);
    verifyEqual(testCase, routeTime_s, [-1e308; 1e308]);
end

function testFixedArrivalNeedsTimeToStopByDeadline(testCase)
    nodes_units = [0, 0; 20, 0; -15, 3];
    edgeCost_units = hypot(nodes_units(:, 1) - nodes_units(:, 1).', ...
        nodes_units(:, 2) - nodes_units(:, 2).');
    blockerVertices_units = [8, -0.3; 9, -0.3; 9, 0.3; 8, 0.3];
    blocker = obstacleAvoidance.obstacles.createObstacle("direct edge blocker", 0, ...
        {blockerVertices_units(:, 1)}, {blockerVertices_units(:, 2)}, 0);
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 14, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [1, 1], 'maxJerk_units_s3', [4, 4]);

    % The blocker cuts the direct y = 0 edge. From node 3 to the goal,
    % the line stays near y = 1 while crossing the blocker's x range.
    % Rule A needs sqrt(2 x 15 / 1) = 5.48 s to reach node 3, so its
    % first layer is 6 s. The 35-unit x edge needs 35 / 10 = 3.5 s,
    % putting the first goal visit at 10 s. The direct rest-to-rest
    % goal time is about 9.2 s, so that goal layer is eligible.
    % Stopping over 35 units needs sqrt(2 x 35 / 1) = 8.37 s.
    % A 14 s deadline leaves 14 - 6 = 8 s, so no source can finish.
    [route_units, ~] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, blocker, initialState, goalState, limits, ...
        (0:goalState.time_s).', struct('GoalTimeMode', "fixedArrival"));
    verifyEmpty(testCase, route_units);

    % A 15 s deadline leaves 15 - 6 = 9 s. The 10 s visit can finish
    % the request by holding at the clear goal until 15 s.
    goalState.time_s = 15;
    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, blocker, initialState, goalState, limits, ...
        (0:goalState.time_s).', struct('GoalTimeMode', "fixedArrival"));
    verifyEqual(testCase, route_units, nodes_units([1, 3, 2, 2], :));
    verifyEqual(testCase, routeTime_s, [0; 6; 10; 15]);
end

function testTransitGoalVisitDoesNotNeedStoppingTime(testCase)
    nodes_units = [0, 0; 20, 0; -10, 0; 20, 10];
    edgeCost_units = Inf(4);
    edgeCost_units(1:5:end) = 0;
    edgeCost_units(1, 3) = 10;
    edgeCost_units(3, 2) = 30;
    edgeCost_units(2, 4) = 10;
    edgeCost_units(4, 2) = 10;

    goalBlockerVertices_units = [19.5, -0.5; 20.5, -0.5; 20.5, 0.5; 19.5, 0.5];
    goalBlocker = obstacleAvoidance.obstacles.createObstacle("goal wait blocker", ...
        [9; 15], {goalBlockerVertices_units(:, 1); goalBlockerVertices_units(:, 1)}, ...
        {goalBlockerVertices_units(:, 2); goalBlockerVertices_units(:, 2)}, 0);
    corridorBlockerVertices_units = [9.5, -0.5; 10.5, -0.5; 10.5, 0.5; 9.5, 0.5];
    corridorBlocker = obstacleAvoidance.obstacles.createObstacle("late corridor blocker", ...
        [9; 30], {corridorBlockerVertices_units(:, 1); corridorBlockerVertices_units(:, 1)}, ...
        {corridorBlockerVertices_units(:, 2); corridorBlockerVertices_units(:, 2)}, 0);

    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 30, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [1, 1], 'maxJerk_units_s3', [4, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [goalBlocker, corridorBlocker], ...
        initialState, goalState, limits, (0:30).', ...
        struct('GoalTimeMode', "fixedArrival"));

    sourceRowIndex = find(all(route_units == nodes_units(3, :), 2), 1, "first");
    goalRows = find(all(route_units == nodes_units(2, :), 2));
    stagingRowIndex = find(all(route_units == nodes_units(4, :), 2), 1, "last");
    verifyNotEmpty(testCase, sourceRowIndex);
    verifyGreaterThanOrEqual(testCase, numel(goalRows), 2);
    verifyNotEmpty(testCase, stagingRowIndex);
    % Rule A puts x = -10 at 5 s. The 30-unit edge then needs 3 s at
    % v = 10, so the first goal visit is at 8 s before the blocker.
    % It is transit because the goal cannot stay clear through 30 s.
    % From this source the deadline leaves 25 s, more than the 7.75 s
    % stopping bound; the transit visit does not violate that bound.
    verifyEqual(testCase, routeTime_s(sourceRowIndex), 5);
    verifyEqual(testCase, routeTime_s(goalRows(1)), 8);
    verifyGreaterThanOrEqual(testCase, routeTime_s(goalRows(1)), ceil(sqrt(2 * 20)));
    completingGoalRowIndex = goalRows(find(goalRows > stagingRowIndex, 1, "first"));
    verifyNotEmpty(testCase, completingGoalRowIndex);
    % Staging is 10 units from the goal, and only its time before the
    % 30 s deadline must cover sqrt(2 x 10 / 1) = 4.47 s to stop.
    verifyGreaterThanOrEqual(testCase, ...
        goalState.time_s - routeTime_s(stagingRowIndex), sqrt(2 * 10));
    verifyEqual(testCase, routeTime_s(end), goalState.time_s);
end

function testMovingStartKeepsOnlyGoalStoppingBound(testCase)
    nodes_units = [0, 0; 20, 0];
    edgeCost_units = [0, 20; 20, 0];
    initialState = struct('time_s', 0, 'position_units', nodes_units(1, :), ...
        'velocity_units_s', [5, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 30, 'position_units', nodes_units(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    limits = struct('maxVelocity_units_s', [10, 10], ...
        'maxAcceleration_units_s2', [1, 1], 'maxJerk_units_s3', [4, 4]);

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, [], initialState, goalState, limits, ...
        (0:30).', struct('GoalTimeMode', "fixedArrival"));

    restStartState = initialState;
    restStartState.velocity_units_s = [0, 0];
    minimumRestToRestTime_s = obstacleAvoidance.input.minimumTravelTime( ...
        restStartState, goalState, limits);
    firstGoalRowIndex = find(all(route_units == nodes_units(2, :), 2), 1, "first");
    verifyNotEmpty(testCase, firstGoalRowIndex);
    % The moving start uses the 20 / 10 = 2 s speed bound. The 30 s
    % deadline leaves enough time for the 6.32 s stopping bound, so the
    % first goal visit does not have to wait until the vehicle stops.
    verifyEqual(testCase, routeTime_s(firstGoalRowIndex), 2);
    verifyLessThan(testCase, routeTime_s(firstGoalRowIndex), minimumRestToRestTime_s);
    verifyEqual(testCase, routeTime_s(end), goalState.time_s);
end

function testPublicPlannerChoosesAReopenedGoalWindow(testCase)
    goalX_units=10;
    vertices_units=[goalX_units-0.5,-0.5;goalX_units+0.5,-0.5; ...
        goalX_units+0.5,0.5;goalX_units-0.5,0.5];
    sourceTimes_s=linspace(3,7,17).';
    blocker=obstacleAvoidance.obstacles.createObstacle('dense goal blocker', ...
        sourceTimes_s,repmat({vertices_units(:,1)},17,1), ...
        repmat({vertices_units(:,2)},17,1),0);
    initial=struct('time_s',0,'position_units',[0,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal=struct('time_s',12,'position_units',[goalX_units,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    limits=struct('xInterval_units',[-2,12],'yInterval_units',[-3,3], ...
        'maxVelocity_units_s',[20,20], ...
        'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
    result=planner(blocker,initial,goal,limits, ...
        struct('GoalTimeMode',"earliestArrival"));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Diagnostics.Validation.Passed,result.Diagnostics.Validation.Message);
    verifyGreaterThan(testCase,result.ArrivalTime_s,7);
    verifyLessThan(testCase,result.ArrivalTime_s,goal.time_s);
end

function blocker=createGoalBlocker(goalX_units,blockStart_s)
    vertices_units=[goalX_units-0.5,-0.5;goalX_units+0.5,-0.5; ...
        goalX_units+0.5,0.5;goalX_units-0.5,0.5];
    blocker=obstacleAvoidance.obstacles.createObstacle('temporary goal blocker', ...
        [blockStart_s;7],{vertices_units(:,1);vertices_units(:,1)}, ...
        {vertices_units(:,2);vertices_units(:,2)},0);
end
