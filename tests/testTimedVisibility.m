function tests = testTimedVisibility
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testTimedVisibility.m')
%**************************************************************************
% PURPOSE
%   - Preserve goal windows, physical clock bounds, and exact timed collision checks.
%**************************************************************************
% INPUTS
%   - MATLAB function-based unit test framework.
%**************************************************************************
% OUTPUTS
%   - Function test array with independently named regression cases.
%**************************************************************************
% UNITS
%   - Position uses coordinate units and time uses seconds.
%**************************************************************************

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testGoalWindowAndTransitContracts(testCase)
    runCases(testCase, { ...
        @staticSceneRetainsFreePointWait; @nonrestGoalRemainsAvailableAsTransitNode; ...
        @clearEarlyGoalEdgeIsRetestedAtTerminalClock; @dominatedGoalTransitRetainsTerminalEdgeProvenance; ...
        @reachableFirstWindowRetainsItsSafeGoalInterval; @missedFirstWindowWaitsAtNearestSafeNode; ...
        @rampBoundSkipsPhysicallyImpossibleFirstWindow; @publicPlannerChoosesAReopenedGoalWindow});
end

function testRestAndStoppingBoundContracts(testCase)
    runCases(testCase, { ...
        @fixedArrivalBoundsIntermediateNodeFromRest; @smallSpeedRestBoundDoesNotUnderflow; ...
        @subnormalSpeedRestBoundUsesAccelerationBranch; @largeDistanceRestBoundDoesNotOverflow; ...
        @fixedArrivalRestBoundRoundsDownAtLargeTime; @fixedArrivalRestBoundAllowsForDisplacementRounding; ...
        @earliestArrivalKeepsItsRelaxedClock; @fixedClockBatchesUnequalAxisSources; ...
        @fixedArrivalStoppingToleranceUsesRemainingTime; ...
        @fixedArrivalStoppingToleranceHandlesOverflowedDuration; @fixedArrivalNeedsTimeToStopByDeadline; ...
        @transitGoalVisitDoesNotNeedStoppingTime; @movingStartKeepsOnlyGoalStoppingBound});
end

function testTimedCollisionAndAccelerationContracts(testCase)
    runCases(testCase, { ...
        @remoteCellsLeaveRouteAndCountsUnchanged; @temporalSegmentBoxMismatchStaysClear; ...
        @exactMovingContactStaysBlockedInBothDirections; @strictInteriorCollisionSkipsOnlyBlockedArrivals; ...
        @proofStopsAtTheFirstClearArrival; @sharpCornerToleranceOvershootStaysBlocked; ...
        @pointOutsideSharpCornerOvershootStaysVisible; @subToleranceMotionStartsAtTheNextPhysicalLayer; ...
        @boundedPairCachePreservesExactSearch});
end

function testObstacleLifetimeSearchContracts(testCase)
    runCases(testCase, { ...
        @finiteLivedStaticObstacleDoesNotDowngradeAnotherWall; ...
        @partiallyActiveStaticObstacleIsCheckedOnItsSubInterval; @timedSearchMovingAndStationaryIntervals; ...
        @mixedStaticObstacleLifetimes});
end

function staticSceneRetainsFreePointWait(testCase)
    % A stationary wait is a zero-length segment. The exact static predicate
    % must decide it by point occupancy, not by degenerate edge algebra that
    % declares every wait blocked whenever a time-invariant obstacle exists.
    farBox_units = [40,40;42,40;42,42;40,42];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'far static obstacle',0,{farBox_units(:,1)},{farBox_units(:,2)},0);
    nodes_units = [0,0;4,0];
    edgeCost_units = [0,4;4,0];
    initial = struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goal = struct('time_s',2,'position_units',nodes_units(2,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    % Keep acceleration and jerk ramps below the search clock precision.
    limits = struct('maxVelocity_units_s', [10,10], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    options = struct('GoalTimeMode',"fixedArrival");
    [route_units,routeTime_s] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,edgeCost_units,obstacle,initial,goal,limits,[0;1;2],options);
    assertNotEmpty(testCase,routeTime_s);
    verifyEqual(testCase,routeTime_s(end),goal.time_s,'AbsTol',1e-12);
    goalRows = all(abs(route_units-goal.position_units)<=1e-12,2);
    % The 0.4 s edge lands on the first layer at or after its physical
    % arrival, then the free goal waits to the given horizon.
    verifyGreaterThanOrEqual(testCase,nnz(goalRows),2);
    verifyEqual(testCase,routeTime_s(find(goalRows,1,'first')),1,'AbsTol',1e-12);
end

function nonrestGoalRemainsAvailableAsTransitNode(testCase)
    % Endpoint timing must not remove the goal coordinate from the geometric
    % graph. This only feasible route crosses it early, visits a refuge, and
    % returns at the given nonrest terminal layer.
    startBlocker_units = [-0.02,-0.08;0.02,-0.08;0.02,0.08;-0.02,0.08];
    startBlocker = obstacleAvoidance.obstacles.createObstacle( ...
        'start wait blocker',[0.2;3], ...
        {startBlocker_units(:,1);startBlocker_units(:,1)}, ...
        {startBlocker_units(:,2);startBlocker_units(:,2)},0);
    chordBlocker_units = [0.45,-0.08;0.55,-0.08;0.55,0.08;0.45,0.08];
    chordBlocker = obstacleAvoidance.obstacles.createObstacle( ...
        'late direct chord blocker',[1.4;1.6], ...
        {chordBlocker_units(:,1);chordBlocker_units(:,1)}, ...
        {chordBlocker_units(:,2);chordBlocker_units(:,2)},0);
    obstacles = obstacleAvoidance.obstacles.combineObstacles( ...
        {startBlocker;chordBlocker});

    nodes_units = [0,0;1,0;1,2];
    edgeCost_units = [0,1,Inf;1,0,2;Inf,2,0];
    initial = struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal = struct('time_s',3,'position_units',nodes_units(2,:), ...
        'velocity_units_s',[0.25,0],'acceleration_units_s2',[0,0]);
    limits = struct('maxVelocity_units_s',[3,3], ...
        'maxAcceleration_units_s2',[10,10],'maxJerk_units_s3',[20,20]);
    [route_units,routeTime_s] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,edgeCost_units,obstacles,initial,goal,limits,(0:3).', ...
        struct('GoalTimeMode',"fixedArrival"));

    expectedRoute_units = nodes_units([1,2,3,2],:);
    verifyEqual(testCase,route_units,expectedRoute_units,'AbsTol',1e-12);
    verifyEqual(testCase,routeTime_s,(0:3).','AbsTol',1e-12);
end

function clearEarlyGoalEdgeIsRetestedAtTerminalClock(testCase)
    % A clear transit arrival does not prove a rest endpoint. Retest the same
    % edge at the first eligible terminal layer when the source cannot wait.
    startBlocker_units = [-0.02,-0.08;0.02,-0.08;0.02,0.08;-0.02,0.08];
    startBlocker = obstacleAvoidance.obstacles.createObstacle( ...
        'start wait blocker',[0.2;5], ...
        {startBlocker_units(:,1);startBlocker_units(:,1)}, ...
        {startBlocker_units(:,2);startBlocker_units(:,2)},0);
    nodes_units = [0,0;4,0];
    edgeCost_units = [0,4;4,0];
    initial = struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal = struct('time_s',5,'position_units',nodes_units(2,:), ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    limits = struct('maxVelocity_units_s',[10,10], ...
        'maxAcceleration_units_s2',[1,1],'maxJerk_units_s3',[20,20]);
    [route_units,routeTime_s] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,edgeCost_units,startBlocker,initial,goal,limits,(0:5).', ...
        struct('GoalTimeMode',"fixedArrival"));

    verifyEqual(testCase,route_units,nodes_units,'AbsTol',1e-12);
    verifyEqual(testCase,routeTime_s,[0;5],'AbsTol',1e-12);
end

function dominatedGoalTransitRetainsTerminalEdgeProvenance(testCase)
    % State dominance at an early goal visit must not discard a distinct
    % incoming edge whose stretched terminal clock has different clearance.
    nodes_units = [0,0;3,0;1.5,-1.5];
    edgeCost_units = hypot( ...
        nodes_units(:,1)-nodes_units(:,1).',nodes_units(:,2)-nodes_units(:,2).');
    blockerCenters_units = [0,0;1.5,-1.5;1.5,0];
    activeIntervals_s = [0.2,4;1.2,4;1.9,2.1];
    halfSize_units = [0.02,0.02,0.06];
    obstacleParts = cell(3,1);
    for obstacleIndex = 1:3
        center_units = blockerCenters_units(obstacleIndex,:);
        halfSize = halfSize_units(obstacleIndex);
        vertices_units = center_units + halfSize * [-1,-1;1,-1;1,1;-1,1];
        obstacleParts{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
            "terminal provenance blocker " + obstacleIndex, ...
            activeIntervals_s(obstacleIndex,:), ...
            {vertices_units(:,1);vertices_units(:,1)}, ...
            {vertices_units(:,2);vertices_units(:,2)},0);
    end
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleParts);
    initial = struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal = struct('time_s',4,'position_units',nodes_units(2,:), ...
        'velocity_units_s',[0.25,0],'acceleration_units_s2',[0,0]);
    limits = struct('maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[10,10],'maxJerk_units_s3',[20,20]);
    [route_units,routeTime_s] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,edgeCost_units,obstacles,initial,goal,limits,(0:4).', ...
        struct('GoalTimeMode',"fixedArrival"));

    verifyEqual(testCase,route_units,nodes_units([1,3,2],:),'AbsTol',1e-12);
    verifyEqual(testCase,routeTime_s,[0;1;4],'AbsTol',1e-12);
end

function reachableFirstWindowRetainsItsSafeGoalInterval(testCase)
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

function missedFirstWindowWaitsAtNearestSafeNode(testCase)
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

function rampBoundSkipsPhysicallyImpossibleFirstWindow(testCase)
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

function publicPlannerChoosesAReopenedGoalWindow(testCase)
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

function fixedArrivalBoundsIntermediateNodeFromRest(testCase)
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

function smallSpeedRestBoundDoesNotUnderflow(testCase)
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

function subnormalSpeedRestBoundUsesAccelerationBranch(testCase)
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

function largeDistanceRestBoundDoesNotOverflow(testCase)
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

function fixedArrivalRestBoundRoundsDownAtLargeTime(testCase)
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

function fixedArrivalRestBoundAllowsForDisplacementRounding(testCase)
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

function earliestArrivalKeepsItsRelaxedClock(testCase)
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

function fixedClockBatchesUnequalAxisSources(testCase)
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

function fixedArrivalStoppingToleranceUsesRemainingTime(testCase)
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

function fixedArrivalStoppingToleranceHandlesOverflowedDuration(testCase)
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

function fixedArrivalNeedsTimeToStopByDeadline(testCase)
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

function transitGoalVisitDoesNotNeedStoppingTime(testCase)
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

function movingStartKeepsOnlyGoalStoppingBound(testCase)
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

function remoteCellsLeaveRouteAndCountsUnchanged(testCase)
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

function temporalSegmentBoxMismatchStaysClear(testCase)
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

function exactMovingContactStaysBlockedInBothDirections(testCase)
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

function strictInteriorCollisionSkipsOnlyBlockedArrivals(testCase)
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

function proofStopsAtTheFirstClearArrival(testCase)
    obstacle = createDeformingBar("departing bar", 4, 6, -0.5, 0, 2);
    sampleTimes_s = (0:0.1:10).';
    [route_units, routeTime_s, record] = runSearch( ...
        obstacle, [0, 0], [10, 0], sampleTimes_s);

    verifyEqual(testCase, route_units(end, :), [10, 0], 'AbsTol', 1e-12);
    verifyEqual(testCase, routeTime_s(end), 3.6, 'AbsTol', 1e-12);
    verifyGreaterThan(testCase, record.ProvenSkippedTransitionCount, 0);
end

function sharpCornerToleranceOvershootStaysBlocked(testCase)
    sliver = createSliver(1e-7);
    [route_units, routeTime_s] = runSearch( ...
        sliver, [-1e-4, 0], [10, 5], (0:10).');

    verifyEmpty(testCase, route_units);
    verifyEmpty(testCase, routeTime_s);
end

function pointOutsideSharpCornerOvershootStaysVisible(testCase)
    sliver = createSliver(1e-7);
    [route_units, routeTime_s] = runSearch( ...
        sliver, [-1, 0], [10, 5], (0:10).');

    verifyEqual(testCase, route_units(end, :), [10, 5], 'AbsTol', 1e-12);
    verifyNotEmpty(testCase, routeTime_s);
end

function subToleranceMotionStartsAtTheNextPhysicalLayer(testCase)
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
    % Keep acceleration and jerk ramps below the search clock precision.
    limits  = struct('maxVelocity_units_s', [1e13, 1e13], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    options = struct('GoalTimeMode', "earliestArrival");

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, obstacle, initialState, goalState, ...
        limits, [0; 1], options);

    verifyEqual(testCase, route_units, nodes_units, 'AbsTol', 1e-12);
    verifyEqual(testCase, routeTime_s, [0; 1], 'AbsTol', 1e-12);
end

function boundedPairCachePreservesExactSearch(testCase)
    obstacles = [ ...
        createDeformingBar("first remote bar", 400, 402, 2, 0, 10), ...
        createDeformingBar("second remote bar", 500, 502, 2, 0, 10)];
    sampleTimes_s = [0; 10];
    [expectedRoute_units, expectedRouteTime_s] = runSearch( ...
        obstacles, [0, 0], [10, 0], sampleTimes_s);

    % With at least two moving cells, 1,450 nodes exceed the eager cache
    % work bound. All extra nodes are disconnected, so only storage strategy
    % changes and the same exact start-to-goal edge remains supplied.
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
    % Keep acceleration and jerk ramps below the search clock precision.
    limits  = struct('maxVelocity_units_s', [4, 4], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    options = struct('GoalTimeMode', "earliestArrival");
    [route_units, routeTime_s] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units, edgeCost_units, obstacles, initialState, goalState, ...
        limits, sampleTimes_s, options);

    verifyEqual(testCase, route_units, expectedRoute_units);
    verifyEqual(testCase, routeTime_s, expectedRouteTime_s);
end

function finiteLivedStaticObstacleDoesNotDowngradeAnotherWall(testCase)
    % An always-active thin wall must stay exactly checked even when another
    % time-invariant obstacle exists only on part of the horizon; the exact
    % check applies per obstacle over each edge's own active sub-interval.
    wall_units = [0.09,-1;0.11,-1;0.11,1;0.09,1];
    wall = obstacleAvoidance.obstacles.createObstacle('thin wall',0, ...
        {wall_units(:,1)},{wall_units(:,2)},0);
    remote_units = [40,40;41,40;41,41;40,41];
    remote = obstacleAvoidance.obstacles.createObstacle('short-lived remote',[0;1], ...
        {remote_units(:,1);remote_units(:,1)},{remote_units(:,2);remote_units(:,2)},0);
    obstacles = obstacleAvoidance.obstacles.combineObstacles({wall;remote});
    nodes_units = [-1,0;2,0;0.1,1.5];
    edgeCost_units = hypot(nodes_units(:,1)-nodes_units(:,1).',nodes_units(:,2)-nodes_units(:,2).');
    initial = struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goal = struct('time_s',4,'position_units',nodes_units(2,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    % Keep acceleration and jerk ramps below the search clock precision.
    limits = struct('maxVelocity_units_s', [4,4], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    [route_units,routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,edgeCost_units,obstacles,initial,goal,limits,(0:0.5:4).', ...
        struct('GoalTimeMode',"earliestArrival"));
    assertNotEmpty(testCase,routeTime_s);
    verifyTrue(testCase,any(all(abs(route_units-nodes_units(3,:))<=1e-12,2)), ...
        'The exact wall must force the detour node into the route.');
    directRows = [all(abs(route_units(1:end-1,:)-nodes_units(1,:))<=1e-12,2), ...
        all(abs(route_units(2:end,:)-nodes_units(2,:))<=1e-12,2)];
    verifyFalse(testCase,any(all(directRows,2)),'The wall-crossing direct edge was accepted.');
end

function partiallyActiveStaticObstacleIsCheckedOnItsSubInterval(testCase)
    % A static blocker that disappears at 2.9 s must still reject an edge
    % whose motion enters it before that instant, and admit the edge whose
    % active sub-segment stays clear.
    blocker_units = [3.8,-0.2;4.2,-0.2;4.2,0.2;3.8,0.2];
    blocker = obstacleAvoidance.obstacles.createObstacle('goal blocker',[0;2.9], ...
        {blocker_units(:,1);blocker_units(:,1)},{blocker_units(:,2);blocker_units(:,2)},0);
    nodes_units = [0,0;4,0];
    edgeCost_units = [0,4;4,0];
    initial = struct('time_s',0,'position_units',nodes_units(1,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goal = struct('time_s',4,'position_units',nodes_units(2,:), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    % Keep acceleration and jerk ramps below the search clock precision.
    limits = struct('maxVelocity_units_s', [2,2], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    [route_units,routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,edgeCost_units,blocker,initial,goal,limits,(0:4).', ...
        struct('GoalTimeMode',"fixedArrival"));
    assertNotEmpty(testCase,routeTime_s);
    verifyEqual(testCase,route_units(end,:),goal.position_units,'AbsTol',1e-12);
    verifyEqual(testCase,routeTime_s(end),4,'AbsTol',1e-12);
    % The direct four-second traversal reaches only x = 2.9 before the
    % blocker disappears. Do not insert a preferred late-departure wait when
    % the first-discovered equal-length physical route is already clear.
    verifyEqual(testCase,routeTime_s(end-1),0,'AbsTol',1e-12);
    verifyEqual(testCase,route_units(end-1,:),nodes_units(1,:),'AbsTol',1e-12);
end

function timedSearchMovingAndStationaryIntervals(testCase)
    box = [-0.2,-3;0.2,-3;0.2,3;-0.2,3];
    nodes = [-5,0;5,0;-2,0;2,0;-2,2;2,2];
    cost = hypot(nodes(:,1)-nodes(:,1).',nodes(:,2)-nodes(:,2).');
    initialState = struct('time_s', 0, 'position_units', nodes(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 12, 'position_units', nodes(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    % Large finite rate limits keep these collision fixtures speed-limited.
    limits = struct('maxVelocity_units_s', [2, 2], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    for hasStationaryInterval = [false,true]
        if hasStationaryInterval
            sourceTimes_s = [0;6;6.5;12]; shifts_units = [0;0;8;8];
            expectedDeparture_s = 3.75; expectedArrival_s = 9;
        else
            sourceTimes_s = [0;12]; shifts_units = [0;8];
            expectedDeparture_s = 2.25; expectedArrival_s = 7.5;
        end
        obstacle = obstacleAvoidance.obstacles.createObstacle('crossing barrier',sourceTimes_s, ...
            repmat({box(:,1)},numel(sourceTimes_s),1), ...
            arrayfun(@(shift)box(:,2)+shift,shifts_units,'UniformOutput',false),0.1);
        % Search must extend partial preparation before reusing its snapshot.
        obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,0]);
        [route_units,routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
            nodes,cost,obstacle,initialState,goalState, ...
            limits,unique([(0:0.25:12).';sourceTimes_s]), ...
            struct('GoalTimeMode',"earliestArrival"));
        assertNotEmpty(testCase,routeTime_s);
        verifyEqual(testCase,routeTime_s(end-1:end),[expectedDeparture_s;expectedArrival_s]);
        verifyEqual(testCase,route_units,[repmat(nodes(1,:),numel(routeTime_s)-1,1);nodes(2,:)]);
        % Search owns preparation, including stale caches on edited sources.
        changed = obstacle;
        changed.x_units = cellfun(@(x)x+20,changed.x_units,'UniformOutput',false);
        changed.originalX_units = cellfun(@(x)x+20,changed.originalX_units,'UniformOutput',false);
        [~,changedTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
            nodes,cost,changed,initialState,goalState, ...
            limits,unique([(0:0.25:12).';sourceTimes_s]), ...
            struct('GoalTimeMode',"earliestArrival"));
        verifyEqual(testCase,changedTime_s(end),5);
    end
end

function mixedStaticObstacleLifetimes(testCase)
    movingBox = [-0.2,-3;0.2,-3;0.2,3;-0.2,3];
    moving = obstacleAvoidance.obstacles.createObstacle('moving',[0;12], ...
        {movingBox(:,1);movingBox(:,1)},{movingBox(:,2);movingBox(:,2)+8},0.1);
    fixedBox = [0.5,-1.25;1.5,-1.25;1.5,1.25;0.5,1.25];
    nodes = [-5,0;5,0;-2,0;2,0;-2,2;2,2];
    cost = hypot(nodes(:,1)-nodes(:,1).',nodes(:,2)-nodes(:,2).');
    initialState = struct('time_s', 0, 'position_units', nodes(1, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    goalState = struct('time_s', 12, 'position_units', nodes(2, :), ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
    % Large finite rate limits keep these collision fixtures speed-limited.
    limits = struct('maxVelocity_units_s', [2, 2], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
    histories = {0,[0;12],[3;6],[0;12]};
    for variant = 1:numel(histories)
        sourceTimes_s = histories{variant};
        x_units = repmat({fixedBox(:,1)},numel(sourceTimes_s),1);
        y_units = repmat({fixedBox(:,2)},numel(sourceTimes_s),1);
        % Near-equal source boundaries must still be treated as moving.
        if variant == 4
            x_units{end} = x_units{end} + 1e-12;
        end
        fixed = obstacleAvoidance.obstacles.createObstacle('stationary',sourceTimes_s,x_units,y_units,0);
        for reversed = [false,true]
            sources = {moving,fixed};
            if reversed
                sources = fliplr(sources);
            end
            obstacles = obstacleAvoidance.obstacles.combineObstacles(sources);
            [route_units,routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
                nodes,cost,obstacles,initialState,goalState, ...
                limits,unique([(0:0.25:12).';sourceTimes_s]), ...
                struct('GoalTimeMode',"earliestArrival"));
            assertNotEmpty(testCase,routeTime_s);
            if variant == 3
                % Once the stationary obstacle disappears, the direct edge
                % opens. A 5.25 s crossing from 3 s is at x = 0.5 by 5.89 s,
                % inside the box that exists until 6 s; the exact sub-interval
                % check rejects it (thirteen samples straddle that contact),
                % so the accepted direct edge takes 5.5 s and stays at
                % x <= 0.45 until the box disappears.
                verifyEqual(testCase,routeTime_s(end-1:end),[3;8.5]);
                verifyEqual(testCase,route_units(end-1:end,:),nodes(1:2,:));
            elseif variant == 4
                % Near-equal boundaries still use the affine moving-cell
                % predicate. It catches the same protected-corner contact as
                % the exact static predicate, including between old samples.
                verifyEqual(testCase,routeTime_s(end-3:end),[3.75;4;8;9.5]);
                verifyEqual(testCase,route_units(end-3:end,:),nodes([1,1,6,2],:));
            else
                % The edge [-2,0]->[2,2] passes at distance exactly zero from
                % the protected corner (0.5,1.25). The complete-interval
                % predicate rejects it, so the route departs after the moving
                % barrier passes and enters through [2,2] with clearance.
                verifyEqual(testCase,routeTime_s(end-2:end),[4;8;9.5]);
                verifyEqual(testCase,route_units(end-2:end,:),nodes([1,6,2],:));
            end
        end
    end
end

function blocker=createGoalBlocker(goalX_units,blockStart_s)
    vertices_units=[goalX_units-0.5,-0.5;goalX_units+0.5,-0.5; ...
        goalX_units+0.5,0.5;goalX_units-0.5,0.5];
    blocker=obstacleAvoidance.obstacles.createObstacle('temporary goal blocker', ...
        [blockStart_s;7],{vertices_units(:,1);vertices_units(:,1)}, ...
        {vertices_units(:,2);vertices_units(:,2)},0);
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
    % Keep acceleration and jerk ramps below the search clock precision.
    limits  = struct('maxVelocity_units_s', [4, 4], ...
        'maxAcceleration_units_s2', [1e30, 1e30], 'maxJerk_units_s3', [1e60, 1e60]);
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
