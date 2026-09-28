function tests = testArrivalSearchRegressions
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testArrivalSearchRegressions.m')
% PURPOSE: Exercise late feasible arrivals, free-clock selection against
%   waiting motions, arrival-time fixed-arrival recovery, and arrivals that
%   do not depend on the horizon when an obstacle clears the goal.
% INPUTS: MATLAB unit test framework and deterministic public planner inputs.
% OUTPUTS: Independent validation and arrival-search regression checks.
% UNITS: Coordinate units, seconds, and physical derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
end

function testCircleDetourBeatsWaiting(testCase)
    result = exampleMovingCircleNoWrap(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,9);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyLessThan(testCase,result.ArrivalTime_s, ...
        result.Diagnostics.VisibilityGraph.RouteTime_s(end));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
end

function testStaticSceneRetainsFreePointWait(testCase)
    % A stationary wait is a zero-length segment. The exact static predicate
    % must decide it by point occupancy, not by degenerate edge algebra that
    % declares every wait blocked whenever a time-invariant obstacle exists.
    farBox_units = [40,40;42,40;42,42;40,42];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'far static obstacle',0,{farBox_units(:,1)},{farBox_units(:,2)},0);
    nodes_units = [0,0;4,0];
    edgeCost_units = [0,4;4,0];
    initial = struct('time_s',0,'position_units',nodes_units(1,:));
    goal = struct('time_s',2,'position_units',nodes_units(2,:));
    limits = struct('maxVelocity_units_s',[10,10]);
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

function testNonrestGoalRemainsAvailableAsTransitNode(testCase)
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

function testClearEarlyGoalEdgeIsRetestedAtTerminalClock(testCase)
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

function testDominatedGoalTransitRetainsTerminalEdgeProvenance(testCase)
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

function testFiniteLivedStaticObstacleDoesNotDowngradeAnotherWall(testCase)
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
    initial = struct('time_s',0,'position_units',nodes_units(1,:));
    goal = struct('time_s',4,'position_units',nodes_units(2,:));
    limits = struct('maxVelocity_units_s',[4,4]);
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

function testPartiallyActiveStaticObstacleIsCheckedOnItsSubInterval(testCase)
    % A static blocker that disappears at 2.9 s must still reject an edge
    % whose motion enters it before that instant, and admit the edge whose
    % active sub-segment stays clear.
    blocker_units = [3.8,-0.2;4.2,-0.2;4.2,0.2;3.8,0.2];
    blocker = obstacleAvoidance.obstacles.createObstacle('goal blocker',[0;2.9], ...
        {blocker_units(:,1);blocker_units(:,1)},{blocker_units(:,2);blocker_units(:,2)},0);
    nodes_units = [0,0;4,0];
    edgeCost_units = [0,4;4,0];
    initial = struct('time_s',0,'position_units',nodes_units(1,:));
    goal = struct('time_s',4,'position_units',nodes_units(2,:));
    limits = struct('maxVelocity_units_s',[2,2]);
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

function testLongRequestUsesBudgetAfterPhysicalBound(testCase)
    box = [-1,-0.5;1,-0.5;1,0.5;-1,0.5];
    wall = obstacleAvoidance.obstacles.createObstacle('static crossing',[0;180],box(:,1),box(:,2));
    remote = obstacleAvoidance.obstacles.createObstacle('remote moving box',[0;180], ...
        {box(:,1)+20;box(:,1)+30},{box(:,2)+60;box(:,2)+60});
    initial = struct('time_s',0,'position_units',[-60,0]);
    goal = struct('time_s',180,'position_units',[60,0]);
    limits = struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
        'maxJerk_units_s3',[2.5,2.5]);
    result = planner([wall;remote],initial,goal,limits,struct('GoalTimeMode','earliestArrival','MaxArrivalTrials',40));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase,result.Diagnostics.TemporalSearch.EarliestPossibleArrival_s,60);
    verifyGreaterThanOrEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s, ...
        result.Diagnostics.TemporalSearch.EarliestPossibleArrival_s-result.Options.ArrivalTimeTolerance_s);
    verifyLessThanOrEqual(testCase,numel(result.Diagnostics.TemporalSearch.TrialTime_s),40);
    % The exact static-only reference arrives at 64 s. The moving obstacle is
    % remote, so the unified timed profile must stay within the 1% gate.
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,64.64);
end

function testMovingTargetPrescreenPreservesSolverBudget(testCase)
    % Necessary endpoint failures must not consume the bounded solver trials.
    % This approaching target is unreachable throughout the original eight
    % half-second slots but becomes reachable well inside its declared history.
    targetMotion = struct( ...
        'time_s',[0;60], ...
        'position_units',[40,0;10,0], ...
        'InterpolationMethod','linear');
    initial = restState(0,[0,0]);
    goal = struct('time_s',60,'targetMotion',targetMotion);
    limits = struct( ...
        'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalTrials',8);

    result = planner([],initial,goal,limits,options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase,result.ArrivalTime_s, ...
        options.TemporalResolution_s * options.MaxArrivalTrials);
    verifyGreaterThan(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,0);
    verifyLessThanOrEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount, ...
        options.MaxArrivalTrials);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount, ...
        numel(result.Diagnostics.TemporalSearch.TrialTime_s));
end

function testOffGridTargetBoundaryRetainsArrivalTimePriority(testCase)
    % Exact target sample times remain candidates even when they are not on
    % the regular arrival grid or at the horizon.
    targetMotion = struct( ...
        'time_s',[0;3.7;10], ...
        'position_units',[1,0;1,0;1,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',10,'targetMotion',targetMotion);
    limits = struct( ...
        'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',10, ...
        'MaxArrivalTrials',1);

    result = planner([],restState(0,[0,0]),goal,limits,options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s,3.7,'AbsTol',1e-12);
    verifyEqual(testCase,result.ArrivalTime_s,3.7,'AbsTol',1e-12);
end

function testNumericallyDistinctGridAndBoundaryAreBothRetained(testCase)
    % An exact source boundary and its rounded grid neighbor are distinct
    % declared clocks. Both remain searchable inside the bounded grid window.
    targetMotion = struct( ...
        'time_s',[0;0.3;1], ...
        'position_units',[0.005,0;0.005,0;0.005,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',1,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.1, ...
        'MaxArrivalCandidates',4);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,0.4,'AbsTol',1e-12);
end

function testLargeAbsoluteTimeRejectsUnrepresentableResolution(testCase)
    % Half-second increments are below the ulp at this absolute time. Reject
    % the ill-defined grid instead of silently changing its resolution.
    initialTime_s = 1e16;
    targetMotion = struct( ...
        'time_s',[initialTime_s;initialTime_s + 10], ...
        'position_units',[0.1,0;0.1,0], ...
        'InterpolationMethod','linear');
    goal = struct( ...
        'time_s',initialTime_s + 10, ...
        'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalCandidates',2);

    verifyError(testCase,@() planner( ...
        [],restState(initialTime_s,[0,0]),goal,struct(),options), ...
        'planner:UnrepresentableTemporalResolution');
end

function testNegativeAbsoluteTimeKeepsFirstRepresentableGridClock(testCase)
    % Toward +Inf, a negative power of two has half the spacing reported by
    % eps(abs(t)). The evaluated-grid search must not skip that first clock.
    initialTime_s = -1;
    firstLaterTime_s = typecast(typecast(initialTime_s,'uint64') - uint64(1),'double');
    resolution_s = firstLaterTime_s - initialTime_s;
    targetMotion = struct( ...
        'time_s',[initialTime_s;0], ...
        'position_units',[1e-18,0;1e-18,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',0,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',resolution_s, ...
        'MaxArrivalCandidates',1);

    result = planner([],restState(initialTime_s,[0,0]),goal,struct(),options);

    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,1);
end

function testGridSearchChecksEvaluatedMidpointRounding(testCase)
    % This resolution is three quarters of one ulp at t = 1. Index one
    % already rounds upward and must not be skipped by inverse arithmetic.
    initialTime_s = 1;
    resolution_s  = 3 * 2 ^ -54;
    firstGridTime_s = initialTime_s + resolution_s;
    targetMotion = struct( ...
        'time_s',[initialTime_s;2], ...
        'position_units',[100,0;100,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',2,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',resolution_s, ...
        'MaxArrivalCandidates',1);

    result = planner([],restState(initialTime_s,[0,0]),goal,struct(),options);

    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,1);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedTime_s, ...
        firstGridTime_s,'AbsTol',0);
end

function testRoundedGridNeverQueriesPastTargetHistory(testCase)
    % Binary rounding can place the final nominal grid time just beyond the
    % declared target history. That point is outside the planning request.
    targetMotion = struct( ...
        'time_s',[0.001;0.009], ...
        'position_units',[100,0;100,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',0.009,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.001);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"arrivalSearchExhausted");
    verifyFalse(testCase,result.Diagnostics.TemporalSearch.CandidateLimitReached);
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.SearchWindowExhausted);
end

function testTinyResolutionRejectsUnrepresentableGrid(testCase)
    % The requested grid cannot advance at this absolute time scale.
    targetMotion = struct( ...
        'time_s',[0.001;0.009], ...
        'position_units',[100,0;100,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',0.009,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',1e-20, ...
        'MaxArrivalCandidates',2);

    verifyError(testCase,@() planner( ...
        [],restState(0,[0,0]),goal,struct(),options), ...
        'planner:UnrepresentableTemporalResolution');
end

function testSubnormalResolutionRejectsInfiniteStepIndex(testCase)
    % A subnormal resolution overflows the finite horizon's step index.
    blocker_units = [3.5,-0.5;4.5,-0.5;4.5,0.5;3.5,0.5];
    blocker = obstacleAvoidance.obstacles.createObstacle( ...
        'terminal blocker',0,{blocker_units(:,1)},{blocker_units(:,2)},0);
    initial = restState(0,[0,0]);
    initial.velocity_units_s = [0.1,0];
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',1e-320, ...
        'MaxArrivalCandidates',2);

    verifyError(testCase,@() planner( ...
        blocker,initial,restState(10,[4,0]),struct(),options), ...
        'planner:UnrepresentableTemporalResolution');
end

function testPrescreenCandidateWorkAndStorageAreBounded(testCase)
    % A huge target history must not turn one permitted solver trial into an
    % unbounded endpoint scan or allocate by the much larger solver cap.
    targetMotion = struct( ...
        'time_s',[0;1e5], ...
        'position_units',[1000,0;1000,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',1e5,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalTrials',1e9, ...
        'MaxArrivalCandidates',8);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"arrivalSearchExhausted");
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,8);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount,0);
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.CandidateLimitReached);
end

function testCoincidentMovingTargetDoesNotSpendSolverBudget(testCase)
    % A target parked at the initial point creates invalid fixed-arrival
    % endpoints, not BMTP trials. Its later departure must remain searchable.
    targetMotion = struct( ...
        'time_s',[0;50;60], ...
        'position_units',[0,0;0,0;1,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',60,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalTrials',1);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,50.5,'AbsTol',1e-12);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount,1);
    verifyGreaterThanOrEqual(testCase, ...
        result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,100);
end

function testDisconnectedSnapshotRetainsValidatedWaitHonestly(testCase)
    result = exampleMovingBarrierWait(struct('PlotOutputs',false,'Verbose',false,'MaxArrivalTrials',1));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyGreaterThan(testCase,result.Diagnostics.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics.DepartureSchedule, ...
        'InitialRouteTimeBound_s'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'FixedArrivalTrialTime_s'));
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Selected);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).NextMethodAllowed);
    verifyFalse(testCase,result.Diagnostics.EarliestArrival.ArrivalTimeSearchUsed);
end

function testMovingGeometryDoesNotUseInitialRouteAsGlobalBound(testCase)
    result = exampleOpeningUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyFalse(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics.DepartureSchedule, ...
        'InitialRouteTimeBound_s'));
    verifyGreaterThan(testCase,result.Diagnostics.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Selected);
end

function testFailedWrappedTimedTimedSearchRetainsValidatedBestSoFar(testCase)
    result = exampleOpeningUShapedObstacle(struct( ...
        'PlotOutputs', false, 'Verbose', false, 'WrapX', true));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    % Exact polynomial sizing removed a control-bound stretch from the
    % departure motion (11.613388881629 s before). At the earlier clock the
    % timed subproblem reports a clean infeasibility (coneprog -2) instead
    % of the numerical breakdown (-10) it used to hit. Either way the
    % validated departure motion must stay selected.
    verifyEqual(testCase, result.ArrivalTime_s, 11.613388695819829, 'AbsTol', 1e-10);
    verifyEqual(testCase, [result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture", "timedVisibility"]);
    verifyTrue(testCase, result.Diagnostics.Attempts(1).Selected);
    verifyEqual(testCase, result.Diagnostics.Attempts(1).Message, "");
    verifyTrue(testCase, isnan(result.Diagnostics.Attempts(1).SolverExitFlag));
    verifyFalse(testCase, result.Diagnostics.Attempts(2).Success);
    verifyFalse(testCase, result.Diagnostics.Attempts(2).Selected);
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureStage, "proposal");
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureKind, ...
        "trajectorySubproblemInfeasible");
    verifyTrue(testCase, result.Diagnostics.Attempts(2).OptimizerIterateUnavailable);
    verifyTrue(testCase, result.Diagnostics.Attempts(2).NextMethodAllowed);

    timedSearch = result.Diagnostics.SolverDiagnostics.TimedSearchAttempt;
    verifyFalse(testCase, timedSearch.Success);
    verifyEqual(testCase, timedSearch.TerminationReason, "timedMotionInfeasible");
    verifyEqual(testCase, timedSearch.FailureStage, "proposal");
    verifyEqual(testCase, timedSearch.FailureKind, "trajectorySubproblemInfeasible");
    verifyTrue(testCase, timedSearch.OptimizerIterateUnavailable);
    verifyEqual(testCase, timedSearch.SolverDiagnostics.LastTrajectoryExitFlag, -2);
    verifySubstring(testCase, timedSearch.Message, "Problem is infeasible");
    verifyEqual(testCase, timedSearch.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyTrue(testCase, isfield(timedSearch.VisibilityGraph, 'TimedSearch'));
    verifyNotEmpty(testCase, timedSearch.VisibilityGraph.RouteTime_s);
    verifyEqual(testCase, timedSearch.Route_units, ...
        timedSearch.VisibilityGraph.Route_units);
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, ...
        "c3DepartureSchedule");
    verifyEqual(testCase, result.Diagnostics.Route_units, [0, 0; 0, -10], 'AbsTol', 1e-12);
end

function testFailedScaledBarrierTimedSearchRetainsValidatedBestSoFar(testCase)
    barrierX_units = [-0.2; -0.2; 0.2; 0.2];
    barrierY_units = [-3; 3; 3; -3];
    obstacleTime_s = [0; 6; 6.5; 12];
    barrierYOffset_units = [0; 0; 8; 8];
    barrier = obstacleAvoidance.obstacles.createObstacle( ...
        'translating barrier', obstacleTime_s, repmat({barrierX_units}, 4, 1), ...
        arrayfun(@(offset_units) barrierY_units + offset_units, ...
        barrierYOffset_units, 'UniformOutput', false), 0.1);

    remoteCenterY_units = 1e9;
    remoteBox_units = [-1, -1; 1, -1; 1, 1; -1, 1] + [0, remoteCenterY_units];
    remoteBox = obstacleAvoidance.obstacles.createObstacle( ...
        'remote convex box', 0, {remoteBox_units(:, 1)}, ...
        {remoteBox_units(:, 2)}, 0.1);
    obstacles = obstacleAvoidance.obstacles.combineObstacles(barrier, remoteBox);

    limits = struct( ...
        'xInterval_units',          [-180, 180], ...
        'yInterval_units',          [-5, remoteCenterY_units + 2], ...
        'maxVelocity_units_s',      [2, 2], ...
        'maxAcceleration_units_s2', [1, 1], ...
        'maxJerk_units_s3',         [2, 2]);
    options = struct( ...
        'GoalTimeMode',                  'earliestArrival', ...
        'WrapX',                         true, ...
        'TemporalResolution_s',          0.5, ...
        'BestSoFarRefinementTrialLimit', 0);

    result = planner(obstacles, restState(0, [-5, 0]), ...
        restState(12, [5, 0]), limits, options);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, "c3DepartureSchedule");
    verifyEqual(testCase, [result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture", "timedVisibility"]);
    verifyTrue(testCase, result.Diagnostics.Attempts(1).Selected);
    verifyFalse(testCase, result.Diagnostics.Attempts(2).Success);
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureStage, "numericalSolver");
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureKind, ...
        "optimizerIterateUnavailable");
    verifyFalse(testCase, result.Diagnostics.Attempts(2).NextMethodAllowed);
    verifyEqual(testCase, result.Diagnostics.Attempts(2).SolverExitFlag, -10);
    verifySubstring(testCase, result.Diagnostics.Attempts(2).Message, "numerically unstable");

    timedSearch = result.Diagnostics.SolverDiagnostics.TimedSearchAttempt;
    verifyEqual(testCase, timedSearch.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyNotEmpty(testCase, timedSearch.VisibilityGraph.RouteTime_s);
    verifyEqual(testCase, timedSearch.Route_units, ...
        timedSearch.VisibilityGraph.Route_units);
end

function testTimedTimedSearchValidationRejectionRemainsTerminal(testCase)
    base = planner([], restState(0, [0, 0]), restState(6, [4, 0]), ...
        fastLimits([-3, 3]), struct('GoalTimeMode', 'fixedArrival'));
    [request, scene] = explicitSearchInputs(base);
    parentRequest = obstacleAvoidance.planning.createParentRequest(request);
    parentRequest.WrapX                   = true;
    parentRequest.WrapY                   = false;
    parentRequest.GoalTime_s              = 6;
    parentRequest.FixedArrivalTrialTime_s = 5;
    request.parentRequest         = parentRequest;

    [rejected, accepted] = obstacleAvoidance.planning.tryTimedArrival( ...
        request, scene.preparedObstacles, ...
        repmat(base.Diagnostics.Attempts, 0, 1), 0, struct(), request.goalState.time_s);

    verifyFalse(testCase, accepted);
    verifyFalse(testCase, rejected.Success);
    verifyTrue(testCase, rejected.Diagnostics.SolverDiagnostics.Accepted);
    verifyFalse(testCase, rejected.Diagnostics.Validation.Passed);
    verifyEqual(testCase, rejected.TerminationReason, "invalidMotion");
    verifySubstring(testCase, rejected.Message, ...
        "failed independent validation");
end

function testBestSoFarRefinementBudgetIsExplicit(testCase)
    options = struct('PlotOutputs',false,'Verbose',false, ...
        'MaxArrivalTrials',40,'BestSoFarRefinementTrialLimit',1);
    result = exampleMovingBarrierWait(options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.MaximumTrialCount,1);
    verifyLessThanOrEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount,1);
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.TrialLimitReached);
    verifyTrue(testCase,result.Diagnostics.EarliestArrival.ArrivalTimeSearchUsed);
    arrivalTimeTrialIndices=find([result.Diagnostics.Attempts.Kind] == ...
        "arrivalTimeTrial");
    verifyNotEmpty(testCase,arrivalTimeTrialIndices);
end

function testWrappedBestSoFarRefinementRetainsValidatedDeparture(testCase)
    x=[-0.2;-0.2;0.2;0.2];
    y=[-3;3;3;-3];
    obstacleTimes_s=[0;6;6.5;12];
    yOffsets_units=[0;0;8;8];
    obstacle=obstacleAvoidance.obstacles.createObstacle('translating barrier', ...
        obstacleTimes_s,repmat({x},4,1), ...
        arrayfun(@(offset_units) {y+offset_units},yOffsets_units),0.1);
    initial=restState(0,[-5,0]);
    goal=restState(12,[5,0]);
    limits=struct('xInterval_units',[-180,180], ...
        'yInterval_units',[-3,3],'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[1,1],'maxJerk_units_s3',[2,2]);
    options=struct('GoalTimeMode','earliestArrival','WrapX',true, ...
        'TemporalResolution_s',0.5,'BestSoFarRefinementTrialLimit',1);

    result=planner(obstacle,initial,goal,limits,options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    % Exact polynomial sizing removed a 2.3e-8 s control-bound stretch
    % (10.1400889258125 s before).
    verifyEqual(testCase,result.ArrivalTime_s,10.140088902645401,'AbsTol',1e-8);
    verifyFalse(testCase,isfield(result.Diagnostics, 'FixedArrivalTrialTime_s'));
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.BestSoFar);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s,7.5,'AbsTol',1e-12);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility","arrivalTimeTrial"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Success);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Selected);
    verifyFalse(testCase,result.Diagnostics.Attempts(3).Success);
    verifyFalse(testCase,result.Diagnostics.Attempts(3).Selected);
    verifyEqual(testCase,result.Options.WrapX,"both");
    verifyEqual(testCase,result.Diagnostics.SuppliedLimits,limits);
    verifyEqual(testCase,result.Diagnostics.RequestedLimits.xInterval_units,[-180,180]);
    verifyEqual(testCase,result.Diagnostics.Limits.xInterval_units,[-29,19]);
end

function testPlanarRequestSelectsRefinementOverWrappedBestSoFar(testCase)
    initial=restState(0,[179,0]);
    goal=restState(12,[-179,0]);
    publicLimits=fastLimits([-3,3]);
    publicLimits.xInterval_units=[-180,180];
    base=planner([],initial,goal,publicLimits, ...
        struct('GoalTimeMode','earliestArrival','WrapX',true));

    base.ArrivalTime_s=10;
    base.Diagnostics.Attempts(1).CandidateArrival_s=10;
    base.Diagnostics.Attempts(1).Success=true;
    base.Diagnostics.Attempts(1).Selected=true;
    base.Diagnostics.ElapsedTime_s=0;

    unwrappedOptions=base.Options;
    unwrappedOptions.WrapX=false;
    unwrappedOptions.WrapY=false;
    unwrappedOptions.TemporalResolution_s=6;
    planarGoal=base.Inputs.goalState;
    request=struct( ...
        'initialState',base.Inputs.initialState, ...
        'goalState',planarGoal, ...
        'limits',base.Diagnostics.Limits, ...
        'options',unwrappedOptions);
    % The parent request is the public wrapped request; the saved inputs
    % below belong to its unwrapped copy.
    parentRequest=obstacleAvoidance.planning.createParentRequest( ...
        struct('goalState',goal,'options',base.Options, ...
        'obstacles',{base.Inputs.obstacles}, ...
        'originalInputs',struct( ...
        'suppliedLimits',publicLimits, ...
        'suppliedGoalState',goal, ...
        'requestedLimits',base.Diagnostics.RequestedLimits, ...
        'requestedGoalState',base.Diagnostics.RequestedGoalState)));
    request.obstacles=base.Inputs.obstacles;
    request.parentRequest=parentRequest;
    request.originalInputs=struct( ...
        'suppliedLimits',base.Diagnostics.Limits, ...
        'requestedLimits',base.Diagnostics.Limits, ...
        'suppliedGoalState',planarGoal, ...
        'requestedGoalState',planarGoal);
    snapshot=obstacleAvoidance.obstacles.snapshot( ...
        base.Diagnostics.PreparedObstacles,request.initialState.time_s);
    scene=struct('preparedObstacles',base.Diagnostics.PreparedObstacles, ...
        'vertexVisibility',obstacleAvoidance.search.createVertexVisibility( ...
        snapshot,request.limits,request.options));
    result=obstacleAvoidance.planning.searchArrivalTimes( ...
        request,scene,base,base.Diagnostics.Attempts,1);

    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.Diagnostics.PreparedObstacles,scene.preparedObstacles);
    verifyEqual(testCase,result.Inputs.initialState,request.initialState);
    verifyEqual(testCase,result.Inputs.goalState.position_units, ...
        request.goalState.position_units);
    verifyEqual(testCase,result.Diagnostics.Limits,request.limits);
    verifyEqual(testCase,result.Options.WrapX,"both");
    verifyEqual(testCase,result.Diagnostics.FixedArrivalTrialTime_s,6,'AbsTol',1e-12);
    verifyEqual(testCase,result.ArrivalTime_s,6,'AbsTol',1e-12);
    verifyFalse(testCase,result.Diagnostics.TemporalSearch.BestSoFar);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s,6,'AbsTol',1e-12);
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyFalse(testCase,result.Diagnostics.Attempts(1).Selected);
    verifyEqual(testCase,result.Diagnostics.Attempts(2).TrialTime_s,6,'AbsTol',1e-12);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).Success);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).Selected);
end

function testArrivalSnapshotFindsAnOpeningMissingAtInitialTime(testCase)
    x = [-0.2;0.2;0.2;-0.2];
    bottomY = [-5;-5;1;1];
    bottom = obstacleAvoidance.obstacles.createObstacle('late-moving bottom wall', ...
        [0;12;12.1;30],{x;x;x;x},{bottomY;bottomY;bottomY;bottomY-6},0);
    topY = [3;3;5;5];
    top = obstacleAvoidance.obstacles.createObstacle( ...
        'stationary top wall',0,{x},{topY},0);
    gateY = [1;1;3;3];
    gate = obstacleAvoidance.obstacles.createObstacle('early opening gate', ...
        [0;2;2.1;30],{x;x;x;x},{gateY;gateY;gateY+3;gateY+3},0);
    obstacles = obstacleAvoidance.obstacles.combineObstacles({bottom;top;gate});
    result = planner(obstacles,restState(0,[-5,0]),restState(30,[5,0]), ...
        fastLimits([-5,5]),struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',40));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,4.5+1e-8);
    % The exact timed profile now sees the opening directly; it must not fall
    % through to a fixed-arrival trial merely to rediscover the same clock.
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
end

function testTimedSearchFindsRouteAfterWholeCurtainDeparts(testCase)
    curtainX = [-0.3;0.3;0.3;-0.3];
    curtainY = [-130;-130;130;130];
    curtain = obstacleAvoidance.obstacles.createObstacle('departing curtain', ...
        [0;2;2.1;30],{curtainX;curtainX;curtainX+20;curtainX+20}, ...
        {curtainY;curtainY;curtainY;curtainY},0);
    blockerX = [-0.2;0.2;0.2;-0.2];
    blockerY = [-1;-1;1;1];
    blocker = obstacleAvoidance.obstacles.createObstacle('late direct blocker', ...
        [0;12;12.1;30],{blockerX;blockerX;blockerX;blockerX}, ...
        {blockerY;blockerY;blockerY+11;blockerY+11},0);
    obstacles = obstacleAvoidance.obstacles.combineObstacles({curtain;blocker});
    result = planner(obstacles,restState(0,[-5,0]),restState(30,[5,0]), ...
        fastLimits([-130,130]),struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',40));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,4+1e-8);
    % The timed node set retains the stationary blocker's exact boundary even
    % while the departing curtain hides it inside the all-time moving-cell union.
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
end

function testChallengedDelayedChordBestSoFarIsRetained(testCase)
    box=[-0.6,-1.5;0.6,-1.5;0.6,1.5;-0.6,1.5];
    obstacleTime_s=[0;5.5;6.5;15];
    obstacle=obstacleAvoidance.obstacles.createObstacle('moving box', ...
        obstacleTime_s,{box(:,1);box(:,1);box(:,1);box(:,1)}, ...
        {box(:,2);box(:,2);box(:,2)+4;box(:,2)+4},0.1);
    initial=struct('time_s',0,'position_units',[-5,0]);
    goal=struct('time_s',15,'position_units',[5,0]);
    limits=struct('xInterval_units',[-6,6], ...
        'yInterval_units',[-4,4],'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[0.5,0.5], ...
        'maxJerk_units_s3',[2.5,2.5]);
    options=struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',1);
    result=planner(obstacle,initial,goal,limits,options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'FixedArrivalTrialTime_s'));
end

function testTimedHomotopyPrecedesDelayedDeparture(testCase)
    angle_rad=(0:23).'*(2*pi/24);
    circle_units=[1.5*cos(angle_rad),1.5*sin(angle_rad)];
    obstacleTime_s=[0;5.5;6.5;15];
    centerY_units=[0;0;3;3];
    xByTime_units=repmat({circle_units(:,1)},4,1);
    yByTime_units=arrayfun(@(offset_units) ...
        circle_units(:,2)+offset_units,centerY_units,'UniformOutput',false);
    obstacle=obstacleAvoidance.obstacles.createObstacle('rising circle', ...
        obstacleTime_s,xByTime_units,yByTime_units,0.1);
    initial=struct('time_s',0,'position_units',[-6,0]);
    goal=struct('time_s',15,'position_units',[6,0]);
    limits=struct('maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[1,1],'maxJerk_units_s3',[2,2]);
    options=struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',1);
    result=planner(obstacle,initial,goal,limits,options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    % The direct departure family waits for the rising obstacle and arrives
    % at 10.7597406907 s. A valid curved timed homotopy is physically earlier
    % and must win even though its spatial path is slightly longer.
    verifyLessThan(testCase,result.ArrivalTime_s,10.7597406907);
    % The shortening pass finds a 12.4631-unit polygon for this homotopy, but
    % preparing it stretches the clock to 8.5534113220 s, later than the
    % retained 13.0600-unit motion at 8.5463 s. Arrival wins: the shorter
    % polygon is refused and the retained homotopy stays under 13.5 units.
    verifyLessThan(testCase,result.ArrivalTime_s,8.5534113220);
    verifyFalse(testCase,result.Diagnostics.SolverDiagnostics.TravelRefinementAccepted);
    verifyLessThan(testCase,result.MotionLength_units,13.5);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"timeExpandedVisibilityGraph");
    verifyTrue(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).Selected);
    verifyLessThan(testCase,result.Diagnostics.Attempts(2).CandidateArrival_s, ...
        result.Diagnostics.Attempts(1).CandidateArrival_s);
end

function testHorizonDoesNotMoveSquareGoalClearing(testCase)
    % A square covers the goal and moves off it at 0.5 units/s; the goal
    % clears at 16.3 s. Timed layers sit at horizon / 8, so the goal window
    % starts at 18 s for a 24 s horizon but at 18.75 s for 30 s. Before the
    % unsampled interval was searched, these arrived at 17.0 s and 18.75 s.
    horizons_s = [24, 30];
    arrivals_s = zeros(size(horizons_s));
    for horizonIndex = 1:numel(horizons_s)
        result = planRisingSquare(horizons_s(horizonIndex), struct('GoalTimeMode', 'earliestArrival'));
        verifySelectedValidMotion(testCase, result);
        arrivals_s(horizonIndex) = result.ArrivalTime_s;
    end
    arrivalTolerance_s = result.Options.ArrivalTimeTolerance_s;
    verifyLessThanOrEqual(testCase, abs(diff(arrivals_s)), arrivalTolerance_s);
    verifyLessThanOrEqual(testCase, max(arrivals_s), 17 + arrivalTolerance_s);
end

function testUnsampledIntervalWithoutClocksKeepsTimedMotion(testCase)
    % With a 30 s arrival grid, the interval between the 15 s layer and the
    % 18.75 s goal window holds no grid clock. The search tries nothing and
    % the validated timed motion stays selected.
    result = planRisingSquare(30, struct('GoalTimeMode', 'earliestArrival', ...
        'TemporalResolution_s', 30));
    verifySelectedValidMotion(testCase, result);
    selectedAttempt = result.Diagnostics.Attempts(result.Diagnostics.EarliestArrival.SelectedAttemptIndex);
    verifyEqual(testCase, selectedAttempt.Kind, "timedVisibility");
    verifyEqual(testCase, result.Diagnostics.TemporalSearch.TrialInterval_s, [15, 18.75]);
    verifyEqual(testCase, result.Diagnostics.TemporalSearch.SolverTrialCount, 0);
    verifyGreaterThan(testCase, result.ArrivalTime_s, 18.75 - result.Options.ArrivalTimeTolerance_s);
end

function testHorizonDoesNotMoveDriftingTriangleClearing(testCase)
    % A different shape, direction, and set of limits: a triangle covers the
    % goal and drifts away diagonally. Before the unsampled interval was
    % searched, a 20 s horizon arrived at 17.5 s while 32 s arrived at 16.5 s.
    horizons_s       = [20, 32];
    arrivals_s       = zeros(size(horizons_s));
    triangle_units   = [-2, -1.5; 2, -1.5; 0, 2];
    triangleLimits   = struct('maxVelocity_units_s', [1.5, 1.5], ...
        'maxAcceleration_units_s2', [0.6, 0.6], 'maxJerk_units_s3', [2, 2]);
    for horizonIndex = 1:numel(horizons_s)
        horizon_s       = horizons_s(horizonIndex);
        endCenter_units = [9, 3] + [0.12, 0.1] * horizon_s;
        obstacle        = obstacleAvoidance.obstacles.createObstacle('drifting triangle', [0; horizon_s], ...
            {triangle_units(:, 1) + 9; triangle_units(:, 1) + endCenter_units(1)}, ...
            {triangle_units(:, 2) + 3; triangle_units(:, 2) + endCenter_units(2)}, 0.1);
        result = planner(obstacle, restState(0, [-3, 0]), restState(horizon_s, [9, 3]), ...
            triangleLimits, struct('GoalTimeMode', 'earliestArrival'));
        verifySelectedValidMotion(testCase, result);
        arrivals_s(horizonIndex) = result.ArrivalTime_s;
    end
    arrivalTolerance_s = result.Options.ArrivalTimeTolerance_s;
    verifyLessThanOrEqual(testCase, abs(diff(arrivals_s)), arrivalTolerance_s);
    verifyLessThanOrEqual(testCase, max(arrivals_s), 16.5 + arrivalTolerance_s);
end

function verifySelectedValidMotion(testCase, result)
    % The returned motion must pass the public validator, and exactly one
    % successful attempt, the one that produced it, must be selected.
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    selectedIndices = find([result.Diagnostics.Attempts.Selected]);
    verifyEqual(testCase, numel(selectedIndices), 1);
    verifyEqual(testCase, result.Diagnostics.EarliestArrival.SelectedAttemptIndex, selectedIndices(1));
    selectedAttempt = result.Diagnostics.Attempts(selectedIndices(1));
    verifyTrue(testCase, selectedAttempt.Success);
    verifyEqual(testCase, selectedAttempt.CandidateArrival_s, result.ArrivalTime_s, ...
        'AbsTol', result.Options.ArrivalTimeTolerance_s);
end

function result = planRisingSquare(horizon_s, options)
    % A 4-by-4 square centered on (8, -6) at 0 s moves up at 0.5 units/s
    % and covers the goal (8, 0) until 16.3 s, counting its 0.15 margin.
    square_units = [-2, -2; 2, -2; 2, 2; -2, 2];
    obstacle     = obstacleAvoidance.obstacles.createObstacle('rising square', [0; horizon_s], ...
        {square_units(:, 1) + 8; square_units(:, 1) + 8}, ...
        {square_units(:, 2) - 6; square_units(:, 2) - 6 + 0.5 * horizon_s}, 0.15);
    limits = struct('maxVelocity_units_s', [2, 2], ...
        'maxAcceleration_units_s2', [0.8, 0.8], 'maxJerk_units_s3', [2.5, 2.5]);
    result = planner(obstacle, restState(0, [-8, 0]), restState(horizon_s, [8, 0]), limits, options);
end

function state = restState(time_s,position_units)
    state = struct('time_s',time_s,'position_units',position_units, ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
end

function [request,scene]=explicitSearchInputs(result)
    request=struct( ...
        'initialState',result.Inputs.initialState, ...
        'goalState',result.Inputs.goalState, ...
        'limits',result.Diagnostics.Limits, ...
        'options',result.Options);
    request.obstacles=result.Inputs.obstacles;
    request.parentRequest=[];
    request.originalInputs=struct( ...
        'suppliedLimits',result.Diagnostics.SuppliedLimits, ...
        'requestedLimits',result.Diagnostics.RequestedLimits, ...
        'suppliedGoalState',result.Diagnostics.SuppliedGoalState, ...
        'requestedGoalState',result.Diagnostics.RequestedGoalState);
    % The search hands every trial the initial-snapshot vertex visibility, so the
    % scene carries it exactly as the planner's own scene product does.
    snapshot=obstacleAvoidance.obstacles.snapshot( ...
        result.Diagnostics.PreparedObstacles,result.Inputs.initialState.time_s);
    scene=struct('preparedObstacles',result.Diagnostics.PreparedObstacles, ...
        'vertexVisibility',obstacleAvoidance.search.createVertexVisibility( ...
        snapshot,result.Diagnostics.Limits,result.Options));
end

function limits = fastLimits(yInterval_units)
    limits = struct('xInterval_units',[-6,6], ...
        'yInterval_units',yInterval_units, ...
        'maxVelocity_units_s',[10,10], ...
        'maxAcceleration_units_s2',[10,10], ...
        'maxJerk_units_s3',[10,10]);
end
