function tests = testArrivalSearchRegressions
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testArrivalSearchRegressions.m')
% PURPOSE: Exercise late feasible arrivals, free-clock selection against
%   waiting motions, and chronological fixed-arrival recovery.
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
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyLessThan(testCase,result.ArrivalTime_s, ...
        result.VisibilityGraph.RouteTime_s(end));
    verifyEqual(testCase,result.SeedSource,"timeExpandedVisibilityGraph");
    verifyFalse(testCase,isfield(result.SolverDiagnostics, ...
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
    % arrival, then the free goal waits to the prescribed horizon.
    verifyGreaterThanOrEqual(testCase,nnz(goalRows),2);
    verifyEqual(testCase,routeTime_s(find(goalRows,1,'first')),1,'AbsTol',1e-12);
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

function testDepartureFamilyDoesNotDependOnSeedLabel(testCase)
    % The delayed-chord construction is selected from the physical request
    % (direct rest-to-rest earliest with moving cells and no timed guide),
    % never from the diagnostic seed label.
    obstacleTime_s = [0;6;6.5;12];
    barrier_units = [-0.2,-3;0.2,-3;0.2,3;-0.2,3];
    shifts_units = [0;0;8;8];
    obstacle = obstacleAvoidance.obstacles.createObstacle('translating barrier', ...
        obstacleTime_s,repmat({barrier_units(:,1)},4,1), ...
        arrayfun(@(s)barrier_units(:,2)+s,shifts_units,'UniformOutput',false),0.1);
    initial = struct('time_s',0,'position_units',[-5,0]);
    goal = struct('time_s',12,'position_units',[5,0]);
    limits = struct('xInterval_units',[-6,6],'yInterval_units',[-3,3], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[1,1],'maxJerk_units_s3',[2,2]);
    normalized = planner([],initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,12]);
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared,0,12);
    coverage = struct('Passed',true,'ExactRegionCount',numel(cells.Regions_units), ...
        'ActiveTimeInterval_s',cells.ActiveTimeInterval_s, ...
        'EndRegions_units',{cells.EndRegions_units});
    route_units = [initial.position_units;goal.position_units];
    results = cell(2,1);
    diagnostics = cell(2,1);
    labels = ["departureSchedule","unrelatedDiagnosticLabel"];
    for k = 1:2
        seed = struct('position_units',route_units,'tau',[0;1],'Source',labels(k));
        [results{k},diagnostics{k}] = bmtpEngine.solve(seed,cells.Regions_units, ...
            coverage,normalized.Inputs.initialState,normalized.Inputs.goalState, ...
            normalized.Limits,normalized.Options);
    end
    verifyTrue(testCase,results{1}.Success,results{1}.Message);
    verifyTrue(testCase,results{2}.Success,results{2}.Message);
    verifyEqual(testCase,results{2}.ArrivalTime_s,results{1}.ArrivalTime_s,'AbsTol',0);
    verifyEqual(testCase,diagnostics{2}.Identifier,"c3DepartureSchedule");
    verifyEqual(testCase,results{2}.position_units,results{1}.position_units,'AbsTol',0);
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
    verifyGreaterThan(testCase,result.TemporalSearch.NecessaryArrivalBound_s,60);
    verifyGreaterThanOrEqual(testCase,result.TemporalSearch.TrialTime_s, ...
        result.TemporalSearch.NecessaryArrivalBound_s-result.Options.ArrivalTimeTolerance_s);
    verifyLessThanOrEqual(testCase,numel(result.TemporalSearch.TrialTime_s),40);
    % The exact static-only reference arrives at 64 s. The moving obstacle is
    % remote, so the unified timed profile must stay within the 1% gate.
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,64.64);
end

function testDisconnectedSnapshotRetainsValidatedWaitHonestly(testCase)
    result = exampleMovingBarrierWait(struct('PlotOutputs',false,'Verbose',false,'MaxArrivalTrials',1));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,isfield(result,'TemporalSearch'));
    verifyFalse(testCase,isfield(result.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyGreaterThan(testCase,result.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
    verifyFalse(testCase,isfield(result.SolverDiagnostics.DepartureSchedule, ...
        'InitialRouteTimeBound_s'));
    verifyFalse(testCase,isfield(result,'FixedArrivalTrialTime_s'));
end

function testMovingGeometryDoesNotUseInitialRouteAsGlobalBound(testCase)
    result = exampleOpeningUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyFalse(testCase,isfield(result,'TemporalSearch'));
    verifyFalse(testCase,isfield(result.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyFalse(testCase,isfield(result.SolverDiagnostics.DepartureSchedule, ...
        'InitialRouteTimeBound_s'));
    verifyGreaterThan(testCase,result.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
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
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyEqual(testCase,result.VisibilityGraph.TimedSearch.TimedSearch. ...
        DynamicEdgeCheckKind,"exactAffineConvexCells");
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
    % while the departing curtain hides it inside the all-time swept union.
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
end

function testChallengedDelayedChordIncumbentIsRetained(testCase)
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
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,isfield(result,'TemporalSearch'));
    verifyFalse(testCase,isfield(result.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyFalse(testCase,isfield(result,'FixedArrivalTrialTime_s'));
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
    verifyLessThan(testCase,result.MotionLength_units,12.6);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"timeExpandedVisibilityGraph");
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyFalse(testCase,isfield(result.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
end

function state = restState(time_s,position_units)
    state = struct('time_s',time_s,'position_units',position_units, ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
end

function limits = fastLimits(yInterval_units)
    limits = struct('xInterval_units',[-6,6], ...
        'yInterval_units',yInterval_units, ...
        'maxVelocity_units_s',[10,10], ...
        'maxAcceleration_units_s2',[10,10], ...
        'maxJerk_units_s3',[10,10]);
end
