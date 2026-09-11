function tests = testGoalVisibilityWindows
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testGoalVisibilityWindows.m')
% PURPOSE: Verify wait guides when a goal is free, hidden, then free again.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Input-derived goal-window and near-goal staging checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testReachableFirstWindowAddsOnlyItsSafeGoalWait(testCase)
    blocker=createGoalBlocker(4,3);
    nodes_units=[0,0;4,0];
    costs_units=[0,4;4,0];
    initial=struct('time_s',0,'position_units',nodes_units(1,:));
    goal=struct('time_s',10,'position_units',nodes_units(2,:));
    limits=struct('maxVelocity_units_s',[4,4]);
    options=struct('GoalTimeMode',"earliestArrival");
    [route_units,routeTime_s,record]=obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodes_units,costs_units,blocker,initial,goal,limits,(0:10).',options);
    verifyEqual(testCase,routeTime_s,[0;1]);
    verifyEqual(testCase,route_units,[0,0;4,0],'AbsTol',1e-12);
    verifyEqual(testCase,record.WaitRouteTime_s,[0;1;2]);
    verifyEqual(testCase,record.WaitRoute_units,[0,0;4,0;4,0],'AbsTol',1e-12);
    verifyLessThan(testCase,routeTime_s(end),goal.time_s);
end

function testMissedFirstWindowWaitsAtNearestSafeNode(testCase)
    blocker=createGoalBlocker(10,2);
    nodes_units=[0,0;10,0;8,0];
    costs_units=Inf(3);
    costs_units(1:4:end)=0;
    costs_units(1,3)=8; costs_units(3,1)=8;
    costs_units(2,3)=2; costs_units(3,2)=2;
    initial=struct('time_s',0,'position_units',nodes_units(1,:));
    goal=struct('time_s',10,'position_units',nodes_units(2,:));
    limits=struct('maxVelocity_units_s',[8,8]);
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

function blocker=createGoalBlocker(goalX_units,blockStart_s)
    vertices_units=[goalX_units-0.5,-0.5;goalX_units+0.5,-0.5; ...
        goalX_units+0.5,0.5;goalX_units-0.5,0.5];
    blocker=obstacleAvoidance.obstacles.createObstacle('temporary goal blocker', ...
        [blockStart_s;7],{vertices_units(:,1);vertices_units(:,1)}, ...
        {vertices_units(:,2);vertices_units(:,2)},0);
end
