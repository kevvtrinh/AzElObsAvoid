function tests = testTimeScopedPlanes
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testTimeScopedPlanes.m')
% PURPOSE: Verify constraints act on their physical time intervals, and
%          moving detours preserve endpoint states and dense-history
%          earliest-arrival motions use fewer active than applicable pairs.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Behavioral constraint, collision, and endpoint regressions.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testDifferentTimeWindowsDoNotConflict(testCase)
    initial = struct('time_s',0,'position_units',[-1,0],'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal = initial; goal.time_s = 6; goal.position_units = [1,0];
    limits = struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[5,5],'maxJerk_units_s3',[10,10]);
    planes = repmat(struct('Active',true,'Normal',[1,0;1,0],'Offset_units',[0.5,0.5], ...
        'TimeFraction',[0,0.1]),1,2);
    planes(2).Normal = -planes(1).Normal; planes(2).TimeFraction = [0.9,1];
    options = optimoptions('coneprog','Display','none','ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    [controls,durations,flag,output] = bmtpEngine.solveTrajectoryStep(1,5,initial,goal,limits,planes,1e-8,6,options,1,true);
    verifyTrue(testCase,flag>0 || flag==-7);
    verifyEqual(testCase,durations,6);
    verifyLessThan(testCase,output.MaximumClearanceSlack_units,1e-6);
    first = bmtpEngine.restrictBezier(squeeze(controls),[0,0.1]);
    last = bmtpEngine.restrictBezier(squeeze(controls),[0.9,1]);
    verifyLessThan(testCase,max(first(:,1)),-0.5);
    verifyGreaterThan(testCase,min(last(:,1)),0.5);
    % The same two exclusions overlap in time and are genuinely incompatible.
    planes(1).TimeFraction = [0,1]; planes(2).TimeFraction = [0,1];
    [~,~,flag,output] = bmtpEngine.solveTrajectoryStep(1,5,initial,goal,limits,planes,1e-8,6,options,1,true);
    verifyTrue(testCase,flag<=0 || output.MaximumClearanceSlack_units>0.4);
end

function testMovingDetourWithNonzeroEndpointVelocity(testCase)
    times_s = (0:0.25:20)';
    box = [-0.5,-0.7;0.5,-0.7;0.5,0.7;-0.5,0.7];
    source = obstacleAvoidance.obstacles.createObstacle('moving rectangle',times_s, ...
        repmat({box(:,1)},numel(times_s),1), ...
        arrayfun(@(t)box(:,2)-2+0.2*t,times_s,'UniformOutput',false),0.1);
    initial = struct('time_s',0,'position_units',[-4,0],'velocity_units_s',[0.15,0]);
    goal = struct('time_s',20,'position_units',[4,0],'velocity_units_s',[0.1,0]);
    result = planner(source,initial,goal,[],struct('GoalTimeMode','fixedArrival'));
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,20,'AbsTol',1e-8);
    verifyEqual(testCase,result.velocity_units_s([1,end],:),[0.15,0;0.1,0],'AbsTol',1e-8);
    verifyLessThan(testCase,result.Polynomial.SegmentCount,80);
    verifyEqual(testCase,result.PlaneCertificate.SolverRegionCount,80);
    verifyGreaterThan(testCase,result.SolverDiagnostics.TrajectorySocpCount,0);
end

function testTimedSearchMovingAndStationaryIntervals(testCase)
    box = [-0.2,-3;0.2,-3;0.2,3;-0.2,3];
    nodes = [-5,0;5,0;-2,0;2,0;-2,2;2,2];
    cost = hypot(nodes(:,1)-nodes(:,1).',nodes(:,2)-nodes(:,2).');
    for hasStationaryInterval = [false,true]
        if hasStationaryInterval
            sourceTimes_s = [0;6;6.5;12]; shifts_units = [0;0;8;8];
            expectedDeparture_s = 3.75; expectedArrival_s = 8.75;
        else
            sourceTimes_s = [0;12]; shifts_units = [0;8];
            expectedDeparture_s = 2.25; expectedArrival_s = 7.25;
        end
        obstacle = obstacleAvoidance.obstacles.createObstacle('crossing barrier',sourceTimes_s, ...
            repmat({box(:,1)},numel(sourceTimes_s),1), ...
            arrayfun(@(shift)box(:,2)+shift,shifts_units,'UniformOutput',false),0.1);
        % Search must extend partial preparation before reusing its snapshot.
        obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,0]);
        [route_units,routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
            nodes,cost,obstacle,struct('time_s',0),struct('time_s',12), ...
            struct('maxVelocity_units_s',[2,2]),unique([(0:0.25:12).';sourceTimes_s]), ...
            struct('GoalTimeMode',"earliestArrival"));
        assertNotEmpty(testCase,routeTime_s);
        verifyEqual(testCase,routeTime_s(end-1:end),[expectedDeparture_s;expectedArrival_s]);
        verifyEqual(testCase,route_units,[repmat(nodes(1,:),numel(routeTime_s)-1,1);nodes(2,:)]);
        % Search owns preparation, including stale caches on edited sources.
        changed = obstacle;
        changed.x_units = cellfun(@(x)x+20,changed.x_units,'UniformOutput',false);
        changed.originalX_units = cellfun(@(x)x+20,changed.originalX_units,'UniformOutput',false);
        [~,changedTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
            nodes,cost,changed,struct('time_s',0),struct('time_s',12), ...
            struct('maxVelocity_units_s',[2,2]),unique([(0:0.25:12).';sourceTimes_s]), ...
            struct('GoalTimeMode',"earliestArrival"));
        verifyEqual(testCase,changedTime_s(end),5);
    end
end

function testSavedMovingDetourEarliestArrival(testCase)
    root = fileparts(mfilename('fullpath'));
    request = jsondecode(fileread(fullfile(root,'fixtures','savedMovingDetour.json')));
    sources = cell(numel(request.obstacles),1);
    for k = 1:numel(sources)
        source = request.obstacles(k); frames = source.keyframes;
        sources{k} = obstacleAvoidance.obstacles.createObstacle(source.name,[frames.time_s].', ...
            arrayfun(@(f)f.vertices_units(:,1),frames,'UniformOutput',false), ...
            arrayfun(@(f)f.vertices_units(:,2),frames,'UniformOutput',false),source.safetyMargin_units);
    end
    obstacles = obstacleAvoidance.obstacles.combineObstacles(sources);
    options = request.options; options.GoalTimeMode = 'earliestArrival';
    result = planner(obstacles,request.initialState,request.goalState,request.limits,options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,117,'AbsTol',1e-8);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"timeExpandedVisibilityGraph");
    verifyLessThan(testCase,result.SolverDiagnostics.TaggedPairCount, ...
        result.SolverDiagnostics.ApplicablePairCount);
end
