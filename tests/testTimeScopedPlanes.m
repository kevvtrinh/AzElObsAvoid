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

function testTravelRefinementAddsNewCollisionPlanes(testCase)
    box = [-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    waypoints = [-3,0;-3,3;3,3;3,0];
    controls = zeros(3,9,2);
    for span = 1:3
        points = [repmat(waypoints(span,:),4,1); ...
            mean(waypoints(span:span+1,:),1);repmat(waypoints(span+1,:),4,1)];
        controls(span,:,:) = reshape(points,1,9,2);
    end
    plane = struct('Active',false,'Verified',false,'ExitFlag',NaN, ...
        'Normal',zeros(2),'Offset_units',zeros(1,2),'SignedGap_units',NaN,'TimeFraction',[0,1]);
    initial = struct('time_s',0,'position_units',waypoints(1,:));
    goal = struct('time_s',12,'position_units',waypoints(end,:));
    limits = struct('xInterval_units',[-10,10],'yInterval_units',[-10,10], ...
        'maxVelocity_units_s',[5,5],'maxAcceleration_units_s2',[10,10],'maxJerk_units_s3',[20,20]);
    request = struct('Degree',8,'InitialState',initial,'GoalState',goal,'Limits',limits, ...
        'Regions_units',{{box}},'RegionMinimum_units',min(box),'RegionMaximum_units',max(box), ...
        'MotionHorizon_s',12,'Options',struct('GoalTimeMode',"fixedArrival"),'Coverage',struct('Passed',true));
    warmStart = struct('SegmentCount',3,'RegionActiveBySegment',true(3,1));
    alternating = struct('ControlPoint_units',controls,'SegmentTime_s',4, ...
        'Planes',repmat(plane,3,1),'TaggedPairs',false(3,1));
    diagnostics = struct('ConicSolver',bmtpEngine.accumulateConicDiagnostics());
    [refined,diagnostics] = bmtpEngine.refineTimedTravel(request,warmStart,alternating,diagnostics,1e-5,1e-8);
    verifyTrue(testCase,diagnostics.TravelRefinementAccepted);
    verifyGreaterThan(testCase,diagnostics.TaggedPairCount,0);
    verifyLessThan(testCase,diagnostics.TravelRefinementFinalLength_units,diagnostics.TravelRefinementInitialLength_units);
    prepared = struct('CertifiedControlPoint_units',refined.ControlPoint_units, ...
        'SegmentTime_s',repmat(refined.SegmentTime_s,3,1));
    verifyTrue(testCase,bmtpEngine.checkFinalMotion(request,warmStart,prepared,1e-8,1e-5).Passed);
end

function testSingleSpanTimedInfeasibilityReturnsNoMotion(testCase)
    limits = struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[5,5],'maxJerk_units_s3',[10,10]);
    options = optimoptions('coneprog','Display','none','ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    planes = repmat(struct('Active',false),1,0);
    % Two units in 0.1 s exceeds the two-units/s velocity bound alone.
    [controls,duration_s,flag] = bmtpEngine.solveTimedTrajectoryStep( ...
        1,8,[-1,0],[1,0.5],limits,planes,1e-8,0.1,"fixedArrival",options);
    verifyLessThanOrEqual(testCase,flag,0);
    verifyEmpty(testCase,controls);
    verifyTrue(testCase,isnan(duration_s));
end

function testSingleSpanTimedMotionIsIndependentlyValid(testCase)
    limits = struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[5,5],'maxJerk_units_s3',[10,10]);
    options = optimoptions('coneprog','Display','none','ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    planes = repmat(struct('Active',false),1,0);
    for degree = [5,8]
        for horizon_s = [5,6,8,10]
            [controls,duration_s,flag,output] = bmtpEngine.solveTimedTrajectoryStep( ...
                1,degree,[-1,0],[1,0.5],limits,planes,1e-8,horizon_s,"fixedArrival",options);
            assertNotEmpty(testCase,controls);
            verifyTrue(testCase,flag>0 || flag==-7);
            verifyEqual(testCase,output.OptimizationConverged,flag>0);
            verifyEqual(testCase,duration_s,horizon_s,'AbsTol',1e-12);
            initial = struct('time_s',0,'position_units',[-1,0]);
            goal = struct('time_s',horizon_s,'position_units',[1,0.5]);
            result = planner([],initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
            seed = struct('position_units',[-1,0;1,0.5],'tau',[0;1],'Source',"timeExpandedVisibilityGraph");
            request = bmtpEngine.createSolveRequest(seed,cell(0,1),struct('Passed',true), ...
                result.Inputs.initialState,result.Inputs.goalState,result.Limits,result.Options);
            prepared = bmtpEngine.prepareFinalMotion(request,controls,duration_s);
            [~,~,reserve_units] = bmtpEngine.createCoordinateTolerances(controls,limits.xInterval_units,limits.yInterval_units);
            target_units = (1+2^20*eps)*result.Options.CollisionClearanceTolerance_units+reserve_units;
            result = bmtpEngine.createMotionOutput(result,request,prepared);
            result.PlaneCertificate = bmtpEngine.checkFinalMotion(request,[],prepared,reserve_units,target_units);
            assertTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
        end
    end
end

function testUnequalSpanClockWithFullEndpointStates(testCase)
    initial = struct('position_units',[-1,0],'velocity_units_s',[0.1,-0.1],'acceleration_units_s2',[0.02,0.01]);
    goal = struct('position_units',[1,0.4],'velocity_units_s',[0.2,0.05],'acceleration_units_s2',[-0.01,0.02]);
    limits = struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[5,5],'maxJerk_units_s3',[10,10]);
    options = optimoptions('coneprog','Display','none','ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    planes = repmat(struct('Active',false),3,0);
    [controls,durations_s,flag] = bmtpEngine.solveTrajectoryStep( ...
        3,5,initial,goal,limits,planes,1e-8,8,options,[1,2,1],true);
    assertTrue(testCase,flag>0 || flag==-7);
    verifyEqual(testCase,durations_s,[2,4,2]);
    polynomial = bmtpEngine.createPowerPolynomial(controls,durations_s,0);
    [~,position,velocity,acceleration] = bmtpEngine.evaluatePolynomial(polynomial,[0;8]);
    verifyEqual(testCase,position,[initial.position_units;goal.position_units],'AbsTol',1e-8);
    verifyEqual(testCase,velocity,[initial.velocity_units_s;goal.velocity_units_s],'AbsTol',1e-8);
    verifyEqual(testCase,acceleration,[initial.acceleration_units_s2;goal.acceleration_units_s2],'AbsTol',1e-8);
    [~,~,~,~,leftJerk] = bmtpEngine.evaluatePolynomial(polynomial,[2;6],[1;2]);
    [~,~,~,~,rightJerk] = bmtpEngine.evaluatePolynomial(polynomial,[2;6],[2;3]);
    verifyEqual(testCase,leftJerk,rightJerk,'AbsTol',1e-8);
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

function testMixedStaticObstacleLifetimes(testCase)
    movingBox = [-0.2,-3;0.2,-3;0.2,3;-0.2,3];
    moving = obstacleAvoidance.obstacles.createObstacle('moving',[0;12], ...
        {movingBox(:,1);movingBox(:,1)},{movingBox(:,2);movingBox(:,2)+8},0.1);
    fixedBox = [0.5,-1.25;1.5,-1.25;1.5,1.25;0.5,1.25];
    nodes = [-5,0;5,0;-2,0;2,0;-2,2;2,2];
    cost = hypot(nodes(:,1)-nodes(:,1).',nodes(:,2)-nodes(:,2).');
    histories = {0,[0;12],[3;6],[0;12]};
    for variant = 1:numel(histories)
        sourceTimes_s = histories{variant};
        x_units = repmat({fixedBox(:,1)},numel(sourceTimes_s),1);
        y_units = repmat({fixedBox(:,2)},numel(sourceTimes_s),1);
        % Near-equal source boundaries must still be treated as moving.
        if variant == 4, x_units{end} = x_units{end}+1e-12; end
        fixed = obstacleAvoidance.obstacles.createObstacle('stationary',sourceTimes_s,x_units,y_units,0);
        for reversed = [false,true]
            sources = {moving,fixed};
            if reversed, sources = fliplr(sources); end
            obstacles = obstacleAvoidance.obstacles.combineObstacles(sources);
            [route_units,routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
                nodes,cost,obstacles,struct('time_s',0),struct('time_s',12), ...
                struct('maxVelocity_units_s',[2,2]),unique([(0:0.25:12).';sourceTimes_s]), ...
                struct('GoalTimeMode',"earliestArrival"));
            assertNotEmpty(testCase,routeTime_s);
            if variant == 3
                % Once the stationary obstacle disappears, the direct edge opens.
                verifyEqual(testCase,routeTime_s(end-1:end),[3;8.25]);
                verifyEqual(testCase,route_units(end-1:end,:),nodes(1:2,:));
            else
                verifyEqual(testCase,routeTime_s(end-3:end),[3.75;5.25;7.25;8.75]);
                verifyEqual(testCase,route_units(end-3:end,:),nodes([1,3,6,2],:));
            end
        end
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
