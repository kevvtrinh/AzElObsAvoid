function tests = testTimeScopedPlanes
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testTimeScopedPlanes.m')
%**************************************************************************
% PURPOSE
%   - Check timed separating lines, trajectory constraints, moving detours,
%     endpoint states, and earliest-arrival behavior.
%**************************************************************************
% INPUTS
%   - MATLAB unit test framework runs the local test functions.
%**************************************************************************
% OUTPUTS
%   - tests (function test array)
%       Tests of timed motion and its independent checks.
%**************************************************************************
% UNITS
%   - Position uses coordinate units and time uses seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testMovingDetourWithNonzeroEndpointVelocity(testCase)
    times_s = (0:0.25:20)';
    box = [-0.5,-0.7;0.5,-0.7;0.5,0.7;-0.5,0.7];
    source = obstacleAvoidance.obstacles.createObstacle('moving rectangle',times_s, ...
        repmat({box(:,1)},numel(times_s),1), ...
        arrayfun(@(t)box(:,2)-2+0.2*t,times_s,'UniformOutput',false),0.1);
    initial = struct('time_s',0,'position_units',[-4,0],'velocity_units_s',[0.15,0]);
    goal = struct('time_s',20,'position_units',[4,0],'velocity_units_s',[0.1,0]);
    result = planner(source,initial,goal,[],struct( ...
        'GoalTimeMode','fixedArrival'));
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,20,'AbsTol',1e-8);
    verifyEqual(testCase,result.velocity_units_s([1,end],:),[0.15,0;0.1,0],'AbsTol',1e-8);
    verifyLessThan(testCase,result.Diagnostics.Polynomial.SegmentCount,80);
    % Eighty identical-velocity source intervals are one exact affine cell.
    verifyEqual(testCase,result.Diagnostics.SeparationProof.SolverRegionCount,1);
    verifyGreaterThan(testCase,result.Diagnostics.SolverDiagnostics.TrajectorySocpCount,0);
end

function testTimedSearchMovingAndStationaryIntervals(testCase)
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

function testMixedStaticObstacleLifetimes(testCase)
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
        if variant == 4, x_units{end} = x_units{end}+1e-12; end
        fixed = obstacleAvoidance.obstacles.createObstacle('stationary',sourceTimes_s,x_units,y_units,0);
        for reversed = [false,true]
            sources = {moving,fixed};
            if reversed, sources = fliplr(sources); end
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
