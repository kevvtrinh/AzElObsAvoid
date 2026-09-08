function tests = testDepartureScheduling
%% Section 0: Header & Readme
% SYNTAX: tests = testDepartureScheduling
% PURPOSE: Check analytic departures against independent continuous validation.
% INPUTS: None.
% OUTPUTS: Deterministic MATLAB function tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testTranslatingBarrierAndTimeOrigin(testCase)
    [initial,goal,limits,options] = request();
    vertices = [-0.3 -3;0.3 -3;0.3 3;-0.3 3];
    for swap = [false true]
        for origin_s = [0 7]
            times = [0;6;6.5;12]+origin_s;
            positions = {vertices;vertices;vertices+[0 8];vertices+[0 8]};
            start = initial; finish = goal;
            start.time_s = origin_s; finish.time_s = 12+origin_s;
            if swap
                for k=1:numel(positions), positions{k}=fliplr(positions{k}); end
                start.position_units=fliplr(start.position_units);
                finish.position_units=fliplr(finish.position_units);
            end
            x=cell(4,1); y=x;
            for k=1:4, x{k}=positions{k}(:,1); y{k}=positions{k}(:,2); end
            obstacle=obstacleAvoidance.obstacles.createObstacle("barrier",times,x,y,0);
            prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle);
            [candidate,details]=obstacleAvoidance.planner.scheduleDirectWait(prepared,start,finish,limits,options);
            check=obstacleAvoidance.validateTrajectory(candidate,obstacle,start,finish,limits,options);
            verifyTrue(testCase,details.Available);
            verifyTrue(testCase,check.Passed,check.Message);
            verifyGreaterThan(testCase,details.DepartureDelay_s,0);
            verifyLessThan(testCase,candidate.ArrivalTime_s-origin_s,10.2);
            if ~swap && origin_s==0, referenceDuration_s=candidate.TrajectoryDuration_s; end
            verifyEqual(testCase,candidate.TrajectoryDuration_s,referenceDuration_s,'AbsTol',1e-8);
        end
    end
end

function testMultipleIntervalsAndUnsafeWaitingPosition(testCase)
    [initial,goal,limits,options] = request(); goal.time_s=20;
    first=obstacleAvoidance.obstacles.createObstacle("first",[0;6],[-.2;.2;.2;-.2],[-1;-1;1;1],0);
    second=obstacleAvoidance.obstacles.createObstacle("second",[7;11],[2.8;3.2;3.2;2.8],[-1;-1;1;1],0);
    later=obstacleAvoidance.obstacles.createObstacle("later",[15;16],[-.2;.2;.2;-.2],[-1;-1;1;1],0);
    obstacles=[first second later];
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacles);
    [candidate,details]=obstacleAvoidance.planner.scheduleDirectWait(prepared,initial,goal,limits,options);
    check=obstacleAvoidance.validateTrajectory(candidate,obstacles,initial,goal,limits,options);
    verifyTrue(testCase,details.Available);
    verifyTrue(testCase,check.Passed,check.Message);
    verifyGreaterThanOrEqual(testCase,size(details.ForbiddenDepartureInterval_s,1),3);
    verifyLessThan(testCase,candidate.ArrivalTime_s,15);
    occupiedStart=obstacleAvoidance.obstacles.createObstacle("unsafe wait",[1;3],[-5.2;-4.8;-4.8;-5.2],[-.2;-.2;.2;.2],0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles([obstacles occupiedStart]);
    [~,details]=obstacleAvoidance.planner.scheduleDirectWait(prepared,initial,goal,limits,options);
    verifyFalse(testCase,details.Available);
end

function testEligibilityAndHorizonRemainExplicit(testCase)
    [initial,goal,limits,options] = request();
    first=obstacleAvoidance.obstacles.createObstacle("late opening",[0;10],[-.2;.2;.2;-.2],[-1;-1;1;1],0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(first);
    [~,details]=obstacleAvoidance.planner.scheduleDirectWait(prepared,initial,goal,limits,options);
    verifyFalse(testCase,details.Available);
    initial.velocity_units_s=[.1 0];
    [~,details]=obstacleAvoidance.planner.scheduleDirectWait(prepared,initial,goal,limits,options);
    verifyFalse(testCase,details.Available);
    verifyEqual(testCase,details.Reason,"inapplicableDirectClock");
    initial.velocity_units_s=[0 0];
    goal.targetTime_s=[0;12]; goal.targetPosition_units=[4 0;5 0];
    [~,details]=obstacleAvoidance.planner.scheduleDirectWait(prepared,initial,goal,limits,options);
    verifyFalse(testCase,details.Available);
    verifyEqual(testCase,details.Reason,"movingTargetClock");
end

function testPlannerStillChoosesAFasterDetour(testCase)
    [initial,goal,limits,options] = request(); goal.time_s=20;
    initial.position_units=[-3 0]; goal.position_units=[3 0];
    obstacle=obstacleAvoidance.obstacles.createObstacle("detour",[0;12],[-.2;.2;.2;-.2],[-1;-1;1;1],0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    [waiting,details]=obstacleAvoidance.planner.scheduleDirectWait(prepared,initial,goal,limits,options);
    result=obstacleAvoidance.planTrajectory(obstacle,initial,goal,limits,options);
    verifyTrue(testCase,details.Available);
    verifyTrue(testCase,result.Success,result.Message);
    check=obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase,check.Passed,check.Message);
    verifyLessThan(testCase,result.ArrivalTime_s,waiting.ArrivalTime_s);
end

function testScheduledMotionKeepsItsRouteIdentity(testCase)
    [initial,goal,limits,options]=request();
    vertices=[-.3 -3;.3 -3;.3 3;-.3 3];
    x={vertices(:,1);vertices(:,1);vertices(:,1);vertices(:,1)};
    y={vertices(:,2);vertices(:,2);vertices(:,2)+8;vertices(:,2)+8};
    obstacle=obstacleAvoidance.obstacles.createObstacle("route identity",[0;6;6.5;12],x,y,0);
    prepared=obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    seed=obstacleAvoidance.search.createEmptyPathGuess();
    seed.Index=4; seed.Source="directWait";
    seed.position_units=[initial.position_units;initial.position_units;goal.position_units];
    seed.EstimatedDuration_s=11.5; seed.tau=[0;4/11.5;1]; seed.Length_units=10;
    [~,template]=obstacleAvoidance.planner.createPlanningRecord(obstacle,initial,goal,limits,options,obstacleAvoidance.validateTrajectory());
    context=struct('UseStaticSolver',false,'UseStateToStateSolver',false, ...
        'SummaryTemplate',template,'StaticGeometry',struct(),'EnclosureGeometry',struct(),'Enclosure',struct());
    timing=obstacleAvoidance.planner.createStageTiming();
    [candidate,summary]=obstacleAvoidance.planner.solvePathGuess(prepared,initial,goal,limits,options,seed,context,timing);
    verifyTrue(testCase,candidate.Success,candidate.Message);
    verifyTrue(testCase,candidate.SolverDiagnostics.DepartureScheduling.Accepted);
    verifyEqual(testCase,candidate.SeedIndex,4);
    verifyEqual(testCase,summary.SeedIndex,4);
    verifyEqual(testCase,candidate.SeedSource,"directWait");
end

function [initial,goal,limits,options] = request()
    initial=struct('time_s',0,'position_units',[-5 0], ...
        'velocity_units_s',[0 0],'acceleration_units_s2',[0 0]);
    goal=initial; goal.time_s=12; goal.position_units=[5 0];
    limits=struct('maxVelocity_units_s',[2 2],'maxAcceleration_units_s2',[1 1], ...
        'maxJerk_units_s3',[2 2],'xInterval_units',[-20 20],'yInterval_units',[-20 20]);
    options=obstacleAvoidance.input.resolvePlannerOptions();
end
