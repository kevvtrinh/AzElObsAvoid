function tests = testPlannerDecisionFlow
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testPlannerDecisionFlow.m')
% PURPOSE: Exercise each public planner routing and proven-rejection decision.
% INPUTS: MATLAB unit test framework and deterministic public planner inputs.
% OUTPUTS: Branch-signature, stable outcome, and independent-validation checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testInitialEndpointBlocked(testCase)
    obstacle=struct('Vertices_units',[-5,-1;-3,-1;-3,1;-5,1]);
    result=planner(obstacle,state(0,[-4,0]),state(10,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"endpointBlocked");
end

function testTerminalReachabilityBlocked(testCase)
    box=[-2,-2;2,-2;2,2;-2,2];
    obstacle=obstacleAvoidance.obstacles.createObstacle('terminal box',[0;9], ...
        {box(:,1);box(:,1)},{box(:,2);box(:,2)},0);
    limits=standardLimits();
    limits.xInterval_units=[-12,12];
    result=planner(obstacle,state(0,[-10,0]),state(10,[0,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"terminalReachabilityBlocked");
end

function testEndpointDerivativeLimit(testCase)
    initial=state(0,[-4,0]);
    initial.velocity_units_s=[3,0];
    result=planner([],initial,state(10,[4,0]),standardLimits(), ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"dynamicEndpointInfeasible");
end

function testEndpointOutsideWorkspace(testCase)
    result=planner([],state(0,[-7,0]),state(10,[4,0]),standardLimits(), ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"endpointOutsideWorkspace");
end

function testTimeWindowInfeasible(testCase)
    limits=standardLimits();
    limits.maxVelocity_units_s=[1,1];
    result=planner([],state(0,[-5,0]),state(1,[5,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"timeWindowInfeasible");
end

function testPeriodicWrapUsesNearestImage(testCase)
    limits=standardLimits();
    limits.xInterval_units=[-180,180];
    initial=state(0,[179,0]);
    goal=state(10,[-179,0]);
    result=planner([],initial,goal,limits, ...
        struct('GoalTimeMode','fixedArrival','WrapX',true));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Inputs.goalState.position_units,[181,0]);
    verifyEqual(testCase,result.RequestedGoalState.position_units,[-179,0]);
    verifyEqual(testCase,result.MotionLength_units,2,'AbsTol',1e-8);
end

function testPeriodicRequestWithObstacleIsRejected(testCase)
    obstacle=struct('Vertices_units',[-1,-1;1,-1;1,1;-1,1]);
    call=@() planner(obstacle,state(0,[179,0]),state(10,[-179,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival','WrapX',true));
    verifyError(testCase,call,'planner:UnsupportedPeriodicRequest');
end

function testTimedMovingTargetIsExplicitlyUnsupported(testCase)
    targetMotion=struct('time_s',[0;10], ...
        'position_units',[4,0;5,0],'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    result=planner([],state(0,[0,0]),goal,standardLimits(), ...
        struct('GoalTimeMode','fixedArrival','FixedArrivalSearch','timeExpanded'));
    verifyFailure(testCase,result,"unsupportedTimedRequest");
end

function testInitiallyOccupiedFutureGoalUsesTemporalSeed(testCase)
    local=[-0.8,-0.8;0.8,-0.8;0.8,0.8;-0.8,0.8];
    first=local+[4,0];
    last=local+[4,6];
    obstacle=obstacleAvoidance.obstacles.createObstacle('departing goal box',[0;10], ...
        {first(:,1);last(:,1)},{first(:,2);last(:,2)},0);
    result=planner(obstacle,state(0,[-4,0]),state(10,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"temporalDirectSeed");
end

function testDisconnectedDynamicSnapshotUsesTemporalSeed(testCase)
    wall=[-0.2,-7;0.2,-7;0.2,7;-0.2,7];
    moved=wall+[0,14];
    obstacle=obstacleAvoidance.obstacles.createObstacle('departing wall',[0;5], ...
        {wall(:,1);moved(:,1)},{wall(:,2);moved(:,2)},0);
    result=planner(obstacle,state(0,[-4,0]),state(12,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"temporalDirectSeed");
end

function testSparseDynamicZeroWaitDeparture(testCase)
    box=[-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    first=box+[0,8];
    last=box+[2,8];
    obstacle=obstacleAvoidance.obstacles.createObstacle('remote mover',[0;20], ...
        {first(:,1);last(:,1)},{first(:,2);last(:,2)},0);
    result=planner(obstacle,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyEqual(testCase,result.SolverDiagnostics.Identifier,"c3JerkLimitedChord");
    verifyFalse(testCase,isfield(result.SolverDiagnostics,'DepartureSchedule'));
    verifyFalse(testCase,isfield(result,'TemporalSearch'));
end

function testArrivalSearchExhausted(testCase)
    targetMotion=struct('time_s',[0;1], ...
        'position_units',[100,0;101,0],'InterpolationMethod','linear');
    goal=struct('time_s',1,'targetMotion',targetMotion);
    result=planner([],state(0,[0,0]),goal,standardLimits(), ...
        struct('GoalTimeMode','earliestArrival','TemporalResolution_s',0.5));
    verifyFailure(testCase,result,"arrivalSearchExhausted");
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyGreaterThan(testCase,numel(result.TemporalSearch.TrialTime_s),0);
end

function value=state(time_s,position_units)
    value=struct('time_s',time_s,'position_units',position_units, ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
end

function limits=standardLimits()
    limits=struct('xInterval_units',[-6,6],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
end

function verifyFailure(testCase,result,reason)
    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,reason);
    verifyEmpty(testCase,result.time_s);
end
