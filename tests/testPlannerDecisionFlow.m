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

function testWorkspaceBoundaryDerivativeIsRejectedBeforePlanning(testCase)
    limits=standardLimits(); limits.xInterval_units=[-1,1];
    initial=state(0,[1,0]); initial.velocity_units_s=[1,0];
    result=planner([],initial,state(10,[0,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"dynamicEndpointInfeasible");
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"notSearched");

    goal=state(10,[-1,0]); goal.velocity_units_s=[1,0];
    result=planner([],state(0,[0,0]),goal,limits, ...
        struct('GoalTimeMode','fixedArrival','FixedArrivalSearch','timeExpanded'));
    verifyFailure(testCase,result,"dynamicEndpointInfeasible");
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"notSearched");
end

function testWorkspaceOvershootFallsThroughToBmtp(testCase)
    limits=standardLimits(); limits.xInterval_units=[-1,1];
    initial=state(0,[0.5,0]);
    initial.velocity_units_s=[0.3,0];
    result=planner([],initial,state(10,[0,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.SolverDiagnostics.Identifier,"bmtpStaticDegree5");
    verifyLessThanOrEqual(testCase,max(result.position_units(:,1)),1);
end

function testPassingAnalyticProbeStillChecksEveryCollisionPair(testCase)
    rectangle=[4,20;6,20;6,22;4,22];
    obstacle=obstacleAvoidance.obstacles.createObstacle( ...
        'distant translating rectangle',[0;5;10], ...
        {rectangle(:,1);rectangle(:,1);rectangle(:,1)}, ...
        {rectangle(:,2);rectangle(:,2)+1;rectangle(:,2)+2},0.1);
    limits=standardLimits();
    limits.xInterval_units=[-5,15]; limits.yInterval_units=[-5,30];
    result=planner(obstacle,state(0,[0,0]),state(10,[10,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.SolverDiagnostics.Identifier,"minimumJerkQuintic");
    verifyGreaterThan(testCase,result.PlaneCertificate.AllPairCount,0);
    verifyEqual(testCase,result.PlaneCertificate.VerifiedPairCount, ...
        result.PlaneCertificate.AllPairCount);
end

function testTimeWindowInfeasible(testCase)
    limits=standardLimits();
    limits.maxVelocity_units_s=[1,1];
    result=planner([],state(0,[-5,0]),state(1,[5,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"timeWindowInfeasible");
end

function testEarliestStaticDirectUsesAnalyticClock(testCase)
    result=planner([],state(0,[0,0]),state(10,[4,1]),standardLimits(), ...
        struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,result.VisibilityGraph.GraphIsFullyEnumerated);
    verifyEqual(testCase,result.SolverDiagnostics.Identifier,"c3JerkLimitedChord");
    verifyEqual(testCase,result.SolverDiagnostics.ConstraintRepresentation,"analyticC3Clock");
end

function testEarliestStaticNonrestUsesPhysicalClockTrials(testCase)
    initial=state(0,[0,0]); initial.velocity_units_s=[0.1,0];
    result=planner([],initial,state(10,[4,0]),standardLimits(), ...
        struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,result.ArrivalTime_s, ...
        'AbsTol',1e-12);
    verifyFalse(testCase,result.TemporalSearch.GlobalEarliestProven);
end

function testFixedMovingTargetMatchesPchipDerivatives(testCase)
    targetMotion=struct('time_s',[0;5;10], ...
        'position_units',[2,0;3,0;4,0], ...
        'InterpolationMethod','pchip');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    options=struct('GoalTimeMode','fixedArrival', ...
        'MatchTargetVelocity',true,'MatchTargetAcceleration',true);
    result=planner([],state(0,[0,0]),goal,standardLimits(),options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Intercept.Time_s,10,'AbsTol',1e-10);
    verifyEqual(testCase,result.Intercept.TargetVelocity_units_s,[0.2,0], ...
        'AbsTol',1e-10);
    verifyEqual(testCase,result.Intercept.TargetAcceleration_units_s2,[0,0], ...
        'AbsTol',1e-10);
    verifyEqual(testCase,result.velocity_units_s(end,:),[0.2,0],'AbsTol',1e-8);
    verifyEqual(testCase,result.acceleration_units_s2(end,:),[0,0],'AbsTol',1e-8);
end

function testEarliestMovingTargetUsesChronologicalClock(testCase)
    targetTime_s=(0:2:12).';
    targetMotion=struct('time_s',targetTime_s, ...
        'position_units',[4+0.1*targetTime_s,ones(size(targetTime_s))], ...
        'InterpolationMethod','linear');
    goal=struct('time_s',12,'targetMotion',targetMotion);
    result=planner([],state(0,[0,0]),goal,standardLimits(), ...
        struct('GoalTimeMode','earliestArrival','TemporalResolution_s',0.5));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyFalse(testCase,result.TemporalSearch.GlobalEarliestProven);
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,result.ArrivalTime_s, ...
        'AbsTol',1e-10);
    verifyEqual(testCase,result.Intercept.Time_s,result.ArrivalTime_s, ...
        'AbsTol',1e-10);
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

function testDenseTimedRouteWithoutWaitWindow(testCase)
    initial=state(0,[0,0]);
    goal=state(3.6,[4,0]);
    limits=standardLimits();
    limits.xInterval_units=[-30,30];
    limits.yInterval_units=[-30,30];
    base=[-0.25,-0.25;0.25,-0.25;0.25,0.25;-0.25,0.25]+[20,20];
    sourceTime_s=linspace(0,3.6,17).';
    xByTime_units=arrayfun(@(time_s) ...
        base(:,1)+0.1*time_s/3.6,sourceTime_s,'UniformOutput',false);
    yByTime_units=repmat({base(:,2)},17,1);
    obstacle=obstacleAvoidance.obstacles.createObstacle( ...
        'remote dense mover',sourceTime_s,xByTime_units,yByTime_units,0);
    result=planner(obstacle,initial,goal,limits, ...
        struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.225));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,3.6,'AbsTol',1e-10);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyEqual(testCase,result.TemporalSearch.TrialStage, ...
        "timeExpandedVisibilityGraph");
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

function testSpatialAndTimedMotionInfeasibilityRemainDistinct(testCase)
    obstacle=struct('Vertices_units',[-1,-1;1,-1;1,1;-1,1]);
    initial=state(0,[-4,0]); goal=state(5.5,[4,0]);
    options=struct('GoalTimeMode','fixedArrival');
    spatial=planner(obstacle,initial,goal,standardLimits(),options);
    verifyFailure(testCase,spatial,"noOptimizedFeasibleIterate");
    options.FixedArrivalSearch='timeExpanded';
    options.TemporalResolution_s=0.5;
    timed=planner(obstacle,initial,goal,standardLimits(),options);
    verifyFailure(testCase,timed,"timedMotionInfeasible");
end

function testStateValidationDecisions(testCase)
    goal=state(10,[4,0]);
    verifyError(testCase,@()planner([],42,goal,standardLimits(),struct()), ...
        'planTrajectory:InvalidState');
    initial=state(0,[0,0]); initial.unusedField=1;
    verifyError(testCase,@()planner([],initial,goal,standardLimits(),struct()), ...
        'planTrajectory:UnsupportedStateField');
    initial=state(0,[0,0]); initial.position_units=[0,NaN];
    verifyError(testCase,@()planner([],initial,goal,standardLimits(),struct()), ...
        'planTrajectory:InvalidState');
    verifyError(testCase,@()planner([],state(0,[0,0]),state(0,[4,0]), ...
        standardLimits(),struct()),'planTrajectory:InvalidTimeOrder');
    verifyError(testCase,@()planner([],state(0,[0,0]),state(10,[0,0]), ...
        standardLimits(),struct()),'planTrajectory:CoincidentEndpoints');
end

function testLimitValidationDecisions(testCase)
    initial=state(0,[0,0]); goal=state(10,[4,0]);
    verifyError(testCase,@()planner([],initial,goal,42,struct()), ...
        'planTrajectory:InvalidLimits');
    limits=standardLimits(); limits.unusedField=1;
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planTrajectory:UnsupportedLimitField');
    limits=standardLimits(); limits.xInterval_units=[1,1];
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planTrajectory:InvalidWorkspace');
    limits=standardLimits(); limits.maxVelocity_units_s=2;
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planTrajectory:MixedLimitModes');
    limits=standardLimits(); limits.maxJerk_units_s3=[4,0];
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planTrajectory:InvalidDerivativeLimit');
end

function testOptionValidationDecisions(testCase)
    initial=state(0,[0,0]); goal=state(10,[4,0]); limits=standardLimits();
    verifyError(testCase,@()planner([],initial,goal,limits,42), ...
        'planTrajectory:InvalidOptions');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('GoalTimeMode','unsupported')),'planner:UnsupportedGoalTimeMode');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('WrapX',2)),'planner:InvalidLogicalOption');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('SampleTime_s',0)),'MATLAB:expectedPositive');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('MaxArrivalTrials',0)),'MATLAB:expectedPositive');
    verifyWarning(testCase,@()planner([],initial,goal,limits, ...
        struct('unusedOption',1)),'planTrajectory:UnknownOptions');
end

function testTargetDerivativeValidationDecisions(testCase)
    initial=state(0,[0,0]); limits=standardLimits();
    verifyError(testCase,@()planner([],initial,state(10,[4,0]),limits, ...
        struct('MatchTargetVelocity',true)),'planner:MissingTarget');
    targetMotion=struct('time_s',[0;10], ...
        'position_units',[3,0;4,0],'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion, ...
        'velocity_units_s',[0,0]);
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('MatchTargetVelocity',true)), ...
        'planner:ConflictingTargetDerivative');
end

function testTargetHistoryValidationDecisions(testCase)
    initial=state(0,[0,0]); limits=standardLimits();
    goal=struct('time_s',10,'targetMotion',struct('time_s',[0;10]));
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planner:InvalidTarget');

    motion=struct('time_s',[0;0],'position_units',[3,0;4,0]);
    goal=struct('time_s',0,'targetMotion',motion);
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planner:InvalidTargetTime');

    motion=struct('time_s',[0;10],'position_units',[3,0;4,0]);
    goal=struct('time_s',11,'targetMotion',motion);
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planner:TargetTimeOutsideHistory');

    motion.InterpolationMethod='unsupported';
    goal=struct('time_s',5,'targetMotion',motion);
    verifyError(testCase,@()planner([],initial,goal,limits,struct()), ...
        'planner:InvalidTargetInterpolation');

    motion=struct('time_s',[0;5;10], ...
        'position_units',[3,0;4,0;6,0],'InterpolationMethod','linear');
    goal=struct('time_s',5,'targetMotion',motion);
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('MatchTargetVelocity',true)), ...
        'planner:UndefinedTargetDerivative');
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
