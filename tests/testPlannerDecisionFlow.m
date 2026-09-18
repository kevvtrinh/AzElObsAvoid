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
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"dynamicEndpointInfeasible");
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"notSearched");

    initial=state(0,[1,0]); initial.acceleration_units_s2=[1,0];
    result=planner([],initial,state(10,[0,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"dynamicEndpointInfeasible");

    goal=state(10,[-1,0]); goal.acceleration_units_s2=[-1,0];
    result=planner([],state(0,[0,0]),goal,limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyFailure(testCase,result,"dynamicEndpointInfeasible");
end

function testWorkspaceOvershootFallsThroughToBmtp(testCase)
    limits=standardLimits(); limits.xInterval_units=[-1,1];
    initial=state(0,[0.5,0]);
    initial.velocity_units_s=[0.3,0];
    result=planner([],initial,state(10,[0,0]),limits, ...
        struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
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
    verifyEqual(testCase,numel(result.Attempts),1);
    verifyEqual(testCase,result.Attempts.Kind,"spatialVisibility");
    verifyTrue(testCase,result.Attempts.Selected);
    verifyFalse(testCase,result.EarliestArrival.GlobalEarliestProven);
    verifyEqual(testCase,result.EarliestArrival.SelectedAttemptIndex,1);
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
    verifyFalse(testCase,result.EarliestArrival.Capabilities.TimedVariableClockBmtp);
    verifyGreaterThanOrEqual(testCase,numel(result.Attempts),1);
    verifyTrue(testCase,all([result.Attempts.Kind] == ...
        "chronologicalFixedArrival"));
    verifyEqual(testCase,nnz([result.Attempts.Selected]),1);
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
    verifyFalse(testCase,result.EarliestArrival.Capabilities.TimedVariableClockBmtp);
    verifyGreaterThanOrEqual(testCase,numel(result.Attempts),1);
    verifyTrue(testCase,all([result.Attempts.Kind] == ...
        "chronologicalFixedArrival"));
    verifyEqual(testCase,nnz([result.Attempts.Selected]),1);
end

function testEarliestMovingTargetMatchesSelectedClockDerivatives(testCase)
    targetMotion=struct('time_s',[0;5;10], ...
        'position_units',[2,0;3,0;4,0], ...
        'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    options=struct('GoalTimeMode','earliestArrival', ...
        'MatchTargetVelocity',true,'MatchTargetAcceleration',true, ...
        'TemporalResolution_s',0.5);
    result=planner([],state(0,[0,0]),goal,standardLimits(),options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThan(testCase,result.ArrivalTime_s,goal.time_s);
    [targetPosition_units,targetVelocity_units_s,targetAcceleration_units_s2]= ...
        obstacleAvoidance.input.targetPositionAtTime( ...
        targetMotion,result.ArrivalTime_s);
    verifyEqual(testCase,result.Intercept.TargetPosition_units, ...
        targetPosition_units,'AbsTol',1e-10);
    verifyEqual(testCase,result.velocity_units_s(end,:), ...
        targetVelocity_units_s,'AbsTol',1e-8);
    verifyEqual(testCase,result.acceleration_units_s2(end,:), ...
        targetAcceleration_units_s2,'AbsTol',1e-8);
end

function testMovingTargetTrialRetainsTrialClockDerivatives(testCase)
    targetMotion=struct('time_s',[0;4.3;10], ...
        'position_units',[2,0;2.43,0;5.28,0], ...
        'InterpolationMethod','linear');
    initial=state(0,[0,0]);
    goal=struct('time_s',10,'targetMotion',targetMotion);
    options=struct('GoalTimeMode','earliestArrival', ...
        'MatchTargetVelocity',true,'MatchTargetAcceleration',true, ...
        'TemporalResolution_s',0.5);
    result=planner([],initial,goal,standardLimits(),options);

    expectedTrialTime_s=3;
    firstDuration_s=targetMotion.time_s(2)-targetMotion.time_s(1);
    expectedTrialVelocity_units_s= ...
        (targetMotion.position_units(2,:)-targetMotion.position_units(1,:))/firstDuration_s;
    expectedTrialPosition_units=targetMotion.position_units(1,:)+ ...
        (expectedTrialTime_s-targetMotion.time_s(1))*expectedTrialVelocity_units_s;
    expectedTrialAcceleration_units_s2=[0,0];
    finalDuration_s=targetMotion.time_s(3)-targetMotion.time_s(2);
    expectedHorizonVelocity_units_s= ...
        (targetMotion.position_units(3,:)-targetMotion.position_units(2,:))/finalDuration_s;

    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.Inputs.goalState.time_s,goal.time_s);
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,expectedTrialTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.ArrivalTime_s,expectedTrialTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.Inputs.goalState.targetMotion,targetMotion);
    verifyEqual(testCase,result.Inputs.goalState.position_units, ...
        expectedTrialPosition_units,'AbsTol',1e-12);
    verifyEqual(testCase,result.Inputs.goalState.velocity_units_s, ...
        expectedTrialVelocity_units_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.Inputs.goalState.acceleration_units_s2, ...
        expectedTrialAcceleration_units_s2,'AbsTol',1e-12);
    verifyEqual(testCase,result.Intercept.Time_s,expectedTrialTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.Intercept.TargetPosition_units, ...
        expectedTrialPosition_units,'AbsTol',1e-12);
    verifyEqual(testCase,result.Intercept.TargetVelocity_units_s, ...
        expectedTrialVelocity_units_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.Intercept.TargetAcceleration_units_s2, ...
        expectedTrialAcceleration_units_s2,'AbsTol',1e-12);
    verifyGreaterThan(testCase,norm(expectedHorizonVelocity_units_s- ...
        expectedTrialVelocity_units_s),0.3);
    verifyGreaterThan(testCase,norm(result.Intercept.TargetVelocity_units_s- ...
        expectedHorizonVelocity_units_s),0.3);
end

function testDisconnectedMovingTargetClockAdvances(testCase)
    wall_units=[-0.4,-5;0.4,-5;0.4,5;-0.4,5];
    obstacle=obstacleAvoidance.obstacles.createObstacle( ...
        'static separating wall',0,{wall_units(:,1)},{wall_units(:,2)},0);
    targetMotion=struct('time_s',[0;10], ...
        'position_units',[4,0;-2,0],'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    limits=standardLimits();
    limits.xInterval_units=[-5,5];
    limits.yInterval_units=[-5,5];
    result=planner(obstacle,state(0,[-4,0]),goal,limits, ...
        struct('GoalTimeMode','earliestArrival','TemporalResolution_s',1));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase,numel(result.Attempts),1);
    verifyEqual(testCase,result.Attempts(1).FailureStage,"search");
    verifyTrue(testCase,result.Attempts(1).MethodFallbackEligible);
    verifyTrue(testCase,any([result.Attempts.Selected]));

    fixedInitial=state(0,[-4,0]);
    fixedInitial.velocity_units_s=[0.1,0];
    fixedGoal=state(10,[4,0]);
    fixedOptions=struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',1,'WrapX',false,'WrapY',false);
    fixedResult=planner(obstacle,fixedInitial,fixedGoal,limits,fixedOptions);
    verifyFalse(testCase,fixedResult.Success);
    verifyEqual(testCase,fixedResult.TerminationReason,"noVisibilityRoute");
    verifyEqual(testCase,numel(fixedResult.Attempts),1);
    verifyFalse(testCase,fixedResult.Attempts.MethodFallbackEligible);
    verifyTrue(testCase,fixedResult.TemporalSearch.TerminalFailure);
    verifyEqual(testCase,fixedResult.Options.GoalTimeMode,"fixedArrival");
    verifyFalse(testCase,fixedResult.Options.WrapX);
    verifyFalse(testCase,fixedResult.Options.WrapY);
    verifyEqual(testCase,fixedResult.SuppliedLimits,limits);
    verifyEqual(testCase,fixedResult.SuppliedGoalState.position_units, ...
        fixedGoal.position_units);
    verifyEqual(testCase,fixedResult.SuppliedGoalState.time_s,4);
    % Anchored to the obstacle this test passed in, not to another field of
    % the same result: a record agreeing with itself proves nothing here.
    verifyEqual(testCase,fixedResult.Inputs.obstacles, ...
        obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,4],true));
    verifyEqual(testCase,fixedResult.Inputs.goalState.time_s,4);
    verifyTrue(testCase,isfield(fixedResult,'OuterRequest'));
    verifyEqual(testCase,fixedResult.OuterRequest.Obstacles,obstacle);
    verifyEqual(testCase,fixedResult.OuterRequest.GoalTime_s,fixedGoal.time_s);
    % The supplied goal provenance is a different field from the outer clock;
    % relocating one does not cover the other.
    verifyEqual(testCase,fixedResult.OuterRequest.SuppliedGoalState,fixedGoal);
    verifyEqual(testCase,fixedResult.OuterRequest.GoalTimeMode, ...
        string(fixedOptions.GoalTimeMode));
    verifyEqual(testCase,fixedResult.OuterRequest.FixedArrivalTrialTime_s,4);
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

function testWrappedNonrestEarliestTrialIsAcceptedOnce(testCase)
    % A chronological trial is planned on its own clock, so its periodic
    % reach follows the trial clock. Accepting it against the outer request
    % in one validation must keep the record consistent: before this gate
    % was unified, the trial passed its own validation and a second pass
    % rejected the same motion against the outer horizon.
    limits=standardLimits();
    limits.xInterval_units=[-180,180];
    initial=state(0,[179,0]); initial.velocity_units_s=[0.1,0];
    goal=state(10,[-179,0]);
    result=planner([],initial,goal,limits, ...
        struct('GoalTimeMode','earliestArrival','WrapX',true));
    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.TerminationReason,"goalReached");
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyEqual(testCase,result.Options.GoalTimeMode,"earliestArrival");
    verifyEqual(testCase,result.Inputs.goalState.time_s,10);
    verifyLessThan(testCase,result.ArrivalTime_s,10);
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,result.ArrivalTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.RequestedGoalState.position_units,[-179,0]);
    verifyEqual(testCase,result.Inputs.goalState.position_units,[181,0]);
    reach=result.Limits.maxVelocity_units_s(1)*(result.Inputs.goalState.time_s-0);
    verifyEqual(testCase,result.Limits.xInterval_units,179+[-reach,reach]);
end

function testPeriodicObstacleImageBlocksTheSeam(testCase)
    % A wall just inside the negative edge of the periodic interval is, in
    % the unwrapped frame, the image between the initial position and the
    % nearest goal image. The planner must detour around that image, the
    % record must keep the supplied obstacle, and the validator must rebuild
    % the same image on its own.
    limits=standardLimits();
    limits.xInterval_units=[-180,180];
    obstacle=struct('Vertices_units',[-180,-3;-179.5,-3;-179.5,3;-180,3]);
    result=planner(obstacle,state(0,[179,0]),state(10,[-179,0]),limits, ...
        struct('GoalTimeMode','fixedArrival','WrapX',true));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Inputs.obstacles,obstacle);
    verifyEqual(testCase,result.Inputs.goalState.position_units,[181,0]);
    verifyEqual(testCase,result.PeriodicImages.ObstacleImageCount,1);
    verifyEqual(testCase,numel(result.PreparedObstacles),1);
    verifyGreaterThanOrEqual(testCase,min(result.PreparedObstacles(1).x_units{1}),180);
    verifyGreaterThan(testCase,result.MotionLength_units,6);
    verifyGreaterThan(testCase,max(abs(result.position_units(:,2))),3);
end

function testPeriodicEarliestTrialsRunInsideTheUnwrappedFrame(testCase)
    % A non-rest earliest-arrival request with a wall at the seam reaches the
    % chronological search inside the unwrapped frame. Each trial is accepted
    % against the periodic request, so the record keeps the supplied obstacle
    % and periodic options while the motion detours around the wall's image.
    limits=standardLimits();
    limits.xInterval_units=[-180,180];
    obstacle=struct('Vertices_units',[-180,-3;-179.5,-3;-179.5,3;-180,3]);
    initial=state(0,[179,0]); initial.velocity_units_s=[0.1,0];
    goal=state(10,[-179,0]);
    options=struct('GoalTimeMode','earliestArrival','WrapX',true, ...
        'WrapY',false,'TemporalResolution_s',6,'MaxArrivalTrials',2);
    % A coarse trial grid keeps the fixture cheap: the first declared clock
    % (6 s) admits the detour, so one trial is planned and accepted.
    result=planner(obstacle,initial,goal,limits,options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyLessThanOrEqual(testCase,numel(result.TemporalSearch.TrialTime_s),2);
    verifyEqual(testCase,result.Options.GoalTimeMode,"earliestArrival");
    verifyTrue(testCase,result.Options.WrapX);
    verifyEqual(testCase,result.Inputs.obstacles,obstacle);
    verifyEqual(testCase,result.Inputs.goalState.time_s,10);
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,result.ArrivalTime_s,'AbsTol',1e-9);
    verifyLessThan(testCase,result.ArrivalTime_s,10);
    verifyGreaterThan(testCase,result.MotionLength_units,6);

    expectedTrialTime_s=initial.time_s+options.TemporalResolution_s;
    reach_units=limits.maxVelocity_units_s(1)*(goal.time_s-initial.time_s);
    expectedBand_units=initial.position_units(1)+[-reach_units,reach_units];
    verifyEqual(testCase,result.SuppliedLimits,limits);
    verifyEqual(testCase,result.SuppliedGoalState.position_units,goal.position_units);
    verifyEqual(testCase,result.SuppliedGoalState.time_s,goal.time_s);
    verifyEqual(testCase,result.RequestedLimits.xInterval_units,limits.xInterval_units);
    verifyEqual(testCase,result.Limits.xInterval_units,expectedBand_units);
    verifyFalse(testCase,result.Options.WrapY);
    verifyFalse(testCase,isfield(result,'OuterRequest'));
    verifyEqual(testCase,result.Inputs.goalState.time_s,goal.time_s);
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,expectedTrialTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.ArrivalTime_s,expectedTrialTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,numel(result.Attempts),1);
    verifyEqual(testCase,result.Attempts.TrialTime_s,expectedTrialTime_s,'AbsTol',1e-12);
end

function testPeriodicAllCandidateFailuresDeclareOuterRequest(testCase)
    obstacle=struct('Vertices_units',[-1,-5;1,-5;1,5;-1,5]);
    initial=state(0,[-4,0]);
    goal=state(12,[4,0]);
    limits=standardLimits();
    limits.yInterval_units=[-4,4];
    options=struct('GoalTimeMode','fixedArrival', ...
        'WrapX',false,'WrapY',true);
    result=planner(obstacle,initial,goal,limits,options);

    reach_units=limits.maxVelocity_units_s(2)*(goal.time_s-initial.time_s);
    expectedBand_units=initial.position_units(2)+[-reach_units,reach_units];
    verifyFailure(testCase,result,"noVisibilityRoute");
    verifyGreaterThan(testCase,numel(result.PeriodicImages.CandidatePlanned),1);
    verifyTrue(testCase,all(result.PeriodicImages.CandidatePlanned));
    verifyFalse(testCase,any(result.PeriodicImages.CandidateTerminationReason == ...
        "goalReached"));
    verifyEqual(testCase,result.Options.GoalTimeMode,string(options.GoalTimeMode));
    verifyFalse(testCase,result.Options.WrapX);
    verifyTrue(testCase,result.Options.WrapY);
    verifyEqual(testCase,result.SuppliedLimits,limits);
    verifyEqual(testCase,result.SuppliedGoalState.position_units,goal.position_units);
    verifyEqual(testCase,result.SuppliedGoalState.time_s,goal.time_s);
    verifyEqual(testCase,result.RequestedLimits.yInterval_units,limits.yInterval_units);
    verifyEqual(testCase,result.Limits.yInterval_units,expectedBand_units);
    verifyEqual(testCase,result.Inputs.obstacles,obstacle);
    verifyEqual(testCase,result.Inputs.goalState.time_s,goal.time_s);
    verifyFalse(testCase,isfield(result,'OuterRequest'));
end

function testPeriodicFarImageBeatsABlockedNearImage(testCase)
    % With a short period every goal image lies inside the band. A tall wall
    % between the initial position and the nearest image makes the far way
    % round shorter, so the shortest valid candidate must win.
    limits=standardLimits();
    limits.xInterval_units=[-5,5];
    wall=struct('Vertices_units',[4.4,-12;4.6,-12;4.6,12;4.4,12]);
    result=planner(wall,state(0,[4,0]),state(10,[-4,0]),limits, ...
        struct('GoalTimeMode','fixedArrival','WrapX',true));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.RequestedGoalState.position_units,[-4,0]);
    verifyEqual(testCase,result.Inputs.goalState.position_units,[-4,0]);
    verifyEqual(testCase,result.PeriodicImages.GoalOffset_units,[-10,0]);
    verifyEqual(testCase,result.MotionLength_units,8,'AbsTol',1e-6);
    verifyGreaterThan(testCase,nnz(result.PeriodicImages.CandidatePlanned),1);

    earliest=planner(wall,state(0,[4,0]),state(10,[-4,0]),limits, ...
        struct('GoalTimeMode','earliestArrival','WrapX',true));
    verifyTrue(testCase,earliest.Success,earliest.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(earliest).Passed);
    verifyEqual(testCase,earliest.PeriodicImages.GoalOffset_units,[-10,0]);
    verifyGreaterThan(testCase,nnz(earliest.PeriodicImages.CandidatePlanned),1);
    verifyLessThan(testCase,earliest.ArrivalTime_s,10);
end

function testPeriodicYAndDualAxisWrap(testCase)
    limits=standardLimits();
    limits.xInterval_units=[-180,180];
    limits.yInterval_units=[-90,90];
    yResult=planner([],state(0,[0,89]),state(10,[0,-89]),limits, ...
        struct('GoalTimeMode','fixedArrival','WrapY',true));
    verifyTrue(testCase,yResult.Success,yResult.Message);
    verifyEqual(testCase,yResult.Inputs.goalState.position_units,[0,91]);
    verifyEqual(testCase,yResult.MotionLength_units,2,'AbsTol',1e-8);
    bothResult=planner([],state(0,[179,89]),state(10,[-179,-89]),limits, ...
        struct('GoalTimeMode','fixedArrival','WrapX',true,'WrapY',true));
    verifyTrue(testCase,bothResult.Success,bothResult.Message);
    verifyEqual(testCase,bothResult.Inputs.goalState.position_units,[181,91]);
    verifyEqual(testCase,bothResult.MotionLength_units,sqrt(8),'AbsTol',1e-8);
end

function testPeriodicMovingTargetIsLiftedAcrossTheSeam(testCase)
    % A target that crosses the seam is lifted by continuity, so the
    % intercept is planned on a two-unit move and the record keeps the
    % supplied periodic target for the validator to lift again.
    limits=standardLimits(); limits.yInterval_units=[-90,90];
    targetMotion=struct('time_s',[0;10], ...
        'position_units',[0,89;0,-89],'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    result=planner([],state(0,[0,85]),goal,limits, ...
        struct('GoalTimeMode','fixedArrival','WrapY',true));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Inputs.goalState.targetMotion.position_units,[0,89;0,91]);
    verifyEqual(testCase,result.RequestedGoalState.targetMotion.position_units,[0,89;0,-89]);
    verifyEqual(testCase,result.Intercept.TargetPosition_units,[0,91],'AbsTol',1e-9);
    verifyEqual(testCase,result.MotionLength_units,6,'AbsTol',1e-6);
end

function testPeriodicSeamCrossingTargetRejectsStaleMatchedDerivative(testCase)
    initial=state(0,[0,0.5]);
    targetMotion=struct('time_s',[0;10], ...
        'position_units',[0,0.9;0,-0.9],'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    limits=standardLimits(); limits.yInterval_units=[-1,1];
    options=struct('GoalTimeMode','fixedArrival','WrapY',true, ...
        'MatchTargetVelocity',true);
    verifyError(testCase,@()planner([],initial,goal,limits,options), ...
        'planner:ConflictingTargetDerivative');
end

function testPeriodicWindingTargetRejectsCoincidentImageEndpoint(testCase)
    initial=state(0,[0,0]);
    targetMotion=struct('time_s',(0:2:10)', ...
        'position_units',[zeros(6,1),[0;0.4;0.8;-0.8;-0.4;0]], ...
        'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    limits=struct('xInterval_units',[-6,6],'yInterval_units',[-1,1], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options=struct('GoalTimeMode','fixedArrival','WrapY',true);
    verifyError(testCase,@()planner([],initial,goal,limits,options), ...
        'planTrajectory:CoincidentEndpoints');
end

function testInitiallyOccupiedFutureGoalUsesArrivalDetour(testCase)
    local=[-0.8,-0.8;0.8,-0.8;0.8,0.8;-0.8,0.8];
    first=local+[4,0];
    last=local+[4,6];
    departing=obstacleAvoidance.obstacles.createObstacle('departing goal box',[0;10], ...
        {first(:,1);last(:,1)},{first(:,2);last(:,2)},0);
    blocker=obstacleAvoidance.obstacles.createObstacle('central blocker',0, ...
        {[-1;1;1;-1]},{[-1;-1;1;1]},0);
    obstacles=obstacleAvoidance.obstacles.combineObstacles({departing;blocker});
    result=planner(obstacles,state(0,[-4,0]),state(10,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"arrivalSpatialSnapshot");
    verifyGreaterThan(testCase,size(result.Route_units,1),2);
end

function testDisconnectedInitialSnapshotUsesArrivalSnapshot(testCase)
    wall=[-0.2,-7;0.2,-7;0.2,7;-0.2,7];
    moved=wall+[0,14];
    obstacle=obstacleAvoidance.obstacles.createObstacle('departing wall',[0;5], ...
        {wall(:,1);moved(:,1)},{wall(:,2);moved(:,2)},0);
    result=planner(obstacle,state(0,[-4,0]),state(12,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"arrivalSpatialSnapshot");
end

function testWrappedResultCarriesOuterProvenance(testCase)
    % A wrapped request is planned as plain requests in the unwrapped frame,
    % so the returned record has to declare the outer request it was accepted
    % against. Pin every provenance field, because only the requested goal was
    % covered before and the rest is what the acceptance gate reads.
    limits=standardLimits();
    limits.xInterval_units=[-180,180];
    result=planner([],state(0,[179,0]),state(10,[-179,0]),limits, ...
        struct('GoalTimeMode','fixedArrival','WrapX',true));
    verifyTrue(testCase,result.Success,result.Message);

    % Supplied provenance is the request exactly as handed in.
    verifyEqual(testCase,result.SuppliedLimits,limits);
    verifyEqual(testCase,result.SuppliedGoalState.position_units,[-179,0]);
    verifyEqual(testCase,result.SuppliedGoalState.time_s,10);

    % Requested limits are normalized but still the periodic workspace, while
    % the effective limits are the unwrapped reach band the images live in.
    verifyEqual(testCase,result.RequestedLimits.xInterval_units,[-180,180]);
    verifyEqual(testCase,result.Limits.xInterval_units,[159,199]);
    verifyEqual(testCase,result.RequestedGoalState.position_units,[-179,0]);

    % The effective goal is the selected image, but its clock is the outer
    % horizon. Position and time on this one struct have different owners.
    verifyEqual(testCase,result.Inputs.goalState.position_units,[181,0]);
    verifyEqual(testCase,result.Inputs.goalState.time_s,10);

    % Wrapping and arrival mode are declared as the outer request, not as the
    % plain unwrapped request each image was actually planned as.
    verifyTrue(testCase,result.Options.WrapX);
    verifyFalse(testCase,result.Options.WrapY);
    verifyEqual(testCase,string(result.Options.GoalTimeMode),"fixedArrival");
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
end

function testCoverageFieldsMatchTheScenePath(testCase)
    % The planner builds coverage in three shapes: static, dynamic
    % earliest-arrival, and dynamic fixed-arrival. Downstream code branches on
    % field presence, so a placeholder field would change behaviour without
    % changing any motion a test already checks. Pin the exact field list.
    staticResult=planner([],state(0,[-4,0]),state(12,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,staticResult.Success,staticResult.Message);
    verifyEqual(testCase,fieldnames(staticResult.PlaneCertificate.Coverage), ...
        {'ExactRegionCount'});

    box=[-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    first=box+[0,8];
    last=box+[2,8];
    mover=obstacleAvoidance.obstacles.createObstacle('remote mover',[0;20], ...
        {first(:,1);last(:,1)},{first(:,2);last(:,2)},0);

    fixedResult=planner(mover,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,fixedResult.Success,fixedResult.Message);
    verifyEqual(testCase,fieldnames(fixedResult.PlaneCertificate.Coverage), ...
        {'ExactRegionCount';'ActiveTimeInterval_s';'EndRegions_units';'BreakTime_s'});

    % BreakTime_s is the one that separates two dynamic runs from each other.
    earliestResult=planner(mover,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,earliestResult.Success,earliestResult.Message);
    verifyEqual(testCase,fieldnames(earliestResult.PlaneCertificate.Coverage), ...
        {'ExactRegionCount';'ActiveTimeInterval_s';'EndRegions_units'});
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
    verifyFalse(testCase,result.VisibilityGraph.GraphIsFullyEnumerated);
    verifyFalse(testCase,isfield(result.SolverDiagnostics,'DepartureSchedule'));
    verifyFalse(testCase,isfield(result,'TemporalSearch'));
    verifyEqual(testCase,numel(result.Attempts),2);
    verifyEqual(testCase,[result.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Attempts(1).Selected);
    verifyFalse(testCase,result.EarliestArrival.GlobalEarliestProven);
    selectedIndex=result.EarliestArrival.SelectedAttemptIndex;
    verifyEqual(testCase,result.EarliestArrival.IncumbentArrival_s, ...
        result.ArrivalTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.Attempts(selectedIndex).CandidateArrival_s, ...
        result.ArrivalTime_s,'AbsTol',1e-12);
end

function testEquivalentSparseAndDenseHistoriesUseCertifiedDeparture(testCase)
    initial=state(0,[0,0]);
    goal=state(3.6,[4,0]);
    limits=standardLimits();
    limits.xInterval_units=[-30,30];
    limits.yInterval_units=[-30,30];
    base=[-0.25,-0.25;0.25,-0.25;0.25,0.25;-0.25,0.25]+[20,20];
    sampleCounts=[2,17];
    results=cell(size(sampleCounts));
    for historyIndex=1:numel(sampleCounts)
        sourceTime_s=linspace(0,3.6,sampleCounts(historyIndex)).';
        xByTime_units=arrayfun(@(time_s) ...
            base(:,1)+0.1*time_s/3.6,sourceTime_s,'UniformOutput',false);
        yByTime_units=repmat({base(:,2)},sampleCounts(historyIndex),1);
        obstacle=obstacleAvoidance.obstacles.createObstacle( ...
            'remote affine mover',sourceTime_s,xByTime_units,yByTime_units,0);
        results{historyIndex}=planner(obstacle,initial,goal,limits, ...
            struct('GoalTimeMode','earliestArrival', ...
            'TemporalResolution_s',0.225));
        verifyTrue(testCase,results{historyIndex}.Success,results{historyIndex}.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(results{historyIndex}).Passed);
        verifyEqual(testCase,results{historyIndex}.VisibilityGraph.SearchKind, ...
            "c3DepartureSchedule");
        verifyFalse(testCase,isfield(results{historyIndex},'TemporalSearch'));
    end
    verifyEqual(testCase,results{1}.ArrivalTime_s,results{2}.ArrivalTime_s, ...
        'AbsTol',1e-8);
    verifyEqual(testCase,results{1}.MotionLength_units, ...
        results{2}.MotionLength_units,'AbsTol',1e-8);
end

function testArrivalSearchExhausted(testCase)
    targetMotion=struct('time_s',[0;1], ...
        'position_units',[100,0;101,0],'InterpolationMethod','linear');
    goal=struct('time_s',1,'targetMotion',targetMotion);
    result=planner([],state(0,[0,0]),goal,standardLimits(), ...
        struct('GoalTimeMode','earliestArrival','TemporalResolution_s',0.5));
    verifyFailure(testCase,result,"arrivalSearchExhausted");
    verifyTrue(testCase,isfield(result,'TemporalSearch'));
    verifyEmpty(testCase,result.TemporalSearch.TrialTime_s);
    verifyGreaterThan(testCase, ...
        result.TemporalSearch.PrescreenedCandidateCount,0);
    verifyEqual(testCase,result.TemporalSearch.SolverTrialCount,0);
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
    initial=state(NaN,[0,0]);
    verifyError(testCase,@()planner([],initial,goal,standardLimits(),struct()), ...
        'MATLAB:expectedFinite');
    verifyError(testCase,@()planner([],state(0,[0,0]),state(0,[4,0]), ...
        standardLimits(),struct()),'planTrajectory:InvalidTimeOrder');
    verifyError(testCase,@()planner([],state(0,[0,0]),state(10,[0,0]), ...
        standardLimits(),struct()),'planTrajectory:CoincidentEndpoints');
end

function testBackwardsTimePrecedesWrappedTargetEvaluation(testCase)
    initial=state(5,[0,0]);
    targetMotion=struct('time_s',[5;10], ...
        'position_units',[1,0;2,0],'InterpolationMethod','linear');
    goal=struct('time_s',0,'targetMotion',targetMotion);
    verifyError(testCase,@()planner([],initial,goal,standardLimits(), ...
        struct('WrapX',true)),'planTrajectory:InvalidTimeOrder');
end

function testBackwardsTimeOnUnwrappedFixedGoal(testCase)
    verifyError(testCase,@()planner([],state(5,[0,0]),state(0,[2,0]), ...
        standardLimits(),struct()),'planTrajectory:InvalidTimeOrder');
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

    limits=standardLimits();
    limits.maxVelocity_units_s=2;
    limits.maxAcceleration_units_s2=2;
    limits.maxJerk_units_s3=4;
    result=planner([],initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.Limits.maxVelocity_units_s,[sqrt(2),sqrt(2)], ...
        'AbsTol',1e-12);
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
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('MaxArrivalTrials',1.5)),'MATLAB:expectedInteger');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('MaxArrivalCandidates',0)),'MATLAB:expectedPositive');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('MaxArrivalCandidates',1.5)),'MATLAB:expectedInteger');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('IncumbentRefinementTrialLimit',-1)),'MATLAB:expectedNonnegative');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('IncumbentRefinementTrialLimit',1.5)),'MATLAB:expectedInteger');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('SpatialProbeIterationLimit',0)),'MATLAB:expectedPositive');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('SpatialProbeIterationLimit',36)), ...
        'planner:InvalidSpatialProbeIterationLimit');
    defaulted=planner([],initial,goal,limits,struct('SampleTime_s',[]));
    verifyTrue(testCase,defaulted.Success,defaulted.Message);
    verifyEqual(testCase,defaulted.Options.SampleTime_s,0.05);
    verifyEqual(testCase,defaulted.Options.IncumbentRefinementTrialLimit,0);
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

    goal.velocity_units_s=[0.1,0];
    matched=planner([],initial,goal,limits, ...
        struct('GoalTimeMode','fixedArrival','MatchTargetVelocity',true));
    verifyTrue(testCase,matched.Success,matched.Message);
    verifyEqual(testCase,matched.velocity_units_s(end,:),[0.1,0], ...
        'AbsTol',1e-8);

    goal=struct('time_s',10,'targetMotion',targetMotion);
    accelerationOnly=planner([],initial,goal,limits, ...
        struct('GoalTimeMode','fixedArrival','MatchTargetAcceleration',true));
    verifyTrue(testCase,accelerationOnly.Success,accelerationOnly.Message);
    verifyEqual(testCase,accelerationOnly.acceleration_units_s2(end,:),[0,0], ...
        'AbsTol',1e-8);
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
