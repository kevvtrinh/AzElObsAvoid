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
    verifyGreaterThan(testCase,result.SeparationProof.AllPairCount,0);
    verifyEqual(testCase,result.SeparationProof.VerifiedPairCount, ...
        result.SeparationProof.AllPairCount);
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
        "arrivalTimeTrial"));
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

function testEarliestMovingTargetUsesArrivalTimeClock(testCase)
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
        "arrivalTimeTrial"));
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
    verifyTrue(testCase,result.Attempts(1).NextMethodAllowed);
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
    verifyFalse(testCase,fixedResult.Attempts.NextMethodAllowed);
    verifyTrue(testCase,fixedResult.TemporalSearch.TerminalFailure);
    verifyEqual(testCase,fixedResult.Options.GoalTimeMode,"fixedArrival");
    verifyEqual(testCase,fixedResult.Options.WrapX,"false");
    verifyEqual(testCase,fixedResult.Options.WrapY,"false");
    verifyEqual(testCase,fixedResult.SuppliedLimits,limits);
    verifyEqual(testCase,fixedResult.SuppliedGoalState.position_units, ...
        fixedGoal.position_units);
    verifyEqual(testCase,fixedResult.SuppliedGoalState.time_s,4);
    % Anchored to the obstacle this test passed in, not to another field of
    % the same result: a record agreeing with itself proves nothing here.
    verifyEqual(testCase,fixedResult.Inputs.obstacles, ...
        obstacleAvoidance.obstacles.prepareObstacles(obstacle,[0,4],true));
    verifyEqual(testCase,fixedResult.Inputs.goalState.time_s,4);
    verifyTrue(testCase,isfield(fixedResult,'ParentRequest'));
    verifyEqual(testCase,fixedResult.ParentRequest.Obstacles,obstacle);
    verifyEqual(testCase,fixedResult.ParentRequest.GoalTime_s,fixedGoal.time_s);
    % The supplied goal provenance is a different field from the outer clock;
    % relocating one does not cover the other.
    verifyEqual(testCase,fixedResult.ParentRequest.SuppliedGoalState,fixedGoal);
    verifyEqual(testCase,fixedResult.ParentRequest.GoalTimeMode, ...
        string(fixedOptions.GoalTimeMode));
    verifyEqual(testCase,fixedResult.ParentRequest.FixedArrivalTrialTime_s,4);
end

function testWrappedWrapUsesNearestImage(testCase)
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
    % An arrival-time trial is planned on its own clock, so its wrapped
    % reach follows the trial clock. Accepting it against the parent request
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

function testWrappedObstacleImageBlocksTheSeam(testCase)
    % A wall just inside the negative edge of the wrapped interval is, in
    % the unwrapped coordinates, the copy between the initial position and the
    % nearest goal copy. The planner must detour around that copy, the
    % record must keep the supplied obstacle, and the validator must rebuild
    % the same copy on its own.
    limits=standardLimits();
    limits.xInterval_units=[-180,180];
    obstacle=struct('Vertices_units',[-180,-3;-179.5,-3;-179.5,3;-180,3]);
    result=planner(obstacle,state(0,[179,0]),state(10,[-179,0]),limits, ...
        struct('GoalTimeMode','fixedArrival','WrapX',true));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Inputs.obstacles,obstacle);
    verifyEqual(testCase,result.Inputs.goalState.position_units,[181,0]);
    verifyEqual(testCase,result.WrappedGoalCopies.ObstacleCopyCount,1);
    verifyEqual(testCase,numel(result.PreparedObstacles),1);
    verifyGreaterThanOrEqual(testCase,min(result.PreparedObstacles(1).x_units{1}),180);
    verifyGreaterThan(testCase,result.MotionLength_units,6);
    verifyGreaterThan(testCase,max(abs(result.position_units(:,2))),3);
end

function testWrappedEarliestTrialsRunInsideTheUnwrappedFrame(testCase)
    % A non-rest earliest-arrival request with a wall at the seam reaches the
    % arrival-time search inside the unwrapped coordinates. Each trial is accepted
    % against the wrapped request, so the record keeps the supplied obstacle
    % and wrapped options while the motion detours around the wall's copy.
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
    verifyEqual(testCase,result.Options.WrapX,"both");
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
    verifyEqual(testCase,result.Options.WrapY,"false");
    verifyFalse(testCase,isfield(result,'ParentRequest'));
    verifyEqual(testCase,result.Inputs.goalState.time_s,goal.time_s);
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,expectedTrialTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.ArrivalTime_s,expectedTrialTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,numel(result.Attempts),1);
    verifyEqual(testCase,result.Attempts.TrialTime_s,expectedTrialTime_s,'AbsTol',1e-12);
end

function testWrappedAllCandidateFailuresDeclareParentRequest(testCase)
    % The wall blocks the direct way. Going over a pole would reach the
    % goal side on a sphere, so caps over both poles block that way too.
    obstacle=struct('Vertices_units',{[-1,-5;1,-5;1,5;-1,5]; ...
        [-6,3.5;6,3.5;6,4;-6,4];[-6,-4;6,-4;6,-3.5;-6,-3.5]});
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
    verifyGreaterThan(testCase,numel(result.WrappedGoalCopies.CandidatePlanned),1);
    verifyTrue(testCase,all(result.WrappedGoalCopies.CandidatePlanned));
    verifyFalse(testCase,any(result.WrappedGoalCopies.CandidateTerminationReason == ...
        "goalReached"));
    verifyEqual(testCase,result.Options.GoalTimeMode,string(options.GoalTimeMode));
    verifyEqual(testCase,result.Options.WrapX,"false");
    verifyEqual(testCase,result.Options.WrapY,"both");
    verifyEqual(testCase,result.SuppliedLimits,limits);
    verifyEqual(testCase,result.SuppliedGoalState.position_units,goal.position_units);
    verifyEqual(testCase,result.SuppliedGoalState.time_s,goal.time_s);
    verifyEqual(testCase,result.RequestedLimits.yInterval_units,limits.yInterval_units);
    verifyEqual(testCase,result.Limits.yInterval_units,expectedBand_units);
    verifyEqual(testCase,result.Inputs.obstacles,obstacle);
    verifyEqual(testCase,result.Inputs.goalState.time_s,goal.time_s);
    verifyFalse(testCase,isfield(result,'ParentRequest'));
end

function testWrappedFarImageBeatsABlockedNearImage(testCase)
    % With a short period every goal copy lies inside the reachable range. A tall wall
    % between the initial position and the nearest copy makes the far way
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
    verifyEqual(testCase,result.WrappedGoalCopies.GoalOffset_units,[-10,0]);
    verifyEqual(testCase,result.MotionLength_units,8,'AbsTol',1e-6);
    verifyGreaterThan(testCase,nnz(result.WrappedGoalCopies.CandidatePlanned),1);

    earliest=planner(wall,state(0,[4,0]),state(10,[-4,0]),limits, ...
        struct('GoalTimeMode','earliestArrival','WrapX',true));
    verifyTrue(testCase,earliest.Success,earliest.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(earliest).Passed);
    verifyEqual(testCase,earliest.WrappedGoalCopies.GoalOffset_units,[-10,0]);
    verifyGreaterThan(testCase,nnz(earliest.WrappedGoalCopies.CandidatePlanned),1);
    verifyLessThan(testCase,earliest.ArrivalTime_s,10);
end

function testWrappedYAndDualAxisWrap(testCase)
    % From (0, 85), goal (4, 85) is nearer than its pole copy (184, 95), so
    % the goal stays put.
    limits = standardLimits();
    limits.xInterval_units = [-180, 180];
    limits.yInterval_units = [-90, 90];
    yResult = planner([], state(0, [0, 85]), state(10, [4, 85]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, yResult.Success, yResult.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(yResult).Passed);
    verifyEqual(testCase, yResult.Inputs.goalState.position_units, [4, 85]);
    verifyEqual(testCase, yResult.MotionLength_units, 4, 'AbsTol', 1e-8);
    % x still shifts by whole turns when both axes wrap.
    bothResult = planner([], state(0, [179, 85]), state(10, [-179, 85]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both", 'WrapY', "both"));
    verifyTrue(testCase, bothResult.Success, bothResult.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(bothResult).Passed);
    verifyEqual(testCase, bothResult.Inputs.goalState.position_units, [181, 85]);
    verifyEqual(testCase, bothResult.MotionLength_units, 2, 'AbsTol', 1e-8);
end

function testWrappedYMakesPoleCopies(testCase)
    % An obstacle at x = 100 to 110, y = 80 to 86 gets a pole copy over the
    % top pole: y mirrored about 90 (94 to 100) and x turned half a turn
    % (280 to 290).
    box = struct('Vertices_units', [100, 80; 110, 80; 110, 86; 100, 86]);
    copies = obstacleAvoidance.input.copyObstaclesAcrossWraps(box, [0, 360; -90, 90], ...
        ["false", "both"], [0, 360; 60, 120]);
    verifyEqual(testCase, numel(copies), 2);
    verifyEqual(testCase, sort(copies(2).y_units{1}), sort(180 - copies(1).y_units{1}), 'AbsTol', 1e-12);
    verifyEqual(testCase, copies(2).x_units{1}, copies(1).x_units{1} + 180, 'AbsTol', 1e-12);
    verifyEqual(testCase, [min(copies(2).y_units{1}), max(copies(2).y_units{1})], [94, 100], 'AbsTol', 1e-12);
    verifyFalse(testCase, contains(copies(1).targetName, "copy"));
    verifyTrue(testCase, contains(copies(2).targetName, "pole copy"));

    % A shape far outside the interval still gets its overlapping pole copy.
    % On y [0 100], y = 1085 to 1095 has a pole copy at 105 to 115.
    farCopies = obstacleAvoidance.input.listWrapImages([0, 1; 1085, 1095], [0, 360; 0, 100], ...
        ["false", "both"], [0, 360; 80, 120]);
    isPoleCopy = farCopies.YScale == -1;
    verifyEqual(testCase, nnz(isPoleCopy), 1);
    verifyEqual(testCase, farCopies.YOffset_units(isPoleCopy) - [1095, 1085], [105, 115], 'AbsTol', 1e-9);

    % Here the pole copy of the goal is far away, so every y mode arrives
    % when no wrapping does, and each result validates.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    arrivals_s = zeros(1, 4);
    wrapModes  = ["false", "both", "forward", "backward"];
    for modeIndex = 1:numel(wrapModes)
        result = planner(box, state(0, [90, 85]), state(40, [120, 85]), limits, ...
            struct('GoalTimeMode', 'earliestArrival', 'WrapY', wrapModes(modeIndex)));
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
        verifyEqual(testCase, result.Options.WrapY, wrapModes(modeIndex));
        arrivals_s(modeIndex) = result.ArrivalTime_s;
    end
    verifyEqual(testCase, arrivals_s, repmat(arrivals_s(1), 1, 4), 'AbsTol', 1e-6);
end

function testPoleCrossingShortensSlew(testCase)
    % From (10, 89) to (190, 89): without y wrapping the slew turns 180 in
    % azimuth. With y wrapping the goal's pole copy is (10, 91), a 2-degree
    % move over the top pole in actuator coordinates.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    options = struct('GoalTimeMode', 'earliestArrival', 'WrapY', "both");
    overPole = planner([], state(0, [10, 89]), state(60, [190, 89]), limits, options);
    verifyTrue(testCase, overPole.Success, overPole.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(overPole).Passed);
    verifyEqual(testCase, overPole.Inputs.goalState.position_units, [10, 91], 'AbsTol', 1e-12);
    verifyTrue(testCase, overPole.WrappedGoalCopies.GoalIsPoleCopy);
    verifyEqual(testCase, overPole.MotionLength_units, 2, 'AbsTol', 1e-6);
    options.WrapY = "false";
    aroundAzimuth = planner([], state(0, [10, 89]), state(60, [190, 89]), limits, options);
    verifyTrue(testCase, aroundAzimuth.Success, aroundAzimuth.Message);
    verifyEqual(testCase, aroundAzimuth.MotionLength_units, 180, 'AbsTol', 1e-6);
    verifyLessThan(testCase, overPole.ArrivalTime_s, aroundAzimuth.ArrivalTime_s);

    % "backward" may only pass the lower pole, so it cannot use the top pole.
    options.WrapY = "backward";
    backward = planner([], state(0, [10, 89]), state(60, [190, 89]), limits, options);
    verifyTrue(testCase, backward.Success, backward.Message);
    verifyLessThanOrEqual(testCase, max(backward.position_units(:, 2)), 90);
    verifyGreaterThan(testCase, backward.MotionLength_units, 2);
end

function testPoleObstacleBlocksTheCrossing(testCase)
    % An obstacle just past the pole, at x = 185 to 195, y = 89.2 to 89.8,
    % has its pole copy at x = 5 to 15, y = 90.2 to 90.8, right across the
    % 2-degree crossing from (10, 89) to (10, 91). The route must go around
    % that copy, and the independent validator, which rebuilds the copies,
    % must agree.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    blocker = struct('Vertices_units', [185, 89.2; 195, 89.2; 195, 89.8; 185, 89.8]);
    result = planner(blocker, state(0, [10, 89]), state(60, [190, 89]), limits, ...
        struct('GoalTimeMode', 'earliestArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase, result.WrappedGoalCopies.GoalIsPoleCopy);
    verifyGreaterThan(testCase, result.MotionLength_units, 2);
    passesThroughCopy = inpolygon(result.position_units(:, 1), result.position_units(:, 2), ...
        [5, 15, 15, 5], [90.2, 90.2, 90.8, 90.8]);
    verifyFalse(testCase, any(passesThroughCopy));
end

function testWrapDirectionLimitsWhichEndMayBeCrossed(testCase)
    % On [0 360], "forward" may pass 360 but not 0, and "backward" may pass
    % 0 but not 360. From 350 to 10, "both" and "forward" take the 20-unit
    % way across 360; "backward" must go the 340-unit way round.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    wrapModes             = ["both", "forward", "backward", "false"];
    expectedGoals_units   = [370, 370, 10, 10];
    expectedLengths_units = [20, 20, 340, 340];
    expectedRanges_units  = [-50, 750; 0, 750; -50, 360; 0, 360];
    for modeIndex = 1:numel(wrapModes)
        result = planner([], state(0, [350, 0]), state(40, [10, 0]), limits, ...
            struct('GoalTimeMode', 'fixedArrival', 'WrapX', wrapModes(modeIndex)));
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
        verifyEqual(testCase, result.Inputs.goalState.position_units(1), expectedGoals_units(modeIndex));
        verifyEqual(testCase, result.MotionLength_units, expectedLengths_units(modeIndex), 'AbsTol', 1e-6);
        verifyEqual(testCase, result.Limits.xInterval_units, expectedRanges_units(modeIndex, :));
    end

    % The other way, from 10 to 350, "backward" crosses 0 and "forward" goes round.
    forward  = planner([], state(0, [10, 0]), state(40, [350, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "forward"));
    backward = planner([], state(0, [10, 0]), state(40, [350, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "Backward"));
    verifyEqual(testCase, forward.MotionLength_units, 340, 'AbsTol', 1e-6);
    verifyEqual(testCase, backward.MotionLength_units, 20, 'AbsTol', 1e-6);
    verifyEqual(testCase, backward.Options.WrapX, "backward");
    verifyGreaterThanOrEqual(testCase, min(forward.position_units(:, 1)), 0);
    verifyLessThanOrEqual(testCase, max(backward.position_units(:, 1)), 360);

    % true and false are still accepted and mean "both" and "false".
    legacy = planner([], state(0, [350, 0]), state(40, [10, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', true, 'WrapY', false));
    verifyEqual(testCase, legacy.Options.WrapX, "both");
    verifyEqual(testCase, legacy.Options.WrapY, "false");
    verifyEqual(testCase, legacy.MotionLength_units, 20, 'AbsTol', 1e-6);
end

function testWrappedYDisplayFoldsBackOverThePole(testCase)
    % A path from (10, 85) over the top pole to (10, 95) is shown as
    % (10, 85) -> (10, 90), then from (190, 90) down to (190, 85): the pole
    % copy of (10, 95) is (190, 85). A NaN row separates the two runs.
    displayPath_units = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 85; 10, 95], [0, 360; -90, 90], ["false", "both"]);
    verifyEqual(testCase, displayPath_units, [10, 85; 10, 90; NaN, NaN; 190, 90; 190, 85], 'AbsTol', 1e-12);
    % Modes are read like the planner's: any case, and only the four names.
    verifyEqual(testCase, obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 85; 10, 95], [0, 360; -90, 90], ["False", "BOTH"]), displayPath_units, 'AbsTol', 1e-12);
    verifyError(testCase, @() obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 85; 10, 95], [0, 360; -90, 90], ["false", "sideways"]), 'createWrappedSpatialPath:InvalidWrapMode');
end

function testPoleDisplayKeepsBoundarySamplesWithTheirSegments(testCase)
    % Touching the upper pole stays in the same display run. Crossing at a
    % recorded sample changes azimuth by half a turn and breaks exactly once.
    intervals_units = [0, 360; -90, 90];
    wrapModes = ["false", "both"];
    touching = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 11, 90; 12, 89], intervals_units, wrapModes);
    verifyEqual(testCase, touching, [10, 89; 11, 90; 12, 89], 'AbsTol', 1e-12);
    starting = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 90; 10, 89], intervals_units, wrapModes);
    verifyEqual(testCase, starting, [10, 90; 10, 89], 'AbsTol', 1e-12);
    ending = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 10, 90], intervals_units, wrapModes);
    verifyEqual(testCase, ending, [10, 89; 10, 90], 'AbsTol', 1e-12);
    xEnding = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [360, 0; 359, 0], intervals_units, wrapModes);
    verifyEqual(testCase, xEnding, [360, 0; 359, 0], 'AbsTol', 1e-12);
    multiplePoles = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 90; 10, 450], intervals_units, wrapModes);
    verifyEqual(testCase, multiplePoles, ...
        [190, 90; 190, -90; NaN, NaN; 10, -90; 10, 90], 'AbsTol', 1e-12);
    [crossing, sampleIndices] = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 11, 90; 12, 91], intervals_units, wrapModes);
    verifyEqual(testCase, crossing, [10, 89; 11, 90; NaN, NaN; 191, 90; 192, 89], 'AbsTol', 1e-12);
    verifyEqual(testCase, sampleIndices, [1; 2; 3; 3; 3]);
    verifyEqual(testCase, nnz(isnan(crossing(:, 1))), 1);

    % A wait exactly on a pole, or a run along it, keeps its neighbor's
    % copy: no jump to x = 190 and no break. The same holds at the lower
    % pole and for a wait exactly on the x seam.
    waiting = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 10, 90; 10, 90; 10, 89], intervals_units, wrapModes);
    verifyEqual(testCase, waiting, [10, 89; 10, 90; 10, 90; 10, 89], 'AbsTol', 1e-12);
    alongPole = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 90; 12, 90; 12, 89], intervals_units, wrapModes);
    verifyEqual(testCase, alongPole, [10, 90; 12, 90; 12, 89], 'AbsTol', 1e-12);
    lowerWait = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, -89; 10, -90; 10, -90; 10, -89], intervals_units, wrapModes);
    verifyEqual(testCase, lowerWait, [10, -89; 10, -90; 10, -90; 10, -89], 'AbsTol', 1e-12);
    seamWait = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [360, 0; 360, 0; 359, 0], intervals_units, ["both", "false"]);
    verifyEqual(testCase, seamWait, [360, 0; 360, 0; 359, 0], 'AbsTol', 1e-12);

    % A wait on the x seam keeps its own y band, so a pole crossing right
    % after it still breaks once, at either pole.
    seamThenPole = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [359, 89; 360, 89; 360, 91], intervals_units, wrapModes);
    verifyEqual(testCase, seamThenPole, [359, 89; 360, 89; 360, 90; NaN, NaN; 180, 90; 180, 89], 'AbsTol', 1e-12);
    seamThenLowerPole = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [359, -89; 360, -89; 360, -91], intervals_units, wrapModes);
    verifyEqual(testCase, seamThenLowerPole, [359, -89; 360, -89; 360, -90; NaN, NaN; 180, -90; 180, -89], 'AbsTol', 1e-12);

    % A long wait on the pole between two ordinary samples borrows from the
    % piece before it, sample by sample, with no jump and no break.
    waitCount = 5000;
    longWait  = [10, 89; repmat([10, 90], waitCount, 1); 10, 89];
    verifyEqual(testCase, obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        longWait, intervals_units, wrapModes), longWait, 'AbsTol', 1e-12);
end

function testPoleDisplayMarkersFollowTimedPath(testCase)
    % Start and terminal markers use the first and last folded path points.
    % A standalone fold of y = 90 would put either marker half a turn away.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    plotOptions = struct('FigureVisible', "off", 'ShowKinematics', false, ...
        'ShowAnimation', false, 'ShowVisibilityGraphs', false, ...
        'ShowSpaceTime', false, 'ShowExpandedWorkspace', false);
    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    endpointPairs_units = [10, 90, 12, 89; 10, 89, 12, 90];
    for pairIndex = 1:size(endpointPairs_units, 1)
        endpointPair_units = endpointPairs_units(pairIndex, :);
        result = planner([], state(0, endpointPair_units(1:2)), ...
            state(10, endpointPair_units(3:4)), limits, ...
            struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
        displayPath_units = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
            result.position_units, [0, 360; -90, 90], ["false", "both"]);
        handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
        startHandle = findobj(handles.WorkspaceAxes, 'DisplayName', 'Start');
        goalHandle = findobj(handles.WorkspaceAxes, 'DisplayName', 'Goal');
        verifyEqual(testCase, [startHandle.XData, startHandle.YData], displayPath_units(1, :), 'AbsTol', 1e-9);
        verifyEqual(testCase, [goalHandle.XData, goalHandle.YData], displayPath_units(end, :), 'AbsTol', 1e-9);
        if pairIndex == 2
            plotOptions.ShowWorkspace = false;
            plotOptions.ShowAnimation = true;
            plotOptions.FrameStride = 1000;
            plotOptions.Pause_s = 0;
            animationHandles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
            currentHandle = findobj(animationHandles.AnimationAxes, 'DisplayName', 'Current state');
            verifyEqual(testCase, [currentHandle.XData, currentHandle.YData], ...
                displayPath_units(end, :), 'AbsTol', 1e-9);
        end
    end
end

function testPoleMovingTargetMarkerFollowsItsTrack(testCase)
    % A target starts at the upper pole and moves below it. The workspace
    % marker uses the track's first piece, which shows (20, 90), not (200, 90).
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [20, 90; 20, 89], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], state(0, [10, 89]), goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    plotOptions = struct('FigureVisible', "off", 'ShowKinematics', false, ...
        'ShowAnimation', false, 'ShowVisibilityGraphs', false, ...
        'ShowSpaceTime', false, 'ShowExpandedWorkspace', false);
    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
    targetHandle = findobj(handles.WorkspaceAxes, 'DisplayName', 'Moving target');
    verifyEqual(testCase, [targetHandle.XData, targetHandle.YData], [20, 90], 'AbsTol', 1e-9);
end

function testWrappedMovingTargetIsUnwrappedAcrossTheSeam(testCase)
    % A target that crosses the seam is unwrapped by continuity, so the
    % intercept is planned on a two-unit move and the record keeps the
    % supplied wrapped target for the validator to lift again.
    limits=standardLimits(); limits.xInterval_units=[-90,90];
    targetMotion=struct('time_s',[0;10], ...
        'position_units',[89,0;-89,0],'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    result=planner([],state(0,[85,0]),goal,limits, ...
        struct('GoalTimeMode','fixedArrival','WrapX',true));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Inputs.goalState.targetMotion.position_units,[89,0;91,0]);
    verifyEqual(testCase,result.RequestedGoalState.targetMotion.position_units,[89,0;-89,0]);
    verifyEqual(testCase,result.Intercept.TargetPosition_units,[91,0],'AbsTol',1e-9);
    verifyEqual(testCase,result.MotionLength_units,6,'AbsTol',1e-6);
end

function testWrappedSeamCrossingTargetMatchesContinuousDerivative(testCase)
    % On x [-1 1], a target from 0.9 to -0.9 crosses the seam and is really
    % 0.9 -> 1.1 over 10 s, so its matched velocity is 0.02, not -0.18.
    % The velocity is read from the continuous path, after unwrapping.
    initial = state(0, [0.5, 0]);
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [0.9, 0; -0.9, 0], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    limits = standardLimits();
    limits.xInterval_units = [-1, 1];
    options = struct('GoalTimeMode', 'fixedArrival', 'WrapX', true, ...
        'MatchTargetVelocity', true);
    result = planner([], initial, goal, limits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Inputs.goalState.velocity_units_s, [0.02, 0], 'AbsTol', 1e-12);

    % The same velocity supplied as a column beside matching is accepted by
    % normalization and by the validator's supplied-value check.
    goal.velocity_units_s = [0.02; 0];
    column = planner([], initial, goal, limits, options);
    verifyTrue(testCase, column.Success, column.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(column).Passed);
    goal.acceleration_units_s2 = [0; 0];
    options.MatchTargetAcceleration = true;
    columns = planner([], initial, goal, limits, options);
    verifyTrue(testCase, columns.Success, columns.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(columns).Passed);
end

function testPoleCrossingTargetMatchesContinuousYDerivative(testCase)
    % From (10, 89), a target moving from (190, 88) to (190, 89) is, over the
    % top pole, the continuous path (10, 92) -> (10, 91). Its matched y
    % velocity is read from that path: -0.1, although it is +0.1 on the
    % sphere. A supplied nonzero y velocity is refused, because its sign
    % over a pole depends on where the target is met.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [190, 88; 190, 89], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    matched = planner([], state(0, [10, 89]), goal, limits, struct('GoalTimeMode', 'fixedArrival', ...
        'WrapY', "both", 'MatchTargetVelocity', true));
    verifyTrue(testCase, matched.Success, matched.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(matched).Passed);
    verifyEqual(testCase, matched.Inputs.goalState.targetMotion.position_units, [10, 92; 10, 91], 'AbsTol', 1e-12);
    verifyEqual(testCase, matched.Inputs.goalState.velocity_units_s, [0, -0.1], 'AbsTol', 1e-12);

    goal.velocity_units_s = [0, 0.1];
    verifyError(testCase, @() planner([], state(0, [10, 89]), goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both")), 'planner:UnsupportedPoleTargetDerivative');
    % A supplied value beside a matched one is refused too: the supplied
    % value is on the sphere, the matched one on the continuous copy.
    verifyError(testCase, @() planner([], state(0, [10, 89]), goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both", 'MatchTargetVelocity', true)), ...
        'planner:UnsupportedPoleTargetDerivative');
    % The validator applies the same rule to a result that claims one.
    tampered = matched;
    tampered.Options.MatchTargetVelocity = false;
    tampered.RequestedGoalState.velocity_units_s = [0, 0.1];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(tampered).Passed);
    % A planned target path must also keep the requested interpolation.
    tampered = matched;
    tampered.Inputs.goalState.targetMotion.InterpolationMethod = "pchip";
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(tampered).Passed);
    % A matched result may not also claim a supplied y velocity.
    tampered = matched;
    tampered.SuppliedGoalState.velocity_units_s = [0, -0.1];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(tampered).Passed);
end

function testWrappedValidatorRequiresBothRequestedGoalRecords(testCase)
    % A wrapped curve needs both goal records to prove that its endpoint is
    % a copy of the request and that matched values were not also supplied.
    wrapped = planner([], state(0, [5, 0]), state(10, [-5, 0]), standardLimits(), ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    plain = planner([], state(0, [0, 0]), state(10, [4, 1]), standardLimits(), ...
        struct('GoalTimeMode', 'fixedArrival'));
    verifyTrue(testCase, wrapped.Success, wrapped.Message);
    verifyTrue(testCase, plain.Success, plain.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(wrapped).Passed);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(plain).Passed);

    for fieldName = ["RequestedGoalState", "SuppliedGoalState"]
        missingWrappedField = rmfield(wrapped, fieldName);
        wrappedValidation = obstacleAvoidance.validateTrajectory(missingWrappedField);
        verifyFalse(testCase, wrappedValidation.Passed);
        verifyEqual(testCase, wrappedValidation.Message, ...
            "The result record is missing option, limit, input, or intercept fields.");

        % Non-wrapped records retain the validator's existing required set.
        missingPlainField = rmfield(plain, fieldName);
        verifyTrue(testCase, obstacleAvoidance.validateTrajectory(missingPlainField).Passed);
    end
end

function testWrappedValidatorRebuildsSuppliedGoal(testCase)
    % The wrapped endpoint record must be derived from the supplied goal.
    % A changed source value cannot be justified by an unchanged copy.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    initial = state(0, [350, 0]);
    goal = state(20, [10, 0]);
    wrapped = planner([], initial, goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyTrue(testCase, wrapped.Success, wrapped.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(wrapped).Passed);

    changedPosition = wrapped;
    changedPosition.SuppliedGoalState.position_units = [12, 0];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedPosition).Passed);
    changedDerivative = wrapped;
    changedDerivative.SuppliedGoalState.velocity_units_s = [0.1, 0];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedDerivative).Passed);
    changedTime = wrapped;
    changedTime.SuppliedGoalState.time_s = 21;
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedTime).Passed);
    missingPosition = wrapped;
    missingPosition.RequestedGoalState = rmfield(missingPosition.RequestedGoalState, 'position_units');
    malformedValidation = obstacleAvoidance.validateTrajectory(missingPosition);
    verifyFalse(testCase, malformedValidation.Passed);
    verifyEqual(testCase, malformedValidation.Message, ...
        "The requested wrapped goal does not match the supplied goal.");

    targetMotion = struct('time_s', [0; 10; 20], ...
        'position_units', [20, 0; 21, 0; 22, 0], 'InterpolationMethod', 'linear');
    movingGoal = struct('time_s', 20, 'targetMotion', targetMotion);
    moving = planner([], state(0, [10, 0]), movingGoal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, moving.Success, moving.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(moving).Passed);
    changedSample = moving;
    changedSample.SuppliedGoalState.targetMotion.position_units(2, 1) = 21.25;
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedSample).Passed);
    changedInterpolation = moving;
    changedInterpolation.SuppliedGoalState.targetMotion.InterpolationMethod = 'pchip';
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedInterpolation).Passed);
end

function testWrappedValidatorRebuildsSuppliedLimits(testCase)
    % x = 370 is a copy of x = 10 on [0 360], but not on [0 720].
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    result = planner([], state(0, [350, 0]), state(20, [10, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    changedPeriod = result;
    changedPeriod.SuppliedLimits.xInterval_units = [0, 720];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedPeriod).Passed);

    % Forward motion near the lower end uses [0 10] rather than [-6 10].
    % Changing the supplied lower end changes that directional clamp.
    forwardLimits = limits;
    forwardLimits.maxVelocity_units_s = [2, 10];
    forward = planner([], state(0, [2, 0]), state(4, [6, 0]), forwardLimits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "forward"));
    verifyTrue(testCase, forward.Success, forward.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(forward).Passed);
    verifyEqual(testCase, forward.Limits.xInterval_units, [0, 10]);
    changedEnd = forward;
    changedEnd.SuppliedLimits.xInterval_units = [1, 360];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedEnd).Passed);
end

function testWrappedValidatorRejectsMissingPlannedTargetPath(testCase)
    limits = standardLimits();
    limits.xInterval_units = [-10, 10];
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [4, 0; 6, 0], 'InterpolationMethod', 'linear');
    result = planner([], state(0, [-8, 0]), struct('time_s', 10, 'targetMotion', targetMotion), ...
        limits, struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    missingPath = result;
    missingPath.Inputs.goalState = rmfield(missingPath.Inputs.goalState, 'targetMotion');
    validation = obstacleAvoidance.validateTrajectory(missingPath);
    verifyFalse(testCase, validation.Passed);
    verifyEqual(testCase, validation.Message, "The planned wrapped target path is missing.");
    % A present but unusable path fails the record check instead of
    % throwing: a missing field, a NaN or repeated time, an unknown method.
    malformedPath = result;
    malformedPath.Inputs.goalState.targetMotion = rmfield(malformedPath.Inputs.goalState.targetMotion, 'position_units');
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(malformedPath).Passed);
    nanTime = result;
    nanTime.Inputs.goalState.targetMotion.time_s = [0; NaN];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(nanTime).Passed);
    repeatedTime = result;
    repeatedTime.Inputs.goalState.targetMotion.time_s = [0; 0];
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(repeatedTime).Passed);
    unknownMethod = result;
    unknownMethod.Inputs.goalState.targetMotion.InterpolationMethod = 'spline';
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(unknownMethod).Passed);
end

function testTargetCopyMetBeforeTheDeadlineIsTried(testCase)
    % The target sits at (190, 89) until 5 s, then moves down to (190, 80).
    % Over the top pole it is (10, 91) at first, a 2-unit move from the
    % start, but its deadline position (10, 100) is outside the y range
    % [79 99] that "forward" and a 1 unit/s y speed allow. The copy must
    % still be tried, because the target is in range before the deadline.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 1], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    targetMotion = struct('time_s', [0; 5; 10], ...
        'position_units', [190, 89; 190, 89; 190, 80], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], state(0, [10, 89]), goal, limits, struct('GoalTimeMode', 'earliestArrival', ...
        'WrapY', "forward"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThan(testCase, result.ArrivalTime_s, 10);
    verifyEqual(testCase, result.Intercept.TargetPosition_units(1), 10, 'AbsTol', 1e-9);
end

function testWrappedEarliestTrialsMatchTheirOwnTargetVelocity(testCase)
    % A wrapped target whose speed changes: each earliest-arrival trial
    % must match the target velocity at its own arrival time, not carry the
    % deadline's matched velocity as if it had been supplied.
    limits = standardLimits();
    limits.xInterval_units = [-10, 10];
    targetMotion = struct('time_s', [0; 5; 10], ...
        'position_units', [4, 0; 6, 0; 9.5, 0], 'InterpolationMethod', 'pchip');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], state(0, [-8, 0]), goal, limits, struct('GoalTimeMode', 'earliestArrival', ...
        'WrapX', "both", 'MatchTargetVelocity', true));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    [~, targetVelocity_units_s] = obstacleAvoidance.input.targetPositionAtTime( ...
        result.Inputs.goalState.targetMotion, result.ArrivalTime_s);
    verifyEqual(testCase, result.velocity_units_s(end, :), targetVelocity_units_s, 'AbsTol', 1e-6);

    % Changing both saved goal records to the deadline speed cannot excuse
    % the different speed at this earlier intercept.
    [~, deadlineVelocity_units_s] = obstacleAvoidance.input.targetPositionAtTime(targetMotion, 10);
    verifyGreaterThan(testCase, max(abs(targetVelocity_units_s - deadlineVelocity_units_s)), 1e-4);
    changedSuppliedVelocity = result;
    changedSuppliedVelocity.SuppliedGoalState.velocity_units_s = deadlineVelocity_units_s;
    changedSuppliedVelocity.RequestedGoalState.velocity_units_s = deadlineVelocity_units_s;
    verifyFalse(testCase, obstacleAvoidance.validateTrajectory(changedSuppliedVelocity).Passed);

    % A velocity the caller supplies beside matching stays a constraint at
    % every trial. An earlier trial where the target moves at another speed
    % then conflicts with it, exactly as it does without wrapping.
    goal.velocity_units_s = deadlineVelocity_units_s;
    for wrapMode = ["false", "both"]
        verifyError(testCase, @() planner([], state(0, [-8, 0]), goal, limits, ...
            struct('GoalTimeMode', 'earliestArrival', 'WrapX', wrapMode, 'MatchTargetVelocity', true)), ...
            'planner:ConflictingTargetDerivative');
    end
end

function testConstantTargetKeepsItsPoleCopyWithoutXWrap(testCase)
    % Without x wrapping, the vehicle stays in [0 360], but sphere copies
    % still repeat in x. A target held at (110, 89) has the same reachable
    % candidates as a fixed goal: (110, 89) and its pole copy (290, 91).
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    targetMotion = struct('time_s', [0; 30], ...
        'position_units', [110, 89; 110, 89], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 30, 'targetMotion', targetMotion);
    result = planner([], state(0, [10, 89]), goal, limits, struct('GoalTimeMode', 'fixedArrival', ...
        'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    fixedGoal = planner([], state(0, [10, 89]), state(30, [110, 89]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, fixedGoal.Success, fixedGoal.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(fixedGoal).Passed);
    continuousTarget = obstacleAvoidance.input.unwrapTargetPath(targetMotion, [10, 89], ...
        [0, 360; -90, 90], ["false", "both"]);
    targetCandidates_units = sortrows(continuousTarget.position_units(end, :) + ...
        result.WrappedGoalCopies.CandidateOffsets_units);
    fixedCandidates_units = sortrows([110, 89] + fixedGoal.WrappedGoalCopies.CandidateOffsets_units);
    verifyEqual(testCase, targetCandidates_units, fixedCandidates_units, 'AbsTol', 1e-9);
    verifyTrue(testCase, any(all(abs(targetCandidates_units - [110, 89]) < 1e-9, 2)));
    verifyTrue(testCase, any(all(abs(targetCandidates_units - [290, 91]) < 1e-9, 2)));
end

function testSphericalTargetKeepsContinuousAzimuthAndFixedGoalCopies(testCase)
    % The third sample is an ordinary copy at x = 362, not x = 2 or a pole
    % copy at x = 182. Only the last step crosses the pole in the display.
    intervals_units = [0, 360; -90, 90];
    targetMotion = struct('time_s', [0; 5; 10], ...
        'position_units', [179, 89; 181, 89; 2, 89], 'InterpolationMethod', 'linear');
    continuous = obstacleAvoidance.input.unwrapTargetPath(targetMotion, [359, 89], ...
        intervals_units, ["false", "both"]);
    verifyEqual(testCase, continuous.position_units, [359, 91; 361, 91; 362, 89], 'AbsTol', 1e-12);
    displayPath_units = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        continuous.position_units, intervals_units, ["false", "both"]);
    verifyEqual(testCase, displayPath_units(1, :), [179, 89], 'AbsTol', 1e-12);
    verifyEqual(testCase, displayPath_units(end, :), [2, 89], 'AbsTol', 1e-12);
    verifyEqual(testCase, nnz(isnan(displayPath_units(:, 1))), 1);

    % The lower pole uses a negative odd copy number. On x [-180 180], the
    % target samples (-1, -89), (1, -89), (-178, -89) stay only 2 units
    % apart when lifted to (179, -91), (181, -91), (182, -89).
    lowerPoleTarget = targetMotion;
    lowerPoleTarget.position_units = [-1, -89; 1, -89; -178, -89];
    lowerContinuous = obstacleAvoidance.input.unwrapTargetPath( ...
        lowerPoleTarget, [179, -89], [-180, 180; -90, 90], ["false", "both"]);
    verifyEqual(testCase, lowerContinuous.position_units, ...
        [179, -91; 181, -91; 182, -89], 'AbsTol', 1e-12);

    % The same (180, 89) position, held constant, must have the same three
    % candidate positions whether supplied as a target or a fixed goal.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    constantTarget = struct('time_s', [0; 10], 'position_units', [180, 89; 180, 89]);
    targetGoal = struct('time_s', 10, 'targetMotion', constantTarget);
    targetResult = planner([], state(0, [359, 89]), targetGoal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    fixedResult = planner([], state(0, [359, 89]), state(10, [180, 89]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, targetResult.Success, targetResult.Message);
    verifyTrue(testCase, fixedResult.Success, fixedResult.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(targetResult).Passed);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(fixedResult).Passed);
    targetCandidates_units = sortrows([360, 91] + targetResult.WrappedGoalCopies.CandidateOffsets_units);
    fixedCandidates_units = sortrows([180, 89] + fixedResult.WrappedGoalCopies.CandidateOffsets_units);
    verifyEqual(testCase, targetCandidates_units, [0, 91; 180, 89; 360, 91], 'AbsTol', 1e-9);
    verifyEqual(testCase, targetCandidates_units, fixedCandidates_units, 'AbsTol', 1e-9);
end

function testSphericalOrdinaryObstacleCopiesReachAzimuthSeam(testCase)
    % A shape two turns away at x = [718 722] still represents one at
    % [-2 2] and [358 362]. Its original and protected boundaries move
    % together; neither gets another safety margin during copying.
    square_units = [718, -1; 722, -1; 722, 1; 718, 1];
    obstacle = obstacleAvoidance.obstacles.createObstacle('remote seam', 0, ...
        {square_units(:, 1)}, {square_units(:, 2)}, 1);
    copies = obstacleAvoidance.input.copyObstaclesAcrossWraps(obstacle, ...
        [0, 360; -90, 90], ["false", "both"], [0, 360; -5, 5]);
    verifyEqual(testCase, numel(copies), 2);
    originalMinimumX_units = sort(arrayfun(@(copy) min(copy.originalX_units{1}), copies));
    protectedMinimumX_units = sort(arrayfun(@(copy) min(copy.x_units{1}), copies));
    verifyEqual(testCase, originalMinimumX_units, [-2; 358], 'AbsTol', 1e-12);
    verifyLessThan(testCase, protectedMinimumX_units(1), originalMinimumX_units(1));
    verifyEqual(testCase, diff(protectedMinimumX_units), 360, 'AbsTol', 1e-12);
    verifyTrue(testCase, all(contains([copies.targetName], "wrap copy")));

    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [5, 1], 'maxAcceleration_units_s2', [5, 2], ...
        'maxJerk_units_s3', [10, 4]);
    result = planner(obstacle, state(0, [10, 0]), state(5, [20, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.WrappedGoalCopies.ObstacleCopyCount, 2);

    % Both the folded 2D view and the expanded workspace show the same two
    % ordinary copies that planning and independent validation used.
    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off'));
    % Later copies hide their legend entries; findall still sees the shapes.
    expandedPolygons = findall(handles.ExpandedWorkspaceAxes, 'Type', 'polygon');
    workspacePolygons = findobj(handles.WorkspaceAxes, 'Type', 'polygon');
    verifyEqual(testCase, numel(expandedPolygons), 2);
    verifyEqual(testCase, numel(workspacePolygons), 4);
    expandedMinimumX_units = arrayfun(@(polygonHandle) min(polygonHandle.Shape.Vertices(:, 1)), expandedPolygons);
    verifyTrue(testCase, any(expandedMinimumX_units < 0) && any(expandedMinimumX_units > 300));
end

function testWrappedGoalWithNoReachableCopyReturnsTruthfulFailure(testCase)
    % From x = 10 at 1 unit/s for 4 s, no copy of x = 200 enters [6 14].
    % A moving target reaches no candidate either; route search must not be
    % given an invented identity copy. The obstacle in [7 8] is still
    % prepared, even though no route search will use it.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [1, 1], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    targetMotion = struct('time_s', [0; 4], 'position_units', [200, 0; 200, 0]);
    obstacle = struct('targetName', "near range", 'time_s', 0, ...
        'x_units', {{[7; 8; 8; 7]}}, 'y_units', {{[10; 10; 11; 11]}}, ...
        'safetyMargin_units', 0, 'status', "visible");
    result = planner(obstacle, state(0, [10, 0]), struct('time_s', 4, 'targetMotion', targetMotion), ...
        limits, struct('GoalTimeMode', 'earliestArrival', 'WrapX', "both"));
    verifyFailure(testCase, result, "timeWindowInfeasible");
    verifyNotEmpty(testCase, result.Message);
    verifyEmpty(testCase, result.WrappedGoalCopies.CandidateOffsets_units);
    verifyEmpty(testCase, result.WrappedGoalCopies.CandidatePlanned);
    verifyEqual(testCase, result.VisibilityGraph.SearchKind, "notSearched");
    verifyEmpty(testCase, result.Attempts);
    verifyEqual(testCase, result.Limits.xInterval_units, [6, 14]);
    verifyEqual(testCase, result.WrappedGoalCopies.ObstacleCopyCount, 1);
    verifyEqual(testCase, numel(result.PreparedObstacles), 1);
    verifyEqual(testCase, result.PreparedObstacles.targetName, "near range");
    verifyTrue(testCase, all(result.PreparedObstacles.InternalPreparation.SamplePrepared));
    verifyEqual(testCase, result.PreparedObstacles.originalX_units, result.PreparedObstacles.x_units);

    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off'));
    verifyNotEmpty(testCase, handles.ExpandedWorkspaceAxes);
    verifyEmpty(testCase, findobj(handles.ExpandedWorkspaceAxes, 'DisplayName', 'Planned goal copy'));

    % A goal velocity above its limit is reported as such, as the unwrapped
    % planner reports it, not as a time-window failure.
    fastGoal = state(1, [100, 0]);
    fastGoal.velocity_units_s = [2, 0];
    tooFast = planner([], state(0, [10, 0]), fastGoal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyFailure(testCase, tooFast, "dynamicEndpointInfeasible");
    verifyEmpty(testCase, tooFast.WrappedGoalCopies.CandidateOffsets_units);
end

function testNoGoalCopyStillReportsUnsupportedObstacleInterval(testCase)
    % A deforming obstacle whose first interval has no usable model, with a
    % goal no copy can reach: the request stops on the interval, as it does
    % without wrapping, instead of querying that geometry for the endpoints.
    collinear_units = [0 0; 0.5 0; 1 0];
    smallSquare     = [0 0; 1 0; 1 1; 0 1];
    largeSquare     = [0 0; 2 0; 2 2; 0 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('deforming', [0; 1; 2], ...
        {collinear_units(:, 1); smallSquare(:, 1); largeSquare(:, 1)}, ...
        {collinear_units(:, 2); smallSquare(:, 2); largeSquare(:, 2)}, 0);
    limits = struct('xInterval_units', [-10, 10], 'yInterval_units', [-10, 10], ...
        'maxVelocity_units_s', [1, 1], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    result = planner(obstacle, state(0.25, [0, 5]), state(0.5, [5, 5]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyFailure(testCase, result, "unsupportedObstacleInterpolation");
    verifyEmpty(testCase, result.WrappedGoalCopies.CandidateOffsets_units);
    verifyEqual(testCase, numel(result.PreparedObstacles), 1);
end

function testVisibilityNodesOnASeamStayOnTheirEdges(testCase)
    % A graph node exactly at x = 360 (a triangle's apex whose other
    % vertices, the start and the goal all lie left of it) is folded to the
    % side its edge is drawn on: x = 360, not x = 0 as a lone fold gives.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    triangle = struct('Vertices_units', [350, -2; 360, 3; 350, 8]);
    result = planner(triangle, state(0, [340, 0]), state(20, [330, 12]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, any(abs(result.VisibilityGraph.NodePosition_units(:, 1) - 360) < 1e-9));
    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off', ...
        'ShowVisibilityGraphs', true, 'ShowSearchEdges', true));
    nodeMarkers = findobj(handles.VisibilityAxes, 'DisplayName', 'Visibility node');
    nodePoints_units = [nodeMarkers.XData(:), nodeMarkers.YData(:)];
    % Each marker must be where its first incident edge, in drawing order
    % (accepted rows first), folded as the edges are drawn, ends at it.
    graph = result.VisibilityGraph;
    nodes_units = graph.NodePosition_units;
    edges = [graph.AcceptedNodeIndex; graph.RejectedNodeIndex];
    for nodeIndex = 1:size(nodes_units, 1)
        edgeRow = find(any(edges == nodeIndex, 2), 1);
        verifyNotEmpty(testCase, edgeRow);
        otherNode = edges(edgeRow, edges(edgeRow, :) ~= nodeIndex);
        if isempty(otherNode)
            otherNode = nodeIndex;
        end
        foldedEdge_units = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
            nodes_units([otherNode(1), nodeIndex], :), [0, 360; -90, 90], ["both", "false"]);
        verifyEqual(testCase, nodePoints_units(nodeIndex, :), foldedEdge_units(end, :), 'AbsTol', 1e-9);
    end
    verifyTrue(testCase, any(abs(nodePoints_units(:, 1) - 360) < 1e-9));

    % Explicit coordinates: nodes at 350, 360 and 370 with an accepted edge
    % 1-2 and a rejected edge 2-3. The seam node follows its accepted edge
    % to x = 360; node 3 follows the rejected edge over the seam to x = 10.
    synthetic = result;
    synthetic.VisibilityGraph.NodePosition_units = [350, 0; 360, 0; 370, 0];
    synthetic.VisibilityGraph.AcceptedNodeIndex  = [1, 2];
    synthetic.VisibilityGraph.RejectedNodeIndex  = [2, 3];
    syntheticHandles = obstacleAvoidance.plotting.plotTrajectory(synthetic, struct('FigureVisible', 'off', ...
        'ShowVisibilityGraphs', true, 'ShowSearchEdges', true));
    syntheticMarkers = findobj(syntheticHandles.VisibilityAxes, 'DisplayName', 'Visibility node');
    verifyEqual(testCase, [syntheticMarkers.XData(:), syntheticMarkers.YData(:)], ...
        [350, 0; 360, 0; 10, 0], 'AbsTol', 1e-9);
end

function testExpandedPlotDrawsTargetOverThePlannedWindow(testCase)
    % A target recorded past the deadline is drawn only up to it, the same
    % window its copies are listed for: from 350 at 0 s to 370 at 10 s,
    % not on to 390 at 20 s.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    targetMotion = struct('time_s', [0; 10; 20], 'position_units', [350, 0; 10, 0; 30, 0], ...
        'InterpolationMethod', 'linear');
    result = planner([], state(0, [340, 0]), struct('time_s', 10, 'targetMotion', targetMotion), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off'));
    targetLine = findobj(handles.ExpandedWorkspaceAxes, 'DisplayName', 'Moving target (unwrapped)');
    verifyNotEmpty(testCase, targetLine);
    verifyEqual(testCase, [min(targetLine.XData), max(targetLine.XData)], [350, 370], 'AbsTol', 1e-9);
end

function testWrappedNoGoalCopyStillRejectsInvalidObstacle(testCase)
    % An unreachable target does not excuse a negative stored margin. The
    % wrapped request must raise the same input error as a direct request.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [1, 1], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    invalidObstacle = struct('targetName', "invalid margin", 'time_s', 0, ...
        'x_units', {{[7; 8; 8; 7]}}, 'y_units', {{[10; 10; 11; 11]}}, ...
        'safetyMargin_units', -1, 'status', "visible");
    unwrappedErrorId = "";
    try
        planner(invalidObstacle, state(0, [10, 0]), state(4, [12, 0]), limits, ...
            struct('GoalTimeMode', 'fixedArrival'));
    catch exception
        unwrappedErrorId = string(exception.identifier);
    end
    verifyNotEqual(testCase, unwrappedErrorId, "");

    targetMotion = struct('time_s', [0; 4], 'position_units', [200, 0; 200, 0]);
    goal = struct('time_s', 4, 'targetMotion', targetMotion);
    verifyError(testCase, @() planner(invalidObstacle, state(0, [10, 0]), goal, limits, ...
        struct('GoalTimeMode', 'earliestArrival', 'WrapX', "both")), char(unwrappedErrorId));
end

function testWrappedStandardObstacleWithoutOriginalsCopiesAcrossAzimuth(testCase)
    % A zero-margin standard record may omit its original-boundary fields.
    % A shape straddling x = 0 appears again at x = 360 on the sphere.
    obstacle = struct('targetName', "west seam", 'time_s', 0, ...
        'x_units', {{[-2; 2; 2; -2]}}, 'y_units', {{[-1; -1; 1; 1]}}, ...
        'safetyMargin_units', 0, 'status', "visible");
    intervals_units = [0, 360; -90, 90];
    range_units = [0, 360; -5, 5];
    copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        obstacle, intervals_units, ["false", "both"], range_units);
    verifyEqual(testCase, numel(copies), 2);
    verifyEqual(testCase, sort(arrayfun(@(copy) min(copy.x_units{1}), copies)), ...
        [-2; 358], 'AbsTol', 1e-12);
    verifyTrue(testCase, all(arrayfun(@(copy) isequal(copy.x_units, copy.originalX_units) && ...
        isequal(copy.y_units, copy.originalY_units), copies)));

    % The x-only identity copy now has normalized fields. Its geometry and
    % name match the source, and preparation accepts that normalized record.
    xOnlyCopies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        obstacle, intervals_units, ["both", "false"], range_units);
    identityIndex = find([xOnlyCopies.targetName] == "west seam");
    verifyEqual(testCase, numel(identityIndex), 1);
    verifyEqual(testCase, xOnlyCopies(identityIndex).WrapTransform, [0, 1, 0]);
    verifyEqual(testCase, rmfield(xOnlyCopies(identityIndex), 'WrapTransform'), ...
        obstacleAvoidance.obstacles.createObstacle(obstacle));
    verifyFalse(testCase, isequaln(xOnlyCopies(identityIndex), obstacle));
    prepared = obstacleAvoidance.obstacles.prepareObstacles(xOnlyCopies, [0, 5], true);
    verifyTrue(testCase, all(arrayfun(@(copy) all(copy.InternalPreparation.SamplePrepared), prepared)));

    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [5, 1], 'maxAcceleration_units_s2', [5, 2], ...
        'maxJerk_units_s3', [10, 4]);
    result = planner(obstacle, state(0, [10, 0]), state(5, [20, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
end

function testStandardObstacleWithoutOriginalsMakesLowerPoleCopy(testCase)
    % Mirroring the rectangle at the lower pole maps x [188 192] to [8 12]
    % and y [-89 -87] to [-93 -91], with no second safety margin.
    obstacle = struct('targetName', "lower pole", 'time_s', 0, ...
        'x_units', {{[188; 192; 192; 188]}}, 'y_units', {{[-89; -89; -87; -87]}}, ...
        'safetyMargin_units', 0, 'status', "visible");
    copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        obstacle, [0, 360; -90, 90], ["false", "both"], [0, 360; -94, -84]);
    poleIndex = find(contains([copies.targetName], "pole copy"));
    verifyEqual(testCase, numel(poleIndex), 1);
    poleCopy = copies(poleIndex);
    verifyEqual(testCase, min(poleCopy.x_units{1}), 8, 'AbsTol', 1e-12);
    verifyEqual(testCase, [min(poleCopy.y_units{1}), max(poleCopy.y_units{1})], ...
        [-93, -91], 'AbsTol', 1e-12);
    verifyEqual(testCase, poleCopy.originalX_units, poleCopy.x_units);
    verifyEqual(testCase, poleCopy.originalY_units, poleCopy.y_units);

    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [5, 1], 'maxAcceleration_units_s2', [5, 2], ...
        'maxJerk_units_s3', [10, 4]);
    result = planner(obstacle, state(0, [10, -89]), state(5, [20, -89]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.WrappedGoalCopies.ObstacleCopyCount, 2);
end

function testWrappedPlotUsesTiedObstacleSnapshots(testCase)
    % The obstacle can be prepared and the motion validated over [0 5] s.
    % It sits near y = 90, outside this request's y reach of [-5 5], but
    % the folded 2D display shows its original and protected snapshots.
    square_units  = [-1, -1; 1, -1; 1, 1; -1, 1] + [100, 90];
    diamond_units = [-2, 0; 0, -2; 2, 0; 0, 2] + [100, 90];
    tied = obstacleAvoidance.obstacles.createObstacle('display tie', [0; 5], ...
        {square_units(:, 1); diamond_units(:, 1)}, ...
        {square_units(:, 2); diamond_units(:, 2)}, 0.5);
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 1], 'maxAcceleration_units_s2', [10, 2], ...
        'maxJerk_units_s3', [20, 4]);
    result = planner(tied, state(0, [10, 0]), state(5, [20, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Limits.yInterval_units, [-5, 5]);
    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off'));
    verifyNotEmpty(testCase, handles.WorkspaceAxes);
    verifyNotEmpty(testCase, handles.SpaceTimeAxes);
    originalPolygons = findobj(handles.WorkspaceAxes, 'Type', 'polygon', 'DisplayName', 'Original obstacle');
    protectedPolygons = findobj(handles.WorkspaceAxes, 'Type', 'polygon', 'DisplayName', 'Protected obstacle');
    verifyNotEmpty(testCase, originalPolygons);
    verifyNotEmpty(testCase, protectedPolygons);
    verifyGreaterThan(testCase, area(protectedPolygons(1).Shape), area(originalPolygons(1).Shape));
end

function testTargetPathStaysContinuousPastANonWrappingXEnd(testCase)
    % Without x wrapping, only the first target sample is kept inside the x
    % interval. From start (359, 89), (179, 89) -> (181, 89) is a 2-unit move
    % in azimuth near the pole: (359, 91) -> (361, 91). Folding the second
    % sample back to (181, 89) would send the target through the pole.
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [179, 89; 181, 89], 'InterpolationMethod', 'linear');
    unwrapped = obstacleAvoidance.input.unwrapTargetPath(targetMotion, [359, 89], ...
        [0, 360; -90, 90], ["false", "both"]);
    verifyEqual(testCase, unwrapped.position_units, [359, 91; 361, 91], 'AbsTol', 1e-12);

    % The vehicle can meet it before it passes x = 360.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    earliest = planner([], state(0, [359, 89]), goal, limits, struct('GoalTimeMode', 'earliestArrival', ...
        'WrapY', "both"));
    verifyTrue(testCase, earliest.Success, earliest.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(earliest).Passed);
    verifyLessThanOrEqual(testCase, max(earliest.position_units(:, 1)), 360);

    % At the deadline it is at (361, 91), past the x end; its only copy
    % inside, (181, 89), is too far for 10 s. A fixed arrival reports that,
    % not an endpoint outside the workspace.
    fixed = planner([], state(0, [359, 89]), goal, limits, struct('GoalTimeMode', 'fixedArrival', ...
        'WrapY', "both"));
    verifyFailure(testCase, fixed, "timeWindowInfeasible");
end

function testTiedMovingObstacleNeedsDeclaredMatchingForPoleCopy(testCase)
    % A square turning into a diamond has two equally good vertex matches.
    % The tie is broken by vertex position, which a mirror does not keep,
    % so a pole copy cannot follow the same motion unless the matching is
    % declared by vertex order.
    square_units  = [-1, -1; 1, -1; 1, 1; -1, 1] + [100, 85];
    diamond_units = [-2, 0; 0, -2; 2, 0; 0, 2] + [100, 85];
    intervals_units = [0, 360; -90, 90];
    range_units     = [0, 360; 60, 120];
    tied = obstacleAvoidance.obstacles.createObstacle("tied square", [0; 10], ...
        {square_units(:, 1); diamond_units(:, 1)}, {square_units(:, 2); diamond_units(:, 2)}, 0);
    verifyError(testCase, @() obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        tied, intervals_units, ["false", "both"], range_units), 'planner:AmbiguousPoleCopyMotion');
    % A repeated closing vertex in one sample does not hide the tie:
    % preparation removes it before matching, and so does the check.
    closedTied = tied;
    closedTied.x_units{1} = [closedTied.x_units{1}; closedTied.x_units{1}(1)];
    closedTied.y_units{1} = [closedTied.y_units{1}; closedTied.y_units{1}(1)];
    closedTied.originalX_units{1} = [closedTied.originalX_units{1}; closedTied.originalX_units{1}(1)];
    closedTied.originalY_units{1} = [closedTied.originalY_units{1}; closedTied.originalY_units{1}(1)];
    verifyError(testCase, @() obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        closedTied, intervals_units, ["false", "both"], range_units), 'planner:AmbiguousPoleCopyMotion');
    % A sample outside the request's times sets no copy bounds either. The
    % square-to-diamond interval [0 5] s is tied; at 20 s the shape sits at
    % y [88 90], whose pole copy [90 92] enters a range up to y = 92 while
    % the early shapes' pole copies (y >= 93) do not. A request over [0 5] s
    % uses only the early samples, so it lists one copy and nothing throws;
    % a request over [0 20] s lists the pole copy and hits the tie.
    nearPole_units = square_units + [0, 4];
    laterNearPole = obstacleAvoidance.obstacles.createObstacle("later near pole", [0; 5; 20], ...
        {square_units(:, 1); diamond_units(:, 1); nearPole_units(:, 1)}, ...
        {square_units(:, 2); diamond_units(:, 2); nearPole_units(:, 2)}, 0);
    verifyEqual(testCase, numel(obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        laterNearPole, intervals_units, ["false", "both"], [0, 360; 60, 92], [0, 5])), 1);
    verifyError(testCase, @() obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        laterNearPole, intervals_units, ["false", "both"], [0, 360; 60, 92], [0, 20]), ...
        'planner:AmbiguousPoleCopyMotion');
    % An obstacle recorded only after the request's times is inactive
    % during them (see obstacle_history_contract.md), so it gets no copies
    % and its tie is never checked.
    laterTied = obstacleAvoidance.obstacles.createObstacle("later tied square", [20; 30], ...
        {square_units(:, 1); diamond_units(:, 1)}, {square_units(:, 2); diamond_units(:, 2)}, 0);
    verifyEmpty(testCase, obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        laterTied, intervals_units, ["false", "both"], range_units, [0, 5]));
    % A tied interval that only touches the request's start or deadline is
    % not used either: square, square, then diamond at 0, 5, and 10 s.
    touchingTied = obstacleAvoidance.obstacles.createObstacle("touching tied square", [0; 5; 10], ...
        {square_units(:, 1); square_units(:, 1); diamond_units(:, 1)}, ...
        {square_units(:, 2); square_units(:, 2); diamond_units(:, 2)}, 0);
    for timeRange_s = {[0, 5], [10, 15]}
        verifyEqual(testCase, numel(obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
            touchingTied, intervals_units, ["false", "both"], range_units, timeRange_s{1})), 2);
    end
    verifyError(testCase, @() obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        touchingTied, intervals_units, ["false", "both"], range_units, [5, 10]), ...
        'planner:AmbiguousPoleCopyMotion');
    % Shifting x keeps the tie-break, so x wrapping alone is fine.
    verifyEqual(testCase, numel(obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        tied, intervals_units, ["both", "false"], [-360, 720; -90, 90])), 3);
    declared = obstacleAvoidance.obstacles.createObstacle("declared square", [0; 10], ...
        {square_units(:, 1); diamond_units(:, 1)}, {square_units(:, 2); diamond_units(:, 2)}, 0, ...
        struct('vertexCorrespondence', "sourceIndex"));
    copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        declared, intervals_units, ["false", "both"], range_units);
    verifyEqual(testCase, numel(copies), 2);
    verifyEqual(testCase, copies(2).y_units{2}, 180 - copies(1).y_units{2}, 'AbsTol', 1e-12);
    % The declared rule decides, even on a record without the derived flag.
    declaredRecord = rmfield(declared, 'UsesSourceIndex');
    verifyEqual(testCase, numel(obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
        declaredRecord, intervals_units, ["false", "both"], range_units)), 2);
end

function testExpandedWorkspacePlotShowsWrappedCopies(testCase)
    % With wrapping on, the plot adds an expanded workspace in the planner's
    % unwrapped coordinates showing the obstacle and its pole copy.
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
    blocker = struct('Vertices_units', [185, 89.2; 195, 89.2; 195, 89.8; 185, 89.8]);
    plotOptions = struct('FigureVisible', "off", 'ShowAnimation', false, ...
        'ShowKinematics', false, 'ShowSpaceTime', true);
    wrapped = planner(blocker, state(0, [10, 89]), state(60, [190, 89]), limits, ...
        struct('GoalTimeMode', 'earliestArrival', 'WrapY', "both"));
    handles = obstacleAvoidance.plotting.plotTrajectory(wrapped, plotOptions);
    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    verifyNotEmpty(testCase, handles.ExpandedWorkspaceAxes);
    obstaclePolygons = findobj(handles.ExpandedWorkspaceAxes, 'Type', 'polygon');
    verifyGreaterThanOrEqual(testCase, numel(obstaclePolygons), 2);
    unwrapped = planner(blocker, state(0, [10, 89]), state(60, [190, 89]), limits, ...
        struct('GoalTimeMode', 'earliestArrival'));
    plainHandles = obstacleAvoidance.plotting.plotTrajectory(unwrapped, plotOptions);
    verifyEmpty(testCase, plainHandles.ExpandedWorkspaceAxes);
    % A saved record with a wrap value the planner would refuse is not drawn.
    badWrap = wrapped;
    badWrap.Options.WrapX = 2;
    verifyError(testCase, @() obstacleAvoidance.plotting.plotTrajectory(badWrap, plotOptions), ...
        'plotTrajectory:InvalidWrapOption');

    % In the 2D workspace an obstacle across the x seam shows on both sides:
    % [355 365] also appears at [-5 5].
    seamObstacle = struct('Vertices_units', [355, -5; 365, -5; 365, 5; 355, 5]);
    seamResult = planner(seamObstacle, state(0, [340, 0]), state(60, [20, 30]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    seamHandles = obstacleAvoidance.plotting.plotTrajectory(seamResult, plotOptions);
    polygonHandles = findobj(seamHandles.WorkspaceAxes, 'Type', 'polygon');
    minimumX_units = arrayfun(@(polygonHandle) min(polygonHandle.Shape.Vertices(:, 1)), polygonHandles);
    verifyTrue(testCase, any(minimumX_units < 0) && any(minimumX_units > 300));
end

function testExpandedPlotAcceptsUnequalRowHistories(testCase)
    % The second moving sample repeats its closing vertex. Its five-value
    % row cannot be stacked with the first sample's four-value row, but
    % obstacle normalization removes the repeat before making copies.
    obstacle = struct('targetName', "row history", 'time_s', [0; 5], ...
        'safetyMargin_units', 0, 'status', ["visible"; "visible"]);
    obstacle.x_units = {[-2, 2, 2, -2]; [-1, 3, 3, -1, -1]};
    obstacle.y_units = {[-1, -1, 1, 1]; [-1, -1, 1, 1, -1]};
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [5, 1], 'maxAcceleration_units_s2', [5, 2], ...
        'maxJerk_units_s3', [10, 4]);
    result = planner(obstacle, state(0, [10, 0]), state(5, [20, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);

    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', "off"));
    polygons = findall(handles.ExpandedWorkspaceAxes, 'Type', 'polygon');
    minimumX_units = arrayfun(@(polygonHandle) min(polygonHandle.Shape.Vertices(:, 1)), polygons);
    original = polygons(minimumX_units < 10);
    shifted = polygons(minimumX_units > 300);
    verifyEqual(testCase, numel(polygons), 2);
    verifyEqual(testCase, string(original.LineStyle), "-");
    verifyEqual(testCase, string(original.DisplayName), "Obstacle");
    verifyEqual(testCase, string(shifted.LineStyle), "--");
    verifyEqual(testCase, string(shifted.DisplayName), "Shifted obstacle copy");
end

function testExpandedPlotStylesCopiesAfterDroppingOutlyingRing(testCase)
    % A two-point region at x = 1000 is removed from the source shape.
    % Using its raw bounds would add false offsets and style the identity
    % copy as shifted. The real shape has identity, shifted, and pole copies.
    obstacle = struct('targetName', "outlying ring", 'time_s', 0, ...
        'safetyMargin_units', 0, 'status', "visible");
    obstacle.x_units = {[-3; 1; 1; -3; NaN; 1000; 1001]};
    obstacle.y_units = {[88; 88; 89; 89; NaN; 88; 89]};
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [5, 1], 'maxAcceleration_units_s2', [5, 2], ...
        'maxJerk_units_s3', [10, 4]);
    result = planner(obstacle, state(0, [10, 89]), state(5, [20, 89]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.WrappedGoalCopies.ObstacleCopyCount, 3);

    figureCleanup = onCleanup(@() close(findall(0, 'Type', 'figure', 'Visible', 'off')));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', "off"));
    polygons = findall(handles.ExpandedWorkspaceAxes, 'Type', 'polygon');
    minimumX_units = arrayfun(@(polygonHandle) min(polygonHandle.Shape.Vertices(:, 1)), polygons);
    original = polygons(minimumX_units < 10);
    pole = polygons(minimumX_units > 150 & minimumX_units < 200);
    shifted = polygons(minimumX_units > 300);
    verifyEqual(testCase, numel(polygons), 3);
    verifyEqual(testCase, string(original.LineStyle), "-");
    verifyEqual(testCase, string(original.DisplayName), "Obstacle");
    verifyEqual(testCase, string(shifted.LineStyle), "--");
    verifyEqual(testCase, string(shifted.DisplayName), "Shifted obstacle copy");
    verifyEqual(testCase, string(pole.LineStyle), ":");
    verifyEqual(testCase, string(pole.DisplayName), "Pole obstacle copy");
end

function testWrappedWindingTargetRejectsCoincidentImageEndpoint(testCase)
    initial=state(0,[0,0]);
    targetMotion=struct('time_s',(0:2:10)', ...
        'position_units',[[0;0.4;0.8;-0.8;-0.4;0],zeros(6,1)], ...
        'InterpolationMethod','linear');
    goal=struct('time_s',10,'targetMotion',targetMotion);
    limits=struct('xInterval_units',[-1,1],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options=struct('GoalTimeMode','fixedArrival','WrapX',true);
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
    % A wrapped request is planned as plain requests in the unwrapped coordinates,
    % so the returned record has to declare the parent request it was accepted
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

    % Requested limits are normalized but still the wrapped workspace, while
    % the effective limits are the unwrapped reachable range the copies live in.
    verifyEqual(testCase,result.RequestedLimits.xInterval_units,[-180,180]);
    verifyEqual(testCase,result.Limits.xInterval_units,[159,199]);
    verifyEqual(testCase,result.RequestedGoalState.position_units,[-179,0]);

    % The effective goal is the selected copy, but its clock is the outer
    % horizon. Position and time on this one struct have different owners.
    verifyEqual(testCase,result.Inputs.goalState.position_units,[181,0]);
    verifyEqual(testCase,result.Inputs.goalState.time_s,10);

    % Wrapping and arrival mode are declared as the parent request, not as the
    % plain unwrapped request each copy was actually planned as.
    verifyEqual(testCase,result.Options.WrapX,"both");
    verifyEqual(testCase,result.Options.WrapY,"false");
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
    verifyEqual(testCase,fieldnames(staticResult.SeparationProof.Coverage), ...
        {'ExactRegionCount'});

    box=[-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    first=box+[0,8];
    last=box+[2,8];
    mover=obstacleAvoidance.obstacles.createObstacle('remote mover',[0;20], ...
        {first(:,1);last(:,1)},{first(:,2);last(:,2)},0);

    fixedResult=planner(mover,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,fixedResult.Success,fixedResult.Message);
    verifyEqual(testCase,fieldnames(fixedResult.SeparationProof.Coverage), ...
        {'ExactRegionCount';'ActiveTimeInterval_s';'EndRegions_units';'BreakTime_s'});

    % BreakTime_s is the one that separates two dynamic runs from each other.
    earliestResult=planner(mover,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,earliestResult.Success,earliestResult.Message);
    verifyEqual(testCase,fieldnames(earliestResult.SeparationProof.Coverage), ...
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
    verifyEqual(testCase,result.EarliestArrival.BestSoFarArrival_s, ...
        result.ArrivalTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.Attempts(selectedIndex).CandidateArrival_s, ...
        result.ArrivalTime_s,'AbsTol',1e-12);
end

function testEquivalentSparseAndDenseHistoriesUseProvenDeparture(testCase)
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
        struct('WrapX',2)),'planner:InvalidWrapOption');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('WrapY',"sideways")),'planner:InvalidWrapOption');
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
        struct('BestSoFarRefinementTrialLimit',-1)),'MATLAB:expectedNonnegative');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('BestSoFarRefinementTrialLimit',1.5)),'MATLAB:expectedInteger');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('SpatialProbeIterationLimit',0)),'MATLAB:expectedPositive');
    verifyError(testCase,@()planner([],initial,goal,limits, ...
        struct('SpatialProbeIterationLimit',36)), ...
        'planner:InvalidSpatialProbeIterationLimit');
    defaulted=planner([],initial,goal,limits,struct('SampleTime_s',[]));
    verifyTrue(testCase,defaulted.Success,defaulted.Message);
    verifyEqual(testCase,defaulted.Options.SampleTime_s,0.05);
    verifyEqual(testCase,defaulted.Options.BestSoFarRefinementTrialLimit,0);
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
