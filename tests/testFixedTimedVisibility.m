function tests = testFixedTimedVisibility
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testFixedTimedVisibility.m')
% PURPOSE: Verify unified fixed-arrival seed selection and physical validation.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Fixed-clock motion, unsupported request, and expected failure checks.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Initial=struct('time_s',0,'position_units',[0,0]);
    testCase.TestData.Goal=struct('time_s',10,'position_units',[5,0]);
    testCase.TestData.Limits=struct('xInterval_units',[-1,6],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
    testCase.TestData.Options=struct('GoalTimeMode','fixedArrival');
end

function testPrescribedArrivalAndContinuousMotion(testCase)
    data=testCase.TestData;
    result=planner([],data.Initial,data.Goal,data.Limits,data.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,10,'AbsTol',1e-8);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,result.VisibilityGraph.GraphIsFullyEnumerated);
    verifyLessThan(testCase,size(result.Route_units,1),9);
end

function testMovingTargetEndsAtExactPrescribedClock(testCase)
    goalTime_s = 13.984378262112314;
    initial = struct('time_s',0.45999999999999996,'position_units',[-4,0]);
    targetMotion = struct( ...
        'time_s',[initial.time_s;goalTime_s], ...
        'position_units',[4,0;5,0]);
    goal = struct('time_s',goalTime_s,'targetMotion',targetMotion);
    limits = struct( ...
        'xInterval_units',[-20,20], ...
        'yInterval_units',[-8,8], ...
        'maxVelocity_units_s',[3,3], ...
        'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options = struct('GoalTimeMode','fixedArrival','WrapX',true);

    result = planner([],initial,goal,limits,options);

    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,goal.time_s);
    verifyEqual(testCase,result.Polynomial.FinalTime_s,goal.time_s);
    verifyEqual(testCase,result.time_s(end),goal.time_s);
end

function testStaticGoalUsesExactPrescribedClock(testCase)
    % This clock pair reproduces one ulp late when the arrival is rebuilt
    % from the start time plus summed durations.
    initial = struct('time_s',0.45999999999999996,'position_units',[0,0]);
    goal = struct('time_s',13.984378262112314,'position_units',[5,1]);
    limits = struct( ...
        'xInterval_units',[-1,6], ...
        'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);

    result = planner([],initial,goal,limits,struct('GoalTimeMode','fixedArrival'));

    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,goal.time_s);
    verifyEqual(testCase,result.Polynomial.FinalTime_s,goal.time_s);
    verifyEqual(testCase,result.time_s(end),goal.time_s);
end

function testNoSilentSpatialNextMethod(testCase)
    data=testCase.TestData;
    wall=struct('Vertices_units',[2,-3;3,-3;3,3;2,3]);
    result=planner(wall,data.Initial,data.Goal,data.Limits,data.Options);
    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"noVisibilityRoute");
    verifyEmpty(testCase,result.time_s);
end

function testFixedTimedSearchRetainsBoundaryVelocity(testCase)
    data=testCase.TestData;
    data.Initial.velocity_units_s=[0.1,0];
    result=planner([],data.Initial,data.Goal,data.Limits,data.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Validation.Passed);
    verifyEqual(testCase,result.velocity_units_s([1,end],:),[0.1,0;0,0], ...
        'AbsTol',1e-8);
end

function testSavedDetourUsesPrescribedDeadline(testCase)
    root=fileparts(mfilename('fullpath'));
    request=jsondecode(fileread(fullfile(root,'fixtures','savedMovingDetour.json')));
    sources=cell(numel(request.obstacles),1);
    for k=1:numel(sources)
        source=request.obstacles(k); frames=source.keyframes;
        sources{k}=obstacleAvoidance.obstacles.createObstacle(source.name,[frames.time_s].', ...
            arrayfun(@(f)f.vertices_units(:,1),frames,'UniformOutput',false), ...
            arrayfun(@(f)f.vertices_units(:,2),frames,'UniformOutput',false),source.safetyMargin_units);
    end
    options=request.options;
    options.GoalTimeMode='fixedArrival';
    result=planner(obstacleAvoidance.obstacles.combineObstacles(sources), ...
        request.initialState,request.goalState,request.limits,options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,180,'AbsTol',1e-8);
    % The unified spatial result stays within one percent of the former
    % explicitly selected timed motion (229.400575729 units).
    verifyLessThan(testCase,result.MotionLength_units,232);
    verifyGreaterThan(testCase,result.SolverDiagnostics.OptimizerSpanCount,16);
end

function testMovingCrossingRetainsProvenEndpointJerk(testCase)
    missionEndTime_s=12;
    time_s=linspace(0,missionEndTime_s,5).';
    halfSize_units=[0.73318128921311621,0.95945964014883089];
    initialAngle_rad=-0.46430464622215828;
    safetyMargin_units=0.14507508417445217;
    centerX_units=1.7787935948757139;
    center_units=[repmat(centerX_units,5,1),linspace(-3.8,3.8,5).'];
    angle_rad=initialAngle_rad+linspace(0,pi/3,5).';
    local_units=[-1,-1;1,-1;1,1;-1,1].*halfSize_units;
    xByTime_units=cell(5,1); yByTime_units=cell(5,1);
    for sampleIndex=1:5
        rotation=[cos(angle_rad(sampleIndex)),-sin(angle_rad(sampleIndex)); ...
            sin(angle_rad(sampleIndex)),cos(angle_rad(sampleIndex))];
        boundary_units=local_units*rotation.'+center_units(sampleIndex,:);
        xByTime_units{sampleIndex}=boundary_units(:,1);
        yByTime_units{sampleIndex}=boundary_units(:,2);
    end
    obstacle=obstacleAvoidance.obstacles.createObstacle('moving crossing regression', ...
        time_s,xByTime_units,yByTime_units,safetyMargin_units);
    initial=struct('time_s',0,'position_units',[-6,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    goal=struct('time_s',missionEndTime_s,'position_units',[6,0], ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
    limits=struct('xInterval_units',[-9,9],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[3,3],'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options=struct('GoalTimeMode','fixedArrival', ...
        'SampleTime_s',0.05, ...
        'TemporalResolution_s',0.75);
    result=planner(obstacle,initial,goal,limits,options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,missionEndTime_s,'AbsTol',1e-8);
    verifyLessThanOrEqual(testCase,max(abs(result.jerk_units_s3),[],1), ...
        limits.maxJerk_units_s3+result.Options.ConstraintTolerance);
end

function testArrivalSnapshotAvoidsTimedSearch(testCase)
    scenario=createRandomAzimuthScenario(26,true);
    scenario.Options.SpatialProbeIterationLimit=2;
    result=planner(scenario.Obstacles,scenario.InitialState, ...
        scenario.GoalState,scenario.Limits,scenario.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "arrivalSpatialSnapshot");
    verifyEqual(testCase,numel(result.Attempts),2);
    verifyEqual(testCase,[result.Attempts.IsShortcut],[true,true]);
    verifyEqual(testCase,[result.Attempts.IterationLimit],[2,2]);
    verifyEqual(testCase,result.Options.SpatialProbeIterationLimit,2);
    verifyEqual(testCase,result.Attempts(1).FailureKind,"iterationLimit");
    verifyTrue(testCase,result.Attempts(1).NextAttemptAllowed);
    verifyGreaterThan(testCase,size(result.Route_units,1),2);
end

function testTimedSearchCrossesARecurrentCurtain(testCase)
    wall=[-0.2,-7;0.2,-7;0.2,7;-0.2,7];
    moved=wall+[0,14];
    obstacle=obstacleAvoidance.obstacles.createObstacle('recurrent curtain', ...
        [0;1;1.1;4;4.1;12], ...
        {wall(:,1);wall(:,1);moved(:,1);moved(:,1);wall(:,1);wall(:,1)}, ...
        {wall(:,2);wall(:,2);moved(:,2);moved(:,2);wall(:,2);wall(:,2)},0);
    initial=struct('time_s',0,'position_units',[-4,0]);
    goal=struct('time_s',12,'position_units',[4,0]);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[4,4],'maxAcceleration_units_s2',[4,4], ...
        'maxJerk_units_s3',[8,8]);
    options=struct('GoalTimeMode','fixedArrival','TemporalResolution_s',0.25);

    result=planner(obstacle,initial,goal,limits,options);

    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Validation.Passed);
    verifyEqual(testCase,result.Message, ...
        "The prescribed goal layer produced an independently validated timed BMTP motion.");
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyEqual(testCase,result.FixedArrivalTrialTime_s,goal.time_s);
    verifyFalse(testCase,isfield(result,'GoalArrivalWindow_s'));
    verifyFalse(testCase,isfield(result,'TrajectoryCoverageEndTime_s'));
    verifyEqual(testCase,numel(result.Attempts),3);
    verifyFalse(testCase,any([result.Attempts(1:2).SolverAttempted]));
end

function testFailedTimedSearchDoesNotLeakSpatialState(testCase)
    wall=[-0.2,-7;0.2,-7;0.2,7;-0.2,7];
    shifted=wall+[0.1,0];
    obstacle=obstacleAvoidance.obstacles.createObstacle('persistent curtain', ...
        [0;6;12],{wall(:,1);shifted(:,1);wall(:,1)}, ...
        {wall(:,2);shifted(:,2);wall(:,2)},0);
    initial=struct('time_s',0,'position_units',[-4,0]);
    goal=struct('time_s',12,'position_units',[4,0]);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[4,4],'maxAcceleration_units_s2',[4,4], ...
        'maxJerk_units_s3',[8,8]);
    options=struct('GoalTimeMode','fixedArrival','TemporalResolution_s',0.25);

    result=planner(obstacle,initial,goal,limits,options);

    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"noTimedRoute");
    verifyEqual(testCase,result.Message, ...
        "No route reached the requested goal layer in the discrete timed graph.");
    verifyEqual(testCase,result.Validation,struct( ...
        'Passed',false,'Message',"No timed motion is available."));
    verifyEqual(testCase,result.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyFalse(testCase,result.VisibilityGraph.IsConnected);
    verifyEmpty(testCase,result.Route_units);
    verifyEmpty(testCase,result.time_s);
    verifyEqual(testCase,numel(result.Attempts),3);
    verifyEqual(testCase,result.Attempts(end).FailureStage,"search");
    verifyEqual(testCase,result.Attempts(end).FailureKind,"noTimedRoute");
    verifyEqual(testCase,fieldnames(result),timedFailureFieldNames());
    verifyEqual(testCase,fieldnames(result.VisibilityGraph),timedGraphFieldNames());
    verifyEqual(testCase,result.MotionLength_units,Inf);
    verifyEqual(testCase,result.IntegratedSquaredJerk_units2_s5,Inf);
    verifyEqual(testCase,result.MaximumConstraintViolation,Inf);
    verifyFalse(testCase,result.OptimizerFeasible);
    verifyFalse(testCase,result.OptimizerIterateUnavailable);
    verifyFalse(testCase,result.AlternativeGuideEligible);
    verifyFalse(testCase,isfield(result,'EarliestArrival'));
    verifyFalse(testCase,isfield(result,'WrappedGoalCopies'));
end

function testTimedResultFieldOrderDoesNotDependOnTheRouteTaken(testCase)
    % A timed result reaches the finalizer by two histories: with no preceding
    % spatial BMTP candidate, and after a failed one. The record is assembled
    % from one constructor, so the field order must not depend on which history
    % produced it. Nothing reads field order today, but a record whose shape
    % varies by route is a schema that cannot be relied on.
    wall=[-0.2,-7;0.2,-7;0.2,7;-0.2,7];
    moved=wall+[0,14];
    curtain=obstacleAvoidance.obstacles.createObstacle('recurrent curtain', ...
        [0;1;1.1;4;4.1;12], ...
        {wall(:,1);wall(:,1);moved(:,1);moved(:,1);wall(:,1);wall(:,1)}, ...
        {wall(:,2);wall(:,2);moved(:,2);moved(:,2);wall(:,2);wall(:,2)},0);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[4,4],'maxAcceleration_units_s2',[4,4], ...
        'maxJerk_units_s3',[8,8]);
    noSpatialSolve=planner(curtain,struct('time_s',0,'position_units',[-4,0]), ...
        struct('time_s',12,'position_units',[4,0]),limits, ...
        struct('GoalTimeMode','fixedArrival','TemporalResolution_s',0.25));
    assertTrue(testCase,noSpatialSolve.Success,noSpatialSolve.Message);

    scenario=createRandomAzimuthScenario(26,true);
    moving=scenario.Obstacles(1);
    returning=obstacleAvoidance.obstacles.createObstacle('returning rectangle', ...
        [0;90;180], ...
        {moving.originalX_units{1};moving.originalX_units{2};moving.originalX_units{1}}, ...
        {moving.originalY_units{1};moving.originalY_units{2};moving.originalY_units{1}}, ...
        moving.safetyMargin_units);
    scenario.Options.SpatialProbeIterationLimit=1;
    afterFailedSpatial=planner(obstacleAvoidance.obstacles.combineObstacles( ...
        {returning;scenario.Obstacles(2)}),scenario.InitialState, ...
        scenario.GoalState,scenario.Limits,scenario.Options);
    assertTrue(testCase,afterFailedSpatial.Success,afterFailedSpatial.Message);

    % The measures must precede the optimizer flags on both routes.
    verifyEqual(testCase,measureBeforeOptimizer(noSpatialSolve), ...
        measureBeforeOptimizer(afterFailedSpatial));
    verifyTrue(testCase,measureBeforeOptimizer(noSpatialSolve));

    exerciseFreshTimedOutcomeSchema(testCase);
end

function exerciseFreshTimedOutcomeSchema(testCase)
    % Exercise every timed construction path that is not already pinned by
    % the two production next method cases above.
    initial=struct('time_s',0,'position_units',[0,0]);
    goal=struct('time_s',6,'position_units',[4,0]);
    limits=struct('xInterval_units',[-2,6],'yInterval_units',[-3,3], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    base=planner([],initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    [request,requestContext,preparedObstacles]=explicitTimedInputs(base);
    attempts=struct('Marker',17);
    elapsedTime_s=1.25;

    unsupportedRequest=request;
    unsupportedRequest.options.GoalTimeMode="earliestArrival";
    unsupportedRequest.goalState.velocity_units_s=[0.1,0];
    [unsupported,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        unsupportedRequest,requestContext,preparedObstacles,attempts,elapsedTime_s,struct());
    verifyFalse(testCase,accepted);
    verifyEqual(testCase,unsupported.TerminationReason,"unsupportedTimedRequest");
    verifyEqual(testCase,unsupported.Message, ...
        "Free-arrival timed visibility requires a fixed-position goal " + ...
        "with zero endpoint velocity and acceleration.");
    verifyEqual(testCase,unsupported.Attempts,attempts);
    verifyGreaterThanOrEqual(testCase,unsupported.ElapsedTime_s,elapsedTime_s);
    verifyEqual(testCase,unsupported.Validation,struct( ...
        'Passed',false,'Message',"No timed motion is available."));
    verifyEqual(testCase,fieldnames(unsupported),timedFailureFieldNames());
    verifyEqual(testCase,fieldnames(unsupported.VisibilityGraph),timedGraphFieldNames());
    verifyFalse(testCase,isfield(unsupported,'EarliestArrival'));
    verifyFalse(testCase,isfield(unsupported,'WrappedGoalCopies'));

    freeWindowRequest=request;
    freeWindowRequest.options.GoalTimeMode="earliestArrival";
    [freeWindow,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        freeWindowRequest,requestContext,preparedObstacles,attempts,elapsedTime_s,struct());
    assertTrue(testCase,accepted,freeWindow.Message);
    verifyEqual(testCase,freeWindow.Message, ...
        "The first reachable goal window produced an independently validated free-clock BMTP motion.");
    verifyTrue(testCase,freeWindow.Validation.Passed);
    verifyEqual(testCase,freeWindow.Attempts,attempts);
    verifyEqual(testCase,freeWindow.GoalArrivalWindow_s,[3.5,6],'AbsTol',1e-8);
    verifyEqual(testCase,freeWindow.TrajectoryCoverageEndTime_s,6);
    verifyFalse(testCase,isfield(freeWindow,'FixedArrivalTrialTime_s'));
    verifyFalse(testCase,isfield(freeWindow,'EarliestArrival'));
    verifyFalse(testCase,isfield(freeWindow,'WrappedGoalCopies'));

    rawLimits=struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[20,20],'maxAcceleration_units_s2',[1,1], ...
        'maxJerk_units_s3',[1,1]);
    rawBase=planner([],struct('time_s',0,'position_units',[-1,0]), ...
        struct('time_s',30,'position_units',[1,0]),rawLimits, ...
        struct('GoalTimeMode','fixedArrival'));
    [rawRequest,rawContext,rawObstacles]=explicitTimedInputs(rawBase);
    rawRequest.goalState.time_s=obstacleAvoidance.input.minimumTravelTime( ...
        rawRequest.initialState,rawRequest.goalState,rawRequest.limits);
    rawContext.parentRequest=createTrialParentRequest(rawRequest,rawContext,30, ...
        rawRequest.goalState.time_s);
    [rawFailure,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        rawRequest,rawContext,rawObstacles,attempts,elapsedTime_s,struct());
    verifyFalse(testCase,accepted);
    verifyEqual(testCase,rawFailure.TerminationReason,"timedMotionInfeasible");
    verifyEqual(testCase,rawFailure.Message, ...
        "The timed route did not produce a feasible BMTP motion: " + ...
        "No optimized collision-free iterate was found. " + ...
        "Trajectory SOCP failed: Problem is infeasible.");
    verifyEqual(testCase,rawFailure.FailureStage,"proposal");
    verifyEqual(testCase,rawFailure.FailureKind,"trajectorySubproblemInfeasible");
    verifyFalse(testCase,rawFailure.SolverDiagnostics.Accepted);
    verifyFalse(testCase,rawFailure.Options.WrapX);
    verifyTrue(testCase,isfield(rawFailure,'ParentRequest'));

    rejectedContext=requestContext;
    rejectedContext.parentRequest=createTrialParentRequest(request,requestContext,6,5);
    [rejected,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        request,rejectedContext,preparedObstacles,attempts,elapsedTime_s,struct());
    verifyFalse(testCase,accepted);
    verifyTrue(testCase,rejected.SolverDiagnostics.Accepted);
    verifyEqual(testCase,rejected.TerminationReason,"invalidMotion");
    verifyEqual(testCase,rejected.Message, ...
        "BMTP returned motion that failed independent validation: " + ...
        "One or more independent core trajectory checks failed.");
    verifyEqual(testCase,rejected.Validation.Message, ...
        "One or more independent core trajectory checks failed.");
    verifyEqual(testCase,rejected.FixedArrivalTrialTime_s,5);
    verifyTrue(testCase,rejected.Options.WrapX);
    verifyFalse(testCase,isfield(rejected,'ParentRequest'));
end

function [request,requestContext,preparedObstacles]=explicitTimedInputs(result)
    % Recover normalized fixture inputs once; production callers pass these
    % values directly rather than reconstructing them from a result.
    request=struct( ...
        'initialState',result.Inputs.initialState, ...
        'goalState',result.Inputs.goalState, ...
        'limits',result.Limits, ...
        'options',result.Options);
    requestContext=struct( ...
        'obstacles',{result.Inputs.obstacles}, ...
        'suppliedLimits',result.SuppliedLimits, ...
        'requestedLimits',result.RequestedLimits, ...
        'suppliedGoalState',result.SuppliedGoalState, ...
        'requestedGoalState',result.RequestedGoalState, ...
        'parentRequest',{[]});
    preparedObstacles=result.PreparedObstacles;
end

function parentRequest=createTrialParentRequest(request,requestContext,goalTime_s,trialTime_s)
    % Declare a distinct parent request so acceptance ownership is observable.
    parentRequest=obstacleAvoidance.planning.createParentRequest(request,requestContext);
    parentRequest.WrapX=true;
    parentRequest.WrapY=false;
    parentRequest.GoalTime_s=goalTime_s;
    parentRequest.FixedArrivalTrialTime_s=trialTime_s;
end

function names=timedFailureFieldNames()
    names={ ...
        'Success';'Message';'TerminationReason';'Inputs';'PreparedObstacles'; ...
        'Limits';'Options';'VisibilityGraph';'Route_units';'time_s'; ...
        'position_units';'velocity_units_s';'acceleration_units_s2'; ...
        'jerk_units_s3';'Polynomial';'SeparationProof';'SolverDiagnostics'; ...
        'Attempts';'Validation';'ArrivalTime_s';'Intercept'; ...
        'TrajectoryDuration_s';'ElapsedTime_s';'SuppliedLimits'; ...
        'RequestedLimits';'RequestedGoalState';'SuppliedGoalState'; ...
        'MotionLength_units';'IntegratedSquaredJerk_units2_s5'; ...
        'MaximumConstraintViolation';'OptimizerFeasible'; ...
        'OptimizerIterateUnavailable';'AlternativeGuideEligible'; ...
        'FailureStage';'FailureKind'};
end

function names=timedGraphFieldNames()
    names={ ...
        'NodePosition_units';'AcceptedNodeIndex';'RejectedNodeIndex'; ...
        'Route_units';'RouteTime_s';'RouteLength_units';'IsConnected'; ...
        'ExpandedCount';'GraphIsFullyEnumerated';'SearchKind';'TimedSearch'};
end

function testTimedFailureFieldOrderMatchesTimedSuccess(testCase)
    % A timed early return that follows a failed spatial solve used to inherit
    % that candidate's field order, so a failed timed record was ordered
    % differently from a successful one. Fresh construction makes both orders
    % the same. Pin it on the failure path, which reaches noTimedRoute after a
    % spatial candidate has already run.
    rectangle=[-0.2,-5;0.2,-5;0.2,5;-0.2,5];
    shifted=rectangle+[0.1,0];
    obstacle=obstacleAvoidance.obstacles.createObstacle('narrow curtain', ...
        [0;5;10], ...
        {rectangle(:,1);shifted(:,1);rectangle(:,1)}, ...
        {rectangle(:,2);shifted(:,2);rectangle(:,2)},0);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-6,6], ...
        'maxVelocity_units_s',[1,1],'maxAcceleration_units_s2',[4,4], ...
        'maxJerk_units_s3',[8,8]);
    result=planner(obstacle,struct('time_s',0,'position_units',[-4,0]), ...
        struct('time_s',10,'position_units',[4,0]),limits, ...
        struct('GoalTimeMode','fixedArrival','TemporalResolution_s',0.25, ...
        'SpatialProbeIterationLimit',1));

    % A spatial candidate must actually have run, or this fixture no longer
    % exercises the history the test exists for.
    assertFalse(testCase,result.Success);
    verifyTrue(testCase,any([result.Attempts.SolverAttempted]));
    verifyTrue(testCase,measureBeforeOptimizer(result));
end

function isBefore=measureBeforeOptimizer(result)
    names=string(fieldnames(result));
    isBefore=find(names=="MotionLength_units",1)<find(names=="OptimizerFeasible",1);
end

function testDuplicateArrivalGuideIsNotSolvedTwice(testCase)
    scenario=createRandomAzimuthScenario(26,true);
    moving=scenario.Obstacles(1);
    returning=obstacleAvoidance.obstacles.createObstacle('returning rectangle', ...
        [0;90;180], ...
        {moving.originalX_units{1};moving.originalX_units{2};moving.originalX_units{1}}, ...
        {moving.originalY_units{1};moving.originalY_units{2};moving.originalY_units{1}}, ...
        moving.safetyMargin_units);
    obstacles=obstacleAvoidance.obstacles.combineObstacles( ...
        {returning;scenario.Obstacles(2)});
    scenario.Options.SpatialProbeIterationLimit=1;

    result=planner(obstacles,scenario.InitialState,scenario.GoalState, ...
        scenario.Limits,scenario.Options);

    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Validation.Passed);
    verifyEqual(testCase,result.Options.SpatialProbeIterationLimit,1);
    verifyEqual(testCase,result.Attempts(1).IterationLimit,1);
    verifyEqual(testCase,numel(result.Attempts),3);
    verifyFalse(testCase,result.Attempts(2).SolverAttempted);
    verifyEqual(testCase,result.Attempts(3).Kind,"timedVisibility");
end
