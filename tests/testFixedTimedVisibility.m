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

function testGivenArrivalAndContinuousMotion(testCase)
    data=testCase.TestData;
    result=planner([],data.Initial,data.Goal,data.Limits,data.Options);
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,10,'AbsTol',1e-8);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,result.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    verifyLessThan(testCase,size(result.Diagnostics.Route_units,1),9);
end

function testMovingTargetEndsAtExactGivenClock(testCase)
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
    verifyEqual(testCase,result.Diagnostics.Polynomial.FinalTime_s,goal.time_s);
    verifyEqual(testCase,result.time_s(end),goal.time_s);
end

function testStaticGoalUsesExactGivenClock(testCase)
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
    verifyEqual(testCase,result.Diagnostics.Polynomial.FinalTime_s,goal.time_s);
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
    verifyTrue(testCase,result.Diagnostics.Validation.Passed);
    verifyEqual(testCase,result.velocity_units_s([1,end],:),[0.1,0;0,0], ...
        'AbsTol',1e-8);
end

function testSavedDetourUsesGivenDeadline(testCase)
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
    verifyGreaterThan(testCase,result.Diagnostics.SolverDiagnostics.OptimizerSpanCount,16);
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
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "arrivalSpatialSnapshot");
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.IsShortcut],[true,true]);
    verifyEqual(testCase,[result.Diagnostics.Attempts.IterationLimit],[2,2]);
    verifyEqual(testCase,result.Options.SpatialProbeIterationLimit,2);
    verifyEqual(testCase,result.Diagnostics.Attempts(1).FailureKind,"iterationLimit");
    verifyTrue(testCase,result.Diagnostics.Attempts(1).NextAttemptAllowed);
    verifyGreaterThan(testCase,size(result.Diagnostics.Route_units,1),2);
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
    verifyTrue(testCase,result.Diagnostics.Validation.Passed);
    verifyEqual(testCase,result.Message, ...
        "The given goal layer produced an independently validated timed BMTP motion.");
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyEqual(testCase,result.Diagnostics.FixedArrivalTrialTime_s,goal.time_s);
    verifyFalse(testCase,isfield(result.Diagnostics, 'GoalArrivalWindow_s'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'TrajectoryCoverageEndTime_s'));
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),3);
    verifyFalse(testCase,any([result.Diagnostics.Attempts(1:2).SolverAttempted]));
    % Fixed-arrival attempts keep the requested time in FixedArrivalTrialTime_s.
    verifyTrue(testCase,isnan(result.Diagnostics.Attempts(end).CandidateArrival_s));
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
    verifyEqual(testCase,result.Diagnostics.Validation,struct( ...
        'Passed',false,'Message',"No timed motion is available."));
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyFalse(testCase,result.Diagnostics.VisibilityGraph.IsConnected);
    verifyEmpty(testCase,result.Diagnostics.Route_units);
    verifyEmpty(testCase,result.time_s);
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),3);
    verifyEqual(testCase,result.Diagnostics.Attempts(end).FailureStage,"search");
    verifyEqual(testCase,result.Diagnostics.Attempts(end).FailureKind,"noTimedRoute");
    % The public planner appends its plot shortcut after the core fields.
    verifyEqual(testCase,fieldnames(result),[timedFailureFieldNames();'plotTrajectory']);
    verifyEqual(testCase,fieldnames(result.Diagnostics),timedFailureDiagnosticFieldNames());
    verifyEqual(testCase,fieldnames(result.Diagnostics.VisibilityGraph),timedGraphFieldNames());
    verifyEqual(testCase,result.MotionLength_units,Inf);
    verifyTrue(testCase,isnan(result.ArrivalTime_s));
    verifyEqual(testCase,result.Diagnostics.IntegratedSquaredJerk_units2_s5,Inf);
    verifyEqual(testCase,result.Diagnostics.MaximumConstraintViolation,Inf);
    verifyFalse(testCase,result.Diagnostics.OptimizerFeasible);
    verifyFalse(testCase,result.Diagnostics.OptimizerIterateUnavailable);
    verifyFalse(testCase,result.Diagnostics.AlternativeGuideEligible);
    verifyFalse(testCase,isfield(result.Diagnostics, 'EarliestArrival'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'WrappedGoalCopies'));
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
    [request,preparedObstacles]=explicitTimedInputs(base);
    attempts=struct('Marker',17);
    elapsedTime_s=1.25;

    unsupportedRequest=request;
    unsupportedRequest.options.GoalTimeMode="earliestArrival";
    unsupportedRequest.goalState.velocity_units_s=[0.1,0];
    [unsupported,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        unsupportedRequest,preparedObstacles,attempts,elapsedTime_s,struct(),unsupportedRequest.goalState.time_s);
    verifyFalse(testCase,accepted);
    verifyEqual(testCase,unsupported.TerminationReason,"unsupportedTimedRequest");
    verifyEqual(testCase,unsupported.Message, ...
        "Free-arrival timed visibility requires a fixed-position goal " + ...
        "with zero endpoint velocity and acceleration.");
    verifyEqual(testCase,unsupported.Diagnostics.Attempts,attempts);
    verifyGreaterThanOrEqual(testCase,unsupported.Diagnostics.ElapsedTime_s,elapsedTime_s);
    verifyEqual(testCase,unsupported.Diagnostics.Validation,struct( ...
        'Passed',false,'Message',"No timed motion is available."));
    verifyEqual(testCase,fieldnames(unsupported),timedFailureFieldNames());
    verifyEqual(testCase,fieldnames(unsupported.Diagnostics),timedFailureDiagnosticFieldNames());
    verifyEqual(testCase,fieldnames(unsupported.Diagnostics.VisibilityGraph),timedGraphFieldNames());
    verifyFalse(testCase,isfield(unsupported.Diagnostics, 'EarliestArrival'));
    verifyFalse(testCase,isfield(unsupported.Diagnostics, 'WrappedGoalCopies'));

    freeWindowRequest=request;
    freeWindowRequest.options.GoalTimeMode="earliestArrival";
    [freeWindow,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        freeWindowRequest,preparedObstacles,attempts,elapsedTime_s,struct(),freeWindowRequest.goalState.time_s);
    assertTrue(testCase,accepted,freeWindow.Message);
    verifyEqual(testCase,freeWindow.Message, ...
        "The first reachable goal window produced an independently validated free-clock BMTP motion.");
    verifyTrue(testCase,freeWindow.Diagnostics.Validation.Passed);
    verifyEqual(testCase,freeWindow.Diagnostics.Attempts,attempts);
    verifyEqual(testCase,freeWindow.Diagnostics.GoalArrivalWindow_s,[3.5,6],'AbsTol',1e-8);
    verifyEqual(testCase,freeWindow.Diagnostics.TrajectoryCoverageEndTime_s,6);
    verifyFalse(testCase,isfield(freeWindow.Diagnostics, 'FixedArrivalTrialTime_s'));
    verifyFalse(testCase,isfield(freeWindow.Diagnostics, 'EarliestArrival'));
    verifyFalse(testCase,isfield(freeWindow.Diagnostics, 'WrappedGoalCopies'));

    rawLimits=struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[20,20],'maxAcceleration_units_s2',[1,1], ...
        'maxJerk_units_s3',[1,1]);
    rawBase=planner([],struct('time_s',0,'position_units',[-1,0]), ...
        struct('time_s',30,'position_units',[1,0]),rawLimits, ...
        struct('GoalTimeMode','fixedArrival'));
    [rawRequest,rawObstacles]=explicitTimedInputs(rawBase);
    rawRequest.goalState.time_s=obstacleAvoidance.input.minimumTravelTime( ...
        rawRequest.initialState,rawRequest.goalState,rawRequest.limits);
    rawRequest.parentRequest=createTrialParentRequest(rawRequest,30, ...
        rawRequest.goalState.time_s);
    [rawFailure,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        rawRequest,rawObstacles,attempts,elapsedTime_s,struct(),rawRequest.goalState.time_s);
    verifyFalse(testCase,accepted);
    verifyEqual(testCase,rawFailure.TerminationReason,"timedMotionInfeasible");
    verifyEqual(testCase,rawFailure.Message, ...
        "The timed route did not produce a feasible BMTP motion: " + ...
        "No optimized collision-free iterate was found. " + ...
        "Trajectory SOCP failed: Problem is infeasible.");
    verifyEqual(testCase,rawFailure.Diagnostics.FailureStage,"proposal");
    verifyEqual(testCase,rawFailure.Diagnostics.FailureKind,"trajectorySubproblemInfeasible");
    verifyFalse(testCase,rawFailure.Diagnostics.SolverDiagnostics.Accepted);
    verifyEqual(testCase,rawFailure.Options.WrapX,"false");
    verifyTrue(testCase,isfield(rawFailure.Diagnostics, 'ParentRequest'));

    rejectedRequest=request;
    rejectedRequest.parentRequest=createTrialParentRequest(request,6,5);
    [rejected,accepted]=obstacleAvoidance.planning.tryTimedArrival( ...
        rejectedRequest,preparedObstacles,attempts,elapsedTime_s,struct(),rejectedRequest.goalState.time_s);
    verifyFalse(testCase,accepted);
    verifyTrue(testCase,rejected.Diagnostics.SolverDiagnostics.Accepted);
    verifyEqual(testCase,rejected.TerminationReason,"invalidMotion");
    % The parent claims x wrapping, but the trial's limits were built without
    % it, so the validator rejects the record before the motion checks.
    verifyEqual(testCase,rejected.Message, ...
        "BMTP returned motion that failed independent validation: " + ...
        "The requested wrapped limits do not match the supplied limits.");
    verifyEqual(testCase,rejected.Diagnostics.Validation.Message, ...
        "The requested wrapped limits do not match the supplied limits.");
    verifyEqual(testCase,rejected.Diagnostics.FixedArrivalTrialTime_s,5);
    verifyTrue(testCase,rejected.Options.WrapX);
    verifyFalse(testCase,isfield(rejected.Diagnostics, 'ParentRequest'));
end

function [request,preparedObstacles]=explicitTimedInputs(result)
    % Recover normalized fixture inputs once; production callers pass these
    % values directly rather than reconstructing them from a result.
    request=struct( ...
        'initialState',result.Inputs.initialState, ...
        'goalState',result.Inputs.goalState, ...
        'limits',result.Diagnostics.Limits, ...
        'options',result.Options);
    request.obstacles=result.Inputs.obstacles;
    request.parentRequest=[];
    request.originalInputs=struct( ...
        'suppliedLimits',result.Diagnostics.SuppliedLimits, ...
        'requestedLimits',result.Diagnostics.RequestedLimits, ...
        'suppliedGoalState',result.Diagnostics.SuppliedGoalState, ...
        'requestedGoalState',result.Diagnostics.RequestedGoalState);
    preparedObstacles=result.Diagnostics.PreparedObstacles;
end

function parentRequest=createTrialParentRequest(request,goalTime_s,trialTime_s)
    % Declare a distinct parent request so acceptance ownership is observable.
    parentRequest=obstacleAvoidance.planning.createParentRequest(request);
    parentRequest.WrapX=true;
    parentRequest.WrapY=false;
    parentRequest.GoalTime_s=goalTime_s;
    parentRequest.FixedArrivalTrialTime_s=trialTime_s;
end

function names=timedFailureFieldNames()
    names={ ...
        'Success';'Message';'TerminationReason';'Inputs';'Options'; ...
        'time_s';'position_units';'velocity_units_s'; ...
        'acceleration_units_s2';'jerk_units_s3';'ArrivalTime_s'; ...
        'MotionLength_units';'Diagnostics'};
end

function names=timedFailureDiagnosticFieldNames()
    names={ ...
        'PreparedObstacles';'Limits';'VisibilityGraph';'Route_units'; ...
        'Polynomial';'SeparationProof';'SolverDiagnostics';'Attempts'; ...
        'Validation';'Intercept';'TrajectoryDuration_s';'ElapsedTime_s'; ...
        'SuppliedLimits';'RequestedLimits';'RequestedGoalState'; ...
        'SuppliedGoalState';'IntegratedSquaredJerk_units2_s5'; ...
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
    verifyTrue(testCase,any([result.Diagnostics.Attempts.SolverAttempted]));
    verifyTrue(testCase,measureBeforeOptimizer(result));
end

function isBefore=measureBeforeOptimizer(result)
    topLevelNames=string(fieldnames(result));
    diagnosticNames=string(fieldnames(result.Diagnostics));
    isBefore=find(topLevelNames=="MotionLength_units",1)<find(topLevelNames=="Diagnostics",1) && ...
        find(diagnosticNames=="IntegratedSquaredJerk_units2_s5",1)< ...
        find(diagnosticNames=="OptimizerFeasible",1);
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
    verifyTrue(testCase,result.Diagnostics.Validation.Passed);
    verifyEqual(testCase,result.Options.SpatialProbeIterationLimit,1);
    verifyEqual(testCase,result.Diagnostics.Attempts(1).IterationLimit,1);
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),3);
    verifyFalse(testCase,result.Diagnostics.Attempts(2).SolverAttempted);
    verifyEqual(testCase,result.Diagnostics.Attempts(3).Kind,"timedVisibility");
end
