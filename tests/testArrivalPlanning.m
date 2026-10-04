function tests = testArrivalPlanning
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests('tests/testArrivalPlanning.m')
%**************************************************************************
% PURPOSE
%   - Preserve arrival clocks, budgets, pipeline schemas, and validated selection.
%**************************************************************************
% INPUTS
%   - MATLAB function-based unit test framework.
%**************************************************************************
% OUTPUTS
%   - Function test array with independently named regression cases.
%**************************************************************************
% UNITS
%   - Position uses coordinate units and time uses seconds.
%**************************************************************************

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testArrivalGridContracts(testCase)
    runCases(testCase, { ...
        @offGridTargetBoundaryRetainsArrivalTimePriority; ...
        @numericallyDistinctGridAndBoundaryAreBothRetained; ...
        @largeAbsoluteTimeRejectsUnrepresentableResolution; ...
        @negativeAbsoluteTimeKeepsFirstRepresentableGridClock; @gridSearchChecksEvaluatedMidpointRounding; ...
        @roundedGridNeverQueriesPastTargetHistory; @tinyResolutionRejectsUnrepresentableGrid; ...
        @subnormalResolutionRejectsInfiniteStepIndex});
end

function testArrivalBudgetContracts(testCase)
    runCases(testCase, { ...
        @longRequestUsesBudgetAfterPhysicalBound; @movingTargetPrescreenPreservesSolverBudget; ...
        @prescreenCandidateWorkAndStorageAreBounded; @coincidentMovingTargetDoesNotSpendSolverBudget; ...
        @bestSoFarRefinementBudgetIsExplicit; @arrivalSearchExhausted});
end

function testFixedArrivalPipelineAndSchemaContracts(testCase)
    [noSpatialSolve, afterFailedSpatial] = createFixedArrivalPipelineFixture();
    runCases(testCase, { ...
        @movingCrossingRetainsProvenEndpointJerk; @arrivalSnapshotAvoidsTimedSearch; ...
        @(testCase) timedSearchCrossesARecurrentCurtain(testCase, noSpatialSolve); ...
        @failedTimedSearchDoesNotLeakSpatialState; ...
        @(testCase) timedResultFieldOrderDoesNotDependOnTheRouteTaken(testCase, noSpatialSolve, afterFailedSpatial); ...
        @exerciseFreshTimedOutcomeSchema; @timedFailureFieldOrderMatchesTimedSuccess; ...
        @(testCase) duplicateArrivalGuideIsNotSolvedTwice(testCase, afterFailedSpatial); ...
        @initiallyOccupiedFutureGoalUsesArrivalDetour; @disconnectedInitialSnapshotUsesArrivalSnapshot; ...
        @coverageFieldsMatchTheScenePath; @movingDetourWithNonzeroEndpointVelocity});
end

function testBestSoFarSelectionContracts(testCase)
    runCases(testCase, { ...
        @disconnectedSnapshotRetainsValidatedWaitHonestly; ...
        @movingGeometryDoesNotUseInitialRouteAsGlobalBound; ...
        @failedWrappedTimedTimedSearchRetainsValidatedBestSoFar; ...
        @failedScaledBarrierTimedSearchRetainsValidatedBestSoFar; ...
        @wrappedBestSoFarRefinementRetainsValidatedDeparture; ...
        @planarRequestSelectsRefinementOverWrappedBestSoFar; @challengedDelayedChordBestSoFarIsRetained; ...
        @sparseDynamicZeroWaitDeparture; @equivalentSparseAndDenseHistoriesUseProvenDeparture});
end

function testFreeArrivalQualityAndHorizonContracts(testCase)
    runCases(testCase, { ...
        @circleDetourBeatsWaiting; @arrivalSnapshotFindsAnOpeningMissingAtInitialTime; ...
        @timedSearchFindsRouteAfterWholeCurtainDeparts; @timedHomotopyPrecedesDelayedDeparture; ...
        @horizonDoesNotMoveSquareGoalClearing; @unsampledIntervalWithoutClocksKeepsTimedMotion; ...
        @horizonDoesNotMoveDriftingTriangleClearing});
end

function offGridTargetBoundaryRetainsArrivalTimePriority(testCase)
    % Exact target sample times remain candidates even when they are not on
    % the regular arrival grid or at the horizon.
    targetMotion = struct( ...
        'time_s',[0;3.7;10], ...
        'position_units',[1,0;1,0;1,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',10,'targetMotion',targetMotion);
    limits = struct( ...
        'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',10, ...
        'MaxArrivalTrials',1);

    result = planner([],restState(0,[0,0]),goal,limits,options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s,3.7,'AbsTol',1e-12);
    verifyEqual(testCase,result.ArrivalTime_s,3.7,'AbsTol',1e-12);
end

function numericallyDistinctGridAndBoundaryAreBothRetained(testCase)
    % An exact source boundary and its rounded grid neighbor are distinct
    % declared clocks. Both remain searchable inside the bounded grid window.
    targetMotion = struct( ...
        'time_s',[0;0.3;1], ...
        'position_units',[0.005,0;0.005,0;0.005,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',1,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.1, ...
        'MaxArrivalCandidates',4);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,0.4,'AbsTol',1e-12);
end

function largeAbsoluteTimeRejectsUnrepresentableResolution(testCase)
    % Half-second increments are below the ulp at this absolute time. Reject
    % the ill-defined grid instead of silently changing its resolution.
    initialTime_s = 1e16;
    targetMotion = struct( ...
        'time_s',[initialTime_s;initialTime_s + 10], ...
        'position_units',[0.1,0;0.1,0], ...
        'InterpolationMethod','linear');
    goal = struct( ...
        'time_s',initialTime_s + 10, ...
        'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalCandidates',2);

    verifyError(testCase,@() planner( ...
        [],restState(initialTime_s,[0,0]),goal,struct(),options), ...
        'planner:UnrepresentableTemporalResolution');
end

function negativeAbsoluteTimeKeepsFirstRepresentableGridClock(testCase)
    % Toward +Inf, a negative power of two has half the spacing reported by
    % eps(abs(t)). The evaluated-grid search must not skip that first clock.
    initialTime_s = -1;
    firstLaterTime_s = typecast(typecast(initialTime_s,'uint64') - uint64(1),'double');
    resolution_s = firstLaterTime_s - initialTime_s;
    targetMotion = struct( ...
        'time_s',[initialTime_s;0], ...
        'position_units',[1e-18,0;1e-18,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',0,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',resolution_s, ...
        'MaxArrivalCandidates',1);

    result = planner([],restState(initialTime_s,[0,0]),goal,struct(),options);

    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,1);
end

function gridSearchChecksEvaluatedMidpointRounding(testCase)
    % This resolution is three quarters of one ulp at t = 1. Index one
    % already rounds upward and must not be skipped by inverse arithmetic.
    initialTime_s = 1;
    resolution_s  = 3 * 2 ^ -54;
    firstGridTime_s = initialTime_s + resolution_s;
    targetMotion = struct( ...
        'time_s',[initialTime_s;2], ...
        'position_units',[100,0;100,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',2,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',resolution_s, ...
        'MaxArrivalCandidates',1);

    result = planner([],restState(initialTime_s,[0,0]),goal,struct(),options);

    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,1);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedTime_s, ...
        firstGridTime_s,'AbsTol',0);
end

function roundedGridNeverQueriesPastTargetHistory(testCase)
    % Binary rounding can place the final nominal grid time just beyond the
    % declared target history. That point is outside the planning request.
    targetMotion = struct( ...
        'time_s',[0.001;0.009], ...
        'position_units',[100,0;100,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',0.009,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.001);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"arrivalSearchExhausted");
    verifyFalse(testCase,result.Diagnostics.TemporalSearch.CandidateLimitReached);
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.SearchWindowExhausted);
end

function tinyResolutionRejectsUnrepresentableGrid(testCase)
    % The requested grid cannot advance at this absolute time scale.
    targetMotion = struct( ...
        'time_s',[0.001;0.009], ...
        'position_units',[100,0;100,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',0.009,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',1e-20, ...
        'MaxArrivalCandidates',2);

    verifyError(testCase,@() planner( ...
        [],restState(0,[0,0]),goal,struct(),options), ...
        'planner:UnrepresentableTemporalResolution');
end

function subnormalResolutionRejectsInfiniteStepIndex(testCase)
    % A subnormal resolution overflows the finite horizon's step index.
    blocker_units = [3.5,-0.5;4.5,-0.5;4.5,0.5;3.5,0.5];
    blocker = obstacleAvoidance.obstacles.createObstacle( ...
        'terminal blocker',0,{blocker_units(:,1)},{blocker_units(:,2)},0);
    initial = restState(0,[0,0]);
    initial.velocity_units_s = [0.1,0];
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',1e-320, ...
        'MaxArrivalCandidates',2);

    verifyError(testCase,@() planner( ...
        blocker,initial,restState(10,[4,0]),struct(),options), ...
        'planner:UnrepresentableTemporalResolution');
end

function longRequestUsesBudgetAfterPhysicalBound(testCase)
    box = [-1,-0.5;1,-0.5;1,0.5;-1,0.5];
    wall = obstacleAvoidance.obstacles.createObstacle('static crossing',[0;180],box(:,1),box(:,2));
    remote = obstacleAvoidance.obstacles.createObstacle('remote moving box',[0;180], ...
        {box(:,1)+20;box(:,1)+30},{box(:,2)+60;box(:,2)+60});
    initial = struct('time_s',0,'position_units',[-60,0]);
    goal = struct('time_s',180,'position_units',[60,0]);
    limits = struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
        'maxJerk_units_s3',[2.5,2.5]);
    result = planner([wall;remote],initial,goal,limits,struct('GoalTimeMode','earliestArrival','MaxArrivalTrials',40));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase,result.Diagnostics.TemporalSearch.EarliestPossibleArrival_s,60);
    verifyGreaterThanOrEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s, ...
        result.Diagnostics.TemporalSearch.EarliestPossibleArrival_s-result.Options.ArrivalTimeTolerance_s);
    verifyLessThanOrEqual(testCase,numel(result.Diagnostics.TemporalSearch.TrialTime_s),40);
    % The exact static-only reference arrives at 64 s. The moving obstacle is
    % remote, so the unified timed profile must stay within the 1% gate.
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,64.64);
end

function movingTargetPrescreenPreservesSolverBudget(testCase)
    % Necessary endpoint failures must not consume the bounded solver trials.
    % This approaching target is unreachable throughout the original eight
    % half-second slots but becomes reachable well inside its declared history.
    targetMotion = struct( ...
        'time_s',[0;60], ...
        'position_units',[40,0;10,0], ...
        'InterpolationMethod','linear');
    initial = restState(0,[0,0]);
    goal = struct('time_s',60,'targetMotion',targetMotion);
    limits = struct( ...
        'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[2,2], ...
        'maxJerk_units_s3',[4,4]);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalTrials',8);

    result = planner([],initial,goal,limits,options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase,result.ArrivalTime_s, ...
        options.TemporalResolution_s * options.MaxArrivalTrials);
    verifyGreaterThan(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,0);
    verifyLessThanOrEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount, ...
        options.MaxArrivalTrials);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount, ...
        numel(result.Diagnostics.TemporalSearch.TrialTime_s));
end

function prescreenCandidateWorkAndStorageAreBounded(testCase)
    % A huge target history must not turn one permitted solver trial into an
    % unbounded endpoint scan or allocate by the much larger solver cap.
    targetMotion = struct( ...
        'time_s',[0;1e5], ...
        'position_units',[1000,0;1000,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',1e5,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalTrials',1e9, ...
        'MaxArrivalCandidates',8);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyFalse(testCase,result.Success);
    verifyEqual(testCase,result.TerminationReason,"arrivalSearchExhausted");
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,8);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount,0);
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.CandidateLimitReached);
end

function coincidentMovingTargetDoesNotSpendSolverBudget(testCase)
    % A target parked at the initial point creates invalid fixed-arrival
    % endpoints, not BMTP trials. Its later departure must remain searchable.
    targetMotion = struct( ...
        'time_s',[0;50;60], ...
        'position_units',[0,0;0,0;1,0], ...
        'InterpolationMethod','linear');
    goal = struct('time_s',60,'targetMotion',targetMotion);
    options = struct( ...
        'GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5, ...
        'MaxArrivalTrials',1);

    result = planner([],restState(0,[0,0]),goal,struct(),options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.ArrivalTime_s,50.5,'AbsTol',1e-12);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount,1);
    verifyGreaterThanOrEqual(testCase, ...
        result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,100);
end

function bestSoFarRefinementBudgetIsExplicit(testCase)
    options = struct('PlotOutputs',false,'Verbose',false, ...
        'MaxArrivalTrials',40,'BestSoFarRefinementTrialLimit',1);
    result = exampleMovingBarrierWait(options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyTrue(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.MaximumTrialCount,1);
    verifyLessThanOrEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount,1);
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.TrialLimitReached);
    verifyTrue(testCase,result.Diagnostics.EarliestArrival.ArrivalTimeSearchUsed);
    arrivalTimeTrialIndices=find([result.Diagnostics.Attempts.Kind] == ...
        "arrivalTimeTrial");
    verifyNotEmpty(testCase,arrivalTimeTrialIndices);
end

function arrivalSearchExhausted(testCase)
    targetMotion=struct('time_s',[0;1], ...
        'position_units',[100,0;101,0],'InterpolationMethod','linear');
    goal=struct('time_s',1,'targetMotion',targetMotion);
    result=planner([],state(0,[0,0]),goal,standardLimits(), ...
        struct('GoalTimeMode','earliestArrival','TemporalResolution_s',0.5));
    verifyFailure(testCase,result,"arrivalSearchExhausted");
    verifyTrue(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyEmpty(testCase,result.Diagnostics.TemporalSearch.TrialTime_s);
    verifyGreaterThan(testCase, ...
        result.Diagnostics.TemporalSearch.PrescreenedCandidateCount,0);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.SolverTrialCount,0);
end

function movingCrossingRetainsProvenEndpointJerk(testCase)
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

function arrivalSnapshotAvoidsTimedSearch(testCase)
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

function timedSearchCrossesARecurrentCurtain(testCase, result)
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Diagnostics.Validation.Passed);
    verifyEqual(testCase,result.Message, ...
        "The given goal layer produced an independently validated timed BMTP motion.");
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyEqual(testCase,result.Diagnostics.FixedArrivalTrialTime_s,12);
    verifyFalse(testCase,isfield(result.Diagnostics, 'GoalArrivalWindow_s'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'TrajectoryCoverageEndTime_s'));
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),3);
    verifyFalse(testCase,any([result.Diagnostics.Attempts(1:2).SolverAttempted]));
    % Fixed-arrival attempts keep the requested time in FixedArrivalTrialTime_s.
    verifyTrue(testCase,isnan(result.Diagnostics.Attempts(end).CandidateArrival_s));
end

function failedTimedSearchDoesNotLeakSpatialState(testCase)
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

function timedResultFieldOrderDoesNotDependOnTheRouteTaken(testCase, noSpatialSolve, afterFailedSpatial)
    % A timed result reaches the finalizer by two histories: with no preceding
    % spatial BMTP candidate, and after a failed one. The record is assembled
    % from one constructor, so the field order must not depend on which history
    % produced it. Nothing reads field order today, but a record whose shape
    % varies by route is a schema that cannot be relied on.
    assertTrue(testCase, noSpatialSolve.Success, noSpatialSolve.Message);
    assertTrue(testCase, afterFailedSpatial.Success, afterFailedSpatial.Message);
    % The measures must precede the optimizer flags on both routes.
    verifyEqual(testCase,measureBeforeOptimizer(noSpatialSolve), ...
        measureBeforeOptimizer(afterFailedSpatial));
    verifyTrue(testCase,measureBeforeOptimizer(noSpatialSolve));
end

function timedFailureFieldOrderMatchesTimedSuccess(testCase)
    % Even after a failed spatial solve, the fresh timed failure must keep
    % the same measure-before-optimizer field order as a successful result.
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

function duplicateArrivalGuideIsNotSolvedTwice(testCase, result)
    assertTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,result.Diagnostics.Validation.Passed);
    verifyEqual(testCase,result.Options.SpatialProbeIterationLimit,1);
    verifyEqual(testCase,result.Diagnostics.Attempts(1).IterationLimit,1);
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),3);
    verifyFalse(testCase,result.Diagnostics.Attempts(2).SolverAttempted);
    verifyEqual(testCase,result.Diagnostics.Attempts(3).Kind,"timedVisibility");
end

function initiallyOccupiedFutureGoalUsesArrivalDetour(testCase)
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
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"arrivalSpatialSnapshot");
    verifyGreaterThan(testCase,size(result.Diagnostics.Route_units,1),2);
end

function disconnectedInitialSnapshotUsesArrivalSnapshot(testCase)
    wall=[-0.2,-7;0.2,-7;0.2,7;-0.2,7];
    moved=wall+[0,14];
    obstacle=obstacleAvoidance.obstacles.createObstacle('departing wall',[0;5], ...
        {wall(:,1);moved(:,1)},{wall(:,2);moved(:,2)},0);
    result=planner(obstacle,state(0,[-4,0]),state(12,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"arrivalSpatialSnapshot");
end

function coverageFieldsMatchTheScenePath(testCase)
    % The planner builds coverage in three shapes: static, dynamic
    % earliest-arrival, and dynamic fixed-arrival. Downstream code branches on
    % field presence, so a placeholder field would change behaviour without
    % changing any motion a test already checks. Pin the exact field list.
    staticResult=planner([],state(0,[-4,0]),state(12,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,staticResult.Success,staticResult.Message);
    verifyEqual(testCase,fieldnames(staticResult.Diagnostics.SeparationProof.Coverage), ...
        {'ExactRegionCount'});

    box=[-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    first=box+[0,8];
    last=box+[2,8];
    mover=obstacleAvoidance.obstacles.createObstacle('remote mover',[0;20], ...
        {first(:,1);last(:,1)},{first(:,2);last(:,2)},0);

    fixedResult=planner(mover,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','fixedArrival'));
    verifyTrue(testCase,fixedResult.Success,fixedResult.Message);
    verifyEqual(testCase,fieldnames(fixedResult.Diagnostics.SeparationProof.Coverage), ...
        {'ExactRegionCount';'ActiveTimeInterval_s';'EndRegions_units';'BreakTime_s'});

    % BreakTime_s is the one that separates two dynamic runs from each other.
    earliestResult=planner(mover,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,earliestResult.Success,earliestResult.Message);
    verifyEqual(testCase,fieldnames(earliestResult.Diagnostics.SeparationProof.Coverage), ...
        {'ExactRegionCount';'ActiveTimeInterval_s';'EndRegions_units'});
end

function movingDetourWithNonzeroEndpointVelocity(testCase)
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

function disconnectedSnapshotRetainsValidatedWaitHonestly(testCase)
    result = exampleMovingBarrierWait(struct('PlotOutputs',false,'Verbose',false,'MaxArrivalTrials',1));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyGreaterThan(testCase,result.Diagnostics.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics.DepartureSchedule, ...
        'InitialRouteTimeBound_s'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'FixedArrivalTrialTime_s'));
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Selected);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).NextMethodAllowed);
    verifyFalse(testCase,result.Diagnostics.EarliestArrival.ArrivalTimeSearchUsed);
end

function movingGeometryDoesNotUseInitialRouteAsGlobalBound(testCase)
    result = exampleOpeningUShapedObstacle(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyFalse(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics.DepartureSchedule, ...
        'InitialRouteTimeBound_s'));
    verifyGreaterThan(testCase,result.Diagnostics.SolverDiagnostics.DepartureSchedule.DepartureDelay_s,0);
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Selected);
end

function failedWrappedTimedTimedSearchRetainsValidatedBestSoFar(testCase)
    result = exampleOpeningUShapedObstacle(struct( ...
        'PlotOutputs', false, 'Verbose', false, 'WrapX', true));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    % Miter-joined margins (e803888) extend the protected corners slightly,
    % so the departure arrives at 11.6134668007 s (square joins gave
    % 11.6133886958 s). The failed timed search must retain this validated motion.
    verifyEqual(testCase, result.ArrivalTime_s, 11.6134668006798, 'AbsTol', 1e-10);
    verifyEqual(testCase, [result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture", "timedVisibility"]);
    verifyTrue(testCase, result.Diagnostics.Attempts(1).Selected);
    verifyEqual(testCase, result.Diagnostics.Attempts(1).Message, "");
    verifyTrue(testCase, isnan(result.Diagnostics.Attempts(1).SolverExitFlag));
    verifyFalse(testCase, result.Diagnostics.Attempts(2).Success);
    verifyFalse(testCase, result.Diagnostics.Attempts(2).Selected);
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureStage, "proposal");
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureKind, ...
        "trajectorySubproblemInfeasible");
    verifyTrue(testCase, result.Diagnostics.Attempts(2).OptimizerIterateUnavailable);
    verifyTrue(testCase, result.Diagnostics.Attempts(2).NextMethodAllowed);

    timedSearch = result.Diagnostics.SolverDiagnostics.TimedSearchAttempt;
    verifyFalse(testCase, timedSearch.Success);
    verifyEqual(testCase, timedSearch.TerminationReason, "timedMotionInfeasible");
    verifyEqual(testCase, timedSearch.FailureStage, "proposal");
    verifyEqual(testCase, timedSearch.FailureKind, "trajectorySubproblemInfeasible");
    verifyTrue(testCase, timedSearch.OptimizerIterateUnavailable);
    verifyEqual(testCase, timedSearch.SolverDiagnostics.LastTrajectoryExitFlag, -2);
    verifySubstring(testCase, timedSearch.Message, "Problem is infeasible");
    verifyEqual(testCase, timedSearch.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyTrue(testCase, isfield(timedSearch.VisibilityGraph, 'TimedSearch'));
    verifyNotEmpty(testCase, timedSearch.VisibilityGraph.RouteTime_s);
    verifyEqual(testCase, timedSearch.Route_units, ...
        timedSearch.VisibilityGraph.Route_units);
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, ...
        "c3DepartureSchedule");
    verifyEqual(testCase, result.Diagnostics.Route_units, [0, 0; 0, -10], 'AbsTol', 1e-12);
end

function failedScaledBarrierTimedSearchRetainsValidatedBestSoFar(testCase)
    barrierX_units = [-0.2; -0.2; 0.2; 0.2];
    barrierY_units = [-3; 3; 3; -3];
    obstacleTime_s = [0; 6; 6.5; 12];
    barrierYOffset_units = [0; 0; 8; 8];
    barrier = obstacleAvoidance.obstacles.createObstacle( ...
        'translating barrier', obstacleTime_s, repmat({barrierX_units}, 4, 1), ...
        arrayfun(@(offset_units) barrierY_units + offset_units, ...
        barrierYOffset_units, 'UniformOutput', false), 0.1);

    remoteCenterY_units = 1e9;
    remoteBox_units = [-1, -1; 1, -1; 1, 1; -1, 1] + [0, remoteCenterY_units];
    remoteBox = obstacleAvoidance.obstacles.createObstacle( ...
        'remote convex box', 0, {remoteBox_units(:, 1)}, ...
        {remoteBox_units(:, 2)}, 0.1);
    obstacles = obstacleAvoidance.obstacles.combineObstacles(barrier, remoteBox);

    limits = struct( ...
        'xInterval_units',          [-180, 180], ...
        'yInterval_units',          [-5, remoteCenterY_units + 2], ...
        'maxVelocity_units_s',      [2, 2], ...
        'maxAcceleration_units_s2', [1, 1], ...
        'maxJerk_units_s3',         [2, 2]);
    options = struct( ...
        'GoalTimeMode',                  'earliestArrival', ...
        'WrapX',                         true, ...
        'TemporalResolution_s',          0.5, ...
        'BestSoFarRefinementTrialLimit', 0);

    result = planner(obstacles, restState(0, [-5, 0]), ...
        restState(12, [5, 0]), limits, options);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, "c3DepartureSchedule");
    verifyEqual(testCase, [result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture", "timedVisibility"]);
    verifyTrue(testCase, result.Diagnostics.Attempts(1).Selected);
    verifyFalse(testCase, result.Diagnostics.Attempts(2).Success);
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureStage, "numericalSolver");
    verifyEqual(testCase, result.Diagnostics.Attempts(2).FailureKind, ...
        "optimizerIterateUnavailable");
    verifyFalse(testCase, result.Diagnostics.Attempts(2).NextMethodAllowed);
    verifyEqual(testCase, result.Diagnostics.Attempts(2).SolverExitFlag, -10);
    verifySubstring(testCase, result.Diagnostics.Attempts(2).Message, "numerically unstable");

    timedSearch = result.Diagnostics.SolverDiagnostics.TimedSearchAttempt;
    verifyEqual(testCase, timedSearch.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyNotEmpty(testCase, timedSearch.VisibilityGraph.RouteTime_s);
    verifyEqual(testCase, timedSearch.Route_units, ...
        timedSearch.VisibilityGraph.Route_units);
end

function wrappedBestSoFarRefinementRetainsValidatedDeparture(testCase)
    x=[-0.2;-0.2;0.2;0.2];
    y=[-3;3;3;-3];
    obstacleTimes_s=[0;6;6.5;12];
    yOffsets_units=[0;0;8;8];
    obstacle=obstacleAvoidance.obstacles.createObstacle('translating barrier', ...
        obstacleTimes_s,repmat({x},4,1), ...
        arrayfun(@(offset_units) {y+offset_units},yOffsets_units),0.1);
    initial=restState(0,[-5,0]);
    goal=restState(12,[5,0]);
    limits=struct('xInterval_units',[-180,180], ...
        'yInterval_units',[-3,3],'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[1,1],'maxJerk_units_s3',[2,2]);
    options=struct('GoalTimeMode','earliestArrival','WrapX',true, ...
        'TemporalResolution_s',0.5,'BestSoFarRefinementTrialLimit',1);

    result=planner(obstacle,initial,goal,limits,options);

    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    % Miter-joined margins (e803888) extend the protected corners slightly,
    % so the departure arrives at 10.1437500676 s (square joins gave
    % 10.1400889026 s). The failed refinement must retain this validated motion.
    verifyEqual(testCase,result.ArrivalTime_s,10.1437500676454,'AbsTol',1e-8);
    verifyFalse(testCase,isfield(result.Diagnostics, 'FixedArrivalTrialTime_s'));
    verifyTrue(testCase,result.Diagnostics.TemporalSearch.BestSoFar);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s,7.5,'AbsTol',1e-12);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility","arrivalTimeTrial"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Success);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Selected);
    verifyFalse(testCase,result.Diagnostics.Attempts(3).Success);
    verifyFalse(testCase,result.Diagnostics.Attempts(3).Selected);
    verifyEqual(testCase,result.Options.WrapX,"both");
    verifyEqual(testCase,result.Diagnostics.SuppliedLimits,limits);
    verifyEqual(testCase,result.Diagnostics.RequestedLimits.xInterval_units,[-180,180]);
    verifyEqual(testCase,result.Diagnostics.Limits.xInterval_units,[-29,19]);
end

function planarRequestSelectsRefinementOverWrappedBestSoFar(testCase)
    initial=restState(0,[179,0]);
    goal=restState(12,[-179,0]);
    publicLimits=fastLimits([-3,3]);
    publicLimits.xInterval_units=[-180,180];
    base=planner([],initial,goal,publicLimits, ...
        struct('GoalTimeMode','earliestArrival','WrapX',true));

    base.ArrivalTime_s=10;
    base.Diagnostics.Attempts(1).CandidateArrival_s=10;
    base.Diagnostics.Attempts(1).Success=true;
    base.Diagnostics.Attempts(1).Selected=true;
    base.Diagnostics.ElapsedTime_s=0;

    unwrappedOptions=base.Options;
    unwrappedOptions.WrapX=false;
    unwrappedOptions.WrapY=false;
    unwrappedOptions.TemporalResolution_s=6;
    planarGoal=base.Inputs.goalState;
    request=struct( ...
        'initialState',base.Inputs.initialState, ...
        'goalState',planarGoal, ...
        'limits',base.Diagnostics.Limits, ...
        'options',unwrappedOptions);
    % The parent request is the public wrapped request; the saved inputs
    % below belong to its unwrapped copy.
    parentRequest=obstacleAvoidance.planning.createParentRequest( ...
        struct('goalState',goal,'options',base.Options, ...
        'obstacles',{base.Inputs.obstacles}, ...
        'originalInputs',struct( ...
        'suppliedLimits',publicLimits, ...
        'suppliedGoalState',goal, ...
        'requestedLimits',base.Diagnostics.RequestedLimits, ...
        'requestedGoalState',base.Diagnostics.RequestedGoalState)));
    request.obstacles=base.Inputs.obstacles;
    request.parentRequest=parentRequest;
    request.originalInputs=struct( ...
        'suppliedLimits',base.Diagnostics.Limits, ...
        'requestedLimits',base.Diagnostics.Limits, ...
        'suppliedGoalState',planarGoal, ...
        'requestedGoalState',planarGoal);
    snapshot=obstacleAvoidance.obstacles.snapshot( ...
        base.Diagnostics.PreparedObstacles,request.initialState.time_s);
    scene=struct('preparedObstacles',base.Diagnostics.PreparedObstacles, ...
        'vertexVisibility',obstacleAvoidance.search.createVertexVisibility( ...
        snapshot,request.limits,request.options));
    result=obstacleAvoidance.planning.searchArrivalTimes( ...
        request,scene,base,base.Diagnostics.Attempts,1);

    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.Diagnostics.PreparedObstacles,scene.preparedObstacles);
    verifyEqual(testCase,result.Inputs.initialState,request.initialState);
    verifyEqual(testCase,result.Inputs.goalState.position_units, ...
        request.goalState.position_units);
    verifyEqual(testCase,result.Diagnostics.Limits,request.limits);
    verifyEqual(testCase,result.Options.WrapX,"both");
    verifyEqual(testCase,result.Diagnostics.FixedArrivalTrialTime_s,6,'AbsTol',1e-12);
    verifyEqual(testCase,result.ArrivalTime_s,6,'AbsTol',1e-12);
    verifyFalse(testCase,result.Diagnostics.TemporalSearch.BestSoFar);
    verifyEqual(testCase,result.Diagnostics.TemporalSearch.TrialTime_s,6,'AbsTol',1e-12);
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyFalse(testCase,result.Diagnostics.Attempts(1).Selected);
    verifyEqual(testCase,result.Diagnostics.Attempts(2).TrialTime_s,6,'AbsTol',1e-12);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).Success);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).Selected);
end

function challengedDelayedChordBestSoFarIsRetained(testCase)
    box=[-0.6,-1.5;0.6,-1.5;0.6,1.5;-0.6,1.5];
    obstacleTime_s=[0;5.5;6.5;15];
    obstacle=obstacleAvoidance.obstacles.createObstacle('moving box', ...
        obstacleTime_s,{box(:,1);box(:,1);box(:,1);box(:,1)}, ...
        {box(:,2);box(:,2);box(:,2)+4;box(:,2)+4},0.1);
    initial=struct('time_s',0,'position_units',[-5,0]);
    goal=struct('time_s',15,'position_units',[5,0]);
    limits=struct('xInterval_units',[-6,6], ...
        'yInterval_units',[-4,4],'maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[0.5,0.5], ...
        'maxJerk_units_s3',[2.5,2.5]);
    options=struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',1);
    result=planner(obstacle,initial,goal,limits,options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'FixedArrivalTrialTime_s'));
end

function sparseDynamicZeroWaitDeparture(testCase)
    box=[-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    first=box+[0,8];
    last=box+[2,8];
    obstacle=obstacleAvoidance.obstacles.createObstacle('remote mover',[0;20], ...
        {first(:,1);last(:,1)},{first(:,2);last(:,2)},0);
    result=planner(obstacle,state(0,[0,0]),state(20,[4,0]), ...
        standardLimits(),struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"c3DepartureSchedule");
    verifyFalse(testCase,result.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics,'DepartureSchedule'));
    verifyFalse(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(1).Selected);
    verifyFalse(testCase,result.Diagnostics.EarliestArrival.GlobalEarliestProven);
    selectedIndex=result.Diagnostics.EarliestArrival.SelectedAttemptIndex;
    verifyEqual(testCase,result.Diagnostics.EarliestArrival.BestSoFarArrival_s, ...
        result.ArrivalTime_s,'AbsTol',1e-12);
    verifyEqual(testCase,result.Diagnostics.Attempts(selectedIndex).CandidateArrival_s, ...
        result.ArrivalTime_s,'AbsTol',1e-12);
end

function equivalentSparseAndDenseHistoriesUseProvenDeparture(testCase)
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
        verifyEqual(testCase,results{historyIndex}.Diagnostics.VisibilityGraph.SearchKind, ...
            "c3DepartureSchedule");
        verifyFalse(testCase,isfield(results{historyIndex}.Diagnostics,'TemporalSearch'));
    end
    verifyEqual(testCase,results{1}.ArrivalTime_s,results{2}.ArrivalTime_s, ...
        'AbsTol',1e-8);
    verifyEqual(testCase,results{1}.MotionLength_units, ...
        results{2}.MotionLength_units,'AbsTol',1e-8);
end

function circleDetourBeatsWaiting(testCase)
    result = exampleMovingCircleNoWrap(struct('PlotOutputs',false,'Verbose',false));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,9);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
    verifyLessThan(testCase,result.ArrivalTime_s, ...
        result.Diagnostics.VisibilityGraph.RouteTime_s(end));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
end

function arrivalSnapshotFindsAnOpeningMissingAtInitialTime(testCase)
    x = [-0.2;0.2;0.2;-0.2];
    bottomY = [-5;-5;1;1];
    bottom = obstacleAvoidance.obstacles.createObstacle('late-moving bottom wall', ...
        [0;12;12.1;30],{x;x;x;x},{bottomY;bottomY;bottomY;bottomY-6},0);
    topY = [3;3;5;5];
    top = obstacleAvoidance.obstacles.createObstacle( ...
        'stationary top wall',0,{x},{topY},0);
    gateY = [1;1;3;3];
    gate = obstacleAvoidance.obstacles.createObstacle('early opening gate', ...
        [0;2;2.1;30],{x;x;x;x},{gateY;gateY;gateY+3;gateY+3},0);
    obstacles = obstacleAvoidance.obstacles.combineObstacles({bottom;top;gate});
    result = planner(obstacles,restState(0,[-5,0]),restState(30,[5,0]), ...
        fastLimits([-5,5]),struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',40));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,4.5+1e-8);
    % The exact timed profile now sees the opening directly; it must not fall
    % through to a fixed-arrival trial merely to rediscover the same clock.
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
end

function timedSearchFindsRouteAfterWholeCurtainDeparts(testCase)
    curtainX = [-0.3;0.3;0.3;-0.3];
    curtainY = [-130;-130;130;130];
    curtain = obstacleAvoidance.obstacles.createObstacle('departing curtain', ...
        [0;2;2.1;30],{curtainX;curtainX;curtainX+20;curtainX+20}, ...
        {curtainY;curtainY;curtainY;curtainY},0);
    blockerX = [-0.2;0.2;0.2;-0.2];
    blockerY = [-1;-1;1;1];
    blocker = obstacleAvoidance.obstacles.createObstacle('late direct blocker', ...
        [0;12;12.1;30],{blockerX;blockerX;blockerX;blockerX}, ...
        {blockerY;blockerY;blockerY+11;blockerY+11},0);
    obstacles = obstacleAvoidance.obstacles.combineObstacles({curtain;blocker});
    result = planner(obstacles,restState(0,[-5,0]),restState(30,[5,0]), ...
        fastLimits([-130,130]),struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',40));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s,4+1e-8);
    % The timed node set retains the stationary blocker's exact boundary even
    % while the departing curtain hides it inside the all-time moving-cell union.
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph");
end

function timedHomotopyPrecedesDelayedDeparture(testCase)
    angle_rad=(0:23).'*(2*pi/24);
    circle_units=[1.5*cos(angle_rad),1.5*sin(angle_rad)];
    obstacleTime_s=[0;5.5;6.5;15];
    centerY_units=[0;0;3;3];
    xByTime_units=repmat({circle_units(:,1)},4,1);
    yByTime_units=arrayfun(@(offset_units) ...
        circle_units(:,2)+offset_units,centerY_units,'UniformOutput',false);
    obstacle=obstacleAvoidance.obstacles.createObstacle('rising circle', ...
        obstacleTime_s,xByTime_units,yByTime_units,0.1);
    initial=struct('time_s',0,'position_units',[-6,0]);
    goal=struct('time_s',15,'position_units',[6,0]);
    limits=struct('maxVelocity_units_s',[2,2], ...
        'maxAcceleration_units_s2',[1,1],'maxJerk_units_s3',[2,2]);
    options=struct('GoalTimeMode','earliestArrival', ...
        'TemporalResolution_s',0.5,'MaxArrivalTrials',1);
    result=planner(obstacle,initial,goal,limits,options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    % The direct departure family waits for the rising obstacle and arrives
    % at 10.7600046331 s. A valid curved timed homotopy is physically earlier
    % and must win even though its spatial path is slightly longer.
    verifyLessThan(testCase,result.ArrivalTime_s,10.7600046331);
    % The shortening pass finds a 12.4646-unit polygon for this homotopy, but
    % preparing it stretches the clock to 8.5742483337 s, later than the
    % retained 13.1152-unit motion at 8.5547 s. Arrival wins: the shorter
    % polygon is refused and the retained homotopy stays under 13.5 units.
    verifyLessThan(testCase,result.ArrivalTime_s,8.5742483337);
    verifyFalse(testCase,result.Diagnostics.SolverDiagnostics.TravelRefinementAccepted);
    verifyLessThan(testCase,result.MotionLength_units,13.5);
    verifyEqual(testCase,result.Diagnostics.VisibilityGraph.SearchKind,"timeExpandedVisibilityGraph");
    verifyTrue(testCase,isfield(result.Diagnostics, 'TemporalSearch'));
    verifyFalse(testCase,isfield(result.Diagnostics.SolverDiagnostics, ...
        'DirectVariableClockAttempt'));
    verifyEqual(testCase,numel(result.Diagnostics.Attempts),2);
    verifyEqual(testCase,[result.Diagnostics.Attempts.Kind], ...
        ["analyticDeparture","timedVisibility"]);
    verifyTrue(testCase,result.Diagnostics.Attempts(2).Selected);
    verifyLessThan(testCase,result.Diagnostics.Attempts(2).CandidateArrival_s, ...
        result.Diagnostics.Attempts(1).CandidateArrival_s);
end

function horizonDoesNotMoveSquareGoalClearing(testCase)
    % A square covers the goal and moves off it at 0.5 units/s; the goal
    % clears at 16.3 s. Timed layers sit at horizon / 8, so the goal window
    % starts at 18 s for a 24 s horizon but at 18.75 s for 30 s. Before the
    % unsampled interval was searched, these arrived at 17.0 s and 18.75 s.
    horizons_s = [24, 30];
    arrivals_s = zeros(size(horizons_s));
    for horizonIndex = 1:numel(horizons_s)
        result = planRisingSquare(horizons_s(horizonIndex), struct('GoalTimeMode', 'earliestArrival'));
        verifySelectedValidMotion(testCase, result);
        arrivals_s(horizonIndex) = result.ArrivalTime_s;
    end
    arrivalTolerance_s = result.Options.ArrivalTimeTolerance_s;
    verifyLessThanOrEqual(testCase, abs(diff(arrivals_s)), arrivalTolerance_s);
    verifyLessThanOrEqual(testCase, max(arrivals_s), 17 + arrivalTolerance_s);
end

function unsampledIntervalWithoutClocksKeepsTimedMotion(testCase)
    % With a 30 s arrival grid, the interval between the 15 s layer and the
    % 18.75 s goal window holds no grid clock. The search tries nothing and
    % the validated timed motion stays selected.
    result = planRisingSquare(30, struct('GoalTimeMode', 'earliestArrival', ...
        'TemporalResolution_s', 30));
    verifySelectedValidMotion(testCase, result);
    selectedAttempt = result.Diagnostics.Attempts(result.Diagnostics.EarliestArrival.SelectedAttemptIndex);
    verifyEqual(testCase, selectedAttempt.Kind, "timedVisibility");
    verifyEqual(testCase, result.Diagnostics.TemporalSearch.TrialInterval_s, [15, 18.75]);
    verifyEqual(testCase, result.Diagnostics.TemporalSearch.SolverTrialCount, 0);
    verifyGreaterThan(testCase, result.ArrivalTime_s, 18.75 - result.Options.ArrivalTimeTolerance_s);
end

function horizonDoesNotMoveDriftingTriangleClearing(testCase)
    % A different shape, direction, and set of limits: a triangle covers the
    % goal and drifts away diagonally. Before the unsampled interval was
    % searched, a 20 s horizon arrived at 17.5 s while 32 s arrived at 16.5 s.
    horizons_s       = [20, 32];
    arrivals_s       = zeros(size(horizons_s));
    triangle_units   = [-2, -1.5; 2, -1.5; 0, 2];
    triangleLimits   = struct('maxVelocity_units_s', [1.5, 1.5], ...
        'maxAcceleration_units_s2', [0.6, 0.6], 'maxJerk_units_s3', [2, 2]);
    for horizonIndex = 1:numel(horizons_s)
        horizon_s       = horizons_s(horizonIndex);
        endCenter_units = [9, 3] + [0.12, 0.1] * horizon_s;
        obstacle        = obstacleAvoidance.obstacles.createObstacle('drifting triangle', [0; horizon_s], ...
            {triangle_units(:, 1) + 9; triangle_units(:, 1) + endCenter_units(1)}, ...
            {triangle_units(:, 2) + 3; triangle_units(:, 2) + endCenter_units(2)}, 0.1);
        result = planner(obstacle, restState(0, [-3, 0]), restState(horizon_s, [9, 3]), ...
            triangleLimits, struct('GoalTimeMode', 'earliestArrival'));
        verifySelectedValidMotion(testCase, result);
        arrivals_s(horizonIndex) = result.ArrivalTime_s;
    end
    arrivalTolerance_s = result.Options.ArrivalTimeTolerance_s;
    verifyLessThanOrEqual(testCase, abs(diff(arrivals_s)), arrivalTolerance_s);
    verifyLessThanOrEqual(testCase, max(arrivals_s), 16.5 + arrivalTolerance_s);
end

function verifySelectedValidMotion(testCase, result)
    % The returned motion must pass the public validator, and exactly one
    % successful attempt, the one that produced it, must be selected.
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    selectedIndices = find([result.Diagnostics.Attempts.Selected]);
    verifyEqual(testCase, numel(selectedIndices), 1);
    verifyEqual(testCase, result.Diagnostics.EarliestArrival.SelectedAttemptIndex, selectedIndices(1));
    selectedAttempt = result.Diagnostics.Attempts(selectedIndices(1));
    verifyTrue(testCase, selectedAttempt.Success);
    verifyEqual(testCase, selectedAttempt.CandidateArrival_s, result.ArrivalTime_s, ...
        'AbsTol', result.Options.ArrivalTimeTolerance_s);
end

function result = planRisingSquare(horizon_s, options)
    % A 4-by-4 square centered on (8, -6) at 0 s moves up at 0.5 units/s
    % and covers the goal (8, 0) until 16.3 s, counting its 0.15 margin.
    square_units = [-2, -2; 2, -2; 2, 2; -2, 2];
    obstacle     = obstacleAvoidance.obstacles.createObstacle('rising square', [0; horizon_s], ...
        {square_units(:, 1) + 8; square_units(:, 1) + 8}, ...
        {square_units(:, 2) - 6; square_units(:, 2) - 6 + 0.5 * horizon_s}, 0.15);
    limits = struct('maxVelocity_units_s', [2, 2], ...
        'maxAcceleration_units_s2', [0.8, 0.8], 'maxJerk_units_s3', [2.5, 2.5]);
    result = planner(obstacle, restState(0, [-8, 0]), restState(horizon_s, [8, 0]), limits, options);
end

function state = restState(time_s,position_units)
    state = struct('time_s',time_s,'position_units',position_units, ...
        'velocity_units_s',[0,0],'acceleration_units_s2',[0,0]);
end

function limits = fastLimits(yInterval_units)
    limits = struct('xInterval_units',[-6,6], ...
        'yInterval_units',yInterval_units, ...
        'maxVelocity_units_s',[10,10], ...
        'maxAcceleration_units_s2',[10,10], ...
        'maxJerk_units_s3',[10,10]);
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

function exerciseFreshTimedOutcomeSchema(testCase)
    % Check the remaining timed outcome constructors with a fresh request.
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
    verifyFalse(testCase,rejected.Success);
    verifyFalse(testCase,rejected.Diagnostics.Validation.Passed);
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

function isBefore=measureBeforeOptimizer(result)
    topLevelNames=string(fieldnames(result));
    diagnosticNames=string(fieldnames(result.Diagnostics));
    isBefore=find(topLevelNames=="MotionLength_units",1)<find(topLevelNames=="Diagnostics",1) && ...
        find(diagnosticNames=="IntegratedSquaredJerk_units2_s5",1)< ...
        find(diagnosticNames=="OptimizerFeasible",1);
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

function [noSpatialSolve, afterFailedSpatial] = createFixedArrivalPipelineFixture()
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

    noSpatialSolve=planner(obstacle,initial,goal,limits,options);
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

    afterFailedSpatial=planner(obstacles,scenario.InitialState,scenario.GoalState, ...
        scenario.Limits,scenario.Options);
end
