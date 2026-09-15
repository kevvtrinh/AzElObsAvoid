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

function testUsableConicIteratePolicy(testCase)
    verifyTrue(testCase,bmtpEngine.hasUsableConicIterate(0,1));
    verifyTrue(testCase,bmtpEngine.hasUsableConicIterate(0,-7));
    verifyFalse(testCase,bmtpEngine.hasUsableConicIterate([],1));
    verifyFalse(testCase,bmtpEngine.hasUsableConicIterate(NaN,1));
    verifyFalse(testCase,bmtpEngine.hasUsableConicIterate(0,0));
end

function testGeneratedPlaneRowsMatchExactOmissionOracle(testCase)
    degree=5;
    segmentCount=2;
    regionCount=3;
    controlCount=segmentCount*(degree+1)*2;
    variableCount=controlCount+4;
    controls=reshape(linspace(-1.3,1.7,controlCount),controlCount,1);
    x=[controls;zeros(4,1)];
    template=struct('Active',true,'Normal',[1,0;0.8,0.6], ...
        'Offset_units',[-0.2,0.1],'TimeFraction',[0,1]);
    planes=repmat(template,segmentCount,regionCount);
    planes(1,2).Normal=[-1,0;-0.6,0.8];
    planes(1,2).Offset_units=[0.3,-0.1];
    planes(1,2).TimeFraction=[0.15,0.8];
    planes(1,3).Normal=[0,-1;0.4,-sqrt(0.84)];
    planes(1,3).Offset_units=[-0.05,0.2];
    planes(2,1).Normal=[0.6,0.8;1,0];
    planes(2,1).TimeFraction=[0.2,0.65];
    planes(2,2).Normal=[-0.8,0.6;-1,0];
    planes(2,2).Offset_units=[0.15,0.25];
    planes(2,3).Normal=[0,-1;-0.8,-0.6];
    planes(2,3).TimeFraction=[0.4,0.9];
    activePairs=true(segmentCount,regionCount);
    slackColumnByPair=zeros(size(activePairs));
    reserve_units=2e-4;
    [rows,bounds]=bmtpEngine.createSelectedPlaneRows(planes,activePairs, ...
        degree,variableCount,slackColumnByPair,reserve_units);
    rowResidual=rows*x-bounds;
    pairResidual=reshape(max(reshape(rowResidual,degree+2,[]),[],1), ...
        regionCount,segmentCount).';
    [selectedPairs,maximumResidual]=bmtpEngine.findViolatedPlanePairs( ...
        x,planes,activePairs,false(size(activePairs)),degree, ...
        slackColumnByPair,reserve_units,-1e9);
    expected=false(size(activePairs));
    for segmentIndex=1:segmentCount
        [~,regionIndex]=max(pairResidual(segmentIndex,:));
        expected(segmentIndex,regionIndex)=true;
    end
    verifyEqual(testCase,maximumResidual,max(pairResidual,[],'all'), ...
        'AbsTol',32*eps(max(1,abs(maximumResidual))));
    verifyEqual(testCase,selectedPairs,expected);
end

function testRetainedPlaneCannotHideAnotherViolatedPair(testCase)
    degree=5;
    variableCount=2*(degree+1)+4;
    x=zeros(variableCount,1);
    planes=repmat(struct('Active',true,'Normal',[1,0;1,0], ...
        'Offset_units',[0.25,0.25],'TimeFraction',[0,1]),1,2);
    planes(2).Offset_units=[0.5,0.5];
    activePairs=true(1,2);
    retainedPairs=[true,false];
    [selectedPairs,maximumResidual]=bmtpEngine.findViolatedPlanePairs( ...
        x,planes,activePairs,retainedPairs,degree,zeros(1,2),0,1e-10);
    verifyEqual(testCase,selectedPairs,[false,true]);
    verifyEqual(testCase,maximumResidual,0.5,'AbsTol',1e-12);
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
    verifyTrue(testCase,output.ConstraintGenerationApplied);
    verifyGreaterThan(testCase,output.SolveCount,1);
    verifyEqual(testCase,output.LoadedPlanePairCount,2);
end

function testTravelRefinementAddsNewCollisionPlanes(testCase)
    box_units = [-0.5, -0.5; 0.5, -0.5; 0.5, 0.5; -0.5, 0.5];
    waypoints_units = [-3, 0; -3, 3; 3, 3; 3, 0];
    controls_units = zeros(3, 9, 2);
    for span = 1:3
        points_units = [repmat(waypoints_units(span, :), 4, 1); ...
            mean(waypoints_units(span:span + 1, :), 1); ...
            repmat(waypoints_units(span + 1, :), 4, 1)];
        controls_units(span, :, :) = reshape(points_units, 1, 9, 2);
    end
    plane = struct( ...
        'Active',          false, ...
        'Verified',        false, ...
        'ExitFlag',        NaN, ...
        'Normal',          zeros(2), ...
        'Offset_units',    zeros(1, 2), ...
        'SignedGap_units', NaN, ...
        'TimeFraction',    [0, 1]);
    initial = struct( ...
        'time_s',                0, ...
        'position_units',        waypoints_units(1, :), ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    goal = struct( ...
        'time_s',                12, ...
        'position_units',        waypoints_units(end, :), ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    limits = struct( ...
        'xInterval_units',          [-10, 10], ...
        'yInterval_units',          [-10, 10], ...
        'maxVelocity_units_s',      [5, 5], ...
        'maxAcceleration_units_s2', [10, 10], ...
        'maxJerk_units_s3',         [20, 20]);
    normalized = planner([], initial, goal, limits, ...
        struct('GoalTimeMode', "earliestArrival"));
    edgeLength_units = vecnorm(diff(waypoints_units), 2, 2);
    seed = struct( ...
        'position_units',    waypoints_units, ...
        'tau',               [0; cumsum(edgeLength_units)] / sum(edgeLength_units), ...
        'UsesVariableClock', true);
    request = bmtpEngine.createSolveRequest(seed, {box_units}, struct('Passed', true), ...
        normalized.Inputs.initialState, normalized.Inputs.goalState, ...
        normalized.Limits, normalized.Options);
    alternating = struct( ...
        'Success',            true, ...
        'SolverMessage',      "A complete time-scoped feasible iterate was retained.", ...
        'ControlPoint_units', controls_units, ...
        'SegmentTime_s',      [4; 4; 4], ...
        'Planes',             repmat(plane, 3, 1), ...
        'TaggedPairs',        false(3, 1));
    diagnostics = struct('ConicSolver', bmtpEngine.accumulateConicDiagnostics());
    reserve_units = normalized.PlaneCertificate.RoundoffReserve_units;
    target_units  = normalized.PlaneCertificate.RequiredGap_units - reserve_units;
    [refined, diagnostics] = bmtpEngine.refineTimedTravel( ...
        request, alternating, diagnostics, target_units, reserve_units);
    verifyTrue(testCase, diagnostics.TravelRefinementAccepted);
    verifyGreaterThan(testCase, diagnostics.TaggedPairCount, 0);
    verifyLessThan(testCase, diagnostics.TravelRefinementFinalLength_units, ...
        diagnostics.TravelRefinementInitialLength_units);
    prepared = struct('CertifiedControlPoint_units', refined.ControlPoint_units, ...
        'SegmentTime_s', refined.SegmentTime_s(:));
    verifyTrue(testCase, bmtpEngine.checkFinalMotion( ...
        request, prepared, reserve_units, target_units).Passed);
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
            [~,reserve_units] = bmtpEngine.createCoordinateTolerances(controls,limits.xInterval_units,limits.yInterval_units);
            target_units = (1+2^20*eps)*result.Options.CollisionClearanceTolerance_units+reserve_units;
            result = bmtpEngine.createMotionOutput(result,request,prepared);
            result.PlaneCertificate = bmtpEngine.checkFinalMotion(request,prepared,reserve_units,target_units);
            assertTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
        end
    end
end

function testSolveRequestCarriesMinimumArrivalBound(testCase)
    initial=struct('time_s',0,'position_units',[-1,0]);
    goal=struct('time_s',10,'position_units',[1,0.5]);
    normalized=planner([],initial,goal,[], ...
        struct('GoalTimeMode',"earliestArrival"));
    seed=struct('position_units',[-1,0;1,0.5],'tau',[0;1], ...
        'Source',"timeExpandedWaitGuide",'UsesVariableClock',true);
    coverage=struct('Passed',true,'MinimumMotionDuration_s',4, ...
        'SeedMotionDuration_s',6, ...
        'ActiveTimeInterval_s',zeros(0,2));
    request=bmtpEngine.createSolveRequest(seed,cell(0,1),coverage, ...
        normalized.Inputs.initialState,normalized.Inputs.goalState, ...
        normalized.Limits,normalized.Options);
    verifyEqual(testCase,request.MinimumMotionDuration_s,4);
    verifyEqual(testCase,request.SeedMotionDuration_s,6);
    verifyTrue(testCase,request.UsesVariableClock);
    verifyEqual(testCase,request.Degree,5);
    warmStart=bmtpEngine.createWarmStart(request);
    verifyEqual(testCase,warmStart.Duration_s,6,'AbsTol',1e-12);
end

function testTimedSolverHonorsRequestIterationLimit(testCase)
    [request, warmStart, diagnostics] = createEmptyTimedSolveFixture(1);
    [result, diagnostics] = bmtpEngine.solveTimedAlternatingTrajectory( ...
        request, warmStart, diagnostics, 0, 0);
    verifyTrue(testCase, result.Success, result.SolverMessage);
    verifyEqual(testCase, request.MaximumAlternatingIterations, 1);
    verifyLessThanOrEqual(testCase, diagnostics.IterationCount, 1);
end

function testTimedSolverReturnsCoherentEmptyRecord(testCase)
    [request, warmStart, diagnostics, target_units, reserve_units] = ...
        createBlockedTimedSolveFixture();
    [result, diagnostics] = bmtpEngine.solveTimedAlternatingTrajectory( ...
        request, warmStart, diagnostics, target_units, reserve_units);
    activePairs = reshape([result.Planes.Active], size(result.Planes));
    verifyFalse(testCase, result.Success);
    verifyEmpty(testCase, result.ControlPoint_units);
    verifyFalse(testCase, any(activePairs, 'all'));
    verifyEqual(testCase, result.TaggedPairs, activePairs);
    verifyEqual(testCase, diagnostics.ApplicablePairCount, 0);
    verifyEqual(testCase, diagnostics.TaggedPairCount, 0);
    verifyEqual(testCase, diagnostics.FinalCollisionPairCount, 0);
end

function testTimedSolverRetainsAtomicRecordAfterLaterRejectedTrial(testCase)
    [request, warmStart, diagnostics, target_units, reserve_units] = ...
        createMovingTimedSolveFixture(1);
    [retainedResult, retainedDiagnostics] = bmtpEngine.solveTimedAlternatingTrajectory( ...
        request, warmStart, diagnostics, target_units, reserve_units);
    [request, warmStart, diagnostics, target_units, reserve_units] = ...
        createMovingTimedSolveFixture(2);
    [result, diagnostics] = bmtpEngine.solveTimedAlternatingTrajectory( ...
        request, warmStart, diagnostics, target_units, reserve_units);
    controlTolerance_units = request.Options.ConstraintTolerance;
    timeTolerance_s        = request.Options.ArrivalTimeTolerance_s;
    planeTolerance_units   = request.Options.ConstraintTolerance;
    verifyTrue(testCase, result.Success, result.SolverMessage);
    verifyEqual(testCase, diagnostics.IterationCount, 2);
    verifyEqual(testCase, diagnostics.LastAttemptMessage, ...
        "A later feasible trial did not improve the retained duration within tolerance.");
    verifyEqual(testCase, result.ControlPoint_units, ...
        retainedResult.ControlPoint_units, 'AbsTol', controlTolerance_units);
    verifyEqual(testCase, result.SegmentTime_s, ...
        retainedResult.SegmentTime_s, 'AbsTol', timeTolerance_s);
    verifyEqual(testCase, result.Planes, retainedResult.Planes, ...
        'AbsTol', planeTolerance_units);
    verifyEqual(testCase, result.TaggedPairs, retainedResult.TaggedPairs);
    verifyEqual(testCase, result.SolverMessage, retainedResult.SolverMessage);
    verifyEqual(testCase, diagnostics.SolverMessage, result.SolverMessage);
    verifyEqual(testCase, diagnostics.ApplicablePairCount, ...
        retainedDiagnostics.ApplicablePairCount);
    verifyEqual(testCase, diagnostics.TaggedPairCount, ...
        retainedDiagnostics.TaggedPairCount);
    verifyEqual(testCase, diagnostics.FinalCollisionPairCount, ...
        retainedDiagnostics.FinalCollisionPairCount);
    verifyGreaterThan(testCase, diagnostics.TaggedPairCount, 0);
end

function testTimedSolverRetainsAtomicRecordAfterLaterFailedTrial(testCase)
    failedTrialIterationLimit = 15;
    [request, warmStart, diagnostics, target_units, reserve_units] = ...
        createMovingTimedSolveFixture(1);
    request.TimedTrajectoryOptions = optimoptions( ...
        request.TimedTrajectoryOptions, 'MaxIterations', failedTrialIterationLimit);
    [retainedResult, retainedDiagnostics] = bmtpEngine.solveTimedAlternatingTrajectory( ...
        request, warmStart, diagnostics, target_units, reserve_units);

    [request, warmStart, diagnostics, target_units, reserve_units] = ...
        createMovingTimedSolveFixture(2);
    request.TimedTrajectoryOptions = optimoptions( ...
        request.TimedTrajectoryOptions, 'MaxIterations', failedTrialIterationLimit);
    [result, diagnostics] = bmtpEngine.solveTimedAlternatingTrajectory( ...
        request, warmStart, diagnostics, target_units, reserve_units);

    controlTolerance_units = request.Options.ConstraintTolerance;
    timeTolerance_s        = request.Options.ArrivalTimeTolerance_s;
    planeTolerance_units   = request.Options.ConstraintTolerance;
    verifyTrue(testCase, retainedResult.Success, retainedResult.SolverMessage);
    verifyTrue(testCase, result.Success, result.SolverMessage);
    verifyEqual(testCase, diagnostics.IterationCount, 2);
    verifyEqual(testCase, diagnostics.LastAttemptMessage, ...
        "Trajectory SOCP failed: Maximum iteration limit is reached.");
    verifyEqual(testCase, result.ControlPoint_units, ...
        retainedResult.ControlPoint_units, 'AbsTol', controlTolerance_units);
    verifyEqual(testCase, result.SegmentTime_s, ...
        retainedResult.SegmentTime_s, 'AbsTol', timeTolerance_s);
    verifyEqual(testCase, result.Planes, retainedResult.Planes, ...
        'AbsTol', planeTolerance_units);
    verifyEqual(testCase, result.TaggedPairs, retainedResult.TaggedPairs);
    verifyEqual(testCase, result.SolverMessage, retainedResult.SolverMessage);
    verifyEqual(testCase, diagnostics.SolverMessage, retainedResult.SolverMessage);
    verifyEqual(testCase, diagnostics.TaggedPairCount, ...
        retainedDiagnostics.TaggedPairCount);
    verifyEqual(testCase, diagnostics.ApplicablePairCount, ...
        retainedDiagnostics.ApplicablePairCount);
    verifyEqual(testCase, diagnostics.FinalCollisionPairCount, ...
        retainedDiagnostics.FinalCollisionPairCount);
    verifyGreaterThan(testCase, diagnostics.TaggedPairCount, 0);
    verifyEqual(testCase, diagnostics.FinalCollisionPairCount, 0);
end

function testTimedProfileDoesNotDependOnSeedSource(testCase)
    initial=struct('time_s',0,'position_units',[-1,0]);
    goal=struct('time_s',10,'position_units',[1,0.5]);
    normalized=planner([],initial,goal,[], ...
        struct('GoalTimeMode',"earliestArrival"));
    coverage=struct('Passed',true,'MinimumMotionDuration_s',4, ...
        'SeedMotionDuration_s',6,'ActiveTimeInterval_s',zeros(0,2), ...
        'BreakTime_s',[0;3;10]);
    firstSeed=struct('position_units',[-1,0;-0.5,0;1,0.5], ...
        'tau',[0;0.25;1],'Source',"firstDiagnosticLabel", ...
        'UsesVariableClock',true);
    secondSeed=firstSeed;
    secondSeed.Source="unrelatedDiagnosticLabel";
    firstRequest=bmtpEngine.createSolveRequest(firstSeed,cell(0,1),coverage, ...
        normalized.Inputs.initialState,normalized.Inputs.goalState, ...
        normalized.Limits,normalized.Options);
    secondRequest=bmtpEngine.createSolveRequest(secondSeed,cell(0,1),coverage, ...
        normalized.Inputs.initialState,normalized.Inputs.goalState, ...
        normalized.Limits,normalized.Options);
    firstWarm=bmtpEngine.createWarmStart(firstRequest);
    secondWarm=bmtpEngine.createWarmStart(secondRequest);
    verifyEqual(testCase,firstRequest.Degree,secondRequest.Degree);
    verifyEqual(testCase,firstRequest.SplitCount,secondRequest.SplitCount);
    verifyEqual(testCase,firstWarm.ControlPoint_units, ...
        secondWarm.ControlPoint_units,'AbsTol',0);
    verifyEqual(testCase,firstWarm.SegmentTime_s, ...
        secondWarm.SegmentTime_s,'AbsTol',0);
    verifyEqual(testCase,firstWarm.RegionActiveBySegment, ...
        secondWarm.RegionActiveBySegment);
end

function testVariableClockUsesActualMovingCellOverlap(testCase)
    box=[-0.25,4.75;0.25,4.75;0.25,5.25;-0.25,5.25];
    controls=zeros(2,6,2);
    controls(1,:,1)=linspace(-2,-1,6);
    controls(2,:,1)=linspace(-1,0,6);
    coverage=struct('Passed',true,'ActiveTimeInterval_s',[0,5;5,10], ...
        'EndRegions_units',{{box;box}});
    request=struct('InitialState',struct('time_s',0), ...
        'Regions_units',{{box;box}},'Coverage',coverage);
    [planes,activePairs,complete,statistics]= ...
        bmtpEngine.createTimeScopedPlanes(controls,[3;3],request,1e-6,1e-8);
    verifyTrue(testCase,complete);
    verifyEqual(testCase,activePairs,[true,false;true,true]);
    verifyEqual(testCase,statistics.ActivePairCount,3);
    verifyEqual(testCase,planes(2,1).TimeFraction,[0,2/3],'AbsTol',1e-12);
    verifyEqual(testCase,planes(2,2).TimeFraction,[2/3,1],'AbsTol',1e-12);
end

function testTimedTrajectoryStepRetainsRelativeClock(testCase)
    limits=struct('xInterval_units',[-3,3],'yInterval_units',[-3,3], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[5,5], ...
        'maxJerk_units_s3',[10,10]);
    options=optimoptions('coneprog','Display','none', ...
        'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    planes=repmat(struct('Active',false),2,0);
    [controls,duration_s,flag]=bmtpEngine.solveTimedTrajectoryStep( ...
        2,5,[-1,0],[1,0],limits,planes,1e-8,8,"fixedArrival", ...
        options,0,[1;3]);
    verifyTrue(testCase,flag>0 || flag==-7);
    verifyNotEmpty(testCase,controls);
    verifyEqual(testCase,duration_s,[2;6],'AbsTol',1e-12);
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
    endpointControls=bmtpEngine.imposeEndpointControls(controls,durations_s,initial,goal);
    verifyEqual(testCase,controls(1,1:3,:),endpointControls(1,1:3,:));
    verifyEqual(testCase,controls(end,end-2:end,:),endpointControls(end,end-2:end,:));
    polynomial = bmtpEngine.createPowerPolynomial(controls,durations_s,0);
    [~,position,velocity,acceleration] = bmtpEngine.evaluatePolynomial(polynomial,[0;8]);
    verifyEqual(testCase,position,[initial.position_units;goal.position_units],'AbsTol',1e-8);
    verifyEqual(testCase,velocity,[initial.velocity_units_s;goal.velocity_units_s],'AbsTol',1e-8);
    verifyEqual(testCase,acceleration,[initial.acceleration_units_s2;goal.acceleration_units_s2],'AbsTol',1e-8);
    [~,~,~,~,leftJerk] = bmtpEngine.evaluatePolynomial(polynomial,[2;6],[1;2]);
    [~,~,~,~,rightJerk] = bmtpEngine.evaluatePolynomial(polynomial,[2;6],[2;3]);
    verifyEqual(testCase,leftJerk,rightJerk,'AbsTol',1e-8);
end

function testConstraintGenerationWithTransformedFixedClock(testCase)
    initial=struct('position_units',[-1,0],'velocity_units_s',[0,0], ...
        'acceleration_units_s2',[0,0]);
    goal=struct('position_units',[1,0],'velocity_units_s',[0,0], ...
        'acceleration_units_s2',[0,0]);
    limits=struct('xInterval_units',[-2,2],'yInterval_units',[-2,2], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[5,5], ...
        'maxJerk_units_s3',[10,10]);
    options=optimoptions('coneprog','Display','none', ...
        'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    emptyPlane=struct('Active',false,'Normal',zeros(2), ...
        'Offset_units',zeros(1,2),'TimeFraction',[0,1]);
    planes=repmat(emptyPlane,9,1);
    planes(4)=struct('Active',true,'Normal',[0,1;0,1], ...
        'Offset_units',[0.1,0.1],'TimeFraction',[0.2,0.8]);
    planes(6)=struct('Active',true,'Normal',[0,-1;0,-1], ...
        'Offset_units',[0.1,0.1],'TimeFraction',[0,1]);
    [controls,durations_s,flag,output]=bmtpEngine.solveTrajectoryStep( ...
        9,5,initial,goal,limits,planes,1e-8,18,options,ones(9,1),true);
    assertTrue(testCase,flag>0 || flag==-7);
    verifyTrue(testCase,output.IntrinsicJerkVariation);
    verifyTrue(testCase,output.ConstraintGenerationComplete);
    verifyGreaterThanOrEqual(testCase,output.ConstraintGenerationRoundCount,1);
    verifyTrue(testCase,output.ConstraintGenerationApplied);
    verifyEqual(testCase,output.LoadedPlanePairCount,2);
    verifyEqual(testCase,durations_s,2*ones(9,1),'AbsTol',1e-12);
    fourth=bmtpEngine.restrictBezier(squeeze(controls(4,:,:)),[0.2,0.8]);
    verifyLessThanOrEqual(testCase,max(fourth(:,2)),-0.1+1e-6);
    verifyGreaterThanOrEqual(testCase,min(controls(6,:,2)),0.1-1e-6);
end

function testConstraintGenerationClosesSeveralPairsInOneSpan(testCase)
    initial=struct('position_units',[-2,0],'velocity_units_s',[0,0], ...
        'acceleration_units_s2',[0,0]);
    goal=struct('position_units',[2,0],'velocity_units_s',[0,0], ...
        'acceleration_units_s2',[0,0]);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-5,5], ...
        'maxVelocity_units_s',[10,10],'maxAcceleration_units_s2',[10,10], ...
        'maxJerk_units_s3',[10,10]);
    plane=struct('Active',true,'Verified',true,'ExitFlag',1, ...
        'Normal',repmat([0,1],2,1),'Offset_units',[0.5,0.5], ...
        'SignedGap_units',1,'TimeFraction',[0.2,0.3]);
    planes=repmat(plane,1,3);
    planes(2).Normal=repmat([0,-1],2,1);
    planes(2).TimeFraction=[0.45,0.55];
    planes(3).TimeFraction=[0.7,0.8];
    options=optimoptions('coneprog','Display','none', ...
        'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-9);
    [controlPoint_units,~,exitFlag,output]= ...
        bmtpEngine.solveTrajectoryStep(1,8,initial,goal,limits,planes, ...
        0,20,options,1,true);
    verifyNotEmpty(testCase,controlPoint_units);
    verifyTrue(testCase,exitFlag>0 || exitFlag==-7);
    verifyTrue(testCase,output.ConstraintGenerationComplete);
    verifyEqual(testCase,output.LoadedPlanePairCount,3);
    verifyGreaterThanOrEqual(testCase,output.ConstraintGenerationRoundCount,3);
    verifyLessThanOrEqual(testCase, ...
        output.MaximumPlaneConstraintResidual,options.ConstraintTolerance);
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
    verifyLessThan(testCase,result.Polynomial.SegmentCount,80);
    % Eighty identical-velocity source intervals are one exact affine cell.
    verifyEqual(testCase,result.PlaneCertificate.SolverRegionCount,1);
    verifyEqual(testCase,result.PreparedObstacles.InternalPreparation.MergedIntervalCount,79);
    verifyGreaterThan(testCase,result.SolverDiagnostics.TrajectorySocpCount,0);
end

function testTimedSearchMovingAndStationaryIntervals(testCase)
    box = [-0.2,-3;0.2,-3;0.2,3;-0.2,3];
    nodes = [-5,0;5,0;-2,0;2,0;-2,2;2,2];
    cost = hypot(nodes(:,1)-nodes(:,1).',nodes(:,2)-nodes(:,2).');
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
    verifyEqual(testCase,result.GoalArrivalWindow_s(2),180,'AbsTol',1e-8);
    verifyGreaterThanOrEqual(testCase,result.ArrivalTime_s, ...
        result.GoalArrivalWindow_s(1)-1e-8);
    verifyLessThanOrEqual(testCase,result.ArrivalTime_s, ...
        result.GoalArrivalWindow_s(2)+1e-8);
    verifyEqual(testCase,result.VisibilityGraph.SearchKind,"timeExpandedVisibilityGraph");
    verifyEqual(testCase,result.SolverDiagnostics.Identifier,"bmtpTimeCellsDegree5");
    verifyEqual(testCase,result.TemporalSearch.TrialStage,"timeExpandedVisibilityGraph");
    verifyEqual(testCase,result.SolverDiagnostics.TaggedPairCount, ...
        result.SolverDiagnostics.ApplicablePairCount);
    verifyEqual(testCase,result.SolverDiagnostics.FinalCollisionPairCount,0);
end

function [request, warmStart, diagnostics] = createEmptyTimedSolveFixture(maximumIterationCount)
    initial = struct('time_s', 0, 'position_units', [-1, 0]);
    goal    = struct('time_s', 10, 'position_units', [1, 0.5]);
    normalized = planner([], initial, goal, [], struct('GoalTimeMode', "earliestArrival"));
    seed = struct( ...
        'position_units',              [initial.position_units; goal.position_units], ...
        'tau',                         [0; 1], ...
        'UsesVariableClock',           true, ...
        'MaximumAlternatingIterations', maximumIterationCount);
    coverage = struct( ...
        'Passed',                  true, ...
        'MinimumMotionDuration_s', 4, ...
        'SeedMotionDuration_s',    6, ...
        'ActiveTimeInterval_s',    zeros(0, 2));
    request = bmtpEngine.createSolveRequest(seed, cell(0, 1), coverage, ...
        normalized.Inputs.initialState, normalized.Inputs.goalState, ...
        normalized.Limits, normalized.Options);
    warmStart = bmtpEngine.createWarmStart(request);
    diagnostics = struct( ...
        'IterationCount',          0, ...
        'Converged',               false, ...
        'ApplicablePairCount',     0, ...
        'TrajectorySocpCount',     0, ...
        'FinalCollisionPairCount', 0, ...
        'SolverMessage',           "", ...
        'ConicSolver',             bmtpEngine.accumulateConicDiagnostics());
end

function [request, warmStart, diagnostics, target_units, reserve_units] = ...
        createBlockedTimedSolveFixture()
    initial = struct('time_s', 0, 'position_units', [-1, 0]);
    goal    = struct('time_s', 10, 'position_units', [1, 0]);
    normalized = planner([], initial, goal, [], struct('GoalTimeMode', "earliestArrival"));
    seed = struct( ...
        'position_units',    [initial.position_units; goal.position_units], ...
        'tau',               [0; 1], ...
        'UsesVariableClock', true);
    blockingRegion_units = [-2, -2; 2, -2; 2, 2; -2, 2];
    coverage = struct( ...
        'Passed',                  true, ...
        'MinimumMotionDuration_s', 0, ...
        'SeedMotionDuration_s',    6, ...
        'ActiveTimeInterval_s',    [0, 10]);
    request = bmtpEngine.createSolveRequest(seed, {blockingRegion_units}, coverage, ...
        normalized.Inputs.initialState, normalized.Inputs.goalState, ...
        normalized.Limits, normalized.Options);
    warmStart = bmtpEngine.createWarmStart(request);
    diagnostics = struct( ...
        'IterationCount',          0, ...
        'Converged',               false, ...
        'ApplicablePairCount',     nnz(warmStart.RegionActiveBySegment), ...
        'TrajectorySocpCount',     0, ...
        'FinalCollisionPairCount', 0, ...
        'SolverMessage',           "", ...
        'ConicSolver',             bmtpEngine.accumulateConicDiagnostics());
    reserve_units = normalized.PlaneCertificate.RoundoffReserve_units;
    target_units  = normalized.PlaneCertificate.RequiredGap_units - reserve_units;
end

function [request, warmStart, diagnostics, target_units, reserve_units] = ...
        createMovingTimedSolveFixture(maximumIterationCount)
    initial = struct('time_s', 0, 'position_units', [-4, 0]);
    goal    = struct('time_s', 20, 'position_units', [4, 0]);
    limits = struct( ...
        'xInterval_units',          [-6, 6], ...
        'yInterval_units',          [-6, 6], ...
        'maxVelocity_units_s',      [3, 3], ...
        'maxAcceleration_units_s2', [3, 3], ...
        'maxJerk_units_s3',         [6, 6]);
    normalized = planner([], initial, goal, limits, struct('GoalTimeMode', "earliestArrival"));
    route_units = [-4, 0; -1, 2; 1, 2; 4, 0];
    routeLength_units = vecnorm(diff(route_units), 2, 2);
    firstRegion_units = [-0.5, -1.5; 0.5, -1.5; 0.5, -0.5; -0.5, -0.5];
    lastRegion_units  = firstRegion_units + [0, 2];
    seed = struct( ...
        'position_units',               route_units, ...
        'tau',                          [0; cumsum(routeLength_units)] / sum(routeLength_units), ...
        'UsesVariableClock',            true, ...
        'MaximumAlternatingIterations', maximumIterationCount);
    coverage = struct( ...
        'Passed',                  true, ...
        'EndRegions_units',        {{lastRegion_units}}, ...
        'ActiveTimeInterval_s',    [0, 20], ...
        'MinimumMotionDuration_s', 0, ...
        'SeedMotionDuration_s',    12);
    request = bmtpEngine.createSolveRequest(seed, {firstRegion_units}, coverage, ...
        normalized.Inputs.initialState, normalized.Inputs.goalState, ...
        normalized.Limits, normalized.Options);
    warmStart = bmtpEngine.createWarmStart(request);
    diagnostics = struct( ...
        'IterationCount',          0, ...
        'Converged',               false, ...
        'ApplicablePairCount',     nnz(warmStart.RegionActiveBySegment), ...
        'TrajectorySocpCount',     0, ...
        'FinalCollisionPairCount', 0, ...
        'PlaneSocpCount',          0, ...
        'SolverMessage',           "");
    reserve_units = normalized.PlaneCertificate.RoundoffReserve_units;
    target_units  = normalized.PlaneCertificate.RequiredGap_units - reserve_units;
end
