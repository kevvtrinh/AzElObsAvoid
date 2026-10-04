function tests = testStaticActivePairBmtp
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testStaticActivePairBmtp.m')
% PURPOSE: Regress collision-driven static BMTP on structurally different concave and multi-obstacle routes.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent validation and solver-representation checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testConcaveCavityEscape(testCase)
    angle_rad=linspace(pi/3,5*pi/3,10).';
    vertices_units=[8*cos(angle_rad),8*sin(angle_rad); ...
        4*cos(flipud(angle_rad)),4*sin(flipud(angle_rad))];
    obstacle=obstacleAvoidance.obstacles.createObstacle( ...
        'C cavity',[0;260],vertices_units(:,1),vertices_units(:,2),0.1);
    limits=struct('xInterval_units',[-23,23],'yInterval_units',[-23,23], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.8,0.8], ...
        'maxJerk_units_s3',[3,3]);
    result=planner(obstacle,state([0,0],0),state([-13,0],260),limits,options());
    verifyValidatedStaticBmtp(testCase,result);
end

function testSeparatedSlalomBarriers(testCase)
    center_units=[-6,-2.5;0,2.5;6,-2.5];
    obstacleList=cell(3,1);
    for obstacleIndex=1:3
        vertices_units=center_units(obstacleIndex,:)+[-0.7,-3;0.7,-3;0.7,3;-0.7,3];
        obstacleList{obstacleIndex}=obstacleAvoidance.obstacles.createObstacle( ...
            "barrier "+obstacleIndex,[0;110],vertices_units(:,1),vertices_units(:,2),0.1);
    end
    obstacles=obstacleAvoidance.obstacles.combineObstacles(obstacleList{:});
    limits=struct('xInterval_units',[-17,17],'yInterval_units',[-10,10], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[1,1], ...
        'maxJerk_units_s3',[2,2]);
    result=planner(obstacles,state([-13,0],0),state([13,0],110),limits,options());
    verifyValidatedStaticBmtp(testCase,result);
    verifyTrue(testCase,result.Diagnostics.SolverDiagnostics.TravelRefinementAccepted);
end

function testArrivalImprovementToleranceEndsRefinement(testCase)
    % An L-shaped wall with both endpoints outside it. The start is above the
    % tall side and the goal sits beside the foot, so the route wraps the top
    % outer corner and runs down the wall. Each rebuild of the separating
    % lines then gains a little less arrival time than the last: refining
    % until a pass gains nothing takes 19 passes here, the default stop 4.
    % Unlike the cavity escape, neither endpoint is enclosed.
    vertices_units=[-8,-6;2,-6;2,-4;0,-4;0,4;-8,4];
    obstacle=obstacleAvoidance.obstacles.createObstacle( ...
        'L wall',[0;120],vertices_units(:,1),vertices_units(:,2),0.1);
    limits=struct('xInterval_units',[-20,20],'yInterval_units',[-10,10], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[1,1], ...
        'maxJerk_units_s3',[2.5,2.5]);
    initial=state([-4,6],0);
    goal=state([4,-2],120);
    stopped=planner(obstacle,initial,goal,limits,options());
    verifyValidatedStaticBmtp(testCase,stopped);
    stoppedSolver=stopped.Diagnostics.SolverDiagnostics;
    verifyTrue(testCase,stoppedSolver.Converged);
    verifyLessThan(testCase,stoppedSolver.IterationCount, ...
        stoppedSolver.MaximumAlternatingIterations);
    % Refining until a pass gains nothing repeats the same passes and then
    % continues, so it can only arrive earlier, and only by the small gains
    % the default stop gave up. The shortening pass may move either arrival
    % by at most one millionth of its duration.
    exactOptions=options();
    exactOptions.ArrivalImprovementTolerance_s=0;
    exact=planner(obstacle,initial,goal,limits,exactOptions);
    verifyValidatedStaticBmtp(testCase,exact);
    verifyLessThanOrEqual(testCase,stoppedSolver.IterationCount, ...
        exact.Diagnostics.SolverDiagnostics.IterationCount);
    verifyGreaterThanOrEqual(testCase,stopped.ArrivalTime_s, ...
        exact.ArrivalTime_s-1e-4);
    verifyLessThanOrEqual(testCase,stopped.ArrivalTime_s-exact.ArrivalTime_s,0.05);
end

function testAlternationMayOutlastRequestBeforeShortening(testCase)
    % Five separate walls require turns around alternating ends. First find
    % and validate a motion with a generous request. A horizon at 1.04 x
    % that arrival is feasible because the witness exists; this is a test
    % scenario, not a bound on the alternation. At this horizon HEAD's third
    % SOCP is infeasible, while one relaxed step now finds a motion inside it.
    barrierCount      = 5;
    generousHorizon_s = 200;
    wallCenterX_units = 4 * ((1:barrierCount) - (barrierCount + 1) / 2);
    obstacleList      = cell(barrierCount, 1);
    for obstacleIndex = 1:barrierCount
        centerX_units = wallCenterX_units(obstacleIndex);
        if mod(obstacleIndex, 2) == 1
            lowerY_units = -6;
            upperY_units = 1.2;
        else
            lowerY_units = -1.2;
            upperY_units = 6;
        end
        vertices_units = [centerX_units - 0.5, lowerY_units; ...
            centerX_units + 0.5, lowerY_units; ...
            centerX_units + 0.5, upperY_units; ...
            centerX_units - 0.5, upperY_units];
        obstacleList{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
            "wall " + obstacleIndex, [0; generousHorizon_s], ...
            vertices_units(:, 1), vertices_units(:, 2), 0.1);
    end
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleList{:});
    startX_units = wallCenterX_units(1) - 3;
    goalX_units  = wallCenterX_units(end) + 3;
    limits = struct( ...
        'xInterval_units',          [startX_units - 2, goalX_units + 2], ...
        'yInterval_units',          [-5, 5], ...
        'maxVelocity_units_s',      [1, 1], ...
        'maxAcceleration_units_s2', [0.75, 0.75], ...
        'maxJerk_units_s3',         [2.5, 2.5]);
    initialState = state([startX_units, 0], 0);
    generousGoal = state([goalX_units, 0], generousHorizon_s);
    generousResult = planner(obstacles, initialState, generousGoal, limits, options());
    verifyValidatedStaticBmtp(testCase, generousResult);

    requestHorizon_s = 1.04 * generousResult.ArrivalTime_s;
    for obstacleIndex = 1:numel(obstacles)
        obstacles(obstacleIndex).time_s(end) = requestHorizon_s;
    end
    tightGoal   = state([goalX_units, 0], requestHorizon_s);
    tightResult = planner(obstacles, initialState, tightGoal, limits, options());
    verifyValidatedStaticBmtp(testCase, tightResult);
    verifyLessThanOrEqual(testCase, tightResult.ArrivalTime_s, requestHorizon_s);
end

function testProvenWallTravelExceedsAcceptedRequest(testCase)
    % At x = 0, a clear path must reach |y| >= 5 + 0.1 = 5.1 units. It
    % starts and ends at y = 0, so y travel is at least 2 x 5.1 = 10.2
    % units. With |vy| <= 1 unit/s, every detour needs at least 10.2 s.
    % The straight rest-to-rest chord takes about 9.69 s, so a 10 s request
    % passes the free-space timing check and must fail at motion acceptance.
    vertices_units   = [-1, -5; 1, -5; 1, 5; -1, 5];
    requestHorizon_s = 10;
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'tall wall', [0; requestHorizon_s], vertices_units(:, 1), ...
        vertices_units(:, 2), 0.1);
    limits = struct( ...
        'xInterval_units',          [-6, 6], ...
        'yInterval_units',          [-7, 7], ...
        'maxVelocity_units_s',      [1, 1], ...
        'maxAcceleration_units_s2', [0.75, 0.75], ...
        'maxJerk_units_s3',         [2.5, 2.5]);
    initialState = state([-4, 0], 0);
    goalState    = state([4, 0], requestHorizon_s);
    [~, straightSegmentTime_s] = bmtpEngine.motion.createC3Chord( ...
        initialState.position_units, goalState.position_units, limits);
    verifyLessThan(testCase, sum(straightSegmentTime_s), requestHorizon_s);
    verifyLessThan(testCase, requestHorizon_s, 2 * (5 + 0.1) / limits.maxVelocity_units_s(2));

    result = planner(obstacle, initialState, goalState, limits, options());
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "timeWindowInfeasible");
    verifyEqual(testCase, result.Diagnostics.FailureStage, "optimization");
    verifyEqual(testCase, result.Diagnostics.FailureKind, "timeWindowInfeasible");
    verifyFalse(testCase, result.Diagnostics.AlternativeGuideEligible);
    verifyTrue(testCase, contains(result.Message, "The collision-free motion needs"));
    verifyTrue(testCase, contains(result.Message, "more than the 10.0 s horizon."));
end

function testAffinePlaneBlockConsolidation(testCase)
    limits=struct('xInterval_units',[-100,100],'yInterval_units',[-100,100]);
    source=plane([1,0;0.5,0.5],[-2,-2]);
    target=plane(0.5*source.Normal,[-1.1,-1.1]);
    reduced=bmtpEngine.separation.removeRedundantPlanes([source,target],limits,0.1,true);
    verifyTrue(testCase,reduced(1).Active);
    verifyFalse(testCase,reduced(2).Active);
end

function testAffinePlaneNeedsOneEndpointWeight(testCase)
    limits=struct('xInterval_units',[-10,10],'yInterval_units',[-10,10]);
    source=plane([1,0;1,0],[0,0]);
    differentEndpointScale=plane([0.5,0;0.75,0],[0,0]);
    reduced=bmtpEngine.separation.removeRedundantPlanes( ...
        [source,differentEndpointScale],limits,0,true);
    verifyTrue(testCase,all([reduced.Active]));
end

function testStaticMeshIgnoresCollinearSeedVertices(testCase)
    limits=struct('xInterval_units',[-10,10],'yInterval_units',[-10,10], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[1,1], ...
        'maxJerk_units_s3',[3,3]);
    initial=state([-6,4],0);
    goal=state([6,-1],100);
    normalized=planner([],initial,goal,limits,options());
    route=[initial.position_units;-2,1;goal.position_units];
    midpointRoute=[route(1,:);mean(route(1:2,:),1);route(2,:); ...
        mean(route(2:3,:),1);route(3,:)];
    coverage=struct('Passed',true,'ExactRegionCount',0);
    firstRequest=bmtpEngine.prepareRequest(seed(route), ...
        struct('regions_units', {cell(0,1)}, 'coverage', coverage), ...
        struct('initialState', normalized.Inputs.initialState, ...
        'goalState', normalized.Inputs.goalState, ...
        'limits', normalized.Diagnostics.Limits, ...
        'options', normalized.Options));
    secondRequest=bmtpEngine.prepareRequest(seed(midpointRoute), ...
        struct('regions_units', {cell(0,1)}, 'coverage', coverage), ...
        struct('initialState', normalized.Inputs.initialState, ...
        'goalState', normalized.Inputs.goalState, ...
        'limits', normalized.Diagnostics.Limits, ...
        'options', normalized.Options));
    firstWarm=bmtpEngine.createStartingCurve(firstRequest);
    secondWarm=bmtpEngine.createStartingCurve(secondRequest);
    verifyEqual(testCase,firstWarm.SegmentCount,12);
    verifyEqual(testCase,secondWarm.SegmentCount,firstWarm.SegmentCount);
    verifyEqual(testCase,secondWarm.Route_units,firstWarm.Route_units,'AbsTol',1e-12);
    verifyEqual(testCase,secondWarm.ControlPoint_units, ...
        firstWarm.ControlPoint_units,'AbsTol',1e-12);
    verifyEqual(testCase,secondWarm.SegmentTime_s, ...
        firstWarm.SegmentTime_s,'AbsTol',1e-12);
end

function testReturnedActivePlanesProveReturnedControls(testCase)
    [request, warmStart, diagnostics, target_units, roundoffReserve_units] = ...
        createStaticAlternatingFixture("earliestArrival");
    [result, diagnostics] = bmtpEngine.optimization.solveActivePairTrajectory( ...
        request, warmStart, diagnostics, target_units, roundoffReserve_units);
    verifyTrue(testCase, result.Success, result.SolverMessage);
    verifyEqual(testCase, result.FailureStage, "");
    verifyEqual(testCase, result.FailureKind, "");
    verifyFalse(testCase, result.AlternativeGuideEligible);
    verifyLessThanOrEqual(testCase, diagnostics.IterationCount, ...
        request.MaximumAlternatingIterations);
    verifyEqual(testCase, size(result.Planes), size(result.TaggedPairs));
    activePairs = reshape([result.Planes.Active], size(result.Planes));
    verifyEqual(testCase, result.TaggedPairs, activePairs);
    for pairIndex = reshape(find(activePairs), 1, [])
        [segmentIndex, regionIndex] = ind2sub(size(result.TaggedPairs), pairIndex);
        checkedPlane = bmtpEngine.separation.verifySeparatingLine( ...
            result.Planes(segmentIndex, regionIndex), ...
            squeeze(result.ControlPoint_units(segmentIndex, :, :)), ...
            request.Regions_units{regionIndex}, roundoffReserve_units, target_units);
        verifyTrue(testCase, checkedPlane.Verified);
    end
    verifyEqual(testCase, diagnostics.ApplicablePairCount, nnz(result.TaggedPairs));
    verifyEqual(testCase, diagnostics.TaggedPairCount, nnz(result.TaggedPairs));
    verifyEqual(testCase, diagnostics.FinalCollisionPairCount, 0);
end

function testRowProofReturnsPlanesThatProveReturnedControls(testCase)
    % The constraint-row proof accepts a motion without per-pair verification;
    % every returned plane must still separate the returned controls exactly.
    [request, warmStart, diagnostics, target_units, roundoffReserve_units] = ...
        createStaticAlternatingFixture("fixedArrival");
    [result, diagnostics] = bmtpEngine.optimization.solveAlternatingTrajectory( ...
        request, warmStart, diagnostics, target_units, roundoffReserve_units);
    verifyTrue(testCase, result.Success, result.SolverMessage);
    verifyTrue(testCase, isstruct(result.PreparedMotion));
    verifyTrue(testCase, isstruct(result.Proof));
    verifyEqual(testCase, numel(result.PreparedMotion.SegmentTime_s), ...
        size(result.Proof.RegionActiveBySegment, 1));
    verifyGreaterThan(testCase, diagnostics.ConstraintRowPairVerificationCount, 0);
    verifyEqual(testCase, diagnostics.ExistingPlanePairVerificationCount, 0);
    verifyEqual(testCase, diagnostics.FinalCollisionPairCount, 0);
    activePairs = reshape([result.Planes.Active], size(result.Planes));
    verifyEqual(testCase, result.TaggedPairs, activePairs);
    for pairIndex = reshape(find(activePairs), 1, [])
        [segmentIndex, regionIndex] = ind2sub(size(result.TaggedPairs), pairIndex);
        checkedPlane = bmtpEngine.separation.verifySeparatingLine( ...
            result.Planes(segmentIndex, regionIndex), ...
            squeeze(result.ControlPoint_units(segmentIndex, :, :)), ...
            request.Regions_units{regionIndex}, roundoffReserve_units, target_units);
        verifyTrue(testCase, checkedPlane.Verified);
    end
end

function [request, warmStart, diagnostics, target_units, roundoffReserve_units] = ...
        createStaticAlternatingFixture(goalTimeMode)
    box_units = [-0.5, -0.5; 0.5, -0.5; 0.5, 0.5; -0.5, 0.5];
    initial   = state([-3, 0], 0);
    goal      = state([3, 0], 60);
    limits = struct( ...
        'xInterval_units',          [-5, 5], ...
        'yInterval_units',          [-5, 5], ...
        'maxVelocity_units_s',      [2, 2], ...
        'maxAcceleration_units_s2', [1, 1], ...
        'maxJerk_units_s3',         [3, 3]);
    normalized = planner([], initial, goal, limits, ...
        struct('GoalTimeMode', goalTimeMode));
    route_units = [initial.position_units; -1, 1.5; 1, 1.5; goal.position_units];
    coverage = struct('Passed', true, 'ExactRegionCount', 1);
    request = bmtpEngine.prepareRequest(seed(route_units), ...
        struct('regions_units', {{box_units}}, 'coverage', coverage), ...
        struct('initialState', normalized.Inputs.initialState, ...
        'goalState', normalized.Inputs.goalState, ...
        'limits', normalized.Diagnostics.Limits, ...
        'options', normalized.Options));
    warmStart = bmtpEngine.createStartingCurve(request);
    diagnostics = struct( ...
        'IterationCount',          0, ...
        'Converged',               false, ...
        'ApplicablePairCount',     nnz(warmStart.RegionActiveBySegment), ...
        'TrajectorySocpCount',     0, ...
        'FinalCollisionPairCount', 0, ...
        'PlaneSocpCount',          0, ...
        'SolverMessage',           "");
    roundoffReserve_units = normalized.Diagnostics.SeparationProof.RoundoffReserve_units;
    target_units  = normalized.Diagnostics.SeparationProof.RequiredGap_units - roundoffReserve_units;
end

function verifyValidatedStaticBmtp(testCase,result)
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyGreaterThan(testCase,result.Diagnostics.SolverDiagnostics.TaggedPairCount,0);
    verifyGreaterThanOrEqual(testCase, ...
        result.Diagnostics.SolverDiagnostics.TransientPlaneRemovalCount,0);
end

function value=state(position_units,time_s)
    value=struct('time_s',time_s,'position_units',position_units);
end

function value=options()
    value=struct('GoalTimeMode','earliestArrival','SampleTime_s',0.1);
end

function value=seed(route_units)
    edgeLength_units=vecnorm(diff(route_units),2,2);
    value=struct('position_units',route_units, ...
        'tau',[0;cumsum(edgeLength_units)]/sum(edgeLength_units));
end

function value=plane(normal,offset_units)
    value=struct('Active',true,'Verified',true,'ExitFlag',1, ...
        'Normal',normal,'Offset_units',offset_units,'SignedGap_units',1);
end
