function tests = testPlannerContract
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerContract
%**************************************************************************
% PURPOSE
%   - Verify the maintained public planner contract without legacy-engine
%     names, private entry points, or implementation-specific solver counts.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%       Deterministic public success, failure, validation, and diagnostic tests.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
    % Add the public planner and independent BMTP engine to the MATLAB path.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
end

function testPlanningSceneOwnsPreparedHistoriesAndHorizon(testCase)
    % Expose prepared obstacle details and one shared horizon classification.
    obstacleTime_s  = [0; 10];
    staticObstacle  = obstacleAvoidance.obstacles.createObstacle("static", obstacleTime_s, [-1 1 1 -1], [-1 -1 1 1], 0.1);
    movingStart_deg = [4 -1; 6 -1; 6 1; 4 1];
    movingEnd_deg   = movingStart_deg + [1 0];
    movingObstacle  = obstacleAvoidance.obstacles.createObstacle("moving", obstacleTime_s, {movingStart_deg(:, 1); movingEnd_deg(:, 1)}, {movingStart_deg(:, 2); movingEnd_deg(:, 2)}, 0.1);
    request         = struct("obstacles", obstacleAvoidance.obstacles.combineObstacles(staticObstacle, movingObstacle), ...
        "initialState", restState(0, [-4 0]), ...
        "goalState", restState(10, [8 0]), ...
        "limits", physicalLimits(), ...
        "options", obstacleAvoidance.input.resolvePlannerOptions(plannerOptions("earliestArrival")));
    scene = obstacleAvoidance.obstacles.preparePlanningScene(request.obstacles, request.initialState, request.goalState);

    verifyEqual(testCase, scene.startTime_s, 0);
    verifyEqual(testCase, scene.endTime_s, 10);
    verifyFalse(testCase, scene.obstaclesRemainStatic);
    verifyEqual(testCase, numel(scene.preparedObstacles), 2);
    verifyEqual(testCase, numel(scene.preparedObstacles(1).time_s), 2);
    verifyEqual(testCase, numel(scene.preparedObstacles(2).time_s), 2);
    verifyTrue(testCase, scene.preparedObstacles(1).InternalPreparation.IsTimeInvariant);
    verifyFalse(testCase, scene.preparedObstacles(2).InternalPreparation.IsTimeInvariant);
    verifyEqual(testCase, scene.preparedObstacles(2).InternalPreparation.IntervalGeometryModel, "linearCorrespondingVertices");

    proposal = obstacleAvoidance.search.createRouteSearchGeometry(request.initialState, request.goalState, request.options, scene);
    verifyEqual(testCase, proposal.start_deg, [-4 0]);
    verifyEqual(testCase, proposal.goal_deg, [8 0]);
    verifyEqual(testCase, proposal.sampleTimes_s, linspace(0, 10, 9).');
    maximumVerticesPerObstacle = zeros(1, numel(scene.preparedObstacles));
    for obstacleIndex = 1:numel(scene.preparedObstacles)
        maximumVerticesPerObstacle(obstacleIndex) = max(cellfun(@numel, scene.preparedObstacles(obstacleIndex).az_deg));
    end
    expectedVertexWork = numel(proposal.sampleTimes_s) * sum(maximumVerticesPerObstacle);
    verifyEqual(testCase, proposal.estimatedVertexWork, expectedVertexWork);
    verifyEqual(testCase, proposal.representation, "sampledObstacleUnion");
    verifyFalse(testCase, proposal.usedDenseEnvelope);
    verifyEqual(testCase, proposal.sampledShapeCount, 18);
    verifyEqual(testCase, size(proposal.edgeStart_deg), size(proposal.edgeEnd_deg));

    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(request.limits, proposal);
    verifyGreaterThanOrEqual(testCase, visibilityGraph.FinalAttemptIndex, 1);
    verifyEqual(testCase, numel(visibilityGraph.Attempts), visibilityGraph.FinalAttemptIndex);
    verifyEqual(testCase, visibilityGraph.NodePosition_deg, visibilityGraph.Attempts(end).Nodes.Positions_deg);
    verifyEqual(testCase, visibilityGraph.EdgeCost_deg, visibilityGraph.Attempts(end).Cost_deg);
    verifyEqual(testCase, visibilityGraph.Attempts(end).OffsetRetryCount, visibilityGraph.Record.CandidateOffsetRetryCount);
    verifyTrue(testCase, isfield(visibilityGraph.Attempts, "FinalCandidatePairs"));
    verifyTrue(testCase, isfield(visibilityGraph.Attempts, "EdgeRejectionReasons"));
    verifyTrue(testCase, isfield(visibilityGraph.Attempts, "RecoverySteps"));

    routeSet = obstacleAvoidance.search.searchRoutes(request.initialState, request.goalState, request.limits, request.options, scene, proposal, visibilityGraph);
    seedSet  = obstacleAvoidance.search.createPathGuesses(request.initialState, request.goalState, request.limits, request.options, routeSet, proposal.shape.Vertices);
    verifyEqual(testCase, seedSet(1).Source, "directPathGuess");
    verifyEqual(testCase, [seedSet.Index], 1:numel(seedSet));
    verifyTrue(testCase, isfield(routeSet, "TimedSearchRecord"));
    verifyTrue(testCase, isfield(routeSet, "SpatialSearchRecord"));
    verifyTrue(testCase, isfield(routeSet, "RouteClassPattern"));
end

function testSpatialSeedEstimateNeverRejectsLongGuideRoute(testCase)
    % Keep route-shaped timing guesses from pruning a solver proposal.
    limits = physicalLimits();
    limits.maxVelocity_deg_s = [2 2];
    request = struct("obstacles", obstacleAvoidance.obstacles.combineObstacles([]), ...
        "initialState", restState(0, [0 0]), ...
        "goalState", restState(1, [1 0]), ...
        "limits", limits, ...
        "options", obstacleAvoidance.input.resolvePlannerOptions(plannerOptions("earliestArrival")));
    spatialRoute_deg = [0 0; 0 8; 1 8; 1 0];
    routeSet         = struct("TimedRoute_deg", zeros(0, 2), ...
        "TimedRouteTime_s", zeros(0, 1), ...
        "SpatialRoutes_deg", {{spatialRoute_deg}}, ...
        "UsesConservativeEnvelope", false);
    proposal = struct("shape", polyshape());

    seedSet = obstacleAvoidance.search.createPathGuesses(request.initialState, request.goalState, request.limits, request.options, routeSet, proposal.shape.Vertices);

    verifyEqual(testCase, numel(seedSet), 2);
    verifyEqual(testCase, seedSet(2).position_deg, spatialRoute_deg);
    verifyEqual(testCase, seedSet(2).EstimatedDuration_s, 0.5, "AbsTol", 1e-12);
    verifyGreaterThan(testCase, seedSet(2).Length_deg, request.goalState.time_s - request.initialState.time_s);
end

function testSpatialSearchAllowsMultipleWindingAndEndsWhenDisconnected(testCase)
    % Remove arbitrary winding rejection without losing finite disconnected exit.
    angle_rad      = linspace(0, 4 * pi, 25).';
    radius_deg     = 1 + 0.1 * angle_rad;
    spiralPath_deg = [radius_deg .* cos(angle_rad), ...
        radius_deg .* sin(angle_rad)];
    nodePosition_deg = [spiralPath_deg(1, :); spiralPath_deg(end, :); ...
        spiralPath_deg(2:end - 1, :)];
    pathNodeIndex = [1, 3:size(nodePosition_deg, 1), 2];
    edgeCost_deg  = Inf(size(nodePosition_deg, 1));
    for edgeIndex = 1:numel(pathNodeIndex) - 1
        firstNode      = pathNodeIndex(edgeIndex);
        secondNode     = pathNodeIndex(edgeIndex + 1);
        edgeLength_deg = norm(nodePosition_deg(firstNode, :) - nodePosition_deg(secondNode, :));
        edgeCost_deg(firstNode, secondNode) = edgeLength_deg;
        edgeCost_deg(secondNode, firstNode) = edgeLength_deg;
    end
    [routes_deg, classPattern, ~] = obstacleAvoidance.search.searchDistinctSpatialRoutes(edgeCost_deg, nodePosition_deg, [0 0], 1, @(first_deg, second_deg) true);
    verifyNumElements(testCase, routes_deg, 1);
    verifyEqual(testCase, abs(classPattern), 2);

    request = struct("obstacles", obstacleAvoidance.obstacles.combineObstacles([]), ...
        "initialState", restState(0, nodePosition_deg(1, :)), ...
        "goalState", restState(20, nodePosition_deg(2, :)), ...
        "limits", physicalLimits(), ...
        "options", obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival", "MaximumSeedCount", 2)));
    scene    = obstacleAvoidance.obstacles.preparePlanningScene(request.obstacles, request.initialState, request.goalState);
    proposal = struct("usedDenseEnvelope", false, "sampleTimes_s", [0; 20], ...
        "goal_deg", nodePosition_deg(2, :), "shape", polyshape(), ...
        "edgeStart_deg", zeros(0, 2), "edgeEnd_deg", zeros(0, 2));
    visibilityGraph = struct("NodePosition_deg", nodePosition_deg, "EdgeCost_deg", edgeCost_deg, ...
        "ObstacleReferencePoints_deg", [0 0]);
    routeSet = obstacleAvoidance.search.searchRoutes(request.initialState, request.goalState, request.limits, request.options, scene, proposal, visibilityGraph);
    verifyEmpty(testCase, routeSet.SpatialRoutes_deg);
    verifyNumElements(testCase, routeSet.DeferredSpatialRoutes_deg, 1);
    verifyFalse(testCase, routeSet.DeferredSpatialSolveAttempted);

    nodePosition_deg = [1 0; 4 0; 0 1; -1 0; 0 -1];
    edgeCost_deg     = Inf(5);
    cycleNodeIndex   = [1 3 4 5 1];
    for edgeIndex = 1:numel(cycleNodeIndex) - 1
        firstNode  = cycleNodeIndex(edgeIndex);
        secondNode = cycleNodeIndex(edgeIndex + 1);
        edgeCost_deg(firstNode, secondNode) = 1;
        edgeCost_deg(secondNode, firstNode) = 1;
    end
    [routes_deg, ~, record] = obstacleAvoidance.search.searchDistinctSpatialRoutes(edgeCost_deg, nodePosition_deg, [0 0], 2, @(first_deg, second_deg) true);
    verifyEmpty(testCase, routes_deg);
    verifyEqual(testCase, record.StateCount, 1);
    verifyFalse(testCase, record.Truncated);
end

function testDenseProposalDefersThenRunsExactTimedSearch(testCase)
    % Dense spatial work may reorder exact timed search but cannot discard it.
    obstacleTime_s     = [0; 10];
    firstVertices_deg  = [20 -2; 22 -2; 22 2; 20 2];
    secondVertices_deg = firstVertices_deg + [0 1];
    movingObstacle     = obstacleAvoidance.obstacles.createObstacle("moving", obstacleTime_s, {firstVertices_deg(:, 1); secondVertices_deg(:, 1)}, {firstVertices_deg(:, 2); secondVertices_deg(:, 2)}, 0.1);
    request            = struct("obstacles", movingObstacle, ...
        "initialState", restState(0, [0 0]), ...
        "goalState", restState(10, [2 0]), ...
        "limits", physicalLimits(), ...
        "options", obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "fixedArrival", "MaximumSeedCount", 1, ...
            "MaximumTimeLayerCount", 3)));
    scene    = obstacleAvoidance.obstacles.preparePlanningScene(request.obstacles, request.initialState, request.goalState);
    proposal = struct("usedDenseEnvelope", true, "sampleTimes_s", [0; 5; 10], ...
        "goal_deg", [2 0], "shape", polyshape(), ...
        "edgeStart_deg", zeros(0, 2), "edgeEnd_deg", zeros(0, 2));
    visibilityGraph = struct();
    visibilityGraph.NodePosition_deg            = [0 0; 2 0];
    visibilityGraph.EdgeCost_deg                = [0 2; 2 0];
    visibilityGraph.ObstacleReferencePoints_deg = zeros(0, 2);

    routeSet = obstacleAvoidance.search.searchRoutes(request.initialState, request.goalState, request.limits, request.options, scene, proposal, visibilityGraph);

    verifyFalse(testCase, routeSet.TimedSearchAttempted);
    verifyTrue(testCase, routeSet.TimedSearchDeferred);
    verifyEqual(testCase, routeSet.TimedSearchSuppressionReason, "deferredDenseTimedSearch");

    routeSet.SpatialSearchRecord.RecoverySentinel = 314;
    recoveredRouteSet = obstacleAvoidance.search.searchRoutes(request.initialState, request.goalState, request.limits, request.options, scene, proposal, visibilityGraph, routeSet);

    verifyTrue(testCase, recoveredRouteSet.TimedSearchAttempted);
    verifyTrue(testCase, recoveredRouteSet.TimedSearchDeferred);
    verifyTrue(testCase, recoveredRouteSet.TimedSearchRecoveryAttempted);
    verifyEqual(testCase, recoveredRouteSet.TimedSearchSuppressionReason, "");
    verifyFalse(testCase, isempty(recoveredRouteSet.TimedRoute_deg));
    verifyEqual(testCase, recoveredRouteSet.TimedRoute_deg(1, :), [0 0]);
    verifyEqual(testCase, recoveredRouteSet.TimedRoute_deg(end, :), [2 0]);
    verifyEqual(testCase, recoveredRouteSet.SpatialSearchRecord.RecoverySentinel, 314);
end

function testTimedSearchDoesNotImposeRestAtIntermediateNodes(testCase)
    % Preserve a feasible constant-velocity route through an intermediate node.
    nodePosition_deg = [0 0; 4 0; 2 0];
    edgeCost_deg     = Inf(3);
    edgeCost_deg(1:4:end) = 0;
    edgeCost_deg(1, 3) = 2;
    edgeCost_deg(3, 1) = 2;
    edgeCost_deg(2, 3) = 2;
    edgeCost_deg(3, 2) = 2;
    initialState = restState(0, nodePosition_deg(1, :));
    initialState.velocity_deg_s = [2 0];
    goalState = restState(2, nodePosition_deg(2, :));
    goalState.velocity_deg_s = [2 0];
    limits = physicalLimits();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [0.01 0.01];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival", "MaximumTimeLayerCount", 3));

    [route_deg, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_deg, edgeCost_deg, obstacleAvoidance.obstacles.prepareObstacles(struct.empty(0, 1)), initialState, goalState, limits, [0; 1; 2], options);

    verifyEqual(testCase, route_deg, nodePosition_deg([1 3 2], :));
    verifyEqual(testCase, routeTime_s, [0; 1; 2]);
end

function testTimedSearchPreservesCostTiesAndCompleteFrontier(testCase)
    % Keep optimal through-node ancestry when more expensive cycles are pruned.
    nodePosition_deg = [0 0; 4 0; 2 0; 2 2];
    edgeCost_deg     = zeros(4);
    initialState     = restState(0, nodePosition_deg(1, :));
    goalState        = restState(5, nodePosition_deg(2, :));
    options          = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "fixedArrival"));
    sampleTimes_s    = (0:5).';

    [route_deg, routeTime_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_deg, edgeCost_deg, obstacleAvoidance.obstacles.prepareObstacles(struct.empty(0, 1)), initialState, goalState, physicalLimits(), sampleTimes_s, options);

    verifyEqual(testCase, route_deg, nodePosition_deg([1 3 3 3 3 2], :));
    verifyEqual(testCase, routeTime_s, sampleTimes_s);
    verifyEqual(testCase, sum(vecnorm(diff(route_deg), 2, 2)), 4);
    verifyEqual(testCase, record.FrontierNodes_deg, nodePosition_deg);
    verifyEqual(testCase, record.ReachableGoalLayerCount, 4);
end

function testTimedSearchPreservesFutureFrontierAtEarlyExit(testCase)
    % An early goal exit still exposes later states discovered by colliding edges.
    nodePosition_deg = [0 0; 2 0; 0 1; 1 1];
    blocked_deg      = [0.4 0.9; 0.6 0.9; 0.6 1.1; 0.4 1.1];
    clear_deg        = blocked_deg + [0 3];
    obstacles        = obstacleAvoidance.obstacles.createObstacle("moving barrier", [0; 1; 1.5; 3], {blocked_deg(:, 1); blocked_deg(:, 1); clear_deg(:, 1); clear_deg(:, 1)}, {blocked_deg(:, 2); blocked_deg(:, 2); clear_deg(:, 2); clear_deg(:, 2)}, 0);
    options          = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));

    [route_deg, routeTime_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_deg, zeros(4), obstacleAvoidance.obstacles.prepareObstacles(obstacles), restState(0, nodePosition_deg(1, :)), restState(3, nodePosition_deg(2, :)), physicalLimits(), (0:0.5:3).', options);

    verifyEqual(testCase, route_deg, nodePosition_deg(1:2, :));
    verifyEqual(testCase, routeTime_s, [0; 1]);
    verifyEqual(testCase, record.FrontierNodes_deg, nodePosition_deg(3:4, :));
end

function testTimedSearchRetainsEverySuppliedTime(testCase)
    % Keep a brief input-defined opening that layer thinning would erase.
    blockedVertices_deg = [-0.15 -2; 0.15 -2; 0.15 2; -0.15 2];
    clearVertices_deg   = blockedVertices_deg + [0 30];
    obstacleTime_s      = [0; 3.8; 3.9; 4.2; 4.3; 5];
    barrier             = obstacleAvoidance.obstacles.createObstacle("briefOpening", obstacleTime_s, {blockedVertices_deg(:, 1); blockedVertices_deg(:, 1); clearVertices_deg(:, 1); clearVertices_deg(:, 1); blockedVertices_deg(:, 1); blockedVertices_deg(:, 1)}, {blockedVertices_deg(:, 2); blockedVertices_deg(:, 2); clearVertices_deg(:, 2); clearVertices_deg(:, 2); blockedVertices_deg(:, 2); blockedVertices_deg(:, 2)}, 0);
    nodePosition_deg    = [-2 0; 2 0; 0 0];
    edgeCost_deg        = Inf(3);
    edgeCost_deg(1, 3) = 2;
    edgeCost_deg(3, 1) = 2;
    edgeCost_deg(2, 3) = 2;
    edgeCost_deg(3, 2) = 2;
    initialState = restState(0, nodePosition_deg(1, :));
    goalState    = restState(5, nodePosition_deg(2, :));
    limits       = physicalLimits();
    limits.maxVelocity_deg_s = [20 20];
    options       = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival", "MaximumTimeLayerCount", 17));
    sampleTimes_s = (0:0.1:5).';

    [route_deg, routeTime_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_deg, edgeCost_deg, obstacleAvoidance.obstacles.prepareObstacles(barrier), initialState, goalState, limits, sampleTimes_s, options);

    verifyEqual(testCase, record.LayerTimes_s, sampleTimes_s, "AbsTol", eps(5));
    verifyEqual(testCase, record.CandidateLayerCount, numel(sampleTimes_s));
    verifyGreaterThan(testCase, record.CandidateLayerCount, options.MaximumTimeLayerCount);
    verifyFalse(testCase, isfield(record, "LayerLimitApplied"));
    verifyFalse(testCase, isempty(route_deg));
    verifyEqual(testCase, route_deg(1, :), initialState.position_deg);
    verifyEqual(testCase, route_deg(end, :), goalState.position_deg);
    verifyEqual(testCase, routeTime_s(end), 4, "AbsTol", 1e-12);
end

function testTimedSearchCanArriveLaterWithoutWaitingAtBlockedSource(testCase)
    % Advance within one safe-wait interval after an earlier edge collision.
    square_deg          = [-0.1 -0.1; 0.1 -0.1; 0.1 0.1; -0.1 0.1];
    obstacleTime_s      = [0; 0.25; 0.5; 1; 2];
    azimuthAtSource_deg = repmat({square_deg(:, 1) - 2}, 5, 1);
    azimuthOnEdge_deg   = repmat({square_deg(:, 1)}, 5, 1);
    sourceElevation_deg = { ...
        square_deg(:, 2) + 3; square_deg(:, 2); square_deg(:, 2); square_deg(:, 2) + 3; square_deg(:, 2) + 3};
    edgeElevation_deg = { ...
        square_deg(:, 2) + 3; square_deg(:, 2); square_deg(:, 2) + 3; square_deg(:, 2) + 3; square_deg(:, 2) + 3};
    sourceBlocker    = obstacleAvoidance.obstacles.createObstacle("sourceBlocker", obstacleTime_s, azimuthAtSource_deg, sourceElevation_deg, 0);
    edgeBlocker      = obstacleAvoidance.obstacles.createObstacle("edgeBlocker", obstacleTime_s, azimuthOnEdge_deg, edgeElevation_deg, 0);
    obstacles        = obstacleAvoidance.obstacles.combineObstacles(sourceBlocker, edgeBlocker);
    nodePosition_deg = [-2 0; 2 0];
    edgeCost_deg     = [0 4; 4 0];
    initialState     = restState(0, nodePosition_deg(1, :));
    initialState.velocity_deg_s = [4 0];
    goalState = restState(2, nodePosition_deg(2, :));
    goalState.velocity_deg_s = [4 0];
    limits = physicalLimits();
    limits.maxVelocity_deg_s = [10 10];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));

    [route_deg, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_deg, edgeCost_deg, obstacleAvoidance.obstacles.prepareObstacles(obstacles), initialState, goalState, limits, obstacleTime_s, options);

    fineTime_s       = linspace(0, 1, 101).';
    finePosition_deg = initialState.position_deg + fineTime_s .* (goalState.position_deg - initialState.position_deg);
    occupied         = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, finePosition_deg(:, 1), finePosition_deg(:, 2), fineTime_s);
    verifyFalse(testCase, any(occupied));
    verifyEqual(testCase, route_deg, nodePosition_deg);
    verifyEqual(testCase, routeTime_s, [0; 1]);
end

function testDenseMovingBarrierRecoversDeferredTimedSeed(testCase)
    % Recover a wait route after dense spatial proposals yield no valid motion.
    angle_rad           = (0:1199).' * (2 * pi / 1200);
    blockedVertices_deg = [0.15 * cos(angle_rad), 10 * sin(angle_rad)];
    clearVertices_deg   = blockedVertices_deg + [0 30];
    obstacleTime_s      = [0; 6; 7; 10];
    barrier             = obstacleAvoidance.obstacles.createObstacle("movingDenseBarrier", obstacleTime_s, {blockedVertices_deg(:, 1); blockedVertices_deg(:, 1); clearVertices_deg(:, 1); clearVertices_deg(:, 1)}, {blockedVertices_deg(:, 2); blockedVertices_deg(:, 2); clearVertices_deg(:, 2); clearVertices_deg(:, 2)}, 0.05);
    limits              = physicalLimits();
    limits.maxVelocity_deg_s      = [2 0.5];
    limits.maxAcceleration_deg_s2 = [4 1];
    limits.maxJerk_deg_s3         = [20 4];
    limits.azimuthInterval_deg    = [-3 3];
    limits.elevationInterval_deg  = [-12 12];
    options = struct();
    options.GoalTimeMode          = "fixedArrival";
    options.MaximumSeedCount      = 2;
    options.MaximumTimeLayerCount = 17;
    options.SampleTime_s          = 0.1;

    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory(barrier, restState(0, [-2 0]), restState(10, [2 0]), limits, options);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, resultDiagnosis.Search.DenseSeedEnvelopeUsed);
    verifyTrue(testCase, resultDiagnosis.SearchCoverage.TimedSearchInitialDeferred);
    verifyTrue(testCase, resultDiagnosis.SearchCoverage.TimedSearchRecoveryAttempted);
    verifyTrue(testCase, resultDiagnosis.SearchCoverage.TimedSearchAttempted);
end

function testDirectWaitRetriesFullHorizonAfterShortTimingEstimate(testCase)
    % Keep a short schedule estimate from rejecting an ample request horizon.
    initialState = restState(0, [0 0]);
    goalState    = restState(20, [2 0]);
    limits       = physicalLimits();
    limits.maxAcceleration_deg_s2 = [0.1 0.1];
    limits.maxJerk_deg_s3         = [0.1 0.1];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));
    seed    = obstacleAvoidance.search.createEmptyPathGuess();
    seed.Index               = 1;
    seed.Source              = "directWait";
    seed.position_deg        = [0 0; 0 0; 2 0];
    seed.tau                 = [0; 0.5; 1];
    seed.EstimatedDuration_s = 2;
    seed.Length_deg          = 2;

    [candidate, diagnostics] = obstacleAvoidance.planner.createWaitThenMoveMotion(seed, initialState, goalState, limits, options, [], []);

    verifyTrue(testCase, candidate.Success, candidate.Message);
    verifyTrue(testCase, diagnostics.HorizonRetryAttempted);
    verifyNotEqual(testCase, diagnostics.InitialTimingTerminationReason, "");
    verifyEqual(testCase, candidate.ArrivalTime_s, goalState.time_s, "AbsTol", 1e-12);
end

function testObstacleFreeEarliestMotionPassesPublicValidation(testCase)
    % Require a finite rest-to-rest direct motion inside the supplied horizon.
    initialState = restState(0, [0 0]);
    goalState    = restState(20, [4 2]);
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory([], initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyTrue(testCase, result.Validation.VelocityWithinLimits);
    verifyTrue(testCase, result.Validation.AccelerationWithinLimits);
    verifyTrue(testCase, result.Validation.JerkWithinLimits);
    verifyEqual(testCase, result.TerminationReason, "goalReached");
    verifyEqual(testCase, resultDiagnosis.Attempts(1).SeedSource, "directRestToRest");
    verifyLessThan(testCase, result.TrajectoryDuration_s, goalState.time_s - initialState.time_s);
end

function testBernsteinOutlierIsNotAnExactRangeRejection(testCase)
    % Require subdivision because one control can exceed the polynomial range.
    safePower = [0; 4; -4];
    within    = obstacleAvoidance.validation.certifyPolynomialRange(safePower, -0.1, 1.1, 0);
    verifyTrue(testCase, within);
end

function testBernsteinSubdivisionFindsInteriorViolation(testCase)
    % Reject a true interior peak even though both endpoints satisfy the bounds.
    violatingPower = [0; 4.4; -4.4];
    within         = obstacleAvoidance.validation.certifyPolynomialRange(violatingPower, -0.1, 0.5, 0);
    verifyFalse(testCase, within);
end

function testStationaryFallbackAcceptsNondyadicTangent(testCase)
    % Keep exact stationary-point resolution for a boundary tangent at tau=1/3.
    tangentPower = [8 / 9; 2 / 3; -1];
    within       = obstacleAvoidance.validation.certifyPolynomialRange(tangentPower, -0.1, 1, 1e-12);
    verifyTrue(testCase, within);
end

function testSuccessAndEndpointFailureShareResultShape(testCase)
    % Preserve every public field on expected failure as well as success.
    initialState = restState(0, [-2 0]);
    goalState    = restState(10, [2 0]);
    limits       = physicalLimits();
    options      = plannerOptions("earliestArrival");
    success      = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
    blocking     = obstacleAvoidance.obstacles.createObstacle("blocked start", [0; 10], [-3 -1 -1 -3], [-1 -1 1 1], 0);
    failure      = obstacleAvoidance.planTrajectory(blocking, initialState, goalState, limits, options);

    verifyTrue(testCase, success.Success);
    verifyFalse(testCase, failure.Success);
    verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
    verifyEqual(testCase, fieldnames(failure), fieldnames(success));
    verifyEqual(testCase, fieldnames(failure.Validation), fieldnames(success.Validation));
    verifyEqual(testCase, fieldnames(failure.PlaneCertificate), fieldnames(success.PlaneCertificate));
end

function testStaticDetourReturnsCertifiedCollisionFreeMotion(testCase)
    % Exercise topology generation, BMTP, and independent static-plane replay.
    obstacle     = obstacleAvoidance.obstacles.createObstacle("center box", [0; 30], [-1 1 1 -1], [-1 -1 1 1], 0.2);
    initialState = restState(0, [-4 0]);
    goalState    = restState(30, [4 0]);
    result       = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyTrue(testCase, result.Validation.CollisionResolved);
    verifyGreaterThan(testCase, result.Validation.MinimumClearance_deg, 0);
    verifyGreaterThan(testCase, sum(vecnorm(diff(result.position_deg, 1, 1), 2, 2)), 8);
end

function testTightHorizonContinuesBeforeLaterSeedRecovery(testCase)
    % Meet the unchanged tight deadline and stop before later-seed recovery.
    missionEndTime_s     = 21;
    obstaclePosition_deg = [ ...
        -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
    obstacle     = obstacleAvoidance.obstacles.createObstacle("tight static U", [0; missionEndTime_s], obstaclePosition_deg(:, 1), obstaclePosition_deg(:, 2), 0.2);
    initialState = restState(0, [0 0]);
    goalState    = restState(missionEndTime_s, [0 -10]);
    limits       = physicalLimits();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [0.75 0.75];
    limits.maxJerk_deg_s3         = [2.5 2.5];

    twoSeedOptions = struct();
    twoSeedOptions.GoalTimeMode     = "earliestArrival";
    twoSeedOptions.MaximumSeedCount = 2;
    [twoSeedResult, twoSeedResultDiagnosis] = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, twoSeedOptions);
    verifyTrue(testCase, twoSeedResult.Success, twoSeedResult.Message);
    twoSeedValidation = obstacleAvoidance.validateTrajectory(twoSeedResult);
    verifyTrue(testCase, twoSeedValidation.Passed, twoSeedValidation.Message);
    verifyEqual(testCase, twoSeedResultDiagnosis.SelectedAttemptIndex, 2);
    diagnostics = testSupport.solverDetails(twoSeedResultDiagnosis, 2);
    verifyGreaterThan(testCase, testSupport.diagnosisValue(diagnostics, "RetainedHorizonRetryCount"), 0);
    verifyLessThanOrEqual(testCase, testSupport.diagnosisValue(diagnostics, "IterationCount"), 35);
    verifyEqual(testCase, twoSeedResultDiagnosis.AttemptedCount, 2);

    recoveryOptions = twoSeedOptions;
    recoveryOptions.MaximumSeedCount = 5;
    [recoveredResult, recoveredResultDiagnosis] = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, recoveryOptions);
    validation = obstacleAvoidance.validateTrajectory(recoveredResult);

    verifyTrue(testCase, recoveredResult.Success, recoveredResult.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyEqual(testCase, recoveredResultDiagnosis.SelectedAttemptIndex, 2);
    verifyEqual(testCase, recoveredResultDiagnosis.AttemptedCount, 2);
    verifyEqual(testCase, recoveredResult.ArrivalTime_s, twoSeedResult.ArrivalTime_s, "AbsTol", 1e-6);
    verifyLessThanOrEqual(testCase, recoveredResult.ArrivalTime_s, missionEndTime_s);
end

function testDirectWaitDoesNotFreezeHorizonStretchedMotion(testCase)
    % A larger deadline must not hide a known earlier feasible direct motion.
    vertices_deg   = [4 -1; 6 -1; 6 1; 4 1];
    firstObstacle  = obstacleAvoidance.obstacles.createObstacle("first", [3; 4.5], vertices_deg(:, 1), vertices_deg(:, 2), 0);
    secondObstacle = obstacleAvoidance.obstacles.createObstacle("second", [8; 9.5], vertices_deg(:, 1), vertices_deg(:, 2), 0);
    obstacles      = obstacleAvoidance.obstacles.combineObstacles(firstObstacle, secondObstacle);
    initialState   = restState(0, [0 0]);
    limits         = physicalLimits();
    limits.azimuthInterval_deg   = [-1 11];
    limits.elevationInterval_deg = [-0.5 0.5];
    options = plannerOptions("earliestArrival");
    for horizon_s = [12 16 24 32]
        goalState  = restState(horizon_s, [10 0]);
        result     = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);
        validation = obstacleAvoidance.validateTrajectory(result);
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, validation.Passed, validation.Message);
        verifyLessThanOrEqual(testCase, result.ArrivalTime_s, 9.5);
        if result.Success
            verifyEqual(testCase, sum(vecnorm(diff(result.position_deg), 2, 2)), 10, "AbsTol", 1e-9);
        end
    end
end

function testDirectWaitRetimingHandlesTranslationAndOffsetClock(testCase)
    % Preserve a diagonal path while shortening a horizon-stretched moving body.
    vertices_deg = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
    obstacles    = obstacleAvoidance.obstacles.createObstacle("obstacle", [3; 9; 9.5; 23], repmat({vertices_deg(:, 1)}, 4, 1), {vertices_deg(:, 2); vertices_deg(:, 2); vertices_deg(:, 2) + 8; vertices_deg(:, 2) + 8}, 0.1);
    initialState = restState(3, [-5 0]);
    goalState    = restState(23, [5 2]);
    limits       = struct();
    limits.maxVelocity_deg_s      = [2 1];
    limits.maxAcceleration_deg_s2 = [1 0.5];
    limits.maxJerk_deg_s3         = [2 1];
    limits.azimuthInterval_deg    = [-6 6];
    limits.elevationInterval_deg  = [-0.5 2.5];
    result     = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, struct("GoalTimeMode", "earliestArrival"));
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    if result.Success
        verifyLessThanOrEqual(testCase, result.ArrivalTime_s, 14.5);
        verifyEqual(testCase, sum(vecnorm(diff(result.position_deg), 2, 2)), norm(goalState.position_deg - initialState.position_deg), "AbsTol", 1e-9);
    end
end

function testEarliestArrivalRefinesObjectiveRelevantDirectWait(testCase)
    % Shorten a validated direct wait without changing its spatial path.
    obstacleTime_s             = [0; 6; 6.5; 12];
    barrierCenterElevation_deg = [0; 0; 8; 8];
    sourcePosition_deg         = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
    azimuthBySlice_deg         = cell(numel(obstacleTime_s), 1);
    elevationBySlice_deg       = cell(numel(obstacleTime_s), 1);
    for sampleIndex = 1:numel(obstacleTime_s)
        translatedPosition_deg = sourcePosition_deg + [0 barrierCenterElevation_deg(sampleIndex)];
        azimuthBySlice_deg{sampleIndex} = translatedPosition_deg(:, 1);
        elevationBySlice_deg{sampleIndex} = translatedPosition_deg(:, 2);
    end
    obstacle     = obstacleAvoidance.obstacles.createObstacle("earliest direct wait", obstacleTime_s, azimuthBySlice_deg, elevationBySlice_deg, 0.1);
    initialState = restState(0, [-5 0]);
    goalState    = restState(12, [5 0]);
    limits       = physicalLimits();
    limits.azimuthInterval_deg   = [-6 6];
    limits.elevationInterval_deg = [-3 3];
    options = struct();
    options.GoalTimeMode     = "earliestArrival";
    options.MaximumSeedCount = 5;
    options.SampleTime_s     = 0.05;

    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, options);
    directWaitIndex = find(string({resultDiagnosis.Routes.Source}) == "directWait", 1);
    verifyTrue(testCase, result.Success, result.Message);
    verifyNotEmpty(testCase, directWaitIndex);
    diagnostics = testSupport.solverDetails(resultDiagnosis, directWaitIndex);
    validation  = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyGreaterThan(testCase, testSupport.diagnosisValue(diagnostics, "RefinementCount"), 0);
    verifyLessThan(testCase, testSupport.diagnosisValue(diagnostics, "FinalWaitTime_s"), testSupport.diagnosisValue(diagnostics, "InitialWaitTime_s"));
end

function testConvergenceSummaryAndRetainedBestTrial(testCase)
    % Preserve convergence evidence and the best validated trajectory trial.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(repositoryRoot, "examples"));
    overrides = struct();
    overrides.CollisionClearanceTolerance_deg = 1e-4;
    overrides.MaximumSeedCount                = 2;
    overrides.FigureVisible                   = "off";
    overrides.PlotOutputs                     = false;
    overrides.ShowAnimation                   = false;
    overrides.ShowKinematicPlot               = false;
    [result, resultDiagnosis] = exampleTargetExitsObstacle(overrides);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    diagnostics = testSupport.solverDetails(resultDiagnosis, resultDiagnosis.SelectedAttemptIndex);
    verifyFalse(testCase, any(startsWith(diagnostics.Field, "TravelRefinement")));
    verifyTrue(testCase, testSupport.diagnosisValue(diagnostics, "PlaneReuseApplied"));
    verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "PlaneReuseCount"), 1);
    verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "ConicSolver.Solver"), 'coneprog');
    verifyGreaterThan(testCase, testSupport.diagnosisValue(diagnostics, "ConicSolver.CallCount"), 0);
    verifyTrue(testCase, testSupport.diagnosisValue(diagnostics, "Converged"));
    verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "SolverMessage"), "The next trajectory SOCP would be unchanged.");
    collisionPairs = testSupport.diagnosisValue(diagnostics, "CollisionPairCountHistory");
    verifyGreaterThan(testCase, collisionPairs(1), 0);
    trialDurations_s      = testSupport.diagnosisValue(diagnostics, "TrialDuration_s");
    collisionFreeTrials_s = trialDurations_s(testSupport.diagnosisValue(diagnostics, "TrialWasCollisionFree"));
    verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "RetainedBestTrialDuration_s"), min(collisionFreeTrials_s), "AbsTol", 1e-12);
end

function testReflectedProgressAxisKeepsShortExactClockDetour(testCase)
    % Preserve a short valid detour when the clock-owning axis runs backward.
    obstacleTime_s = [0; 30];
    firstObstacle  = obstacleAvoidance.obstacles.createObstacle("first floating barrier", obstacleTime_s, [4.4 5.6 5.6 4.4], [-0.45 -0.45 0.45 0.45], 0.1);
    secondObstacle = obstacleAvoidance.obstacles.createObstacle("second floating barrier", obstacleTime_s, [-5.6 -4.4 -4.4 -5.6], [-0.45 -0.45 0.45 0.45], 0.1);
    obstacles      = obstacleAvoidance.obstacles.combineObstacles(firstObstacle, secondObstacle);
    initialState   = restState(0, [10 0]);
    goalState      = restState(30, [-10 0]);
    limits         = physicalLimits();
    limits.azimuthInterval_deg   = [-12 12];
    limits.elevationInterval_deg = [-4 4];
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, plannerOptions("earliestArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyTrue(testCase, result.Validation.CollisionResolved);
    diagnostics = resultDiagnosis.PathRefinement;
    verifyTrue(testCase, testSupport.diagnosisValue(diagnostics, "Success"), testSupport.diagnosisValue(diagnostics, "Message"));
    verifyTrue(testCase, testSupport.diagnosisValue(diagnostics, "TravelRefinement.Attempted"));
    verifyLessThanOrEqual(testCase, testSupport.diagnosisValue(diagnostics, "TravelRefinement.FinalLength_deg"), testSupport.diagnosisValue(diagnostics, "TravelRefinement.InitialLength_deg"));
    verifyEqual(testCase, result.TrajectoryDuration_s, 12.5, "AbsTol", 1e-9);
    verifyEqual(testCase, result.Route_deg, result.position_deg);
    verifyEqual(testCase, resultDiagnosis.Routes(1).Length_deg, sum(vecnorm(diff(result.position_deg), 2, 2)), "AbsTol", 1e-12);
    % The refined detour must remain within 1.25 percent of the direct distance.
    directLength_deg = norm(goalState.position_deg - initialState.position_deg);
    verifyLessThan(testCase, resultDiagnosis.Routes(1).Length_deg, 1.0125 * directLength_deg, "The exact-clock detour contains unnecessary joint travel.");
end

function testNearStartBarrierKeepsShortOneSidedExactClockDetour(testCase)
    % A local static obstruction must retain a short, one-sided detour.
    obstacle     = obstacleAvoidance.obstacles.createObstacle("offset rectangle", [0; 30], [-9 -6 -6 -9], [0 0 2.8 2.8], 0.1);
    initialState = restState(0, [-10 3.2]);
    goalState    = restState(30, [10 -1]);
    limits       = physicalLimits();
    limits.azimuthInterval_deg   = [-20 20];
    limits.elevationInterval_deg = [-10 10];
    result = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, plannerOptions("earliestArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyTrue(testCase, result.Validation.CollisionResolved);
    verifyEqual(testCase, result.TrajectoryDuration_s, 12.5, "AbsTol", 1e-9);
    verifyLessThan(testCase, sum(vecnorm(diff(result.position_deg), 2, 2)), 20.55, "The exact-clock detour contains unnecessary joint travel.");
    progress            = (result.position_deg(:, 1) - initialState.position_deg(1)) / (goalState.position_deg(1) - initialState.position_deg(1));
    directElevation_deg = initialState.position_deg(2) + (goalState.position_deg(2) - initialState.position_deg(2)) * progress;
    verifyGreaterThanOrEqual(testCase, min(result.position_deg(:, 2) - directElevation_deg), -1e-9);
end

function testEarliestDefaultReportsUtilization(testCase)
    % Report measured envelope use on success.
    initialState = restState(0, [0 0]);
    goalState    = restState(20, [4 2]);
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory([], initialState, goalState, physicalLimits());

    verifyTrue(testCase, result.Success, result.Message);
    summary = resultDiagnosis.Attempts(resultDiagnosis.SelectedAttemptIndex);
    verifyEqual(testCase, result.Options.GoalTimeMode, "earliestArrival");
    verifyGreaterThan(testCase, summary.KinematicUtilization, 0);
    verifyLessThanOrEqual(testCase, summary.KinematicUtilization, 1 + 1e-6);
    verifyEqual(testCase, resultDiagnosis.Selection.JerkRole, "hardConstraintOnly");
end

function testRankingUsesOnlyDeclaredObjectiveQuantities(testCase)
    % Prefer shorter equal-arrival motions without a hidden utilization policy.
    summary = repmat(struct("MotionLength_deg", 10, "KinematicUtilization", 1, "ArrivalTime_s", 5, "ValidationPassed", true), 2, 1);
    summary(2).MotionLength_deg = 9;
    summary(2).KinematicUtilization = 0.1;
    for mode = ["fixedArrival", "earliestArrival"]
        selection = obstacleAvoidance.planner.selectValidatedCandidate(summary, struct("GoalTimeMode", mode));
        ranking   = selection.Ranking;
        verifyEqual(testCase, ranking.OrderedCandidateIndices(1), 2);
        verifyFalse(testCase, any(contains(ranking.ColumnNames, "KinematicUtilization")));
    end
    summary(2).ArrivalTime_s = 6;
    selection = obstacleAvoidance.planner.selectValidatedCandidate(summary, struct("GoalTimeMode", "earliestArrival"));
    verifyEqual(testCase, selection.Ranking.OrderedCandidateIndices(1), 1);
end

function testFixedArrivalBelowPhysicalMinimumReturnsFailure(testCase)
    % An infeasible clock is an expected result rather than a thrown error.
    initialState = restState(0, [0 0]);
    goalState    = restState(0.25, [1 0]);
    limits       = physicalLimits();
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, plannerOptions("fixedArrival"));

    verifyFalse(testCase, result.Success);
    verifyNotEmpty(testCase, result.Message);
    verifyTrue(testCase, any(result.TerminationReason == ["noValidatedSeed", "fixedArrivalInfeasible", "timeWindowInfeasible"]));
    verifyEmpty(testCase, result.time_s);
    verifyTrue(testCase, isfield(resultDiagnosis, "Timing"));
end

function testUnsupportedEndpointDerivativesThrowNamedError(testCase)
    % The compact BMTP scope rejects non-rest endpoints explicitly.
    initialState = restState(0, [0 0]);
    initialState.velocity_deg_s = [0.1 0];
    goalState = restState(10, [2 0]);
    request   = @() obstacleAvoidance.planTrajectory([], initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));
    verifyError(testCase, request, "bmtpEngine:UnsupportedRequest");
end

function testNoPathReturnsRecognizedDiagnostics(testCase)
    % A full-height wall must fail with retained search evidence.
    wall         = obstacleAvoidance.obstacles.createObstacle("full-height wall", [0; 20], [-0.5 0.5 0.5 -0.5], [-90 -90 90 90], 0);
    initialState = restState(0, [-5 0]);
    goalState    = restState(12, [5 0]);
    limits       = physicalLimits();
    limits.elevationInterval_deg = [-10 10];
    options = plannerOptions("earliestArrival");
    options.MaximumSeedCount = 3;
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory(wall, initialState, goalState, limits, options);

    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
    verifyEmpty(testCase, result.time_s);
    verifyTrue(testCase, isfield(resultDiagnosis.Search, "ExpandedCount"));
    verifyGreaterThanOrEqual(testCase, resultDiagnosis.AttemptedCount, 1);
end

function testRepeatedDirectRequestIsDeterministic(testCase)
    % Identical input must reproduce every selected motion sample exactly.
    initialState = restState(0, [-1 0.5]);
    goalState    = restState(12, [3 -1]);
    options      = plannerOptions("earliestArrival");
    first        = obstacleAvoidance.planTrajectory([], initialState, goalState, physicalLimits(), options);
    second       = obstacleAvoidance.planTrajectory([], initialState, goalState, physicalLimits(), options);

    verifyEqual(testCase, second.TerminationReason, first.TerminationReason);
    verifyEqual(testCase, second.time_s, first.time_s);
    verifyEqual(testCase, second.position_deg, first.position_deg);
    verifyEqual(testCase, second.velocity_deg_s, first.velocity_deg_s);
    verifyEqual(testCase, second.acceleration_deg_s2, first.acceleration_deg_s2);
    verifyEqual(testCase, second.jerk_deg_s3, first.jerk_deg_s3);
end

function testInterceptValidatesInitialStateBeforeFieldAccess(testCase)
    % Use the shared identified state error for malformed public input.
    targetMotion = struct();
    targetMotion.time_s       = [0; 1];
    targetMotion.position_deg = [0 0; 1 0];
    verifyError(testCase, @() obstacleAvoidance.planMovingTargetIntercept(struct(), targetMotion, physicalLimits(), struct()), "planTrajectory:InvalidState");
end

function testInterceptEchoesTheFixedArrivalModeItUses(testCase)
    % Intercept trials select one terminal time, regardless of the caller mode.
    targetMotion = struct();
    targetMotion.time_s       = [0; 5];
    targetMotion.position_deg = [0 0; 4 0];
    interceptOptions = struct("InterceptMode", "specifiedTime", ...
        "SpecifiedInterceptTime_s", 5, ...
        "PlannerOptions", plannerOptions("earliestArrival"));
    [result, resultDiagnosis] = obstacleAvoidance.planMovingTargetIntercept(restState(0, [0 0]), targetMotion, physicalLimits(), interceptOptions);

    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.Options.GoalTimeMode, "fixedArrival");
    verifyEqual(testCase, result.Options.GoalTimeMode, "fixedArrival");
    verifyEqual(testCase, testSupport.diagnosisValue(resultDiagnosis.InterceptSearch, "OptimalityStatus"), "notAnOptimization");

    % Exercise the bounded time-grid search with BMTP and a PCHIP target.
    movingInitialState = restState(0, [0 0]);
    movingTarget       = struct("time_s", (0:0.5:12).', ...
        "position_deg", repmat([6 1], 25, 1), ...
        "InterpolationMethod", "pchip");
    asapOptions = struct();
    asapOptions.InterceptMode           = "earliest";
    asapOptions.MaximumSearchDuration_s = 12;
    asapOptions.PlannerOptions          = struct();
    [asapResult, asapResultDiagnosis] = obstacleAvoidance.planMovingTargetIntercept(movingInitialState, movingTarget, physicalLimits(), asapOptions);
    verifyTrue(testCase, asapResult.Success, asapResult.Message);
    verifyTrue(testCase, asapResult.Validation.Passed, asapResult.Validation.Message);
    verifyEqual(testCase, asapResult.Intercept.Mode, "earliest");
    verifyEqual(testCase, testSupport.diagnosisValue(asapResultDiagnosis.InterceptSearch, "OptimalityStatus"), "resolutionBounded");
    verifyEqual(testCase, testSupport.diagnosisValue(asapResultDiagnosis.InterceptSearch, "MaximumCoarseStep_s"), 0.5, "AbsTol", 1e-12);

    atTimeOptions = asapOptions;
    atTimeOptions.InterceptMode            = "specifiedTime";
    atTimeOptions.SpecifiedInterceptTime_s = 10;
    atTimeResult = obstacleAvoidance.planMovingTargetIntercept(movingInitialState, movingTarget, physicalLimits(), atTimeOptions);
    verifyTrue(testCase, atTimeResult.Success, atTimeResult.Message);
    verifyTrue(testCase, atTimeResult.Validation.Passed, atTimeResult.Validation.Message);
    verifyEqual(testCase, atTimeResult.Intercept.Mode, "specifiedTime");
    verifyEqual(testCase, atTimeResult.Intercept.Time_s, 10, "AbsTol", 1e-12);
    verifyLessThan(testCase, asapResult.Intercept.Time_s, atTimeResult.Intercept.Time_s);

    certifiedOptions = struct();
    certifiedOptions.InterceptMode           = "earliest";
    certifiedOptions.MaximumSearchDuration_s = 12;
    certifiedOptions.PlannerOptions          = struct();
    certifiedTarget = movingTarget;
    certifiedTarget.InterpolationMethod = "linear";
    [certifiedResult, certifiedResultDiagnosis] = obstacleAvoidance.planMovingTargetIntercept(restState(0, [0 0]), certifiedTarget, physicalLimits(), certifiedOptions);
    verifyTrue(testCase, certifiedResult.Success, certifiedResult.Message);
    verifyEqual(testCase, testSupport.diagnosisValue(certifiedResultDiagnosis.InterceptSearch, "OptimalityStatus"), "certifiedEarliest");
end

function state = restState(time_s, position_deg)
    % Create one two-axis rest endpoint.
    state = struct("time_s", time_s, "position_deg", position_deg, ...
        "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function limits = physicalLimits()
    % Create neutral two-axis bounds used by the public-contract cases.
    limits = struct();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [1 1];
    limits.maxJerk_deg_s3         = [2 2];
    limits.azimuthInterval_deg    = [-180 180];
    limits.elevationInterval_deg  = [-90 90];
end

function options = plannerOptions(goalTimeMode)
    % Resolve only behavior-level public choices for deterministic tests.
    options = struct("GoalTimeMode", goalTimeMode, ...
        "SampleTime_s", 0.05);
end
