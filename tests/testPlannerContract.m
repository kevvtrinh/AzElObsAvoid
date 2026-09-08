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
%   - Position is coordinate units; time is seconds; derivatives use units/s powers.
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
    movingStart_units = [4 -1; 6 -1; 6 1; 4 1];
    movingEnd_units   = movingStart_units + [1 0];
    movingObstacle  = obstacleAvoidance.obstacles.createObstacle("moving", obstacleTime_s, {movingStart_units(:, 1); movingEnd_units(:, 1)}, {movingStart_units(:, 2); movingEnd_units(:, 2)}, 0.1);
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

    proposal = obstacleAvoidance.search.createRouteSearchGeometry(request.initialState, request.goalState, scene);
    verifyEqual(testCase, proposal.start_units, [-4 0]);
    verifyEqual(testCase, proposal.goal_units, [8 0]);
    verifyEqual(testCase, proposal.sampleTimes_s, linspace(0, 10, 9).');
    maximumVerticesPerObstacle = zeros(1, numel(scene.preparedObstacles));
    % Exercise each obstacle covered by this regression.
    for obstacleIndex = 1:numel(scene.preparedObstacles)
        maximumVerticesPerObstacle(obstacleIndex) = max(cellfun(@numel, scene.preparedObstacles(obstacleIndex).x_units));
    end
    expectedVertexWork = numel(proposal.sampleTimes_s) * sum(maximumVerticesPerObstacle);
    verifyEqual(testCase, proposal.estimatedVertexWork, expectedVertexWork);
    verifyEqual(testCase, proposal.representation, "sampledObstacleUnion");
    verifyFalse(testCase, proposal.usedDenseEnvelope);
    verifyEqual(testCase, proposal.sampledShapeCount, 18);
    verifyEqual(testCase, size(proposal.edgeStart_units), size(proposal.edgeEnd_units));

    visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(request.limits, proposal);
    verifyGreaterThanOrEqual(testCase, visibilityGraph.FinalAttemptIndex, 1);
    verifyEqual(testCase, numel(visibilityGraph.Attempts), visibilityGraph.FinalAttemptIndex);
    verifyEqual(testCase, visibilityGraph.NodePosition_units, visibilityGraph.Attempts(end).Nodes.Positions_units);
    verifyEqual(testCase, visibilityGraph.EdgeCost_units, visibilityGraph.Attempts(end).Cost_units);
    verifyEqual(testCase, visibilityGraph.Attempts(end).OffsetRetryCount, visibilityGraph.Record.CandidateOffsetRetryCount);
    verifyTrue(testCase, isfield(visibilityGraph.Attempts, "FinalCandidatePairs"));
    verifyTrue(testCase, isfield(visibilityGraph.Attempts, "EdgeRejectionReasons"));
    verifyTrue(testCase, isfield(visibilityGraph.Attempts, "RecoverySteps"));

    routeSet = obstacleAvoidance.search.searchRoutes(request.initialState, request.goalState, request.limits, request.options, scene, proposal, visibilityGraph);
    seedSet  = obstacleAvoidance.search.createPathGuesses(request.initialState, request.goalState, request.limits, routeSet, proposal.shape.Vertices);
    verifyEqual(testCase, seedSet(1).Source, "directPathGuess");
    verifyEqual(testCase, [seedSet.Index], 1:numel(seedSet));
    verifyTrue(testCase, isfield(routeSet, "TimedSearchRecord"));
    verifyTrue(testCase, isfield(routeSet, "SpatialSearchRecord"));
    verifyTrue(testCase, isfield(routeSet, "RouteClassPattern"));
end

function testSpatialSeedEstimateNeverRejectsLongGuideRoute(testCase)
    % Keep route-shaped timing guesses from pruning a solver proposal.
    limits = physicalLimits();
    limits.maxVelocity_units_s = [2 2];
    request = struct("obstacles", obstacleAvoidance.obstacles.combineObstacles([]), ...
        "initialState", restState(0, [0 0]), ...
        "goalState", restState(1, [1 0]), ...
        "limits", limits, ...
        "options", obstacleAvoidance.input.resolvePlannerOptions(plannerOptions("earliestArrival")));
    spatialRoute_units = [0 0; 0 8; 1 8; 1 0];
    routeSet         = struct("TimedRoute_units", zeros(0, 2), ...
        "TimedRouteTime_s", zeros(0, 1), ...
        "SpatialRoutes_units", {{spatialRoute_units}}, ...
        "UsesConservativeEnvelope", false);
    proposal = struct("shape", polyshape());

    seedSet = obstacleAvoidance.search.createPathGuesses(request.initialState, request.goalState, request.limits, routeSet, proposal.shape.Vertices);

    verifyEqual(testCase, numel(seedSet), 2);
    verifyEqual(testCase, seedSet(2).position_units, spatialRoute_units);
    verifyEqual(testCase, seedSet(2).EstimatedDuration_s, 0.5, "AbsTol", 1e-12);
    verifyGreaterThan(testCase, seedSet(2).Length_units, request.goalState.time_s - request.initialState.time_s);
end

function testSpatialSearchAllowsMultipleWindingAndEndsWhenDisconnected(testCase)
    % Remove arbitrary winding rejection without losing finite disconnected exit.
    angle_rad      = linspace(0, 4 * pi, 25).';
    radius_units     = 1 + 0.1 * angle_rad;
    spiralPath_units = [radius_units .* cos(angle_rad), ...
        radius_units .* sin(angle_rad)];
    nodePosition_units = [spiralPath_units(1, :); spiralPath_units(end, :); ...
        spiralPath_units(2:end - 1, :)];
    pathNodeIndex = [1, 3:size(nodePosition_units, 1), 2];
    edgeCost_units  = Inf(size(nodePosition_units, 1));
    % Exercise each edge covered by this regression.
    for edgeIndex = 1:numel(pathNodeIndex) - 1
        firstNode      = pathNodeIndex(edgeIndex);
        secondNode     = pathNodeIndex(edgeIndex + 1);
        edgeLength_units = norm(nodePosition_units(firstNode, :) - nodePosition_units(secondNode, :));
        edgeCost_units(firstNode, secondNode) = edgeLength_units;
        edgeCost_units(secondNode, firstNode) = edgeLength_units;
    end
    [routes_units, classPattern, ~] = obstacleAvoidance.search.searchDistinctSpatialRoutes(edgeCost_units, nodePosition_units, [0 0], 1, @(first_units, second_units) true);
    verifyNumElements(testCase, routes_units, 1);
    verifyEqual(testCase, abs(classPattern), 2);

    request = struct("obstacles", obstacleAvoidance.obstacles.combineObstacles([]), ...
        "initialState", restState(0, nodePosition_units(1, :)), ...
        "goalState", restState(20, nodePosition_units(2, :)), ...
        "limits", physicalLimits(), ...
        "options", obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival", "MaximumSeedCount", 2)));
    scene    = obstacleAvoidance.obstacles.preparePlanningScene(request.obstacles, request.initialState, request.goalState);
    proposal = struct("usedDenseEnvelope", false, "sampleTimes_s", [0; 20], ...
        "goal_units", nodePosition_units(2, :), "shape", polyshape(), ...
        "edgeStart_units", zeros(0, 2), "edgeEnd_units", zeros(0, 2));
    visibilityGraph = struct("NodePosition_units", nodePosition_units, "EdgeCost_units", edgeCost_units, ...
        "ObstacleReferencePoints_units", [0 0]);
    routeSet = obstacleAvoidance.search.searchRoutes(request.initialState, request.goalState, request.limits, request.options, scene, proposal, visibilityGraph);
    verifyEmpty(testCase, routeSet.SpatialRoutes_units);
    verifyNumElements(testCase, routeSet.DeferredSpatialRoutes_units, 1);
    verifyFalse(testCase, routeSet.DeferredSpatialSolveAttempted);

    nodePosition_units = [1 0; 4 0; 0 1; -1 0; 0 -1];
    edgeCost_units     = Inf(5);
    cycleNodeIndex   = [1 3 4 5 1];
    % Exercise each edge covered by this regression.
    for edgeIndex = 1:numel(cycleNodeIndex) - 1
        firstNode  = cycleNodeIndex(edgeIndex);
        secondNode = cycleNodeIndex(edgeIndex + 1);
        edgeCost_units(firstNode, secondNode) = 1;
        edgeCost_units(secondNode, firstNode) = 1;
    end
    [routes_units, ~, record] = obstacleAvoidance.search.searchDistinctSpatialRoutes(edgeCost_units, nodePosition_units, [0 0], 2, @(first_units, second_units) true);
    verifyEmpty(testCase, routes_units);
    verifyEqual(testCase, record.StateCount, 1);
    verifyFalse(testCase, record.Truncated);
end

function testDenseProposalDefersThenRunsExactTimedSearch(testCase)
    % Dense spatial work may reorder exact timed search but cannot discard it.
    obstacleTime_s     = [0; 10];
    firstVertices_units  = [20 -2; 22 -2; 22 2; 20 2];
    secondVertices_units = firstVertices_units + [0 1];
    movingObstacle     = obstacleAvoidance.obstacles.createObstacle("moving", obstacleTime_s, {firstVertices_units(:, 1); secondVertices_units(:, 1)}, {firstVertices_units(:, 2); secondVertices_units(:, 2)}, 0.1);
    request            = struct("obstacles", movingObstacle, ...
        "initialState", restState(0, [0 0]), ...
        "goalState", restState(10, [2 0]), ...
        "limits", physicalLimits(), ...
        "options", obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "fixedArrival", "MaximumSeedCount", 1, ...
            "MaximumTimeLayerCount", 3)));
    scene    = obstacleAvoidance.obstacles.preparePlanningScene(request.obstacles, request.initialState, request.goalState);
    proposal = struct("usedDenseEnvelope", true, "sampleTimes_s", [0; 5; 10], ...
        "goal_units", [2 0], "shape", polyshape(), ...
        "edgeStart_units", zeros(0, 2), "edgeEnd_units", zeros(0, 2));
    visibilityGraph = struct();
    visibilityGraph.NodePosition_units            = [0 0; 2 0];
    visibilityGraph.EdgeCost_units                = [0 2; 2 0];
    visibilityGraph.ObstacleReferencePoints_units = zeros(0, 2);

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
    verifyFalse(testCase, isempty(recoveredRouteSet.TimedRoute_units));
    verifyEqual(testCase, recoveredRouteSet.TimedRoute_units(1, :), [0 0]);
    verifyEqual(testCase, recoveredRouteSet.TimedRoute_units(end, :), [2 0]);
    verifyEqual(testCase, recoveredRouteSet.SpatialSearchRecord.RecoverySentinel, 314);
end

function testTimedSearchDoesNotImposeRestAtIntermediateNodes(testCase)
    % Preserve a feasible constant-velocity route through an intermediate node.
    nodePosition_units = [0 0; 4 0; 2 0];
    edgeCost_units     = Inf(3);
    edgeCost_units(1:4:end) = 0;
    edgeCost_units(1, 3) = 2;
    edgeCost_units(3, 1) = 2;
    edgeCost_units(2, 3) = 2;
    edgeCost_units(3, 2) = 2;
    initialState = restState(0, nodePosition_units(1, :));
    initialState.velocity_units_s = [2 0];
    goalState = restState(2, nodePosition_units(2, :));
    goalState.velocity_units_s = [2 0];
    limits = physicalLimits();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [0.01 0.01];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival", "MaximumTimeLayerCount", 3));

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, edgeCost_units, obstacleAvoidance.obstacles.prepareObstacles(struct.empty(0, 1)), initialState, goalState, limits, [0; 1; 2], options);

    verifyEqual(testCase, route_units, nodePosition_units([1 3 2], :));
    verifyEqual(testCase, routeTime_s, [0; 1; 2]);
end

function testTimedSearchPreservesCostTiesAndCompleteFrontier(testCase)
    % Keep optimal through-node ancestry when more expensive cycles are pruned.
    nodePosition_units = [0 0; 4 0; 2 0; 2 2];
    edgeCost_units     = zeros(4);
    initialState     = restState(0, nodePosition_units(1, :));
    goalState        = restState(5, nodePosition_units(2, :));
    options          = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "fixedArrival"));
    sampleTimes_s    = (0:5).';

    [route_units, routeTime_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, edgeCost_units, obstacleAvoidance.obstacles.prepareObstacles(struct.empty(0, 1)), initialState, goalState, physicalLimits(), sampleTimes_s, options);

    verifyEqual(testCase, route_units, nodePosition_units([1 3 3 3 3 2], :));
    verifyEqual(testCase, routeTime_s, sampleTimes_s);
    verifyEqual(testCase, sum(vecnorm(diff(route_units), 2, 2)), 4);
    verifyEqual(testCase, record.FrontierNodes_units, nodePosition_units);
    verifyEqual(testCase, record.ReachableGoalLayerCount, 4);
end

function testTimedSearchPreservesFutureFrontierAtEarlyExit(testCase)
    % An early goal exit still exposes later states discovered by colliding edges.
    nodePosition_units = [0 0; 2 0; 0 1; 1 1];
    blocked_units      = [0.4 0.9; 0.6 0.9; 0.6 1.1; 0.4 1.1];
    clear_units        = blocked_units + [0 3];
    obstacles        = obstacleAvoidance.obstacles.createObstacle("moving barrier", [0; 1; 1.5; 3], {blocked_units(:, 1); blocked_units(:, 1); clear_units(:, 1); clear_units(:, 1)}, {blocked_units(:, 2); blocked_units(:, 2); clear_units(:, 2); clear_units(:, 2)}, 0);
    options          = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));

    [route_units, routeTime_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, zeros(4), obstacleAvoidance.obstacles.prepareObstacles(obstacles), restState(0, nodePosition_units(1, :)), restState(3, nodePosition_units(2, :)), physicalLimits(), (0:0.5:3).', options);

    verifyEqual(testCase, route_units, nodePosition_units(1:2, :));
    verifyEqual(testCase, routeTime_s, [0; 1]);
    verifyEqual(testCase, record.FrontierNodes_units, nodePosition_units(3:4, :));
end

function testTimedSearchRetainsEverySuppliedTime(testCase)
    % Keep a brief input-defined opening that layer thinning would erase.
    blockedVertices_units = [-0.15 -2; 0.15 -2; 0.15 2; -0.15 2];
    clearVertices_units   = blockedVertices_units + [0 30];
    obstacleTime_s      = [0; 3.8; 3.9; 4.2; 4.3; 5];
    barrier             = obstacleAvoidance.obstacles.createObstacle("briefOpening", obstacleTime_s, {blockedVertices_units(:, 1); blockedVertices_units(:, 1); clearVertices_units(:, 1); clearVertices_units(:, 1); blockedVertices_units(:, 1); blockedVertices_units(:, 1)}, {blockedVertices_units(:, 2); blockedVertices_units(:, 2); clearVertices_units(:, 2); clearVertices_units(:, 2); blockedVertices_units(:, 2); blockedVertices_units(:, 2)}, 0);
    nodePosition_units    = [-2 0; 2 0; 0 0];
    edgeCost_units        = Inf(3);
    edgeCost_units(1, 3) = 2;
    edgeCost_units(3, 1) = 2;
    edgeCost_units(2, 3) = 2;
    edgeCost_units(3, 2) = 2;
    initialState = restState(0, nodePosition_units(1, :));
    goalState    = restState(5, nodePosition_units(2, :));
    limits       = physicalLimits();
    limits.maxVelocity_units_s = [20 20];
    options       = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival", "MaximumTimeLayerCount", 17));
    sampleTimes_s = (0:0.1:5).';

    [route_units, routeTime_s, record] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, edgeCost_units, obstacleAvoidance.obstacles.prepareObstacles(barrier), initialState, goalState, limits, sampleTimes_s, options);

    verifyEqual(testCase, record.LayerTimes_s, sampleTimes_s, "AbsTol", eps(5));
    verifyEqual(testCase, record.CandidateLayerCount, numel(sampleTimes_s));
    verifyGreaterThan(testCase, record.CandidateLayerCount, options.MaximumTimeLayerCount);
    verifyFalse(testCase, isfield(record, "LayerLimitApplied"));
    verifyFalse(testCase, isempty(route_units));
    verifyEqual(testCase, route_units(1, :), initialState.position_units);
    verifyEqual(testCase, route_units(end, :), goalState.position_units);
    verifyEqual(testCase, routeTime_s(end), 4, "AbsTol", 1e-12);
end

function testTimedSearchCanArriveLaterWithoutWaitingAtBlockedSource(testCase)
    % Advance within one safe-wait interval after an earlier edge collision.
    square_units          = [-0.1 -0.1; 0.1 -0.1; 0.1 0.1; -0.1 0.1];
    obstacleTime_s      = [0; 0.25; 0.5; 1; 2];
    xAtSource_units = repmat({square_units(:, 1) - 2}, 5, 1);
    xOnEdge_units   = repmat({square_units(:, 1)}, 5, 1);
    sourceY_units = { ...
        square_units(:, 2) + 3; square_units(:, 2); square_units(:, 2); square_units(:, 2) + 3; square_units(:, 2) + 3};
    edgeY_units = { ...
        square_units(:, 2) + 3; square_units(:, 2); square_units(:, 2) + 3; square_units(:, 2) + 3; square_units(:, 2) + 3};
    sourceBlocker    = obstacleAvoidance.obstacles.createObstacle("sourceBlocker", obstacleTime_s, xAtSource_units, sourceY_units, 0);
    edgeBlocker      = obstacleAvoidance.obstacles.createObstacle("edgeBlocker", obstacleTime_s, xOnEdge_units, edgeY_units, 0);
    obstacles        = obstacleAvoidance.obstacles.combineObstacles(sourceBlocker, edgeBlocker);
    nodePosition_units = [-2 0; 2 0];
    edgeCost_units     = [0 4; 4 0];
    initialState     = restState(0, nodePosition_units(1, :));
    initialState.velocity_units_s = [4 0];
    goalState = restState(2, nodePosition_units(2, :));
    goalState.velocity_units_s = [4 0];
    limits = physicalLimits();
    limits.maxVelocity_units_s = [10 10];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));

    [route_units, routeTime_s] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, edgeCost_units, obstacleAvoidance.obstacles.prepareObstacles(obstacles), initialState, goalState, limits, obstacleTime_s, options);

    fineTime_s       = linspace(0, 1, 101).';
    finePosition_units = initialState.position_units + fineTime_s .* (goalState.position_units - initialState.position_units);
    occupied         = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles, finePosition_units(:, 1), finePosition_units(:, 2), fineTime_s);
    verifyFalse(testCase, any(occupied));
    verifyEqual(testCase, route_units, nodePosition_units);
    verifyEqual(testCase, routeTime_s, [0; 1]);
end

function testDenseMovingBarrierRecoversDeferredTimedSeed(testCase)
    % Recover a wait route after dense spatial proposals yield no valid motion.
    angle_rad           = (0:1199).' * (2 * pi / 1200);
    blockedVertices_units = [0.15 * cos(angle_rad), 10 * sin(angle_rad)];
    clearVertices_units   = blockedVertices_units + [0 30];
    obstacleTime_s      = [0; 6; 7; 10];
    barrier             = obstacleAvoidance.obstacles.createObstacle("movingDenseBarrier", obstacleTime_s, {blockedVertices_units(:, 1); blockedVertices_units(:, 1); clearVertices_units(:, 1); clearVertices_units(:, 1)}, {blockedVertices_units(:, 2); blockedVertices_units(:, 2); clearVertices_units(:, 2); clearVertices_units(:, 2)}, 0.05);
    limits              = physicalLimits();
    limits.maxVelocity_units_s      = [2 0.5];
    limits.maxAcceleration_units_s2 = [4 1];
    limits.maxJerk_units_s3         = [20 4];
    limits.xInterval_units    = [-3 3];
    limits.yInterval_units  = [-12 12];
    options = struct();
    options.GoalTimeMode          = "fixedArrival";
    options.MaximumSeedCount      = 2;
    options.MaximumTimeLayerCount = 17;
    options.SampleTime_s          = 0.1;

    [result, resultDiagnosis] = planner(barrier, restState(0, [-2 0]), restState(10, [2 0]), limits, options);

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
    limits.maxAcceleration_units_s2 = [0.1 0.1];
    limits.maxJerk_units_s3         = [0.1 0.1];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));
    seed    = obstacleAvoidance.search.createEmptyPathGuess();
    seed.Index               = 1;
    seed.Source              = "directWait";
    seed.position_units        = [0 0; 0 0; 2 0];
    seed.tau                 = [0; 0.5; 1];
    seed.EstimatedDuration_s = 2;
    seed.Length_units          = 2;

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
    [result, resultDiagnosis] = planner([], initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));

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
    success      = planner([], initialState, goalState, limits, options);
    blocking     = obstacleAvoidance.obstacles.createObstacle("blocked start", [0; 10], [-3 -1 -1 -3], [-1 -1 1 1], 0);
    failure      = planner(blocking, initialState, goalState, limits, options);

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
    result       = planner(obstacle, initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyTrue(testCase, result.Validation.CollisionResolved);
    verifyGreaterThan(testCase, result.Validation.MinimumClearance_units, 0);
    verifyGreaterThan(testCase, sum(vecnorm(diff(result.position_units, 1, 1), 2, 2)), 8);
end

function testTightHorizonContinuesBeforeLaterSeedRecovery(testCase)
    % Meet the unchanged tight deadline and stop before later-seed recovery.
    missionEndTime_s     = 21;
    obstaclePosition_units = [ ...
        -8 7; -5 7; -5 -4; 5 -4; 5 7; 8 7; 8 -7; -8 -7];
    obstacle     = obstacleAvoidance.obstacles.createObstacle("tight static U", [0; missionEndTime_s], obstaclePosition_units(:, 1), obstaclePosition_units(:, 2), 0.2);
    initialState = restState(0, [0 0]);
    goalState    = restState(missionEndTime_s, [0 -10]);
    limits       = physicalLimits();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [0.75 0.75];
    limits.maxJerk_units_s3         = [2.5 2.5];

    twoSeedOptions = struct();
    twoSeedOptions.GoalTimeMode     = "earliestArrival";
    twoSeedOptions.MaximumSeedCount = 2;
    [twoSeedResult, twoSeedResultDiagnosis] = planner(obstacle, initialState, goalState, limits, twoSeedOptions);
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
    [recoveredResult, recoveredResultDiagnosis] = planner(obstacle, initialState, goalState, limits, recoveryOptions);
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
    vertices_units   = [4 -1; 6 -1; 6 1; 4 1];
    firstObstacle  = obstacleAvoidance.obstacles.createObstacle("first", [3; 4.5], vertices_units(:, 1), vertices_units(:, 2), 0);
    secondObstacle = obstacleAvoidance.obstacles.createObstacle("second", [8; 9.5], vertices_units(:, 1), vertices_units(:, 2), 0);
    obstacles      = obstacleAvoidance.obstacles.combineObstacles(firstObstacle, secondObstacle);
    initialState   = restState(0, [0 0]);
    limits         = physicalLimits();
    limits.xInterval_units   = [-1 11];
    limits.yInterval_units = [-0.5 0.5];
    options = plannerOptions("earliestArrival");
    % Exercise each horizon s covered by this regression.
    for horizon_s = [12 16 24 32]
        goalState  = restState(horizon_s, [10 0]);
        result     = planner(obstacles, initialState, goalState, limits, options);
        validation = obstacleAvoidance.validateTrajectory(result);
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, validation.Passed, validation.Message);
        verifyLessThanOrEqual(testCase, result.ArrivalTime_s, 9.5);
        if result.Success
            verifyEqual(testCase, sum(vecnorm(diff(result.position_units), 2, 2)), 10, "AbsTol", 1e-9);
        end
    end
end

function testDirectWaitRetimingHandlesTranslationAndOffsetClock(testCase)
    % Preserve a diagonal path while shortening a horizon-stretched moving body.
    vertices_units = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
    obstacles    = obstacleAvoidance.obstacles.createObstacle("obstacle", [3; 9; 9.5; 23], repmat({vertices_units(:, 1)}, 4, 1), {vertices_units(:, 2); vertices_units(:, 2); vertices_units(:, 2) + 8; vertices_units(:, 2) + 8}, 0.1);
    initialState = restState(3, [-5 0]);
    goalState    = restState(23, [5 2]);
    limits       = struct();
    limits.maxVelocity_units_s      = [2 1];
    limits.maxAcceleration_units_s2 = [1 0.5];
    limits.maxJerk_units_s3         = [2 1];
    limits.xInterval_units    = [-6 6];
    limits.yInterval_units  = [-0.5 2.5];
    result     = planner(obstacles, initialState, goalState, limits, struct("GoalTimeMode", "earliestArrival"));
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    if result.Success
        verifyLessThanOrEqual(testCase, result.ArrivalTime_s, 14.5);
        verifyEqual(testCase, sum(vecnorm(diff(result.position_units), 2, 2)), norm(goalState.position_units - initialState.position_units), "AbsTol", 1e-9);
    end
end

function testEarliestArrivalRefinesObjectiveRelevantDirectWait(testCase)
    % Shorten a validated direct wait without changing its spatial path.
    obstacleTime_s             = [0; 6; 6.5; 12];
    barrierCenterY_units = [0; 0; 8; 8];
    sourcePosition_units         = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
    xBySlice_units         = cell(numel(obstacleTime_s), 1);
    yBySlice_units       = cell(numel(obstacleTime_s), 1);
    % Exercise each sample covered by this regression.
    for sampleIndex = 1:numel(obstacleTime_s)
        translatedPosition_units = sourcePosition_units + [0 barrierCenterY_units(sampleIndex)];
        xBySlice_units{sampleIndex} = translatedPosition_units(:, 1);
        yBySlice_units{sampleIndex} = translatedPosition_units(:, 2);
    end
    obstacle     = obstacleAvoidance.obstacles.createObstacle("earliest direct wait", obstacleTime_s, xBySlice_units, yBySlice_units, 0.1);
    initialState = restState(0, [-5 0]);
    goalState    = restState(12, [5 0]);
    limits       = physicalLimits();
    limits.xInterval_units   = [-6 6];
    limits.yInterval_units = [-3 3];
    options = struct();
    options.GoalTimeMode     = "earliestArrival";
    options.MaximumSeedCount = 5;
    options.SampleTime_s     = 0.05;

    [result, resultDiagnosis] = planner(obstacle, initialState, goalState, limits, options);
    directWaitIndex = find(string({resultDiagnosis.Routes.Source}) == "directWait", 1);
    verifyTrue(testCase, result.Success, result.Message);
    verifyNotEmpty(testCase, directWaitIndex);
    diagnostics = testSupport.solverDetails(resultDiagnosis, directWaitIndex);
    validation  = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, testSupport.diagnosisValue(diagnostics, "DepartureScheduling.Accepted"));
    verifyLessThan(testCase, testSupport.diagnosisValue(diagnostics, "FinalWaitTime_s"), testSupport.diagnosisValue(diagnostics, "InitialWaitTime_s"));
end

function testConvergenceSummaryAndRetainedBestTrial(testCase)
    % Preserve convergence evidence and the best validated trajectory trial.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(repositoryRoot, "examples"));
    overrides = struct();
    overrides.CollisionClearanceTolerance_units = 1e-4;
    overrides.MaximumSeedCount                = 2;
    overrides.FigureVisible                   = "off";
    overrides.PlotOutputs                     = false;
    overrides.ShowAnimation                   = false;
    overrides.ShowKinematicPlot               = false;
    [result, resultDiagnosis] = exampleTargetExitsObstacle(overrides);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    diagnostics = testSupport.solverDetails(resultDiagnosis, resultDiagnosis.SelectedAttemptIndex);
    verifyTrue(testCase, testSupport.diagnosisValue(diagnostics, "TravelRefinementAttempted"));
    verifyLessThanOrEqual(testCase, ...
        testSupport.diagnosisValue(diagnostics, "TravelRefinementFinalLength_units"), ...
        testSupport.diagnosisValue(diagnostics, "TravelRefinementInitialLength_units"));
    verifyEqual(testCase, ...
        testSupport.diagnosisValue(diagnostics, "TravelRefinementFinalDuration_s"), ...
        result.TrajectoryDuration_s, "AbsTol", 1e-9);
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
    limits.xInterval_units   = [-12 12];
    limits.yInterval_units = [-4 4];
    [result, resultDiagnosis] = planner(obstacles, initialState, goalState, limits, plannerOptions("earliestArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyTrue(testCase, result.Validation.CollisionResolved);
    diagnostics = testSupport.solverDetails(resultDiagnosis, resultDiagnosis.SelectedAttemptIndex);
    verifyEqual(testCase, testSupport.diagnosisValue(diagnostics, "Identifier"), "monotoneStaticCorridor");
    verifyEqual(testCase, result.TrajectoryDuration_s, 12.5, "AbsTol", 1e-9);
    verifyEqual(testCase, resultDiagnosis.Routes(resultDiagnosis.SelectedAttemptIndex).Length_units, sum(vecnorm(diff(result.Route_units), 2, 2)), "AbsTol", 1e-12);
    % The refined detour must remain within 1.25 percent of the direct distance.
    directLength_units = norm(goalState.position_units - initialState.position_units);
    verifyLessThan(testCase, sum(vecnorm(diff(result.position_units), 2, 2)), 1.0125 * directLength_units, "The exact-clock detour contains unnecessary joint travel.");
end

function testNearStartBarrierKeepsShortOneSidedExactClockDetour(testCase)
    % A local static obstruction must retain a short, one-sided detour.
    obstacle     = obstacleAvoidance.obstacles.createObstacle("offset rectangle", [0; 30], [-9 -6 -6 -9], [0 0 2.8 2.8], 0.1);
    initialState = restState(0, [-10 3.2]);
    goalState    = restState(30, [10 -1]);
    limits       = physicalLimits();
    limits.xInterval_units   = [-20 20];
    limits.yInterval_units = [-10 10];
    result = planner(obstacle, initialState, goalState, limits, plannerOptions("earliestArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyTrue(testCase, result.Validation.CollisionResolved);
    verifyEqual(testCase, result.TrajectoryDuration_s, 12.5, "AbsTol", 1e-9);
    verifyLessThan(testCase, sum(vecnorm(diff(result.position_units), 2, 2)), 20.55, "The exact-clock detour contains unnecessary joint travel.");
    progress            = (result.position_units(:, 1) - initialState.position_units(1)) / (goalState.position_units(1) - initialState.position_units(1));
    directY_units = initialState.position_units(2) + (goalState.position_units(2) - initialState.position_units(2)) * progress;
    verifyGreaterThanOrEqual(testCase, min(result.position_units(:, 2) - directY_units), -1e-9);
end

function testEarliestDefaultReportsUtilization(testCase)
    % Report measured envelope use on success.
    initialState = restState(0, [0 0]);
    goalState    = restState(20, [4 2]);
    [result, resultDiagnosis] = planner([], initialState, goalState, physicalLimits());

    verifyTrue(testCase, result.Success, result.Message);
    summary = resultDiagnosis.Attempts(resultDiagnosis.SelectedAttemptIndex);
    verifyEqual(testCase, result.Options.GoalTimeMode, "earliestArrival");
    verifyGreaterThan(testCase, summary.KinematicUtilization, 0);
    verifyLessThanOrEqual(testCase, summary.KinematicUtilization, 1 + 1e-6);
    verifyEqual(testCase, resultDiagnosis.Selection.JerkRole, "hardConstraintOnly");
end

function testRankingUsesOnlyDeclaredObjectiveQuantities(testCase)
    % Prefer shorter equal-arrival motions without a hidden utilization policy.
    summary = repmat(struct("MotionLength_units", 10, "KinematicUtilization", 1, "ArrivalTime_s", 5, "ValidationPassed", true), 2, 1);
    summary(2).MotionLength_units = 9;
    summary(2).KinematicUtilization = 0.1;
    % Exercise each mode covered by this regression.
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
    [result, resultDiagnosis] = planner([], initialState, goalState, limits, plannerOptions("fixedArrival"));

    verifyFalse(testCase, result.Success);
    verifyNotEmpty(testCase, result.Message);
    verifyTrue(testCase, any(result.TerminationReason == ["noValidatedSeed", "fixedArrivalInfeasible", "timeWindowInfeasible"]));
    verifyEmpty(testCase, result.time_s);
    verifyTrue(testCase, isfield(resultDiagnosis, "Timing"));
end

function testNonRestEndpointUsesStateToStateSolver(testCase)
    % A non-rest endpoint bypasses BMTP and returns a validated motion.
    initialState                = restState(0, [0 0]);
    initialState.velocity_units_s = [0.1 0];
    goalState                   = restState(10, [2 0]);
    result                      = planner([], initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));
    validation                  = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
end

function testNoPathReturnsRecognizedDiagnostics(testCase)
    % A full-height wall must fail with retained search evidence.
    wall         = obstacleAvoidance.obstacles.createObstacle("full-height wall", [0; 20], [-0.5 0.5 0.5 -0.5], [-90 -90 90 90], 0);
    initialState = restState(0, [-5 0]);
    goalState    = restState(12, [5 0]);
    limits       = physicalLimits();
    limits.yInterval_units = [-10 10];
    options = plannerOptions("earliestArrival");
    options.MaximumSeedCount = 3;
    [result, resultDiagnosis] = planner(wall, initialState, goalState, limits, options);

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
    first        = planner([], initialState, goalState, physicalLimits(), options);
    second       = planner([], initialState, goalState, physicalLimits(), options);

    verifyEqual(testCase, second.TerminationReason, first.TerminationReason);
    verifyEqual(testCase, second.time_s, first.time_s);
    verifyEqual(testCase, second.position_units, first.position_units);
    verifyEqual(testCase, second.velocity_units_s, first.velocity_units_s);
    verifyEqual(testCase, second.acceleration_units_s2, first.acceleration_units_s2);
    verifyEqual(testCase, second.jerk_units_s3, first.jerk_units_s3);
end

function testInterceptValidatesInitialStateBeforeFieldAccess(testCase)
    % Use the shared identified state error for malformed public input.
    targetMotion = struct();
    targetMotion.time_s       = [0; 1];
    targetMotion.position_units = [0 0; 1 0];
    verifyError(testCase, @() planner([], struct(), struct("time_s", 1, "targetMotion", targetMotion), physicalLimits()), "planner:InvalidState");
end

function testInterceptEchoesTheFixedArrivalModeItUses(testCase)
    % Intercept trials select one terminal time, regardless of the caller mode.
    targetMotion = struct();
    targetMotion.time_s       = [0; 5];
    targetMotion.position_units = [0 0; 4 0];
    goal = struct("time_s", 5, "targetMotion", targetMotion);
    [result, resultDiagnosis] = planner([], restState(0, [0 0]), goal, physicalLimits(), plannerOptions("fixedArrival"));

    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.Options.GoalTimeMode, "fixedArrival");
    verifyEqual(testCase, testSupport.diagnosisValue(resultDiagnosis.InterceptSearch, "OptimalityStatus"), "notAnOptimization");

    % Exercise the bounded time-grid search with BMTP and a PCHIP target.
    movingInitialState = restState(0, [0 0]);
    movingTarget       = struct("time_s", (0:0.5:12).', ...
        "position_units", repmat([6 1], 25, 1), ...
        "InterpolationMethod", "pchip");
    goal = struct("time_s", 12, "targetMotion", movingTarget);
    [asapResult, asapResultDiagnosis] = planner([], movingInitialState, goal, physicalLimits());
    verifyTrue(testCase, asapResult.Success, asapResult.Message);
    verifyTrue(testCase, asapResult.Validation.Passed, asapResult.Validation.Message);
    verifyEqual(testCase, asapResult.Intercept.Mode, "earliest");
    verifyEqual(testCase, testSupport.diagnosisValue(asapResultDiagnosis.InterceptSearch, "OptimalityStatus"), "resolutionBounded");
    verifyEqual(testCase, testSupport.diagnosisValue(asapResultDiagnosis.InterceptSearch, "MaximumCoarseStep_s"), 0.5, "AbsTol", 1e-12);

    goal.time_s = 10;
    atTimeResult = planner([], movingInitialState, goal, physicalLimits(), struct("GoalTimeMode", "fixedArrival"));
    verifyTrue(testCase, atTimeResult.Success, atTimeResult.Message);
    verifyTrue(testCase, atTimeResult.Validation.Passed, atTimeResult.Validation.Message);
    verifyEqual(testCase, atTimeResult.Intercept.Mode, "specifiedTime");
    verifyEqual(testCase, atTimeResult.Intercept.Time_s, 10, "AbsTol", 1e-12);
    verifyLessThan(testCase, asapResult.Intercept.Time_s, atTimeResult.Intercept.Time_s);

    certifiedTarget = movingTarget;
    certifiedTarget.InterpolationMethod = "linear";
    [certifiedResult, certifiedResultDiagnosis] = planner([], restState(0, [0 0]), struct("time_s", 12, "targetMotion", certifiedTarget), physicalLimits());
    verifyTrue(testCase, certifiedResult.Success, certifiedResult.Message);
    verifyEqual(testCase, testSupport.diagnosisValue(certifiedResultDiagnosis.InterceptSearch, "OptimalityStatus"), "certifiedEarliest");
end

function testSpecifiedInterceptMatchesTargetDerivatives(testCase)
    % Match target derivatives without sending a non-rest request to BMTP.
    initialState = restState(0, [0 0]);
    targetTime_s = [0; 5; 10; 15];
    targetMotion = struct("time_s", targetTime_s, ...
        "position_units", [2 + 0.2 * targetTime_s, 1 + 0.03 * targetTime_s.^2], ...
        "InterpolationMethod", "pchip");
    cases    = [true false; false true; true true];
    policies = ["zero", "target"];
    % Exercise each case covered by this regression.
    for caseIndex = 1:size(cases, 1)
        goal = struct("time_s", 10, "targetMotion", targetMotion, ...
            "MatchTargetVelocity", cases(caseIndex, 1), ...
            "MatchTargetAcceleration", cases(caseIndex, 2));
        result = planner([], initialState, goal, physicalLimits(), struct("GoalTimeMode", "fixedArrival", "MaximumSeedCount", 1));
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
        verifyEqual(testCase, result.Intercept.TerminalVelocityPolicy, policies(cases(caseIndex, 1) + 1));
        verifyEqual(testCase, result.Intercept.TerminalAccelerationPolicy, policies(cases(caseIndex, 2) + 1));
    end
end

function testFixedArrivalSupportsNonRestTerminalState(testCase)
    % Exercise the same invariant without the moving-target wrapper.
    initialState = restState(0, [-1 0]);
    goalState    = struct("time_s", 8, "position_units", [3 1], ...
        "velocity_units_s", [0.2 -0.1], "acceleration_units_s2", [0.05 0]);
    options                  = plannerOptions("fixedArrival");
    options.MaximumSeedCount = 1;
    result                   = planner([], initialState, goalState, physicalLimits(), options);
    validation               = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyEqual(testCase, result.position_units(end, :), goalState.position_units, "AbsTol", 1e-8);
    verifyEqual(testCase, result.velocity_units_s(end, :), goalState.velocity_units_s, "AbsTol", 1e-8);
    verifyEqual(testCase, result.acceleration_units_s2(end, :), goalState.acceleration_units_s2, "AbsTol", 1e-8);
end

function state = restState(time_s, position_units)
    % Create one two-axis rest endpoint.
    state = struct("time_s", time_s, "position_units", position_units, ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
end

function limits = physicalLimits()
    % Create neutral two-axis bounds used by the public-contract cases.
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3         = [2 2];
    limits.xInterval_units    = [-180 180];
    limits.yInterval_units  = [-90 90];
end

function options = plannerOptions(goalTimeMode)
    % Resolve only behavior-level public choices for deterministic tests.
    options = struct("GoalTimeMode", goalTimeMode, ...
        "SampleTime_s", 0.05);
end

function testTargetGoalPreservesExplicitTerminalDerivatives(testCase)
    initial = restState(0,[0 0]);
    target = struct('time_s',[0;10],'position_units',[2 0;4 1]);
    goal = struct('time_s',8,'targetMotion',target,'velocity_units_s',[0.1 0], ...
        'acceleration_units_s2',[0 0.01]);
    result = planner([],initial,goal,physicalLimits(),struct('GoalTimeMode',"fixedArrival"));
    verifyTrue(testCase,result.Success,result.Message);
    verifyEqual(testCase,result.Inputs.goalState.velocity_units_s,goal.velocity_units_s);
    verifyEqual(testCase,result.Inputs.goalState.acceleration_units_s2,goal.acceleration_units_s2);
    verifyEqual(testCase,result.Intercept.TerminalVelocityPolicy,"specified");
    verifyEqual(testCase,result.Intercept.TerminalAccelerationPolicy,"specified");
    check = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase,check.Passed,check.Message);
    verifyError(testCase,@()planner([],initial,goal,physicalLimits()),"planner:UnsupportedMovingDerivative");
end
