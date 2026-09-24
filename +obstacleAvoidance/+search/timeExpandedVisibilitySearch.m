function [route_units, routeTime_s, timedSearchDetails] = timeExpandedVisibilitySearch( ...
    nodePosition_units, edgeCost_units, obstacles, initialState, goalState, ...
    limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_units, routeTime_s, timedSearchDetails] = ...
%       obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
%       nodePosition_units, edgeCost_units, obstacles, initialState, ...
%       goalState, limits, sampleTimes_s, options)
%**************************************************************************
% PURPOSE
%   - Search routes that move between candidate positions or wait at them.
%     Each position is considered at several times, so a blocked passage
%     can become usable after an obstacle moves.
%   - Check each connection throughout its travel time. The returned route
%     is a proposal for BMTP, not yet a complete validated vehicle motion.
%**************************************************************************
% INPUTS
%   - nodePosition_units (N-by-2 numeric array)
%       Search nodes with the start first and goal second.
%   - edgeCost_units (N-by-N numeric array)
%       Finite entries allow a connection; Inf disables it. Connection
%       lengths are calculated from nodePosition_units during the search.
%   - obstacles (standard protected obstacle array)
%       Static and moving geometry.
%   - initialState (scalar struct)
%       Normalized initial endpoint state.
%   - goalState (scalar struct)
%       Normalized goal endpoint state.
%   - limits (scalar struct)
%       Checked workspace, speed, acceleration, and jerk limits.
%   - sampleTimes_s (numeric vector)
%       Candidate search times. One layer contains every node at one time.
%   - options (scalar struct)
%       Normalized timed-search options.
%**************************************************************************
% OUTPUTS
%   - route_units (N-by-2 numeric array)
%       Selected route positions, or empty if the search finds no route.
%   - routeTime_s (N-by-1 numeric array)
%       Time at each route point, or empty if the search finds no route.
%   - timedSearchDetails (scalar struct)
%       Search counts, the selected period when the goal stays clear, and
%       an additional route allowing a later arrival within that period.
%       GoalWindowPreviousLayerTime_s is the layer just before that period:
%       waiting at the goal from that layer to the period start was not
%       clear. It is NaN when the period starts at the first layer. It is not
%       a lower bound on arrival; a motion may finish at the goal earlier.
%       Finding no route is a normal outcome; invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position and edge cost are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Set The Positions And Times To Search

% A layer contains all route points at one time. The search can move to
% a different point at a later layer, or wait at the same point if it stays clear.
layerTimes_s          = unique([initialState.time_s; sampleTimes_s(:); goalState.time_s]);
layerTimes_s          = layerTimes_s(layerTimes_s >= initialState.time_s & layerTimes_s <= goalState.time_s);
layerCount            = numel(layerTimes_s);
nodeCount             = size(nodePosition_units, 1);
initialPosition_units = nodePosition_units(1, :);
goalPosition_units    = nodePosition_units(2, :);
if isfield(initialState, 'position_units')
    initialPosition_units = initialState.position_units;
end
if isfield(goalState, 'position_units')
    goalPosition_units = goalState.position_units;
end
% Even without obstacles, distance / maximum speed limits how early the
% vehicle can arrive. Use acceleration and jerk limits too when available.
displacement_units       = abs(goalPosition_units - initialPosition_units);
minimumDuration_s        = max(displacement_units ./ limits.maxVelocity_units_s);
minimumGoalArrivalTime_s = initialState.time_s + minimumDuration_s;

endpointFieldNames           = {'position_units', 'velocity_units_s', 'acceleration_units_s2'};
hasEndpointMotionValues      = all(isfield(initialState, endpointFieldNames)) && all(isfield(goalState, endpointFieldNames));
hasAccelerationAndJerkLimits = all(isfield(limits, {'maxAcceleration_units_s2', 'maxJerk_units_s3'}));
if hasEndpointMotionValues && hasAccelerationAndJerkLimits
    minimumDuration_s        = obstacleAvoidance.input.minimumTravelTime(initialState, goalState, limits);
    minimumGoalArrivalTime_s = initialState.time_s + minimumDuration_s;
end
% Allow for roundoff when comparing large absolute times.
timeTolerance_s                = 256 * eps(max(1, max(abs(layerTimes_s))));
goalLayerIsEligible            = layerTimes_s >= minimumGoalArrivalTime_s - timeTolerance_s;
hasGoalVelocityAndAcceleration = all(isfield(goalState, ...
    {'velocity_units_s', 'acceleration_units_s2'}));
if options.GoalTimeMode == "fixedArrival" && hasGoalVelocityAndAcceleration && ...
        any([goalState.velocity_units_s, goalState.acceleration_units_s2] ~= 0)
    % A goal with nonzero velocity or acceleration cannot be reached early
    % and held still. Require arrival at the specified goal time.
    goalLayerIsEligible(:)   = false;
    goalLayerIsEligible(end) = true;
end

%% Section 2: Prepare Obstacles And Reusable Collision Checks

% Keep one prepared obstacle history for all route checks in this search.
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles, [initialState.time_s, goalState.time_s]);
% Save shapes for repeated queries at the same time. Reduce the number
% saved for large boundaries or many obstacles to limit memory use.
for obstacleIndex = 1:numel(obstacles)
    obstacles(obstacleIndex).InternalPreparation.QueryGeometryCache = containers.Map( ...
        'KeyType', 'double', 'ValueType', 'any');
    maximumBoundaryRowCount = max([1; cellfun(@numel, obstacles(obstacleIndex).x_units(:))]);
    shapeCacheCapacity      = floor(2^14 / max(1, numel(obstacles)) / maximumBoundaryRowCount);
    obstacles(obstacleIndex).InternalPreparation.QueryGeometryCacheCapacity = shapeCacheCapacity;
end
maximumCacheBytes = 300 * 1024 ^ 2;
% Check each static obstacle only during the times it exists. Use its
% prepared boundary, which already includes the safety margin.
obstacleIsStationary = arrayfun(@(obstacle) obstacle.InternalPreparation.SamplesExactlyEqual, obstacles);
dynamicObstacles     = obstacles(~obstacleIsStationary);
staticObstacles      = obstacles(obstacleIsStationary);
staticShapeCount     = numel(staticObstacles);
movingObstacleCells  = obstacleAvoidance.obstacles.createTimeCells( ...
    dynamicObstacles, initialState.time_s, goalState.time_s);
staticShapes            = cell(staticShapeCount, 1);
staticEdgeStart_units   = cell(staticShapeCount, 1);
staticEdgeEnd_units     = cell(staticShapeCount, 1);
staticActiveIntervals_s = repmat([-Inf, Inf], staticShapeCount, 1);
staticShapeIsPrepared   = false(staticShapeCount, 1);
for staticObstacleIndex = 1:staticShapeCount
    obstacle            = staticObstacles(staticObstacleIndex);
    preparation         = obstacle.InternalPreparation;
    preparedSampleIndex = find(preparation.SamplePrepared, 1, "first");
    if isempty(preparedSampleIndex)
        continue;
    end
    staticShapeIsPrepared(staticObstacleIndex) = true;
    staticShapes{staticObstacleIndex}          = preparation.SampleShapes{preparedSampleIndex};
    staticEdgeStart_units{staticObstacleIndex} = preparation.SampleEdgeStart_units{preparedSampleIndex};
    staticEdgeEnd_units{staticObstacleIndex}   = preparation.SampleEdgeEnd_units{preparedSampleIndex};
    if numel(obstacle.time_s) > 1
        staticActiveIntervals_s(staticObstacleIndex, :) = [obstacle.time_s(1), obstacle.time_s(end)];
    end
end
% Some moving obstacles use a fixed enclosure covering their whole motion
% between samples. Check that saved shape only during its active interval,
% together with the other stationary shapes.
[stationaryShapes, stationaryStarts_units, stationaryEnds_units, stationaryActive_s, movingObstacleCells] = ...
    extractStationaryEnclosures(dynamicObstacles, movingObstacleCells);
staticShapes            = [staticShapes; stationaryShapes];
staticEdgeStart_units   = [staticEdgeStart_units; stationaryStarts_units];
staticEdgeEnd_units     = [staticEdgeEnd_units; stationaryEnds_units];
staticActiveIntervals_s = [staticActiveIntervals_s; stationaryActive_s];
staticShapeIsPrepared   = [staticShapeIsPrepared; true(numel(stationaryShapes), 1)];
staticShapeCount        = numel(staticShapes);
% Reuse a static-edge check only when the obstacle exists for the entire
% edge travel time. Otherwise, check just the overlapping part of the edge.
% Each cache row represents one directed node pair; columns are shapes.
staticEdgeClearanceCache = zeros(0, 0, 'uint8');
if staticShapeCount > 0 && nodeCount^2 * staticShapeCount <= maximumCacheBytes
    staticEdgeClearanceCache = zeros(nodeCount^2, staticShapeCount, 'uint8');
end
% Prepare region bounds and edge timing data for moving-obstacle checks.
% These reject impossible overlaps before the detailed collision calculation.
movingCellLookup = obstacleAvoidance.search.createMovingCellIndex(movingObstacleCells, nodePosition_units);

%% Section 3: Find Clear Positions And Waiting Periods

nodeIsFree = false(layerCount, nodeCount);
for layerIndex = 1:layerCount
    nodeIsFree(layerIndex, :) = ~obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
        obstacles, nodePosition_units(:, 1), nodePosition_units(:, 2), layerTimes_s(layerIndex), false).';
end
waitIsClear = false(max(0, layerCount - 1), nodeCount);
for layerIndex = 1:layerCount - 1
    candidateNodeIndices = find( ...
        nodeIsFree(layerIndex, :) & nodeIsFree(layerIndex + 1, :));
    % Test stationary waits only at nodes free in both adjacent layers.
    if ~isempty(candidateNodeIndices)
        waitIsClear(layerIndex, candidateNodeIndices) = edgeIsClear( ...
            candidateNodeIndices, candidateNodeIndices, layerTimes_s(layerIndex), layerTimes_s(layerIndex + 1));
    end
end

% A wait window is a consecutive group of layers joined by clear waits.
% For example, a vehicle arriving at t = 1 can wait until t = 3 if that
% position stays clear throughout. Earlier arrival in this window remains usable.
waitWindowStartsHere           = nodeIsFree;
waitWindowStartsHere(2:end, :) = nodeIsFree(2:end, :) & ~waitIsClear;
waitWindowEndLayerIndex        = repmat(uint32((1:layerCount).'), 1, nodeCount);
for layerIndex = layerCount - 1:-1:1
    continuingNodeIndices = find(waitIsClear(layerIndex, :));
    waitWindowEndLayerIndex(layerIndex, continuingNodeIndices) = ...
        waitWindowEndLayerIndex(layerIndex + 1, continuingNodeIndices);
end
motionEdgeExists = isfinite(edgeCost_units);
motionEdgeExists(1:nodeCount + 1:end) = false;
minimumEdgeDuration_s = zeros(nodeCount, nodeCount);
motionEdgeLengths_units = zeros(nodeCount, nodeCount);
for sourceNodeIndex = 1:nodeCount
    displacement_units = nodePosition_units - nodePosition_units(sourceNodeIndex, :);
    minimumEdgeDuration_s(sourceNodeIndex, :) = ...
        max(abs(displacement_units) ./ limits.maxVelocity_units_s, [], 2).';
    for targetNodeIndex = reshape(find(motionEdgeExists(sourceNodeIndex, :)), 1, [])
        motionEdgeLengths_units(sourceNodeIndex, targetNodeIndex) = ...
            norm(displacement_units(targetNodeIndex, :));
    end
end
% Keep the timing and waiting information with the allowed node connections.
% The candidate-building helpers use these arrays to list possible moves.
timedConnectionData = struct( ...
    'LayerTimes_s',                 layerTimes_s, ...
    'NodeIsFree',                   nodeIsFree, ...
    'IsWaitComponentStart',         waitWindowStartsHere, ...
    'WaitComponentFinalLayerIndex', waitWindowEndLayerIndex, ...
    'MotionEdgeExists',             motionEdgeExists, ...
    'MinimumEdgeDuration_s',        minimumEdgeDuration_s, ...
    'MotionEdgeLengths_units',      motionEdgeLengths_units);

%% Section 4: Search For A Route Through Successive Times

distanceToGoal_units           = vecnorm(nodePosition_units - nodePosition_units(2, :), 2, 2);
isEarliestArrival              = options.GoalTimeMode == "earliestArrival";
goalArrivalCompletesRequest    = false(layerCount, 1);
nextValidGoalArrivalLayerIndex = zeros(layerCount, 1);
% For fixed arrival, an earlier visit to the goal can finish the request
% only if waiting there stays clear through the required arrival time.
if ~isEarliestArrival
    goalArrivalCompletesRequest = goalLayerIsEligible & nodeIsFree(:, 2) & ...
        double(waitWindowEndLayerIndex(:, 2)) == layerCount;
    nextValidGoalLayerIndex = 0;
    for layerIndex = layerCount:-1:1
        if goalArrivalCompletesRequest(layerIndex)
            nextValidGoalLayerIndex = layerIndex;
        end
        nextValidGoalArrivalLayerIndex(layerIndex) = nextValidGoalLayerIndex;
    end
end
% Each search state is a node at a layer. Save the shortest route reaching
% it and the preceding node/layer, so the selected route can be traced back.
nodeIsReachable         = false(layerCount, nodeCount);
routeLengthToNode_units = Inf(layerCount, nodeCount);
parentLayerIndex        = zeros(layerCount, nodeCount, "uint32");
parentNodeIndex         = zeros(layerCount, nodeCount, "uint32");

nodeIsReachable(1, 1)         = nodeIsFree(1, 1);
routeLengthToNode_units(1, 1) = 0;

[rejectedCount, expandedCount, provenSkipCount] = deal(0);

bestGoalRouteLength_units       = Inf;
selectedGoalEdgeStartLayerIndex = 0;
selectedGoalEdgeStartNodeIndex  = 0;
selectedGoalArrivalLayerIndex   = 0;
if isEarliestArrival
    % Process possible arrivals in time order. Continue through the first
    % reachable goal wait window when a later route in that window is needed.
    % Every move advances time, so the search cannot loop backward.
    % Edge-check columns: departure layer, source node, target node, last
    % allowed arrival layer, edge length, ordering index, last proven blocked layer.
    pendingEdgeChecks   = cell(layerCount, 1);
    pendingNodeArrivals = cell(layerCount, 1);
    for layerIndex = 1:layerCount
        checkPendingEdges(layerIndex);
        applyPendingNodeArrivals(layerIndex);
        goalWaitWindowIsComplete = hasReachedGoalWaitWindowEnd(layerIndex);
        if goalWaitWindowIsComplete
            break
        end
        if layerIndex == layerCount
            break
        end
        scheduleLayerExpansion(layerIndex);
    end
else
    for layerIndex = 1:layerCount - 1
        propagateFixedArrivalLayer(layerIndex);
    end
end

%% Section 5: Reconstruct The Selected Route And Its Arrival Times

% Earliest-arrival mode selects the first reachable eligible goal layer.
% Fixed-arrival mode uses the shortest selected route and includes any wait
% at the goal needed to finish at the requested time.
if isEarliestArrival
    firstGoalLayerIndex = find(nodeIsReachable(:, 2) & goalLayerIsEligible, 1, "first");
    goalLayerIndex      = firstGoalLayerIndex;
    % Keep a later-arrival route within the same clear goal window to give
    % BMTP more timing room. The selected earliest route remains separate.
    waitRouteGoalLayerIndex = firstGoalLayerIndex;
    if ~isempty(firstGoalLayerIndex)
        waitRouteGoalLayerIndex = double(waitWindowEndLayerIndex(firstGoalLayerIndex, 2));
        if waitRouteGoalLayerIndex == layerCount
            waitRouteGoalLayerIndex = firstGoalLayerIndex;
        end
    end
    [route_units, routeTime_s] = reconstructTimedRoute( ...
        nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex, 2);
    [waitRoute_units, waitRouteTime_s] = reconstructTimedRoute( ...
        nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, waitRouteGoalLayerIndex, 2);
else
    goalLayerIndex = zeros(0, 1);
    route_units    = zeros(0, 2);
    routeTime_s    = zeros(0, 1);
    if selectedGoalArrivalLayerIndex > 0
        [route_units, routeTime_s] = reconstructTimedRoute( ...
            nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, ...
            selectedGoalEdgeStartLayerIndex, selectedGoalEdgeStartNodeIndex);
        route_units(end + 1, :) = nodePosition_units(2, :);
        routeTime_s(end + 1, 1) = layerTimes_s(selectedGoalArrivalLayerIndex);
        if selectedGoalArrivalLayerIndex < layerCount
            route_units(end + 1, :) = nodePosition_units(2, :);
            routeTime_s(end + 1, 1) = layerTimes_s(end);
        end
        goalLayerIndex = layerCount;
    end
    waitRoute_units = route_units;
    waitRouteTime_s = routeTime_s;
end
selectedGoalWindowStartTime_s = NaN;
selectedGoalWindowEndTime_s   = NaN;
goalWindowPreviousLayerTime_s = NaN;
if ~isempty(goalLayerIndex)
    goalWindowStartLayerIndices = find( ...
        waitWindowStartsHere(:, 2) & nodeIsFree(:, 2));
    selectedGoalWindowIndex           = nnz(goalWindowStartLayerIndices <= goalLayerIndex);
    selectedGoalWindowStartLayerIndex = goalWindowStartLayerIndices(selectedGoalWindowIndex);
    selectedGoalWindowEndLayerIndex   = double( ...
        waitWindowEndLayerIndex(selectedGoalWindowStartLayerIndex, 2));
    selectedGoalWindowStartTime_s = layerTimes_s(selectedGoalWindowStartLayerIndex);
    selectedGoalWindowEndTime_s   = layerTimes_s(selectedGoalWindowEndLayerIndex);
    % A window that starts after the first layer means waiting at the goal
    % from the previous layer to this one was not clear. No layer sampled
    % the times in between.
    if selectedGoalWindowStartLayerIndex > 1
        goalWindowPreviousLayerTime_s = layerTimes_s(selectedGoalWindowStartLayerIndex - 1);
    end
end
timedSearchDetails = struct( ...
    "NodeCount",                     nodeCount, ...
    "RejectedTransitionCount",       rejectedCount, ...
    "ProvenSkippedTransitionCount",  provenSkipCount, ...
    "ExpandedCount",                 expandedCount, ...
    "SelectedGoalWindowStartTime_s", selectedGoalWindowStartTime_s, ...
    "SelectedGoalWindowEndTime_s",   selectedGoalWindowEndTime_s, ...
    "GoalWindowPreviousLayerTime_s", goalWindowPreviousLayerTime_s, ...
    "MinimumGoalArrivalTime_s",      minimumGoalArrivalTime_s, ...
    "WaitRoute_units",               waitRoute_units, ...
    "WaitRouteTime_s",               waitRouteTime_s);

%% Section 6: Search Steps And Shared Collision Checks

% These nested functions share the current search arrays and obstacle data.
% Their updates remain in this search call; they are not separate planners.

function checkPendingEdges(layerIndex)
    % Test moves scheduled to arrive at this layer. A blocked move may try
    % the next arrival layer within the same target wait window.
    edgeChecks = pendingEdgeChecks{layerIndex};
    pendingEdgeChecks{layerIndex} = zeros(0, 7);
    if ~isempty(edgeChecks)
        edgeChecks         = sortrows(edgeChecks, [1, 6]);
        sourceLayerIndices = unique(edgeChecks(:, 1), "stable").';
        for sourceLayerIndex = sourceLayerIndices
            sourceEventRows      = find(edgeChecks(:, 1) == sourceLayerIndex);
            eventIsProvenBlocked = ...
                layerIndex <= edgeChecks(sourceEventRows, 7);
            eventIsClear          = false(numel(sourceEventRows), 1);
            blockingCellIndices   = zeros(numel(sourceEventRows), 1, "uint32");
            collisionTimes_s      = NaN(numel(sourceEventRows), 1);
            uncheckedEventOffsets = find(~eventIsProvenBlocked);
            if ~isempty(uncheckedEventOffsets)
                eventRowsToCheck = sourceEventRows(uncheckedEventOffsets);
                [checkedEdgeIsClear, checkedBlockingCellIndices, checkedCollisionTimes_s] = edgeIsClear( ...
                    edgeChecks(eventRowsToCheck, 2), edgeChecks(eventRowsToCheck, 3), ...
                    layerTimes_s(sourceLayerIndex), layerTimes_s(layerIndex));
                eventIsClear(uncheckedEventOffsets)        = checkedEdgeIsClear;
                blockingCellIndices(uncheckedEventOffsets) = checkedBlockingCellIndices;
                collisionTimes_s(uncheckedEventOffsets)    = checkedCollisionTimes_s;
            end

            clearEventRows = sourceEventRows(eventIsClear);
            if ~isempty(clearEventRows)
                clearProposals = [ ...
                    edgeChecks(clearEventRows, 1:3), ...
                    edgeChecks(clearEventRows, 5), ...
                    ones(numel(clearEventRows), 1), ...
                    edgeChecks(clearEventRows, 6)];
                pendingNodeArrivals{layerIndex} = [ ...
                    pendingNodeArrivals{layerIndex}; clearProposals];
            end

            failedEventOffsets = find(~eventIsClear);
            provenSkipCount    = provenSkipCount + nnz(eventIsProvenBlocked);
            rejectedCount      = rejectedCount + numel(failedEventOffsets);
            for failureIndex = 1:numel(failedEventOffsets)
                eventOffset = failedEventOffsets(failureIndex);
                eventRow    = sourceEventRows(eventOffset);
                if ~eventIsProvenBlocked(eventOffset)
                    blockedLayerCount = provenBlockedLayerCount( ...
                        edgeChecks(eventRow, 2), edgeChecks(eventRow, 3), ...
                        sourceLayerIndex, layerIndex, edgeChecks(eventRow, 4), ...
                        blockingCellIndices(eventOffset), collisionTimes_s(eventOffset));
                    edgeChecks(eventRow, 7) = layerIndex + blockedLayerCount - 1;
                end
                if layerIndex < edgeChecks(eventRow, 4)
                    pendingEdgeChecks{layerIndex + 1}(end + 1, :) = ...
                        edgeChecks(eventRow, :);
                end
            end
        end
    end
end

function applyPendingNodeArrivals(layerIndex)
    % Arrival columns: departure layer, source node, target node, edge length,
    % move kind (0 = wait, 1 = travel), and ordering index. Sort first so
    % equal-length routes keep the same winner on repeated runs.
    arrivalProposals = pendingNodeArrivals{layerIndex};
    if ~isempty(arrivalProposals)
        arrivalProposals = sortrows(arrivalProposals, [1, 5, 6]);
        for proposalIndex = 1:size(arrivalProposals, 1)
            [nodeIsReachable, routeLengthToNode_units, parentLayerIndex, parentNodeIndex] = keepShorterRouteAtNodeAndTime( ...
                nodeIsReachable, routeLengthToNode_units, parentLayerIndex, parentNodeIndex, ...
                arrivalProposals(proposalIndex, 1), arrivalProposals(proposalIndex, 2), ...
                layerIndex, arrivalProposals(proposalIndex, 3), arrivalProposals(proposalIndex, 4));
        end
    end
end

function goalWaitWindowIsComplete = hasReachedGoalWaitWindowEnd(layerIndex)
    % After the first eligible goal arrival, finish its clear wait window
    % so a later-arrival route is available to BMTP. If the window already
    % reaches the request deadline, the first arrival is enough.
    goalWaitWindowIsComplete = false;
    firstGoalLayerIndex      = find( ...
        nodeIsReachable(1:layerIndex, 2) & goalLayerIsEligible(1:layerIndex), 1, "first");
    if ~isempty(firstGoalLayerIndex)
        selectedGoalLayerIndex = double(waitWindowEndLayerIndex(firstGoalLayerIndex, 2));
        if selectedGoalLayerIndex == layerCount
            selectedGoalLayerIndex = firstGoalLayerIndex;
        end
        if layerIndex >= selectedGoalLayerIndex
            goalWaitWindowIsComplete = true;
        end
    end
end

function scheduleLayerExpansion(layerIndex)
    % Queue clear waits for the next layer and moving connections for their
    % earliest allowed arrival layers. Collision checks run when due.
    currentNodeIndices = find(nodeIsReachable(layerIndex, :));
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        if waitIsClear(layerIndex, currentNodeIndex)
            waitProposal = [layerIndex, currentNodeIndex, currentNodeIndex, ...
                0, 0, currentNodeIndex];
            pendingNodeArrivals{layerIndex + 1}(end + 1, :) = waitProposal;
        else
            rejectedCount = rejectedCount + 1;
        end
    end

    [motionCandidates, candidateRejectedCount] = buildLayerCandidates( ...
        currentNodeIndices, layerIndex, timedConnectionData);
    rejectedCount      = rejectedCount + candidateRejectedCount;
    targetLayerIndices = unique(motionCandidates(:, 3)).';
    for targetLayerIndex = targetLayerIndices
        candidateRows = find(motionCandidates(:, 3) == targetLayerIndex);
        eventBlock    = [ ...
            repmat(layerIndex, numel(candidateRows), 1), ...
            motionCandidates(candidateRows, [1, 2, 4, 5]), ...
            candidateRows, zeros(numel(candidateRows), 1)];
        pendingEdgeChecks{targetLayerIndex} = [ ...
            pendingEdgeChecks{targetLayerIndex}; eventBlock];
    end
end

function propagateFixedArrivalLayer(layerIndex)
    % Extend every reachable node by waiting or moving. Keep shorter routes
    % to each state, while retaining goal arrivals that can finish on time.
    currentNodeIndices = find(nodeIsReachable(layerIndex, :));
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        if waitIsClear(layerIndex, currentNodeIndex)
            [nodeIsReachable, routeLengthToNode_units, parentLayerIndex, parentNodeIndex] = keepShorterRouteAtNodeAndTime( ...
                nodeIsReachable, routeLengthToNode_units, parentLayerIndex, parentNodeIndex, ...
                layerIndex, currentNodeIndex, layerIndex + 1, currentNodeIndex, 0);
        else
            rejectedCount = rejectedCount + 1;
        end
    end
    [motionCandidates, candidateRejectedCount] = buildLayerCandidates( ...
        currentNodeIndices, layerIndex, timedConnectionData);
    rejectedCount        = rejectedCount + candidateRejectedCount;
    motionCandidateCount = size(motionCandidates, 1);
    motionIsPending      = true(motionCandidateCount, 1);
    while any(motionIsPending)
        queriedTargetLayerIndices = unique(motionCandidates(motionIsPending, 3));
        for targetLayerIndex = reshape(queriedTargetLayerIndices, 1, [])
            queryIndices           = find(motionIsPending & motionCandidates(:, 3) == targetLayerIndex);
            trialRouteLength_units = reshape( ...
                routeLengthToNode_units(layerIndex, motionCandidates(queryIndices, 1)), [], 1) + ...
                motionCandidates(queryIndices, 5);
            knownRouteLength_units = reshape( ...
                routeLengthToNode_units(targetLayerIndex, motionCandidates(queryIndices, 2)), [], 1);
            % A longer route to the same node/time cannot improve that state.
            % Keep a goal candidate if a later arrival in its wait window may
            % complete the fixed-time request.
            hasShorterKnownRoute          = trialRouteLength_units > knownRouteLength_units + 1e-12;
            candidateTerminalLayerIndices = ...
                nextValidGoalArrivalLayerIndex(motionCandidates(queryIndices, 3));
            allowsRequiredGoalArrival = motionCandidates(queryIndices, 2) == 2 & ...
                candidateTerminalLayerIndices > 0 & ...
                candidateTerminalLayerIndices <= motionCandidates(queryIndices, 4);
            hasShorterKnownRoute(allowsRequiredGoalArrival)     = false;
            motionIsPending(queryIndices(hasShorterKnownRoute)) = false;

            rejectedCount          = rejectedCount + nnz(hasShorterKnownRoute);
            queryIndices           = queryIndices(~hasShorterKnownRoute);
            trialRouteLength_units = trialRouteLength_units(~hasShorterKnownRoute);
            % The straight distance to the goal is the least additional
            % route length possible. Skip a route already longer than the
            % best completed route even with that optimistic remainder.
            remainingDistance_units = distanceToGoal_units(motionCandidates(queryIndices, 2));
            cannotShortenGoalRoute = trialRouteLength_units + remainingDistance_units > bestGoalRouteLength_units + 1e-12;
            motionIsPending(queryIndices(cannotShortenGoalRoute)) = false;

            rejectedCount = rejectedCount + nnz(cannotShortenGoalRoute);
            queryIndices  = queryIndices(~cannotShortenGoalRoute);
            if isempty(queryIndices)
                continue
            end
            queryIsClear = edgeIsClear( ...
                motionCandidates(queryIndices, 1), motionCandidates(queryIndices, 2), ...
                layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
            clearIndices = queryIndices(queryIsClear);
            for motionIndex = reshape(clearIndices, 1, [])
                [nodeIsReachable, routeLengthToNode_units, parentLayerIndex, parentNodeIndex] = keepShorterRouteAtNodeAndTime( ...
                    nodeIsReachable, routeLengthToNode_units, parentLayerIndex, parentNodeIndex, ...
                    layerIndex, motionCandidates(motionIndex, 1), ...
                    motionCandidates(motionIndex, 3), motionCandidates(motionIndex, 2), ...
                    motionCandidates(motionIndex, 5));
            end
            arrivalCompletesRequest = motionCandidates(clearIndices, 2) == 2 & ...
                goalArrivalCompletesRequest(motionCandidates(clearIndices, 3));
            terminalIndices = clearIndices(arrivalCompletesRequest);
            if ~isempty(terminalIndices)
                goalRouteLengths_units = reshape( ...
                    routeLengthToNode_units(layerIndex, motionCandidates(terminalIndices, 1)), [], 1) + ...
                    motionCandidates(terminalIndices, 5);
                [trialGoalRouteLength_units, shortestGoalRouteOffset] = min(goalRouteLengths_units);
                if trialGoalRouteLength_units < bestGoalRouteLength_units - 1e-12
                    terminalIndex = terminalIndices(shortestGoalRouteOffset);

                    bestGoalRouteLength_units       = trialGoalRouteLength_units;
                    selectedGoalEdgeStartLayerIndex = layerIndex;
                    selectedGoalEdgeStartNodeIndex  = motionCandidates(terminalIndex, 1);
                    selectedGoalArrivalLayerIndex   = motionCandidates(terminalIndex, 3);
                end
            end
            rejectedCount                 = rejectedCount + nnz(~queryIsClear);
            motionIsPending(queryIndices) = false;
            advanceIndices                = queryIndices( ...
                ~queryIsClear & motionCandidates(queryIndices, 3) < motionCandidates(queryIndices, 4));
            motionCandidates(advanceIndices, 3) = motionCandidates(advanceIndices, 3) + 1;
            motionIsPending(advanceIndices)     = true;
            % A clear visit to the goal may be too early or unable to wait
            % through the deadline. Try the next allowed finishing layer
            % within this connection's target wait window.
            clearTransitGoalIndices = queryIndices( ...
                queryIsClear & motionCandidates(queryIndices, 2) == 2 & ...
                ~goalArrivalCompletesRequest(motionCandidates(queryIndices, 3)));
            for motionIndex = reshape(clearTransitGoalIndices, 1, [])
                currentTargetLayerIndex = motionCandidates(motionIndex, 3);
                if currentTargetLayerIndex == layerCount
                    continue
                end
                futureTerminalLayerIndex = ...
                    nextValidGoalArrivalLayerIndex(currentTargetLayerIndex + 1);
                if futureTerminalLayerIndex > 0 && ...
                        futureTerminalLayerIndex <= motionCandidates(motionIndex, 4)
                    motionCandidates(motionIndex, 3) = futureTerminalLayerIndex;
                    motionIsPending(motionIndex)     = true;
                end
            end
        end
    end
end

function [isClear, blockingCellIndices, collisionTimes_s] = ...
        edgeIsClear(firstNodeIndices, secondNodeIndices, departureTime_s, arrivalTime_s)
    % Check the entire travel interval against static and moving regions.
    % Equal start and end positions represent waiting at a point.
    firstNodeIndices        = firstNodeIndices(:);
    secondNodeIndices       = secondNodeIndices(:);
    departurePosition_units = nodePosition_units(firstNodeIndices, :);
    arrivalPosition_units   = nodePosition_units(secondNodeIndices, :);
    edgeCount               = numel(firstNodeIndices);
    isClear                 = true(edgeCount, 1);
    blockingCellIndices     = zeros(edgeCount, 1, "uint32");
    collisionTimes_s        = NaN(edgeCount, 1);

    % Check only the part of travel during which a stationary obstacle
    % exists. For a wait, the same check tests one fixed position.
    % Cached values: 0 = unchecked, 1 = clear, 2 = blocked.
    cacheKeys = firstNodeIndices + nodeCount * (secondNodeIndices - 1);
    for staticGeometryIndex = find(staticShapeIsPrepared).'
        activeTimeIntervals_s = staticActiveIntervals_s(staticGeometryIndex, :);
        overlapStart_s        = max(departureTime_s, activeTimeIntervals_s(1));
        overlapEnd_s          = min(arrivalTime_s, activeTimeIntervals_s(2));
        if overlapStart_s > overlapEnd_s
            continue
        end
        edgeIndicesToCheck = find(isClear);
        if isempty(edgeIndicesToCheck)
            break
        end
        obstacleCoversTravelInterval = overlapStart_s <= departureTime_s && overlapEnd_s >= arrivalTime_s;
        if obstacleCoversTravelInterval && ~isempty(staticEdgeClearanceCache)
            cachedClearanceState = staticEdgeClearanceCache(cacheKeys(edgeIndicesToCheck), staticGeometryIndex);
            isClear(edgeIndicesToCheck(cachedClearanceState == 2)) = false;
            edgeIndicesToCheck = edgeIndicesToCheck(cachedClearanceState == 0);
            if isempty(edgeIndicesToCheck)
                continue
            end
        end
        startFraction = 0;
        endFraction   = 1;
        if arrivalTime_s > departureTime_s
            edgeDuration_s = arrivalTime_s - departureTime_s;
            startFraction  = (overlapStart_s - departureTime_s) / edgeDuration_s;
            endFraction    = (overlapEnd_s - departureTime_s) / edgeDuration_s;
        end
        displacement_units   = arrivalPosition_units(edgeIndicesToCheck, :) - departurePosition_units(edgeIndicesToCheck, :);
        subStart_units       = departurePosition_units(edgeIndicesToCheck, :) + startFraction * displacement_units;
        subEnd_units         = departurePosition_units(edgeIndicesToCheck, :) + endFraction * displacement_units;
        staticSegmentIsClear = obstacleAvoidance.search.checkVisibilitySegments( ...
            subStart_units, subEnd_units, staticShapes{staticGeometryIndex}, ...
            staticEdgeStart_units{staticGeometryIndex}, staticEdgeEnd_units{staticGeometryIndex});
        isClear(edgeIndicesToCheck) = staticSegmentIsClear;
        if obstacleCoversTravelInterval && ~isempty(staticEdgeClearanceCache)
            staticEdgeClearanceCache(cacheKeys(edgeIndicesToCheck), staticGeometryIndex) = 1 + uint8(~staticSegmentIsClear);
        end
    end
    if isempty(movingObstacleCells.Regions_units) || ~any(isClear)
        return
    end
    edgeIndicesToCheck = find(isClear);
    [dynamicIsClear, dynamicBlockingCellIndices, dynamicCollisionTimes_s] = ...
        obstacleAvoidance.search.affineEdgesAreClear( ...
        firstNodeIndices(edgeIndicesToCheck), secondNodeIndices(edgeIndicesToCheck), ...
        departureTime_s, arrivalTime_s, movingCellLookup);
    isClear(edgeIndicesToCheck)             = dynamicIsClear;
    blockingCellIndices(edgeIndicesToCheck) = dynamicBlockingCellIndices;
    collisionTimes_s(edgeIndicesToCheck)    = dynamicCollisionTimes_s;
end

function blockedLayerCount = provenBlockedLayerCount( ...
        sourceNodeIndex, targetNodeIndex, sourceLayerIndex, targetLayerIndex, ...
        finalTargetLayerIndex, cellIndex, collisionTime_s)
    % Reuse a known collision to prove that later arrivals are also blocked.
    % At that same collision time, a slower trip is less far along the edge.
    % Count consecutive arrival layers whose positions still lie clearly
    % inside the same obstacle region. If the check is uncertain, count only
    % the current layer and let the search check the next one normally.
    blockedLayerCount = 1;
    if cellIndex == 0 || ~isfinite(collisionTime_s)
        return
    end
    cellIndex = double(cellIndex);
    if any(~isfinite([movingCellLookup.CellLower_units(cellIndex, :), ...
            movingCellLookup.CellUpper_units(cellIndex, :)]))
        return
    end

    sourceTime_s   = layerTimes_s(sourceLayerIndex);
    arrivalTimes_s = layerTimes_s(targetLayerIndex:finalTargetLayerIndex);
    if collisionTime_s < sourceTime_s || collisionTime_s > arrivalTimes_s(1)
        return
    end

    activeTimeIntervals_s = movingObstacleCells.ActiveTimeInterval_s(cellIndex, :);
    cellDuration_s        = diff(activeTimeIntervals_s);
    if ~(cellDuration_s > 0) || collisionTime_s < activeTimeIntervals_s(1) || collisionTime_s > activeTimeIntervals_s(2)
        return
    end
    cellTimeFraction  = (collisionTime_s - activeTimeIntervals_s(1)) / cellDuration_s;
    regionStart_units = movingObstacleCells.Regions_units{cellIndex};
    region_units      = regionStart_units + cellTimeFraction .* ...
        (movingObstacleCells.EndRegions_units{cellIndex} - regionStart_units);
    vertexCount            = size(region_units, 1);
    nextVertexIndices      = [2:vertexCount, 1];
    regionEdgeVector_units = region_units(nextVertexIndices, :) - region_units;
    signedArea_units2      = sum( ...
        region_units(:, 1) .* region_units(nextVertexIndices, 2) - ...
        region_units(:, 2) .* region_units(nextVertexIndices, 1)) / 2;

    source_units      = nodePosition_units(sourceNodeIndex, :);
    target_units      = nodePosition_units(targetNodeIndex, :);
    collisionOffset_s = collisionTime_s - sourceTime_s;
    firstPathFraction = collisionOffset_s / (arrivalTimes_s(1) - sourceTime_s);
    if ~isfinite(firstPathFraction)
        return
    end
    % Along the travel edge, each polygon-edge cross product changes
    % linearly: edge side = side at source + path fraction x side change.
    displacement_units      = target_units - source_units;
    relativeSourceX_units   = source_units(1) - region_units(:, 1);
    relativeSourceY_units   = source_units(2) - region_units(:, 2);
    edgeSideAtSource_units2 = regionEdgeVector_units(:, 1) .* relativeSourceY_units - ...
        regionEdgeVector_units(:, 2) .* relativeSourceX_units;
    edgeSideChange_units2 = regionEdgeVector_units(:, 1) .* displacement_units(2) - ...
        regionEdgeVector_units(:, 2) .* displacement_units(1);
    coordinateScale_units = max([1; abs(region_units(:)); ...
        abs(source_units(:)); abs(target_units(:))]);
    interiorCheckTolerance_units2 = 16 * obstacleAvoidance.search.createResidualBound(coordinateScale_units);
    if abs(signedArea_units2) <= interiorCheckTolerance_units2
        return
    end

    % Multiply cross products by the polygon direction so positive means
    % inside each edge. Require more than the numerical tolerance: touching
    % or an uncertain sign must not justify skipping a later collision check.
    orientationSign           = sign(signedArea_units2);
    inwardSideAtSource_units2 = orientationSign * edgeSideAtSource_units2;
    inwardSideChange_units2   = orientationSign * edgeSideChange_units2;
    firstInwardSide_units2    = inwardSideAtSource_units2 + ...
        firstPathFraction .* inwardSideChange_units2;
    if any(firstInwardSide_units2 <= interiorCheckTolerance_units2)
        return
    end

    % Find the smallest progress fraction still inside every region edge.
    % It sets the latest arrival that this known collision can rule out.
    edgeCanBecomeClear = inwardSideChange_units2 > 0;
    if ~any(edgeCanBecomeClear)
        blockedLayerCount = numel(arrivalTimes_s);
        return
    end
    minimumBlockedPathFraction = max( ...
        (interiorCheckTolerance_units2 - inwardSideAtSource_units2(edgeCanBecomeClear)) ./ ...
        inwardSideChange_units2(edgeCanBecomeClear));
    if minimumBlockedPathFraction <= 0 || collisionOffset_s == 0
        blockedLayerCount = numel(arrivalTimes_s);
        return
    end
    firstUnprovenArrival_s   = sourceTime_s + collisionOffset_s / minimumBlockedPathFraction;
    lastBlockedArrivalOffset = min(numel(arrivalTimes_s), ...
        obstacleAvoidance.search.sortedUpperBound(arrivalTimes_s, firstUnprovenArrival_s));
    lastBlockedArrivalOffset = max(1, lastBlockedArrivalOffset);
    % Recheck the final candidate directly so roundoff at the threshold
    % cannot include an arrival that is only on the boundary.
    while lastBlockedArrivalOffset > 1
        trialPathFraction = collisionOffset_s / ...
            (arrivalTimes_s(lastBlockedArrivalOffset) - sourceTime_s);
        trialInwardSide_units2 = inwardSideAtSource_units2 + ...
            trialPathFraction .* inwardSideChange_units2;
        if all(trialInwardSide_units2 > interiorCheckTolerance_units2)
            break
        end
        lastBlockedArrivalOffset = lastBlockedArrivalOffset - 1;
    end
    blockedLayerCount = lastBlockedArrivalOffset;
end
end

%% Section 7: Local Functions

function [motionCandidates, rejectedCount] = buildLayerCandidates( ...
        sourceNodeIndices, sourceLayerIndex, timedConnectionData)
    % List possible moves in the same layer/node order for every batch.
    % Batching limits temporary memory; the speed limit still determines the
    % earliest time each connection could finish.
    % Columns: source node, target node, first arrival layer to try, last
    % layer in that target wait window, and physical connection length.
    motionCandidates = zeros(0, 5);
    rejectedCount    = 0;
    if isempty(sourceNodeIndices)
        return
    end

    layerCount  = numel(timedConnectionData.LayerTimes_s);
    nodeCount   = size(timedConnectionData.NodeIsFree, 2);
    sourceCount = numel(sourceNodeIndices);

    maximumCandidateArrayElements = 1024 ^ 2;
    sourceBatchSize               = max(1, ...
        floor(maximumCandidateArrayElements / max(1, layerCount * nodeCount)));
    batchCount      = ceil(sourceCount / sourceBatchSize);
    candidateBlocks = cell(batchCount, 1);
    for batchIndex = 1:batchCount
        firstSourceOffset      = 1 + (batchIndex - 1) * sourceBatchSize;
        finalSourceOffset      = min(sourceCount, batchIndex * sourceBatchSize);
        batchSourceNodeIndices = sourceNodeIndices(firstSourceOffset:finalSourceOffset);
        [candidateBlocks{batchIndex}, batchRejectedCount] = buildCandidateBatch( ...
            batchSourceNodeIndices, sourceLayerIndex, timedConnectionData);
        rejectedCount = rejectedCount + batchRejectedCount;
    end
    motionCandidates = vertcat(candidateBlocks{:});
end

function [motionCandidates, rejectedCount] = buildCandidateBatch( ...
        sourceNodeIndices, sourceLayerIndex, timedConnectionData)
    % Candidate arrays use [arrival layer, target node, source node].
    % Check a bounded group of sources while keeping the same result order.
    layerTimes_s            = timedConnectionData.LayerTimes_s;
    nodeIsFree              = timedConnectionData.NodeIsFree;
    waitWindowStartsHere    = timedConnectionData.IsWaitComponentStart;
    waitWindowEndLayerIndex = timedConnectionData.WaitComponentFinalLayerIndex;
    motionEdgeExists        = timedConnectionData.MotionEdgeExists;
    minimumEdgeDuration_s   = timedConnectionData.MinimumEdgeDuration_s;
    motionEdgeLengths_units = timedConnectionData.MotionEdgeLengths_units;

    sourceTime_s = layerTimes_s(sourceLayerIndex);
    layerCount   = numel(layerTimes_s);
    nodeCount    = size(nodeIsFree, 2);
    sourceCount  = numel(sourceNodeIndices);

    earliestEdgeArrivalTimes_s      = sourceTime_s + minimumEdgeDuration_s(sourceNodeIndices, :).' - 1e-12;
    firstAllowedArrivalLayerIndices = 1 + sum( ...
        reshape(layerTimes_s, [], 1, 1) <= ...
        reshape(earliestEdgeArrivalTimes_s, 1, nodeCount, sourceCount), 1);
    firstAllowedArrivalLayerIndices(isnan(earliestEdgeArrivalTimes_s)) = layerCount + 1;
    % Every move to a different node takes positive time. Require a later
    % destination layer even when the computed minimum time is extremely small.
    firstAllowedArrivalLayerIndices = max(firstAllowedArrivalLayerIndices, sourceLayerIndex + 1);
    % Try the first speed-allowed layer and the start of each later wait
    % window. If that arrival is blocked, the search advances within the
    % window; after a clear arrival it can wait there instead.
    candidateCanStart = reshape(waitWindowStartsHere, layerCount, nodeCount, 1) & ...
        (reshape((1:layerCount).', [], 1, 1) >= firstAllowedArrivalLayerIndices);
    feasiblePairIndices = find(firstAllowedArrivalLayerIndices <= layerCount);
    firstEntryIndices   = firstAllowedArrivalLayerIndices(feasiblePairIndices) + ...
        layerCount * (feasiblePairIndices - 1);
    targetNodeIndices = 1 + mod(feasiblePairIndices - 1, nodeCount);
    % The occupancy array has layers as rows and nodes as columns. With
    % 4 layers, node 3 at layer 2 is element 2 + 4 x (3 - 1) = 10.
    firstStateIndices = firstAllowedArrivalLayerIndices(feasiblePairIndices) + ...
        layerCount * (targetNodeIndices - 1);
    candidateCanStart(firstEntryIndices) = nodeIsFree(firstStateIndices);
    edgeIsEnabled = motionEdgeExists(sourceNodeIndices, :).';
    candidateCanStart = candidateCanStart & ...
        reshape(edgeIsEnabled, 1, nodeCount, sourceCount);
    rejectedCount = nnz( ...
        edgeIsEnabled & ~reshape(any(candidateCanStart, 1), nodeCount, sourceCount));

    % List arrival layers first within each target/source pair. This matches
    % looping over source nodes, then target nodes, then arrival layers.
    [entryLayerIndices, targetNodeIndices, sourceOffsetIndices] = ind2sub( ...
        [layerCount, nodeCount, sourceCount], find(candidateCanStart));
    selectedSourceNodeIndices = reshape(sourceNodeIndices(sourceOffsetIndices), [], 1);
    finalStateIndices         = entryLayerIndices + layerCount * (targetNodeIndices - 1);
    finalLayerIndices         = double(waitWindowEndLayerIndex(finalStateIndices));
    edgeIndices               = selectedSourceNodeIndices + nodeCount * (targetNodeIndices - 1);
    motionCandidates          = [selectedSourceNodeIndices, targetNodeIndices, ...
        entryLayerIndices, finalLayerIndices, ...
        motionEdgeLengths_units(edgeIndices)];
end

function [nodeIsReachable, routeLengthToNode_units, parentLayerIndex, parentNodeIndex] = ...
        keepShorterRouteAtNodeAndTime(nodeIsReachable, routeLengthToNode_units, parentLayerIndex, ...
        parentNodeIndex, sourceLayerIndex, sourceNodeIndex, targetLayerIndex, ...
        targetNodeIndex, edgeLength_units)
    % Compare routes reaching the same node at the same time. Keep the
    % shorter route; improvements within 1e-12 keep the first stored route.
    % Save its preceding node/layer so reconstruction can follow it backward.
    trialRouteLength_units = routeLengthToNode_units(sourceLayerIndex, sourceNodeIndex) + edgeLength_units;
    knownRouteLength_units = routeLengthToNode_units(targetLayerIndex, targetNodeIndex);
    if trialRouteLength_units >= knownRouteLength_units - 1e-12
        return
    end

    nodeIsReachable(targetLayerIndex, targetNodeIndex)         = true;
    routeLengthToNode_units(targetLayerIndex, targetNodeIndex) = trialRouteLength_units;
    parentLayerIndex(targetLayerIndex, targetNodeIndex)        = uint32(sourceLayerIndex);
    parentNodeIndex(targetLayerIndex, targetNodeIndex)         = uint32(sourceNodeIndex);
end

function [route_units, routeTime_s] = reconstructTimedRoute( ...
        nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, ...
        goalLayerIndex, goalNodeIndex)
    % Follow the saved preceding node/layer from goal back to start.
    % A wait repeats a position at a later time and must remain in the route.
    route_units = zeros(0, 2);
    routeTime_s = zeros(0, 1);
    if isempty(goalLayerIndex)
        return
    end
    layerPathIndices = goalLayerIndex;
    nodePathIndices  = goalNodeIndex;
    while ~(layerPathIndices(1) == 1 && nodePathIndices(1) == 1)
        priorLayerIndex = double(parentLayerIndex(layerPathIndices(1), nodePathIndices(1)));
        priorNodeIndex  = double(parentNodeIndex(layerPathIndices(1), nodePathIndices(1)));
        % Index 0 means no preceding state was saved. If this happens before
        % reaching the start, return no route rather than an incomplete route.
        if priorLayerIndex == 0 || priorNodeIndex == 0
            route_units = zeros(0, 2);
            routeTime_s = zeros(0, 1);
            return
        end
        layerPathIndices = [priorLayerIndex; layerPathIndices]; %#ok<AGROW>
        nodePathIndices  = [priorNodeIndex; nodePathIndices]; %#ok<AGROW>
    end
    route_units = nodePosition_units(nodePathIndices, :);
    routeTime_s = layerTimes_s(layerPathIndices);
end

function [enclosureShapes, enclosureEdgeStart_units, enclosureEdgeEnd_units, activeTimeIntervals_s, remainingCells] = ...
        extractStationaryEnclosures(obstacles, remainingCells)
    % A moving-cell enclosure stays fixed throughout its active interval.
    % Check its whole saved shape as stationary geometry during that time,
    % and remove those cells from the separate moving-region checks.
    enclosureShapes          = cell(0, 1);
    enclosureEdgeStart_units = cell(0, 1);
    enclosureEdgeEnd_units   = cell(0, 1);
    activeTimeIntervals_s    = zeros(0, 2);
    cellIsRetained           = true(numel(remainingCells.Regions_units), 1);

    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        preparation = obstacle.InternalPreparation;
        movingCellIntervalIndices = find(preparation.IntervalUsesMovingCells);
        for intervalIndex = reshape(movingCellIntervalIndices, 1, [])
            interval_s     = obstacle.time_s(intervalIndex:intervalIndex + 1).';
            cellIsSelected = remainingCells.SourceObstacleIndex == obstacleIndex & ...
                remainingCells.ActiveTimeInterval_s(:, 1) >= interval_s(1) & ...
                remainingCells.ActiveTimeInterval_s(:, 2) <= interval_s(2);
            if ~any(cellIsSelected)
                continue
            end

            firstSelectedCellIndex               = find(cellIsSelected, 1);
            enclosureShapes{end + 1, 1}          = preparation.IntervalUnionShapes{intervalIndex}; %#ok<AGROW>
            enclosureEdgeStart_units{end + 1, 1} = preparation.IntervalUnionEdgeStart_units{intervalIndex}; %#ok<AGROW>
            enclosureEdgeEnd_units{end + 1, 1}   = preparation.IntervalUnionEdgeEnd_units{intervalIndex}; %#ok<AGROW>
            activeTimeIntervals_s(end + 1, :)    = remainingCells.ActiveTimeInterval_s(firstSelectedCellIndex, :); %#ok<AGROW>
            cellIsRetained(cellIsSelected)       = false;
        end
    end

    remainingCells.Regions_units        = remainingCells.Regions_units(cellIsRetained);
    remainingCells.EndRegions_units     = remainingCells.EndRegions_units(cellIsRetained);
    remainingCells.ActiveTimeInterval_s = remainingCells.ActiveTimeInterval_s(cellIsRetained, :);
    remainingCells.SourceObstacleIndex  = remainingCells.SourceObstacleIndex(cellIsRetained);
end
