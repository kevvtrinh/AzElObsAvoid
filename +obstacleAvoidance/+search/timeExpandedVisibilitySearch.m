function [route_units, routeTime_s, record] = timeExpandedVisibilitySearch( ...
    nodePosition_units, edgeCost_units, obstacles, initialState, goalState, ...
    limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_units, routeTime_s, record] = ...
%       obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
%       nodePosition_units, edgeCost_units, obstacles, initialState, ...
%       goalState, limits, sampleTimes_s, options)
%**************************************************************************
% PURPOSE
%   - Search timed reachability using waits and moving edges.
%   - Check static and affine moving geometry over complete active intervals.
%**************************************************************************
% INPUTS
%   - nodePosition_units (N-by-2 numeric array)
%       Search nodes with the start first and goal second.
%   - edgeCost_units (N-by-N numeric array)
%       Finite entries enable motion edges.
%   - obstacles (canonical protected obstacle array)
%       Static and moving geometry.
%   - initialState (scalar struct)
%       Normalized initial endpoint state.
%   - goalState (scalar struct)
%       Normalized goal endpoint state.
%   - limits (scalar struct)
%       Normalized workspace and derivative limits.
%   - sampleTimes_s (numeric vector)
%       Candidate temporal layers.
%   - options (scalar struct)
%       Normalized timed-search options.
%**************************************************************************
% OUTPUTS
%   - route_units (N-by-2 numeric array)
%       Selected timed route, or an empty array on search exhaustion.
%   - routeTime_s (N-by-1 numeric array)
%       Route knot times, or an empty array on search exhaustion.
%   - record (scalar struct)
%       Search counts, selected goal window, and wait-seed route. Exhaustion
%       is an ordinary planning outcome; invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position and edge cost are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Propagate The Reachability Frontier

layerTimes_s = unique([initialState.time_s; sampleTimes_s(:); goalState.time_s]);
layerTimes_s = layerTimes_s(layerTimes_s >= initialState.time_s & layerTimes_s <= goalState.time_s);
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
displacement_units       = abs(goalPosition_units - initialPosition_units);
minimumDuration_s        = max(displacement_units ./ limits.maxVelocity_units_s);
minimumGoalArrivalTime_s = initialState.time_s + minimumDuration_s;

stateFields            = {'position_units', 'velocity_units_s', 'acceleration_units_s2'};
hasEndpointDerivatives = all(isfield(initialState, stateFields)) && all(isfield(goalState, stateFields));
hasDerivativeLimits    = all(isfield(limits, {'maxAcceleration_units_s2', 'maxJerk_units_s3'}));
if hasEndpointDerivatives && hasDerivativeLimits
    minimumDuration_s = obstacleAvoidance.input.minimumTravelTime(initialState, goalState, limits);
    minimumGoalArrivalTime_s = initialState.time_s + minimumDuration_s;
end
timeTolerance_s     = 256 * eps(max(1, max(abs(layerTimes_s))));
goalLayerIsEligible = true(layerCount, 1);
if options.GoalTimeMode == "earliestArrival"
    goalLayerIsEligible = layerTimes_s >= minimumGoalArrivalTime_s - timeTolerance_s;
end
% The local obstacle snapshot stays unchanged throughout this search.
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles, [initialState.time_s, goalState.time_s]);
% Keep fewer cached boundaries for larger boundaries or more obstacles.
for obstacleIndex = 1:numel(obstacles)
    obstacles(obstacleIndex).InternalPreparation.QueryGeometryCache = containers.Map( ...
        'KeyType', 'double', 'ValueType', 'any');
    boundarySize  = max([1; cellfun(@numel, obstacles(obstacleIndex).x_units(:))]);
    cacheCapacity = floor(2^14 / max(1, numel(obstacles)) / boundarySize);
    obstacles(obstacleIndex).InternalPreparation.QueryGeometryCacheCapacity = cacheCapacity;
end
maximumCacheBytes = 300 * 1024 ^ 2;
% Time-invariant obstacles are checked exactly, one obstacle at a time, over
% the part of each edge's clock during which that obstacle exists. The
% prepared sample geometry is the protected boundary itself, so no snapshot or
% closure tolerance is introduced.
isStatic = arrayfun(@(obstacle) obstacle.InternalPreparation.SamplesExactlyEqual, obstacles);
dynamicObstacles = obstacles(~isStatic);
staticObstacles  = obstacles(isStatic);
staticCount      = numel(staticObstacles);
dynamicCells          = obstacleAvoidance.obstacles.createTimeCells( ...
    dynamicObstacles, initialState.time_s, goalState.time_s);
staticShapes          = cell(staticCount, 1);
staticEdgeStart_units = cell(staticCount, 1);
staticEdgeEnd_units   = cell(staticCount, 1);
staticActive_s        = repmat([-Inf, Inf], staticCount, 1);
staticExists          = false(staticCount, 1);
for staticObstacleIndex = 1:staticCount
    obstacle            = staticObstacles(staticObstacleIndex);
    preparation         = obstacle.InternalPreparation;
    preparedSampleIndex = find(preparation.SamplePrepared, 1, "first");
    if isempty(preparedSampleIndex)
        continue;
    end
    staticExists(staticObstacleIndex)          = true;
    staticShapes{staticObstacleIndex}          = preparation.SampleShapes{preparedSampleIndex};
    staticEdgeStart_units{staticObstacleIndex} = preparation.SampleEdgeStart_units{preparedSampleIndex};
    staticEdgeEnd_units{staticObstacleIndex}   = preparation.SampleEdgeEnd_units{preparedSampleIndex};
    if numel(obstacle.time_s) > 1
        staticActive_s(staticObstacleIndex, :) = [obstacle.time_s(1), obstacle.time_s(end)];
    end
end
% Swept corresponding intervals are stationary geometry on their own
% absolute lifetimes. Check their cached union boundary as one exact region.
[stationaryShapes, stationaryStarts_units, stationaryEnds_units, stationaryActive_s, dynamicCells] = ...
    stationaryGeometryCells(dynamicObstacles, dynamicCells);
staticShapes          = [staticShapes; stationaryShapes];
staticEdgeStart_units = [staticEdgeStart_units; stationaryStarts_units];
staticEdgeEnd_units   = [staticEdgeEnd_units; stationaryEnds_units];
staticActive_s        = [staticActive_s; stationaryActive_s];
staticExists          = [staticExists; true(numel(stationaryShapes), 1)];
staticCount           = numel(staticShapes);
% Cache only clock-independent answers: edges whose whole clock lies inside
% the obstacle's lifetime. Partially covered edges are checked directly.
staticEdgeCache = zeros(0, 0, 'uint8');
if staticCount > 0 && nodeCount^2 * staticCount <= maximumCacheBytes
    staticEdgeCache = zeros(nodeCount^2, staticCount, 'uint8');
end
nodeIsFree = false(layerCount, nodeCount);
for layerIndex = 1:layerCount
    nodeIsFree(layerIndex, :) = ~obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
        obstacles, nodePosition_units(:, 1), nodePosition_units(:, 2), layerTimes_s(layerIndex), false).';
end
waitIsClear = false(max(0, layerCount - 1), nodeCount);
for layerIndex = 1:layerCount - 1
    candidateNodeIndices = find(nodeIsFree(layerIndex, :) & nodeIsFree(layerIndex + 1, :));
    % Test stationary waits only at nodes free in both adjacent layers.
    if ~isempty(candidateNodeIndices)
        waitIsClear(layerIndex, candidateNodeIndices) = edgeIsClear( ...
            candidateNodeIndices, candidateNodeIndices, layerTimes_s(layerIndex), layerTimes_s(layerIndex + 1));
    end
end

% Within a clear-wait interval, an earlier arrival can wait to match a later one.
isWaitComponentStart = nodeIsFree;
isWaitComponentStart(2:end, :) = nodeIsFree(2:end, :) & ~waitIsClear;
waitComponentFinalLayerIndex = repmat(uint32((1:layerCount).'), 1, nodeCount);
for layerIndex = layerCount - 1:-1:1
    continuingNodeIndices = find(waitIsClear(layerIndex, :));
    waitComponentFinalLayerIndex(layerIndex, continuingNodeIndices) = ...
        waitComponentFinalLayerIndex(layerIndex + 1, continuingNodeIndices);
end
motionEdgeExists = isfinite(edgeCost_units);
motionEdgeExists(1:nodeCount + 1:end) = false;
minimumEdgeDuration_s   = zeros(nodeCount, nodeCount);
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
distanceToGoal_units = vecnorm(nodePosition_units - nodePosition_units(2, :), 2, 2);
goalCanWaitToFinal = nodeIsFree(:, 2) & double(waitComponentFinalLayerIndex(:, 2)) == layerCount;
reachable               = false(layerCount, nodeCount);
spatialCost_units       = Inf(layerCount, nodeCount);
parentLayerIndex        = zeros(layerCount, nodeCount, "uint32");
parentNodeIndex         = zeros(layerCount, nodeCount, "uint16");
reachable(1, 1)         = nodeIsFree(1, 1);
spatialCost_units(1, 1) = 0;
[rejectedCount, expandedCount] = deal(0);
goalCostBound_units = Inf;
isEarliestArrival   = options.GoalTimeMode == "earliestArrival";
for layerIndex = 1:layerCount - 1
    % Once the goal is reached, retain its complete clear wait component.
    % That gives BMTP a kinematically useful arrive-then-wait seed without
    % crossing a later interval in which the goal becomes occupied.
    firstGoalLayerIndex = find(reachable(1:layerIndex, 2) & goalLayerIsEligible(1:layerIndex), 1, "first");
    if isEarliestArrival && ~isempty(firstGoalLayerIndex)
        selectedGoalLayerIndex = double(waitComponentFinalLayerIndex(firstGoalLayerIndex, 2));
        if selectedGoalLayerIndex == layerCount
            selectedGoalLayerIndex = firstGoalLayerIndex;
        end
        if layerIndex >= selectedGoalLayerIndex
            break;
        end
    end
    currentNodeIndices = find(reachable(layerIndex, :));
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        % Add a same-node transition only when the obstacle sweep permits
        % waiting through the full layer interval.
        if waitIsClear(layerIndex, currentNodeIndex)
            [reachable, spatialCost_units, parentLayerIndex, parentNodeIndex] = updateTemporalState( ...
                reachable, spatialCost_units, parentLayerIndex, parentNodeIndex, ...
                layerIndex, currentNodeIndex, layerIndex + 1, currentNodeIndex, 0);
        else
            rejectedCount = rejectedCount + 1;
        end
    end
    goalCostBound_units = min([goalCostBound_units; spatialCost_units(goalCanWaitToFinal, 2)]);
    [motionCandidates, candidateRejectedCount] = buildLayerCandidates( ...
        currentNodeIndices, layerTimes_s(layerIndex), layerTimes_s, motionEdgeExists, ...
        minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, ...
        isWaitComponentStart, waitComponentFinalLayerIndex);
    rejectedCount        = rejectedCount + candidateRejectedCount;
    motionCandidateCount = size(motionCandidates, 1);
    motionIsPending      = true(motionCandidateCount, 1);

    % Keep the first clear entry per wait interval; later entries can be reached by waiting.
    while any(motionIsPending)
        queriedTargetLayerIndices = unique(motionCandidates(motionIsPending, 3));
        for targetLayerIndex = reshape(queriedTargetLayerIndices, 1, [])
            queryIndices = find(motionIsPending & motionCandidates(:, 3) == targetLayerIndex);
            % Fixed-arrival requests use their prescribed final layer;
            % forward search already resolved earliest-arrival selection.
            if ~isEarliestArrival
                trialCost_units = reshape( ...
                    spatialCost_units(layerIndex, motionCandidates(queryIndices, 1)), [], 1) + ...
                    motionCandidates(queryIndices, 5);
                storedCost_units = reshape( ...
                    spatialCost_units(targetLayerIndex, motionCandidates(queryIndices, 2)), [], 1);
                isDominated = trialCost_units > storedCost_units + 1e-12;
                motionIsPending(queryIndices(isDominated)) = false;
                rejectedCount = rejectedCount + nnz(isDominated);
                queryIndices = queryIndices(~isDominated);
                trialCost_units = trialCost_units(~isDominated);
                remainingDistance_units = distanceToGoal_units(motionCandidates(queryIndices, 2));
                cannotImproveGoal = trialCost_units + remainingDistance_units > goalCostBound_units + 1e-12;
                motionIsPending(queryIndices(cannotImproveGoal)) = false;
                rejectedCount = rejectedCount + nnz(cannotImproveGoal);
                queryIndices = queryIndices(~cannotImproveGoal);
            end
            if isempty(queryIndices)
                continue;
            end
            queryIsClear = edgeIsClear(motionCandidates(queryIndices, 1), motionCandidates(queryIndices, 2), ...
                layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
            clearIndices = queryIndices(queryIsClear);
            for motionIndex = reshape(clearIndices, 1, [])
                sourceNodeIndex = motionCandidates(motionIndex, 1);
                targetNodeIndex = motionCandidates(motionIndex, 2);
                candidateTargetLayerIndex = motionCandidates(motionIndex, 3);
                edgeLength_units = motionCandidates(motionIndex, 5);
                [reachable, spatialCost_units, parentLayerIndex, parentNodeIndex] = updateTemporalState( ...
                    reachable, spatialCost_units, parentLayerIndex, parentNodeIndex, ...
                    layerIndex, sourceNodeIndex, candidateTargetLayerIndex, targetNodeIndex, edgeLength_units);
            end
            reachesGoal = motionCandidates(clearIndices, 2) == 2;
            canWaitAtGoal = goalCanWaitToFinal(motionCandidates(clearIndices, 3));
            clearGoalIndices = clearIndices(reachesGoal & canWaitAtGoal);
            if ~isempty(clearGoalIndices)
                clearGoalLayerIndices = motionCandidates(clearGoalIndices, 3);
                goalCostBound_units = min( ...
                    [goalCostBound_units; spatialCost_units(clearGoalLayerIndices, 2)]);
            end
            rejectedCount = rejectedCount + nnz(~queryIsClear);
            motionIsPending(queryIndices) = false;
            advanceIndices = queryIndices( ...
                ~queryIsClear & motionCandidates(queryIndices, 3) < motionCandidates(queryIndices, 4));
            motionCandidates(advanceIndices, 3) = motionCandidates(advanceIndices, 3) + 1;
            motionIsPending(advanceIndices) = true;
        end
    end
end

%% Section 2: Reconstruct The Goal And Arrive-Then-Wait Routes

% Earliest-arrival mode stops at the first reachable goal layer. Fixed-arrival
% mode must evaluate its prescribed horizon.
if isEarliestArrival
    firstGoalLayerIndex = find(reachable(:, 2) & goalLayerIsEligible, 1, "first");
    goalLayerIndex         = firstGoalLayerIndex;
    waitSeedGoalLayerIndex = firstGoalLayerIndex;
    if ~isempty(firstGoalLayerIndex)
        waitSeedGoalLayerIndex = double(waitComponentFinalLayerIndex(firstGoalLayerIndex, 2));
        if waitSeedGoalLayerIndex == layerCount
            waitSeedGoalLayerIndex = firstGoalLayerIndex;
        end
    end
else
    goalLayerIndex = find(reachable(:, 2) & (1:layerCount).' == layerCount, 1, "first");
    waitSeedGoalLayerIndex = goalLayerIndex;
end
[route_units, routeTime_s] = reconstructTimedRoute( ...
    nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex, 2);
[waitRoute_units, waitRouteTime_s] = reconstructTimedRoute( ...
    nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, waitSeedGoalLayerIndex, 2);
selectedGoalWindowStartTime_s = NaN;
selectedGoalWindowEndTime_s   = NaN;
if ~isempty(goalLayerIndex)
    goalWindowStartLayerIndices = find(isWaitComponentStart(:, 2) & nodeIsFree(:, 2));
    selectedGoalWindowIndex           = nnz(goalWindowStartLayerIndices <= goalLayerIndex);
    selectedGoalWindowStartLayerIndex = goalWindowStartLayerIndices(selectedGoalWindowIndex);
    selectedGoalWindowEndLayerIndex   = double( ...
        waitComponentFinalLayerIndex(selectedGoalWindowStartLayerIndex, 2));
    selectedGoalWindowStartTime_s     = layerTimes_s(selectedGoalWindowStartLayerIndex);
    selectedGoalWindowEndTime_s       = layerTimes_s(selectedGoalWindowEndLayerIndex);
end
record = struct( ...
    "NodeCount",                     nodeCount, ...
    "RejectedTransitionCount",       rejectedCount, ...
    "ExpandedCount",                 expandedCount, ...
    "SelectedGoalWindowStartTime_s", selectedGoalWindowStartTime_s, ...
    "SelectedGoalWindowEndTime_s",   selectedGoalWindowEndTime_s, ...
    "MinimumGoalArrivalTime_s",      minimumGoalArrivalTime_s, ...
    "WaitRoute_units",               waitRoute_units, ...
    "WaitRouteTime_s",               waitRouteTime_s, ...
    "DynamicEdgeCheckKind",          "exactAffineConvexCells");

%% Section 3: Local Functions

function isClear = edgeIsClear(firstNodeIndices, secondNodeIndices, first_s, second_s)
    % Check the complete segment clock against exact static geometry and every
    % affine moving convex cell. A zero-length edge is a stationary point path.
    firstNodeIndices  = firstNodeIndices(:);
    secondNodeIndices = secondNodeIndices(:);
    first_units       = nodePosition_units(firstNodeIndices, :);
    second_units      = nodePosition_units(secondNodeIndices, :);
    edgeCount         = numel(firstNodeIndices);
    isClear           = true(edgeCount, 1);

    % Exact check against every time-invariant obstacle over the sub-segment
    % traversed while that obstacle exists. A zero-length wait reduces to
    % point containment inside the same predicate.
    cacheKeys = firstNodeIndices + nodeCount * (secondNodeIndices - 1);
    for staticGeometryIndex = find(staticExists).'
        active_s       = staticActive_s(staticGeometryIndex, :);
        overlapStart_s = max(first_s, active_s(1));
        overlapEnd_s   = min(second_s, active_s(2));
        if overlapStart_s > overlapEnd_s
            continue
        end
        candidateIndices = find(isClear);
        if isempty(candidateIndices)
            break
        end
        coversWholeClock = overlapStart_s <= first_s && overlapEnd_s >= second_s;
        if coversWholeClock && ~isempty(staticEdgeCache)
            known = staticEdgeCache(cacheKeys(candidateIndices), staticGeometryIndex);
            isClear(candidateIndices(known == 2)) = false;
            candidateIndices = candidateIndices(known == 0);
            if isempty(candidateIndices)
                continue
            end
        end
        startFraction = 0;
        endFraction   = 1;
        if second_s > first_s
            edgeDuration_s = second_s - first_s;
            startFraction  = (overlapStart_s - first_s) / edgeDuration_s;
            endFraction    = (overlapEnd_s - first_s) / edgeDuration_s;
        end
        displacement_units = second_units(candidateIndices, :) - first_units(candidateIndices, :);
        subStart_units      = first_units(candidateIndices, :) + startFraction * displacement_units;
        subEnd_units        = first_units(candidateIndices, :) + endFraction * displacement_units;
        exactlyClear        = obstacleAvoidance.search.checkVisibilitySegments( ...
            subStart_units, subEnd_units, staticShapes{staticGeometryIndex}, ...
            staticEdgeStart_units{staticGeometryIndex}, staticEdgeEnd_units{staticGeometryIndex});
        isClear(candidateIndices) = exactlyClear;
        if coversWholeClock && ~isempty(staticEdgeCache)
            staticEdgeCache(cacheKeys(candidateIndices), staticGeometryIndex) = 1 + uint8(~exactlyClear);
        end
    end
    if isempty(dynamicCells.Regions_units) || ~any(isClear)
        return
    end
    candidateIndices = find(isClear);
    isClear(candidateIndices) = affineEdgesAreClear( ...
        first_units(candidateIndices, :), second_units(candidateIndices, :), ...
        first_s, second_s, dynamicCells);
end
end

function [candidates, rejectedCount] = buildLayerCandidates( ...
        sourceNodeIndices, sourceTime_s, layerTimes_s, motionEdgeExists, ...
        minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, ...
        isWaitComponentStart, waitComponentFinalLayerIndex)
    % Enumerate [entry layer, target node, source node] in the original order.
    % Bound the temporary logical tensor while retaining the vectorized path
    % for ordinary inputs and the velocity-only duration lower bound.
    candidates    = zeros(0, 5);
    rejectedCount = 0;
    if isempty(sourceNodeIndices)
        return
    end

    layerCount  = numel(layerTimes_s);
    nodeCount   = size(nodeIsFree, 2);
    sourceCount = numel(sourceNodeIndices);
    maximumCandidateTensorElements = 1024 ^ 2;
    sourceBatchSize = max(1, ...
        floor(maximumCandidateTensorElements / max(1, layerCount * nodeCount)));
    batchCount      = ceil(sourceCount / sourceBatchSize);
    candidateBlocks = cell(batchCount, 1);
    for batchIndex = 1:batchCount
        firstSourceOffset = 1 + (batchIndex - 1) * sourceBatchSize;
        finalSourceOffset = min(sourceCount, batchIndex * sourceBatchSize);
        batchSourceNodeIndices = sourceNodeIndices(firstSourceOffset:finalSourceOffset);
        [candidateBlocks{batchIndex}, batchRejectedCount] = buildCandidateBatch( ...
            batchSourceNodeIndices, sourceTime_s, layerTimes_s, motionEdgeExists, ...
            minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, ...
            isWaitComponentStart, waitComponentFinalLayerIndex);
        rejectedCount = rejectedCount + batchRejectedCount;
    end
    candidates = vertcat(candidateBlocks{:});
end

function [candidates, rejectedCount] = buildCandidateBatch( ...
        sourceNodeIndices, sourceTime_s, layerTimes_s, motionEdgeExists, ...
        minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, ...
        isWaitComponentStart, waitComponentFinalLayerIndex)
    % Vectorize one bounded block and preserve source/target/layer ordering.
    layerCount  = numel(layerTimes_s);
    nodeCount   = size(nodeIsFree, 2);
    sourceCount = numel(sourceNodeIndices);
    earliestTime_s = sourceTime_s + minimumEdgeDuration_s(sourceNodeIndices, :).' - 1e-12;
    firstFeasibleLayerIndices = 1 + sum( ...
        reshape(layerTimes_s, [], 1, 1) <= ...
        reshape(earliestTime_s, 1, nodeCount, sourceCount), 1);
    firstFeasibleLayerIndices(isnan(earliestTime_s)) = layerCount + 1;
    candidateCanStart = reshape(isWaitComponentStart, layerCount, nodeCount, 1) & ...
        (reshape((1:layerCount).', [], 1, 1) >= firstFeasibleLayerIndices);
    feasiblePairIndices = find(firstFeasibleLayerIndices <= layerCount);
    firstEntryIndices = firstFeasibleLayerIndices(feasiblePairIndices) + ...
        layerCount * (feasiblePairIndices - 1);
    targetNodeIndices = 1 + mod(feasiblePairIndices - 1, nodeCount);
    firstStateIndices = firstFeasibleLayerIndices(feasiblePairIndices) + ...
        layerCount * (targetNodeIndices - 1);
    candidateCanStart(firstEntryIndices) = nodeIsFree(firstStateIndices);
    edgeIsEnabled = motionEdgeExists(sourceNodeIndices, :).';
    candidateCanStart = candidateCanStart & ...
        reshape(edgeIsEnabled, 1, nodeCount, sourceCount);
    rejectedCount = nnz( ...
        edgeIsEnabled & ~reshape(any(candidateCanStart, 1), nodeCount, sourceCount));

    % Column-major enumeration matches the original source/target/layer loops.
    [entryLayerIndices, targetNodeIndices, sourceOffsetIndices] = ind2sub( ...
        [layerCount, nodeCount, sourceCount], find(candidateCanStart));
    selectedSourceNodeIndices = reshape(sourceNodeIndices(sourceOffsetIndices), [], 1);
    finalStateIndices = entryLayerIndices + layerCount * (targetNodeIndices - 1);
    finalLayerIndices = double(waitComponentFinalLayerIndex(finalStateIndices));
    edgeIndices = selectedSourceNodeIndices + nodeCount * (targetNodeIndices - 1);
    candidates = [selectedSourceNodeIndices, targetNodeIndices, ...
        entryLayerIndices, finalLayerIndices, ...
        motionEdgeLengths_units(edgeIndices)];
end

function isClear = affineEdgesAreClear(first_units, second_units, first_s, second_s, cells)
    % A path point and every vertex of a time cell are affine in time. Each
    % convex half-space residual is therefore quadratic; its real roots
    % partition the clock into intervals of constant inside/outside sign.
    isClear = true(size(first_units, 1), 1);
    for cellIndex = 1:numel(cells.Regions_units)
        overlapStart_s = max(first_s, cells.ActiveTimeInterval_s(cellIndex, 1));
        overlapEnd_s   = min(second_s, cells.ActiveTimeInterval_s(cellIndex, 2));
        if overlapStart_s > overlapEnd_s
            continue
        end

        candidateIndices = find(isClear);
        if isempty(candidateIndices)
            break
        end

        edgeDuration_s = second_s - first_s;
        if edgeDuration_s > 0
            startFraction = (overlapStart_s - first_s) / edgeDuration_s;
            endFraction   = (overlapEnd_s - first_s) / edgeDuration_s;
        else
            startFraction = 0;
            endFraction   = 0;
        end

        pathStart_units = first_units(candidateIndices, :) + startFraction .* ...
            (second_units(candidateIndices, :) - first_units(candidateIndices, :));
        pathEnd_units = first_units(candidateIndices, :) + endFraction .* ...
            (second_units(candidateIndices, :) - first_units(candidateIndices, :));

        cellDuration_s    = diff(cells.ActiveTimeInterval_s(cellIndex, :));
        cellStartFraction = (overlapStart_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
        cellEndFraction   = (overlapEnd_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
        regionStart_units        = cells.Regions_units{cellIndex};
        regionDelta_units        = cells.EndRegions_units{cellIndex} - regionStart_units;
        overlapRegionStart_units = regionStart_units + cellStartFraction .* regionDelta_units;
        overlapRegionEnd_units   = regionStart_units + cellEndFraction .* regionDelta_units;
        pointTouchesCell = affinePointsTouchConvex( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, overlapRegionEnd_units);
        isClear(candidateIndices(pointTouchesCell)) = false;
    end
end

function pointTouchesCell = affinePointsTouchConvex( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units)
    % Test path points against one affine moving convex cell in one batch.
    % Residual roots, clock ends, and interval midpoints form exact sign probes.
    % Rows are cell edges and columns are path points, preserving the original
    % point-by-point arithmetic and residual tolerance.
    pointCount       = size(pointStart_units, 1);
    pointTouchesCell = false(pointCount, 1);
    if pointCount == 0
        return
    end

    vertexCount       = size(regionStart_units, 1);
    following         = [2:vertexCount, 1];
    edgeStart_units   = regionStart_units(following, :) - regionStart_units;
    edgeDelta_units   = (regionEnd_units(following, :) - regionEnd_units) - edgeStart_units;
    regionDelta_units = regionEnd_units - regionStart_units;
    pointDelta_units  = pointEnd_units - pointStart_units;

    relativeStartX_units = pointStart_units(:, 1).' - regionStart_units(:, 1);
    relativeStartY_units = pointStart_units(:, 2).' - regionStart_units(:, 2);
    relativeDeltaX_units = pointDelta_units(:, 1).' - regionDelta_units(:, 1);
    relativeDeltaY_units = pointDelta_units(:, 2).' - regionDelta_units(:, 2);
    quadratic_units2 = edgeDelta_units(:, 1) .* relativeDeltaY_units - ...
        edgeDelta_units(:, 2) .* relativeDeltaX_units;
    linear_units2 = (edgeDelta_units(:, 1) .* relativeStartY_units - ...
        edgeDelta_units(:, 2) .* relativeStartX_units) + ...
        (edgeStart_units(:, 1) .* relativeDeltaY_units - ...
        edgeStart_units(:, 2) .* relativeDeltaX_units);
    constant_units2 = edgeStart_units(:, 1) .* relativeStartY_units - ...
        edgeStart_units(:, 2) .* relativeStartX_units;

    % Real roots of each residual inside the unit clock; NaN marks none.
    rootScale         = max(1, max(max(abs(quadratic_units2), abs(linear_units2)), abs(constant_units2)));
    rootTolerance     = 256 * eps(rootScale);
    isLinear          = abs(quadratic_units2) <= rootTolerance;
    discriminant      = linear_units2 .* linear_units2 - 4 * quadratic_units2 .* constant_units2;
    hasQuadraticRoots = ~isLinear & discriminant >= -rootTolerance;
    rootRadius        = sqrt(max(0, discriminant));
    firstRoot         = (-linear_units2 - rootRadius) ./ (2 * quadratic_units2);
    secondRoot        = (-linear_units2 + rootRadius) ./ (2 * quadratic_units2);
    linearRoot        = -constant_units2 ./ linear_units2;
    hasLinearRoot     = isLinear & abs(linear_units2) > rootTolerance;

    firstRoot(isLinear)                              = linearRoot(isLinear);
    firstRoot(~hasQuadraticRoots & ~hasLinearRoot)  = NaN;
    secondRoot(~hasQuadraticRoots)                  = NaN;
    firstRoot(~(firstRoot >= 0 & firstRoot <= 1))   = NaN;
    secondRoot(~(secondRoot >= 0 & secondRoot <= 1)) = NaN;
    cuts = sort([zeros(pointCount, 1), ones(pointCount, 1), firstRoot.', secondRoot.'], 2);
    probes     = [cuts, (cuts(:, 1:end - 1) + cuts(:, 2:end)) / 2];
    probeClock = reshape(probes, 1, pointCount, []);
    residual_units2 = quadratic_units2 .* probeClock.^2 + ...
        linear_units2 .* probeClock + constant_units2;

    middleRegion_units = (regionStart_units + regionEnd_units) / 2;
    signedArea_units2 = sum( ...
        middleRegion_units(:, 1) .* middleRegion_units(following, 2) - ...
        middleRegion_units(:, 2) .* middleRegion_units(following, 1)) / 2;
    regionScale_units     = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units      = max(max(abs(pointStart_units), [], 2), max(abs(pointEnd_units), [], 2));
    coordinateScale_units = max(1, max(regionScale_units, pointScale_units));
    residualTolerance_units2 = reshape(4096 * eps(coordinateScale_units.^2), 1, pointCount);
    if signedArea_units2 >= 0
        inside = residual_units2 >= -residualTolerance_units2;
    else
        inside = residual_units2 <= residualTolerance_units2;
    end
    pointTouchesCell = reshape(any(all(inside, 1), 3), pointCount, 1);
end
function [reachable, spatialCost_units, parentLayerIndex, parentNodeIndex] = ...
        updateTemporalState(reachable, spatialCost_units, parentLayerIndex, ...
        parentNodeIndex, sourceLayerIndex, sourceNodeIndex, targetLayerIndex, ...
        targetNodeIndex, edgeLength_units)
    % Arrival layer is fixed by the state. Retain only the shortest spatial
    % ancestry, with deterministic first-discovered selection on exact ties.
    trialCost_units  = spatialCost_units(sourceLayerIndex, sourceNodeIndex) + edgeLength_units;
    storedCost_units = spatialCost_units(targetLayerIndex, targetNodeIndex);
    if trialCost_units >= storedCost_units - 1e-12
        return
    end

    reachable(targetLayerIndex, targetNodeIndex)        = true;
    spatialCost_units(targetLayerIndex, targetNodeIndex) = trialCost_units;
    parentLayerIndex(targetLayerIndex, targetNodeIndex) = uint32(sourceLayerIndex);
    parentNodeIndex(targetLayerIndex, targetNodeIndex)   = uint16(sourceNodeIndex);
end

function [route_units, routeTime_s] = reconstructTimedRoute( ...
        nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, ...
        goalLayerIndex, goalNodeIndex)
    % Follow temporal parents backward, including waits.
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
        % Stop at the start sentinel; an early sentinel means corrupt parents.
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

function [shapes, starts_units, ends_units, active_s, cells] = ...
        stationaryGeometryCells(obstacles, cells)
    % Move exact swept-union intervals into the stationary geometry arrays.
    shapes         = cell(0, 1);
    starts_units   = cell(0, 1);
    ends_units     = cell(0, 1);
    active_s       = zeros(0, 2);
    cellIsRetained = true(numel(cells.Regions_units), 1);

    for obstacleIndex = 1:numel(obstacles)
        obstacle    = obstacles(obstacleIndex);
        preparation = obstacle.InternalPreparation;
        sweptIntervalIndices = find( ...
            preparation.IntervalGeometryModel == "sweptCorrespondingConvexCells");
        for intervalIndex = reshape(sweptIntervalIndices, 1, [])
            interval_s = obstacle.time_s(intervalIndex:intervalIndex + 1).';
            cellIsSelected = cells.SourceObstacleIndex == obstacleIndex & ...
                cells.ActiveTimeInterval_s(:, 1) >= interval_s(1) & ...
                cells.ActiveTimeInterval_s(:, 2) <= interval_s(2);
            if ~any(cellIsSelected)
                continue
            end

            firstSelectedCellIndex = find(cellIsSelected, 1);
            shapes{end + 1, 1}       = preparation.IntervalUnionShapes{intervalIndex}; %#ok<AGROW>
            starts_units{end + 1, 1} = preparation.IntervalUnionEdgeStart_units{intervalIndex}; %#ok<AGROW>
            ends_units{end + 1, 1}   = preparation.IntervalUnionEdgeEnd_units{intervalIndex}; %#ok<AGROW>
            active_s(end + 1, :)      = cells.ActiveTimeInterval_s(firstSelectedCellIndex, :); %#ok<AGROW>
            cellIsRetained(cellIsSelected) = false;
        end
    end

    cells.Regions_units        = cells.Regions_units(cellIsRetained);
    cells.EndRegions_units     = cells.EndRegions_units(cellIsRetained);
    cells.ActiveTimeInterval_s = cells.ActiveTimeInterval_s(cellIsRetained, :);
    cells.SourceObstacleIndex  = cells.SourceObstacleIndex(cellIsRetained);
end
