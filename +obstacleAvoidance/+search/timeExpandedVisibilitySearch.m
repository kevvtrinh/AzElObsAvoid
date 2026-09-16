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
% Cache a conservative space-time broad phase for the fixed node segments.
% The boxes contain every point accepted by the exact residual predicate,
% including its tolerance near sharp corners. A cache miss therefore removes
% work only; uncertain and degenerating cells remain on the exact path.
nodeScale_units = max([1; abs(nodePosition_units(:))]);
[dynamicCellLower_units, dynamicCellUpper_units, dynamicCellIsCounterclockwise] = ...
    createCellBoxes(dynamicCells, nodeScale_units);
dynamicPairCache = createPairCellCache( ...
    nodePosition_units, dynamicCellLower_units, dynamicCellUpper_units, ...
    dynamicCells.ActiveTimeInterval_s);
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
parentNodeIndex         = zeros(layerCount, nodeCount, "uint32");
reachable(1, 1)         = nodeIsFree(1, 1);
spatialCost_units(1, 1) = 0;
[rejectedCount, expandedCount, certifiedSkipCount] = deal(0);
goalCostBound_units = Inf;
isEarliestArrival   = options.GoalTimeMode == "earliestArrival";
if isEarliestArrival
    % Earliest search is a directed acyclic graph in physical time. Schedule
    % each motion check at its target layer instead of eagerly walking that
    % candidate to the end of the request before advancing the source layer.
    % This changes no layer or edge verdict and avoids speculative checks after
    % the first reachable goal. Event columns are [source layer, source node,
    % target node, final layer, edge length, source-local order, certified end].
    pendingMotionEvents = cell(layerCount, 1);
    stateProposals      = cell(layerCount, 1);
    for layerIndex = 1:layerCount
        motionEvents = pendingMotionEvents{layerIndex};
        pendingMotionEvents{layerIndex} = zeros(0, 7);
        if ~isempty(motionEvents)
            motionEvents = sortrows(motionEvents, [1, 6]);
            sourceLayerIndices = unique(motionEvents(:, 1), "stable").';
            for sourceLayerIndex = sourceLayerIndices
                eventOffsets = find(motionEvents(:, 1) == sourceLayerIndex);
                eventIsCertifiedBlocked = ...
                    layerIndex <= motionEvents(eventOffsets, 7);
                eventIsClear        = false(numel(eventOffsets), 1);
                blockingCellIndices = zeros(numel(eventOffsets), 1, "uint32");
                witnessTimes_s      = NaN(numel(eventOffsets), 1);
                exactEventOffsets   = find(~eventIsCertifiedBlocked);
                if ~isempty(exactEventOffsets)
                    exactRows = eventOffsets(exactEventOffsets);
                    [exactIsClear, exactBlockingCellIndices, exactWitnessTimes_s] = edgeIsClear( ...
                        motionEvents(exactRows, 2), motionEvents(exactRows, 3), ...
                        layerTimes_s(sourceLayerIndex), layerTimes_s(layerIndex));
                    eventIsClear(exactEventOffsets)        = exactIsClear;
                    blockingCellIndices(exactEventOffsets) = exactBlockingCellIndices;
                    witnessTimes_s(exactEventOffsets)      = exactWitnessTimes_s;
                end

                clearOffsets = eventOffsets(eventIsClear);
                if ~isempty(clearOffsets)
                    clearProposals = [ ...
                        motionEvents(clearOffsets, 1:3), ...
                        motionEvents(clearOffsets, 5), ...
                        ones(numel(clearOffsets), 1), ...
                        motionEvents(clearOffsets, 6)];
                    stateProposals{layerIndex} = [ ...
                        stateProposals{layerIndex}; clearProposals];
                end

                failedEventOffsets = find(~eventIsClear);
                certifiedSkipCount = certifiedSkipCount + nnz(eventIsCertifiedBlocked);
                rejectedCount      = rejectedCount + numel(failedEventOffsets);
                for failureIndex = 1:numel(failedEventOffsets)
                    eventOffset = failedEventOffsets(failureIndex);
                    eventRow    = eventOffsets(eventOffset);
                    if ~eventIsCertifiedBlocked(eventOffset)
                        blockedLayerCount = certifiedBlockedLayerCount( ...
                            motionEvents(eventRow, 2), motionEvents(eventRow, 3), ...
                            sourceLayerIndex, layerIndex, motionEvents(eventRow, 4), ...
                            blockingCellIndices(eventOffset), witnessTimes_s(eventOffset));
                        motionEvents(eventRow, 7) = layerIndex + blockedLayerCount - 1;
                    end
                    if layerIndex < motionEvents(eventRow, 4)
                        pendingMotionEvents{layerIndex + 1}(end + 1, :) = ...
                            motionEvents(eventRow, :);
                    end
                end
            end
        end

        proposals = stateProposals{layerIndex};
        if ~isempty(proposals)
            proposals = sortrows(proposals, [1, 5, 6]);
            for proposalIndex = 1:size(proposals, 1)
                [reachable, spatialCost_units, parentLayerIndex, parentNodeIndex] = updateTemporalState( ...
                    reachable, spatialCost_units, parentLayerIndex, parentNodeIndex, ...
                    proposals(proposalIndex, 1), proposals(proposalIndex, 2), ...
                    layerIndex, proposals(proposalIndex, 3), proposals(proposalIndex, 4));
            end
        end

        % Once the goal is reached, retain its complete clear wait component.
        firstGoalLayerIndex = find( ...
            reachable(1:layerIndex, 2) & goalLayerIsEligible(1:layerIndex), 1, "first");
        if ~isempty(firstGoalLayerIndex)
            selectedGoalLayerIndex = double(waitComponentFinalLayerIndex(firstGoalLayerIndex, 2));
            if selectedGoalLayerIndex == layerCount
                selectedGoalLayerIndex = firstGoalLayerIndex;
            end
            if layerIndex >= selectedGoalLayerIndex
                break
            end
        end
        if layerIndex == layerCount
            break
        end

        currentNodeIndices = find(reachable(layerIndex, :));
        for currentNodeIndex = reshape(currentNodeIndices, 1, [])
            expandedCount = expandedCount + 1;
            if waitIsClear(layerIndex, currentNodeIndex)
                waitProposal = [layerIndex, currentNodeIndex, currentNodeIndex, ...
                    0, 0, currentNodeIndex];
                stateProposals{layerIndex + 1}(end + 1, :) = waitProposal;
            else
                rejectedCount = rejectedCount + 1;
            end
        end

        [motionCandidates, candidateRejectedCount] = buildLayerCandidates( ...
            currentNodeIndices, layerIndex, layerTimes_s(layerIndex), layerTimes_s, motionEdgeExists, ...
            minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, ...
            isWaitComponentStart, waitComponentFinalLayerIndex);
        rejectedCount = rejectedCount + candidateRejectedCount;
        targetLayerIndices = unique(motionCandidates(:, 3)).';
        for targetLayerIndex = targetLayerIndices
            candidateRows = find(motionCandidates(:, 3) == targetLayerIndex);
            eventBlock = [ ...
                repmat(layerIndex, numel(candidateRows), 1), ...
                motionCandidates(candidateRows, [1, 2, 4, 5]), ...
                candidateRows, zeros(numel(candidateRows), 1)];
            pendingMotionEvents{targetLayerIndex} = [ ...
                pendingMotionEvents{targetLayerIndex}; eventBlock];
        end
    end
else
    for layerIndex = 1:layerCount - 1
        currentNodeIndices = find(reachable(layerIndex, :));
        for currentNodeIndex = reshape(currentNodeIndices, 1, [])
            expandedCount = expandedCount + 1;
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
            currentNodeIndices, layerIndex, layerTimes_s(layerIndex), layerTimes_s, motionEdgeExists, ...
            minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, ...
            isWaitComponentStart, waitComponentFinalLayerIndex);
        rejectedCount        = rejectedCount + candidateRejectedCount;
        motionCandidateCount = size(motionCandidates, 1);
        motionIsPending      = true(motionCandidateCount, 1);
        while any(motionIsPending)
            queriedTargetLayerIndices = unique(motionCandidates(motionIsPending, 3));
            for targetLayerIndex = reshape(queriedTargetLayerIndices, 1, [])
                queryIndices = find(motionIsPending & motionCandidates(:, 3) == targetLayerIndex);
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
                if isempty(queryIndices)
                    continue
                end
                queryIsClear = edgeIsClear( ...
                    motionCandidates(queryIndices, 1), motionCandidates(queryIndices, 2), ...
                    layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
                clearIndices = queryIndices(queryIsClear);
                for motionIndex = reshape(clearIndices, 1, [])
                    [reachable, spatialCost_units, parentLayerIndex, parentNodeIndex] = updateTemporalState( ...
                        reachable, spatialCost_units, parentLayerIndex, parentNodeIndex, ...
                        layerIndex, motionCandidates(motionIndex, 1), ...
                        motionCandidates(motionIndex, 3), motionCandidates(motionIndex, 2), ...
                        motionCandidates(motionIndex, 5));
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
    "CertifiedSkippedTransitionCount", certifiedSkipCount, ...
    "ExpandedCount",                 expandedCount, ...
    "SelectedGoalWindowStartTime_s", selectedGoalWindowStartTime_s, ...
    "SelectedGoalWindowEndTime_s",   selectedGoalWindowEndTime_s, ...
    "MinimumGoalArrivalTime_s",      minimumGoalArrivalTime_s, ...
    "WaitRoute_units",               waitRoute_units, ...
    "WaitRouteTime_s",               waitRouteTime_s, ...
    "DynamicEdgeCheckKind",          "exactAffineConvexCells");

%% Section 3: Local Functions

function [isClear, blockingCellIndices, witnessTimes_s] = ...
        edgeIsClear(firstNodeIndices, secondNodeIndices, first_s, second_s)
    % Check the complete segment clock against exact static geometry and every
    % affine moving convex cell. A zero-length edge is a stationary point path.
    firstNodeIndices  = firstNodeIndices(:);
    secondNodeIndices = secondNodeIndices(:);
    first_units       = nodePosition_units(firstNodeIndices, :);
    second_units      = nodePosition_units(secondNodeIndices, :);
    edgeCount         = numel(firstNodeIndices);
    isClear           = true(edgeCount, 1);
    blockingCellIndices = zeros(edgeCount, 1, "uint32");
    witnessTimes_s      = NaN(edgeCount, 1);

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
    [dynamicIsClear, dynamicBlockingCellIndices, dynamicWitnessTimes_s] = affineEdgesAreClear( ...
        first_units(candidateIndices, :), second_units(candidateIndices, :), ...
        firstNodeIndices(candidateIndices), secondNodeIndices(candidateIndices), ...
        first_s, second_s, dynamicCells, dynamicPairCache, dynamicCellIsCounterclockwise);
    isClear(candidateIndices)             = dynamicIsClear;
    blockingCellIndices(candidateIndices) = dynamicBlockingCellIndices;
    witnessTimes_s(candidateIndices)      = dynamicWitnessTimes_s;
end

function blockedLayerCount = certifiedBlockedLayerCount( ...
        sourceNodeIndex, targetNodeIndex, sourceLayerIndex, targetLayerIndex, ...
        finalTargetLayerIndex, cellIndex, witnessTime_s)
    % A strict interior point at one physical time proves a contiguous prefix
    % of later arrival clocks colliding with the same affine convex cell. The
    % positive residual reserve excludes tolerance-only contact. Any uncertain
    % geometry falls back to the original one-layer retry.
    blockedLayerCount = 1;
    if cellIndex == 0 || ~isfinite(witnessTime_s)
        return
    end
    cellIndex = double(cellIndex);
    if any(~isfinite([dynamicCellLower_units(cellIndex, :), ...
            dynamicCellUpper_units(cellIndex, :)]))
        return
    end

    sourceTime_s = layerTimes_s(sourceLayerIndex);
    arrivalTimes_s = layerTimes_s(targetLayerIndex:finalTargetLayerIndex);
    if witnessTime_s < sourceTime_s || witnessTime_s > arrivalTimes_s(1)
        return
    end

    active_s = dynamicCells.ActiveTimeInterval_s(cellIndex, :);
    cellDuration_s = diff(active_s);
    if ~(cellDuration_s > 0) || witnessTime_s < active_s(1) || witnessTime_s > active_s(2)
        return
    end
    cellClock = (witnessTime_s - active_s(1)) / cellDuration_s;
    regionStart_units = dynamicCells.Regions_units{cellIndex};
    region_units = regionStart_units + cellClock .* ...
        (dynamicCells.EndRegions_units{cellIndex} - regionStart_units);
    vertexCount = size(region_units, 1);
    following   = [2:vertexCount, 1];
    edge_units  = region_units(following, :) - region_units;
    signedArea_units2 = sum( ...
        region_units(:, 1) .* region_units(following, 2) - ...
        region_units(:, 2) .* region_units(following, 1)) / 2;

    source_units = nodePosition_units(sourceNodeIndex, :);
    target_units = nodePosition_units(targetNodeIndex, :);
    witnessOffset_s = witnessTime_s - sourceTime_s;
    firstPathClock  = witnessOffset_s / (arrivalTimes_s(1) - sourceTime_s);
    if ~isfinite(firstPathClock)
        return
    end
    displacement_units = target_units - source_units;
    relativeSourceX_units = source_units(1) - region_units(:, 1);
    relativeSourceY_units = source_units(2) - region_units(:, 2);
    residualConstant_units2 = edge_units(:, 1) .* relativeSourceY_units - ...
        edge_units(:, 2) .* relativeSourceX_units;
    residualClock_units2 = edge_units(:, 1) .* displacement_units(2) - ...
        edge_units(:, 2) .* displacement_units(1);
    coordinateScale_units = max([1; abs(region_units(:)); ...
        abs(source_units(:)); abs(target_units(:))]);
    certificateBound_units2 = 16 * createResidualBound(coordinateScale_units);
    if abs(signedArea_units2) <= certificateBound_units2
        return
    end

    orientationSign = sign(signedArea_units2);
    signedConstant_units2 = orientationSign * residualConstant_units2;
    signedClock_units2    = orientationSign * residualClock_units2;
    firstResidual_units2 = signedConstant_units2 + ...
        firstPathClock .* signedClock_units2;
    if any(firstResidual_units2 <= certificateBound_units2)
        return
    end

    % At the fixed witness time, the path clock decreases monotonically as
    % arrival is delayed. Only a positive clock coefficient can therefore
    % lose strictness. Its exact threshold supplies the complete blocked
    % arrival prefix without repeatedly probing every half-space.
    canLoseStrictness = signedClock_units2 > 0;
    if ~any(canLoseStrictness)
        blockedLayerCount = numel(arrivalTimes_s);
        return
    end
    criticalPathClock = max( ...
        (certificateBound_units2 - signedConstant_units2(canLoseStrictness)) ./ ...
        signedClock_units2(canLoseStrictness));
    if criticalPathClock <= 0 || witnessOffset_s == 0
        blockedLayerCount = numel(arrivalTimes_s);
        return
    end
    firstFailedArrival_s = sourceTime_s + witnessOffset_s / criticalPathClock;
    finalStrictOffset = min(numel(arrivalTimes_s), ...
        sortedUpperBound(arrivalTimes_s, firstFailedArrival_s));
    finalStrictOffset = max(1, finalStrictOffset);
    while finalStrictOffset > 1
        trialClock = witnessOffset_s / ...
            (arrivalTimes_s(finalStrictOffset) - sourceTime_s);
        trialResidual_units2 = signedConstant_units2 + ...
            trialClock .* signedClock_units2;
        if all(trialResidual_units2 > certificateBound_units2)
            break
        end
        finalStrictOffset = finalStrictOffset - 1;
    end
    blockedLayerCount = finalStrictOffset;
end
end

function [candidates, rejectedCount] = buildLayerCandidates( ...
        sourceNodeIndices, sourceLayerIndex, sourceTime_s, layerTimes_s, motionEdgeExists, ...
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
            batchSourceNodeIndices, sourceLayerIndex, sourceTime_s, layerTimes_s, motionEdgeExists, ...
            minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, ...
            isWaitComponentStart, waitComponentFinalLayerIndex);
        rejectedCount = rejectedCount + batchRejectedCount;
    end
    candidates = vertcat(candidateBlocks{:});
end

function [candidates, rejectedCount] = buildCandidateBatch( ...
        sourceNodeIndices, sourceLayerIndex, sourceTime_s, layerTimes_s, motionEdgeExists, ...
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
    % Every non-self motion has positive physical duration. Keeping its first
    % target strictly after the source makes the chronological event buckets a
    % true DAG, including when the velocity bound falls below roundoff slack.
    firstFeasibleLayerIndices = max(firstFeasibleLayerIndices, sourceLayerIndex + 1);
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

function [isClear, blockingCellIndices, witnessTimes_s] = affineEdgesAreClear( ...
        first_units, second_units, firstNodeIndices, secondNodeIndices, ...
        first_s, second_s, cells, pairCache, cellIsCounterclockwise)
    % A path point and every vertex of a time cell are affine in time. Each
    % convex half-space residual is therefore quadratic; its real roots
    % partition the clock into intervals of constant inside/outside sign.
    edgeCount           = size(first_units, 1);
    isClear             = true(edgeCount, 1);
    blockingCellIndices = zeros(edgeCount, 1, "uint32");
    witnessTimes_s      = NaN(edgeCount, 1);
    if edgeCount == 0 || isempty(cells.Regions_units)
        return
    end
    if edgeCount == 1
        [isClear, blockingCellIndices, witnessTimes_s] = affineSingleEdgeIsClear( ...
            first_units, second_units, firstNodeIndices, secondNodeIndices, ...
            first_s, second_s, cells, pairCache, cellIsCounterclockwise);
        return
    end

    % Intersect the cached spatial segment/box clock with each cell's active
    % portion of this edge clock. Only surviving edge/cell pairs reach the
    % unchanged exact affine residual calculation below.
    cellBlocks = cell(edgeCount, 1);
    edgeBlocks = cell(edgeCount, 1);
    edgeDuration_s = second_s - first_s;
    for edgeIndex = 1:edgeCount
        [pairCellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
            pairCellCandidates( ...
            firstNodeIndices(edgeIndex), secondNodeIndices(edgeIndex), pairCache);
        if isempty(pairCellIndices)
            continue
        end
        finalActiveOffset = sortedUpperBound(activeStart_s, second_s);
        if finalActiveOffset == 0
            continue
        end
        pairOffsets = (1:finalActiveOffset).';
        pairOffsets = pairOffsets(activeEnd_s(pairOffsets) >= first_s);
        if isempty(pairOffsets)
            continue
        end
        pairCellIndices = pairCellIndices(pairOffsets);
        active_s = cells.ActiveTimeInterval_s(pairCellIndices, :);
        overlapStart_s = max(first_s, active_s(:, 1));
        overlapEnd_s   = min(second_s, active_s(:, 2));
        meetsClock = overlapStart_s <= overlapEnd_s;
        if edgeDuration_s > 0
            overlapClockStart = (overlapStart_s - first_s) / edgeDuration_s;
            overlapClockEnd   = (overlapEnd_s - first_s) / edgeDuration_s;
            absoluteTimeScale_s = max([ ...
                repmat(max(abs([first_s, second_s])), numel(pairCellIndices), 1), ...
                abs(active_s)], [], 2);
            clockGuard = 512 * eps(max(1, absoluteTimeScale_s)) / edgeDuration_s;
            clockGuard(~isfinite(clockGuard)) = Inf;
            overlapClockStart = overlapClockStart - clockGuard;
            overlapClockEnd   = overlapClockEnd + clockGuard;
        else
            overlapClockStart = zeros(size(overlapStart_s));
            overlapClockEnd   = zeros(size(overlapEnd_s));
        end
        pairQEnter = qEnter(pairOffsets);
        pairQExit  = qExit(pairOffsets);
        meetsPathClock = pairQExit >= overlapClockStart & ...
            pairQEnter <= overlapClockEnd;
        retainedOffsets = find(meetsClock & meetsPathClock);
        cellBlocks{edgeIndex} = pairCellIndices(retainedOffsets);
        edgeBlocks{edgeIndex} = repmat(edgeIndex, numel(retainedOffsets), 1);
    end
    candidateCellIndices = vertcat(cellBlocks{:});
    candidateEdgeIndices = vertcat(edgeBlocks{:});
    if isempty(candidateCellIndices)
        return
    end
    [candidateCellIndices, order] = sort(candidateCellIndices);
    candidateEdgeIndices = candidateEdgeIndices(order);
    firstBlockOffsets = [1; 1 + find(diff(candidateCellIndices) ~= 0)];
    finalBlockOffsets = [firstBlockOffsets(2:end) - 1; numel(candidateCellIndices)];

    for blockIndex = 1:numel(firstBlockOffsets)
        if ~any(isClear)
            break
        end
        cellIndex = candidateCellIndices(firstBlockOffsets(blockIndex));
        blockOffsets = firstBlockOffsets(blockIndex):finalBlockOffsets(blockIndex);
        candidateIndices = candidateEdgeIndices(blockOffsets);
        candidateIndices = candidateIndices(isClear(candidateIndices));
        if isempty(candidateIndices)
            continue
        end
        overlapStart_s = max(first_s, cells.ActiveTimeInterval_s(cellIndex, 1));
        overlapEnd_s   = min(second_s, cells.ActiveTimeInterval_s(cellIndex, 2));

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
        if isscalar(candidateIndices)
            [pointTouchesCell, witnessClock] = affinePointTouchesConvexScalar( ...
                pathStart_units, pathEnd_units, overlapRegionStart_units, ...
                overlapRegionEnd_units, cellIsCounterclockwise(cellIndex));
        else
            [pointTouchesCell, witnessClock] = affinePointsTouchConvex( ...
                pathStart_units, pathEnd_units, overlapRegionStart_units, ...
                overlapRegionEnd_units, cellIsCounterclockwise(cellIndex));
        end
        blockedIndices = candidateIndices(pointTouchesCell);
        isClear(blockedIndices) = false;
        blockingCellIndices(blockedIndices) = uint32(cellIndex);
        witnessTimes_s(blockedIndices) = overlapStart_s + ...
            witnessClock(pointTouchesCell) .* (overlapEnd_s - overlapStart_s);
    end
end

function [isClear, blockingCellIndex, witnessTime_s] = affineSingleEdgeIsClear( ...
        first_units, second_units, firstNodeIndex, secondNodeIndex, ...
        first_s, second_s, cells, pairCache, cellIsCounterclockwise)
    % Preserve the exact batch predicate while avoiding block assembly for one edge.
    isClear           = true;
    blockingCellIndex = uint32(0);
    witnessTime_s     = NaN;
    [pairCellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
        pairCellCandidates(firstNodeIndex, secondNodeIndex, pairCache);
    if isempty(pairCellIndices)
        return
    end

    finalActiveOffset = sortedUpperBound(activeStart_s, second_s);
    if finalActiveOffset == 0
        return
    end
    pairOffsets = (1:finalActiveOffset).';
    pairOffsets = pairOffsets(activeEnd_s(pairOffsets) >= first_s);
    if isempty(pairOffsets)
        return
    end
    pairCellIndices = pairCellIndices(pairOffsets);
    active_s        = cells.ActiveTimeInterval_s(pairCellIndices, :);
    overlapStart_s  = max(first_s, active_s(:, 1));
    overlapEnd_s    = min(second_s, active_s(:, 2));
    meetsClock      = overlapStart_s <= overlapEnd_s;
    edgeDuration_s  = second_s - first_s;
    if edgeDuration_s > 0
        overlapClockStart = (overlapStart_s - first_s) / edgeDuration_s;
        overlapClockEnd   = (overlapEnd_s - first_s) / edgeDuration_s;
        absoluteTimeScale_s = max([ ...
            repmat(max(abs([first_s, second_s])), numel(pairCellIndices), 1), ...
            abs(active_s)], [], 2);
        clockGuard = 512 * eps(max(1, absoluteTimeScale_s)) / edgeDuration_s;
        clockGuard(~isfinite(clockGuard)) = Inf;
        overlapClockStart = overlapClockStart - clockGuard;
        overlapClockEnd   = overlapClockEnd + clockGuard;
    else
        overlapClockStart = zeros(size(overlapStart_s));
        overlapClockEnd   = zeros(size(overlapEnd_s));
    end
    pairQEnter = qEnter(pairOffsets);
    pairQExit  = qExit(pairOffsets);
    meetsPathClock = pairQExit >= overlapClockStart & pairQEnter <= overlapClockEnd;
    candidateCellIndices = sort(pairCellIndices(meetsClock & meetsPathClock));

    for cellIndex = reshape(candidateCellIndices, 1, [])
        overlapStart_s = max(first_s, cells.ActiveTimeInterval_s(cellIndex, 1));
        overlapEnd_s   = min(second_s, cells.ActiveTimeInterval_s(cellIndex, 2));
        if edgeDuration_s > 0
            startFraction = (overlapStart_s - first_s) / edgeDuration_s;
            endFraction   = (overlapEnd_s - first_s) / edgeDuration_s;
        else
            startFraction = 0;
            endFraction   = 0;
        end
        displacement_units = second_units - first_units;
        pathStart_units     = first_units + startFraction .* displacement_units;
        pathEnd_units       = first_units + endFraction .* displacement_units;

        cellDuration_s    = diff(cells.ActiveTimeInterval_s(cellIndex, :));
        cellStartFraction = (overlapStart_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
        cellEndFraction   = (overlapEnd_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
        regionStart_units = cells.Regions_units{cellIndex};
        regionDelta_units = cells.EndRegions_units{cellIndex} - regionStart_units;
        overlapRegionStart_units = regionStart_units + cellStartFraction .* regionDelta_units;
        overlapRegionEnd_units   = regionStart_units + cellEndFraction .* regionDelta_units;
        [pointTouchesCell, witnessClock] = affinePointTouchesConvexScalar( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, overlapRegionEnd_units, ...
            cellIsCounterclockwise(cellIndex));
        if pointTouchesCell
            isClear           = false;
            blockingCellIndex = uint32(cellIndex);
            witnessTime_s     = overlapStart_s + witnessClock * (overlapEnd_s - overlapStart_s);
            return
        end
    end
end

function residualBound_units2 = createResidualBound(coordinateScale_units)
    % Shared algebraic slack for the exact predicate and its broad phase.
    residualBound_units2 = 4096 * eps(coordinateScale_units .^ 2);
end

function [lower_units, upper_units, isCounterclockwise] = createCellBoxes(cells, nodeScale_units)
    % Outer boxes of the tolerance-expanded affine cells over their clocks.
    cellCount   = numel(cells.Regions_units);
    lower_units = zeros(cellCount, 2);
    upper_units = zeros(cellCount, 2);
    isCounterclockwise = false(cellCount, 1);
    for cellIndex = 1:cellCount
        regionStart_units = cells.Regions_units{cellIndex};
        regionEnd_units   = cells.EndRegions_units{cellIndex};
        cellBounds_units  = [regionStart_units; regionEnd_units];
        vertexCount       = size(regionStart_units, 1);
        followingIndices  = [2:vertexCount, 1];
        precedingIndices  = [vertexCount, 1:vertexCount - 1];
        middleRegion_units = (regionStart_units + regionEnd_units) / 2;
        middleSignedArea_units2 = sum( ...
            middleRegion_units(:, 1) .* middleRegion_units(followingIndices, 2) - ...
            middleRegion_units(:, 2) .* middleRegion_units(followingIndices, 1)) / 2;
        isCounterclockwise(cellIndex) = middleSignedArea_units2 >= 0;

        edgeStart_units = regionStart_units(followingIndices, :) - regionStart_units;
        edgeEnd_units   = regionEnd_units(followingIndices, :) - regionEnd_units;
        edgeDelta_units = edgeEnd_units - edgeStart_units;
        edgeDeltaScale_units2 = sum(edgeDelta_units.^2, 2);
        edgeFraction = zeros(vertexCount, 1);
        edgeIsMoving = edgeDeltaScale_units2 > 0;
        edgeFraction(edgeIsMoving) = min(1, max(0, ...
            -sum(edgeStart_units(edgeIsMoving, :) .* edgeDelta_units(edgeIsMoving, :), 2) ./ ...
            edgeDeltaScale_units2(edgeIsMoving)));
        shortestEdge_units = min(vecnorm(edgeStart_units + edgeFraction .* edgeDelta_units, 2, 2));
        longestEdge_units  = max(vecnorm(edgeStart_units, 2, 2), vecnorm(edgeEnd_units, 2, 2));

        cornerConstant_units2 = crossProduct2D(edgeStart_units(precedingIndices, :), edgeStart_units);
        cornerLinear_units2 = crossProduct2D(edgeStart_units(precedingIndices, :), edgeDelta_units) + ...
            crossProduct2D(edgeDelta_units(precedingIndices, :), edgeStart_units);
        cornerQuadratic_units2 = crossProduct2D( ...
            edgeDelta_units(precedingIndices, :), edgeDelta_units);
        smallestCorner_units2 = minimumAbsoluteQuadratic( ...
            cornerQuadratic_units2, cornerLinear_units2, cornerConstant_units2);

        regionDelta_units = regionEnd_units - regionStart_units;
        areaConstant_units2 = sum( ...
            crossProduct2D(regionStart_units, regionStart_units(followingIndices, :))) / 2;
        areaLinear_units2 = sum( ...
            crossProduct2D(regionStart_units, regionDelta_units(followingIndices, :)) + ...
            crossProduct2D(regionDelta_units, regionStart_units(followingIndices, :))) / 2;
        areaQuadratic_units2 = sum( ...
            crossProduct2D(regionDelta_units, regionDelta_units(followingIndices, :))) / 2;
        smallestArea_units2 = minimumAbsoluteQuadratic( ...
            areaQuadratic_units2, areaLinear_units2, areaConstant_units2);

        coordinateScale_units = max([1; abs(cellBounds_units(:)); nodeScale_units]);
        residualBound_units2  = createResidualBound(coordinateScale_units);
        cellFlattens = shortestEdge_units <= 0 || smallestArea_units2 <= residualBound_units2 || ...
            any(smallestCorner_units2 <= residualBound_units2);
        boxMargin_units = Inf;
        if ~cellFlattens
            smallestSine = min(smallestCorner_units2 ./ ...
                (longestEdge_units(precedingIndices) .* longestEdge_units));
            boxMargin_units = 2 * residualBound_units2 / (shortestEdge_units * smallestSine);
        end
        lower_units(cellIndex, :) = min(cellBounds_units, [], 1) - boxMargin_units;
        upper_units(cellIndex, :) = max(cellBounds_units, [], 1) + boxMargin_units;
    end
end

function cache = createPairCellCache( ...
        nodePosition_units, cellLower_units, cellUpper_units, activeIntervals_s)
    % Closed segment/box parameter intervals for directed node pairs. Bound
    % eager storage; larger products use the identical calculation on demand.
    nodeCount   = size(nodePosition_units, 1);
    cellCount   = size(cellLower_units, 1);
    pairCount   = nodeCount ^ 2;
    maximumMaterializedPairCount     = 65536;
    maximumMaterializedPairCellTests = 4e6;
    materialize = pairCount <= maximumMaterializedPairCount && ...
        pairCount * cellCount <= maximumMaterializedPairCellTests;
    cache = struct( ...
        'NodeCount',         nodeCount, ...
        'IsMaterialized',    materialize, ...
        'NodePosition_units', nodePosition_units, ...
        'CellLower_units',   cellLower_units, ...
        'CellUpper_units',   cellUpper_units, ...
        'ActiveIntervals_s', activeIntervals_s, ...
        'CellIndices',       {cell(0, 1)}, ...
        'QEnter',            {cell(0, 1)}, ...
        'QExit',             {cell(0, 1)}, ...
        'ActiveStart_s',     {cell(0, 1)}, ...
        'ActiveEnd_s',       {cell(0, 1)});
    if ~materialize
        return
    end

    cellIndices = cell(pairCount, 1);
    qEnterCache = cell(pairCount, 1);
    qExitCache  = cell(pairCount, 1);
    activeStartCache_s = cell(pairCount, 1);
    activeEndCache_s   = cell(pairCount, 1);
    for secondNodeIndex = 1:nodeCount
        for firstNodeIndex = 1:nodeCount
            pairIndex = firstNodeIndex + nodeCount * (secondNodeIndex - 1);
            first_units = nodePosition_units(firstNodeIndex, :);
            second_units = nodePosition_units(secondNodeIndex, :);
            [cellIndices{pairIndex}, qEnterCache{pairIndex}, ...
                qExitCache{pairIndex}, activeStartCache_s{pairIndex}, ...
                activeEndCache_s{pairIndex}] = computePairCellEntry( ...
                first_units, second_units, cellLower_units, cellUpper_units, ...
                activeIntervals_s);
        end
    end
    cache.CellIndices   = cellIndices;
    cache.QEnter        = qEnterCache;
    cache.QExit         = qExitCache;
    cache.ActiveStart_s = activeStartCache_s;
    cache.ActiveEnd_s   = activeEndCache_s;
end

function [cellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
        pairCellCandidates(firstNodeIndex, secondNodeIndex, cache)
    % Return a materialized entry or calculate that exact entry on demand.
    if cache.IsMaterialized
        pairIndex    = firstNodeIndex + cache.NodeCount * (secondNodeIndex - 1);
        cellIndices  = cache.CellIndices{pairIndex};
        qEnter       = cache.QEnter{pairIndex};
        qExit        = cache.QExit{pairIndex};
        activeStart_s = cache.ActiveStart_s{pairIndex};
        activeEnd_s   = cache.ActiveEnd_s{pairIndex};
        return
    end
    [cellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
        computePairCellEntry( ...
        cache.NodePosition_units(firstNodeIndex, :), ...
        cache.NodePosition_units(secondNodeIndex, :), ...
        cache.CellLower_units, cache.CellUpper_units, cache.ActiveIntervals_s);
end

function [selected, selectedQEnter, selectedQExit, activeStart_s, activeEnd_s] = ...
        computePairCellEntry(first_units, second_units, cellLower_units, ...
        cellUpper_units, activeIntervals_s)
    % Conservative segment/box parameter clocks for one directed node pair.
    cellCount   = size(cellLower_units, 1);
    delta_units = second_units - first_units;
    qEnter      = zeros(cellCount, 1);
    qExit       = ones(cellCount, 1);
    canMeet     = true(cellCount, 1);
    uncertain   = any(isnan(cellLower_units) | isnan(cellUpper_units), 2);
    for dimensionIndex = 1:2
        if delta_units(dimensionIndex) == 0
            canMeet = canMeet & ( ...
                first_units(dimensionIndex) >= cellLower_units(:, dimensionIndex) & ...
                first_units(dimensionIndex) <= cellUpper_units(:, dimensionIndex));
        else
            firstIntersection = (cellLower_units(:, dimensionIndex) - ...
                first_units(dimensionIndex)) / delta_units(dimensionIndex);
            secondIntersection = (cellUpper_units(:, dimensionIndex) - ...
                first_units(dimensionIndex)) / delta_units(dimensionIndex);
            qEnter = max(qEnter, min(firstIntersection, secondIntersection));
            qExit  = min(qExit, max(firstIntersection, secondIntersection));
        end
    end
    qGuard = 256 * eps(max(1, max(abs([qEnter, qExit]), [], 2)));
    qGuard(~isfinite(qGuard)) = 0;
    canMeet = canMeet & qEnter <= qExit + qGuard & ...
        qExit >= -qGuard & qEnter <= 1 + qGuard;
    canMeet(uncertain) = true;
    qEnter(uncertain)  = 0;
    qExit(uncertain)   = 1;
    selected = find(canMeet);
    if ~isempty(selected)
        [~, activeOrder] = sortrows( ...
            [activeIntervals_s(selected, 1), selected], [1, 2]);
        selected = selected(activeOrder);
    end
    selectedQEnter = max(0, qEnter(selected) - qGuard(selected));
    selectedQExit  = min(1, qExit(selected) + qGuard(selected));
    activeStart_s  = activeIntervals_s(selected, 1);
    activeEnd_s    = activeIntervals_s(selected, 2);
end

function finalOffset = sortedUpperBound(sortedValues, queryValue)
    % Last index whose sorted value is not greater than the scalar query.
    lowOffset  = 1;
    highOffset = numel(sortedValues);
    finalOffset = 0;
    while lowOffset <= highOffset
        middleOffset = floor((lowOffset + highOffset) / 2);
        if sortedValues(middleOffset) <= queryValue
            finalOffset = middleOffset;
            lowOffset   = middleOffset + 1;
        else
            highOffset = middleOffset - 1;
        end
    end
end

function minimumAbsolute = minimumAbsoluteQuadratic(quadratic, linear, constant)
    % Smallest magnitude of a quadratic on the closed unit interval.
    minimumAbsolute = min(abs(constant), abs(quadratic + linear + constant));
    isCurved = quadratic ~= 0;
    stationaryClock = -linear ./ (2 * quadratic);
    stationaryIsInside = isCurved & stationaryClock > 0 & stationaryClock < 1;
    if any(stationaryIsInside)
        stationaryValue = abs( ...
            quadratic .* stationaryClock.^2 + linear .* stationaryClock + constant);
        minimumAbsolute(stationaryIsInside) = min( ...
            minimumAbsolute(stationaryIsInside), stationaryValue(stationaryIsInside));
    end
    discriminant = linear.^2 - 4 * quadratic .* constant;
    rootRadius   = sqrt(max(0, discriminant));
    firstRoot    = (-linear - rootRadius) ./ (2 * quadratic);
    secondRoot   = (-linear + rootRadius) ./ (2 * quadratic);
    hasRoot = isCurved & discriminant >= 0 & ...
        ((firstRoot >= 0 & firstRoot <= 1) | (secondRoot >= 0 & secondRoot <= 1));
    isLinear   = ~isCurved & linear ~= 0;
    linearRoot = -constant ./ linear;
    hasRoot = hasRoot | (isLinear & linearRoot >= 0 & linearRoot <= 1);
    hasRoot = hasRoot | (~isCurved & linear == 0 & constant == 0);
    minimumAbsolute(hasRoot) = 0;
end

function crossProduct_units2 = crossProduct2D(first_units, second_units)
    % Scalar planar cross product for matching vector rows.
    crossProduct_units2 = first_units(:, 1) .* second_units(:, 2) - ...
        first_units(:, 2) .* second_units(:, 1);
end

function [pointTouchesCell, strictWitnessClock] = affinePointsTouchConvex( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
        isCounterclockwise)
    % Test path points against one affine moving convex cell in one batch.
    % Residual roots, clock ends, and interval midpoints form exact sign probes.
    % Rows are cell edges and columns are path points, preserving the original
    % point-by-point arithmetic and residual tolerance.
    pointCount       = size(pointStart_units, 1);
    pointTouchesCell = false(pointCount, 1);
    strictWitnessClock = NaN(pointCount, 1);
    if pointCount == 0
        return
    end
    if pointCount == 1
        [pointTouchesCell, strictWitnessClock] = affinePointTouchesConvexScalar( ...
            pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
            isCounterclockwise);
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
    regionScale_units     = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units      = max(max(abs(pointStart_units), [], 2), max(abs(pointEnd_units), [], 2));
    coordinateScale_units = max(1, max(regionScale_units, pointScale_units));
    residualTolerance_units2 = createResidualBound(coordinateScale_units);
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

    residualTolerance_units2 = reshape(residualTolerance_units2, 1, pointCount);
    if isCounterclockwise
        inside = residual_units2 >= -residualTolerance_units2;
        strictInside = residual_units2 > 16 * residualTolerance_units2;
    else
        inside = residual_units2 <= residualTolerance_units2;
        strictInside = residual_units2 < -16 * residualTolerance_units2;
    end
    pointTouchesCell = reshape(any(all(inside, 1), 3), pointCount, 1);
    strictByPointAndProbe = reshape(all(strictInside, 1), pointCount, []);
    for pointIndex = reshape(find(pointTouchesCell), 1, [])
        strictProbeClocks = probes(pointIndex, strictByPointAndProbe(pointIndex, :));
        if ~isempty(strictProbeClocks)
            % For a later arrival the path point at a fixed physical time
            % moves toward its source. The latest strict witness therefore
            % gives the longest conservative retry certificate along the same
            % directed segment; the collision predicate itself is unchanged.
            lowerClock = max(strictProbeClocks);
            finiteCuts = cuts(pointIndex, isfinite(cuts(pointIndex, :)));
            nextCuts    = finiteCuts(finiteCuts > lowerClock);
            if ~isempty(nextCuts)
                upperClock = min(nextCuts);
                for refinementIndex = 1:12
                    trialClock = (lowerClock + upperClock) / 2;
                    trialResidual_units2 = quadratic_units2(:, pointIndex) * trialClock ^ 2 + ...
                        linear_units2(:, pointIndex) * trialClock + constant_units2(:, pointIndex);
                    if isCounterclockwise
                        trialIsStrict = all(trialResidual_units2 > ...
                            16 * residualTolerance_units2(pointIndex));
                    else
                        trialIsStrict = all(trialResidual_units2 < ...
                            -16 * residualTolerance_units2(pointIndex));
                    end
                    if trialIsStrict
                        lowerClock = trialClock;
                    else
                        upperClock = trialClock;
                    end
                end
            end
            strictWitnessClock(pointIndex) = lowerClock;
        end
    end
end

function [pointTouchesCell, strictWitnessClock] = affinePointTouchesConvexScalar( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
        isCounterclockwise)
    % Keep the overwhelmingly common one-edge predicate in two dimensions;
    % the general batch path uses page arrays for several simultaneous edges.
    vertexCount       = size(regionStart_units, 1);
    following         = [2:vertexCount, 1];
    edgeStart_units   = regionStart_units(following, :) - regionStart_units;
    edgeDelta_units   = (regionEnd_units(following, :) - regionEnd_units) - edgeStart_units;
    regionDelta_units = regionEnd_units - regionStart_units;
    pointDelta_units  = pointEnd_units - pointStart_units;
    relativeStartX_units = pointStart_units(1) - regionStart_units(:, 1);
    relativeStartY_units = pointStart_units(2) - regionStart_units(:, 2);
    relativeDeltaX_units = pointDelta_units(1) - regionDelta_units(:, 1);
    relativeDeltaY_units = pointDelta_units(2) - regionDelta_units(:, 2);
    quadratic_units2 = edgeDelta_units(:, 1) .* relativeDeltaY_units - ...
        edgeDelta_units(:, 2) .* relativeDeltaX_units;
    linear_units2 = (edgeDelta_units(:, 1) .* relativeStartY_units - ...
        edgeDelta_units(:, 2) .* relativeStartX_units) + ...
        (edgeStart_units(:, 1) .* relativeDeltaY_units - ...
        edgeStart_units(:, 2) .* relativeDeltaX_units);
    constant_units2 = edgeStart_units(:, 1) .* relativeStartY_units - ...
        edgeStart_units(:, 2) .* relativeStartX_units;
    regionScale_units = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units  = max(max(abs(pointStart_units), [], 2), ...
        max(abs(pointEnd_units), [], 2));
    coordinateScale_units = max(1, max(regionScale_units, pointScale_units));
    residualTolerance_units2 = createResidualBound(coordinateScale_units);
    if ~isCounterclockwise
        quadratic_units2 = -quadratic_units2;
        linear_units2    = -linear_units2;
        constant_units2  = -constant_units2;
    end
    pointTouchesCell   = false;
    strictWitnessClock = NaN;
    endResidual_units2 = quadratic_units2 + linear_units2 + constant_units2;
    if all(endResidual_units2 > 16 * residualTolerance_units2)
        pointTouchesCell   = true;
        strictWitnessClock = 1;
        return
    end

    % A half-space that excludes the point for the whole unit clock proves
    % clearance without enumerating residual roots. The extra full residual
    % reserve keeps this broad proof strictly inside the exact predicate.
    maximumResidual_units2 = max(constant_units2, endResidual_units2);
    isConcave               = quadratic_units2 < 0;
    stationaryClock         = -linear_units2 ./ (2 * quadratic_units2);
    stationaryIsInside      = isConcave & stationaryClock > 0 & stationaryClock < 1;
    if any(stationaryIsInside)
        stationaryResidual_units2 = quadratic_units2 .* stationaryClock.^2 + ...
            linear_units2 .* stationaryClock + constant_units2;
        maximumResidual_units2(stationaryIsInside) = max( ...
            maximumResidual_units2(stationaryIsInside), ...
            stationaryResidual_units2(stationaryIsInside));
    end
    if any(maximumResidual_units2 < -2 * residualTolerance_units2)
        return
    end

    rootScale         = max(1, max(max(abs(quadratic_units2), ...
        abs(linear_units2)), abs(constant_units2)));
    rootTolerance     = 256 * eps(rootScale);
    isLinear          = abs(quadratic_units2) <= rootTolerance;
    discriminant      = linear_units2 .* linear_units2 - ...
        4 * quadratic_units2 .* constant_units2;
    hasQuadraticRoots = ~isLinear & discriminant >= -rootTolerance;
    rootRadius        = sqrt(max(0, discriminant));
    firstRoot         = (-linear_units2 - rootRadius) ./ (2 * quadratic_units2);
    secondRoot        = (-linear_units2 + rootRadius) ./ (2 * quadratic_units2);
    linearRoot        = -constant_units2 ./ linear_units2;
    hasLinearRoot     = isLinear & abs(linear_units2) > rootTolerance;
    firstRoot(isLinear)                             = linearRoot(isLinear);
    firstRoot(~hasQuadraticRoots & ~hasLinearRoot) = NaN;
    secondRoot(~hasQuadraticRoots)                 = NaN;
    firstRoot(~(firstRoot >= 0 & firstRoot <= 1))  = NaN;
    secondRoot(~(secondRoot >= 0 & secondRoot <= 1)) = NaN;

    finiteRoots = [firstRoot(isfinite(firstRoot)); secondRoot(isfinite(secondRoot))];
    cuts   = sort([0, 1, finiteRoots.']);
    probes = [cuts, (cuts(1:end - 1) + cuts(2:end)) / 2];
    residual_units2 = quadratic_units2 .* probes.^2 + ...
        linear_units2 .* probes + constant_units2;
    inside      = residual_units2 >= -residualTolerance_units2;
    strictProbe = all(residual_units2 > 16 * residualTolerance_units2, 1);
    pointTouchesCell = any(all(inside, 1));
    if ~pointTouchesCell || ~any(strictProbe)
        return
    end

    lowerClock = max(probes(strictProbe));
    finiteCuts = cuts(isfinite(cuts));
    nextCuts   = finiteCuts(finiteCuts > lowerClock);
    if ~isempty(nextCuts)
        upperClock = min(nextCuts);
        for refinementIndex = 1:12
            trialClock = (lowerClock + upperClock) / 2;
            trialResidual_units2 = quadratic_units2 * trialClock ^ 2 + ...
                linear_units2 * trialClock + constant_units2;
            trialIsStrict = all(trialResidual_units2 > 16 * residualTolerance_units2);
            if trialIsStrict
                lowerClock = trialClock;
            else
                upperClock = trialClock;
            end
        end
    end
    strictWitnessClock = lowerClock;
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
    parentNodeIndex(targetLayerIndex, targetNodeIndex)   = uint32(sourceNodeIndex);
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
        sweptIntervalIndices = find(preparation.IntervalUsesSweptCells);
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
