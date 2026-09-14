function [route_units, routeTime_s, record] = timeExpandedVisibilitySearch(nodePosition_units, edgeCost_units, obstacles, initialState, goalState, limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX: [route_units, routeTime_s, record] =
%   obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, edgeCost_units,
%   obstacles, initialState, goalState, limits, sampleTimes_s, options)
% PURPOSE: Search forward reachability using waits and moving edges at every supplied planning time.
%   Static segments and affine moving convex cells are checked over their complete active
%   intervals. Layer-snapped arrivals do not prove global earliest arrival.
% INPUTS: nodePosition_units (N-by-2 numeric matrix) Nodes with start first and goal second.
%   edgeCost_units (N-by-N numeric matrix) Finite entries enable motion edges. obstacles (canonical
%   protected obstacle struct array) initialState, goalState, limits, options (scalar structs)
%   sampleTimes_s (numeric vector) Candidate times retained exactly as temporal search layers.
% OUTPUTS: route_units (M-by-2 numeric matrix), routeTime_s (M-by-1 numeric vector) Selected timed
%   route, or documented empty arrays on exhaustion. record (scalar struct) Search counts, frontier,
%   and best partial ancestry.
% UNITS: Position and edge cost are coordinate units; time is seconds.

%% Section 1: Propagate The Reachability Frontier
layerTimes_s = unique([initialState.time_s; sampleTimes_s(:); goalState.time_s]);
layerTimes_s = layerTimes_s(layerTimes_s >= initialState.time_s & layerTimes_s <= goalState.time_s);
layerCount   = numel(layerTimes_s);
nodeCount    = size(nodePosition_units, 1);
initialPosition_units=nodePosition_units(1,:);
goalPosition_units=nodePosition_units(2,:);
if isfield(initialState,'position_units')
    initialPosition_units=initialState.position_units;
end
if isfield(goalState,'position_units')
    goalPosition_units=goalState.position_units;
end
minimumGoalArrivalTime_s = initialState.time_s + ...
    max(abs(goalPosition_units-initialPosition_units)./ ...
    limits.maxVelocity_units_s);
hasEndpointDerivatives = all(isfield(initialState, ...
    {'position_units','velocity_units_s','acceleration_units_s2'})) && ...
    all(isfield(goalState, ...
    {'position_units','velocity_units_s','acceleration_units_s2'}));
hasDerivativeLimits = all(isfield(limits, ...
    {'maxAcceleration_units_s2','maxJerk_units_s3'}));
if hasEndpointDerivatives && hasDerivativeLimits
    minimumGoalArrivalTime_s = initialState.time_s + ...
        obstacleAvoidance.input.minimumTravelTime(initialState,goalState,limits);
end
timeTolerance_s = 256*eps(max(1,max(abs(layerTimes_s))));
goalLayerIsEligible = true(layerCount,1);
if options.GoalTimeMode == "earliestArrival"
    goalLayerIsEligible = ...
        layerTimes_s >= minimumGoalArrivalTime_s-timeTolerance_s;
end
% The local obstacle snapshot stays unchanged throughout this search.
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles,[initialState.time_s,goalState.time_s]);
% Keep cached boundaries local, with fewer entries for larger boundaries or more obstacles.
for j=1:numel(obstacles)
    obstacles(j).InternalPreparation.QueryGeometryCache=containers.Map('KeyType','double','ValueType','any');
    obstacles(j).InternalPreparation.QueryGeometryCacheCapacity=floor(2^14 / max(1,numel(obstacles)) / ...
        max([1;cellfun(@numel,obstacles(j).x_units(:))]));
end
maximumCacheBytes = 300 * 1024 ^ 2;
% Time-invariant obstacles are checked exactly, one obstacle at a time, over
% the part of each edge's clock during which that obstacle exists. The
% prepared sample geometry is the protected boundary itself, so no snapshot or
% closure tolerance is introduced.
isStatic = arrayfun(@(obstacle) obstacle.InternalPreparation.SamplesExactlyEqual, obstacles);
dynamicObstacles = obstacles(~isStatic);
staticObstacles = obstacles(isStatic);
staticCount = numel(staticObstacles);
dynamicCells = obstacleAvoidance.obstacles.createTimeCells( ...
    dynamicObstacles,initialState.time_s,goalState.time_s);
staticShapes = cell(staticCount,1);
staticEdgeStart_units = cell(staticCount,1);
staticEdgeEnd_units = cell(staticCount,1);
staticActive_s = repmat([-Inf,Inf],staticCount,1);
staticExists = false(staticCount,1);
for k = 1:staticCount
    obstacle = staticObstacles(k);
    preparation = obstacle.InternalPreparation;
    preparedSampleIndex = find(preparation.SamplePrepared,1,"first");
    if isempty(preparedSampleIndex), continue; end
    staticExists(k) = true;
    staticShapes{k} = preparation.SampleShapes{preparedSampleIndex};
    staticEdgeStart_units{k} = preparation.SampleEdgeStart_units{preparedSampleIndex};
    staticEdgeEnd_units{k} = preparation.SampleEdgeEnd_units{preparedSampleIndex};
    if numel(obstacle.time_s)>1
        staticActive_s(k,:) = [obstacle.time_s(1),obstacle.time_s(end)];
    end
end
% Cache only clock-independent answers: edges whose whole clock lies inside
% the obstacle's lifetime. Partially covered edges are checked directly.
staticEdgeCache = zeros(0,0,'uint8');
if staticCount > 0 && nodeCount^2*staticCount <= maximumCacheBytes
    staticEdgeCache = zeros(nodeCount^2,staticCount,'uint8');
end
nodeIsFree   = false(layerCount, nodeCount);
for layerIndex = 1:layerCount
    nodeIsFree(layerIndex, :) = ~obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
        obstacles,nodePosition_units(:,1),nodePosition_units(:,2),layerTimes_s(layerIndex),false).';
end
waitIsClear = false(max(0, layerCount - 1), nodeCount);
for layerIndex = 1:layerCount - 1
    candidateNodeIndices = find(nodeIsFree(layerIndex, :) & nodeIsFree(layerIndex + 1, :));
    % Test a stationary wait only at nodes that are free in both adjacent layers; all other waits remain unavailable.
    if ~isempty(candidateNodeIndices)
        waitIsClear(layerIndex, candidateNodeIndices) = edgeIsClear(candidateNodeIndices, candidateNodeIndices, layerTimes_s(layerIndex), layerTimes_s(layerIndex + 1));
    end
end

% Within a clear-wait interval, an earlier arrival can wait to match a later one.
isWaitComponentStart = nodeIsFree;
isWaitComponentStart(2:end, :) = nodeIsFree(2:end, :) & ~waitIsClear;
waitComponentFinalLayerIndex = repmat(uint32((1:layerCount).'), 1, nodeCount);
for layerIndex = layerCount - 1:-1:1
    continuingNodeIndices = find(waitIsClear(layerIndex, :));
    waitComponentFinalLayerIndex(layerIndex, continuingNodeIndices) = waitComponentFinalLayerIndex(layerIndex + 1, continuingNodeIndices);
end
motionEdgeExists = isfinite(edgeCost_units);
motionEdgeExists(1:nodeCount + 1:end) = false;
minimumEdgeDuration_s = zeros(nodeCount, nodeCount);
motionEdgeLengths_units = zeros(nodeCount, nodeCount);
for sourceIndex = 1:nodeCount
    displacement_units = nodePosition_units - nodePosition_units(sourceIndex, :);
    minimumEdgeDuration_s(sourceIndex, :) = max(abs(displacement_units) ./ limits.maxVelocity_units_s, [], 2).';
    for targetIndex = reshape(find(motionEdgeExists(sourceIndex, :)), 1, [])
        motionEdgeLengths_units(sourceIndex, targetIndex) = norm(displacement_units(targetIndex, :));
    end
end
distanceToGoal_units = vecnorm(nodePosition_units - nodePosition_units(2, :), 2, 2);
goalCanWaitToFinal = nodeIsFree(:, 2) & double(waitComponentFinalLayerIndex(:, 2)) == layerCount;
reachable        = false(layerCount, nodeCount);
spatialCost_units  = Inf(layerCount, nodeCount);
parentLayerIndex = zeros(layerCount, nodeCount, "uint32");
parentNodeIndex  = zeros(layerCount, nodeCount, "uint16");
reachable(1, 1) = nodeIsFree(1, 1);
spatialCost_units(1, 1) = 0;
[waitCount, motionCount, rejectedCount, expandedCount, goalBoundRejectionCount, candidateBatchSplitCount] = deal(0);
goalCostBound_units=Inf;
exploredNodes_units = zeros(0, 2);
for layerIndex = 1:layerCount - 1
    % Once the goal is reached, retain its complete clear wait component.
    % That gives BMTP a kinematically useful arrive-then-wait seed without
    % crossing a later interval in which the goal becomes occupied.
    firstGoalLayerIndex=find(reachable(1:layerIndex,2) & ...
        goalLayerIsEligible(1:layerIndex),1,"first");
    if options.GoalTimeMode == "earliestArrival" && ~isempty(firstGoalLayerIndex)
        selectedGoalLayerIndex=double(waitComponentFinalLayerIndex(firstGoalLayerIndex,2));
        if selectedGoalLayerIndex==layerCount
            selectedGoalLayerIndex=firstGoalLayerIndex;
        end
        if layerIndex>=selectedGoalLayerIndex, break; end
    end
    currentNodeIndices          = find(reachable(layerIndex, :));
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        exploredNodes_units(end + 1, :) = nodePosition_units(currentNodeIndex, :); %#ok<AGROW>
        % Add the same-node transition only when the obstacle sweep permits waiting through the full layer interval.
        if waitIsClear(layerIndex, currentNodeIndex)
            waitCount = waitCount + 1;
            [reachable,spatialCost_units,parentLayerIndex,parentNodeIndex] = ...
                updateTemporalState(reachable,spatialCost_units, ...
                parentLayerIndex,parentNodeIndex,layerIndex,currentNodeIndex, ...
                layerIndex+1,currentNodeIndex,0);
        else
            rejectedCount = rejectedCount + 1;
        end
    end
    goalCostBound_units=min([goalCostBound_units;spatialCost_units(goalCanWaitToFinal,2)]);
    [motionCandidates, candidateRejections, batchSplitCount] = buildLayerCandidates(currentNodeIndices, layerTimes_s(layerIndex), layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex);
    rejectedCount = rejectedCount + candidateRejections;
    candidateBatchSplitCount = candidateBatchSplitCount + batchSplitCount;
    motionCandidateCount = size(motionCandidates, 1);
    pendingMotion    = true(motionCandidateCount, 1);

    % Keep the first clear entry per wait interval; later entries can be reached by waiting.
    while any(pendingMotion)
        queriedTargetLayers = unique(motionCandidates(pendingMotion, 3));
        for targetLayerIndex = reshape(queriedTargetLayers, 1, [])
            queryIndices = find(pendingMotion & motionCandidates(:, 3) == targetLayerIndex);
            % Use the prescribed final layer for fixed-arrival requests; earliest-arrival selection was resolved during forward search.
            if options.GoalTimeMode ~= "earliestArrival"
                trialCost_units  = reshape(spatialCost_units(layerIndex, motionCandidates(queryIndices, 1)), [], 1) + motionCandidates(queryIndices, 5);
                storedCost_units = reshape(spatialCost_units(targetLayerIndex, motionCandidates(queryIndices, 2)), [], 1);
                isDominated=trialCost_units>storedCost_units+1e-12;
                pendingMotion(queryIndices(isDominated)) = false;
                rejectedCount = rejectedCount + nnz(isDominated);
                queryIndices  = queryIndices(~isDominated);
                trialCost_units=trialCost_units(~isDominated);
                cannotImproveGoal=trialCost_units+ ...
                    distanceToGoal_units(motionCandidates(queryIndices,2))> ...
                    goalCostBound_units+1e-12;
                pendingMotion(queryIndices(cannotImproveGoal))=false;
                rejectedCount=rejectedCount+nnz(cannotImproveGoal);
                goalBoundRejectionCount=goalBoundRejectionCount+nnz(cannotImproveGoal);
                queryIndices=queryIndices(~cannotImproveGoal);
            end
            if isempty(queryIndices)
                continue;
            end
            queryIsClear = edgeIsClear(motionCandidates(queryIndices, 1), motionCandidates(queryIndices, 2), layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
            clearIndices = queryIndices(queryIsClear);
            motionCount  = motionCount + numel(clearIndices);
            for motionIndex = reshape(clearIndices, 1, [])
                sourceNodeIndex=motionCandidates(motionIndex,1);
                targetNodeIndex=motionCandidates(motionIndex,2);
                candidateTargetLayerIndex=motionCandidates(motionIndex,3);
                [reachable,spatialCost_units,parentLayerIndex,parentNodeIndex] = ...
                    updateTemporalState(reachable,spatialCost_units, ...
                    parentLayerIndex,parentNodeIndex,layerIndex,sourceNodeIndex, ...
                    candidateTargetLayerIndex,targetNodeIndex,motionCandidates(motionIndex,5));
            end
            clearGoalIndices=clearIndices(motionCandidates(clearIndices,2)==2 & ...
                goalCanWaitToFinal(motionCandidates(clearIndices,3)));
            if ~isempty(clearGoalIndices)
                clearGoalLayers=motionCandidates(clearGoalIndices,3);
                goalCostBound_units=min([goalCostBound_units; ...
                    spatialCost_units(clearGoalLayers,2)]);
            end
            rejectedCount = rejectedCount + nnz(~queryIsClear);
            pendingMotion(queryIndices) = false;
            advanceIndices = queryIndices(~queryIsClear & motionCandidates(queryIndices, 3) < motionCandidates(queryIndices, 4));
            motionCandidates(advanceIndices, 3) = motionCandidates(advanceIndices, 3) + 1;
            pendingMotion(advanceIndices) = true;
        end
    end
end
%% Section 2: Reconstruct Goal And Best-Partial Routes
deepestLayerIndex = find(any(reachable, 2), 1, "last");
[frontier_units, bestPartial_units] = deal(zeros(0, 2));
% Report no partial timed route when even the start layer has no reachable state.
if ~isempty(deepestLayerIndex)
    frontierNodeIndices = find(reachable(deepestLayerIndex, :));
    frontier_units        = nodePosition_units(frontierNodeIndices, :);
    [~, bestIndex]       = min(vecnorm(frontier_units - nodePosition_units(2, :), 2, 2));
    [bestPartial_units, ~] = reconstructTimedRoute(nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, deepestLayerIndex, frontierNodeIndices(bestIndex));
end
% Continue searching only until the first reachable goal layer in earliest-arrival mode; fixed-arrival mode must evaluate its prescribed horizon.
firstReachableGoalLayerIndex = find(reachable(:,2),1,"first");
if options.GoalTimeMode == "earliestArrival"
    firstGoalLayerIndex = find(reachable(:, 2) & goalLayerIsEligible, 1, "first");
    goalLayerIndex=firstGoalLayerIndex;
    waitSeedGoalLayerIndex=firstGoalLayerIndex;
    if ~isempty(firstGoalLayerIndex)
        waitSeedGoalLayerIndex=double(waitComponentFinalLayerIndex(firstGoalLayerIndex,2));
        if waitSeedGoalLayerIndex==layerCount
            waitSeedGoalLayerIndex=firstGoalLayerIndex;
        end
    end
else
    goalLayerIndex = find(reachable(:, 2) & (1:layerCount).' == layerCount, 1, "first");
    waitSeedGoalLayerIndex=goalLayerIndex;
end
[route_units, routeTime_s] = reconstructTimedRoute(nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex, 2);
[waitRoute_units,waitRouteTime_s]=reconstructTimedRoute(nodePosition_units, ...
    layerTimes_s,parentLayerIndex,parentNodeIndex,waitSeedGoalLayerIndex,2);
selectedGoalWindowIndex=0;
selectedGoalWindowStartTime_s=NaN;
selectedGoalWindowEndTime_s=NaN;
if ~isempty(goalLayerIndex)
    goalWindowStartLayerIndices=find(isWaitComponentStart(:,2) & nodeIsFree(:,2));
    selectedGoalWindowIndex=nnz(goalWindowStartLayerIndices<=goalLayerIndex);
    selectedGoalWindowStartLayerIndex= ...
        goalWindowStartLayerIndices(selectedGoalWindowIndex);
    selectedGoalWindowEndLayerIndex=double( ...
        waitComponentFinalLayerIndex(selectedGoalWindowStartLayerIndex,2));
    selectedGoalWindowStartTime_s= ...
        layerTimes_s(selectedGoalWindowStartLayerIndex);
    selectedGoalWindowEndTime_s=layerTimes_s(selectedGoalWindowEndLayerIndex);
end
record = struct("LayerTimes_s", layerTimes_s, ...
    "CandidateLayerCount", layerCount, "NodeCount", nodeCount, ...
    "WaitEdgeCount", waitCount, "MotionEdgeCount", motionCount, ...
    "RejectedTransitionCount", rejectedCount, ...
    "GoalCostBoundRejectionCount", goalBoundRejectionCount, ...
    "CandidateBatchSplitCount", candidateBatchSplitCount, ...
    "ExpandedCount", expandedCount, ...
    "ExploredNodes_units", exploredNodes_units, "FrontierNodes_units", frontier_units, ...
    "BestPartialRoute_units", bestPartial_units, ...
    "SelectedGoalLayerIndex", goalLayerIndex, ...
    "FirstReachableGoalLayerIndex",firstReachableGoalLayerIndex, ...
    "DynamicBoundChangedGoalLayer", ...
    ~isempty(goalLayerIndex) && goalLayerIndex~=firstReachableGoalLayerIndex, ...
    "SelectedGoalWindowIndex",selectedGoalWindowIndex, ...
    "SelectedGoalWindowStartTime_s",selectedGoalWindowStartTime_s, ...
    "SelectedGoalWindowEndTime_s",selectedGoalWindowEndTime_s, ...
    "MinimumGoalArrivalTime_s",minimumGoalArrivalTime_s, ...
    "EligibleGoalLayerCount",nnz(goalLayerIsEligible), ...
    "WaitSeedGoalLayerIndex",waitSeedGoalLayerIndex, ...
    "WaitRoute_units",waitRoute_units,"WaitRouteTime_s",waitRouteTime_s, ...
    "ReachableGoalLayerCount", nnz(reachable(:, 2)), ...
    "DynamicEdgeCheckKind","exactAffineConvexCells");
function clear = edgeIsClear(firstNodeIndices, secondNodeIndices, first_s, second_s)
    % Check the complete segment clock against exact static geometry and every
    % affine moving convex cell. A zero-length edge is a stationary point path.
    firstNodeIndices = firstNodeIndices(:);
    secondNodeIndices = secondNodeIndices(:);
    first_units = nodePosition_units(firstNodeIndices, :);
    second_units = nodePosition_units(secondNodeIndices, :);
    edgeCount    = numel(firstNodeIndices);
    clear        = true(edgeCount, 1);
    % Exact check against every time-invariant obstacle over the sub-segment
    % traversed while that obstacle exists. A zero-length wait reduces to
    % point containment inside the same predicate.
    cacheKeys = firstNodeIndices + nodeCount*(secondNodeIndices-1);
    for staticIndex = find(staticExists).'
        active_s = staticActive_s(staticIndex,:);
        overlapStart_s = max(first_s,active_s(1));
        overlapEnd_s = min(second_s,active_s(2));
        if overlapStart_s > overlapEnd_s, continue; end
        candidate = find(clear);
        if isempty(candidate), break; end
        coversWholeClock = overlapStart_s <= first_s && overlapEnd_s >= second_s;
        if coversWholeClock && ~isempty(staticEdgeCache)
            known = staticEdgeCache(cacheKeys(candidate),staticIndex);
            clear(candidate(known==2)) = false;
            candidate = candidate(known==0);
            if isempty(candidate), continue; end
        end
        startFraction = 0; endFraction = 1;
        if second_s > first_s
            startFraction = (overlapStart_s-first_s)/(second_s-first_s);
            endFraction = (overlapEnd_s-first_s)/(second_s-first_s);
        end
        subStart_units = first_units(candidate,:) + startFraction*(second_units(candidate,:)-first_units(candidate,:));
        subEnd_units = first_units(candidate,:) + endFraction*(second_units(candidate,:)-first_units(candidate,:));
        exactlyClear = obstacleAvoidance.search.checkVisibilitySegments( ...
            subStart_units,subEnd_units,staticShapes{staticIndex}, ...
            staticEdgeStart_units{staticIndex},staticEdgeEnd_units{staticIndex});
        clear(candidate) = exactlyClear;
        if coversWholeClock && ~isempty(staticEdgeCache)
            staticEdgeCache(cacheKeys(candidate),staticIndex) = 1+uint8(~exactlyClear);
        end
    end
    if isempty(dynamicCells.Regions_units) || ~any(clear)
        return;
    end
    candidate=find(clear);
    clear(candidate)=affineEdgesAreClear(first_units(candidate,:), ...
        second_units(candidate,:),first_s,second_s,dynamicCells);
end
end
%% Section 3: Local Functions
function [candidates, rejectedCount, batchSplitCount] = buildLayerCandidates(sourceNodes, sourceTime_s, layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex)
    % Enumerate [entry layer, target node, source node] in the original order.
    % Bound the temporary logical tensor while retaining the vectorized path
    % for ordinary inputs and the velocity-only duration lower bound.
    candidates = zeros(0, 5);
    rejectedCount = 0;
    batchSplitCount = 0;
    if isempty(sourceNodes), return; end
    layerCount = numel(layerTimes_s);
    nodeCount = size(nodeIsFree, 2);
    sourceCount = numel(sourceNodes);
    maximumCandidateTensorElements = 1024 ^ 2;
    sourceBatchSize = max(1, floor(maximumCandidateTensorElements / max(1, layerCount * nodeCount)));
    batchCount = ceil(sourceCount / sourceBatchSize);
    batchSplitCount = batchCount - 1;
    candidateBlocks = cell(batchCount, 1);
    for batchIndex = 1:batchCount
        firstSourceOffset = 1 + (batchIndex - 1) * sourceBatchSize;
        finalSourceOffset = min(sourceCount, batchIndex * sourceBatchSize);
        batchSources = sourceNodes(firstSourceOffset:finalSourceOffset);
        [candidateBlocks{batchIndex}, batchRejectedCount] = buildCandidateBatch(batchSources, sourceTime_s, layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex);
        rejectedCount = rejectedCount + batchRejectedCount;
    end
    candidates = vertcat(candidateBlocks{:});
end

function [candidates, rejectedCount] = buildCandidateBatch(sourceNodes, sourceTime_s, layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex)
    % Vectorize one bounded block and preserve source/target/layer ordering.
    layerCount = numel(layerTimes_s);
    nodeCount = size(nodeIsFree, 2);
    sourceCount = numel(sourceNodes);
    earliestTime_s = sourceTime_s + minimumEdgeDuration_s(sourceNodes, :).' - 1e-12;
    firstFeasibleLayers = 1 + sum(reshape(layerTimes_s, [], 1, 1) <= reshape(earliestTime_s, 1, nodeCount, sourceCount), 1);
    firstFeasibleLayers(isnan(earliestTime_s)) = layerCount + 1;
    starts = reshape(isWaitComponentStart, layerCount, nodeCount, 1) & (reshape((1:layerCount).', [], 1, 1) >= firstFeasibleLayers);
    feasiblePairs = find(firstFeasibleLayers <= layerCount);
    firstEntries = firstFeasibleLayers(feasiblePairs) + layerCount * (feasiblePairs - 1);
    targetNodes = 1 + mod(feasiblePairs - 1, nodeCount);
    firstStates = firstFeasibleLayers(feasiblePairs) + layerCount * (targetNodes - 1);
    starts(firstEntries) = nodeIsFree(firstStates);
    enabledEdges = motionEdgeExists(sourceNodes, :).';
    starts = starts & reshape(enabledEdges, 1, nodeCount, sourceCount);
    rejectedCount = nnz(enabledEdges & ~reshape(any(starts, 1), nodeCount, sourceCount));
    % Column-major enumeration matches the original source/target/layer loops.
    [entryLayers, targetNodes, sourceOffsets] = ind2sub([layerCount, nodeCount, sourceCount], find(starts));
    selectedSources = reshape(sourceNodes(sourceOffsets), [], 1);
    finalStates = entryLayers + layerCount * (targetNodes - 1);
    finalLayers = double(waitComponentFinalLayerIndex(finalStates));
    edgeIndices = selectedSources + nodeCount * (targetNodes - 1);
    candidates = [selectedSources, targetNodes, entryLayers, finalLayers, motionEdgeLengths_units(edgeIndices)];
end

function clear = affineEdgesAreClear(first_units,second_units,first_s,second_s,cells)
    % A path point and every vertex of a time cell are affine in time. Each
    % convex half-space residual is therefore quadratic; its real roots
    % partition the clock into intervals of constant inside/outside sign.
    clear=true(size(first_units,1),1);
    for cellIndex=1:numel(cells.Regions_units)
        overlapStart_s=max(first_s,cells.ActiveTimeInterval_s(cellIndex,1));
        overlapEnd_s=min(second_s,cells.ActiveTimeInterval_s(cellIndex,2));
        if overlapStart_s>overlapEnd_s,continue;end
        candidates=find(clear);
        if isempty(candidates),break;end
        edgeDuration_s=second_s-first_s;
        if edgeDuration_s>0
            startFraction=(overlapStart_s-first_s)/edgeDuration_s;
            endFraction=(overlapEnd_s-first_s)/edgeDuration_s;
        else
            startFraction=0;
            endFraction=0;
        end
        pathStart_units=first_units(candidates,:)+startFraction.* ...
            (second_units(candidates,:)-first_units(candidates,:));
        pathEnd_units=first_units(candidates,:)+endFraction.* ...
            (second_units(candidates,:)-first_units(candidates,:));
        cellDuration_s=diff(cells.ActiveTimeInterval_s(cellIndex,:));
        cellStartFraction=(overlapStart_s-cells.ActiveTimeInterval_s(cellIndex,1))/cellDuration_s;
        cellEndFraction=(overlapEnd_s-cells.ActiveTimeInterval_s(cellIndex,1))/cellDuration_s;
        regionStart_units=cells.Regions_units{cellIndex};
        regionDelta_units=cells.EndRegions_units{cellIndex}-regionStart_units;
        overlapRegionStart_units=regionStart_units+cellStartFraction.*regionDelta_units;
        overlapRegionEnd_units=regionStart_units+cellEndFraction.*regionDelta_units;
        for candidateOffset=1:numel(candidates)
            if affinePointTouchesConvex(pathStart_units(candidateOffset,:), ...
                    pathEnd_units(candidateOffset,:),overlapRegionStart_units, ...
                    overlapRegionEnd_units)
                clear(candidates(candidateOffset))=false;
            end
        end
    end
end

function touches = affinePointTouchesConvex(pointStart_units,pointEnd_units, ...
        regionStart_units,regionEnd_units)
    vertexCount=size(regionStart_units,1);
    following=[2:vertexCount,1];
    edgeStart_units=regionStart_units(following,:)-regionStart_units;
    edgeDelta_units=(regionEnd_units(following,:)-regionEnd_units)-edgeStart_units;
    relativeStart_units=pointStart_units-regionStart_units;
    relativeDelta_units=(pointEnd_units-pointStart_units)- ...
        (regionEnd_units-regionStart_units);
    coefficients_units2=[cross2(edgeDelta_units,relativeDelta_units), ...
        cross2(edgeDelta_units,relativeStart_units)+ ...
        cross2(edgeStart_units,relativeDelta_units), ...
        cross2(edgeStart_units,relativeStart_units)];
    cuts=unique([0;1;quadraticUnitRoots(coefficients_units2)]);
    probes=unique([cuts;(cuts(1:end-1)+cuts(2:end))/2]);
    middleRegion_units=(regionStart_units+regionEnd_units)/2;
    signedArea_units2=sum(cross2(middleRegion_units, ...
        middleRegion_units(following,:)))/2;
    coordinateScale_units=max(1,max(abs([pointStart_units;pointEnd_units; ...
        regionStart_units;regionEnd_units]),[],'all'));
    residualTolerance_units2=4096*eps(coordinateScale_units^2);
    residual_units2=coefficients_units2(:,1).*probes.'.^2+ ...
        coefficients_units2(:,2).*probes.'+coefficients_units2(:,3);
    if signedArea_units2>=0
        touches=any(all(residual_units2>=-residualTolerance_units2,1));
    else
        touches=any(all(residual_units2<=residualTolerance_units2,1));
    end
end

function values=quadraticUnitRoots(coefficients)
    values=zeros(0,1);
    for row=1:size(coefficients,1)
        a=coefficients(row,1);b=coefficients(row,2);c=coefficients(row,3);
        scale=max(1,max(abs(coefficients(row,:))));
        tolerance=256*eps(scale);
        if abs(a)<=tolerance
            if abs(b)>tolerance
                root=-c/b;
                if root>=0 && root<=1,values(end+1,1)=root;end %#ok<AGROW>
            end
            continue
        end
        discriminant=b*b-4*a*c;
        if discriminant<-tolerance,continue;end
        discriminant=max(0,discriminant);
        roots=[-b-sqrt(discriminant);-b+sqrt(discriminant)]/(2*a);
        values=[values;roots(roots>=0 & roots<=1)]; %#ok<AGROW>
    end
end

function value=cross2(first,second)
    value=first(:,1).*second(:,2)-first(:,2).*second(:,1);
end
function [reachable,spatialCost_units,parentLayerIndex,parentNodeIndex] = ...
        updateTemporalState(reachable,spatialCost_units,parentLayerIndex, ...
        parentNodeIndex,sourceLayerIndex,sourceNodeIndex,targetLayerIndex, ...
        targetNodeIndex,edgeLength_units)
    % Arrival layer is fixed by the state. Retain only the shortest spatial
    % ancestry, with deterministic first-discovered selection on exact ties.
    trialCost_units          = spatialCost_units(sourceLayerIndex, sourceNodeIndex) + edgeLength_units;
    storedCost_units         = spatialCost_units(targetLayerIndex, targetNodeIndex);
    if trialCost_units>=storedCost_units-1e-12
        return;
    end
    reachable(targetLayerIndex, targetNodeIndex) = true;
    spatialCost_units(targetLayerIndex, targetNodeIndex) = trialCost_units;
    parentLayerIndex(targetLayerIndex, targetNodeIndex) = uint32(sourceLayerIndex);
    parentNodeIndex(targetLayerIndex, targetNodeIndex) = uint16(sourceNodeIndex);
end
function [route_units, routeTime_s] = reconstructTimedRoute(nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex, goalNodeIndex)
    % Follow temporal parents backward, including waits.
    route_units   = zeros(0, 2);
    routeTime_s = zeros(0, 1);
    if isempty(goalLayerIndex)
        return;
    end
    layerPath = goalLayerIndex;
    nodePath  = goalNodeIndex;
    while ~(layerPath(1) == 1 && nodePath(1) == 1)
        priorLayerIndex = double(parentLayerIndex(layerPath(1), nodePath(1)));
        priorNodeIndex  = double(parentNodeIndex(layerPath(1), nodePath(1)));
        % Stop backtracking at the recorded start sentinel; encountering it earlier would indicate corrupt parent data.
        if priorLayerIndex == 0 || priorNodeIndex == 0
            route_units   = zeros(0, 2);
            routeTime_s = zeros(0, 1);
            return;
        end
        layerPath = [priorLayerIndex; layerPath]; %#ok<AGROW>
        nodePath  = [priorNodeIndex; nodePath]; %#ok<AGROW>
    end
    route_units   = nodePosition_units(nodePath, :);
    routeTime_s = layerTimes_s(layerPath);
end
