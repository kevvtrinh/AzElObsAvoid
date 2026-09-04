function [routes_deg, classPattern, searchRecord] = ...
        searchDistinctSpatialRoutes( ...
        edgeCost_deg, nodePosition_deg, obstacleReferencePoints_deg, ...
        maximumClassCount, edgeCheck, options)
%% Section 0: Header & Readme
% SYNTAX
%   [routes_deg, classPattern, searchRecord] = ...
%       obstacleAvoidance.search.searchDistinctSpatialRoutes( ...
%       edgeCost_deg, nodePosition_deg, obstacleReferencePoints_deg, ...
%       maximumClassCount, edgeCheck)
%   [routes_deg, classPattern, searchRecord] = ...
%       obstacleAvoidance.search.searchDistinctSpatialRoutes( ...
%       edgeCost_deg, nodePosition_deg, obstacleReferencePoints_deg, ...
%       maximumClassCount, edgeCheck, options)
%**************************************************************************
% PURPOSE
%   - Search visibility-node and route-class states for shortest routes that
%     pass obstacle reference points in distinct ways.
%   - Shorten each route using only visible chords that preserve its class.
%**************************************************************************
% INPUTS
%   - edgeCost_deg (N-by-N numeric matrix)
%       Symmetric finite visibility-edge costs with start and goal first.
%   - nodePosition_deg (N-by-2 numeric matrix)
%       Visibility nodes in [azimuth elevation] coordinates.
%   - obstacleReferencePoints_deg (R-by-2 numeric matrix)
%       Points used to tell route classes apart.
%   - maximumClassCount (nonnegative integer scalar)
%       Requested number of distinct classes; zero disables the search.
%   - edgeCheck (scalar function handle)
%       Exact proposal-geometry chord predicate used during cleanup.
%   - options (resolved scalar planner-options struct, optional)
%       CancellationCheckFcn enables cooperative caller cancellation.
%**************************************************************************
% OUTPUTS
%   - routes_deg (cell column of N-by-2 numeric arrays)
%       Deterministically ordered shortest routes for distinct classes.
%   - classPattern (M-by-R integer-valued numeric matrix)
%       Route-class pattern corresponding to every returned route.
%   - searchRecord (scalar struct)
%       Search, frontier, best-partial, and cleanup evidence.
%**************************************************************************
% UNITS
%   - Position and edge cost are degrees; class patterns are dimensionless.
%**************************************************************************

%% Section 1: Expand Route-Class Visibility States

if nargin < 6 || isempty(options)
    options = struct("CancellationCheckFcn", []);
end
nodeCount = size(nodePosition_deg, 1);
referenceCount = size(obstacleReferencePoints_deg, 1);
classWidth = max(1, referenceCount);
routes_deg = cell(0, 1);
classPattern = zeros(0, referenceCount);
stateCapacity = max(16, 2 * nodeCount);
stateCount = 1;
stateNode = zeros(1, stateCapacity);
stateNode(1) = 1;
stateClass = zeros(stateCapacity, classWidth);
stateCost_deg = Inf(1, stateCapacity);
stateCost_deg(1) = 0;
parentState = zeros(1, stateCapacity);
closed = false(1, stateCapacity);
stateLookup = dictionary(stateKey(1, stateClass(1, :)), 1);
rejectedCount = 0;
phase = zeros(nodeCount, classWidth);
if referenceCount > 0
    phase(:, 1:referenceCount) = atan2( ...
        nodePosition_deg(:, 2) - obstacleReferencePoints_deg(:, 2).', ...
        nodePosition_deg(:, 1) - obstacleReferencePoints_deg(:, 1).');
end
reference = principalAngle(phase - phase(1, :)) / (2 * pi);
classFunction = @(route_deg) routeClassPattern( ...
    route_deg, obstacleReferencePoints_deg);
cleanup = struct("CandidateCount", 0, "VisibilityRejectedCount", 0, ...
    "HomologyRejectedCount", 0, "AcceptedCount", 0, ...
    "LengthReduction_deg", 0);
cleanupFields = string(fieldnames(cleanup));
pollStride = 32;
expandedCount = 0;

% Without an arbitrary winding cap, a disconnected cyclic component could
% contain infinitely many lifted states. Prove ordinary graph reachability
% first. Expand each reached adjacency row once so this guard stays linear in
% the stored graph size.
reachableNode = false(1, nodeCount);
reachableNode(1) = true;
reachableQueue = zeros(1, nodeCount);
reachableQueue(1) = 1;
queueHead = 1;
queueTail = 1;
while queueHead <= queueTail
    currentNode = reachableQueue(queueHead);
    queueHead = queueHead + 1;
    newReachableNode = find( ...
        isfinite(edgeCost_deg(currentNode, :)) & ~reachableNode);
    reachableNode(newReachableNode) = true;
    newReachableCount = numel(newReachableNode);
    reachableQueue(queueTail + (1:newReachableCount)) = newReachableNode;
    queueTail = queueTail + newReachableCount;
end
goalIsReachable = nodeCount >= 2 && reachableNode(2);

while goalIsReachable && numel(routes_deg) < maximumClassCount
    expandedCount = expandedCount + 1;
    if mod(expandedCount - 1, pollStride) == 0
        obstacleAvoidance.input.throwIfCancellationRequested(options);
    end
    unsettledCost_deg = stateCost_deg(1:stateCount);
    unsettledCost_deg(closed(1:stateCount)) = Inf;
    [currentCost_deg, currentState] = min(unsettledCost_deg);
    if ~isfinite(currentCost_deg)
        break;
    end
    closed(currentState) = true;
    currentNode = stateNode(currentState);
    if currentNode == 2
        statePath = reconstructStatePath(parentState, currentState);
        route_deg = nodePosition_deg(stateNode(statePath), :);
        requiredClass = stateClass(currentState, 1:referenceCount);
        [route_deg, routeCleanup] = shortenVisibilityRoute( ...
            route_deg, edgeCheck, classFunction, requiredClass);
        routes_deg{end + 1, 1} = route_deg; %#ok<AGROW>
        if referenceCount > 0
            classPattern(end + 1, :) = requiredClass; %#ok<AGROW>
        else
            classPattern = zeros(numel(routes_deg), 0);
        end
        for fieldIndex = 1:numel(cleanupFields)
            fieldName = cleanupFields(fieldIndex);
            cleanup.(fieldName) = cleanup.(fieldName) + ...
                routeCleanup.(fieldName);
        end
        continue;
    end
    neighbors = find(isfinite(edgeCost_deg(currentNode, :)));
    for neighbor = reshape(neighbors, 1, [])
        if neighbor == currentNode
            rejectedCount = rejectedCount + 1;
            continue;
        end
        step = principalAngle( ...
            phase(neighbor, :) - phase(currentNode, :)) / (2 * pi);
        classStep = round(reference(currentNode, :) + step - ...
            reference(neighbor, :));
        trialClass = stateClass(currentState, :) + classStep;
        trialKey = stateKey(neighbor, trialClass);
        if ~isKey(stateLookup, trialKey)
            stateCount = stateCount + 1;
            if stateCount > stateCapacity
                stateCapacity = 2 * stateCapacity;
                [stateNode, stateClass, stateCost_deg, parentState, closed] = ...
                    growStateStorage(stateNode, stateClass, stateCost_deg, ...
                    parentState, closed, stateCapacity);
            end
            nextState = stateCount;
            stateNode(nextState) = neighbor;
            stateClass(nextState, :) = trialClass;
            stateCost_deg(nextState) = Inf;
            parentState(nextState) = 0;
            closed(nextState) = false;
            stateLookup(trialKey) = nextState;
        else
            nextState = stateLookup(trialKey);
        end
        if closed(nextState)
            rejectedCount = rejectedCount + 1;
            continue;
        end
        trialCost_deg = currentCost_deg + ...
            edgeCost_deg(currentNode, neighbor);
        if trialCost_deg < stateCost_deg(nextState) - 1e-12
            stateCost_deg(nextState) = trialCost_deg;
            parentState(nextState) = currentState;
        end
    end
end

%% Section 2: Assemble Search And Cleanup Evidence

finiteState = isfinite(stateCost_deg(1:stateCount));
frontierState = finiteState & ~closed(1:stateCount);
activeNode = stateNode(1:stateCount);
expandedState = closed(1:stateCount) & activeNode ~= 2;
bestPartial_deg = zeros(0, 2);
if any(finiteState)
    finiteIndex = find(finiteState);
    [~, bestIndex] = min(vecnorm( ...
        nodePosition_deg(activeNode(finiteIndex), :) - ...
        nodePosition_deg(2, :), 2, 2));
    bestStatePath = reconstructStatePath( ...
        parentState, finiteIndex(bestIndex));
    bestPartial_deg = nodePosition_deg(stateNode(bestStatePath), :);
end
stoppedAtClassLimit = maximumClassCount > 0 && ...
    numel(routes_deg) >= maximumClassCount && any(frontierState);
searchRecord = struct("ExpandedCount", nnz(expandedState), ...
    "RejectedTransitionCount", rejectedCount, ...
    "ExploredNodes_deg", nodePosition_deg(activeNode(expandedState), :), ...
    "FrontierNodes_deg", nodePosition_deg(activeNode(frontierState), :), ...
    "BestPartialRoute_deg", bestPartial_deg, "StateCount", stateCount, ...
    "Truncated", stoppedAtClassLimit, ...
    "StoppedAtClassLimit", stoppedAtClassLimit, ...
    "RouteCleanupAttemptedCount", numel(routes_deg), ...
    "RouteCleanupCandidateCount", cleanup.CandidateCount, ...
    "RouteCleanupVisibilityRejectedCount", cleanup.VisibilityRejectedCount, ...
    "RouteCleanupHomologyRejectedCount", cleanup.HomologyRejectedCount, ...
    "RouteCleanupAcceptedCount", cleanup.AcceptedCount, ...
    "RouteCleanupLengthReduction_deg", cleanup.LengthReduction_deg);
if maximumClassCount == 0
    searchRecord.FrontierNodes_deg = zeros(0, 2);
end
end

%% Section 3: Local Functions

function statePath = reconstructStatePath(parentState, targetState)
% Recover stored augmented-state ancestry without recomputing decisions.
statePath = targetState;
while statePath(1) ~= 1
    statePath = [parentState(statePath(1)), statePath]; %#ok<AGROW>
end
end

function key = stateKey(nodeIndex, classPattern)
% Encode one augmented state without floating-point or ordering ambiguity.
key = strjoin([string(nodeIndex), ...
    string(double(classPattern(:).'))], ":");
end

function [stateNode, stateClass, stateCost_deg, parentState, closed] = ...
        growStateStorage(stateNode, stateClass, stateCost_deg, ...
        parentState, closed, newCapacity)
% Double state storage without changing the input-driven search frontier.
oldCapacity = numel(stateNode);
nextNode = zeros(1, newCapacity);
nextNode(1:oldCapacity) = stateNode;
stateNode = nextNode;
nextClass = zeros(newCapacity, size(stateClass, 2));
nextClass(1:oldCapacity, :) = stateClass;
stateClass = nextClass;
nextCost_deg = Inf(1, newCapacity);
nextCost_deg(1:oldCapacity) = stateCost_deg;
stateCost_deg = nextCost_deg;
nextParent = zeros(1, newCapacity);
nextParent(1:oldCapacity) = parentState;
parentState = nextParent;
nextClosed = false(1, newCapacity);
nextClosed(1:oldCapacity) = closed;
closed = nextClosed;
end

function pattern = routeClassPattern(route_deg, referencePoints_deg)
% Evaluate the same open-route class pattern used by search transitions.
pattern = zeros(1, size(referencePoints_deg, 1));
if isempty(referencePoints_deg)
    return;
end
phase = atan2(route_deg(:, 2) - referencePoints_deg(:, 2).', ...
    route_deg(:, 1) - referencePoints_deg(:, 1).');
reference = principalAngle(phase - phase(1, :)) / (2 * pi);
for edgeIndex = 1:size(route_deg, 1) - 1
    step = principalAngle( ...
        phase(edgeIndex + 1, :) - phase(edgeIndex, :)) / (2 * pi);
    pattern = pattern + round( ...
        reference(edgeIndex, :) + step - reference(edgeIndex + 1, :));
end
end

function angle = principalAngle(angle)
% Normalize angular change to the deterministic principal interval.
angle = atan2(sin(angle), cos(angle));
end

function [cleanedRoute_deg, record] = shortenVisibilityRoute( ...
        route_deg, visibilityFunction, signatureFunction, requiredSignature)
% Shorten a route only with visible chords that preserve its route class.
cleanedRoute_deg = route_deg;
initialLength_deg = obstacleAvoidance.geometry.routeLength(route_deg);
record = struct("CandidateCount", 0, "VisibilityRejectedCount", 0, ...
    "HomologyRejectedCount", 0, "AcceptedCount", 0, ...
    "LengthReduction_deg", 0);
if size(route_deg, 1) < 3 || ...
        ~isequal(signatureFunction(route_deg), requiredSignature)
    return;
end
while size(cleanedRoute_deg, 1) >= 3
    currentLength_deg = ...
        obstacleAvoidance.geometry.routeLength(cleanedRoute_deg);
    lengthTolerance_deg = max(1e-12, 1e-12 * currentLength_deg);
    bestReduction_deg = 0;
    bestRoute_deg = cleanedRoute_deg;
    for firstIndex = 1:size(cleanedRoute_deg, 1) - 2
        for secondIndex = firstIndex + 2:size(cleanedRoute_deg, 1)
            record.CandidateCount = record.CandidateCount + 1;
            reduction_deg = obstacleAvoidance.geometry.routeLength( ...
                cleanedRoute_deg(firstIndex:secondIndex, :)) - ...
                norm(cleanedRoute_deg(secondIndex, :) - ...
                cleanedRoute_deg(firstIndex, :));
            if reduction_deg <= lengthTolerance_deg
                continue;
            end
            if ~visibilityFunction(cleanedRoute_deg(firstIndex, :), ...
                    cleanedRoute_deg(secondIndex, :))
                record.VisibilityRejectedCount = ...
                    record.VisibilityRejectedCount + 1;
                continue;
            end
            candidate_deg = [cleanedRoute_deg(1:firstIndex, :); ...
                cleanedRoute_deg(secondIndex:end, :)];
            if ~isequal(signatureFunction(candidate_deg), requiredSignature)
                record.HomologyRejectedCount = ...
                    record.HomologyRejectedCount + 1;
                continue;
            end
            if reduction_deg <= bestReduction_deg + lengthTolerance_deg
                continue;
            end
            bestReduction_deg = reduction_deg;
            bestRoute_deg = candidate_deg;
        end
    end
    if bestReduction_deg <= lengthTolerance_deg
        break;
    end
    cleanedRoute_deg = bestRoute_deg;
    record.AcceptedCount = record.AcceptedCount + 1;
end
record.LengthReduction_deg = initialLength_deg - ...
    obstacleAvoidance.geometry.routeLength(cleanedRoute_deg);
end
