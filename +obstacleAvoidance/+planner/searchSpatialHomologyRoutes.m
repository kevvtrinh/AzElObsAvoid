function [routes, signatures, record] = searchSpatialHomologyRoutes( ...
        cost_deg, positions_deg, representatives_deg, maximumCount, visibilityFunction)
%% Section 0: Header & Readme
% SYNTAX
%   [routes, signatures, record] = ...
%       obstacleAvoidance.planner.searchSpatialHomologyRoutes( ...
%       cost_deg, positions_deg, representatives_deg, maximumCount, ...
%       visibilityFunction)
%**************************************************************************
% PURPOSE
%   - Search graph-node and winding-signature states for shortest distinct
%     route classes, then shorten each using certified visible same-class chords.
%**************************************************************************
% INPUTS
%   - cost_deg (N-by-N numeric matrix)
%       Symmetric finite visibility-edge costs with start and goal first.
%   - positions_deg (N-by-2 numeric matrix)
%       Visibility nodes in [azimuth elevation] coordinates.
%   - representatives_deg (R-by-2 numeric matrix)
%       One interior winding reference for each occupied region.
%   - maximumCount (nonnegative integer scalar)
%       Requested number of distinct classes; zero disables the search.
%   - visibilityFunction (scalar function handle)
%       Exact proposal-geometry chord predicate used during cleanup.
%**************************************************************************
% OUTPUTS
%   - routes (cell vector)
%       Deterministically ordered shortest routes for distinct signatures.
%   - signatures (M-by-R int8 matrix)
%       Winding signature corresponding to every returned route.
%   - record (scalar struct)
%       Search/frontier/best-partial and decision-faithful cleanup evidence.
%**************************************************************************
% UNITS
%   - Position and edge cost are degrees; winding values are dimensionless.
%**************************************************************************

%% Section 1: Expand Augmented Visibility States

nodeCount = size(positions_deg, 1);
representativeCount = size(representatives_deg, 1);
signatureWidth = max(1, representativeCount);
routes = cell(0, 1);
signatures = zeros(0, representativeCount, "int8");
stateCapacity = max(16, 2 * nodeCount);
stateCount = 1;
stateNode = zeros(1, stateCapacity);
stateNode(1) = 1;
stateSignature = zeros(stateCapacity, signatureWidth, "int8");
stateCost_deg = Inf(1, stateCapacity);
stateCost_deg(1) = 0;
parentState = zeros(1, stateCapacity);
closed = false(1, stateCapacity);
rejectedCount = 0;
phase = zeros(nodeCount, signatureWidth);
if representativeCount > 0
    phase(:, 1:representativeCount) = atan2( ...
        positions_deg(:, 2) - representatives_deg(:, 2).', ...
        positions_deg(:, 1) - representatives_deg(:, 1).');
end
reference = principalAngle(phase - phase(1, :)) / (2 * pi);
signatureFunction = @(route_deg) routeSignature(route_deg, representatives_deg);
cleanup = struct("CandidateCount", 0, "VisibilityRejectedCount", 0, ...
    "HomologyRejectedCount", 0, "AcceptedCount", 0, "LengthReduction_deg", 0);
cleanupFields = string(fieldnames(cleanup));
while numel(routes) < maximumCount
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
        route_deg = positions_deg(stateNode(statePath), :);
        signature = stateSignature(currentState, 1:representativeCount);
        [route_deg, routeCleanup] = ...
            obstacleAvoidance.geometry.shortenVisibilityRoute( ...
            route_deg, visibilityFunction, signatureFunction, signature);
        if routeCleanup.AcceptedCount > 0
            [route_deg, alternativeCleanup] = cleanupRouteAlternatives( ...
                route_deg, stateNode(statePath), cost_deg, positions_deg, ...
                visibilityFunction, signatureFunction, signature);
            for fieldIndex = 1:numel(cleanupFields)
                fieldName = cleanupFields(fieldIndex);
                routeCleanup.(fieldName) = routeCleanup.(fieldName) + ...
                    alternativeCleanup.(fieldName);
            end
        end
        routes{end + 1, 1} = route_deg; %#ok<AGROW>
        if representativeCount > 0
            signatures(end + 1, :) = signature; %#ok<AGROW>
        else
            signatures = zeros(numel(routes), 0, "int8");
        end
        for fieldIndex = 1:numel(cleanupFields)
            fieldName = cleanupFields(fieldIndex);
            cleanup.(fieldName) = cleanup.(fieldName) + routeCleanup.(fieldName);
        end
        continue;
    end
    neighbors = find(isfinite(cost_deg(currentNode, :)));
    for neighbor = reshape(neighbors, 1, [])
        if neighbor == currentNode
            rejectedCount = rejectedCount + 1;
            continue;
        end
        step = principalAngle(phase(neighbor, :) - phase(currentNode, :)) / (2 * pi);
        signatureStep = round(reference(currentNode, :) + step - reference(neighbor, :));
        trialSignature = int8(double(stateSignature(currentState, :)) + signatureStep);
        if any(abs(double(trialSignature)) > 1)
            rejectedCount = rejectedCount + 1;
            continue;
        end
        sameState = stateNode(1:stateCount) == neighbor & ...
            all(stateSignature(1:stateCount, :) == trialSignature, 2).';
        nextState = find(sameState, 1);
        if isempty(nextState)
            stateCount = stateCount + 1;
            if stateCount > stateCapacity
                stateCapacity = 2 * stateCapacity;
                [stateNode, stateSignature, stateCost_deg, parentState, closed] = ...
                    growStateStorage(stateNode, stateSignature, stateCost_deg, ...
                    parentState, closed, stateCapacity);
            end
            nextState = stateCount;
            stateNode(nextState) = neighbor;
            stateSignature(nextState, :) = trialSignature;
            stateCost_deg(nextState) = Inf;
            parentState(nextState) = 0;
            closed(nextState) = false;
        elseif closed(nextState)
            rejectedCount = rejectedCount + 1;
            continue;
        end
        trialCost_deg = currentCost_deg + cost_deg(currentNode, neighbor);
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
        positions_deg(activeNode(finiteIndex), :) - positions_deg(2, :), 2, 2));
    bestStatePath = reconstructStatePath(parentState, finiteIndex(bestIndex));
    bestPartial_deg = positions_deg(stateNode(bestStatePath), :);
end
record = struct("ExpandedCount", nnz(expandedState), ...
    "RejectedTransitionCount", rejectedCount, ...
    "ExploredNodes_deg", positions_deg(activeNode(expandedState), :), ...
    "FrontierNodes_deg", positions_deg(activeNode(frontierState), :), ...
    "BestPartialRoute_deg", bestPartial_deg, "StateCount", stateCount, ...
    "Truncated", false, "RouteCleanupAttemptedCount", numel(routes), ...
    "RouteCleanupCandidateCount", cleanup.CandidateCount, ...
    "RouteCleanupVisibilityRejectedCount", cleanup.VisibilityRejectedCount, ...
    "RouteCleanupHomologyRejectedCount", cleanup.HomologyRejectedCount, ...
    "RouteCleanupAcceptedCount", cleanup.AcceptedCount, ...
    "RouteCleanupLengthReduction_deg", cleanup.LengthReduction_deg);
if maximumCount == 0
    record.FrontierNodes_deg = zeros(0, 2);
end
end

%% Section 3: Local Functions

function [bestRoute_deg, record] = cleanupRouteAlternatives( ...
        bestRoute_deg, primaryNodePath, cost_deg, positions_deg, ...
        visibilityFunction, signatureFunction, requiredSignature)
% Evaluate each unique graph-valid waypoint route for one discovered class.
fields = ["CandidateCount", "VisibilityRejectedCount", ...
    "HomologyRejectedCount", "AcceptedCount", "LengthReduction_deg"];
record = struct("CandidateCount", 0, "VisibilityRejectedCount", 0, ...
    "HomologyRejectedCount", 0, "AcceptedCount", 0, "LengthReduction_deg", 0);
searchGraph = graph(cost_deg, "upper", "omitselfloops");
seenPaths = cell(size(positions_deg, 1) + 1, 1);
seenPaths{1} = primaryNodePath;
seenCount = 1;
bestLength_deg = routeLength(bestRoute_deg);
for waypoint = 1:size(positions_deg, 1)
    firstPath = shortestpath(searchGraph, 1, waypoint);
    secondPath = shortestpath(searchGraph, waypoint, 2);
    if isempty(firstPath) || isempty(secondPath)
        continue;
    end
    nodePath = [firstPath, secondPath(2:end)];
    isDuplicate = false;
    for seenIndex = 1:seenCount
        if isequal(nodePath, seenPaths{seenIndex})
            isDuplicate = true;
            break;
        end
    end
    if isDuplicate
        continue;
    end
    seenCount = seenCount + 1;
    seenPaths{seenCount} = nodePath;
    candidate_deg = positions_deg(nodePath, :);
    candidateSignature = signatureFunction(candidate_deg);
    [cleaned_deg, cleanup] = obstacleAvoidance.geometry.shortenVisibilityRoute( ...
        candidate_deg, visibilityFunction, signatureFunction, candidateSignature);
    for fieldIndex = 1:numel(fields)
        fieldName = fields(fieldIndex);
        record.(fieldName) = record.(fieldName) + cleanup.(fieldName);
    end
    cleanedLength_deg = routeLength(cleaned_deg);
    if isequal(candidateSignature, requiredSignature) && ...
            cleanedLength_deg < bestLength_deg - 1e-12
        bestRoute_deg = cleaned_deg;
        bestLength_deg = cleanedLength_deg;
    end
end
end

function statePath = reconstructStatePath(parentState, targetState)
% Recover stored augmented-state ancestry without recomputing decisions.
statePath = targetState;
while statePath(1) ~= 1
    statePath = [parentState(statePath(1)), statePath]; %#ok<AGROW>
end
end

function length_deg = routeLength(route_deg)
% Measure Euclidean polyline length for deterministic portfolio selection.
length_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
end

function [stateNode, stateSignature, stateCost_deg, parentState, closed] = ...
        growStateStorage(stateNode, stateSignature, stateCost_deg, ...
        parentState, closed, newCapacity)
% Double state storage without changing the input-driven search frontier.
oldCapacity = numel(stateNode);
nextNode = zeros(1, newCapacity);
nextNode(1:oldCapacity) = stateNode;
stateNode = nextNode;
nextSignature = zeros(newCapacity, size(stateSignature, 2), "int8");
nextSignature(1:oldCapacity, :) = stateSignature;
stateSignature = nextSignature;
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

function signature = routeSignature(route_deg, representatives_deg)
% Evaluate the same open-route winding signature used by search transitions.
signature = zeros(1, size(representatives_deg, 1), "int8");
if isempty(representatives_deg)
    return;
end
phase = atan2(route_deg(:, 2) - representatives_deg(:, 2).', ...
    route_deg(:, 1) - representatives_deg(:, 1).');
reference = principalAngle(phase - phase(1, :)) / (2 * pi);
for edgeIndex = 1:size(route_deg, 1) - 1
    step = principalAngle(phase(edgeIndex + 1, :) - phase(edgeIndex, :)) / (2 * pi);
    signature = int8(double(signature) + round( ...
        reference(edgeIndex, :) + step - reference(edgeIndex + 1, :)));
end
end

function angle = principalAngle(angle)
% Normalize angular change to the deterministic principal interval.
angle = atan2(sin(angle), cos(angle));
end
