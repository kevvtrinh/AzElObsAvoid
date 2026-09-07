function [routes_units, classPattern, searchRecord] = searchDistinctSpatialRoutes(edgeCost_units, nodePosition_units, obstacleReferencePoints_units, maximumClassCount, edgeCheck)
%% Section 0: Header & Readme
% SYNTAX
%   [routes_units, classPattern, searchRecord] = ...
%       obstacleAvoidance.search.searchDistinctSpatialRoutes( ...
%       edgeCost_units, nodePosition_units, obstacleReferencePoints_units, ...
%       maximumClassCount, edgeCheck)
%
% PURPOSE
%   - Search visibility-node and route-class states for shortest routes that
%     pass obstacle reference points in distinct ways.
%   - Shorten each route using only visible chords that preserve its class.
%
% INPUTS
%   - edgeCost_units (N-by-N numeric matrix)
%       Symmetric finite visibility-edge costs with start and goal first.
%   - nodePosition_units (N-by-2 numeric matrix)
%       Visibility nodes in [x y] coordinates.
%   - obstacleReferencePoints_units (R-by-2 numeric matrix)
%       Points used to tell route classes apart.
%   - maximumClassCount (nonnegative integer scalar)
%       Requested number of distinct classes; zero disables the search.
%   - edgeCheck (scalar function handle)
%       Exact proposal-geometry chord predicate used during cleanup.
%
% OUTPUTS
%   - routes_units (cell column of N-by-2 numeric arrays)
%       Deterministically ordered shortest routes for distinct classes.
%   - classPattern (M-by-R integer-valued numeric matrix)
%       Route-class pattern corresponding to every returned route.
%   - searchRecord (scalar struct)
%       Search, frontier, best-partial, and cleanup evidence.
%
% UNITS
%   - Position and edge cost are coordinate units; class patterns are dimensionless.
%

%% Section 1: Expand Route-Class Visibility States

nodeCount      = size(nodePosition_units, 1);
referenceCount = size(obstacleReferencePoints_units, 1);
classWidth     = max(1, referenceCount);
routes_units     = cell(0, 1);
classPattern   = zeros(0, referenceCount);
stateCapacity  = max(16, 2 * nodeCount);
stateCount     = 1;
stateNode      = zeros(1, stateCapacity);
stateNode(1) = 1;
stateClass    = zeros(stateCapacity, classWidth);
stateCost_units = Inf(1, stateCapacity);
stateCost_units(1) = 0;
parentState   = zeros(1, stateCapacity);
closed        = false(1, stateCapacity);
stateLookup   = dictionary(stateKey(1, stateClass(1, :)), 1);
rejectedCount = 0;
phase         = zeros(nodeCount, classWidth);
% Track topological signatures only when reference cuts exist; otherwise ordinary node identity fully defines the state.
if referenceCount > 0
    phase(:, 1:referenceCount) = atan2(nodePosition_units(:, 2) - obstacleReferencePoints_units(:, 2).', nodePosition_units(:, 1) - obstacleReferencePoints_units(:, 1).');
end
reference     = principalAngle(phase - phase(1, :)) / (2 * pi);
classFunction = @(route_units) routeClassPattern(route_units, obstacleReferencePoints_units);
cleanup       = struct();
cleanup.CandidateCount          = 0;
cleanup.VisibilityRejectedCount = 0;
cleanup.RouteClassRejectedCount = 0;
cleanup.AcceptedCount           = 0;
cleanup.LengthReduction_units     = 0;
cleanupFields = string(fieldnames(cleanup));
expandedCount = 0;

% Check ordinary reachability first: a disconnected cyclic component
% can otherwise generate infinitely many winding states.
component       = conncomp(graph(isfinite(edgeCost_units), "upper"));
goalIsReachable = nodeCount >= 2 && component(1) == component(2);

% Continue iterating until the stopping condition for find distinct spatial routes is satisfied.
while goalIsReachable && numel(routes_units) < maximumClassCount
    expandedCount     = expandedCount + 1;
    unsettledCost_units = stateCost_units(1:stateCount);
    unsettledCost_units(closed(1:stateCount)) = Inf;
    [currentCost_units, currentState] = min(unsettledCost_units);
    % Stop Dijkstra expansion when every remaining frontier state is unreachable.
    if ~isfinite(currentCost_units)
        break;
    end
    closed(currentState) = true;
    currentNode = stateNode(currentState);
    % Stop expansion after dequeuing the goal because Dijkstra has then proven its cheapest state path.
    if currentNode == 2
        statePath     = reconstructStatePath(parentState, currentState);
        route_units     = nodePosition_units(stateNode(statePath), :);
        requiredClass = stateClass(currentState, 1:referenceCount);
        [route_units, routeCleanup] = shortenVisibilityRoute(route_units, edgeCheck, classFunction, requiredClass);
        routes_units{end + 1, 1} = route_units; %#ok<AGROW>
        % Track topological signatures only when reference cuts exist; otherwise ordinary node identity fully defines the state.
        if referenceCount > 0
            classPattern(end + 1, :) = requiredClass; %#ok<AGROW>
        else
            classPattern = zeros(numel(routes_units), 0);
        end
        % Apply the required validation or transfer to each field.
        for fieldIndex = 1:numel(cleanupFields)
            fieldName = cleanupFields(fieldIndex);
            cleanup.(fieldName) = cleanup.(fieldName) + routeCleanup.(fieldName);
        end
        continue;
    end
    neighbors = find(isfinite(edgeCost_units(currentNode, :)));
    % Process each geometric neighbor while constructing or checking the region topology.
    for neighbor = reshape(neighbors, 1, [])
        % Skip self-edges because they cannot improve cost or topology.
        if neighbor == currentNode
            rejectedCount = rejectedCount + 1;
            continue;
        end
        step       = principalAngle(phase(neighbor, :) - phase(currentNode, :)) / (2 * pi);
        classStep  = round(reference(currentNode, :) + step - reference(neighbor, :));
        trialClass = stateClass(currentState, :) + classStep;
        trialKey   = stateKey(neighbor, trialClass);
        if ~isKey(stateLookup, trialKey)
            stateCount = stateCount + 1;
            if stateCount > stateCapacity
                stateCapacity = 2 * stateCapacity;
                [stateNode, stateClass, stateCost_units, parentState, closed] = growStateStorage(stateNode, stateClass, stateCost_units, parentState, closed, stateCapacity);
            end
            nextState = stateCount;
            stateNode(nextState) = neighbor;
            stateClass(nextState, :) = trialClass;
            stateCost_units(nextState) = Inf;
            parentState(nextState) = 0;
            closed(nextState) = false;
            stateLookup(trialKey) = nextState;
        else
            nextState = stateLookup(trialKey);
        end
        % Do not reopen finalized Dijkstra states; their cheapest cost is already proven.
        if closed(nextState)
            rejectedCount = rejectedCount + 1;
            continue;
        end
        trialCost_units = currentCost_units + edgeCost_units(currentNode, neighbor);
        % Replace the stored search state only with a strictly cheaper route; equal or worse routes keep the existing parent.
        if trialCost_units < stateCost_units(nextState) - 1e-12
            stateCost_units(nextState) = trialCost_units;
            parentState(nextState) = currentState;
        end
    end
end

%% Section 2: Assemble Search And Cleanup Evidence

finiteState     = isfinite(stateCost_units(1:stateCount));
frontierState   = finiteState & ~closed(1:stateCount);
activeNode      = stateNode(1:stateCount);
expandedState   = closed(1:stateCount) & activeNode ~= 2;
bestPartial_units = zeros(0, 2);
% Retain the cheapest reachable state as partial progress when the goal remains unreachable.
if any(finiteState)
    finiteIndex = find(finiteState);
    [~, bestIndex] = min(vecnorm(nodePosition_units(activeNode(finiteIndex), :) - nodePosition_units(2, :), 2, 2));
    bestStatePath   = reconstructStatePath(parentState, finiteIndex(bestIndex));
    bestPartial_units = nodePosition_units(stateNode(bestStatePath), :);
end
stoppedAtClassLimit = maximumClassCount > 0 && numel(routes_units) >= maximumClassCount && any(frontierState);
searchRecord        = struct("ExpandedCount", nnz(expandedState), ...
    "RejectedTransitionCount", rejectedCount, ...
    "ExploredNodes_units", nodePosition_units(activeNode(expandedState), :), ...
    "FrontierNodes_units", nodePosition_units(activeNode(frontierState), :), ...
    "BestPartialRoute_units", bestPartial_units, "StateCount", stateCount, ...
    "Truncated", stoppedAtClassLimit, ...
    "StoppedAtClassLimit", stoppedAtClassLimit, ...
    "RouteShorteningAttemptedCount", numel(routes_units), ...
    "RouteShorteningCandidateCount", cleanup.CandidateCount, ...
    "RouteShorteningVisibilityRejectedCount", cleanup.VisibilityRejectedCount, ...
    "RouteShorteningRouteClassRejectedCount", cleanup.RouteClassRejectedCount, ...
    "RouteShorteningAcceptedCount", cleanup.AcceptedCount, ...
    "RouteShorteningLengthReduction_units", cleanup.LengthReduction_units);
if maximumClassCount == 0
    searchRecord.FrontierNodes_units = zeros(0, 2);
end
end

%% Section 3: Local Functions

function statePath = reconstructStatePath(parentState, targetState)
    % Reconstruct the route from stored parent states.
    statePath = targetState;
    % Continue iterating until the stopping condition for complete reconstruct state path is satisfied.
    while statePath(1) ~= 1
        statePath = [parentState(statePath(1)), statePath]; %#ok<AGROW>
    end
end

function key = stateKey(nodeIndex, classPattern)
    % Build a unique key for a node and winding state.
    key = strjoin([string(nodeIndex), string(double(classPattern(:).'))], ":");
end

function [stateNode, stateClass, stateCost_units, parentState, closed] = growStateStorage(stateNode, stateClass, stateCost_units, parentState, closed, newCapacity)
    % Double state storage when it fills.
    oldCapacity = numel(stateNode);
    nextNode    = zeros(1, newCapacity);
    nextNode(1:oldCapacity) = stateNode;
    stateNode = nextNode;
    nextClass = zeros(newCapacity, size(stateClass, 2));
    nextClass(1:oldCapacity, :) = stateClass;
    stateClass   = nextClass;
    nextCost_units = Inf(1, newCapacity);
    nextCost_units(1:oldCapacity) = stateCost_units;
    stateCost_units = nextCost_units;
    nextParent    = zeros(1, newCapacity);
    nextParent(1:oldCapacity) = parentState;
    parentState = nextParent;
    nextClosed  = false(1, newCapacity);
    nextClosed(1:oldCapacity) = closed;
    closed = nextClosed;
end

function pattern = routeClassPattern(route_units, referencePoints_units)
    % Compute the route's winding class using the search transition rule.
    pattern = zeros(1, size(referencePoints_units, 1));
    % Without reference cuts there is only one route class, so no signature expansion is needed.
    if isempty(referencePoints_units)
        return;
    end
    phase     = atan2(route_units(:, 2) - referencePoints_units(:, 2).', route_units(:, 1) - referencePoints_units(:, 1).');
    reference = principalAngle(phase - phase(1, :)) / (2 * pi);
    % Process each geometric edge while constructing or checking the region topology.
    for edgeIndex = 1:size(route_units, 1) - 1
        step    = principalAngle(phase(edgeIndex + 1, :) - phase(edgeIndex, :)) / (2 * pi);
        pattern = pattern + round(reference(edgeIndex, :) + step - reference(edgeIndex + 1, :));
    end
end

function angle = principalAngle(angle)
    % Wrap angular change to the principal interval.
    angle = atan2(sin(angle), cos(angle));
end

function [cleanedRoute_units, record] = shortenVisibilityRoute(route_units, visibilityFunction, signatureFunction, requiredSignature)
    % Take visible shortcuts only when they preserve the winding class.
    cleanedRoute_units  = route_units;
    initialLength_units = obstacleAvoidance.geometry.routeLength(route_units);
    record            = struct();
    record.CandidateCount          = 0;
    record.VisibilityRejectedCount = 0;
    record.RouteClassRejectedCount = 0;
    record.AcceptedCount           = 0;
    record.LengthReduction_units     = 0;
    % Reject routes that are too short or belong to a different topological class than requested.
    if size(route_units, 1) < 3 || ~isequal(signatureFunction(route_units), requiredSignature)
        return;
    end
    % Continue iterating until the stopping condition for complete shorten visibility route is satisfied.
    while size(cleanedRoute_units, 1) >= 3
        currentLength_units   = obstacleAvoidance.geometry.routeLength(cleanedRoute_units);
        lengthTolerance_units = max(1e-12, 1e-12 * currentLength_units);
        bestReduction_units   = 0;
        bestRoute_units       = cleanedRoute_units;
        % Process each first needed to complete shorten visibility route.
        for firstIndex = 1:size(cleanedRoute_units, 1) - 2
            % Process each second needed to complete shorten visibility route.
            for secondIndex = firstIndex + 2:size(cleanedRoute_units, 1)
                record.CandidateCount = record.CandidateCount + 1;
                reduction_units = obstacleAvoidance.geometry.routeLength(cleanedRoute_units(firstIndex:secondIndex, :)) - norm(cleanedRoute_units(secondIndex, :) - cleanedRoute_units(firstIndex, :));
                % Discard shortcuts whose length reduction is within numerical noise.
                if reduction_units <= lengthTolerance_units
                    continue;
                end
                % Reject this shortcut when it crosses protected geometry; visible shortcuts remain eligible for class-preserving smoothing.
                if ~visibilityFunction(cleanedRoute_units(firstIndex, :), cleanedRoute_units(secondIndex, :))
                    record.VisibilityRejectedCount = record.VisibilityRejectedCount + 1;
                    continue;
                end
                candidate_units = [cleanedRoute_units(1:firstIndex, :); ...
                    cleanedRoute_units(secondIndex:end, :)];
                % Reject a geometrically shorter candidate if it changes the required route class.
                if ~isequal(signatureFunction(candidate_units), requiredSignature)
                    record.RouteClassRejectedCount = record.RouteClassRejectedCount + 1;
                    continue;
                end
                % Keep the existing shortcut when this candidate does not improve route-length reduction beyond tolerance.
                if reduction_units <= bestReduction_units + lengthTolerance_units
                    continue;
                end
                bestReduction_units = reduction_units;
                bestRoute_units     = candidate_units;
            end
        end
        % Stop smoothing when no admissible shortcut provides a meaningful reduction.
        if bestReduction_units <= lengthTolerance_units
            break;
        end
        cleanedRoute_units = bestRoute_units;
        record.AcceptedCount = record.AcceptedCount + 1;
    end
    record.LengthReduction_units = initialLength_units - obstacleAvoidance.geometry.routeLength(cleanedRoute_units);
end
