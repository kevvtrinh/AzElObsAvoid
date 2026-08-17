function graph = buildAzElSpaceTimeVisibilityGraph( ...
        obstacleField, snapshotGraphs, initialState, goalState, options)
%% Section 0: Header & Readme
% SYNTAX
%   emptyGraph = azElInternal.buildAzElSpaceTimeVisibilityGraph()
%   graph = azElInternal.buildAzElSpaceTimeVisibilityGraph( ...
%       obstacleField, snapshotGraphs, initialState, goalState, options)
%**************************************************************************
% PURPOSE
%   - Build one directed visibility graph in azimuth, elevation, and time.
%   - Compare moving detours and waiting with one earliest-arrival search.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%       Complete protected static or time-varying obstacle history.
%   - snapshotGraphs (structure array)
%       Two-dimensional visibility graphs that supply query-driven spatial
%       candidates. The space-time search does not use their edge times.
%   - initialState, goalState (scalar structs)
%       time_s and N-by-2 position_deg endpoint data.
%   - options (scalar resolved options struct)
%       GoalTimeMode, MaximumVelocity_deg_s, SpaceTimeLayerCount,
%       MaximumSpaceTimeCandidateCount, CollisionTimePaddingSamples, and
%       MaximumSpaceTimeDiagnosticEdgeCount control this internal search.
%**************************************************************************
% OUTPUTS
%   - graph (scalar struct)
%       Stable success-or-failure graph, search trace, selected timed path,
%       counts, candidate-reduction data, and resolved options.
%**************************************************************************
% UNITS
%   - Nodes use [azimuth_deg elevation_deg time_s]. Edge distance is in
%     degrees, duration is in seconds, and velocity is deg/s.
%**************************************************************************

%% Section 1: Validate Inputs & Build Spatial Candidates

graph = emptySpaceTimeGraph();
if nargin == 0
    return;
end
fieldIsPacked = isstruct(obstacleField) && isscalar(obstacleField) && ...
    isfield(obstacleField, "Format") && ...
    string(obstacleField.Format) == "AzElTimeObstacleField";
if ~fieldIsPacked
    error("buildAzElSpaceTimeVisibilityGraph:InvalidObstacleField", ...
        "obstacleField must come from buildAzElTimeObstacleField.");
end
requiredStateFields = ["time_s" "position_deg"];
if ~isstruct(initialState) || ~isscalar(initialState) || ...
        ~all(isfield(initialState, requiredStateFields)) || ...
        ~isstruct(goalState) || ~isscalar(goalState) || ...
        ~all(isfield(goalState, requiredStateFields))
    error("buildAzElSpaceTimeVisibilityGraph:InvalidState", ...
        "initialState and goalState must contain time_s and position_deg.");
end
if goalState.time_s <= initialState.time_s
    error("buildAzElSpaceTimeVisibilityGraph:InvalidTimeWindow", ...
        "goalState.time_s must be greater than initialState.time_s.");
end
graph.Options = spaceTimeOptions(options);
[candidatePosition_deg, candidateDiagnostics] = ...
    collectSpatialCandidates(snapshotGraphs, initialState.position_deg, ...
    goalState.position_deg, graph.Options.MaximumSpaceTimeCandidateCount);
spatialPosition_deg = [initialState.position_deg; ...
    goalState.position_deg; candidatePosition_deg];
spatialNodeCount = size(spatialPosition_deg, 1);

%% Section 2: Create Time Layers & Reject Occupied Nodes

layerTime_s = linspace(initialState.time_s, goalState.time_s, ...
    graph.Options.SpaceTimeLayerCount).';
layerCount = numel(layerTime_s);
nodeAzElTime = [ ...
    repmat(spatialPosition_deg, layerCount, 1), ...
    repelem(layerTime_s, spatialNodeCount, 1)];
pointOptions = struct( ...
    "CollisionMode", "polygon", ...
    "TimePaddingSamples", graph.Options.CollisionTimePaddingSamples, ...
    "BoundaryIsOccupied", false);
nodeOccupied = queryAzElTimeObstacle( ...
    obstacleField, nodeAzElTime(:, 1), nodeAzElTime(:, 2), ...
    nodeAzElTime(:, 3), pointOptions);
nodeActiveMask = ~reshape(logical(nodeOccupied), ...
    spatialNodeCount, layerCount).';
nodeActiveMask(1, :) = false;
nodeActiveMask(1, 1) = true;
if graph.Options.GoalTimeMode == "fixedarrival"
    nodeActiveMask(1:end - 1, 2) = false;
end

%% Section 3: Expand Directed Space-Time Visibility Edges

nodeCount = layerCount * spatialNodeCount;
costToCome_deg = inf(nodeCount, 1);
parentNodeIndex = zeros(nodeCount, 1);
costToCome_deg(1) = 0;
generatedEdgeCount = 0;
acceptedEdgeCount = 0;
collisionRejectedEdgeCount = 0;
dynamicsRejectedEdgeCount = 0;
acceptedSourceNodeIndex = zeros(0, 1);
acceptedTargetNodeIndex = zeros(0, 1);
rejectedSourceNodeIndex = zeros(0, 1);
rejectedTargetNodeIndex = zeros(0, 1);
rejectedReason = strings(0, 1);
evaluatedLayerCount = 1;
terminalNodeIndex = 0;
velocityTolerance_deg_s = 1e-10 * max( ...
    1, max(graph.Options.MaximumVelocity_deg_s));
costTolerance_deg = 1e-10;

for targetLayerIndex = 2:layerCount
    targetTime_s = layerTime_s(targetLayerIndex);
    targetSpatialIndex = find(nodeActiveMask(targetLayerIndex, :));
    for sourceLayerIndex = 1:targetLayerIndex - 1
        sourceNodeOffset = (sourceLayerIndex - 1) * spatialNodeCount;
        sourceSpatialIndex = find(isfinite(costToCome_deg( ...
            sourceNodeOffset + (1:spatialNodeCount))) & ...
            nodeActiveMask(sourceLayerIndex, :).');
        if isempty(sourceSpatialIndex)
            continue;
        end
        edgeDuration_s = targetTime_s - layerTime_s(sourceLayerIndex);
        for sourceListIndex = 1:numel(sourceSpatialIndex)
            sourceIndex = sourceSpatialIndex(sourceListIndex);
            sourceNodeIndex = sourceNodeOffset + sourceIndex;
            sourcePosition_deg = spatialPosition_deg(sourceIndex, :);
            for targetListIndex = 1:numel(targetSpatialIndex)
                targetIndex = targetSpatialIndex(targetListIndex);
                targetNodeIndex = ...
                    (targetLayerIndex - 1) * spatialNodeCount + targetIndex;
                targetPosition_deg = spatialPosition_deg(targetIndex, :);
                edgeDelta_deg = targetPosition_deg - sourcePosition_deg;
                generatedEdgeCount = generatedEdgeCount + 1;
                edgeVelocity_deg_s = abs(edgeDelta_deg) / edgeDuration_s;
                if any(edgeVelocity_deg_s > ...
                        graph.Options.MaximumVelocity_deg_s + ...
                        velocityTolerance_deg_s)
                    dynamicsRejectedEdgeCount = ...
                        dynamicsRejectedEdgeCount + 1;
                    [rejectedSourceNodeIndex, rejectedTargetNodeIndex, ...
                        rejectedReason] = retainRejectedEdge( ...
                        rejectedSourceNodeIndex, rejectedTargetNodeIndex, ...
                        rejectedReason, sourceNodeIndex, targetNodeIndex, ...
                        "velocity", graph.Options. ...
                        MaximumSpaceTimeDiagnosticEdgeCount);
                    continue;
                end
                collisionMask = queryAzElTimedPathCollision( ...
                    obstacleField, ...
                    [layerTime_s(sourceLayerIndex); targetTime_s], ...
                    [sourcePosition_deg; targetPosition_deg], struct( ...
                    "TimePaddingSamples", ...
                    graph.Options.CollisionTimePaddingSamples, ...
                    "BoundaryIsOccupied", false, ...
                    "StopAtFirstCollision", true));
                if any(collisionMask)
                    collisionRejectedEdgeCount = ...
                        collisionRejectedEdgeCount + 1;
                    [rejectedSourceNodeIndex, rejectedTargetNodeIndex, ...
                        rejectedReason] = retainRejectedEdge( ...
                        rejectedSourceNodeIndex, rejectedTargetNodeIndex, ...
                        rejectedReason, sourceNodeIndex, targetNodeIndex, ...
                        "collision", graph.Options. ...
                        MaximumSpaceTimeDiagnosticEdgeCount);
                    continue;
                end
                acceptedEdgeCount = acceptedEdgeCount + 1;
                if numel(acceptedSourceNodeIndex) < graph.Options. ...
                        MaximumSpaceTimeDiagnosticEdgeCount
                    acceptedSourceNodeIndex(end + 1, 1) = ...
                        sourceNodeIndex; %#ok<AGROW>
                    acceptedTargetNodeIndex(end + 1, 1) = ...
                        targetNodeIndex; %#ok<AGROW>
                end
                edgeDistance_deg = hypot(edgeDelta_deg(1), edgeDelta_deg(2));
                candidateCost_deg = ...
                    costToCome_deg(sourceNodeIndex) + edgeDistance_deg;
                incumbentCost_deg = costToCome_deg(targetNodeIndex);
                parentIsBetter = candidateCost_deg < ...
                    incumbentCost_deg - costTolerance_deg;
                equalCostHasStableParent = abs(candidateCost_deg - ...
                    incumbentCost_deg) <= costTolerance_deg && ...
                    (parentNodeIndex(targetNodeIndex) == 0 || ...
                    sourceNodeIndex < parentNodeIndex(targetNodeIndex));
                if parentIsBetter || equalCostHasStableParent
                    costToCome_deg(targetNodeIndex) = candidateCost_deg;
                    parentNodeIndex(targetNodeIndex) = sourceNodeIndex;
                end
            end
        end
    end
    evaluatedLayerCount = targetLayerIndex;
    goalNodeIndex = (targetLayerIndex - 1) * spatialNodeCount + 2;
    if graph.Options.GoalTimeMode == "earliestarrival" && ...
            isfinite(costToCome_deg(goalNodeIndex))
        terminalNodeIndex = goalNodeIndex;
        break;
    end
end
if graph.Options.GoalTimeMode == "fixedarrival"
    terminalNodeIndex = (layerCount - 1) * spatialNodeCount + 2;
    if ~isfinite(costToCome_deg(terminalNodeIndex))
        terminalNodeIndex = 0;
    end
end

%% Section 4: Assemble The Path & Search Diagnostics

graph.Representation = "discreteSpaceTimeVisibilityGraph";
graph.LayerTime_s = layerTime_s;
graph.SpatialNodePosition_deg = spatialPosition_deg;
graph.NodeAzElTime = nodeAzElTime;
graph.NodeActiveMask = nodeActiveMask;
graph.CandidateReductionDiagnostics = candidateDiagnostics;
graph.ParentNodeIndex = parentNodeIndex;
graph.CostToCome_deg = costToCome_deg;
graph.GeneratedEdgeCount = generatedEdgeCount;
graph.AcceptedEdgeCount = acceptedEdgeCount;
graph.CollisionRejectedEdgeCount = collisionRejectedEdgeCount;
graph.DynamicsRejectedEdgeCount = dynamicsRejectedEdgeCount;
graph.AcceptedEdgeSourceNodeIndex = acceptedSourceNodeIndex;
graph.AcceptedEdgeTargetNodeIndex = acceptedTargetNodeIndex;
graph.RejectedEdgeSourceNodeIndex = rejectedSourceNodeIndex;
graph.RejectedEdgeTargetNodeIndex = rejectedTargetNodeIndex;
graph.RejectedEdgeReason = rejectedReason;
graph.AcceptedDiagnosticEdgeCount = numel(acceptedSourceNodeIndex);
graph.RejectedDiagnosticEdgeCount = numel(rejectedSourceNodeIndex);
graph.DiagnosticEdgeTraceWasLimited = ...
    acceptedEdgeCount > numel(acceptedSourceNodeIndex) || ...
    collisionRejectedEdgeCount + dynamicsRejectedEdgeCount > ...
    numel(rejectedSourceNodeIndex);
graph.EvaluatedLayerCount = evaluatedLayerCount;
graph.ExpandedNodeIndex = find(isfinite(costToCome_deg));
finalLayerOffset = (evaluatedLayerCount - 1) * spatialNodeCount;
graph.FrontierNodeIndex = finalLayerOffset + find(isfinite( ...
    costToCome_deg(finalLayerOffset + (1:spatialNodeCount))));
[graph.BestPartialNodeIndex, graph.BestPartialDistance_deg] = ...
    bestPartialNode(costToCome_deg, nodeAzElTime, ...
    goalState.position_deg, evaluatedLayerCount, spatialNodeCount);
if terminalNodeIndex == 0
    graph.Message = "No space-time route reached the goal before the horizon.";
    graph.TerminationReason = "openSetExhausted";
    return;
end

pathNodeIndex = terminalNodeIndex;
while pathNodeIndex(1) ~= 1
    parentIndex = parentNodeIndex(pathNodeIndex(1));
    if parentIndex == 0
        error("buildAzElSpaceTimeVisibilityGraph:CorruptParentTrace", ...
            "The selected space-time parent trace does not reach the start.");
    end
    pathNodeIndex = [parentIndex; pathNodeIndex]; %#ok<AGROW>
end
pathAzElTime = nodeAzElTime(pathNodeIndex, :);
pathStep_deg = diff(pathAzElTime(:, 1:2), 1, 1);
pathDistance_deg = hypot(pathStep_deg(:, 1), pathStep_deg(:, 2));
pathDuration_s = diff(pathAzElTime(:, 3));
waitEdge = pathDistance_deg <= 1e-10;
graph.Success = true;
graph.Message = "The directed space-time visibility graph reached the goal.";
graph.TerminationReason = "goalReached";
graph.PathNodeIndex = pathNodeIndex;
graph.PathPosition_deg = pathAzElTime(:, 1:2);
graph.PathTime_s = pathAzElTime(:, 3);
graph.PathCost_deg = sum(pathDistance_deg);
graph.ArrivalTime_s = pathAzElTime(end, 3);
graph.PathEdgeCount = numel(pathNodeIndex) - 1;
graph.WaitEdgeCount = nnz(waitEdge);
graph.WaitDuration_s = sum(pathDuration_s(waitEdge));
end

%% Section 5: Local Functions

function options = spaceTimeOptions(options)
% PURPOSE
%   - Read and validate the internal space-time controls in one place.
requiredFields = ["GoalTimeMode" "MaximumVelocity_deg_s" ...
    "SpaceTimeLayerCount" "MaximumSpaceTimeCandidateCount" ...
    "CollisionTimePaddingSamples" ...
    "MaximumSpaceTimeDiagnosticEdgeCount"];
if ~isstruct(options) || ~isscalar(options) || ...
        ~all(isfield(options, requiredFields))
    error("buildAzElSpaceTimeVisibilityGraph:InvalidOptions", ...
        "options does not contain every required space-time control.");
end
options.GoalTimeMode = lower(string(options.GoalTimeMode));
if ~isscalar(options.GoalTimeMode) || ~any( ...
        options.GoalTimeMode == ["earliestarrival" "fixedarrival"])
    error("buildAzElSpaceTimeVisibilityGraph:InvalidGoalTimeMode", ...
        "GoalTimeMode must be earliestArrival or fixedArrival.");
end
validateattributes(options.MaximumVelocity_deg_s, {'numeric'}, ...
    {'real', 'positive', 'vector', 'numel', 2});
options.MaximumVelocity_deg_s = ...
    reshape(double(options.MaximumVelocity_deg_s), 1, 2);
validateattributes(options.SpaceTimeLayerCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 3});
validateattributes(options.MaximumSpaceTimeCandidateCount, {'numeric'}, ...
    {'real', 'scalar', 'positive'});
validateattributes(options.CollisionTimePaddingSamples, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
validateattributes(options.MaximumSpaceTimeDiagnosticEdgeCount, ...
    {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
end

function [candidatePosition_deg, diagnostics] = collectSpatialCandidates( ...
        snapshotGraphs, startPosition_deg, goalPosition_deg, maximumCount)
% PURPOSE
%   - Use query-driven snapshot routes before unused graph nodes.
routePosition_deg = zeros(0, 2);
otherPosition_deg = zeros(0, 2);
for graphIndex = 1:numel(snapshotGraphs)
    snapshotGraph = snapshotGraphs(graphIndex);
    if isfield(snapshotGraph, "Success") && snapshotGraph.Success
        if isfield(snapshotGraph, "PathPosition_deg")
            routePosition_deg = [routePosition_deg; ...
                snapshotGraph.PathPosition_deg]; %#ok<AGROW>
        end
        if isfield(snapshotGraph, "AlternativePathPosition_deg")
            routePosition_deg = [routePosition_deg; ...
                snapshotGraph.AlternativePathPosition_deg]; %#ok<AGROW>
        end
    end
    if isfield(snapshotGraph, "NodePosition_deg")
        activePosition_deg = snapshotGraph.NodePosition_deg;
        if isfield(snapshotGraph, "CandidateActiveMask") && ...
                size(activePosition_deg, 1) >= 2 + ...
                numel(snapshotGraph.CandidateActiveMask)
            activePosition_deg = activePosition_deg( ...
                [true; true; snapshotGraph.CandidateActiveMask(:)], :);
        end
        otherPosition_deg = [otherPosition_deg; ...
            activePosition_deg]; %#ok<AGROW>
    end
end
endpointPosition_deg = [startPosition_deg; goalPosition_deg];
routePosition_deg = removeEndpointRows(routePosition_deg, ...
    endpointPosition_deg);
otherPosition_deg = removeEndpointRows(otherPosition_deg, ...
    endpointPosition_deg);
routePosition_deg = unique(routePosition_deg, "rows", "stable");
otherPosition_deg = unique(otherPosition_deg, "rows", "stable");
otherPosition_deg = removeExistingRows( ...
    otherPosition_deg, routePosition_deg);
allPosition_deg = [routePosition_deg; otherPosition_deg];
inputCount = size(allPosition_deg, 1);
retainedCount = min(inputCount, maximumCount);
candidatePosition_deg = allPosition_deg(1:retainedCount, :);
if retainedCount < inputCount
    warning("buildAzElSpaceTimeVisibilityGraph:CandidateReduction", ...
        "The space-time graph retained %d of %d spatial candidates. " + ...
        "Increase MaximumSpaceTimeCandidateCount to retain more nodes.", ...
        retainedCount, inputCount);
end
diagnostics = struct( ...
    "InputCandidateCount", inputCount, ...
    "RouteCandidateCount", size(routePosition_deg, 1), ...
    "OtherGraphCandidateCount", size(otherPosition_deg, 1), ...
    "RetainedCandidateCount", retainedCount, ...
    "DroppedCandidateCount", inputCount - retainedCount, ...
    "MaximumCandidateCount", maximumCount);
end

function position_deg = removeEndpointRows(position_deg, endpoint_deg)
% PURPOSE
%   - Remove endpoint copies from the candidate-only spatial set.
if isempty(position_deg)
    return;
end
isEndpoint = false(size(position_deg, 1), 1);
for endpointIndex = 1:size(endpoint_deg, 1)
    isEndpoint = isEndpoint | all(abs(position_deg - ...
        endpoint_deg(endpointIndex, :)) <= 1e-10, 2);
end
position_deg = position_deg(~isEndpoint, :);
end

function position_deg = removeExistingRows(position_deg, existing_deg)
% PURPOSE
%   - Keep stable positions that do not already occur in a preferred set.
if isempty(position_deg) || isempty(existing_deg)
    return;
end
isExisting = false(size(position_deg, 1), 1);
for existingIndex = 1:size(existing_deg, 1)
    isExisting = isExisting | all(abs(position_deg - ...
        existing_deg(existingIndex, :)) <= 1e-10, 2);
end
position_deg = position_deg(~isExisting, :);
end

function [sourceIndex, targetIndex, reason] = retainRejectedEdge( ...
        sourceIndex, targetIndex, reason, newSourceIndex, ...
        newTargetIndex, newReason, maximumCount)
% PURPOSE
%   - Retain a bounded trace without changing any search decision.
if numel(sourceIndex) >= maximumCount
    return;
end
sourceIndex(end + 1, 1) = newSourceIndex;
targetIndex(end + 1, 1) = newTargetIndex;
reason(end + 1, 1) = newReason;
end

function [nodeIndex, distance_deg] = bestPartialNode( ...
        costToCome_deg, nodeAzElTime, goalPosition_deg, ...
        evaluatedLayerCount, spatialNodeCount)
% PURPOSE
%   - Return the reached state with the smallest spatial goal distance.
maximumNodeIndex = evaluatedLayerCount * spatialNodeCount;
reachedNodeIndex = find(isfinite(costToCome_deg(1:maximumNodeIndex)));
if isempty(reachedNodeIndex)
    nodeIndex = 0;
    distance_deg = Inf;
    return;
end
offset_deg = nodeAzElTime(reachedNodeIndex, 1:2) - goalPosition_deg;
candidateDistance_deg = hypot(offset_deg(:, 1), offset_deg(:, 2));
ranking = [candidateDistance_deg, ...
    nodeAzElTime(reachedNodeIndex, 3), costToCome_deg(reachedNodeIndex), ...
    reachedNodeIndex];
[~, order] = sortrows(ranking, [1 2 3 4]);
nodeIndex = reachedNodeIndex(order(1));
distance_deg = candidateDistance_deg(order(1));
end

function graph = emptySpaceTimeGraph()
% PURPOSE
%   - Define the stable schema for success and expected search failure.
graph = struct( ...
    "Success", false, ...
    "Message", "The space-time graph was not evaluated.", ...
    "TerminationReason", "notEvaluated", ...
    "Representation", "discreteSpaceTimeVisibilityGraph", ...
    "LayerTime_s", zeros(0, 1), ...
    "SpatialNodePosition_deg", zeros(0, 2), ...
    "NodeAzElTime", zeros(0, 3), ...
    "NodeActiveMask", false(0, 0), ...
    "ParentNodeIndex", zeros(0, 1), ...
    "CostToCome_deg", zeros(0, 1), ...
    "ExpandedNodeIndex", zeros(0, 1), ...
    "FrontierNodeIndex", zeros(0, 1), ...
    "BestPartialNodeIndex", 0, ...
    "BestPartialDistance_deg", Inf, ...
    "AcceptedEdgeSourceNodeIndex", zeros(0, 1), ...
    "AcceptedEdgeTargetNodeIndex", zeros(0, 1), ...
    "RejectedEdgeSourceNodeIndex", zeros(0, 1), ...
    "RejectedEdgeTargetNodeIndex", zeros(0, 1), ...
    "RejectedEdgeReason", strings(0, 1), ...
    "GeneratedEdgeCount", 0, ...
    "AcceptedEdgeCount", 0, ...
    "CollisionRejectedEdgeCount", 0, ...
    "DynamicsRejectedEdgeCount", 0, ...
    "AcceptedDiagnosticEdgeCount", 0, ...
    "RejectedDiagnosticEdgeCount", 0, ...
    "DiagnosticEdgeTraceWasLimited", false, ...
    "EvaluatedLayerCount", 0, ...
    "PathNodeIndex", zeros(0, 1), ...
    "PathPosition_deg", zeros(0, 2), ...
    "PathTime_s", zeros(0, 1), ...
    "PathCost_deg", Inf, ...
    "ArrivalTime_s", Inf, ...
    "PathEdgeCount", 0, ...
    "WaitEdgeCount", 0, ...
    "WaitDuration_s", 0, ...
    "CandidateReductionDiagnostics", struct(), ...
    "Options", struct());
end
