function [routes, statistics] = buildAdaptiveRoutes(request, ...
        spatialResolution_deg, nominalArrivalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [routes, statistics] = buildAdaptiveRoutes(request, ...
%       spatialResolution_deg, nominalArrivalTime_s)
%**************************************************************************
% PURPOSE
%   - Derive deterministic route seeds from canonical polygon boundaries at
%     the planner-selected refinement level; callers never provide routes.
%**************************************************************************
% INPUTS
%   - request (normalized scalar planning request)
%   - spatialResolution_deg (positive scalar private resolution)
%   - nominalArrivalTime_s (finite scalar)
%**************************************************************************
% OUTPUTS
%   - routes (cell column)
%       Start-to-goal position arrays selected internally.
%   - statistics (scalar struct)
%       Node, edge, and route counts for factual diagnostics.
%**************************************************************************
% UNITS
%   - Position and spatial resolution are degrees; time is seconds.

%% Section 1: Build Boundary-Derived Candidate Nodes
goalState = evaluateAzElGoal(request.goal, nominalArrivalTime_s);
startPosition_deg = request.initialState.position_deg;
goalPosition_deg = goalState.position_deg;
routes = {[startPosition_deg; goalPosition_deg]};

if isempty(request.obstacles)
    statistics = struct("nodeCount", 2, "edgeCount", 1, ...
        "routeCount", 1);
    return;
end

nodes_deg = [startPosition_deg; goalPosition_deg];
offsetBase_deg = request.options.safetyMargin_deg + ...
    max(0.4 .* spatialResolution_deg, ...
        10 .* request.options.clearanceTolerance_deg);

for obstacleIndex = 1:numel(request.obstacles)
    obstacle = request.obstacles{obstacleIndex};
    routeSampleIndices = representativeRouteSampleIndices( ...
        obstacle, spatialResolution_deg);
    for sampleIndex = reshape(routeSampleIndices, 1, [])
        regions_deg = splitAzElRegions(obstacle.az_deg{sampleIndex}, ...
            obstacle.el_deg{sampleIndex});
        for regionIndex = 1:numel(regions_deg)
            shiftedRegions = shiftRegionsForWrap( ...
                regions_deg{regionIndex}, startPosition_deg(1), ...
                goalPosition_deg(1), request);
            for shiftIndex = 1:numel(shiftedRegions)
                newNodes_deg = boundaryOffsetNodes( ...
                    shiftedRegions{shiftIndex}, offsetBase_deg, ...
                    spatialResolution_deg);
                nodes_deg = [nodes_deg; newNodes_deg]; %#ok<AGROW>
            end
        end
    end
end
nodes_deg = filterAndDeduplicateNodes(nodes_deg, request, ...
    spatialResolution_deg, nominalArrivalTime_s);
nodes_deg = ensureEndpointsFirst(nodes_deg, startPosition_deg, ...
    goalPosition_deg);
routes = appendEnvelopeRoutes(routes, nodes_deg, startPosition_deg, ...
    goalPosition_deg, request, spatialResolution_deg);

%% Section 2: Construct A Provisional Visibility Graph
nodeCount = size(nodes_deg, 1);
edgeWeight = inf(nodeCount, nodeCount);
edgeCount = 0;
candidateEdge = candidateEdgeMask(nodes_deg);
for firstNodeIndex = 1:(nodeCount - 1)
    for secondNodeIndex = (firstNodeIndex + 1):nodeCount
        if candidateEdge(firstNodeIndex, secondNodeIndex) && ...
                provisionalEdgeIsSafe(nodes_deg(firstNodeIndex, :), ...
                nodes_deg(secondNodeIndex, :), startPosition_deg, ...
                goalPosition_deg, nominalArrivalTime_s, request, ...
                spatialResolution_deg)
            delta_deg = nodes_deg(secondNodeIndex, :) - ...
                nodes_deg(firstNodeIndex, :);
            physicalWeight_s = max(abs(delta_deg) ./ ...
                request.limits.maxVelocity_deg_s);
            geometricTieBreak = 1e-9 .* norm(delta_deg);
            edgeWeight(firstNodeIndex, secondNodeIndex) = ...
                physicalWeight_s + geometricTieBreak;
            edgeWeight(secondNodeIndex, firstNodeIndex) = ...
                edgeWeight(firstNodeIndex, secondNodeIndex);
            edgeCount = edgeCount + 1;
        end
    end
end

%% Section 3: Extract Deterministic Alternative Routes
routes = appendDirectionalGraphRoutes(routes, nodes_deg, edgeWeight, ...
    spatialResolution_deg);
routes = appendCommonNeighborRoutes(routes, nodes_deg, edgeWeight, ...
    spatialResolution_deg);
workingWeight = edgeWeight;
maximumRouteCount = min(8, max(3, ceil(sqrt(nodeCount))));
maximumAttempts = 3 .* maximumRouteCount;
for attemptIndex = 1:floor(maximumAttempts(1))
    routeIndices = shortestPathIndices(workingWeight, 1, 2);
    if isempty(routeIndices)
        break;
    end
    routePosition_deg = nodes_deg(routeIndices, :);
    routePosition_deg = removeCollinearRouteNodes(routePosition_deg, ...
        spatialResolution_deg);
    if ~routeAlreadyPresent(routes, routePosition_deg, ...
            spatialResolution_deg)
        routes{end + 1, 1} = routePosition_deg; %#ok<AGROW>
    end
    for edgeIndex = 1:(numel(routeIndices) - 1)
        firstIndex = routeIndices(edgeIndex);
        secondIndex = routeIndices(edgeIndex + 1);
        workingWeight(firstIndex, secondIndex) = ...
            1.8 .* workingWeight(firstIndex, secondIndex) + ...
            spatialResolution_deg;
        workingWeight(secondIndex, firstIndex) = ...
            workingWeight(firstIndex, secondIndex);
    end
    if numel(routes) >= maximumRouteCount
        break;
    end
end

statistics = struct( ...
    "nodeCount", nodeCount, ...
    "edgeCount", edgeCount, ...
    "routeCount", numel(routes));
end

function routes = appendEnvelopeRoutes(routes, nodes_deg, ...
        startPosition_deg, goalPosition_deg, request, ...
        spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   routes = appendEnvelopeRoutes(routes, nodes_deg, ...
%       startPosition_deg, goalPosition_deg, request, ...
%       spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Add scene-scale upper, lower, left, and right bypasses derived from
%     internally generated obstacle-boundary extrema.
%**************************************************************************
% INPUTS
%   - routes (cell column)
%   - nodes_deg (N-by-2 numeric)
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric)
%   - request (normalized scalar request)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - routes (augmented cell column)
%**************************************************************************
% UNITS
%   - Positions and resolution are degrees; time is seconds.

if size(nodes_deg, 1) <= 2
    return;
end
boundaryNodes_deg = nodes_deg(3:end, :);
envelopeMargin_deg = 0.75 .* spatialResolution_deg;
minimum_deg = min(boundaryNodes_deg, [], 1) - envelopeMargin_deg;
maximum_deg = max(boundaryNodes_deg, [], 1) + envelopeMargin_deg;
minimum_deg(2) = max(minimum_deg(2), ...
    request.limits.elevation_deg(1));
maximum_deg(2) = min(maximum_deg(2), ...
    request.limits.elevation_deg(2));
if ~request.options.azimuthWrap
    minimum_deg(1) = max(minimum_deg(1), ...
        request.limits.azimuth_deg(1));
    maximum_deg(1) = min(maximum_deg(1), ...
        request.limits.azimuth_deg(2));
end
candidateRoutes = { ...
    [startPosition_deg; minimum_deg(1), maximum_deg(2); ...
        maximum_deg; goalPosition_deg]
    [startPosition_deg; minimum_deg; ...
        maximum_deg(1), minimum_deg(2); goalPosition_deg]
    [startPosition_deg; minimum_deg; ...
        minimum_deg(1), maximum_deg(2); goalPosition_deg]
    [startPosition_deg; maximum_deg(1), minimum_deg(2); ...
        maximum_deg; goalPosition_deg]};
for candidateIndex = 1:numel(candidateRoutes)
    candidateRoute_deg = removeCollinearRouteNodes( ...
        candidateRoutes{candidateIndex}, spatialResolution_deg);
    if routeAlreadyPresent(routes, candidateRoute_deg, ...
            spatialResolution_deg)
        continue;
    end
    routes{end + 1, 1} = candidateRoute_deg; %#ok<AGROW>
end
end

function routes = appendDirectionalGraphRoutes(routes, nodes_deg, ...
        edgeWeight, spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   routes = appendDirectionalGraphRoutes(routes, nodes_deg, ...
%       edgeWeight, spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Extract upper, lower, right, and left graph routes so alternatives
%     represent distinct scene-scale topologies rather than edge variants.
%**************************************************************************
% INPUTS
%   - routes (cell column)
%   - nodes_deg (N-by-2 numeric)
%   - edgeWeight (N-by-N provisional graph)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - routes (augmented cell column)
%**************************************************************************
% UNITS
%   - Positions and resolution are degrees.

nodeCount = size(nodes_deg, 1);
if nodeCount <= 3
    return;
end
interiorIndices = (3:nodeCount).';
medianAzimuth_deg = median(nodes_deg(interiorIndices, 1));
medianElevation_deg = median(nodes_deg(interiorIndices, 2));
directionalMasks = { ...
    nodes_deg(:, 2) >= medianElevation_deg
    nodes_deg(:, 2) <= medianElevation_deg
    nodes_deg(:, 1) >= medianAzimuth_deg
    nodes_deg(:, 1) <= medianAzimuth_deg};
for directionIndex = 1:numel(directionalMasks)
    allowedNode = directionalMasks{directionIndex};
    allowedNode(1:2) = true;
    restrictedWeight = inf(size(edgeWeight));
    restrictedWeight(allowedNode, allowedNode) = ...
        edgeWeight(allowedNode, allowedNode);
    routeIndices = shortestPathIndices(restrictedWeight, 1, 2);
    if isempty(routeIndices)
        continue;
    end
    candidateRoute_deg = removeCollinearRouteNodes( ...
        nodes_deg(routeIndices, :), spatialResolution_deg);
    if ~routeAlreadyPresent(routes, candidateRoute_deg, ...
            spatialResolution_deg)
        routes{end + 1, 1} = candidateRoute_deg; %#ok<AGROW>
    end
end
end

function routes = appendCommonNeighborRoutes(routes, nodes_deg, ...
        edgeWeight, spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   routes = appendCommonNeighborRoutes(routes, nodes_deg, edgeWeight, ...
%       spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Add geometrically diverse one-turn detours before graph penalties can
%     spend the route set on near-duplicate shortest paths.
%**************************************************************************
% INPUTS
%   - routes (cell column)
%   - nodes_deg (N-by-2 numeric)
%   - edgeWeight (N-by-N provisional graph)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - routes (augmented cell column)
%**************************************************************************
% UNITS
%   - Positions and resolution are degrees.

commonNeighbor = find(isfinite(edgeWeight(1, :)) & ...
    isfinite(edgeWeight(2, :)));
commonNeighbor = setdiff(commonNeighbor, [1, 2], "stable");
if isempty(commonNeighbor)
    return;
end

combinedWeight = edgeWeight(1, commonNeighbor) + ...
    edgeWeight(2, commonNeighbor);
[~, shortestOrder] = sort(combinedWeight, "ascend");
[~, upperOrder] = sort(nodes_deg(commonNeighbor, 2), "descend");
[~, lowerOrder] = sort(nodes_deg(commonNeighbor, 2), "ascend");
[~, rightOrder] = sort(nodes_deg(commonNeighbor, 1), "descend");
[~, leftOrder] = sort(nodes_deg(commonNeighbor, 1), "ascend");
selectionOrder = unique([ ...
    reshape(shortestOrder(1:min(2, numel(shortestOrder))), 1, []), ...
    reshape(upperOrder(1:min(2, numel(upperOrder))), 1, []), ...
    reshape(lowerOrder(1:min(2, numel(lowerOrder))), 1, []), ...
    rightOrder(1), leftOrder(1)], "stable");

maximumDiverseRouteCount = 6;
addedCount = 0;
for selectionIndex = reshape(selectionOrder, 1, [])
    nodeIndex = commonNeighbor(selectionIndex);
    candidateRoute_deg = [nodes_deg(1, :); ...
        nodes_deg(nodeIndex, :); nodes_deg(2, :)];
    if ~routeAlreadyPresent(routes, candidateRoute_deg, ...
            spatialResolution_deg)
        routes{end + 1, 1} = candidateRoute_deg; %#ok<AGROW>
        addedCount = addedCount + 1;
    end
    if addedCount >= maximumDiverseRouteCount
        break;
    end
end
end

function candidateEdge = candidateEdgeMask(nodes_deg)
%% Section 0: Header & Readme
% SYNTAX
%   candidateEdge = candidateEdgeMask(nodes_deg)
%**************************************************************************
% PURPOSE
%   - Keep the provisional visibility graph sparse and deterministic with
%     local geometric neighbors plus direct endpoint connections.
%**************************************************************************
% INPUTS
%   - nodes_deg (N-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - candidateEdge (N-by-N logical symmetric matrix)
%**************************************************************************
% UNITS
%   - Node coordinates are degrees.

nodeCount = size(nodes_deg, 1);
candidateEdge = false(nodeCount, nodeCount);
if nodeCount <= 48
    candidateEdge = triu(true(nodeCount), 1);
    candidateEdge = candidateEdge | candidateEdge.';
    return;
end

neighborCount = min(nodeCount - 1, max(16, ...
    ceil(2 .* sqrt(nodeCount))));
for nodeIndex = 1:nodeCount
    distanceSquared_deg2 = sum((nodes_deg - ...
        nodes_deg(nodeIndex, :)).^2, 2);
    distanceSquared_deg2(nodeIndex) = Inf;
    [~, sortedIndices] = sort(distanceSquared_deg2, "ascend");
    selectedIndices = sortedIndices(1:neighborCount);
    candidateEdge(nodeIndex, selectedIndices) = true;
end
candidateEdge(1:2, :) = true;
candidateEdge(:, 1:2) = true;
candidateEdge(1:(nodeCount + 1):end) = false;
candidateEdge = candidateEdge | candidateEdge.';
end

function sampleIndices = representativeRouteSampleIndices( ...
        obstacle, spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   sampleIndices = representativeRouteSampleIndices( ...
%       obstacle, spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Select time slices that define spatial topology at the current
%     resolution while retaining endpoints, extrema, and topology changes.
%   - Leave the complete obstacle history intact for final validation.
%**************************************************************************
% INPUTS
%   - obstacle (normalized scalar canonical obstacle)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - sampleIndices (increasing integer row)
%**************************************************************************
% UNITS
%   - Geometry signatures and resolution are degrees.

sampleCount = numel(obstacle.time_s);
if sampleCount <= 3
    sampleIndices = 1:sampleCount;
    return;
end

geometrySignature_deg = nan(sampleCount, 6);
topologyChangeIndices = zeros(0, 1);
for sampleIndex = 1:sampleCount
    azimuth_deg = obstacle.az_deg{sampleIndex};
    elevation_deg = obstacle.el_deg{sampleIndex};
    finiteMask = isfinite(azimuth_deg) & isfinite(elevation_deg);
    if any(finiteMask)
        finiteAzimuth_deg = azimuth_deg(finiteMask);
        finiteElevation_deg = elevation_deg(finiteMask);
        geometrySignature_deg(sampleIndex, :) = [ ...
            mean(finiteAzimuth_deg), mean(finiteElevation_deg), ...
            min(finiteAzimuth_deg), max(finiteAzimuth_deg), ...
            min(finiteElevation_deg), max(finiteElevation_deg)];
    end
    if sampleIndex > 1 && ( ...
            ~isequal(size(obstacle.az_deg{sampleIndex - 1}), ...
                size(azimuth_deg)) || ...
            ~isequal(isfinite(obstacle.az_deg{sampleIndex - 1}), ...
                isfinite(azimuth_deg)))
        topologyChangeIndices = [topologyChangeIndices; ...
            sampleIndex - 1; sampleIndex]; %#ok<AGROW>
    end
end

signatureRange_deg = max(geometrySignature_deg, [], 1, ...
    "omitmissing") - min(geometrySignature_deg, [], 1, "omitmissing");
finiteRange_deg = signatureRange_deg(isfinite(signatureRange_deg));
if isempty(finiteRange_deg)
    geometryExcursion_deg = 0;
else
    geometryExcursion_deg = max(finiteRange_deg);
end
adaptiveSampleCount = max(3, ...
    ceil(geometryExcursion_deg ./ spatialResolution_deg) + 2);
adaptiveSampleCount = min(sampleCount, adaptiveSampleCount);
uniformIndices = round(linspace(1, sampleCount, adaptiveSampleCount));

extremumIndices = zeros(0, 1);
for signatureIndex = 1:size(geometrySignature_deg, 2)
    signature = geometrySignature_deg(:, signatureIndex);
    finiteIndices = find(isfinite(signature));
    if isempty(finiteIndices)
        continue;
    end
    [~, minimumLocalIndex] = min(signature(finiteIndices));
    [~, maximumLocalIndex] = max(signature(finiteIndices));
    extremumIndices = [extremumIndices; ...
        finiteIndices(minimumLocalIndex); ...
        finiteIndices(maximumLocalIndex)]; %#ok<AGROW>
end
sampleIndices = unique([1; sampleCount; uniformIndices(:); ...
    extremumIndices; topologyChangeIndices]).';
end

function shiftedRegions = shiftRegionsForWrap(vertices_deg, ...
        startAzimuth_deg, goalAzimuth_deg, request)
%% Section 0: Header & Readme
% SYNTAX
%   shiftedRegions = shiftRegionsForWrap(vertices_deg, startAzimuth_deg, ...
%       goalAzimuth_deg, request)
%**************************************************************************
% PURPOSE
%   - Place canonical polygons near the continuous unwrapped route corridor.
%**************************************************************************
% INPUTS
%   - vertices_deg (N-by-2 numeric)
%   - startAzimuth_deg, goalAzimuth_deg (scalars)
%   - request (normalized scalar request)
%**************************************************************************
% OUTPUTS
%   - shiftedRegions (cell column)
%**************************************************************************
% UNITS
%   - Azimuth is degrees.

if ~request.options.azimuthWrap
    shiftedRegions = {vertices_deg};
    return;
end
span_deg = diff(request.limits.azimuth_deg);
referenceAzimuth_deg = 0.5 .* (startAzimuth_deg + goalAzimuth_deg);
centerShift = round((referenceAzimuth_deg - mean(vertices_deg(:, 1))) ./ ...
    span_deg);
shiftedVertices_deg = vertices_deg;
shiftedVertices_deg(:, 1) = shiftedVertices_deg(:, 1) + ...
    centerShift .* span_deg;
shiftedRegions = {shiftedVertices_deg};
end

function nodes_deg = boundaryOffsetNodes(vertices_deg, offsetBase_deg, ...
        spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   nodes_deg = boundaryOffsetNodes(vertices_deg, offsetBase_deg, ...
%       spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Generate deterministic exterior node candidates around vertices,
%     edges, and the polygon bounding box at one private refinement scale.
%**************************************************************************
% INPUTS
%   - vertices_deg (N-by-2 polygon)
%   - offsetBase_deg (positive scalar)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - nodes_deg (M-by-2 numeric)
%**************************************************************************
% UNITS
%   - Positions and offsets are degrees.

centroid_deg = mean(vertices_deg, 1);
closedVertices_deg = [vertices_deg; vertices_deg(1, :)];
boundarySamples_deg = zeros(0, 2);
maximumSamplesPerEdge = 24;
for edgeIndex = 1:size(vertices_deg, 1)
    edgeStart_deg = closedVertices_deg(edgeIndex, :);
    edgeEnd_deg = closedVertices_deg(edgeIndex + 1, :);
    edgeLength_deg = norm(edgeEnd_deg - edgeStart_deg);
    sampleCount = min(maximumSamplesPerEdge, max(1, ...
        ceil(edgeLength_deg ./ max(spatialResolution_deg, eps))));
    interpolationFraction = (0:(sampleCount - 1)).' ./ sampleCount;
    edgeSamples_deg = edgeStart_deg + interpolationFraction .* ...
        (edgeEnd_deg - edgeStart_deg);
    boundarySamples_deg = [boundarySamples_deg; edgeSamples_deg]; ...
        %#ok<AGROW>
end

nodes_deg = zeros(0, 2);
offsetScales = [1, 1.8];
for sampleIndex = 1:size(boundarySamples_deg, 1)
    radialDirection = boundarySamples_deg(sampleIndex, :) - centroid_deg;
    if norm(radialDirection) <= eps
        radialDirection = [1, 0];
    else
        radialDirection = radialDirection ./ norm(radialDirection);
    end
    for offsetIndex = 1:numel(offsetScales)
        nodes_deg(end + 1, :) = boundarySamples_deg(sampleIndex, :) + ...
            offsetScales(offsetIndex) .* offsetBase_deg .* ...
            radialDirection; %#ok<AGROW>
    end
end

minimum_deg = min(vertices_deg, [], 1) - offsetBase_deg;
maximum_deg = max(vertices_deg, [], 1) + offsetBase_deg;
boundingNodes_deg = [ ...
    minimum_deg; ...
    minimum_deg(1), maximum_deg(2); ...
    maximum_deg; ...
    maximum_deg(1), minimum_deg(2)];
nodes_deg = [nodes_deg; boundingNodes_deg];
end

function nodes_deg = filterAndDeduplicateNodes(nodes_deg, request, ...
        spatialResolution_deg, nominalArrivalTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   nodes_deg = filterAndDeduplicateNodes(nodes_deg, request, ...
%       spatialResolution_deg, nominalArrivalTime_s)
%**************************************************************************
% PURPOSE
%   - Enforce physical domain bounds, nominal clearance, and stable node
%     deduplication without exposing candidate controls to callers.
%**************************************************************************
% INPUTS
%   - nodes_deg (N-by-2 numeric)
%   - request (normalized scalar request)
%   - spatialResolution_deg (positive scalar)
%   - nominalArrivalTime_s (finite scalar)
%**************************************************************************
% OUTPUTS
%   - nodes_deg (M-by-2 numeric)
%**************************************************************************
% UNITS
%   - Positions and resolution are degrees; time is seconds.

elevationInside = nodes_deg(:, 2) >= request.limits.elevation_deg(1) & ...
    nodes_deg(:, 2) <= request.limits.elevation_deg(2);
if request.options.azimuthWrap
    startAzimuth_deg = request.initialState.position_deg(1);
    span_deg = diff(request.limits.azimuth_deg);
    azimuthInside = nodes_deg(:, 1) >= startAzimuth_deg - span_deg & ...
        nodes_deg(:, 1) <= startAzimuth_deg + span_deg;
else
    azimuthInside = nodes_deg(:, 1) >= request.limits.azimuth_deg(1) & ...
        nodes_deg(:, 1) <= request.limits.azimuth_deg(2);
end
nodes_deg = nodes_deg(elevationInside & azimuthInside, :);

clearanceGuard_deg = request.options.safetyMargin_deg + ...
    0.08 .* spatialResolution_deg;
keep = true(size(nodes_deg, 1), 1);
for nodeIndex = 3:size(nodes_deg, 1)
    clearance_deg = azElObstacleClearance(request.obstacles, ...
        nodes_deg(nodeIndex, :), nominalArrivalTime_s, request.options);
    keep(nodeIndex) = clearance_deg > clearanceGuard_deg;
end
nodes_deg = nodes_deg(keep, :);

deduplicationScale_deg = max(0.2 .* spatialResolution_deg, 1e-8);
quantized = round(nodes_deg ./ deduplicationScale_deg);
[~, uniqueIndices] = unique(quantized, "rows", "stable");
nodes_deg = nodes_deg(uniqueIndices, :);
end

function nodes_deg = ensureEndpointsFirst(nodes_deg, startPosition_deg, ...
        goalPosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   nodes_deg = ensureEndpointsFirst(nodes_deg, startPosition_deg, ...
%       goalPosition_deg)
%**************************************************************************
% PURPOSE
%   - Preserve start and goal as graph nodes one and two after filtering.
%**************************************************************************
% INPUTS
%   - nodes_deg (N-by-2 numeric)
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - nodes_deg (N-by-2 numeric)
%**************************************************************************
% UNITS
%   - Positions are degrees.

isEndpoint = vecnorm(nodes_deg - startPosition_deg, 2, 2) <= 1e-10 | ...
    vecnorm(nodes_deg - goalPosition_deg, 2, 2) <= 1e-10;
nodes_deg = [startPosition_deg; goalPosition_deg; nodes_deg(~isEndpoint, :)];
end

function isSafe = provisionalEdgeIsSafe(firstPosition_deg, ...
        secondPosition_deg, startPosition_deg, goalPosition_deg, ...
        nominalArrivalTime_s, request, spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   isSafe = provisionalEdgeIsSafe(firstPosition_deg, ...
%       secondPosition_deg, startPosition_deg, goalPosition_deg, ...
%       nominalArrivalTime_s, request, spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Reject visibly blocked seed edges; final continuous certification is
%     performed later by the independent command validator.
%**************************************************************************
% INPUTS
%   - firstPosition_deg, secondPosition_deg (1-by-2 numeric)
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric)
%   - nominalArrivalTime_s (finite scalar)
%   - request (normalized scalar request)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - isSafe (logical scalar)
%**************************************************************************
% UNITS
%   - Positions and resolution are degrees; time is seconds.

edgeLength_deg = norm(secondPosition_deg - firstPosition_deg);
sampleSpacing_deg = max([0.35 .* spatialResolution_deg, ...
    0.2 .* request.options.safetyMargin_deg, 0.02]);
sampleCount = min(24, max(3, ceil(edgeLength_deg ./ sampleSpacing_deg) + 1));
interpolationFraction = linspace(0, 1, sampleCount).';
samplePosition_deg = firstPosition_deg + interpolationFraction .* ...
    (secondPosition_deg - firstPosition_deg);

firstFraction = routeProgressEstimate(firstPosition_deg, ...
    startPosition_deg, goalPosition_deg);
secondFraction = routeProgressEstimate(secondPosition_deg, ...
    startPosition_deg, goalPosition_deg);
sampleFraction = firstFraction + interpolationFraction .* ...
    (secondFraction - firstFraction);
sampleTime_s = request.initialState.time_s + sampleFraction .* ...
    (nominalArrivalTime_s - request.initialState.time_s);

clearanceGuard_deg = request.options.safetyMargin_deg + ...
    0.05 .* spatialResolution_deg;
isSafe = true;
for sampleIndex = 1:sampleCount
    clearance_deg = azElObstacleClearance(request.obstacles, ...
        samplePosition_deg(sampleIndex, :), sampleTime_s(sampleIndex), ...
        request.options);
    if clearance_deg <= clearanceGuard_deg
        isSafe = false;
        return;
    end
end
end

function fraction = routeProgressEstimate(position_deg, startPosition_deg, ...
        goalPosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   fraction = routeProgressEstimate(position_deg, startPosition_deg, ...
%       goalPosition_deg)
%**************************************************************************
% PURPOSE
%   - Estimate a deterministic provisional edge time from endpoint distance.
%**************************************************************************
% INPUTS
%   - position_deg, startPosition_deg, goalPosition_deg (1-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - fraction (scalar in [0, 1])
%**************************************************************************
% UNITS
%   - Positions are degrees; fraction is dimensionless.

distanceFromStart_deg = norm(position_deg - startPosition_deg);
distanceToGoal_deg = norm(goalPosition_deg - position_deg);
denominator_deg = distanceFromStart_deg + distanceToGoal_deg;
if denominator_deg == 0
    fraction = 0;
else
    fraction = distanceFromStart_deg ./ denominator_deg;
end
end

function path = shortestPathIndices(edgeWeight, startIndex, goalIndex)
%% Section 0: Header & Readme
% SYNTAX
%   path = shortestPathIndices(edgeWeight, startIndex, goalIndex)
%**************************************************************************
% PURPOSE
%   - Run deterministic Dijkstra search on one dense symmetric graph.
%**************************************************************************
% INPUTS
%   - edgeWeight (N-by-N nonnegative matrix with Inf for missing edges)
%   - startIndex, goalIndex (positive integers)
%**************************************************************************
% OUTPUTS
%   - path (row vector of node indices, empty when disconnected)
%**************************************************************************
% UNITS
%   - Edge weights approximate physical seconds.

nodeCount = size(edgeWeight, 1);
distance = inf(nodeCount, 1);
previous = zeros(nodeCount, 1);
visited = false(nodeCount, 1);
distance(startIndex) = 0;

for visitIndex = 1:nodeCount
    candidateDistance = distance;
    candidateDistance(visited) = Inf;
    [minimumDistance, currentIndex] = min(candidateDistance);
    if ~isfinite(minimumDistance)
        break;
    end
    if currentIndex == goalIndex
        break;
    end
    visited(currentIndex) = true;
    neighbors = find(isfinite(edgeWeight(currentIndex, :)) & ...
        ~visited.');
    for neighborIndex = neighbors
        alternativeDistance = distance(currentIndex) + ...
            edgeWeight(currentIndex, neighborIndex);
        if alternativeDistance < distance(neighborIndex)
            distance(neighborIndex) = alternativeDistance;
            previous(neighborIndex) = currentIndex;
        end
    end
end

if ~isfinite(distance(goalIndex))
    path = zeros(1, 0);
    return;
end
path = goalIndex;
while path(1) ~= startIndex
    predecessor = previous(path(1));
    if predecessor == 0
        path = zeros(1, 0);
        return;
    end
    path = [predecessor, path]; %#ok<AGROW>
end
end

function routePosition_deg = removeCollinearRouteNodes( ...
        routePosition_deg, spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   routePosition_deg = removeCollinearRouteNodes( ...
%       routePosition_deg, spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Remove numerically redundant interior nodes without changing a turn.
%**************************************************************************
% INPUTS
%   - routePosition_deg (N-by-2 numeric)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - routePosition_deg (M-by-2 numeric)
%**************************************************************************
% UNITS
%   - Positions and resolution are degrees.

if size(routePosition_deg, 1) <= 2
    return;
end
keep = true(size(routePosition_deg, 1), 1);
for routeIndex = 2:(size(routePosition_deg, 1) - 1)
    previousVector = routePosition_deg(routeIndex, :) - ...
        routePosition_deg(routeIndex - 1, :);
    nextVector = routePosition_deg(routeIndex + 1, :) - ...
        routePosition_deg(routeIndex, :);
    crossMagnitude_deg2 = abs(previousVector(1) .* nextVector(2) - ...
        previousVector(2) .* nextVector(1));
    scale_deg2 = max(norm(previousVector) .* norm(nextVector), eps);
    isNearlyCollinear = crossMagnitude_deg2 ./ scale_deg2 < 1e-4;
    isTinyEdge = min(norm(previousVector), norm(nextVector)) < ...
        0.05 .* spatialResolution_deg;
    keep(routeIndex) = ~(isNearlyCollinear || isTinyEdge);
end
routePosition_deg = routePosition_deg(keep, :);
end

function isPresent = routeAlreadyPresent(routes, candidateRoute_deg, ...
        spatialResolution_deg)
%% Section 0: Header & Readme
% SYNTAX
%   isPresent = routeAlreadyPresent(routes, candidateRoute_deg, ...
%       spatialResolution_deg)
%**************************************************************************
% PURPOSE
%   - Detect deterministic duplicate routes after graph penalization.
%**************************************************************************
% INPUTS
%   - routes (cell column)
%   - candidateRoute_deg (N-by-2 numeric)
%   - spatialResolution_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - isPresent (logical scalar)
%**************************************************************************
% UNITS
%   - Positions and resolution are degrees.

isPresent = false;
tolerance_deg = max(1e-8, 0.02 .* spatialResolution_deg);
for routeIndex = 1:numel(routes)
    existingRoute_deg = routes{routeIndex};
    if isequal(size(existingRoute_deg), size(candidateRoute_deg)) && ...
            all(vecnorm(existingRoute_deg - candidateRoute_deg, 2, 2) <= ...
            tolerance_deg)
        isPresent = true;
        return;
    end
end
end
