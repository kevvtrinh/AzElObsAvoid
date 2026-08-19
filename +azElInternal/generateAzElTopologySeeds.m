function [seeds, diagnostics] = generateAzElTopologySeeds( ...
        obstacles, initialState, goalState, limits, options, planningTimer)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics] = azElInternal.generateAzElTopologySeeds( ...
%       obstacles, initialState, goalState, limits, options, planningTimer)
%**************************************************************************
% PURPOSE
%   - Generate bounded visibility seeds with time layers and waiting edges.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle struct array)
%   - initialState, goalState (normalized scalar state structs)
%   - limits (normalized physical limits struct)
%   - options (resolved planner options), planningTimer (tic timer handle)
%**************************************************************************
% OUTPUTS
%   - seeds (column struct array): Geometry, time law, reduction flag, and duration.
%   - diagnostics (scalar struct): Graph, time-layer, count, and trace data.
%**************************************************************************
% UNITS
%   - Position is degrees. Time and duration are seconds.
%**************************************************************************
%% Section 1: Create The Direct Visibility Seed
start_deg = initialState.position_deg;
goal_deg = azElInternal.goalPositionAtTime(goalState, goalState.time_s);
if options.AllowAzimuthWrapping
    azimuthTurns = round((start_deg(1) - goal_deg(1)) / 360);
    goal_deg(1) = goal_deg(1) + 360 * azimuthTurns;
end
availableDuration_s = goalState.time_s - initialState.time_s;
directDuration_s = min(availableDuration_s, max(1e-3, ...
    norm(goal_deg - start_deg) / max(limits.maxVelocity_deg_s)));
seedTemplate = struct( ...
    "Index", 0, "Source", "", "position_deg", zeros(0, 2), ...
    "tau", zeros(0, 1), "CorridorBoundary_deg", zeros(0, 2), ...
    "UsesReducedGeometry", false, "EstimatedDuration_s", NaN, "Length_deg", NaN);
directSeed = seedTemplate;
directSeed.Index = 1;
directSeed.Source = "directVisibilityEdge";
directSeed.position_deg = [start_deg; goal_deg];
directSeed.tau = [0; 1];
directSeed.EstimatedDuration_s = directDuration_s;
directSeed.Length_deg = norm(goal_deg - start_deg);
seeds = directSeed;
diagnostics = emptyDiagnostics(start_deg, goal_deg);
diagnostics.SeedCluster.Distance_deg = options.SeedClusterDistance_deg;
if options.DirectSeedOnly || options.MaximumSeedCount == 1 || isempty(obstacles)
    diagnostics.GeneratedSeedCount = numel(seeds);
    return;
end
if toc(planningTimer) >= options.MaximumPlanningTime_s
    diagnostics.GeneratedSeedCount = numel(seeds);
    return;
end
%% Section 2: Build One Protected-Geometry Visibility Graph
sampleTimes_s = obstacleSampleTimes(obstacles, initialState.time_s, goalState.time_s);
[sweptShape, usedDenseEnvelope] = azElInternal.denseSweptSeedEnvelope( ...
    obstacles, sampleTimes_s, [start_deg; goal_deg], 10e3);
if usedDenseEnvelope
    sampledShapeCount = numel(sampleTimes_s) * numel(obstacles);
    sampledNodes_deg = sweptShape.Vertices;
else
    [sweptShape, sampledShapeCount, sampledNodes_deg] = ...
        sweptObstacleShape(obstacles, sampleTimes_s);
end
[sweptShape, diagnostics.SeedCluster] = azElInternal.clusterAzElSeedShape( ...
    sweptShape, options.SeedClusterDistance_deg, [start_deg; goal_deg]);
clusterCreated = diagnostics.SeedCluster.ClusterGroupCount > 0;
spatialSeedTemplate = seedTemplate;
timedSeedTemplate = seedTemplate;
timedSeedTemplate.UsesReducedGeometry = usedDenseEnvelope;
if usedDenseEnvelope
    spatialSeedTemplate.CorridorBoundary_deg = sweptShape.Vertices;
elseif clusterCreated
    spatialSeedTemplate.CorridorBoundary_deg = diagnostics.SeedCluster.ClusterBoundary_deg;
end
spatialSeedTemplate.UsesReducedGeometry = usedDenseEnvelope || clusterCreated;
[nodePosition_deg, edgeCost_deg, graphRecord] = buildVisibilityGraph( ...
    sweptShape, start_deg, goal_deg, options);
diagnostics.Bounds_deg = graphRecord.Bounds_deg;
diagnostics.CandidateOffset_deg = graphRecord.CandidateOffset_deg;
diagnostics.SampleTimes_s = sampleTimes_s;
diagnostics.NodeCount = size(nodePosition_deg, 1);
diagnostics.NodePosition_deg = nodePosition_deg;
diagnostics.VisibilityEdgeCount = graphRecord.VisibilityEdgeCount;
diagnostics.AcceptedEdges_deg = graphRecord.AcceptedEdges_deg;
diagnostics.RejectedEdges_deg = graphRecord.RejectedEdges_deg;
diagnostics.RejectedTransitionCount = graphRecord.RejectedTransitionCount;
diagnostics.SampledShapeCount = sampledShapeCount;
diagnostics.DenseSeedEnvelopeUsed = usedDenseEnvelope;
coverage = diagnostics.Coverage;
coverage.ExactSpatialProposalUsed = ~spatialSeedTemplate.UsesReducedGeometry;
coverage.ReducedSpatialProposalUsed = spatialSeedTemplate.UsesReducedGeometry;
hasChangingObstacles = obstacleHistoryChanges( ...
    obstacles, initialState.time_s, goalState.time_s);
coverage.CompletenessLossReason = "boundedSeedNodeAndTimeSearch";
if spatialSeedTemplate.UsesReducedGeometry
    coverage.CompletenessLossReason = "reducedSpatialProposalAndBoundedSearch";
end
coverage.TimedSearchSuppressionReason = "";
if ~hasChangingObstacles
    coverage.TimedSearchSuppressionReason = "staticObstacleHistory";
end
if usedDenseEnvelope
    diagnostics.DenseSeedEnvelope_deg = sweptShape.Vertices;
end
if toc(planningTimer) >= options.MaximumPlanningTime_s
    if coverage.TimedSearchSuppressionReason == ""
        coverage.TimedSearchSuppressionReason = "planningTimeLimit";
    end
    diagnostics.Coverage = coverage;
    diagnostics.GeneratedSeedCount = numel(seeds);
    return;
end
%% Section 3: Search The Time-Expanded Visibility Graph
if hasChangingObstacles && numel(seeds) < options.MaximumSeedCount
    coverage.TimedSearchAttempted = true;
    coverage.TimedSearchUsesExactObstacles = true;
    directTimedCost_deg = [0, norm(goal_deg - start_deg); norm(goal_deg - start_deg), 0];
    [timedRoute_deg, timedRouteTime_s, timeRecord] = timeExpandedVisibilitySearch( ...
        nodePosition_deg(1:2, :), directTimedCost_deg, ...
        obstacles, initialState, goalState, limits, sampleTimes_s, options, planningTimer);
    diagnostics = appendTimeRecord(diagnostics, timeRecord);
    seeds = appendTimedSeed(seeds, seedTemplate, timedRoute_deg, timedRouteTime_s);
    if toc(planningTimer) >= options.MaximumPlanningTime_s
        coverage.TimedSearchSuppressionReason = "planningTimeLimitAfterDirectTimedSearch";
    elseif numel(seeds) < options.MaximumSeedCount
        coverage.ExtendedTimedSearchAttempted = true;
        maximumTimedNodeCount = 24;
        directNode_deg = start_deg + ((1:7).' / 8) .* (goal_deg - start_deg);
        timedPosition_deg = [start_deg; goal_deg; directNode_deg; ...
            selectCandidateVertices(sampledNodes_deg, start_deg, ...
            goal_deg, maximumTimedNodeCount - 2 - size(directNode_deg, 1))];
        timedEdgeCost_deg = hypot(timedPosition_deg(:, 1) - timedPosition_deg(:, 1).', ...
            timedPosition_deg(:, 2) - timedPosition_deg(:, 2).');
        [timedRoute_deg, timedRouteTime_s, timeRecord] = timeExpandedVisibilitySearch( ...
            timedPosition_deg, timedEdgeCost_deg, obstacles, ...
            initialState, goalState, limits, sampleTimes_s, options, planningTimer);
        diagnostics = appendTimeRecord(diagnostics, timeRecord);
        seeds = appendTimedSeed(seeds, timedSeedTemplate, timedRoute_deg, timedRouteTime_s);
    else
        coverage.TimedSearchSuppressionReason = "seedLimitAfterDirectTimedSearch";
    end
end
diagnostics.Coverage = coverage;
%% Section 4: Search Distinct Spatial Visibility Routes
sideModes = [0 1 -1];
baseNodePath = zeros(0, 1);
for sideModeIndex = 1:numel(sideModes)
    if numel(seeds) >= options.MaximumSeedCount
        break;
    end
    allowedNode = sideAllowedNodes(nodePosition_deg, start_deg, goal_deg, ...
        sideModes(sideModeIndex), graphRecord.CandidateOffset_deg);
    [nodePath, searchRecord] = shortestVisibilityPath(edgeCost_deg, allowedNode, nodePosition_deg);
    diagnostics = appendSearchRecord(diagnostics, searchRecord);
    if isempty(nodePath)
        continue;
    end
    if isempty(baseNodePath)
        baseNodePath = nodePath;
    end
    route_deg = removeCollinearPoints(nodePosition_deg(nodePath, :));
    if routeDuplicates(route_deg, seeds, graphRecord.CandidateOffset_deg)
        continue;
    end
    % The public seed cap bounds this public structure growth.
    seeds(end + 1, 1) = createSpatialSeed(spatialSeedTemplate, ...
        numel(seeds) + 1, route_deg, ...
        directDuration_s, availableDuration_s, limits); %#ok<AGROW>
end
%% Section 5: Add Edge-Removal Visibility Alternatives
if numel(seeds) < options.MaximumSeedCount && numel(baseNodePath) > 2
    for pathEdgeIndex = 1:numel(baseNodePath) - 1
        if numel(seeds) >= options.MaximumSeedCount
            break;
        end
        alternativeCost_deg = edgeCost_deg;
        firstNodeIndex = baseNodePath(pathEdgeIndex);
        secondNodeIndex = baseNodePath(pathEdgeIndex + 1);
        alternativeCost_deg(firstNodeIndex, secondNodeIndex) = Inf;
        alternativeCost_deg(secondNodeIndex, firstNodeIndex) = Inf;
        [nodePath, searchRecord] = shortestVisibilityPath( ...
            alternativeCost_deg, true(size(nodePosition_deg, 1), 1), nodePosition_deg);
        diagnostics = appendSearchRecord(diagnostics, searchRecord);
        if isempty(nodePath)
            continue;
        end
        route_deg = removeCollinearPoints(nodePosition_deg(nodePath, :));
        if routeDuplicates(route_deg, seeds, graphRecord.CandidateOffset_deg)
            continue;
        end
        % The public seed cap is nine, so this growth remains bounded.
        seeds(end + 1, 1) = createSpatialSeed(spatialSeedTemplate, ...
            numel(seeds) + 1, route_deg, ...
            directDuration_s, availableDuration_s, limits); %#ok<AGROW>
    end
end
diagnostics.GeneratedSeedCount = numel(seeds);
end
%% Section 6: Local Functions
function [sweptShape, sampledShapeCount, sampledNodes_deg] = sweptObstacleShape( ...
        obstacles, sampleTimes_s)
%   - Union protected source and midpoint geometry for seed construction.
maximumShapeCount = numel(sampleTimes_s) * numel(obstacles);
sampledShapes = cell(maximumShapeCount, 1);
sampledShapeCount = 0;
sampledNodes_deg = zeros(0, 2);
for sampleTimeIndex = 1:numel(sampleTimes_s)
    sampleTime_s = sampleTimes_s(sampleTimeIndex);
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacleShapeAtTime(obstacles(obstacleIndex), sampleTime_s);
        if isempty(shape.Vertices)
            continue;
        end
        sampledShapeCount = sampledShapeCount + 1;
        sampledShapes{sampledShapeCount} = shape;
        sampledNodes_deg = [sampledNodes_deg; shape.Vertices]; %#ok<AGROW>
    end
end
sampledNodes_deg = sampledNodes_deg(all(isfinite(sampledNodes_deg), 2), :);
if sampledShapeCount == 0
    sweptShape = polyshape();
else
    % One balanced Boolean operation avoids repeated growth of the union.
    sweptShape = union([sampledShapes{1:sampledShapeCount}]);
end
end
function [nodePosition_deg, edgeCost_deg, record] = ...
        buildVisibilityGraph(sweptShape, start_deg, goal_deg, options)
%   - Build one bounded exact segment-visibility graph around swept geometry.
allPosition_deg = [start_deg; goal_deg; sweptShape.Vertices];
minimum_deg = min(allPosition_deg, [], 1);
maximum_deg = max(allPosition_deg, [], 1);
coordinateScale_deg = max(1, max(abs(allPosition_deg), [], "all"));
% This seed-only offset prevents candidate vertices from lying exactly on
% protected boundaries. It does not alter collision geometry or its margin.
candidateOffset_deg = max(1e-3, 256 * eps(coordinateScale_deg));
[edgeStart_deg, edgeEnd_deg] = boundaryEdges(sweptShape);
visibilityWorkBudget = 1e6;
candidateLimit = floor(sqrt(2 * visibilityWorkBudget / max(1, size(edgeStart_deg, 1)))) - 2;
candidateLimit = min(96, max(24, candidateLimit));
candidateShape = sweptShape;
if ~isempty(sweptShape.Vertices)
    candidateShape = polybuffer(sweptShape, candidateOffset_deg, "JointType", "miter");
end
candidatePosition_deg = candidateShape.Vertices;
insideWorkspace = ...
    candidatePosition_deg(:, 1) >= options.AzimuthInterval_deg(1) & ...
    candidatePosition_deg(:, 1) <= options.AzimuthInterval_deg(2) & ...
    candidatePosition_deg(:, 2) >= options.ElevationInterval_deg(1) & ...
    candidatePosition_deg(:, 2) <= options.ElevationInterval_deg(2);
candidatePosition_deg = candidatePosition_deg(insideWorkspace, :);
candidatePosition_deg = selectCandidateVertices( ...
    candidatePosition_deg, start_deg, goal_deg, candidateLimit);
nodePosition_deg = unique([start_deg; goal_deg; candidatePosition_deg], "rows", "stable");
nodeCount = size(nodePosition_deg, 1);
edgeCost_deg = Inf(nodeCount);
edgeCost_deg(1:nodeCount + 1:end) = 0;
visibilityEdgeCount = 0;
rejectedTransitionCount = 0;
maximumRetainedEdgeCount = 2000;
acceptedEdges_deg = zeros(maximumRetainedEdgeCount, 4);
rejectedEdges_deg = zeros(maximumRetainedEdgeCount, 4);
for firstNodeIndex = 1:nodeCount - 1
    for secondNodeIndex = firstNodeIndex + 1:nodeCount
        firstPosition_deg = nodePosition_deg(firstNodeIndex, :);
        secondPosition_deg = nodePosition_deg(secondNodeIndex, :);
        if segmentIsVisible( ...
                firstPosition_deg, secondPosition_deg, sweptShape, ...
                edgeStart_deg, edgeEnd_deg)
            distance_deg = norm(secondPosition_deg - firstPosition_deg);
            edgeCost_deg(firstNodeIndex, secondNodeIndex) = distance_deg;
            edgeCost_deg(secondNodeIndex, firstNodeIndex) = distance_deg;
            visibilityEdgeCount = visibilityEdgeCount + 1;
            if visibilityEdgeCount <= maximumRetainedEdgeCount
                acceptedEdges_deg(visibilityEdgeCount, :) = ...
                    [firstPosition_deg, secondPosition_deg];
            end
        else
            rejectedTransitionCount = rejectedTransitionCount + 1;
            if rejectedTransitionCount <= maximumRetainedEdgeCount
                rejectedEdges_deg(rejectedTransitionCount, :) = ...
                    [firstPosition_deg, secondPosition_deg];
            end
        end
    end
end
record = struct( ...
    "Bounds_deg", [minimum_deg(1), maximum_deg(1), minimum_deg(2), maximum_deg(2)], ...
    "CandidateOffset_deg", candidateOffset_deg, "VisibilityEdgeCount", visibilityEdgeCount, ...
    "AcceptedEdges_deg", acceptedEdges_deg( ...
    1:min(visibilityEdgeCount, maximumRetainedEdgeCount), :), ...
    "RejectedEdges_deg", rejectedEdges_deg( ...
    1:min(rejectedTransitionCount, maximumRetainedEdgeCount), :), ...
    "RejectedTransitionCount", rejectedTransitionCount);
end
function selectedPosition_deg = selectCandidateVertices( ...
        candidatePosition_deg, start_deg, goal_deg, maximumCount)
%   - Bound dense polygon input while retaining global and endpoint supports.
candidatePosition_deg = unique(candidatePosition_deg, "rows", "stable");
candidateCount = size(candidatePosition_deg, 1);
if candidateCount <= maximumCount
    selectedPosition_deg = candidatePosition_deg;
    return;
end
endpointNeighborCount = min(4, floor(maximumCount / 6));
directionCount = min(16, floor(maximumCount / 3));
uniformCount = maximumCount - directionCount - 2 * endpointNeighborCount;
selectedIndex = unique(round(linspace(1, candidateCount, uniformCount))).';
direction_rad = (0:directionCount - 1).' * (2 * pi / directionCount);
direction = [cos(direction_rad), sin(direction_rad)];
for directionIndex = 1:size(direction, 1)
    [~, supportIndex] = max(candidatePosition_deg * direction(directionIndex, :).');
    selectedIndex(end + 1, 1) = supportIndex; %#ok<AGROW>
end
for reference_deg = [start_deg; goal_deg].'
    distance_deg = vecnorm(candidatePosition_deg - reference_deg.', 2, 2);
    [~, order] = sort(distance_deg, "ascend");
    selectedIndex = [selectedIndex; order(1:endpointNeighborCount)]; %#ok<AGROW>
end
selectedIndex = unique(selectedIndex, "stable");
selectedIndex = selectedIndex(1:min(maximumCount, numel(selectedIndex)));
selectedPosition_deg = candidatePosition_deg(selectedIndex, :);
end
function [edgeStart_deg, edgeEnd_deg] = boundaryEdges(shape)
%   - Convert polyshape rings to closed boundary-edge pairs.
[azimuth_deg, elevation_deg] = boundary(shape);
edgeStart_deg = zeros(0, 2);
edgeEnd_deg = zeros(0, 2);
finiteRow = isfinite(azimuth_deg) & isfinite(elevation_deg);
runStart = find(finiteRow & [true; ~finiteRow(1:end - 1)]);
runEnd = find(finiteRow & [~finiteRow(2:end); true]);
for runIndex = 1:numel(runStart)
    ring_deg = [ ...
        azimuth_deg(runStart(runIndex):runEnd(runIndex)), ...
        elevation_deg(runStart(runIndex):runEnd(runIndex))];
    if size(ring_deg, 1) < 2
        continue;
    end
    if norm(ring_deg(end, :) - ring_deg(1, :)) <= 1e-12
        ring_deg(end, :) = [];
    end
    edgeStart_deg = [edgeStart_deg; ring_deg]; %#ok<AGROW>
    edgeEnd_deg = [edgeEnd_deg; ring_deg([2:end 1], :)]; %#ok<AGROW>
end
end
function visible = segmentIsVisible( ...
        firstPosition_deg, secondPosition_deg, shape, ...
        edgeStart_deg, edgeEnd_deg)
%   - Reject a segment that enters, crosses, or touches protected geometry.
visible = true;
if isempty(shape.Vertices)
    return;
end
midpoint_deg = (firstPosition_deg + secondPosition_deg) / 2;
if isinterior(shape, midpoint_deg(1), midpoint_deg(2))
    visible = false;
    return;
end
segment_deg = secondPosition_deg - firstPosition_deg;
boundarySegment_deg = edgeEnd_deg - edgeStart_deg;
offset_deg = edgeStart_deg - firstPosition_deg;
denominator_deg2 = segment_deg(1) * boundarySegment_deg(:, 2) - ...
    segment_deg(2) * boundarySegment_deg(:, 1);
coordinateScale_deg = max(1, max(abs([ ...
    firstPosition_deg, secondPosition_deg, ...
    edgeStart_deg(:).', edgeEnd_deg(:).'])));
intersectionTolerance_deg2 = 512 * eps(coordinateScale_deg^2);
nonparallel = abs(denominator_deg2) > intersectionTolerance_deg2;
firstFraction = Inf(size(denominator_deg2));
secondFraction = Inf(size(denominator_deg2));
firstFraction(nonparallel) = ( ...
    offset_deg(nonparallel, 1) .* ...
    boundarySegment_deg(nonparallel, 2) - ...
    offset_deg(nonparallel, 2) .* ...
    boundarySegment_deg(nonparallel, 1)) ./ ...
    denominator_deg2(nonparallel);
secondFraction(nonparallel) = ( ...
    offset_deg(nonparallel, 1) * segment_deg(2) - ...
    offset_deg(nonparallel, 2) * segment_deg(1)) ./ ...
    denominator_deg2(nonparallel);
crossesBoundary = nonparallel & ...
    firstFraction >= -1e-12 & firstFraction <= 1 + 1e-12 & ...
    secondFraction >= -1e-12 & secondFraction <= 1 + 1e-12;
if any(crossesBoundary)
    visible = false;
    return;
end
parallelOffset_deg2 = offset_deg(:, 1) * segment_deg(2) - offset_deg(:, 2) * segment_deg(1);
collinear = ~nonparallel & abs(parallelOffset_deg2) <= intersectionTolerance_deg2;
if any(collinear)
    segmentLengthSquared_deg2 = sum(segment_deg.^2);
    projection = (edgeStart_deg(collinear, :) - firstPosition_deg) * ...
        segment_deg.' / max(segmentLengthSquared_deg2, eps);
    nextProjection = (edgeEnd_deg(collinear, :) - firstPosition_deg) * ...
        segment_deg.' / max(segmentLengthSquared_deg2, eps);
    overlaps = max(min(projection, nextProjection), 0) <= ...
        min(max(projection, nextProjection), 1) + 1e-12;
    visible = ~any(overlaps);
end
end
function [route_deg, routeTime_s, record] = ...
        timeExpandedVisibilitySearch( ...
        nodePosition_deg, edgeCost_deg, obstacles, initialState, ...
        goalState, limits, sampleTimes_s, options, planningTimer)
%   - Search forward time layers with visible motion and waiting edges.
layerTimes_s = boundedTimeLayers(sampleTimes_s, initialState.time_s, goalState.time_s, 17);
layerCount = numel(layerTimes_s);
nodeCount = size(nodePosition_deg, 1);
nodeIsFree = false(layerCount, nodeCount);
for layerIndex = 1:layerCount
    if toc(planningTimer) >= options.MaximumPlanningTime_s
        break;
    end
    queryTime_s = repmat(layerTimes_s(layerIndex), nodeCount, 1);
    nodeIsFree(layerIndex, :) = ~queryAzElTimeObstacle( ...
        obstacles, nodePosition_deg(:, 1), ...
        nodePosition_deg(:, 2), queryTime_s).';
end
reachable = false(layerCount, nodeCount);
spatialCost_deg = Inf(layerCount, nodeCount);
parentLayerIndex = zeros(layerCount, nodeCount, "uint16");
parentNodeIndex = zeros(layerCount, nodeCount, "uint16");
reachable(1, 1) = nodeIsFree(1, 1);
spatialCost_deg(1, 1) = 0;
waitEdgeCount = 0;
motionEdgeCount = 0;
rejectedTransitionCount = 0;
expandedCount = 0;
exploredNodes_deg = zeros(0, 2);
for layerIndex = 1:layerCount - 1
    if toc(planningTimer) >= options.MaximumPlanningTime_s
        break;
    end
    if options.GoalTimeMode == "earliestArrival" && reachable(layerIndex, 2)
        break;
    end
    currentNodeIndices = find(reachable(layerIndex, :));
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        exploredNodes_deg(end + 1, :) = ...
            nodePosition_deg(currentNodeIndex, :); %#ok<AGROW>
        if nodeIsFree(layerIndex + 1, currentNodeIndex) && ...
                motionEdgeIsClear( ...
                obstacles, nodePosition_deg(currentNodeIndex, :), ...
                nodePosition_deg(currentNodeIndex, :), ...
                layerTimes_s(layerIndex), ...
                layerTimes_s(layerIndex + 1))
            [reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex] = updateTemporalState( ...
                reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex, layerIndex, currentNodeIndex, ...
                layerIndex + 1, currentNodeIndex, 0);
            waitEdgeCount = waitEdgeCount + 1;
        else
            rejectedTransitionCount = rejectedTransitionCount + 1;
        end
        visibleNeighbor = find(isfinite(edgeCost_deg(currentNodeIndex, :)));
        targetNodeIndices = unique([1, 2, visibleNeighbor]);
        targetNodeIndices(targetNodeIndices == currentNodeIndex) = [];
        for targetNodeIndex = reshape(targetNodeIndices, 1, [])
            displacement_deg = nodePosition_deg(targetNodeIndex, :) - ...
                nodePosition_deg(currentNodeIndex, :);
            speedDuration_s = abs(displacement_deg) ./ limits.maxVelocity_deg_s;
            accelerationDuration_s = 2 * sqrt(abs(displacement_deg) ./ ...
                limits.maxAcceleration_deg_s2);
            minimumDuration_s = max([speedDuration_s, accelerationDuration_s]);
            targetLayerIndex = find( ...
                layerTimes_s > layerTimes_s(layerIndex) + ...
                minimumDuration_s - 1e-12, 1, "first");
            if isempty(targetLayerIndex) || ...
                    ~nodeIsFree(targetLayerIndex, targetNodeIndex)
                rejectedTransitionCount = rejectedTransitionCount + 1;
                continue;
            end
            isClear = motionEdgeIsClear( ...
                obstacles, nodePosition_deg(currentNodeIndex, :), ...
                nodePosition_deg(targetNodeIndex, :), ...
                layerTimes_s(layerIndex), ...
                layerTimes_s(targetLayerIndex));
            if ~isClear
                rejectedTransitionCount = rejectedTransitionCount + 1;
                continue;
            end
            [reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex] = updateTemporalState( ...
                reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex, layerIndex, currentNodeIndex, ...
                targetLayerIndex, targetNodeIndex, ...
                norm(displacement_deg));
            motionEdgeCount = motionEdgeCount + 1;
        end
    end
end
deepestLayerIndex = find(any(reachable, 2), 1, "last");
frontierNodes_deg = zeros(0, 2);
if ~isempty(deepestLayerIndex)
    frontierNodes_deg = nodePosition_deg( ...
        reachable(deepestLayerIndex, :), :);
end
if options.GoalTimeMode == "earliestArrival"
    goalLayerIndex = find(reachable(:, 2), 1, "first");
else
    goalLayerIndex = layerCount;
    if ~reachable(goalLayerIndex, 2)
        goalLayerIndex = [];
    end
end
[route_deg, routeTime_s] = reconstructTimedRoute( ...
    nodePosition_deg, layerTimes_s, parentLayerIndex, parentNodeIndex, ...
    goalLayerIndex);
record = struct("LayerTimes_s", layerTimes_s, "NodeCount", nodeCount, ...
    "WaitEdgeCount", waitEdgeCount, "MotionEdgeCount", motionEdgeCount, ...
    "RejectedTransitionCount", rejectedTransitionCount, ...
    "ExpandedCount", expandedCount, "ExploredNodes_deg", exploredNodes_deg, ...
    "FrontierNodes_deg", frontierNodes_deg);
end
function layerTimes_s = boundedTimeLayers( ...
        sampleTimes_s, startTime_s, endTime_s, maximumLayerCount)
%   - Retain source timing and a uniform base in one bounded layer set.
uniformTime_s = linspace(startTime_s, endTime_s, 9).';
candidateTime_s = unique([startTime_s; sampleTimes_s(:); ...
    uniformTime_s; endTime_s]);
candidateTime_s = candidateTime_s( ...
    candidateTime_s >= startTime_s & candidateTime_s <= endTime_s);
if numel(candidateTime_s) <= maximumLayerCount
    layerTimes_s = candidateTime_s;
    return;
end
targetTime_s = linspace( ...
    startTime_s, endTime_s, maximumLayerCount).';
selectedIndex = zeros(maximumLayerCount, 1);
for targetIndex = 1:maximumLayerCount
    [~, selectedIndex(targetIndex)] = min( ...
        abs(candidateTime_s - targetTime_s(targetIndex)));
end
selectedIndex = unique([1; selectedIndex; numel(candidateTime_s)]);
layerTimes_s = candidateTime_s(selectedIndex);
end
function clear = motionEdgeIsClear( ...
        obstacles, firstPosition_deg, secondPosition_deg, ...
        firstTime_s, secondTime_s)
%   - Check a moving seed edge at deterministic trajectory-time samples.
sampleCount = 13;
sampleFraction = linspace(0, 1, sampleCount).';
samplePosition_deg = firstPosition_deg + sampleFraction .* ...
    (secondPosition_deg - firstPosition_deg);
sampleTime_s = firstTime_s + sampleFraction * ...
    (secondTime_s - firstTime_s);
occupied = queryAzElTimeObstacle( ...
    obstacles, samplePosition_deg(:, 1), ...
    samplePosition_deg(:, 2), sampleTime_s);
clear = ~any(occupied);
end
function [reachable, spatialCost_deg, parentLayerIndex, ...
        parentNodeIndex] = updateTemporalState( ...
        reachable, spatialCost_deg, parentLayerIndex, parentNodeIndex, ...
        sourceLayerIndex, sourceNodeIndex, targetLayerIndex, ...
        targetNodeIndex, edgeLength_deg)
%   - Apply deterministic shortest-distance tie breaking at one timed state.
trialCost_deg = spatialCost_deg( ...
    sourceLayerIndex, sourceNodeIndex) + edgeLength_deg;
storedCost_deg = spatialCost_deg(targetLayerIndex, targetNodeIndex);
costIsEqual = abs(trialCost_deg - storedCost_deg) <= 1e-12;
isLaterFinalTransition = targetLayerIndex == size(reachable, 1) && edgeLength_deg > 0 && ...
    sourceLayerIndex > double(parentLayerIndex( ...
    targetLayerIndex, targetNodeIndex));
if trialCost_deg > storedCost_deg + 1e-12 || ...
        (costIsEqual && ~isLaterFinalTransition)
    return;
end
reachable(targetLayerIndex, targetNodeIndex) = true;
spatialCost_deg(targetLayerIndex, targetNodeIndex) = trialCost_deg;
parentLayerIndex(targetLayerIndex, targetNodeIndex) = ...
    uint16(sourceLayerIndex);
parentNodeIndex(targetLayerIndex, targetNodeIndex) = ...
    uint16(sourceNodeIndex);
end
function [route_deg, routeTime_s] = reconstructTimedRoute( ...
        nodePosition_deg, layerTimes_s, parentLayerIndex, ...
        parentNodeIndex, goalLayerIndex)
%   - Reconstruct one forward time-layer path without losing wait states.
route_deg = zeros(0, 2);
routeTime_s = zeros(0, 1);
if isempty(goalLayerIndex)
    return;
end
layerPath = goalLayerIndex;
nodePath = 2;
while ~(layerPath(1) == 1 && nodePath(1) == 1)
    priorLayerIndex = double(parentLayerIndex( ...
        layerPath(1), nodePath(1)));
    priorNodeIndex = double(parentNodeIndex( ...
        layerPath(1), nodePath(1)));
    if priorLayerIndex == 0 || priorNodeIndex == 0
        route_deg = zeros(0, 2);
        routeTime_s = zeros(0, 1);
        return;
    end
    layerPath = [priorLayerIndex; layerPath]; %#ok<AGROW>
    nodePath = [priorNodeIndex; nodePath]; %#ok<AGROW>
end
route_deg = nodePosition_deg(nodePath, :);
routeTime_s = layerTimes_s(layerPath);
end
function source = timedRouteSource(route_deg)
%   - Name a pure direct waiting path separately from a timed detour.
positionChanges = [true; vecnorm(diff(route_deg, 1, 1), 2, 2) > 1e-12];
distinctRoute_deg = route_deg(positionChanges, :);
hasWait = any(~positionChanges(2:end));
if hasWait && size(distinctRoute_deg, 1) == 2
    source = "directWait";
else
    source = "timeExpandedVisibilityGraph";
end
end
function tau = normalizedTimeLaw(time_s)
%   - Convert strictly increasing layer times to a normalized seed law.
duration_s = time_s(end) - time_s(1);
tau = (time_s - time_s(1)) / duration_s;
end
function duplicate = temporalSeedDuplicates(seed, seeds)
%   - Compare both geometry and time law so distinct waits remain available.
duplicate = false;
sampleTau = linspace(0, 1, 101).';
sampledRoute_deg = interp1(seed.tau, seed.position_deg, sampleTau, "linear");
for seedIndex = 1:numel(seeds)
    sampledSeed_deg = interp1(seeds(seedIndex).tau, ...
        seeds(seedIndex).position_deg, sampleTau, "linear");
    priorDuration_s = seeds(seedIndex).EstimatedDuration_s;
    durationTolerance_s = 1e-9 * max([1, abs(seed.EstimatedDuration_s), abs(priorDuration_s)]);
    if abs(seed.EstimatedDuration_s - priorDuration_s) <= ...
            durationTolerance_s && ...
            max(vecnorm(sampledRoute_deg - sampledSeed_deg, 2, 2)) <= 1e-6
        duplicate = true;
        return;
    end
end
end
function seeds = appendTimedSeed( ...
        seeds, seedTemplate, route_deg, routeTime_s)
%   - Append one distinct timed visibility route within the public seed cap.
if isempty(route_deg)
    return;
end
timedSeed = seedTemplate;
timedSeed.Index = numel(seeds) + 1;
timedSeed.Source = timedRouteSource(route_deg);
timedSeed.position_deg = route_deg;
timedSeed.tau = normalizedTimeLaw(routeTime_s);
timedSeed.EstimatedDuration_s = routeTime_s(end) - routeTime_s(1);
timedSeed.Length_deg = polylineLength(route_deg);
if ~temporalSeedDuplicates(timedSeed, seeds)
    seeds(end + 1, 1) = timedSeed;
end
end
function allowedNode = sideAllowedNodes( ...
        nodePosition_deg, start_deg, goal_deg, sideMode, tolerance_deg)
%   - Restrict one visibility search to an input-defined side of the route.
allowedNode = true(size(nodePosition_deg, 1), 1);
if sideMode == 0
    return;
end
sideValue_deg2 = signedSide( ...
    nodePosition_deg(:, 1), nodePosition_deg(:, 2), start_deg, goal_deg);
sideTolerance_deg2 = tolerance_deg * ...
    max(1, norm(goal_deg - start_deg));
if sideMode > 0
    allowedNode = sideValue_deg2 >= -sideTolerance_deg2;
else
    allowedNode = sideValue_deg2 <= sideTolerance_deg2;
end
allowedNode(1:2) = true;
end
function [nodePath, record] = shortestVisibilityPath( ...
        edgeCost_deg, allowedNode, nodePosition_deg)
%   - Run deterministic Dijkstra search and retain expanded visibility nodes.
nodeCount = size(edgeCost_deg, 1);
costToCome_deg = Inf(nodeCount, 1);
parentNodeIndex = zeros(nodeCount, 1, "uint16");
closed = ~allowedNode(:);
costToCome_deg(1) = 0;
expandedCount = 0;
rejectedTransitionCount = 0;
exploredNodes_deg = zeros(0, 2);
while ~closed(2)
    unsettledCost_deg = costToCome_deg;
    unsettledCost_deg(closed) = Inf;
    [currentCost_deg, currentNodeIndex] = min(unsettledCost_deg);
    if ~isfinite(currentCost_deg)
        break;
    end
    closed(currentNodeIndex) = true;
    if currentNodeIndex == 2
        break;
    end
    expandedCount = expandedCount + 1;
    exploredNodes_deg(end + 1, :) = ...
        nodePosition_deg(currentNodeIndex, :); %#ok<AGROW>
    neighborNodeIndices = find(isfinite(edgeCost_deg(currentNodeIndex, :)));
    for neighborNodeIndex = reshape(neighborNodeIndices, 1, [])
        if closed(neighborNodeIndex) || ...
                neighborNodeIndex == currentNodeIndex
            rejectedTransitionCount = rejectedTransitionCount + 1;
            continue;
        end
        trialCost_deg = currentCost_deg + ...
            edgeCost_deg(currentNodeIndex, neighborNodeIndex);
        if trialCost_deg < costToCome_deg(neighborNodeIndex) - 1e-12
            costToCome_deg(neighborNodeIndex) = trialCost_deg;
            parentNodeIndex(neighborNodeIndex) = uint16(currentNodeIndex);
        end
    end
end
nodePath = zeros(0, 1);
if isfinite(costToCome_deg(2))
    nodePath = 2;
    while nodePath(1) ~= 1
        parentIndex = double(parentNodeIndex(nodePath(1)));
        if parentIndex == 0
            nodePath = zeros(0, 1);
            break;
        end
        nodePath = [parentIndex; nodePath]; %#ok<AGROW>
    end
end
frontierNodes_deg = nodePosition_deg(isfinite(costToCome_deg) & ~closed, :);
record = struct("ExpandedCount", expandedCount, ...
    "RejectedTransitionCount", rejectedTransitionCount, ...
    "ExploredNodes_deg", exploredNodes_deg, "FrontierNodes_deg", frontierNodes_deg);
end
function diagnostics = appendSearchRecord(diagnostics, record)
%   - Add complete counts and a bounded deterministic diagnostic trace.
diagnostics.ExpandedCount = diagnostics.ExpandedCount + record.ExpandedCount;
diagnostics.RejectedTransitionCount = ...
    diagnostics.RejectedTransitionCount + record.RejectedTransitionCount;
diagnostics.ExploredNodes_deg = appendBoundedTrace( ...
    diagnostics.ExploredNodes_deg, record.ExploredNodes_deg, 2000);
diagnostics.FrontierNodes_deg = appendBoundedTrace( ...
    diagnostics.FrontierNodes_deg, record.FrontierNodes_deg, 2000);
end
function diagnostics = appendTimeRecord(diagnostics, record)
%   - Add one time-layer search without reconstructing its decisions.
diagnostics.TemporalLayerTimes_s = record.LayerTimes_s;
diagnostics.TemporalLayerCount = numel(record.LayerTimes_s);
diagnostics.TemporalNodeCount = max(diagnostics.TemporalNodeCount, record.NodeCount);
diagnostics.WaitEdgeCount = diagnostics.WaitEdgeCount + record.WaitEdgeCount;
diagnostics.MotionEdgeCount = diagnostics.MotionEdgeCount + record.MotionEdgeCount;
diagnostics.ExpandedCount = diagnostics.ExpandedCount + record.ExpandedCount;
diagnostics.RejectedTransitionCount = ...
    diagnostics.RejectedTransitionCount + record.RejectedTransitionCount;
diagnostics.ExploredNodes_deg = appendBoundedTrace( ...
    diagnostics.ExploredNodes_deg, record.ExploredNodes_deg, 2000);
diagnostics.FrontierNodes_deg = appendBoundedTrace( ...
    diagnostics.FrontierNodes_deg, record.FrontierNodes_deg, 2000);
end
function seed = createSpatialSeed( ...
        seedTemplate, seedIndex, route_deg, directDuration_s, ...
        availableDuration_s, limits)
%   - Assemble one spatial visibility route with a length-based time law.
seed = seedTemplate;
seed.Index = seedIndex;
seed.Source = "visibilityGraph";
seed.position_deg = route_deg;
[seed.tau, seed.Length_deg] = routeTau(route_deg);
seed.EstimatedDuration_s = min(availableDuration_s, max( ...
    directDuration_s, seed.Length_deg / max(limits.maxVelocity_deg_s)));
end
function changing = obstacleHistoryChanges(obstacles, startTime_s, endTime_s)
%   - Detect geometry or active-span changes inside the request window.
changing = false;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    hasFiniteActiveSpan = numel(obstacle.time_s) > 1;
    if hasFiniteActiveSpan && ...
            (obstacle.time_s(1) > startTime_s || ...
            obstacle.time_s(end) < endTime_s)
        changing = true;
        return;
    end
    for sampleIndex = 2:numel(obstacle.time_s)
        sameAzimuth = isequaln( ...
            obstacle.az_deg{sampleIndex}, obstacle.az_deg{1});
        sameElevation = isequaln( ...
            obstacle.el_deg{sampleIndex}, obstacle.el_deg{1});
        if ~sameAzimuth || ~sameElevation
            changing = true;
            return;
        end
    end
end
end
function sampleTimes_s = obstacleSampleTimes(obstacles, startTime_s, endTime_s)
%   - Use source times and interval midpoints without event detection.
sampleTimes_s = [startTime_s; endTime_s];
verticesPerLayer = 0;
for obstacleIndex = 1:numel(obstacles)
    time_s = obstacles(obstacleIndex).time_s(:);
    midTime_s = (time_s(1:end - 1) + time_s(2:end)) / 2;
    sampleTimes_s = [sampleTimes_s; time_s; midTime_s]; %#ok<AGROW>
    maximumVertexCount = 0;
    for sampleIndex = 1:numel(obstacles(obstacleIndex).az_deg)
        maximumVertexCount = max(maximumVertexCount, ...
            numel(obstacles(obstacleIndex).az_deg{sampleIndex}));
    end
    verticesPerLayer = verticesPerLayer + maximumVertexCount;
end
sampleTimes_s = unique(sampleTimes_s( ...
    sampleTimes_s >= startTime_s & sampleTimes_s <= endTime_s));
unionVertexBudget = 50e3;
maximumLayerCount = min(33, max(9, ...
    floor(unionVertexBudget / max(1, verticesPerLayer))));
sampleTimes_s = boundedTimeLayers( ...
    sampleTimes_s, startTime_s, endTime_s, maximumLayerCount);
end
function sideValue = signedSide(azimuth_deg, elevation_deg, start_deg, goal_deg)
%   - Measure deterministic side of the input-defined start-to-goal line.
direction_deg = goal_deg - start_deg;
sideValue = direction_deg(1) * (elevation_deg - start_deg(2)) - ...
    direction_deg(2) * (azimuth_deg - start_deg(1));
end
function route_deg = removeCollinearPoints(route_deg)
%   - Remove spatial points that do not change route direction.
if size(route_deg, 1) <= 2
    return;
end
routeStep_deg = diff(route_deg, 1, 1);
turn_deg2 = routeStep_deg(1:end - 1, 1) .* ...
    routeStep_deg(2:end, 2) - ...
    routeStep_deg(1:end - 1, 2) .* routeStep_deg(2:end, 1);
keep = [true; abs(turn_deg2) > 1e-10; true];
route_deg = route_deg(keep, :);
end
function duplicate = routeDuplicates(route_deg, seeds, tolerance_deg)
%   - Reject geometrically indistinguishable bounded graph routes.
duplicate = false;
sampleTau = linspace(0, 1, 101).';
[parameterizedTau, ~] = routeTau(route_deg);
sampledRoute_deg = interp1( ...
    parameterizedTau, route_deg, sampleTau, "linear");
for seedIndex = 1:numel(seeds)
    sampledSeed_deg = interp1( ...
        seeds(seedIndex).tau, seeds(seedIndex).position_deg, ...
        sampleTau, "linear");
    if max(vecnorm(sampledRoute_deg - sampledSeed_deg, 2, 2)) <= ...
            max(1e-6, 0.5 * tolerance_deg)
        duplicate = true;
        return;
    end
end
end
function [tau, length_deg] = routeTau(route_deg)
%   - Parameterize a route by normalized cumulative Euclidean length.
cumulativeLength_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
length_deg = cumulativeLength_deg(end);
if length_deg <= 0
    tau = linspace(0, 1, size(route_deg, 1)).';
else
    tau = cumulativeLength_deg / length_deg;
end
end
function length_deg = polylineLength(route_deg)
%   - Measure route length while repeated wait positions contribute zero.
length_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
end
function values = appendBoundedTrace(values, additions, maximumCount)
%   - Retain a deterministic prefix while preserving complete counts.
remainingCount = maximumCount - size(values, 1);
if remainingCount > 0
    values = [values; additions(1:min(remainingCount, ...
        size(additions, 1)), :)];
end
end
function diagnostics = emptyDiagnostics(start_deg, goal_deg)
%   - Define stable bounded diagnostics before a graph is required.
diagnostics = struct( ...
    "GraphType", "timeExpandedVisibilityGraph", ...
    "Bounds_deg", [NaN NaN NaN NaN], "Resolution_deg", NaN, ...
    "CandidateOffset_deg", NaN, "SampleTimes_s", zeros(0, 1), ...
    "SampledShapeCount", 0, "DenseSeedEnvelopeUsed", false, ...
    "DenseSeedEnvelope_deg", zeros(0, 2), "Coverage", struct( ...
    "ExactSpatialProposalUsed", false, "ReducedSpatialProposalUsed", false, ...
    "TimedSearchAttempted", false, "ExtendedTimedSearchAttempted", false, ...
    "TimedSearchUsesExactObstacles", false, "TimedSearchSuppressionReason", "graphNotBuilt", ...
    "CompletenessLost", true, "CompletenessLossReason", "directSeedOnly"), ...
    "SeedCluster", struct("Distance_deg", 0, "SourceRegionCount", 0, ...
    "ClusterGroupCount", 0, "ClusteredRegionCount", 0, "ClusterBoundary_deg", zeros(0, 2)), ...
    "NodeCount", 0, "NodePosition_deg", zeros(0, 2), ...
    "VisibilityEdgeCount", 0, "AcceptedEdges_deg", zeros(0, 4), ...
    "RejectedEdges_deg", zeros(0, 4), ...
    "TemporalLayerTimes_s", zeros(0, 1), "TemporalLayerCount", 0, ...
    "TemporalNodeCount", 0, "WaitEdgeCount", 0, "MotionEdgeCount", 0, ...
    "ExpandedCount", 0, "RejectedTransitionCount", 0, ...
    "GeneratedSeedCount", 1, "ExploredNodes_deg", zeros(0, 2), ...
    "FrontierNodes_deg", zeros(0, 2), "Start_deg", start_deg, ...
    "Goal_deg", goal_deg, "TraceDownsampleRule", ...
    "Time samples cap at 33, temporal nodes at 24, traces at 2000 each");
end
