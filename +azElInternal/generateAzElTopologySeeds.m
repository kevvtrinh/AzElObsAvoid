function [seeds, diagnostics] = generateAzElTopologySeeds( ...
        obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics] = azElInternal.generateAzElTopologySeeds( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Generate a bounded deterministic set of visibility-graph seeds.
%   - Represent moving-obstacle timing with time layers and waiting edges.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle struct array)
%   - initialState, goalState (normalized scalar state structs)
%   - limits (normalized physical limits struct)
%   - options (resolved planner options)
%**************************************************************************
% OUTPUTS
%   - seeds (column struct array)
%       Geometry, normalized time law, source, and duration estimate.
%   - diagnostics (scalar struct)
%       Visibility nodes, edges, time layers, counts, and bounded trace.
%**************************************************************************
% UNITS
%   - Position is degrees. Time and duration are seconds.
%**************************************************************************

%% Section 1: Create The Direct Visibility Seed
start_deg = initialState.position_deg;
goal_deg = goalPositionAtHorizon(goalState);
if options.AllowAzimuthWrapping
    azimuthTurns = round((start_deg(1) - goal_deg(1)) / 360);
    goal_deg(1) = goal_deg(1) + 360 * azimuthTurns;
end
availableDuration_s = goalState.time_s - initialState.time_s;
directDuration_s = min(availableDuration_s, max(1e-3, ...
    norm(goal_deg - start_deg) / max(limits.maxVelocity_deg_s)));
seedTemplate = struct( ...
    "Index", 0, "Source", "", ...
    "position_deg", zeros(0, 2), "tau", zeros(0, 1), ...
    "EstimatedDuration_s", NaN, "Length_deg", NaN);
directSeed = seedTemplate;
directSeed.Index = 1;
directSeed.Source = "directVisibilityEdge";
directSeed.position_deg = [start_deg; goal_deg];
directSeed.tau = [0; 1];
directSeed.EstimatedDuration_s = directDuration_s;
directSeed.Length_deg = norm(goal_deg - start_deg);
seeds = directSeed;
diagnostics = emptyDiagnostics(start_deg, goal_deg);
if options.DirectSeedOnly || options.MaximumSeedCount == 1 || ...
        isempty(obstacles)
    diagnostics.GeneratedSeedCount = numel(seeds);
    return;
end

%% Section 2: Build One Protected-Geometry Visibility Graph
sampleTimes_s = obstacleSampleTimes( ...
    obstacles, initialState.time_s, goalState.time_s);
[sweptShape, sampledShapeCount, temporalCandidatePosition_deg, ...
    gapCandidatePosition_deg, sweptGeometryMode] = ...
    sweptObstacleShape( ...
    obstacles, sampleTimes_s);
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
diagnostics.RejectedTransitionCount = ...
    graphRecord.RejectedTransitionCount;
diagnostics.SampledShapeCount = sampledShapeCount;
diagnostics.SweptGeometryMode = sweptGeometryMode;
[initialSweptNodePath, sweptSearchRecord] = shortestVisibilityPath( ...
    edgeCost_deg, true(size(nodePosition_deg, 1), 1), ...
    nodePosition_deg);
diagnostics = appendSearchRecord(diagnostics, sweptSearchRecord);

%% Section 3: Search The Time-Expanded Visibility Graph
if obstacleHistoryChanges(obstacles) && ...
        numel(seeds) < options.MaximumSeedCount
    directTimedCost_deg = [0, norm(goal_deg - start_deg); ...
        norm(goal_deg - start_deg), 0];
    [timedRoute_deg, timedRouteTime_s, timeRecord] = ...
        azElInternal.searchAzElTimeExpandedGraph( ...
        nodePosition_deg(1:2, :), directTimedCost_deg, ...
        obstacles, initialState, ...
        goalState, limits, sampleTimes_s, options);
    diagnostics = appendTimeRecord(diagnostics, timeRecord);
    seeds = appendTimedSeed( ...
        seeds, seedTemplate, timedRoute_deg, timedRouteTime_s);
    if numel(seeds) < options.MaximumSeedCount && ...
            (isempty(initialSweptNodePath) || ...
            (sweptGeometryMode == "boundingBoxEnvelope" && ...
            options.GoalTimeMode == "earliestArrival"))
        maximumTimedNodeCount = min(48, ...
            16 + ceil(5 * sqrt(numel(obstacles))));
        gapCandidatePosition_deg = unique( ...
            gapCandidatePosition_deg, "rows", "stable");
        temporalCandidatePosition_deg = unique( ...
            temporalCandidatePosition_deg, "rows", "stable");
        endpointCount = min(4, floor((maximumTimedNodeCount - 2) / 2));
        endpointPosition_deg = unique([endpointSupportVertices( ...
            temporalCandidatePosition_deg, start_deg, ...
            goal_deg - start_deg, endpointCount); ...
            endpointSupportVertices(temporalCandidatePosition_deg, ...
            goal_deg, start_deg - goal_deg, endpointCount)], ...
            "rows", "stable");
        remainingCount = maximumTimedNodeCount - 2 - ...
            size(endpointPosition_deg, 1);
        boundaryCount = ceil(0.55 * remainingCount);
        routeBoundaryPosition_deg = selectCandidateVertices( ...
            temporalCandidatePosition_deg, start_deg, goal_deg, ...
            boundaryCount);
        supportPosition_deg = unique([endpointPosition_deg; ...
            routeBoundaryPosition_deg], "rows", "stable");
        gapCount = maximumTimedNodeCount - 2 - ...
            size(supportPosition_deg, 1);
        gapCandidatePosition_deg = selectCandidateVertices( ...
            gapCandidatePosition_deg, start_deg, goal_deg, gapCount);
        temporalCandidatePosition_deg = [ ...
            supportPosition_deg; gapCandidatePosition_deg];
        timedNodePosition_deg = unique([start_deg; goal_deg; ...
            temporalCandidatePosition_deg], "rows", "stable");
        timedEdgeCost_deg = hypot( ...
            timedNodePosition_deg(:, 1) - timedNodePosition_deg(:, 1).', ...
            timedNodePosition_deg(:, 2) - timedNodePosition_deg(:, 2).');
        [timedRoute_deg, timedRouteTime_s, timeRecord] = ...
            azElInternal.searchAzElTimeExpandedGraph( ...
            timedNodePosition_deg, timedEdgeCost_deg, obstacles, ...
            initialState, goalState, limits, sampleTimes_s, options);
        diagnostics = appendTimeRecord(diagnostics, timeRecord);
        seeds = appendTimedSeed( ...
            seeds, seedTemplate, timedRoute_deg, timedRouteTime_s);
    end
end

%% Section 4: Search Distinct Spatial Visibility Routes

sideModes = [0 1 -1];
baseNodePath = initialSweptNodePath;
for sideModeIndex = 1:numel(sideModes)
    if numel(seeds) >= options.MaximumSeedCount
        break;
    end
    if sideModeIndex == 1
        nodePath = initialSweptNodePath;
    else
        allowedNode = sideAllowedNodes( ...
            nodePosition_deg, start_deg, goal_deg, ...
            sideModes(sideModeIndex), ...
            graphRecord.CandidateOffset_deg);
        [nodePath, searchRecord] = shortestVisibilityPath( ...
            edgeCost_deg, allowedNode, nodePosition_deg);
        diagnostics = appendSearchRecord(diagnostics, searchRecord);
    end
    if isempty(nodePath)
        continue;
    end
    if isempty(baseNodePath)
        baseNodePath = nodePath;
    end
    route_deg = removeCollinearPoints(nodePosition_deg(nodePath, :));
    if routeDuplicates( ...
            route_deg, seeds, graphRecord.CandidateOffset_deg)
        continue;
    end
    % The public seed cap is nine, so bounded growth is clearer than a
    % partially filled public structure array.
    seeds(end + 1, 1) = createSpatialSeed( ...
        seedTemplate, numel(seeds) + 1, route_deg, ...
        directDuration_s, availableDuration_s, limits); %#ok<AGROW>
end

%% Section 5: Add Edge-Removal Visibility Alternatives

if numel(seeds) < options.MaximumSeedCount && ...
        numel(baseNodePath) > 2
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
            alternativeCost_deg, true(size(nodePosition_deg, 1), 1), ...
            nodePosition_deg);
        diagnostics = appendSearchRecord(diagnostics, searchRecord);
        if isempty(nodePath)
            continue;
        end
        route_deg = removeCollinearPoints( ...
            nodePosition_deg(nodePath, :));
        if routeDuplicates( ...
                route_deg, seeds, graphRecord.CandidateOffset_deg)
            continue;
        end
        % The public seed cap is nine, so this growth remains bounded.
        seeds(end + 1, 1) = createSpatialSeed( ...
            seedTemplate, numel(seeds) + 1, route_deg, ...
            directDuration_s, availableDuration_s, limits); %#ok<AGROW>
    end
end
diagnostics.GeneratedSeedCount = numel(seeds);
end

%% Section 6: Local Functions
function [sweptShape, sampledShapeCount, candidatePosition_deg, ...
        gapPosition_deg, sweptGeometryMode] = ...
        sweptObstacleShape( ...
        obstacles, sampleTimes_s)
% PURPOSE
%   - Union samples and retain offset temporal boundary candidates.
maximumShapeCount = numel(sampleTimes_s) * numel(obstacles);
sampledShapes = cell(maximumShapeCount, 1);
sampledCandidatePosition = cell(maximumShapeCount, 1);
gapCandidatePosition = cell(numel(sampleTimes_s), 1);
sampledShapeCount = 0;
for sampleTimeIndex = 1:numel(sampleTimes_s)
    sampleTime_s = sampleTimes_s(sampleTimeIndex);
    sampleCenter_deg = NaN(numel(obstacles), 2);
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacleShapeAtTime( ...
            obstacles(obstacleIndex), sampleTime_s);
        if isempty(shape.Vertices)
            continue;
        end
        sampledShapeCount = sampledShapeCount + 1;
        sampledShapes{sampledShapeCount} = shape;
        vertices_deg = shape.Vertices;
        finiteVertex = all(isfinite(vertices_deg), 2);
        center_deg = mean(vertices_deg(finiteVertex, :), 1);
        sampleCenter_deg(obstacleIndex, :) = center_deg;
        outwardDirection = vertices_deg - center_deg;
        outwardNorm = vecnorm(outwardDirection, 2, 2);
        outwardDirection = outwardDirection ./ max(outwardNorm, eps);
        sampledCandidatePosition{sampledShapeCount} = ...
            vertices_deg + 1e-3 * outwardDirection;
    end
    sampleCenter_deg = sampleCenter_deg( ...
        all(isfinite(sampleCenter_deg), 2), :);
    [firstCenterIndex, secondCenterIndex] = find( ...
        triu(true(size(sampleCenter_deg, 1)), 1));
    if ~isempty(firstCenterIndex)
        centerDistance_deg = vecnorm( ...
            sampleCenter_deg(firstCenterIndex, :) - ...
            sampleCenter_deg(secondCenterIndex, :), 2, 2);
        nearbyPair = centerDistance_deg <= 1.5 * min(centerDistance_deg);
        firstCenterIndex = firstCenterIndex(nearbyPair);
        secondCenterIndex = secondCenterIndex(nearbyPair);
    end
    gapCandidatePosition{sampleTimeIndex} = ( ...
        sampleCenter_deg(firstCenterIndex, :) + ...
        sampleCenter_deg(secondCenterIndex, :)) / 2;
end
if sampledShapeCount == 0
    sweptShape = polyshape();
    sweptGeometryMode = "empty";
elseif sampledShapeCount > 24
    sampledVertices_deg = vertcat( ...
        sampledCandidatePosition{1:sampledShapeCount});
    sampledVertices_deg = sampledVertices_deg( ...
        all(isfinite(sampledVertices_deg), 2), :);
    minimum_deg = min(sampledVertices_deg, [], 1);
    maximum_deg = max(sampledVertices_deg, [], 1);
    sweptShape = polyshape( ...
        [minimum_deg(1), minimum_deg(2); ...
        maximum_deg(1), minimum_deg(2); ...
        maximum_deg; minimum_deg(1), maximum_deg(2)]);
    sweptGeometryMode = "boundingBoxEnvelope";
else
    % One balanced Boolean operation avoids repeated growth of the union.
    sweptShape = union([sampledShapes{1:sampledShapeCount}]);
    sweptGeometryMode = "polygonUnion";
end
candidatePosition_deg = vertcat( ...
    sampledCandidatePosition{1:sampledShapeCount});
gapPosition_deg = vertcat(gapCandidatePosition{:});
end
function [nodePosition_deg, edgeCost_deg, record] = ...
        buildVisibilityGraph(sweptShape, start_deg, goal_deg, options)
% PURPOSE
%   - Build one bounded exact segment-visibility graph around swept geometry.
allPosition_deg = [start_deg; goal_deg; sweptShape.Vertices];
minimum_deg = min(allPosition_deg, [], 1);
maximum_deg = max(allPosition_deg, [], 1);
coordinateScale_deg = max(1, max(abs(allPosition_deg), [], "all"));
% This seed-only offset prevents candidate vertices from lying exactly on
% protected boundaries. It does not alter collision geometry or its margin.
candidateOffset_deg = max(1e-3, 256 * eps(coordinateScale_deg));
candidateShape = sweptShape;
if ~isempty(sweptShape.Vertices)
    candidateShape = polybuffer( ...
        sweptShape, candidateOffset_deg, "JointType", "miter");
end
candidatePosition_deg = candidateShape.Vertices;
if ~isempty(candidatePosition_deg)
    candidateMinimum_deg = min(candidatePosition_deg, [], 1);
    candidateMaximum_deg = max(candidatePosition_deg, [], 1);
    outerClearance_deg = max(1, 0.05 * ...
        min(candidateMaximum_deg - candidateMinimum_deg));
    candidateMinimum_deg = max(candidateMinimum_deg - ...
        outerClearance_deg, [options.AzimuthInterval_deg(1), ...
        options.ElevationInterval_deg(1)]);
    candidateMaximum_deg = min(candidateMaximum_deg + ...
        outerClearance_deg, [options.AzimuthInterval_deg(2), ...
        options.ElevationInterval_deg(2)]);
    outerSupport_deg = [ ...
        candidateMinimum_deg; ...
        candidateMaximum_deg(1), candidateMinimum_deg(2); ...
        candidateMaximum_deg; ...
        candidateMinimum_deg(1), candidateMaximum_deg(2)];
else
    outerSupport_deg = zeros(0, 2);
end
insideWorkspace = ...
    candidatePosition_deg(:, 1) >= options.AzimuthInterval_deg(1) & ...
    candidatePosition_deg(:, 1) <= options.AzimuthInterval_deg(2) & ...
    candidatePosition_deg(:, 2) >= options.ElevationInterval_deg(1) & ...
    candidatePosition_deg(:, 2) <= options.ElevationInterval_deg(2);
candidatePosition_deg = candidatePosition_deg(insideWorkspace, :);
candidatePosition_deg = selectCandidateVertices( ...
    candidatePosition_deg, start_deg, goal_deg, 92);
outerSupport_deg = outerSupport_deg( ...
    outerSupport_deg(:, 1) >= options.AzimuthInterval_deg(1) & ...
    outerSupport_deg(:, 1) <= options.AzimuthInterval_deg(2) & ...
    outerSupport_deg(:, 2) >= options.ElevationInterval_deg(1) & ...
    outerSupport_deg(:, 2) <= options.ElevationInterval_deg(2), :);
candidatePosition_deg = [candidatePosition_deg; outerSupport_deg];
nodePosition_deg = unique( ...
    [start_deg; goal_deg; candidatePosition_deg], "rows", "stable");
nodeCount = size(nodePosition_deg, 1);
edgeCost_deg = Inf(nodeCount);
edgeCost_deg(1:nodeCount + 1:end) = 0;
[edgeStart_deg, edgeEnd_deg] = boundaryEdges(sweptShape);
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
    "Bounds_deg", [minimum_deg(1), maximum_deg(1), ...
    minimum_deg(2), maximum_deg(2)], ...
    "CandidateOffset_deg", candidateOffset_deg, ...
    "VisibilityEdgeCount", visibilityEdgeCount, ...
    "AcceptedEdges_deg", acceptedEdges_deg( ...
    1:min(visibilityEdgeCount, maximumRetainedEdgeCount), :), ...
    "RejectedEdges_deg", rejectedEdges_deg( ...
    1:min(rejectedTransitionCount, maximumRetainedEdgeCount), :), ...
    "RejectedTransitionCount", rejectedTransitionCount);
end
function selectedPosition_deg = selectCandidateVertices( ...
        candidatePosition_deg, start_deg, goal_deg, maximumCount)
% PURPOSE
%   - Retain paired boundary supports near successive direct-route stations.
candidatePosition_deg = unique(candidatePosition_deg, "rows", "stable");
candidateCount = size(candidatePosition_deg, 1);
if candidateCount <= maximumCount
    selectedPosition_deg = candidatePosition_deg;
    return;
end
segment_deg = goal_deg - start_deg;
segmentLength_deg = norm(segment_deg);
segmentLengthSquared_deg2 = max(sum(segment_deg.^2), eps);
projection = (candidatePosition_deg - start_deg) * segment_deg.' / ...
    segmentLengthSquared_deg2;
projection = min(1, max(0, projection));
projectedPosition_deg = start_deg + projection .* segment_deg;
distanceToSegment_deg = vecnorm( ...
    candidatePosition_deg - projectedPosition_deg, 2, 2);
signedOffset_deg = (segment_deg(1) * ...
    (candidatePosition_deg(:, 2) - start_deg(2)) - segment_deg(2) * ...
    (candidatePosition_deg(:, 1) - start_deg(1))) / segmentLength_deg;
stationCount = ceil(maximumCount / 3);
targetProjection = repelem(linspace(0, 1, stationCount).', 3);
targetProjection = targetProjection(1:maximumCount);
targetSide = repmat([1; -1; 1], stationCount, 1);
selectedIndex = zeros(maximumCount, 1);
for selectionIndex = 1:maximumCount
    score_deg = distanceToSegment_deg + segmentLength_deg * ...
        abs(projection - targetProjection(selectionIndex));
    score_deg(targetSide(selectionIndex) * signedOffset_deg < 0) = Inf;
    score_deg(selectedIndex(1:selectionIndex - 1)) = Inf;
    [~, selectedIndex(selectionIndex)] = min(score_deg);
end
selectedPosition_deg = candidatePosition_deg(selectedIndex, :);
end
function selectedPosition_deg = endpointSupportVertices( ...
        candidatePosition_deg, reference_deg, direction_deg, maximumCount)
% PURPOSE
%   - Retain angularly distinct supports near one route endpoint.
offset_deg = candidatePosition_deg - reference_deg;
[~, distanceOrder] = sort(vecnorm(offset_deg, 2, 2), "ascend");
nearbyCount = min(32, numel(distanceOrder));
nearbyPosition_deg = candidatePosition_deg( ...
    distanceOrder(1:nearbyCount), :);
nearbyOffset_deg = nearbyPosition_deg - reference_deg;
angle_rad = atan2( ...
    direction_deg(1) * nearbyOffset_deg(:, 2) - ...
    direction_deg(2) * nearbyOffset_deg(:, 1), ...
    nearbyOffset_deg * direction_deg.');
[~, angleOrder] = sort(angle_rad, "ascend");
supportOrder = unique(round(linspace( ...
    1, numel(angleOrder), min(maximumCount, numel(angleOrder)))));
selectedPosition_deg = nearbyPosition_deg( ...
    angleOrder(supportOrder), :);
end
function [edgeStart_deg, edgeEnd_deg] = boundaryEdges(shape)
% PURPOSE
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
% PURPOSE
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
parallelOffset_deg2 = offset_deg(:, 1) * segment_deg(2) - ...
    offset_deg(:, 2) * segment_deg(1);
collinear = ~nonparallel & ...
    abs(parallelOffset_deg2) <= intersectionTolerance_deg2;
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
function source = timedRouteSource(route_deg)
% PURPOSE
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
% PURPOSE
%   - Convert strictly increasing layer times to a normalized seed law.
duration_s = time_s(end) - time_s(1);
tau = (time_s - time_s(1)) / duration_s;
end
function duplicate = temporalSeedDuplicates(seed, seeds)
% PURPOSE
%   - Compare both geometry and time law so distinct waits remain available.
duplicate = false;
sampleTau = linspace(0, 1, 101).';
sampledRoute_deg = interp1( ...
    seed.tau, seed.position_deg, sampleTau, "linear");
for seedIndex = 1:numel(seeds)
    sampledSeed_deg = interp1( ...
        seeds(seedIndex).tau, seeds(seedIndex).position_deg, ...
        sampleTau, "linear");
    if max(vecnorm(sampledRoute_deg - sampledSeed_deg, 2, 2)) <= 1e-6
        duplicate = true;
        return;
    end
end
end
function seeds = appendTimedSeed( ...
        seeds, seedTemplate, route_deg, routeTime_s)
% PURPOSE
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
% PURPOSE
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
% PURPOSE
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
record = struct( ...
    "ExpandedCount", expandedCount, ...
    "RejectedTransitionCount", rejectedTransitionCount, ...
    "ExploredNodes_deg", exploredNodes_deg);
end
function diagnostics = appendSearchRecord(diagnostics, record)
% PURPOSE
%   - Add complete counts and a bounded deterministic diagnostic trace.
diagnostics.ExpandedCount = diagnostics.ExpandedCount + ...
    record.ExpandedCount;
diagnostics.RejectedTransitionCount = ...
    diagnostics.RejectedTransitionCount + ...
    record.RejectedTransitionCount;
diagnostics.ExploredNodes_deg = appendBoundedTrace( ...
    diagnostics.ExploredNodes_deg, record.ExploredNodes_deg, 2000);
end
function diagnostics = appendTimeRecord(diagnostics, record)
% PURPOSE
%   - Add one time-layer search without reconstructing its decisions.
diagnostics.TemporalLayerTimes_s = record.LayerTimes_s;
diagnostics.TemporalLayerCount = numel(record.LayerTimes_s);
diagnostics.TemporalNodeCount = max( ...
    diagnostics.TemporalNodeCount, record.NodeCount);
diagnostics.TemporalNodePosition_deg = record.NodePosition_deg;
diagnostics.TemporalExploredNodes_deg = appendBoundedTrace( ...
    diagnostics.TemporalExploredNodes_deg, record.ExploredNodes_deg, 2000);
diagnostics.WaitEdgeCount = diagnostics.WaitEdgeCount + ...
    record.WaitEdgeCount;
diagnostics.MotionEdgeCount = diagnostics.MotionEdgeCount + ...
    record.MotionEdgeCount;
diagnostics.ExpandedCount = diagnostics.ExpandedCount + ...
    record.ExpandedCount;
diagnostics.RejectedTransitionCount = ...
    diagnostics.RejectedTransitionCount + ...
    record.RejectedTransitionCount;
diagnostics.ExploredNodes_deg = appendBoundedTrace( ...
    diagnostics.ExploredNodes_deg, record.ExploredNodes_deg, 2000);
end
function seed = createSpatialSeed( ...
        seedTemplate, seedIndex, route_deg, directDuration_s, ...
        availableDuration_s, limits)
% PURPOSE
%   - Assemble one spatial visibility route with a length-based time law.
seed = seedTemplate;
seed.Index = seedIndex;
seed.Source = "visibilityGraph";
seed.position_deg = route_deg;
[seed.tau, seed.Length_deg] = routeTau(route_deg);
seed.EstimatedDuration_s = min(availableDuration_s, max( ...
    directDuration_s, seed.Length_deg / max(limits.maxVelocity_deg_s)));
end
function changing = obstacleHistoryChanges(obstacles)
% PURPOSE
%   - Detect input geometry changes without snapshot event inference.
changing = false;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
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
% PURPOSE
%   - Use source times and interval midpoints without event detection.
sampleTimes_s = [startTime_s; endTime_s];
for obstacleIndex = 1:numel(obstacles)
    time_s = obstacles(obstacleIndex).time_s(:);
    midTime_s = (time_s(1:end - 1) + time_s(2:end)) / 2;
    sampleTimes_s = [sampleTimes_s; time_s; midTime_s]; %#ok<AGROW>
end
sampleTimes_s = unique(sampleTimes_s( ...
    sampleTimes_s >= startTime_s & sampleTimes_s <= endTime_s));
maximumSampleTimeCount = max(9, min(33, ...
    floor(48 / sqrt(max(1, numel(obstacles))))));
if numel(sampleTimes_s) > maximumSampleTimeCount
    targetTime_s = linspace( ...
        startTime_s, endTime_s, maximumSampleTimeCount).';
    selectedIndex = zeros(maximumSampleTimeCount, 1);
    for targetIndex = 1:maximumSampleTimeCount
        [~, selectedIndex(targetIndex)] = min( ...
            abs(sampleTimes_s - targetTime_s(targetIndex)));
    end
    selectedIndex = unique([1; selectedIndex; numel(sampleTimes_s)]);
    sampleTimes_s = sampleTimes_s(selectedIndex);
end
end
function sideValue = signedSide(azimuth_deg, elevation_deg, start_deg, goal_deg)
% PURPOSE
%   - Measure deterministic side of the input-defined start-to-goal line.
direction_deg = goal_deg - start_deg;
sideValue = direction_deg(1) * (elevation_deg - start_deg(2)) - ...
    direction_deg(2) * (azimuth_deg - start_deg(1));
end
function route_deg = removeCollinearPoints(route_deg)
% PURPOSE
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
% PURPOSE
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
% PURPOSE
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
% PURPOSE
%   - Measure route length while repeated wait positions contribute zero.
length_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
end
function values = appendBoundedTrace(values, additions, maximumCount)
% PURPOSE
%   - Retain a deterministic prefix while preserving complete counts.
remainingCount = maximumCount - size(values, 1);
if remainingCount > 0
    values = [values; additions(1:min(remainingCount, ...
        size(additions, 1)), :)];
end
end
function position_deg = goalPositionAtHorizon(goalState)
% PURPOSE
%   - Select the fixed or sampled target position at the planning horizon.
if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    position_deg = interp1( ...
        goalState.targetTime_s, goalState.targetPosition_deg, ...
        goalState.time_s, goalState.InterpolationMethod);
else
    position_deg = goalState.position_deg;
end
end
function diagnostics = emptyDiagnostics(start_deg, goal_deg)
% PURPOSE
%   - Define stable bounded diagnostics before a graph is required.
diagnostics = struct( ...
    "GraphType", "timeExpandedVisibilityGraph", ...
    "Bounds_deg", [NaN NaN NaN NaN], ...
    "Resolution_deg", NaN, ...
    "CandidateOffset_deg", NaN, ...
    "SampleTimes_s", zeros(0, 1), ...
    "SampledShapeCount", 0, ...
    "SweptGeometryMode", "empty", ...
    "NodeCount", 0, ...
    "NodePosition_deg", zeros(0, 2), ...
    "VisibilityEdgeCount", 0, ...
    "AcceptedEdges_deg", zeros(0, 4), ...
    "RejectedEdges_deg", zeros(0, 4), ...
    "TemporalLayerTimes_s", zeros(0, 1), ...
    "TemporalLayerCount", 0, ...
    "TemporalNodeCount", 0, ...
    "TemporalNodePosition_deg", zeros(0, 2), ...
    "TemporalExploredNodes_deg", zeros(0, 2), ...
    "WaitEdgeCount", 0, ...
    "MotionEdgeCount", 0, ...
    "ExpandedCount", 0, ...
    "RejectedTransitionCount", 0, ...
    "GeneratedSeedCount", 1, ...
    "ExploredNodes_deg", zeros(0, 2), ...
    "Start_deg", start_deg, ...
    "Goal_deg", goal_deg, ...
    "TraceDownsampleRule", ...
    "Seed time and temporal-node caps scale with obstacle count");
end
