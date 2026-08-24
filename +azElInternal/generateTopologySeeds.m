function [seeds, diagnostics] = generateTopologySeeds(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics] = azElInternal.generateTopologySeeds( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Generate a small deterministic set of route-topology proposals. The
%     function tries the direct edge first, summarizes obstacle history into
%     a spatial graph, adds a bounded timed proposal when geometry moves, and
%     finally searches distinct homology signatures. Seeds are never treated
%     as proof that a collision-free timed motion exists.
%**************************************************************************
% INPUTS
%   - obstacles (prepared struct array), protected obstacle histories.
%   - initialState, goalState (scalar structs), planning endpoints and time.
%   - limits (scalar struct), workspace and derivative limits.
%   - options (scalar struct), resolved bounded-search controls.
%**************************************************************************
% OUTPUTS
%   - seeds (struct array), deterministic geometric or timed proposals.
%   - diagnostics (scalar struct), complete bounded-search evidence.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; path length is degrees.
%**************************************************************************

%% Section 1: Create The Direct Visibility Seed

% The direct seed is always retained as the baseline candidate, even when it
% crosses an obstacle. Later construction and validation reject it honestly.
start_deg = initialState.position_deg;
goal_deg = azElInternal.goalPositionAtTime(goalState, goalState.time_s);
% Move the goal to the nearest equivalent azimuth turn when circular wrapping is enabled.
if options.AllowAzimuthWrapping
    azimuthTurns = round((start_deg(1) - goal_deg(1)) / 360);
    goal_deg(1) = goal_deg(1) + 360 * azimuthTurns;
end
availableDuration_s = goalState.time_s - initialState.time_s;
directDuration_s = min(availableDuration_s, max(1e-3, norm(goal_deg - start_deg) / max(limits.maxVelocity_deg_s)));
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
% A direct-only request, one-slot budget, or obstacle-free scene needs no topology graph.
if options.DirectSeedOnly || options.MaximumSeedCount == 1 || isempty(obstacles)
    diagnostics.GeneratedSeedCount = numel(seeds);
    return;
end

%% Section 2: Build One Protected-Geometry Visibility Graph

% Spatial graph geometry is used only to propose routes. Exact prepared
% obstacle histories remain authoritative for timed edges and final motion
% validation, so a reduced envelope cannot make an invalid path succeed.
sampleTimes_s = obstacleSampleTimes(obstacles, initialState.time_s, goalState.time_s);
[sweptShape, usedDenseEnvelope] = azElInternal.denseSweptEnvelope( ...
    obstacles, sampleTimes_s, [start_deg; goal_deg], 10e3);
hasChangingObstacles = obstacleHistoryChanges(obstacles, initialState.time_s, goalState.time_s);
coverage = diagnostics.Coverage;
coverage.TimedSearchSuppressionReason = "";
diagnostics.SampleTimes_s = sampleTimes_s;
% Static histories cannot benefit from the additional time-layer search.
if ~hasChangingObstacles
    coverage.TimedSearchSuppressionReason = "staticObstacleHistory";
elseif usedDenseEnvelope
    coverage.TimedSearchSuppressionReason = "timedQueryWorkLimit";
else
    coverage.TimedSearchAttempted = true;
    coverage.TimedSearchUsesExactObstacles = true;
    directTimedPosition_deg = [start_deg; goal_deg];
    directTimedCost_deg = [0, directSeed.Length_deg; directSeed.Length_deg, 0];
    [timedRoute_deg, timedRouteTime_s, timeRecord] = ...
        azElInternal.timeExpandedVisibilitySearch( ...
        directTimedPosition_deg, directTimedCost_deg, obstacles, ...
        initialState, goalState, limits, sampleTimes_s, options);
    diagnostics = appendTimeRecord(diagnostics, timeRecord);
    seeds = appendTimedSeed(seeds, seedTemplate, timedRoute_deg, timedRouteTime_s);
end
% Record dense-envelope work explicitly because it consumes a different bounded search budget.
if usedDenseEnvelope
    sampledShapeCount = numel(sampleTimes_s) * numel(obstacles);
    sampledNodes_deg = sweptShape.Vertices;
else
    [sweptShape, sampledShapeCount, sampledNodes_deg] = sweptObstacleShape(obstacles, sampleTimes_s);
end
[sweptShape, diagnostics.SeedCluster] = azElInternal.clusterSeedShape( ...
    sweptShape, options.SeedClusterDistance_deg, [start_deg; goal_deg], ...
    "azElInternal:clusterSeedShape:InvalidShape");
clusterCreated = diagnostics.SeedCluster.ClusterGroupCount > 0;
representative_deg = homologyRepresentatives(sweptShape);
diagnostics.HomologyRepresentative_deg = representative_deg;
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
    sweptShape, start_deg, goal_deg, limits, false, 1, 0);
diagnostics.Bounds_deg = graphRecord.Bounds_deg;
diagnostics.CandidateOffset_deg = graphRecord.CandidateOffset_deg;
diagnostics.CandidateOffsetRetryCount = graphRecord.CandidateOffsetRetryCount;
diagnostics.NodeCount = size(nodePosition_deg, 1);
diagnostics.NodePosition_deg = nodePosition_deg;
diagnostics.VisibilityWorkBudget = graphRecord.VisibilityWorkBudget;
diagnostics.EstimatedExhaustiveVisibilityWork = graphRecord.EstimatedExhaustiveVisibilityWork;
diagnostics.ExhaustiveVisibilityUsed = graphRecord.ExhaustiveVisibilityUsed;
diagnostics.ExhaustiveVisibilityFallbackUsed = graphRecord.ExhaustiveVisibilityFallbackUsed;
diagnostics.VisibilityCandidatePairCount = graphRecord.VisibilityCandidatePairCount;
diagnostics.VisibilityEdgeCount = graphRecord.VisibilityEdgeCount;
diagnostics.AcceptedEdges_deg = graphRecord.AcceptedEdges_deg;
diagnostics.RejectedEdges_deg = graphRecord.RejectedEdges_deg;
diagnostics.RejectedTransitionCount = graphRecord.RejectedTransitionCount;
diagnostics.SampledShapeCount = sampledShapeCount;
diagnostics.DenseSeedEnvelopeUsed = usedDenseEnvelope;
coverage.ExactSpatialProposalUsed = ~spatialSeedTemplate.UsesReducedGeometry;
coverage.ReducedSpatialProposalUsed = spatialSeedTemplate.UsesReducedGeometry;
coverage.CompletenessLossReason = "boundedSeedNodeAndTimeSearch";
if spatialSeedTemplate.UsesReducedGeometry
    coverage.CompletenessLossReason = "reducedSpatialProposalAndBoundedSearch";
end
if usedDenseEnvelope
    diagnostics.DenseSeedEnvelope_deg = sweptShape.Vertices;
end

%% Section 3: Search One Extended Timed Proposal

% A time-layer graph can represent waits and moving-obstacle passages that a
% static swept envelope would hide. Work is explicitly bounded by node/layer
% counts, so diagnostics must preserve when this proposal was suppressed.
reservedTimedSeedCount = 0;
timedRoutes_deg = cell(0, 1);
timedRouteTimes_s = cell(0, 1);
% Add timed search only when geometry changes and the retained seed budget still has room.
if hasChangingObstacles && ~usedDenseEnvelope && numel(seeds) < options.MaximumSeedCount
    coverage.ExtendedTimedSearchAttempted = true;
    maximumTimedNodeCount = 24;
    directNode_deg = start_deg + ((1:7).' / 8) .* (goal_deg - start_deg);
    candidateNodeSets_deg = {sampledNodes_deg};
    if numel(obstacles) > 1
        obstacleDistance_deg = Inf(numel(obstacles), 1);
        obstacleNodeSets_deg = cell(numel(obstacles), 1);
        for obstacleIndex = 1:numel(obstacles)
            [~, ~, obstacleNodeSets_deg{obstacleIndex}] = sweptObstacleShape( ...
                obstacles(obstacleIndex), sampleTimes_s);
            obstacleDistance_deg(obstacleIndex) = minimumRouteDistance( ...
                obstacleNodeSets_deg{obstacleIndex}, start_deg, goal_deg);
        end
        [~, nearestObstacleIndex] = min(obstacleDistance_deg);
        candidateNodeSets_deg = [ ...
            obstacleNodeSets_deg(nearestObstacleIndex); candidateNodeSets_deg];
    end
    trialSeeds = seeds;
    for candidateSetIndex = 1:numel(candidateNodeSets_deg)
        timedPosition_deg = [start_deg; goal_deg; directNode_deg; ...
            selectCandidateVertices(candidateNodeSets_deg{candidateSetIndex}, ...
            start_deg, goal_deg, maximumTimedNodeCount - 2 - size(directNode_deg, 1))];
        timedEdgeCost_deg = hypot( ...
            timedPosition_deg(:, 1) - timedPosition_deg(:, 1).', ...
            timedPosition_deg(:, 2) - timedPosition_deg(:, 2).');
        [timedRoute_deg, timedRouteTime_s, timeRecord] = ...
            azElInternal.timeExpandedVisibilitySearch( ...
            timedPosition_deg, timedEdgeCost_deg, obstacles, initialState, ...
            goalState, limits, sampleTimes_s, options);
        diagnostics = appendTimeRecord(diagnostics, timeRecord);
        nextTrialSeeds = appendTimedSeed( ...
            trialSeeds, timedSeedTemplate, timedRoute_deg, timedRouteTime_s);
        if numel(nextTrialSeeds) > numel(trialSeeds)
            timedRoutes_deg{end + 1, 1} = timedRoute_deg; %#ok<AGROW>
            timedRouteTimes_s{end + 1, 1} = timedRouteTime_s; %#ok<AGROW>
            trialSeeds = nextTrialSeeds;
        end
    end
    reservedTimedSeedCount = numel(trialSeeds) - numel(seeds);
    if graphRecord.CandidateOffsetRetryCount >= 3 && options.MaximumSeedCount >= 3 && ...
            numel(seeds) + reservedTimedSeedCount >= options.MaximumSeedCount
        reservedTimedSeedCount = 0;
    end
end

%% Section 4: Search Distinct Spatial Visibility Routes

% Homology signatures keep topologically different detours instead of merely
% returning several small perturbations of the same route.
maximumClassCount = max(0, options.MaximumSeedCount - reservedTimedSeedCount - numel(seeds));
[nodePaths, classSignatures, searchRecord] = homologyVisibilityPaths( ...
    edgeCost_deg, nodePosition_deg, representative_deg, maximumClassCount);
diagnostics = appendSearchRecord(diagnostics, searchRecord);
diagnostics.HomologySearchAttempted = maximumClassCount > 0;
diagnostics.HomologyClassSignatures = classSignatures;
diagnostics.HomologyClassCount = size(classSignatures, 1);
diagnostics.HomologyStateCount = searchRecord.StateCount;
diagnostics.HomologySearchTruncated = searchRecord.Truncated;

% Convert each discovered homology-class node path into one nonduplicate spatial seed.
for classIndex = 1:numel(nodePaths)
    route_deg = nodePosition_deg(nodePaths{classIndex}, :);
    % Keep the visibility route only when it adds spatial diversity beyond existing seeds.
    if ~routeDuplicates(route_deg, seeds, graphRecord.CandidateOffset_deg)
        seed = createSpatialSeed(spatialSeedTemplate, numel(seeds) + 1, ...
            route_deg, directDuration_s, availableDuration_s, limits);
        seeds(end + 1, 1) = seed; %#ok<AGROW>
    end
end
for timedSeedIndex = 1:numel(timedRoutes_deg)
    if numel(seeds) >= options.MaximumSeedCount
        break;
    end
    seeds = appendTimedSeed(seeds, timedSeedTemplate, ...
        timedRoutes_deg{timedSeedIndex}, timedRouteTimes_s{timedSeedIndex});
end
diagnostics.Coverage = coverage;
diagnostics.GeneratedSeedCount = numel(seeds);
end


function [sweptShape, sampledShapeCount, sampledNodes_deg] = sweptObstacleShape(obstacles, sampleTimes_s)
% Union protected source and midpoint geometry for seed construction.
maximumShapeCount = numel(sampleTimes_s) * numel(obstacles);
sampledShapes = cell(maximumShapeCount, 1);
sampledShapeCount = 0;
sampledNodes_deg = zeros(0, 2);

% Evaluate protected geometry at every selected seed-search time.
for sampleTimeIndex = 1:numel(sampleTimes_s)
    sampleTime_s = sampleTimes_s(sampleTimeIndex);

    % Add the active slice from each obstacle to this time layer's union input.
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), sampleTime_s);
        % Empty obstacle slices contribute neither geometry nor topology events.
        if isempty(shape.Vertices)
            continue;
        end
        sampledShapeCount = sampledShapeCount + 1;
        sampledShapes{sampledShapeCount} = shape;
        sampledNodes_deg = [sampledNodes_deg; shape.Vertices]; %#ok<AGROW>
    end
end
sampledNodes_deg = sampledNodes_deg(all(isfinite(sampledNodes_deg), 2), :);
% No sampled geometry means the spatial graph can operate on an empty envelope.
if sampledShapeCount == 0
    sweptShape = polyshape();
else
    % One balanced Boolean operation avoids repeated growth of the union.
    sweptShape = union([sampledShapes{1:sampledShapeCount}]);
end
end

function [nodePosition_deg, edgeCost_deg, record] = buildVisibilityGraph( ...
        sweptShape, start_deg, goal_deg, limits, forceExhaustiveVisibility, ...
        candidateOffsetMultiplier, candidateOffsetRetryCount)
% Build one bounded sparse segment-visibility graph around swept geometry.
allPosition_deg = [start_deg; goal_deg; sweptShape.Vertices];
minimum_deg = min(allPosition_deg, [], 1);
maximum_deg = max(allPosition_deg, [], 1);
coordinateScale_deg = max(1, max(abs(allPosition_deg), [], "all"));
% Offset seed vertices only; collision geometry and its margin are unchanged.
candidateOffset_deg = candidateOffsetMultiplier * max(1e-3, 256 * eps(coordinateScale_deg));
[edgeStart_deg, edgeEnd_deg] = azElInternal.geometry.boundaryToEdges(sweptShape, 1e-12);
visibilityWorkBudget = 1e6;
candidateLimit = floor(sqrt(2 * visibilityWorkBudget / max(1, size(edgeStart_deg, 1)))) - 2;
candidateLimit = min(96, max(24, candidateLimit));
candidateShape = sweptShape;
if ~isempty(sweptShape.Vertices)
    candidateShape = polybuffer(sweptShape, candidateOffset_deg, "JointType", "miter");
end
candidatePosition_deg = candidateShape.Vertices;
insideWorkspace = candidatePosition_deg(:, 1) >= limits.azimuthInterval_deg(1) & ...
    candidatePosition_deg(:, 1) <= limits.azimuthInterval_deg(2) & ...
    candidatePosition_deg(:, 2) >= limits.elevationInterval_deg(1) & ...
    candidatePosition_deg(:, 2) <= limits.elevationInterval_deg(2);
candidatePosition_deg = candidatePosition_deg(insideWorkspace, :);
candidatePosition_deg = selectCandidateVertices( candidatePosition_deg, start_deg, goal_deg, candidateLimit);
workspaceCorner_deg = zeros(0, 2);
% After repeated clearance retries, workspace corners provide bounded global detour support.
if candidateOffsetRetryCount >= 3
    workspaceCorner_deg = [limits.azimuthInterval_deg([1 1 2 2]).', limits.elevationInterval_deg([1 2 1 2]).'];
end
nodePosition_deg = unique([start_deg; goal_deg; workspaceCorner_deg; candidatePosition_deg], "rows", "stable");
nodeCount = size(nodePosition_deg, 1);
edgeCost_deg = Inf(nodeCount);
edgeCost_deg(1:nodeCount + 1:end) = 0;
candidatePairMask = triu(true(nodeCount), 1);
maximumPairCount = nodeCount * (nodeCount - 1) / 2;
estimatedExhaustiveWork = maximumPairCount * max(1, size(edgeStart_deg, 1));
exhaustiveVisibilityAffordable = estimatedExhaustiveWork <= visibilityWorkBudget;
exhaustiveVisibilityUsed = nodeCount < 4 || forceExhaustiveVisibility;
% Delaunay adjacency proposes a sparse graph before the bounded exhaustive fallback is considered.
if nodeCount >= 4 && ~forceExhaustiveVisibility
    triangulation = delaunayTriangulation(nodePosition_deg);
    candidatePairs = sort(edges(triangulation), 2);
    candidatePairMask = false(nodeCount);
    candidatePairMask(sub2ind([nodeCount nodeCount], candidatePairs(:, 1), candidatePairs(:, 2))) = true;
    candidatePairMask(1:2, 3:end) = true;
    candidatePairMask(1, 2) = true;
end
visibilityEdgeCount = 0;
rejectedTransitionCount = 0;
maximumRetainedEdgeCount = 2000;
acceptedEdges_deg = zeros(maximumRetainedEdgeCount, 4);
rejectedEdges_deg = zeros(maximumRetainedEdgeCount, 4);

% Visit each unordered node pair exactly once.
for firstNodeIndex = 1:nodeCount - 1

    % Test all later nodes selected by the sparse visibility candidate mask.
    for secondNodeIndex = firstNodeIndex + 1:nodeCount
        if ~candidatePairMask(firstNodeIndex, secondNodeIndex)
            continue;
        end
        firstPosition_deg = nodePosition_deg(firstNodeIndex, :);
        secondPosition_deg = nodePosition_deg(secondNodeIndex, :);
        if segmentIsVisible( firstPosition_deg, secondPosition_deg, sweptShape, edgeStart_deg, edgeEnd_deg)
            distance_deg = norm(secondPosition_deg - firstPosition_deg);
            edgeCost_deg(firstNodeIndex, secondNodeIndex) = distance_deg;
            edgeCost_deg(secondNodeIndex, firstNodeIndex) = distance_deg;
            visibilityEdgeCount = visibilityEdgeCount + 1;
            if visibilityEdgeCount <= maximumRetainedEdgeCount
                acceptedEdges_deg(visibilityEdgeCount, :) = [firstPosition_deg, secondPosition_deg];
            end
        else
            rejectedTransitionCount = rejectedTransitionCount + 1;
            if rejectedTransitionCount <= maximumRetainedEdgeCount
                rejectedEdges_deg(rejectedTransitionCount, :) = [firstPosition_deg, secondPosition_deg];
            end
        end
    end
end
componentIndex = conncomp(graph(isfinite(edgeCost_deg), "upper"));
endpointsConnected = componentIndex(1) == componentIndex(2);
% Retry all node pairs only when the work estimate is affordable and the sparse graph disconnected the endpoints.
if ~forceExhaustiveVisibility && exhaustiveVisibilityAffordable && ~endpointsConnected
    [nodePosition_deg, edgeCost_deg, record] = buildVisibilityGraph( ...
        sweptShape, start_deg, goal_deg, limits, true, candidateOffsetMultiplier, candidateOffsetRetryCount);
    record.ExhaustiveVisibilityFallbackUsed = true;
    return;
end
maximumCandidateOffsetMultiplier = 64;
% Finish a recovered topology's ladder; validation uses original geometry.
needsClearanceRetry = ~endpointsConnected || candidateOffsetRetryCount > 0;
% Increase the boundary offset when the graph exists but its route has insufficient protected clearance.
if needsClearanceRetry && candidateOffsetMultiplier < maximumCandidateOffsetMultiplier
    [nodePosition_deg, edgeCost_deg, record] = buildVisibilityGraph( ...
        sweptShape, start_deg, goal_deg, limits, true, 4 * candidateOffsetMultiplier, candidateOffsetRetryCount + 1);
    return;
end
record = struct( ...
    "Bounds_deg", [minimum_deg(1), maximum_deg(1), minimum_deg(2), maximum_deg(2)], ...
    "CandidateOffset_deg", candidateOffset_deg, ...
    "CandidateOffsetRetryCount", candidateOffsetRetryCount, ...
    "VisibilityWorkBudget", visibilityWorkBudget, ...
    "EstimatedExhaustiveVisibilityWork", estimatedExhaustiveWork, ...
    "ExhaustiveVisibilityUsed", exhaustiveVisibilityUsed, ...
    "ExhaustiveVisibilityFallbackUsed", false, ...
    "VisibilityCandidatePairCount", nnz(candidatePairMask), ...
    "VisibilityEdgeCount", visibilityEdgeCount, ...
    "AcceptedEdges_deg", acceptedEdges_deg( ...
    1:min(visibilityEdgeCount, maximumRetainedEdgeCount), :), ...
    "RejectedEdges_deg", rejectedEdges_deg( ...
    1:min(rejectedTransitionCount, maximumRetainedEdgeCount), :), "RejectedTransitionCount", rejectedTransitionCount);
end

function selectedPosition_deg = selectCandidateVertices(candidatePosition_deg, start_deg, goal_deg, maximumCount)
% Bound dense polygon input while retaining global and endpoint supports.
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

% Retain one extreme obstacle vertex in each evenly spaced support direction.
for directionIndex = 1:size(direction, 1)
    [~, supportIndex] = max(candidatePosition_deg * direction(directionIndex, :).');
    selectedIndex(end + 1, 1) = supportIndex; %#ok<AGROW>
end

% Also retain nearby vertices around both endpoints so graph access is not undersampled.
for reference_deg = [start_deg; goal_deg].'
    distance_deg = vecnorm(candidatePosition_deg - reference_deg.', 2, 2);
    [~, order] = sort(distance_deg, "ascend");
    selectedIndex = [selectedIndex; order(1:endpointNeighborCount)]; %#ok<AGROW>
end
selectedIndex = unique(selectedIndex, "stable");
selectedIndex = selectedIndex(1:min(maximumCount, numel(selectedIndex)));
selectedPosition_deg = candidatePosition_deg(selectedIndex, :);
end

function distance_deg = minimumRouteDistance(position_deg, start_deg, goal_deg)
% Rank obstacle histories by distance to the direct planning segment.
if isempty(position_deg)
    distance_deg = Inf;
    return;
end
routeVector_deg = goal_deg - start_deg;
routeFraction = ((position_deg - start_deg) * routeVector_deg.') / ...
    max(sum(routeVector_deg.^2), eps);
routeProjection_deg = start_deg + min(1, max(0, routeFraction)) .* ...
    routeVector_deg;
distance_deg = min(vecnorm(position_deg - routeProjection_deg, 2, 2));
end

function visible = segmentIsVisible(firstPosition_deg, secondPosition_deg, shape, edgeStart_deg, edgeEnd_deg)
% Reject a segment that enters, crosses, or touches protected geometry.
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
denominator_deg2 = segment_deg(1) * boundarySegment_deg(:, 2) - segment_deg(2) * boundarySegment_deg(:, 1);
coordinateScale_deg = max(1, max(abs([ firstPosition_deg, secondPosition_deg, edgeStart_deg(:).', edgeEnd_deg(:).'])));
intersectionTolerance_deg2 = 512 * eps(coordinateScale_deg^2);
nonparallel = abs(denominator_deg2) > intersectionTolerance_deg2;
firstFraction = Inf(size(denominator_deg2));
secondFraction = Inf(size(denominator_deg2));
firstFraction(nonparallel) = ( ...
    offset_deg(nonparallel, 1) .* ...
    boundarySegment_deg(nonparallel, 2) - ...
    offset_deg(nonparallel, 2) .* boundarySegment_deg(nonparallel, 1)) ./ denominator_deg2(nonparallel);
secondFraction(nonparallel) = ( ...
    offset_deg(nonparallel, 1) * segment_deg(2) - ...
    offset_deg(nonparallel, 2) * segment_deg(1)) ./ denominator_deg2(nonparallel);
crossesBoundary = nonparallel & ...
    firstFraction >= -1e-12 & firstFraction <= 1 + 1e-12 & secondFraction >= -1e-12 & secondFraction <= 1 + 1e-12;
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
    overlaps = max(min(projection, nextProjection), 0) <= min(max(projection, nextProjection), 1) + 1e-12;
    visible = ~any(overlaps);
end
end

function source = timedRouteSource(route_deg)
% Name a pure direct waiting path separately from a timed detour.
positionChanges = [true; vecnorm(diff(route_deg, 1, 1), 2, 2) > 1e-12];
distinctRoute_deg = route_deg(positionChanges, :);
hasWait = any(~positionChanges(2:end));
if hasWait && size(distinctRoute_deg, 1) == 2
    source = "directWait";
else
    source = "timeExpandedVisibilityGraph";
end
end

function duplicate = temporalSeedDuplicates(seed, seeds)
% Compare both geometry and time law so distinct waits remain available.
duplicate = false;
sampleTau = linspace(0, 1, 101).';
sampledRoute_deg = interp1(seed.tau, seed.position_deg, sampleTau, "linear");

% Compare against every retained seed because timing differences can preserve distinct waits.
for seedIndex = 1:numel(seeds)
    sampledSeed_deg = interp1(seeds(seedIndex).tau, seeds(seedIndex).position_deg, sampleTau, "linear");
    priorDuration_s = seeds(seedIndex).EstimatedDuration_s;
    durationTolerance_s = 1e-9 * max([1, abs(seed.EstimatedDuration_s), abs(priorDuration_s)]);
    if abs(seed.EstimatedDuration_s - priorDuration_s) <= durationTolerance_s && ...
            max(vecnorm(sampledRoute_deg - sampledSeed_deg, 2, 2)) <= 1e-6
        duplicate = true;
        return;
    end
end
end

function seeds = appendTimedSeed(seeds, seedTemplate, route_deg, routeTime_s)
% Append one distinct timed visibility route within the public seed cap.
if isempty(route_deg)
    return;
end
timedSeed = seedTemplate;
timedSeed.Index = numel(seeds) + 1;
timedSeed.Source = timedRouteSource(route_deg);
timedSeed.position_deg = route_deg;
timedSeed.tau = (routeTime_s - routeTime_s(1)) / (routeTime_s(end) - routeTime_s(1));
timedSeed.EstimatedDuration_s = routeTime_s(end) - routeTime_s(1);
timedSeed.Length_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
if ~temporalSeedDuplicates(timedSeed, seeds)
    seeds(end + 1, 1) = timedSeed;
end
end

function [nodePaths, classSignatures, record] = homologyVisibilityPaths( ...
        edgeCost_deg, nodePosition_deg, representative_deg, maximumClassCount)
% Search a bounded graph augmented by 2-D homology signatures.
% The path-integral state follows Bhattacharya et al.; see citation.md.
nodeCount = size(edgeCost_deg, 1);
representativeCount = size(representative_deg, 1);
nodePaths = cell(0, 1);
classSignatures = zeros(0, representativeCount, "int8");
record = struct("ExpandedCount", 0, "RejectedTransitionCount", 0, ...
    "ExploredNodes_deg", zeros(0, 2), "FrontierNodes_deg", zeros(0, 2), "StateCount", 0, "Truncated", false);
% A zero homology budget disables this optional diversity search without affecting existing seeds.
if maximumClassCount == 0
    return;
end
phase_rad = atan2(nodePosition_deg(:, 2) - representative_deg(:, 2).', ...
    nodePosition_deg(:, 1) - representative_deg(:, 1).');
referenceTurn = principalAngle(phase_rad - phase_rad(1, :)) / (2 * pi);
maximumStateCount = min(4000, max(200, 8 * nodeCount * maximumClassCount));
stateNodeIndex = zeros(maximumStateCount, 1, "uint16");
stateSignature = zeros(maximumStateCount, representativeCount, "int8");
stateCost_deg = Inf(maximumStateCount, 1);
parentStateIndex = zeros(maximumStateCount, 1, "uint16");
closed = false(maximumStateCount, 1);
stateNodeIndex(1) = 1;
stateCost_deg(1) = 0;
stateCount = 1;
rejectedTransitionCount = 0;

% Expand lowest-cost augmented states until enough homology classes are found or work is exhausted.
while size(classSignatures, 1) < maximumClassCount
    unsettledCost_deg = stateCost_deg;
    unsettledCost_deg(closed) = Inf;
    [currentCost_deg, currentStateIndex] = min(unsettledCost_deg);
    if ~isfinite(currentCost_deg)
        break;
    end
    closed(currentStateIndex) = true;
    currentNodeIndex = double(stateNodeIndex(currentStateIndex));
    if currentNodeIndex == 2
        statePath = currentStateIndex;

        % Reconstruct this completed augmented-state path from goal to start.
        while statePath(1) ~= 1
            statePath = [double(parentStateIndex(statePath(1))); statePath]; %#ok<AGROW>
        end
        nodePaths{end + 1, 1} = double(stateNodeIndex(statePath)); %#ok<AGROW>
        classSignatures(end + 1, :) = stateSignature(currentStateIndex, :); %#ok<AGROW>
        continue;
    end
    neighborNodeIndices = find(isfinite(edgeCost_deg(currentNodeIndex, :)));

    % Extend the current signature across every visible neighboring graph edge.
    for neighborNodeIndex = reshape(neighborNodeIndices, 1, [])
        if neighborNodeIndex == currentNodeIndex
            rejectedTransitionCount = rejectedTransitionCount + 1;
            continue;
        end
        phaseStep = principalAngle(phase_rad(neighborNodeIndex, :) - phase_rad(currentNodeIndex, :)) / (2 * pi);
        signatureStep = round(referenceTurn(currentNodeIndex, :) + phaseStep - referenceTurn(neighborNodeIndex, :));
        trialSignature = int8(double(stateSignature(currentStateIndex, :)) + signatureStep);
        if any(abs(double(trialSignature)) > 1)
            rejectedTransitionCount = rejectedTransitionCount + 1;
            continue;
        end
        sameState = stateNodeIndex(1:stateCount) == neighborNodeIndex & ...
            all(stateSignature(1:stateCount, :) == trialSignature, 2);
        neighborStateIndex = find(sameState, 1);
        if isempty(neighborStateIndex)
            % Stop creating signature states at the documented work cap and expose truncation in diagnostics.
            if stateCount >= maximumStateCount
                record.Truncated = true;
                rejectedTransitionCount = rejectedTransitionCount + 1;
                continue;
            end
            stateCount = stateCount + 1;
            neighborStateIndex = stateCount;
            stateNodeIndex(neighborStateIndex) = uint16(neighborNodeIndex);
            stateSignature(neighborStateIndex, :) = trialSignature;
        elseif closed(neighborStateIndex)
            rejectedTransitionCount = rejectedTransitionCount + 1;
            continue;
        end
        trialCost_deg = currentCost_deg + edgeCost_deg(currentNodeIndex, neighborNodeIndex);
        if trialCost_deg < stateCost_deg(neighborStateIndex) - 1e-12
            stateCost_deg(neighborStateIndex) = trialCost_deg;
            parentStateIndex(neighborStateIndex) = uint16(currentStateIndex);
        end
    end
end
frontierState = isfinite(stateCost_deg(1:stateCount)) & ~closed(1:stateCount);
expandedState = closed(1:stateCount) & stateNodeIndex(1:stateCount) ~= 2;
record.ExpandedCount = sum(expandedState);
record.RejectedTransitionCount = rejectedTransitionCount;
record.ExploredNodes_deg = nodePosition_deg(double(stateNodeIndex(expandedState)), :);
record.FrontierNodes_deg = nodePosition_deg(double(stateNodeIndex(frontierState)), :);
record.StateCount = stateCount;
end

function diagnostics = appendSearchRecord(diagnostics, record)
% Add complete counts and a bounded deterministic diagnostic trace.
diagnostics.ExpandedCount = diagnostics.ExpandedCount + record.ExpandedCount;
diagnostics.RejectedTransitionCount = diagnostics.RejectedTransitionCount + record.RejectedTransitionCount;
diagnostics.ExploredNodes_deg = appendBoundedTrace( diagnostics.ExploredNodes_deg, record.ExploredNodes_deg, 2000);
diagnostics.FrontierNodes_deg = appendBoundedTrace( diagnostics.FrontierNodes_deg, record.FrontierNodes_deg, 2000);
end

function diagnostics = appendTimeRecord(diagnostics, record)
% Add one time-layer search without reconstructing its decisions.
diagnostics.TemporalLayerTimes_s = record.LayerTimes_s;
diagnostics.TemporalLayerCount = numel(record.LayerTimes_s);
diagnostics.TemporalNodeCount = max(diagnostics.TemporalNodeCount, record.NodeCount);
diagnostics.WaitEdgeCount = diagnostics.WaitEdgeCount + record.WaitEdgeCount;
diagnostics.MotionEdgeCount = diagnostics.MotionEdgeCount + record.MotionEdgeCount;
diagnostics = appendSearchRecord(diagnostics, record);
end

function seed = createSpatialSeed( seedTemplate, seedIndex, route_deg, directDuration_s, availableDuration_s, limits)
% Assemble one spatial visibility route with a length-based time law.
seed = seedTemplate;
seed.Index = seedIndex;
seed.Source = "visibilityGraph";
seed.position_deg = route_deg;
[seed.tau, seed.Length_deg] = routeTau(route_deg);
seed.EstimatedDuration_s = min(availableDuration_s, max( ...
    directDuration_s, seed.Length_deg / max(limits.maxVelocity_deg_s)));
end

function changing = obstacleHistoryChanges(obstacles, startTime_s, endTime_s)
% Detect geometry or active-span changes inside the request window.
changing = false;

% Stop as soon as any obstacle shows timing or geometry variation in the request window.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    hasFiniteActiveSpan = numel(obstacle.time_s) > 1;
    if hasFiniteActiveSpan && (obstacle.time_s(1) > startTime_s || obstacle.time_s(end) < endTime_s)
        changing = true;
        return;
    end

    % Compare adjacent stored slices to detect motion or deformation.
    for sampleIndex = 2:numel(obstacle.time_s)
        sameAzimuth = isequaln( obstacle.az_deg{sampleIndex}, obstacle.az_deg{1});
        sameElevation = isequaln( obstacle.el_deg{sampleIndex}, obstacle.el_deg{1});
        if ~sameAzimuth || ~sameElevation
            changing = true;
            return;
        end
    end
end
end

function sampleTimes_s = obstacleSampleTimes(obstacles, startTime_s, endTime_s)
% Use source times and interval midpoints without event detection.
sampleTimes_s = [startTime_s; endTime_s];
verticesPerLayer = 0;

% Collect every obstacle's source times, midpoint times, and worst per-layer vertex count.
for obstacleIndex = 1:numel(obstacles)
    time_s = obstacles(obstacleIndex).time_s(:);
    midTime_s = (time_s(1:end - 1) + time_s(2:end)) / 2;
    sampleTimes_s = [sampleTimes_s; time_s; midTime_s]; %#ok<AGROW>
    maximumVertexCount = 0;

    % Measure the densest slice for this obstacle when estimating timed-query work.
    for sampleIndex = 1:numel(obstacles(obstacleIndex).az_deg)
        maximumVertexCount = max(maximumVertexCount, numel(obstacles(obstacleIndex).az_deg{sampleIndex}));
    end
    verticesPerLayer = verticesPerLayer + maximumVertexCount;
end
sampleTimes_s = unique(sampleTimes_s( sampleTimes_s >= startTime_s & sampleTimes_s <= endTime_s));
unionVertexBudget = 50e3;
maximumLayerCount = min(33, max(9, floor(unionVertexBudget / max(1, verticesPerLayer))));
sampleTimes_s = azElInternal.boundedTimeLayers( ...
    sampleTimes_s, startTime_s, endTime_s, maximumLayerCount);
end

function representative_deg = homologyRepresentatives(shape)
% Select one guaranteed interior point for each connected shape region.
shapeRegions = regions(shape);
representative_deg = zeros(numel(shapeRegions), 2);

% Assign one homology reference point to every disconnected occupied region.
for regionIndex = 1:numel(shapeRegions)
    [candidate_deg, radius_deg] = incenter( triangulation(shapeRegions(regionIndex)));
    [~, largestRadiusIndex] = max(radius_deg);
    representative_deg(regionIndex, :) = candidate_deg(largestRadiusIndex, :);
end
end

function angle_rad = principalAngle(angle_rad)
% Map angular change to the deterministic principal interval.
angle_rad = atan2(sin(angle_rad), cos(angle_rad));
end

function duplicate = routeDuplicates(route_deg, seeds, tolerance_deg)
% Compare sampled route geometry so equivalent spatial seeds are removed.
duplicate = false;
sampleTau = linspace(0, 1, 101).';
[parameterizedTau, ~] = routeTau(route_deg);
sampledRoute_deg = interp1( parameterizedTau, route_deg, sampleTau, "linear");

% Compare the proposed geometry with every already retained route at common arc-length samples.
for seedIndex = 1:numel(seeds)
    sampledSeed_deg = interp1( seeds(seedIndex).tau, seeds(seedIndex).position_deg, sampleTau, "linear");
    if max(vecnorm(sampledRoute_deg - sampledSeed_deg, 2, 2)) <= max(1e-6, 0.5 * tolerance_deg)
        duplicate = true;
        return;
    end
end
end

function [tau, length_deg] = routeTau(route_deg)
% Parameterize a route by normalized cumulative Euclidean length.
cumulativeLength_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
length_deg = cumulativeLength_deg(end);
if length_deg <= 0
    tau = linspace(0, 1, size(route_deg, 1)).';
else
    tau = cumulativeLength_deg / length_deg;
end
end

function values = appendBoundedTrace(values, additions, maximumCount)
% Append diagnostics without exceeding the documented retained-trace cap.
remainingCount = maximumCount - size(values, 1);
if remainingCount > 0
    values = [values; additions(1:min(remainingCount, size(additions, 1)), :)];
end
end

function diagnostics = emptyDiagnostics(start_deg, goal_deg)
% Initialize the stable search-diagnostics schema before any early exit.
diagnostics = struct( ...
    "GraphType", "timeExpandedVisibilityGraph", ...
    "Bounds_deg", [NaN NaN NaN NaN], "Resolution_deg", NaN, ...
    "CandidateOffset_deg", NaN, "SampleTimes_s", zeros(0, 1), ...
    "CandidateOffsetRetryCount", 0, ...
    "SampledShapeCount", 0, "DenseSeedEnvelopeUsed", false, ...
    "DenseSeedEnvelope_deg", zeros(0, 2), "Coverage", struct( ...
    "ExactSpatialProposalUsed", false, "ReducedSpatialProposalUsed", false, ...
    "TimedSearchAttempted", false, "ExtendedTimedSearchAttempted", false, ...
    "TimedSearchUsesExactObstacles", false, "TimedSearchSuppressionReason", "graphNotBuilt", ...
    "CompletenessLost", true, "CompletenessLossReason", "directSeedOnly"), ...
    "SeedCluster", struct("Distance_deg", 0, "SourceRegionCount", 0, ...
    "ClusterGroupCount", 0, "ClusteredRegionCount", 0, "ClusterBoundary_deg", zeros(0, 2)), ...
    "NodeCount", 0, "NodePosition_deg", zeros(0, 2), ...
    "VisibilityWorkBudget", 1e6, ...
    "EstimatedExhaustiveVisibilityWork", 0, ...
    "ExhaustiveVisibilityUsed", false, ...
    "ExhaustiveVisibilityFallbackUsed", false, ...
    "VisibilityCandidatePairCount", 0, ...
    "VisibilityEdgeCount", 0, "AcceptedEdges_deg", zeros(0, 4), ...
    "RejectedEdges_deg", zeros(0, 4), ...
    "TemporalLayerTimes_s", zeros(0, 1), "TemporalLayerCount", 0, ...
    "TemporalNodeCount", 0, "WaitEdgeCount", 0, "MotionEdgeCount", 0, ...
    "HomologySearchAttempted", false, "HomologyRepresentative_deg", zeros(0, 2), ...
    "HomologyClassSignatures", zeros(0, 0, "int8"), ...
    "HomologyClassCount", 0, "HomologyStateCount", 0, "HomologySearchTruncated", false, ...
    "ExpandedCount", 0, "RejectedTransitionCount", 0, ...
    "GeneratedSeedCount", 1, "ExploredNodes_deg", zeros(0, 2), ...
    "FrontierNodes_deg", zeros(0, 2), "Start_deg", start_deg, ...
    "Goal_deg", goal_deg, "TraceDownsampleRule", ...
    "Time samples cap at 33, temporal nodes at 24, homology states at 4000, traces at 2000 each");
end
