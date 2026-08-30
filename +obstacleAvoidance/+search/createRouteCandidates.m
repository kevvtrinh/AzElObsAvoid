function [seeds, diagnostics] = createRouteCandidates( ...
        obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics] = ...
%       obstacleAvoidance.search.createRouteCandidates( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Create deterministic direct, timed, and distinct spatial topology
%     proposals with first-class bounded-search diagnostics.
%**************************************************************************
% INPUTS
%   - obstacles (prepared canonical obstacle struct array)
%       Static or time-varying protected geometry.
%   - initialState, goalState (scalar structs)
%       Endpoint states and request interval.
%   - limits (scalar struct)
%       Workspace and independent-axis physical limits.
%   - options (resolved scalar struct)
%       Seed count, clustering, wrapping, and goal-time policy.
%**************************************************************************
% OUTPUTS
%   - seeds (struct array)
%       Deterministically ordered proposals; direct is always first.
%   - diagnostics (scalar struct)
%       Nodes, edges, rejections, explored/frontier states, and best partial.
%**************************************************************************
% UNITS
%   - Position and path length are degrees; time is seconds.
%**************************************************************************

%% Section 1: Create The Required Direct Proposal

start_deg = initialState.position_deg;
goal_deg = obstacleAvoidance.input.goalPositionAtTime( goalState, goalState.time_s);
if options.AllowAzimuthWrapping
    goal_deg(1) = goal_deg(1) + 360 * round( (start_deg(1) - goal_deg(1)) / 360);
end
available_s = goalState.time_s - initialState.time_s;
directRoute_deg = [start_deg; goal_deg];
directLength_deg = norm(goal_deg - start_deg);
directDuration_s = min(available_s, max( routeDuration(directRoute_deg, limits), ...
    directLength_deg / max(limits.maxVelocity_deg_s)));
template = obstacleAvoidance.search.createSeed();
seeds = template;
seeds.Index = 1;
seeds.Source = "directVisibilityEdge";
seeds.position_deg = directRoute_deg;
seeds.tau = [0; 1];
seeds.EstimatedDuration_s = directDuration_s;
seeds.Length_deg = directLength_deg;
diagnostics = emptyDiagnostics(start_deg, goal_deg, options.SeedClusterDistance_deg);
if options.MaximumSeedCount == 1 || isempty(obstacles)
    return;
end

%% Section 2: Create One Protected Visibility Graph

sampleTimes_s = obstacleTimes(obstacles, initialState.time_s, goalState.time_s);
obstacleAvoidance.input.throwIfCancellationRequested(options);
[sweptShape, usedDenseEnvelope] = obstacleAvoidance.search.denseSweptEnvelope( ...
    obstacles, sampleTimes_s, [start_deg; goal_deg], 10e3);
if usedDenseEnvelope
    sampledShapeCount = numel(sampleTimes_s) * numel(obstacles);
else
    [sweptShape, sampledShapeCount] = sampledShape( obstacles, sampleTimes_s);
end
[sweptShape, diagnostics.SeedCluster] = obstacleAvoidance.search.clusterSeedShape( ...
    sweptShape, options.SeedClusterDistance_deg, [start_deg; goal_deg], ...
    "createRouteCandidates:InvalidClusterShape");
[nodePosition_deg, edgeCost_deg, graphRecord] = ...
    createVisibilityGraph(sweptShape, start_deg, goal_deg, limits, 1, 0);
obstacleAvoidance.input.throwIfCancellationRequested(options);
diagnostics.SampleTimes_s = sampleTimes_s;
diagnostics.SampledShapeCount = sampledShapeCount;
diagnostics.DenseSeedEnvelopeUsed = usedDenseEnvelope;
diagnostics.NodeCount = size(nodePosition_deg, 1);
diagnostics.NodePosition_deg = nodePosition_deg;
if usedDenseEnvelope
    diagnostics.DenseSeedEnvelope_deg = sweptShape.Vertices;
end
graphFields = ["Bounds_deg", "CandidateOffset_deg", "CandidateOffsetRetryCount", ...
    "VisibilityWorkBudget", "EstimatedExhaustiveVisibilityWork", ...
    "ExhaustiveVisibilityUsed", "ExhaustiveVisibilityFallbackUsed", ...
    "VisibilityCandidatePairCount", "VisibilityEdgeCount", ...
    "AcceptedEdges_deg", "RejectedEdges_deg", "RejectedTransitionCount"];
for fieldIndex = 1:numel(graphFields)
    fieldName = graphFields(fieldIndex);
    diagnostics.(fieldName) = graphRecord.(fieldName);
end
representatives_deg = createRepresentatives(sweptShape);
diagnostics.HomologyRepresentative_deg = representatives_deg;
usesReducedGeometry = usedDenseEnvelope || diagnostics.SeedCluster.ClusterGroupCount > 0;
diagnostics.Coverage.ExactSpatialProposalUsed = ~usesReducedGeometry;
diagnostics.Coverage.ReducedSpatialProposalUsed = usesReducedGeometry;
diagnostics.Coverage.TimedSearchSuppressionReason = "staticObstacleHistory";

%% Section 3: Search Complete Input-Derived Time Layers

hasChangingHistory = obstacleAvoidance.obstacles.hasChangingHistory( ...
    obstacles, initialState.time_s, goalState.time_s);
if hasChangingHistory && usedDenseEnvelope
    diagnostics.Coverage.TimedSearchSuppressionReason = "timedQueryWorkLimit";
elseif hasChangingHistory
    diagnostics.Coverage.TimedSearchAttempted = true;
    diagnostics.Coverage.ExtendedTimedSearchAttempted = true;
    diagnostics.Coverage.TimedSearchUsesExactObstacles = true;
    diagnostics.Coverage.TimedSearchSuppressionReason = "";
    timedCost_deg = hypot( nodePosition_deg(:, 1) - nodePosition_deg(:, 1).', ...
        nodePosition_deg(:, 2) - nodePosition_deg(:, 2).');
    [route_deg, routeTime_s, timedRecord] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodePosition_deg, timedCost_deg, obstacles, initialState, ...
        goalState, limits, sampleTimes_s, options);
    diagnostics.TemporalLayerTimes_s = timedRecord.LayerTimes_s;
    diagnostics.TemporalLayerCount = numel(timedRecord.LayerTimes_s);
    diagnostics.TemporalNodeCount = timedRecord.NodeCount;
    diagnostics.WaitEdgeCount = timedRecord.WaitEdgeCount;
    diagnostics.MotionEdgeCount = timedRecord.MotionEdgeCount;
    diagnostics = appendSearchDiagnostics(diagnostics, timedRecord);
    seeds = appendTimedSeed(seeds, template, route_deg, routeTime_s);
end

%% Section 4: Select Distinct Spatial Classes

maximumClassCount = max(0, options.MaximumSeedCount - numel(seeds));
[edgeStart_deg, edgeEnd_deg] = ...
    obstacleAvoidance.geometry.boundaryToEdges(sweptShape, 1e-12);
visibilityFunction = @(first_deg, second_deg) segmentsAreVisible( ...
    first_deg, second_deg, sweptShape, edgeStart_deg, edgeEnd_deg);
[routes, signatures, searchRecord] = ...
    obstacleAvoidance.planner.searchSpatialHomologyRoutes( ...
    edgeCost_deg, nodePosition_deg, representatives_deg, ...
    maximumClassCount, visibilityFunction, options);
diagnostics = appendSearchDiagnostics(diagnostics, searchRecord);
diagnostics.HomologySearchAttempted = maximumClassCount > 0;
diagnostics.HomologyClassSignatures = signatures;
diagnostics.HomologyClassCount = size(signatures, 1);
diagnostics.HomologyStateCount = searchRecord.StateCount;
spatialTemplate = template;
spatialTemplate.CorridorBoundary_deg = sweptShape.Vertices;
spatialTemplate.UsesReducedGeometry = usesReducedGeometry;
for routeIndex = 1:numel(routes)
    route_deg = routes{routeIndex};
    seed = createSpatialSeed(spatialTemplate, numel(seeds) + 1, ...
        route_deg, directDuration_s, available_s, limits);
    distinctLengthTolerance_deg = 1e-9 * max(1, directLength_deg);
    if seed.EstimatedDuration_s <= available_s && ...
            seed.Length_deg > directLength_deg + distinctLengthTolerance_deg
        seeds(end + 1, 1) = seed; %#ok<AGROW>
    end
end
cleanupFields = ["RouteCleanupAttemptedCount", "RouteCleanupCandidateCount", ...
    "RouteCleanupVisibilityRejectedCount", "RouteCleanupHomologyRejectedCount", ...
    "RouteCleanupAcceptedCount", "RouteCleanupLengthReduction_deg"];
for fieldIndex = 1:numel(cleanupFields)
    fieldName = cleanupFields(fieldIndex);
    diagnostics.(fieldName) = searchRecord.(fieldName);
end
diagnostics.GeneratedSeedCount = numel(seeds);
if usesReducedGeometry
    diagnostics.Coverage.CompletenessLossReason = ...
        "reducedSpatialProposalAndBoundedSearch";
else
    diagnostics.Coverage.CompletenessLossReason = ...
        "boundedSeedNodeAndTimeSearch";
end
end

%% Section 5: Local Functions

function [shape, count] = sampledShape(obstacles, times_s)
% Union all exact proposal slices without changing authoritative obstacles.
parts = cell(numel(times_s) * numel(obstacles), 1);
count = 0;
for timeIndex = 1:numel(times_s)
    for obstacleIndex = 1:numel(obstacles)
        part = obstacleAvoidance.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), times_s(timeIndex));
        if ~isempty(part.Vertices)
            count = count + 1;
            parts{count} = part;
        end
    end
end
shape = polyshape();
if count > 0
    shape = union([parts{1:count}]);
end
end

function [positions_deg, cost_deg, record] = createVisibilityGraph( ...
        shape, start_deg, goal_deg, limits, offsetMultiplier, offsetRetryCount)
% Create a work-derived sparse graph and affordable exhaustive fallback.
[edgeStart_deg, edgeEnd_deg] = obstacleAvoidance.geometry.boundaryToEdges(shape, 1e-12);
workBudget = 1e6;
candidateLimit = max(2, floor(sqrt(2 * workBudget / max(1, size(edgeStart_deg, 1)))) - 2);
all_deg = [start_deg; goal_deg; shape.Vertices];
scale_deg = bmtpEngine.createCoordinateTolerances(all_deg);
baseOffset_deg = max(1e-3, 256 * eps(scale_deg));
offset_deg = offsetMultiplier * baseOffset_deg;
candidateShape = shape;
if ~isempty(shape.Vertices)
    candidateShape = polybuffer(shape, offset_deg, "JointType", "miter");
end
candidates_deg = candidateShape.Vertices;
inside = candidates_deg(:, 1) >= limits.azimuthInterval_deg(1) & ...
    candidates_deg(:, 1) <= limits.azimuthInterval_deg(2) & ...
    candidates_deg(:, 2) >= limits.elevationInterval_deg(1) & ...
    candidates_deg(:, 2) <= limits.elevationInterval_deg(2);
candidates_deg = unique(candidates_deg(inside, :), "rows", "stable");
if size(candidates_deg, 1) > candidateLimit
    candidates_deg = selectVisibilityCandidates( ...
        candidates_deg, start_deg, goal_deg, candidateLimit);
end
positions_deg = unique([start_deg; goal_deg; candidates_deg], "rows", "stable");
nodeCount = size(positions_deg, 1);
pairMask = triu(true(nodeCount), 1);
usedExhaustive = nodeCount < 4;
if nodeCount >= 4
    triangulation = delaunayTriangulation(positions_deg);
    pairs = sort(edges(triangulation), 2);
    pairMask = false(nodeCount);
    pairMask(sub2ind([nodeCount nodeCount], pairs(:, 1), pairs(:, 2))) = true;
    pairMask(1:2, 3:end) = true;
    pairMask(1, 2) = true;
end
[cost_deg, accepted_deg, rejected_deg, acceptedCount, rejectedCount] = ...
    createEdges(positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg);
component = conncomp(graph(isfinite(cost_deg), "upper"));
if component(1) ~= component(2) && nodeCount >= 4
    [boundaryStart_deg, boundaryEnd_deg] = ...
        obstacleAvoidance.geometry.boundaryToEdges(candidateShape, 1e-12);
    [startFound, startIndex] = ismember(boundaryStart_deg, positions_deg, "rows");
    [endFound, endIndex] = ismember(boundaryEnd_deg, positions_deg, "rows");
    boundaryPairs = sort([startIndex(startFound & endFound), ...
        endIndex(startFound & endFound)], 2);
    boundaryPairs = boundaryPairs(boundaryPairs(:, 1) ~= boundaryPairs(:, 2), :);
    augmentedPairMask = pairMask;
    augmentedPairMask(sub2ind([nodeCount nodeCount], ...
        boundaryPairs(:, 1), boundaryPairs(:, 2))) = true;
    if nnz(augmentedPairMask) > nnz(pairMask)
        pairMask = augmentedPairMask;
        [cost_deg, accepted_deg, rejected_deg, acceptedCount, rejectedCount] = ...
            createEdges(positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg);
        component = conncomp(graph(isfinite(cost_deg), "upper"));
    end
end
exhaustiveWork = nodeCount * (nodeCount - 1) / 2 * ...
    max(1, size(edgeStart_deg, 1));
usedFallback = component(1) ~= component(2) && ...
    ~usedExhaustive && exhaustiveWork <= workBudget;
if usedFallback
    pairMask = triu(true(nodeCount), 1);
    [cost_deg, accepted_deg, rejected_deg, acceptedCount, rejectedCount] = ...
        createEdges(positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg);
    usedExhaustive = true;
    component = conncomp(graph(isfinite(cost_deg), "upper"));
end
maximumOffset_deg = max([diff(limits.azimuthInterval_deg), ...
    diff(limits.elevationInterval_deg)]);
if component(1) ~= component(2) && offset_deg < maximumOffset_deg
    nextOffset_deg = min(4 * offset_deg, maximumOffset_deg);
    [positions_deg, cost_deg, record] = createVisibilityGraph( ...
        shape, start_deg, goal_deg, limits, nextOffset_deg / baseOffset_deg, ...
        offsetRetryCount + 1);
    record.ExhaustiveVisibilityUsed = ...
        record.ExhaustiveVisibilityUsed || usedExhaustive;
    record.ExhaustiveVisibilityFallbackUsed = ...
        record.ExhaustiveVisibilityFallbackUsed || usedFallback;
    return;
end
minimum_deg = min(all_deg, [], 1);
maximum_deg = max(all_deg, [], 1);
record = struct("Bounds_deg", [minimum_deg(1), maximum_deg(1), ...
    minimum_deg(2), maximum_deg(2)], "CandidateOffset_deg", offset_deg, ...
    "CandidateOffsetRetryCount", offsetRetryCount, ...
    "VisibilityWorkBudget", workBudget, "EstimatedExhaustiveVisibilityWork", exhaustiveWork, ...
    "ExhaustiveVisibilityUsed", usedExhaustive, ...
    "ExhaustiveVisibilityFallbackUsed", usedFallback, ...
    "VisibilityCandidatePairCount", nnz(pairMask), "VisibilityEdgeCount", acceptedCount, ...
    "AcceptedEdges_deg", accepted_deg, "RejectedEdges_deg", rejected_deg, ...
    "RejectedTransitionCount", rejectedCount);
end

function selected_deg = selectVisibilityCandidates(candidates_deg, start_deg, goal_deg, count)
% Retain global supports, endpoint access, and uniform boundary coverage.
endpointCount = min(4, floor(count / 6));
directionCount = min(16, floor(count / 3));
uniformCount = count - directionCount - 2 * endpointCount;
selected = unique(round(linspace(1, size(candidates_deg, 1), uniformCount))).';
if directionCount > 0
    angle_rad = (0:directionCount - 1).' * (2 * pi / directionCount);
    direction = [cos(angle_rad), sin(angle_rad)];
    [~, support] = max(candidates_deg * direction.', [], 1);
    selected = [selected; support(:)];
end
for reference_deg = [start_deg; goal_deg].'
    [~, order] = sort(vecnorm(candidates_deg - reference_deg.', 2, 2));
    selected = [selected; order(1:endpointCount)]; %#ok<AGROW>
end
selected = unique(selected, "stable");
selected_deg = candidates_deg(selected(1:min(count, numel(selected))), :);
end

function [cost_deg, accepted_deg, rejected_deg, acceptedCount, rejectedCount] = ...
        createEdges(positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg)
% Evaluate proposed pairs and retain the first 2000 decisions by category.
[firstNode, secondNode] = find(pairMask);
first_deg = positions_deg(firstNode, :);
second_deg = positions_deg(secondNode, :);
visible = segmentsAreVisible(first_deg, second_deg, shape, edgeStart_deg, edgeEnd_deg);
distance_deg = vecnorm(second_deg - first_deg, 2, 2);
nodeCount = size(positions_deg, 1);
cost_deg = Inf(nodeCount);
cost_deg(1:nodeCount + 1:end) = 0;
linearIndex = sub2ind([nodeCount nodeCount], firstNode(visible), secondNode(visible));
cost_deg(linearIndex) = distance_deg(visible);
cost_deg = min(cost_deg, cost_deg.');
acceptedCount = nnz(visible);
rejectedCount = nnz(~visible);
accepted_deg = [first_deg(visible, :), second_deg(visible, :)];
rejected_deg = [first_deg(~visible, :), second_deg(~visible, :)];
accepted_deg = accepted_deg(1:min(2000, acceptedCount), :);
rejected_deg = rejected_deg(1:min(2000, rejectedCount), :);
end

function visible = segmentsAreVisible(first_deg, second_deg, shape, edgeStart_deg, edgeEnd_deg)
% Test segment-boundary intersections in one work-bounded vectorized batch.
visible = true(size(first_deg, 1), 1);
if isempty(shape.Vertices)
    return;
end
middle_deg = (first_deg + second_deg) / 2;
visible = ~isinterior(shape, middle_deg(:, 1), middle_deg(:, 2));
segment_deg = second_deg - first_deg;
boundary_deg = edgeEnd_deg - edgeStart_deg;
offsetAz_deg = edgeStart_deg(:, 1).' - first_deg(:, 1);
offsetEl_deg = edgeStart_deg(:, 2).' - first_deg(:, 2);
denominator = segment_deg(:, 1) .* boundary_deg(:, 2).' - ...
    segment_deg(:, 2) .* boundary_deg(:, 1).';
scale_deg = bmtpEngine.createCoordinateTolerances( ...
    first_deg, second_deg, edgeStart_deg, edgeEnd_deg);
tolerance = 512 * eps(scale_deg^2);
nonparallel = abs(denominator) > tolerance;
safeDenominator = denominator;
safeDenominator(~nonparallel) = 1;
firstFraction = (offsetAz_deg .* boundary_deg(:, 2).' - ...
    offsetEl_deg .* boundary_deg(:, 1).') ./ safeDenominator;
secondFraction = (offsetAz_deg .* segment_deg(:, 2) - ...
    offsetEl_deg .* segment_deg(:, 1)) ./ safeDenominator;
crosses = nonparallel & firstFraction >= -1e-12 & firstFraction <= 1 + 1e-12 & ...
    secondFraction >= -1e-12 & secondFraction <= 1 + 1e-12;
collinear = ~nonparallel & abs(offsetAz_deg .* segment_deg(:, 2) - ...
    offsetEl_deg .* segment_deg(:, 1)) <= tolerance;
segmentScale = max(sum(segment_deg.^2, 2), eps);
firstProjection = (offsetAz_deg .* segment_deg(:, 1) + ...
    offsetEl_deg .* segment_deg(:, 2)) ./ segmentScale;
nextOffsetAz_deg = edgeEnd_deg(:, 1).' - first_deg(:, 1);
nextOffsetEl_deg = edgeEnd_deg(:, 2).' - first_deg(:, 2);
secondProjection = (nextOffsetAz_deg .* segment_deg(:, 1) + ...
    nextOffsetEl_deg .* segment_deg(:, 2)) ./ segmentScale;
overlaps = collinear & max(min(firstProjection, secondProjection), 0) <= ...
    min(max(firstProjection, secondProjection), 1) + 1e-12;
visible = visible & ~any(crosses | overlaps, 2);
end

function representatives_deg = createRepresentatives(shape)
% Select one guaranteed interior point for each connected occupied region.
shapeRegions = regions(shape);
representatives_deg = zeros(numel(shapeRegions), 2);
for regionIndex = 1:numel(shapeRegions)
    [candidate_deg, radius_deg] = incenter(triangulation(shapeRegions(regionIndex)));
    [~, largestIndex] = max(radius_deg);
    representatives_deg(regionIndex, :) = candidate_deg(largestIndex, :);
end
end

function seeds = appendTimedSeed(seeds, template, route_deg, routeTime_s)
% Append a distinct timed route while preserving an absolute-duration class.
if isempty(route_deg) || routeTime_s(end) <= routeTime_s(1)
    return;
end
seed = template;
seed.Index = numel(seeds) + 1;
seed.Source = "timeExpandedVisibilityGraph";
positionChanges = [true; vecnorm(diff(route_deg, 1, 1), 2, 2) > 1e-12];
if any(~positionChanges(2:end)) && nnz(positionChanges) == 2
    seed.Source = "directWait";
end
seed.position_deg = route_deg;
seed.tau = (routeTime_s - routeTime_s(1)) / (routeTime_s(end) - routeTime_s(1));
seed.EstimatedDuration_s = routeTime_s(end) - routeTime_s(1);
seed.Length_deg = routeLength(route_deg);
seeds(end + 1, 1) = seed;
end

function seed = createSpatialSeed(template, index, route_deg, ...
        directDuration_s, available_s, limits)
% Create one reachable spatial proposal with a conservative warm duration.
seed = template;
seed.Index = index;
seed.Source = "visibilityGraph";
seed.position_deg = route_deg;
[seed.tau, seed.Length_deg] = routeTau(route_deg);
minimumDuration_s = routeDuration(route_deg, limits);
seed.EstimatedDuration_s = min(available_s, max([directDuration_s, ...
    minimumDuration_s, seed.Length_deg / max(limits.maxVelocity_deg_s)]));
if minimumDuration_s > available_s
    seed.EstimatedDuration_s = Inf;
end
end

function diagnostics = appendSearchDiagnostics(diagnostics, record)
% Append decision-faithful search evidence and full counts.
diagnostics.ExpandedCount = diagnostics.ExpandedCount + record.ExpandedCount;
diagnostics.RejectedTransitionCount = diagnostics.RejectedTransitionCount + ...
    record.RejectedTransitionCount;
diagnostics.ExploredNodes_deg = [diagnostics.ExploredNodes_deg; record.ExploredNodes_deg];
diagnostics.FrontierNodes_deg = [diagnostics.FrontierNodes_deg; record.FrontierNodes_deg];
if ~isempty(record.BestPartialRoute_deg)
    diagnostics.BestPartialRoute_deg = record.BestPartialRoute_deg;
end
end

function diagnostics = emptyDiagnostics(start_deg, goal_deg, clusterDistance_deg)
% Initialize every stable field before direct-only and no-path exits.
coverage = struct("ExactSpatialProposalUsed", false, ...
    "ReducedSpatialProposalUsed", false, "TimedSearchAttempted", false, ...
    "ExtendedTimedSearchAttempted", false, "TimedSearchUsesExactObstacles", false, ...
    "TimedSearchSuppressionReason", "graphNotBuilt", ...
    "CompletenessLost", true, "CompletenessLossReason", "graphNotBuilt");
cluster = struct("Distance_deg", clusterDistance_deg, ...
    "SourceRegionCount", 0, "ClusterGroupCount", 0, ...
    "ClusteredRegionCount", 0, "ClusterBoundary_deg", zeros(0, 2));
diagnostics = struct("GraphType", "timeExpandedVisibilityGraph", ...
    "Bounds_deg", [NaN NaN NaN NaN], "Resolution_deg", NaN, ...
    "CandidateOffset_deg", NaN, "CandidateOffsetRetryCount", 0, ...
    "SampleTimes_s", zeros(0, 1), "SampledShapeCount", 0, "DenseSeedEnvelopeUsed", false, ...
    "DenseSeedEnvelope_deg", zeros(0, 2), "Coverage", coverage, ...
    "SeedCluster", cluster, "NodeCount", 0, ...
    "NodePosition_deg", zeros(0, 2), "VisibilityWorkBudget", 1e6, ...
    "EstimatedExhaustiveVisibilityWork", 0, "ExhaustiveVisibilityUsed", false, ...
    "ExhaustiveVisibilityFallbackUsed", false, ...
    "VisibilityCandidatePairCount", 0, "VisibilityEdgeCount", 0, ...
    "AcceptedEdges_deg", zeros(0, 4), "RejectedEdges_deg", zeros(0, 4), ...
    "TemporalLayerTimes_s", zeros(0, 1), "TemporalLayerCount", 0, ...
    "TemporalNodeCount", 0, "WaitEdgeCount", 0, "MotionEdgeCount", 0, ...
    "HomologySearchAttempted", false, "HomologyRepresentative_deg", zeros(0, 2), ...
    "HomologyClassSignatures", zeros(0, 0, "int8"), ...
    "HomologyClassCount", 0, "HomologyStateCount", 0, "HomologySearchTruncated", false, ...
    "RouteCleanupAttemptedCount", 0, "RouteCleanupCandidateCount", 0, ...
    "RouteCleanupVisibilityRejectedCount", 0, "RouteCleanupHomologyRejectedCount", 0, ...
    "RouteCleanupAcceptedCount", 0, ...
    "RouteCleanupLengthReduction_deg", 0, "ExpandedCount", 0, ...
    "RejectedTransitionCount", 0, "GeneratedSeedCount", 1, ...
    "ExploredNodes_deg", zeros(0, 2), "FrontierNodes_deg", zeros(0, 2), ...
    "BestPartialRoute_deg", zeros(0, 2), "Start_deg", start_deg, ...
    "Goal_deg", goal_deg, "TraceDownsampleRule", ...
    "Temporal/explored states are complete; static edge traces retain " + ...
    "the first 2000 accepted and rejected decisions");
end

function times_s = obstacleTimes(obstacles, start_s, end_s)
% Retain all source, midpoint, endpoint, and uniform request times.
times_s = [start_s; linspace(start_s, end_s, 9).'; end_s];
for obstacleIndex = 1:numel(obstacles)
    source_s = obstacles(obstacleIndex).time_s(:);
    times_s = [times_s; source_s; (source_s(1:end - 1) + source_s(2:end)) / 2]; %#ok<AGROW>
end
times_s = unique(times_s(times_s >= start_s & times_s <= end_s));
end

function [tau, length_deg] = routeTau(route_deg)
% Parameterize a polyline by normalized cumulative Euclidean length.
cumulative_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
length_deg = cumulative_deg(end);
if length_deg <= 0
    tau = linspace(0, 1, size(route_deg, 1)).';
else
    tau = cumulative_deg / length_deg;
end
end

function duration_s = routeDuration(route_deg, limits)
% Bound independent-axis traversal by total variation and velocity limits.
duration_s = max([1e-3, sum(abs(diff(route_deg, 1, 1)), 1) ./ limits.maxVelocity_deg_s]);
end

function length_deg = routeLength(route_deg)
% Measure Euclidean polyline length.
length_deg = sum(vecnorm(diff(route_deg, 1, 1), 2, 2));
end
