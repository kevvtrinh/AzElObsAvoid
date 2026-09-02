function [seeds, diagnostics] = createRouteCandidates(scene, request)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics] = ...
%       obstacleAvoidance.search.createRouteCandidates( ...
%       scene, request)
%**************************************************************************
% PURPOSE
%   - Create deterministic direct, timed, and distinct spatial topology
%     proposals with first-class bounded-search diagnostics.
%**************************************************************************
% INPUTS
%   - scene (scalar prepared-scene struct)
%       Prepared obstacles and physical request horizon.
%   - request (scalar planning-request struct)
%       Normalized states, limits, and resolved options.
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

initialState = request.initialState;
goalState = request.goalState;
limits = request.limits;
options = request.options;
obstacles = scene.preparedObstacles;
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
diagnostics = emptyDiagnostics(start_deg, goal_deg);
if options.MaximumSeedCount == 1 || isempty(obstacles)
    return;
end

%% Section 2: Create One Protected Visibility Graph

% Route search needs one spatial obstacle view, but that view can only suggest
% topology. Create it separately so its sample times and any conservative
% dense-history fallback remain inspectable before graph construction.
proposal = obstacleAvoidance.search.createProposalGeometry(scene, request);
sampleTimes_s = proposal.sampleTimes_s;
sweptShape = proposal.shape;
usedDenseEnvelope = proposal.usedDenseEnvelope;
sampledShapeCount = proposal.sampledShapeCount;
diagnostics.SeedCluster.SourceRegionCount = numel(regions(sweptShape));
% Visibility routes are only spatial starting suggestions. Build the graph as
% an inspectable sequence of offset attempts so downstream route search uses
% the exact accepted edges while final validation retains sole approval.
visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
    proposal, request);
nodePosition_deg = visibilityGraph.NodePosition_deg;
edgeCost_deg = visibilityGraph.EdgeCost_deg;
graphRecord = visibilityGraph.Record;
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
usesReducedGeometry = usedDenseEnvelope;
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
    timedSearchOptions = options;
    if options.GoalTimeMode == "balancedArrival"
        % Retain the shortest final-horizon ancestry, then remove terminal
        % dwell. The spatial candidates supply the fast end of the Pareto
        % comparison; this timed seed supplies a later route that exploits
        % obstacle motion without paying for irrelevant goal waiting.
        timedSearchOptions.GoalTimeMode = "fixedArrival";
    end
    [route_deg, routeTime_s, timedRecord] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodePosition_deg, timedCost_deg, obstacles, initialState, ...
        goalState, limits, sampleTimes_s, timedSearchOptions);
    if options.GoalTimeMode == "balancedArrival"
        [route_deg, routeTime_s] = trimTerminalGoalDwell( ...
            route_deg, routeTime_s, goal_deg);
    end
    diagnostics.TemporalLayerTimes_s = timedRecord.LayerTimes_s;
    diagnostics.TemporalLayerCount = numel(timedRecord.LayerTimes_s);
    diagnostics.TemporalCandidateLayerCount = timedRecord.CandidateLayerCount;
    diagnostics.TemporalLayerLimitApplied = timedRecord.LayerLimitApplied;
    diagnostics.TemporalNodeCount = timedRecord.NodeCount;
    diagnostics.WaitEdgeCount = timedRecord.WaitEdgeCount;
    diagnostics.MotionEdgeCount = timedRecord.MotionEdgeCount;
    diagnostics.TimedSearchGoalTimeMode = timedSearchOptions.GoalTimeMode;
    diagnostics = appendSearchDiagnostics(diagnostics, timedRecord);
    seeds = appendTimedSeed(seeds, template, route_deg, routeTime_s);
end

%% Section 4: Select Distinct Spatial Classes

maximumClassCount = max(0, options.MaximumSeedCount - numel(seeds));
edgeStart_deg = proposal.edgeStart_deg;
edgeEnd_deg = proposal.edgeEnd_deg;
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

function [route_deg, routeTime_s] = trimTerminalGoalDwell( ...
        route_deg, routeTime_s, goal_deg)
% Remove only repeated goal occupancy after the first verified arrival.
if isempty(route_deg)
    return;
end
coordinateScale_deg = bmtpEngine.createCoordinateTolerances( ...
    route_deg, goal_deg);
goalTolerance_deg = 256 * eps(coordinateScale_deg);
firstGoalIndex = find(vecnorm(route_deg - goal_deg, 2, 2) <= ...
    goalTolerance_deg, 1, "first");
if isempty(firstGoalIndex)
    return;
end
route_deg = route_deg(1:firstGoalIndex, :);
routeTime_s = routeTime_s(1:firstGoalIndex);
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
seed.Length_deg = obstacleAvoidance.geometry.routeLength(route_deg);
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

function diagnostics = emptyDiagnostics(start_deg, goal_deg)
% Initialize every stable field before direct-only and no-path exits.
coverage = struct("ExactSpatialProposalUsed", false, ...
    "ReducedSpatialProposalUsed", false, "TimedSearchAttempted", false, ...
    "ExtendedTimedSearchAttempted", false, "TimedSearchUsesExactObstacles", false, ...
    "TimedSearchSuppressionReason", "graphNotBuilt", ...
    "CompletenessLost", true, "CompletenessLossReason", "graphNotBuilt");
cluster = struct("Distance_deg", 0, ...
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
    "TemporalCandidateLayerCount", 0, "TemporalLayerLimitApplied", false, ...
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
