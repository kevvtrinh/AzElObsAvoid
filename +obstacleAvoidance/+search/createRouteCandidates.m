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

options = request.options;
obstacles = scene.preparedObstacles;
seeds = obstacleAvoidance.search.createSeeds([], [], request);
start_deg = seeds.position_deg(1, :);
goal_deg = seeds.position_deg(end, :);
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
representatives_deg = visibilityGraph.ObstacleReferencePoints_deg;
diagnostics.HomologyRepresentative_deg = representatives_deg;
usesReducedGeometry = usedDenseEnvelope;
diagnostics.Coverage.ExactSpatialProposalUsed = ~usesReducedGeometry;
diagnostics.Coverage.ReducedSpatialProposalUsed = usesReducedGeometry;
diagnostics.Coverage.TimedSearchSuppressionReason = "staticObstacleHistory";

%% Section 3: Search Timed And Distinct Spatial Routes

% Timed search can exploit obstacle motion, while spatial search preserves
% distinct ways around occupied regions. Coordinate both before seed creation
% so route decisions and search records remain directly inspectable.
routeSet = obstacleAvoidance.search.searchRoutes( ...
    scene, request, proposal, visibilityGraph);
if routeSet.TimedSearchAttempted
    diagnostics.Coverage.TimedSearchAttempted = true;
    diagnostics.Coverage.ExtendedTimedSearchAttempted = true;
    diagnostics.Coverage.TimedSearchUsesExactObstacles = true;
    diagnostics.Coverage.TimedSearchSuppressionReason = "";
    timedRecord = routeSet.TimedSearchRecord;
    diagnostics.TemporalLayerTimes_s = timedRecord.LayerTimes_s;
    diagnostics.TemporalLayerCount = numel(timedRecord.LayerTimes_s);
    diagnostics.TemporalCandidateLayerCount = timedRecord.CandidateLayerCount;
    diagnostics.TemporalLayerLimitApplied = timedRecord.LayerLimitApplied;
    diagnostics.TemporalNodeCount = timedRecord.NodeCount;
    diagnostics.WaitEdgeCount = timedRecord.WaitEdgeCount;
    diagnostics.MotionEdgeCount = timedRecord.MotionEdgeCount;
    diagnostics.TimedSearchGoalTimeMode = ...
        routeSet.TimedSearchOptions.GoalTimeMode;
    diagnostics = appendSearchDiagnostics(diagnostics, timedRecord);
else
    diagnostics.Coverage.TimedSearchSuppressionReason = ...
        routeSet.TimedSearchSuppressionReason;
end

%% Section 4: Create Deterministic Seeds

% Routes become useful to motion solvers only after a deterministic conversion
% to indexed seeds with duration and reduced-geometry provenance. This stage
% may reject unreachable or duplicate spatial suggestions but cannot approve
% any motion.
seeds = obstacleAvoidance.search.createSeeds(routeSet, proposal, request);
searchRecord = routeSet.SpatialSearchRecord;
diagnostics = appendSearchDiagnostics(diagnostics, searchRecord);
diagnostics.HomologySearchAttempted = ...
    routeSet.MaximumSpatialClassCount > 0;
diagnostics.HomologyClassSignatures = routeSet.RouteClassPattern;
diagnostics.HomologyClassCount = ...
    size(routeSet.RouteClassPattern, 1);
diagnostics.HomologyStateCount = searchRecord.StateCount;
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
