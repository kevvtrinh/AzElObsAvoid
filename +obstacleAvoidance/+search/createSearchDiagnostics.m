function diagnostics = createSearchDiagnostics(proposal, visibilityGraph, routeSet, seeds)
%% Section 0: Header & Readme
% SYNTAX
%   diagnostics = obstacleAvoidance.search.createSearchDiagnostics( ...
%       proposal, visibilityGraph, routeSet, seeds)
%
% PURPOSE
%   - Assemble stable search diagnostics from completed production stages
%     without recomputing proposal, graph, route, or seed decisions.
%
% INPUTS
%   - proposal, visibilityGraph, routeSet (scalar structs)
%       Completed stage records, or empty structs when graph work was skipped.
%   - seeds (nonempty route-seed struct array)
%       Final deterministic seed order.
%
% OUTPUTS
%   - diagnostics (scalar struct)
%       Stable graph, route, rejection, coverage, and bounded-search evidence.
%
% UNITS
%   - Position and path length are coordinate units; time is seconds.
%

%% Section 1: Create Stable Direct-Only Diagnostics

start_units   = seeds(1).position_units(1, :);
goal_units    = seeds(1).position_units(end, :);
diagnostics = emptyDiagnostics(start_units, goal_units);
diagnostics.GeneratedSeedCount = numel(seeds);
if isempty(fieldnames(proposal))
    return;
end

%% Section 2: Copy Proposal And Visibility Evidence

diagnostics.SampleTimes_s         = proposal.sampleTimes_s;
diagnostics.SampledShapeCount     = proposal.sampledShapeCount;
diagnostics.DenseSeedEnvelopeUsed = proposal.usedDenseEnvelope;
diagnostics.NodeCount             = size(visibilityGraph.NodePosition_units, 1);
diagnostics.NodePosition_units      = visibilityGraph.NodePosition_units;
if proposal.usedDenseEnvelope
    diagnostics.DenseSeedEnvelope_units = proposal.shape.Vertices;
end
diagnostics.GraphType          = "visibilityGraph";
diagnostics.VisibilityAttempts = visibilityGraph.Attempts;
graphRecord = visibilityGraph.Record;
graphFields = ["Bounds_units", "CandidateOffset_units", ...
    "CandidateOffsetRetryCount", "VisibilityWorkBudget", ...
    "EstimatedExhaustiveVisibilityWork", "ExhaustiveVisibilityUsed", ...
    "ExhaustiveVisibilityFallbackUsed", ...
    "VisibilityCandidatePairCount", "VisibilityEdgeCount", ...
    "AcceptedEdges_units", "RejectedEdges_units", ...
    "RejectedTransitionCount"];
% Apply the required validation or transfer to each field.
for fieldIndex = 1:numel(graphFields)
    fieldName = graphFields(fieldIndex);
    diagnostics.(fieldName) = graphRecord.(fieldName);
end
diagnostics.RouteClassRepresentative_units          = visibilityGraph.ObstacleReferencePoints_units;
diagnostics.Coverage.ExactSpatialProposalUsed     = ~proposal.usedDenseEnvelope;
diagnostics.Coverage.ReducedSpatialProposalUsed   = proposal.usedDenseEnvelope;
diagnostics.Coverage.TimedSearchInitialDeferred   = routeSet.TimedSearchDeferred;
diagnostics.Coverage.TimedSearchRecoveryAttempted = routeSet.TimedSearchRecoveryAttempted;
diagnostics.Coverage.MultiWindingRouteCount       = numel(routeSet.DeferredSpatialRoutes_units);
diagnostics.Coverage.MultiWindingSolveAttempted   = routeSet.DeferredSpatialSolveAttempted;
diagnostics.Coverage.TimedSearchSuppressionReason = "staticObstacleHistory";

%% Section 3: Copy Timed And Spatial Search Evidence

if routeSet.TimedSearchAttempted
    diagnostics.Coverage.TimedSearchAttempted          = true;
    diagnostics.GraphType                              = "timeExpandedVisibilityGraph";
    diagnostics.Coverage.TimedSearchUsesExactObstacles = true;
    diagnostics.Coverage.TimedSearchSuppressionReason  = "";
    timedRecord = routeSet.TimedSearchRecord;
    diagnostics.TemporalLayerTimes_s        = timedRecord.LayerTimes_s;
    diagnostics.TemporalLayerCount          = numel(timedRecord.LayerTimes_s);
    diagnostics.TemporalCandidateLayerCount = timedRecord.CandidateLayerCount;
    diagnostics.TemporalNodeCount           = timedRecord.NodeCount;
    diagnostics.WaitEdgeCount               = timedRecord.WaitEdgeCount;
    diagnostics.MotionEdgeCount             = timedRecord.MotionEdgeCount;
    diagnostics.TimedSearchGoalTimeMode     = routeSet.TimedSearchOptions.GoalTimeMode;
    diagnostics = appendSearchDiagnostics(diagnostics, timedRecord);
    diagnostics.TimedBestPartialRoute_units = timedRecord.BestPartialRoute_units;
    diagnostics.SelectedGoalLayerIndex    = timedRecord.SelectedGoalLayerIndex;
    diagnostics.ReachableGoalLayerCount   = timedRecord.ReachableGoalLayerCount;
else
    diagnostics.Coverage.TimedSearchSuppressionReason = routeSet.TimedSearchSuppressionReason;
end
searchRecord = routeSet.SpatialSearchRecord;
diagnostics  = appendSearchDiagnostics(diagnostics, searchRecord);
diagnostics.SpatialBestPartialRoute_units = searchRecord.BestPartialRoute_units;
% Fall back to the graph search's partial route when no evaluated seed produced a complete result.
if ~isempty(diagnostics.TimedBestPartialRoute_units)
    diagnostics.BestPartialRoute_units = diagnostics.TimedBestPartialRoute_units;
else
    diagnostics.BestPartialRoute_units = diagnostics.SpatialBestPartialRoute_units;
end
diagnostics.RouteClassSearchAttempted     = routeSet.MaximumSpatialClassCount > 0;
diagnostics.RouteClassSignatures          = routeSet.RouteClassPattern;
diagnostics.RouteClassCount               = size(routeSet.RouteClassPattern, 1);
diagnostics.RouteClassStateCount          = searchRecord.StateCount;
diagnostics.RouteClassSearchTruncated     = searchRecord.Truncated;
diagnostics.RouteClassStoppedAtClassLimit = searchRecord.StoppedAtClassLimit;
cleanupFields = ["RouteShorteningAttemptedCount", ...
    "RouteShorteningCandidateCount", "RouteShorteningVisibilityRejectedCount", ...
    "RouteShorteningRouteClassRejectedCount", "RouteShorteningAcceptedCount", ...
    "RouteShorteningLengthReduction_units"];
% Apply the required validation or transfer to each field.
for fieldIndex = 1:numel(cleanupFields)
    fieldName = cleanupFields(fieldIndex);
    diagnostics.(fieldName) = searchRecord.(fieldName);
end
% Attribute completeness loss to the reduced envelope when used; otherwise attribute it to the bounded graph search.
if proposal.usedDenseEnvelope
    diagnostics.Coverage.CompletenessLossReason = "reducedSpatialProposalAndBoundedSearch";
else
    diagnostics.Coverage.CompletenessLossReason = "boundedSeedNodeAndTimeSearch";
end
end

%% Section 4: Local Functions

function diagnostics = appendSearchDiagnostics(diagnostics, record)
    % Append search traces and counts.
    diagnostics.ExpandedCount           = diagnostics.ExpandedCount + record.ExpandedCount;
    diagnostics.RejectedTransitionCount = diagnostics.RejectedTransitionCount + record.RejectedTransitionCount;
    diagnostics.ExploredNodes_units       = [diagnostics.ExploredNodes_units; ...
        record.ExploredNodes_units];
    diagnostics.FrontierNodes_units = [diagnostics.FrontierNodes_units; ...
        record.FrontierNodes_units];
end

function diagnostics = emptyDiagnostics(start_units, goal_units)
    % Initialize diagnostics, including direct-only and no-path cases.
    coverage = struct();
    coverage.ExactSpatialProposalUsed      = false;
    coverage.ReducedSpatialProposalUsed    = false;
    coverage.TimedSearchAttempted          = false;
    coverage.TimedSearchInitialDeferred    = false;
    coverage.TimedSearchRecoveryAttempted  = false;
    coverage.MultiWindingRouteCount        = 0;
    coverage.MultiWindingSolveAttempted    = false;
    coverage.TimedSearchUsesExactObstacles = false;
    coverage.TimedSearchSuppressionReason  = "graphNotBuilt";
    coverage.CompletenessLost              = true;
    coverage.CompletenessLossReason        = "graphNotBuilt";
    diagnostics = struct("GraphType", "notBuilt", ...
        "Bounds_units", [NaN NaN NaN NaN], ...
        "VisibilityAttempts", struct([]), ...
        "TimedBestPartialRoute_units", zeros(0, 2), "SpatialBestPartialRoute_units", zeros(0, 2), ...
        "SelectedGoalLayerIndex", [], "ReachableGoalLayerCount", 0, ...
        "CandidateOffset_units", NaN, "CandidateOffsetRetryCount", 0, ...
        "SampleTimes_s", zeros(0, 1), "SampledShapeCount", 0, ...
        "DenseSeedEnvelopeUsed", false, ...
        "DenseSeedEnvelope_units", zeros(0, 2), "Coverage", coverage, ...
        "NodeCount", 0, ...
        "NodePosition_units", zeros(0, 2), "VisibilityWorkBudget", 1e6, ...
        "EstimatedExhaustiveVisibilityWork", 0, ...
        "ExhaustiveVisibilityUsed", false, ...
        "ExhaustiveVisibilityFallbackUsed", false, ...
        "VisibilityCandidatePairCount", 0, "VisibilityEdgeCount", 0, ...
        "AcceptedEdges_units", zeros(0, 4), ...
        "RejectedEdges_units", zeros(0, 4), ...
        "TemporalLayerTimes_s", zeros(0, 1), "TemporalLayerCount", 0, ...
        "TemporalCandidateLayerCount", 0, ...
        "TemporalNodeCount", 0, "WaitEdgeCount", 0, "MotionEdgeCount", 0, ...
        "RouteClassSearchAttempted", false, ...
        "RouteClassRepresentative_units", zeros(0, 2), ...
        "RouteClassSignatures", zeros(0, 0), ...
        "RouteClassCount", 0, "RouteClassStateCount", 0, ...
        "RouteClassSearchTruncated", false, ...
        "RouteClassStoppedAtClassLimit", false, ...
        "RouteShorteningAttemptedCount", 0, ...
        "RouteShorteningCandidateCount", 0, ...
        "RouteShorteningVisibilityRejectedCount", 0, ...
        "RouteShorteningRouteClassRejectedCount", 0, ...
        "RouteShorteningAcceptedCount", 0, ...
        "RouteShorteningLengthReduction_units", 0, "ExpandedCount", 0, ...
        "RejectedTransitionCount", 0, "GeneratedSeedCount", 1, ...
        "ExploredNodes_units", zeros(0, 2), ...
        "FrontierNodes_units", zeros(0, 2), ...
        "BestPartialRoute_units", zeros(0, 2), "Start_units", start_units, ...
        "Goal_units", goal_units, "TraceDownsampleRule", ...
        "Temporal/explored states are complete; static edge traces retain " + ...
        "the first 2000 accepted and rejected decisions");
end
