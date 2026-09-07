function visibilityGraph = createVisibilityGraph(limits, proposal)
%% Section 0: Header & Readme
% SYNTAX
%   visibilityGraph = obstacleAvoidance.search.createVisibilityGraph( ...
%       limits, proposal)
%
% PURPOSE
%   - Connect points with clear straight segments; retry farther from obstacles if needed.
%   - Return all attempts and the final graph used by route search.
%
% INPUTS
%   - limits: workspace bounds.
%   - proposal (scalar proposal-geometry struct)
%       Spatial shape, endpoints, and reusable boundary edges.
%
% OUTPUTS
%   - visibilityGraph (scalar struct)
%       Final nodes and costs, all offset attempts, and the graph
%       record used by search diagnostics.
%
% UNITS
%   - Positions, graph costs, bounds, and offsets are coordinate units.
%

%% Section 1: Create The Offset Schedule Inputs

% Choose initial and retry offsets from obstacle and workspace dimensions.

shape               = proposal.shape;
start_units           = proposal.start_units;
goal_units            = proposal.goal_units;
allPositions_units    = [start_units; goal_units; shape.Vertices];
coordinateScale_units = bmtpEngine.createCoordinateTolerances(allPositions_units);
baseOffset_units      = max(1e-3, 256 * eps(coordinateScale_units));
maximumOffset_units   = max([diff(limits.xInterval_units), diff(limits.yInterval_units)]);
workBudget          = 1e6;

%% Section 2: Retry Visibility Attempts

% Retry disconnected visibility graphs with a wider offset.
% These spatial checks do not validate the final timed motion.

attempts                  = repmat(createEmptyAttempt(), 0, 1);
candidateOffset_units       = baseOffset_units;
offsetRetryCount          = 0;
anyExhaustiveUsed         = false;
anyExhaustiveFallbackUsed = false;
% Continue the search until build visibility graph reaches an explicit termination condition.
while true
    attempt = obstacleAvoidance.search.createVisibilityAttempt(shape, start_units, goal_units, limits, candidateOffset_units, offsetRetryCount, workBudget);
    attempts(end + 1, 1) = attempt; %#ok<AGROW>
    anyExhaustiveUsed         = anyExhaustiveUsed || attempt.ExhaustiveVisibilityUsed;
    anyExhaustiveFallbackUsed = anyExhaustiveFallbackUsed || attempt.ExhaustiveVisibilityFallbackUsed;
    % Stop expanding the visibility offset after connectivity is achieved or the permitted offset is exhausted.
    if attempt.IsConnected || candidateOffset_units >= maximumOffset_units
        break;
    end
    candidateOffset_units = min(4 * candidateOffset_units, maximumOffset_units);
    offsetRetryCount    = offsetRetryCount + 1;
end

%% Section 3: Create The Final Graph Record

% Keep final-attempt diagnostics and aggregate the exhaustive-search flags.

finalAttempt = attempts(end);
minimum_units  = min(allPositions_units, [], 1);
maximum_units  = max(allPositions_units, [], 1);
record       = struct("Bounds_units", [minimum_units(1), maximum_units(1), ...
    minimum_units(2), maximum_units(2)], "CandidateOffset_units", finalAttempt.CandidateOffset_units, "CandidateOffsetRetryCount", finalAttempt.OffsetRetryCount, "VisibilityWorkBudget", workBudget, "EstimatedExhaustiveVisibilityWork", finalAttempt.EstimatedExhaustiveVisibilityWork, "ExhaustiveVisibilityUsed", anyExhaustiveUsed, "ExhaustiveVisibilityFallbackUsed", anyExhaustiveFallbackUsed, "VisibilityCandidatePairCount", finalAttempt.VisibilityCandidatePairCount, "VisibilityEdgeCount", finalAttempt.VisibilityEdgeCount, "AcceptedEdges_units", finalAttempt.AcceptedEdges_units, "RejectedEdges_units", finalAttempt.RejectedEdges_units, "RejectedTransitionCount", finalAttempt.RejectedTransitionCount);
visibilityGraph = struct("NodePosition_units", finalAttempt.Nodes.Positions_units, ...
    "EdgeCost_units", finalAttempt.Cost_units, ...
    "ObstacleReferencePoints_units", ...
    createObstacleReferencePoints(shape), "Attempts", attempts, "FinalAttemptIndex", numel(attempts), "IsConnected", finalAttempt.IsConnected, "Record", record);
end

%% Section 4: Local Functions

function attempt = createEmptyAttempt()
    % Initialize visibility-attempt records.
    attempt = struct();
    attempt.OffsetRetryCount                  = 0;
    attempt.CandidateOffset_units               = NaN;
    attempt.Nodes                             = struct();
    attempt.InitialPairSet                    = struct();
    attempt.FinalCandidatePairs               = zeros(0, 2);
    attempt.AcceptedEdges_units                 = zeros(0, 4);
    attempt.RejectedEdges_units                 = zeros(0, 4);
    attempt.EdgeRejectionReasons              = strings(0, 1);
    attempt.GraphComponents                   = zeros(1, 0);
    attempt.RecoverySteps                     = strings(0, 1);
    attempt.ExhaustiveVisibilityUsed          = false;
    attempt.ExhaustiveVisibilityFallbackUsed  = false;
    attempt.IsConnected                       = false;
    attempt.Cost_units                          = zeros(0);
    attempt.VisibilityEdgeCount               = 0;
    attempt.RejectedTransitionCount           = 0;
    attempt.VisibilityCandidatePairCount      = 0;
    attempt.EstimatedExhaustiveVisibilityWork = 0;
end

function referencePoints_units = createObstacleReferencePoints(shape)
    % Select one guaranteed interior point for each connected occupied region.
    shapeRegions        = regions(shape);
    referencePoints_units = zeros(numel(shapeRegions), 2);
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:numel(shapeRegions)
        [candidate_units, radius_units] = incenter(triangulation(shapeRegions(regionIndex)));
        [~, largestIndex]           = max(radius_units);
        referencePoints_units(regionIndex, :) = candidate_units(largestIndex, :);
    end
end
