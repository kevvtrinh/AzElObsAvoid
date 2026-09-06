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
%   - Positions, graph costs, bounds, and offsets are degrees.
%

%% Section 1: Create The Offset Schedule Inputs

% Choose initial and retry offsets from obstacle and workspace dimensions.

shape               = proposal.shape;
start_deg           = proposal.start_deg;
goal_deg            = proposal.goal_deg;
allPositions_deg    = [start_deg; goal_deg; shape.Vertices];
coordinateScale_deg = bmtpEngine.createCoordinateTolerances(allPositions_deg);
baseOffset_deg      = max(1e-3, 256 * eps(coordinateScale_deg));
maximumOffset_deg   = max([diff(limits.azimuthInterval_deg), diff(limits.elevationInterval_deg)]);
workBudget          = 1e6;

%% Section 2: Retry Visibility Attempts

% Retry disconnected visibility graphs with a wider offset.
% These spatial checks do not validate the final timed motion.

attempts                  = repmat(createEmptyAttempt(), 0, 1);
candidateOffset_deg       = baseOffset_deg;
offsetRetryCount          = 0;
anyExhaustiveUsed         = false;
anyExhaustiveFallbackUsed = false;
% Continue the search until build visibility graph reaches an explicit termination condition.
while true
    attempt = obstacleAvoidance.search.createVisibilityAttempt(shape, start_deg, goal_deg, limits, candidateOffset_deg, offsetRetryCount, workBudget);
    attempts(end + 1, 1) = attempt; %#ok<AGROW>
    anyExhaustiveUsed         = anyExhaustiveUsed || attempt.ExhaustiveVisibilityUsed;
    anyExhaustiveFallbackUsed = anyExhaustiveFallbackUsed || attempt.ExhaustiveVisibilityFallbackUsed;
    % Stop expanding the visibility offset after connectivity is achieved or the permitted offset is exhausted.
    if attempt.IsConnected || candidateOffset_deg >= maximumOffset_deg
        break;
    end
    candidateOffset_deg = min(4 * candidateOffset_deg, maximumOffset_deg);
    offsetRetryCount    = offsetRetryCount + 1;
end

%% Section 3: Create The Final Graph Record

% Keep final-attempt diagnostics and aggregate the exhaustive-search flags.

finalAttempt = attempts(end);
minimum_deg  = min(allPositions_deg, [], 1);
maximum_deg  = max(allPositions_deg, [], 1);
record       = struct("Bounds_deg", [minimum_deg(1), maximum_deg(1), ...
    minimum_deg(2), maximum_deg(2)], "CandidateOffset_deg", finalAttempt.CandidateOffset_deg, "CandidateOffsetRetryCount", finalAttempt.OffsetRetryCount, "VisibilityWorkBudget", workBudget, "EstimatedExhaustiveVisibilityWork", finalAttempt.EstimatedExhaustiveVisibilityWork, "ExhaustiveVisibilityUsed", anyExhaustiveUsed, "ExhaustiveVisibilityFallbackUsed", anyExhaustiveFallbackUsed, "VisibilityCandidatePairCount", finalAttempt.VisibilityCandidatePairCount, "VisibilityEdgeCount", finalAttempt.VisibilityEdgeCount, "AcceptedEdges_deg", finalAttempt.AcceptedEdges_deg, "RejectedEdges_deg", finalAttempt.RejectedEdges_deg, "RejectedTransitionCount", finalAttempt.RejectedTransitionCount);
visibilityGraph = struct("NodePosition_deg", finalAttempt.Nodes.Positions_deg, ...
    "EdgeCost_deg", finalAttempt.Cost_deg, ...
    "ObstacleReferencePoints_deg", ...
    createObstacleReferencePoints(shape), "Attempts", attempts, "FinalAttemptIndex", numel(attempts), "IsConnected", finalAttempt.IsConnected, "Record", record);
end

%% Section 4: Local Functions

function attempt = createEmptyAttempt()
    % Initialize visibility-attempt records.
    attempt = struct();
    attempt.OffsetRetryCount                  = 0;
    attempt.CandidateOffset_deg               = NaN;
    attempt.Nodes                             = struct();
    attempt.InitialPairSet                    = struct();
    attempt.FinalCandidatePairs               = zeros(0, 2);
    attempt.AcceptedEdges_deg                 = zeros(0, 4);
    attempt.RejectedEdges_deg                 = zeros(0, 4);
    attempt.EdgeRejectionReasons              = strings(0, 1);
    attempt.GraphComponents                   = zeros(1, 0);
    attempt.RecoverySteps                     = strings(0, 1);
    attempt.ExhaustiveVisibilityUsed          = false;
    attempt.ExhaustiveVisibilityFallbackUsed  = false;
    attempt.IsConnected                       = false;
    attempt.Cost_deg                          = zeros(0);
    attempt.VisibilityEdgeCount               = 0;
    attempt.RejectedTransitionCount           = 0;
    attempt.VisibilityCandidatePairCount      = 0;
    attempt.EstimatedExhaustiveVisibilityWork = 0;
end

function referencePoints_deg = createObstacleReferencePoints(shape)
    % Select one guaranteed interior point for each connected occupied region.
    shapeRegions        = regions(shape);
    referencePoints_deg = zeros(numel(shapeRegions), 2);
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:numel(shapeRegions)
        [candidate_deg, radius_deg] = incenter(triangulation(shapeRegions(regionIndex)));
        [~, largestIndex]           = max(radius_deg);
        referencePoints_deg(regionIndex, :) = candidate_deg(largestIndex, :);
    end
end
