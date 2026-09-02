function edgeCheck = evaluateVisibilityPairs( ...
        positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg)
%% Section 0: Header & Readme
% SYNTAX
%   edgeCheck = obstacleAvoidance.search.evaluateVisibilityPairs( ...
%       positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg)
%**************************************************************************
% PURPOSE
%   - Check candidate graph segments against proposal geometry.
%   - Return graph costs and retained accepted and rejected decisions.
%**************************************************************************
% INPUTS
%   - positions_deg (N-by-2 finite numeric matrix)
%       Graph nodes in [azimuth elevation] order.
%   - pairMask (N-by-N logical matrix)
%       Upper-triangular candidate-pair selection.
%   - shape (scalar polyshape)
%       Spatial proposal obstacle used for visibility checks.
%   - edgeStart_deg, edgeEnd_deg (M-by-2 numeric matrices)
%       Ordered proposal-boundary edge endpoints.
%**************************************************************************
% OUTPUTS
%   - edgeCheck (scalar struct)
%       Symmetric costs, pair decisions, rejection reasons, and counts.
%**************************************************************************
% UNITS
%   - Positions and edge costs are degrees.
%**************************************************************************

%% Section 1: Check Candidate Segments

% A visible proposal edge only proves that a straight spatial suggestion
% avoids this proposal shape. Timed motion validation remains authoritative.

[firstNodeIndex, secondNodeIndex] = find(pairMask);
first_deg = positions_deg(firstNodeIndex, :);
second_deg = positions_deg(secondNodeIndex, :);
isVisible = obstacleAvoidance.search.checkVisibilitySegments( ...
    first_deg, second_deg, shape, edgeStart_deg, edgeEnd_deg);
distance_deg = vecnorm(second_deg - first_deg, 2, 2);

%% Section 2: Create Graph Costs And Details

nodeCount = size(positions_deg, 1);
cost_deg = Inf(nodeCount);
cost_deg(1:nodeCount + 1:end) = 0;
linearIndex = sub2ind([nodeCount nodeCount], ...
    firstNodeIndex(isVisible), secondNodeIndex(isVisible));
cost_deg(linearIndex) = distance_deg(isVisible);
cost_deg = min(cost_deg, cost_deg.');
acceptedCount = nnz(isVisible);
rejectedCount = nnz(~isVisible);
acceptedEdges_deg = [first_deg(isVisible, :), second_deg(isVisible, :)];
rejectedEdges_deg = [first_deg(~isVisible, :), second_deg(~isVisible, :)];
acceptedEdges_deg = acceptedEdges_deg( ...
    1:min(2000, acceptedCount), :);
rejectedEdges_deg = rejectedEdges_deg( ...
    1:min(2000, rejectedCount), :);
rejectionReasons = repmat( ...
    "blockedByProposalGeometry", rejectedCount, 1);
rejectionReasons = rejectionReasons(1:min(2000, rejectedCount));
edgeCheck = struct( ...
    "Cost_deg", cost_deg, ...
    "CandidatePairs", [firstNodeIndex secondNodeIndex], ...
    "IsVisible", isVisible, ...
    "AcceptedEdges_deg", acceptedEdges_deg, ...
    "RejectedEdges_deg", rejectedEdges_deg, ...
    "RejectionReasons", rejectionReasons, ...
    "AcceptedCount", acceptedCount, ...
    "RejectedCount", rejectedCount);
end
