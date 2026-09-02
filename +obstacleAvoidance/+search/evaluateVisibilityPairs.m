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
isVisible = segmentsAreVisible( ...
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

%% Section 3: Local Functions

function isVisible = segmentsAreVisible( ...
        first_deg, second_deg, shape, edgeStart_deg, edgeEnd_deg)
% Test segment-boundary intersections in one work-bounded vectorized batch.
isVisible = true(size(first_deg, 1), 1);
if isempty(shape.Vertices)
    return;
end
middle_deg = (first_deg + second_deg) / 2;
isVisible = ~isinterior(shape, middle_deg(:, 1), middle_deg(:, 2));
segment_deg = second_deg - first_deg;
boundary_deg = edgeEnd_deg - edgeStart_deg;
offsetAzimuth_deg = edgeStart_deg(:, 1).' - first_deg(:, 1);
offsetElevation_deg = edgeStart_deg(:, 2).' - first_deg(:, 2);
denominator = segment_deg(:, 1) .* boundary_deg(:, 2).' - ...
    segment_deg(:, 2) .* boundary_deg(:, 1).';
scale_deg = bmtpEngine.createCoordinateTolerances( ...
    first_deg, second_deg, edgeStart_deg, edgeEnd_deg);
tolerance_deg2 = 512 * eps(scale_deg^2);
isNonparallel = abs(denominator) > tolerance_deg2;
safeDenominator = denominator;
safeDenominator(~isNonparallel) = 1;
firstFraction = (offsetAzimuth_deg .* boundary_deg(:, 2).' - ...
    offsetElevation_deg .* boundary_deg(:, 1).') ./ safeDenominator;
secondFraction = (offsetAzimuth_deg .* segment_deg(:, 2) - ...
    offsetElevation_deg .* segment_deg(:, 1)) ./ safeDenominator;
crosses = isNonparallel & firstFraction >= -1e-12 & ...
    firstFraction <= 1 + 1e-12 & secondFraction >= -1e-12 & ...
    secondFraction <= 1 + 1e-12;
isCollinear = ~isNonparallel & ...
    abs(offsetAzimuth_deg .* segment_deg(:, 2) - ...
    offsetElevation_deg .* segment_deg(:, 1)) <= tolerance_deg2;
segmentScale_deg2 = max(sum(segment_deg.^2, 2), eps);
firstProjection = (offsetAzimuth_deg .* segment_deg(:, 1) + ...
    offsetElevation_deg .* segment_deg(:, 2)) ./ segmentScale_deg2;
nextOffsetAzimuth_deg = edgeEnd_deg(:, 1).' - first_deg(:, 1);
nextOffsetElevation_deg = edgeEnd_deg(:, 2).' - first_deg(:, 2);
secondProjection = (nextOffsetAzimuth_deg .* segment_deg(:, 1) + ...
    nextOffsetElevation_deg .* segment_deg(:, 2)) ./ segmentScale_deg2;
overlaps = isCollinear & ...
    max(min(firstProjection, secondProjection), 0) <= ...
    min(max(firstProjection, secondProjection), 1) + 1e-12;
isVisible = isVisible & ~any(crosses | overlaps, 2);
end
