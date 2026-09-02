function pairSet = createVisibilityPairs( ...
        positions_deg, obstacleEdgeCount, workBudget)
%% Section 0: Header & Readme
% SYNTAX
%   pairSet = obstacleAvoidance.search.createVisibilityPairs( ...
%       positions_deg, obstacleEdgeCount, workBudget)
%**************************************************************************
% PURPOSE
%   - Create the sparse candidate pairs for one visibility attempt.
%   - Report the estimated work of an exhaustive candidate set.
%**************************************************************************
% INPUTS
%   - positions_deg (N-by-2 finite numeric matrix)
%       Graph nodes in [azimuth elevation] order, with endpoints first.
%   - obstacleEdgeCount (nonnegative integer scalar)
%       Number of proposal-boundary edges checked per candidate pair.
%   - workBudget (positive finite scalar)
%       Maximum pair-edge work allowed for exhaustive recovery.
%**************************************************************************
% OUTPUTS
%   - pairSet (scalar struct)
%       Pair mask, pair indices, exhaustive-work estimate, and whether the
%       initial set is already exhaustive.
%**************************************************************************
% UNITS
%   - Positions are degrees; work values are dimensionless counts.
%**************************************************************************

%% Section 1: Create Sparse Or Exhaustive Pairs

% Delaunay edges keep ordinary attempts affordable. Endpoint-to-boundary
% pairs remain explicit because route search cannot repair a graph whose
% required endpoints were never offered connections.

nodeCount = size(positions_deg, 1);
pairMask = triu(true(nodeCount), 1);
usedExhaustive = nodeCount < 4;
if nodeCount >= 4
    triangulation = delaunayTriangulation(positions_deg);
    pairs = sort(edges(triangulation), 2);
    pairMask = false(nodeCount);
    pairMask(sub2ind( ...
        [nodeCount nodeCount], pairs(:, 1), pairs(:, 2))) = true;
    pairMask(1:2, 3:end) = true;
    pairMask(1, 2) = true;
end
[firstNodeIndex, secondNodeIndex] = find(pairMask);
estimatedExhaustiveWork = nodeCount * (nodeCount - 1) / 2 * ...
    max(1, obstacleEdgeCount);
pairSet = struct( ...
    "PairMask", pairMask, ...
    "CandidatePairs", [firstNodeIndex secondNodeIndex], ...
    "EstimatedExhaustiveWork", estimatedExhaustiveWork, ...
    "UsedExhaustive", usedExhaustive, ...
    "WorkBudget", workBudget);
end
