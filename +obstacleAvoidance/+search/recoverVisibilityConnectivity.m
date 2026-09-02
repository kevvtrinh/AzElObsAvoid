function recovery = recoverVisibilityConnectivity( ...
        nodes, pairSet, edgeCheck, shape, edgeStart_deg, edgeEnd_deg)
%% Section 0: Header & Readme
% SYNTAX
%   recovery = obstacleAvoidance.search.recoverVisibilityConnectivity( ...
%       nodes, pairSet, edgeCheck, shape, edgeStart_deg, edgeEnd_deg)
%**************************************************************************
% PURPOSE
%   - Add boundary and affordable exhaustive pairs to a disconnected graph.
%   - Preserve each recovery decision and the final connected components.
%**************************************************************************
% INPUTS
%   - nodes (scalar visibility-node struct)
%       Positions and offset proposal shape for one attempt.
%   - pairSet (scalar visibility-pair struct)
%       Initial pair mask, work estimate, and exhaustive-use state.
%   - edgeCheck (scalar visibility-edge-check struct)
%       Initial checked costs and edge decisions.
%   - shape (scalar polyshape)
%       Spatial proposal obstacle used for visibility checks.
%   - edgeStart_deg, edgeEnd_deg (M-by-2 numeric matrices)
%       Ordered proposal-boundary edge endpoints.
%**************************************************************************
% OUTPUTS
%   - recovery (scalar struct)
%       Final pair mask and edge check, components, recovery steps, and
%       exhaustive-path flags.
%**************************************************************************
% UNITS
%   - Geometry and graph costs are degrees; work is dimensionless.
%**************************************************************************

%% Section 1: Check Initial Connectivity

% Route search requires start and goal in one component. Check that invariant
% after actual edge rejection rather than inferring it from candidate pairs.

positions_deg = nodes.Positions_deg;
nodeCount = size(positions_deg, 1);
pairMask = pairSet.PairMask;
usedExhaustive = pairSet.UsedExhaustive;
recoverySteps = strings(0, 1);
component = conncomp(graph(isfinite(edgeCheck.Cost_deg), "upper"));

%% Section 2: Add Offset-Boundary Adjacencies

% Delaunay selection can omit consecutive vertices of the offset boundary.
% Offer those known local adjacencies before considering every node pair.

if component(1) ~= component(2) && nodeCount >= 4
    [boundaryStart_deg, boundaryEnd_deg] = ...
        obstacleAvoidance.geometry.boundaryToEdges( ...
        nodes.CandidateShape, 1e-12);
    [startFound, startIndex] = ismember( ...
        boundaryStart_deg, positions_deg, "rows");
    [endFound, endIndex] = ismember( ...
        boundaryEnd_deg, positions_deg, "rows");
    boundaryPairs = sort([startIndex(startFound & endFound), ...
        endIndex(startFound & endFound)], 2);
    boundaryPairs = boundaryPairs( ...
        boundaryPairs(:, 1) ~= boundaryPairs(:, 2), :);
    augmentedPairMask = pairMask;
    augmentedPairMask(sub2ind([nodeCount nodeCount], ...
        boundaryPairs(:, 1), boundaryPairs(:, 2))) = true;
    if nnz(augmentedPairMask) > nnz(pairMask)
        pairMask = augmentedPairMask;
        edgeCheck = obstacleAvoidance.search.evaluateVisibilityPairs( ...
            positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg);
        component = conncomp( ...
            graph(isfinite(edgeCheck.Cost_deg), "upper"));
        recoverySteps(end + 1, 1) = "addedBoundaryPairs";
    end
end

%% Section 3: Use An Affordable Exhaustive Fallback

% Exhaustive visibility can recover connectivity missed by sparse pairs, but
% only when the precomputed pair-edge work remains within the same bound used
% by the original graph construction.

usedFallback = component(1) ~= component(2) && ...
    ~usedExhaustive && ...
    pairSet.EstimatedExhaustiveWork <= pairSet.WorkBudget;
if usedFallback
    pairMask = triu(true(nodeCount), 1);
    edgeCheck = obstacleAvoidance.search.evaluateVisibilityPairs( ...
        positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg);
    usedExhaustive = true;
    component = conncomp(graph(isfinite(edgeCheck.Cost_deg), "upper"));
    recoverySteps(end + 1, 1) = "usedExhaustivePairs";
end

%% Section 4: Assemble Recovery Details

recovery = struct( ...
    "PairMask", pairMask, ...
    "EdgeCheck", edgeCheck, ...
    "Components", component, ...
    "RecoverySteps", recoverySteps, ...
    "UsedExhaustive", usedExhaustive, ...
    "UsedExhaustiveFallback", usedFallback, ...
    "IsConnected", component(1) == component(2));
end
