function attempt = createVisibilityAttempt(shape, start_units, goal_units, limits, candidateOffset_units, offsetRetryCount, workBudget)
%% Section 0: Header & Readme
% SYNTAX
%   attempt = obstacleAvoidance.search.createVisibilityAttempt( ...
%       shape, start_units, goal_units, limits, candidateOffset_units, ...
%       offsetRetryCount, workBudget)
%
% PURPOSE
%   - Create, check, and recover one offset visibility-graph attempt.
%   - Return every representation needed to inspect its decisions.
%
% INPUTS
%   - shape (scalar polyshape)
%       Spatial obstacle representation used only for route proposals.
%   - start_units, goal_units (1-by-2 finite numeric rows)
%       Required endpoint positions in [x y] order.
%   - limits (scalar struct)
%       Workspace intervals in coordinate units.
%   - candidateOffset_units (positive finite scalar)
%       Outward obstacle-boundary offset for this attempt.
%   - offsetRetryCount (nonnegative integer scalar)
%       Zero-based index of this attempt in the offset schedule.
%   - workBudget (positive finite scalar)
%       Pair-edge work cap governing node count and exhaustive recovery.
%
% OUTPUTS
%   - attempt (scalar struct)
%       Raw and retained nodes, candidate pairs, accepted and rejected
%       edges, components, recovery steps, costs, and connectivity state.
%
% UNITS
%   - Positions, offsets, bounds, and graph costs are coordinate units.
%

%% Section 1: Bound The Candidate Nodes

% Convert the pair/edge work budget into a node limit.

[edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape, 1e-12);
candidateLimit = max(2, floor(sqrt(2 * workBudget / max(1, size(edgeStart_units, 1)))) - 2);
nodes          = createVisibilityNodes(shape, start_units, goal_units, limits, candidateOffset_units, candidateLimit);

%% Section 2: Create And Check Candidate Pairs

% Check proposed edges and record both accepted and rejected connections.

nodeCount      = size(nodes.Positions_units, 1);
pairMask       = triu(true(nodeCount), 1);
usedExhaustive = nodeCount < 4;
if nodeCount >= 4
    triangulation = delaunayTriangulation(nodes.Positions_units);
    pairs         = sort(edges(triangulation), 2);
    pairMask      = false(nodeCount);
    pairMask(sub2ind([nodeCount nodeCount], pairs(:, 1), pairs(:, 2))) = true;
    pairMask(1:2, 3:end) = true;
    pairMask(1, 2) = true;
end
[firstNodeIndex, secondNodeIndex] = find(pairMask);
estimatedExhaustiveWork = nodeCount * (nodeCount - 1) / 2 * max(1, size(edgeStart_units, 1));
pairSet                 = struct("PairMask", pairMask, ...
    "CandidatePairs", [firstNodeIndex secondNodeIndex], ...
    "EstimatedExhaustiveWork", estimatedExhaustiveWork, ...
    "UsedExhaustive", usedExhaustive, ...
    "WorkBudget", workBudget);
edgeCheck = evaluateVisibilityPairs(nodes.Positions_units, pairSet.PairMask, shape, edgeStart_units, edgeEnd_units);

%% Section 3: Recover Missing Connectivity

% Try boundary edges and affordable all-pairs edges to connect the graph.

recovery = recoverVisibilityConnectivity(nodes, pairSet, edgeCheck, shape, edgeStart_units, edgeEnd_units);

%% Section 4: Assemble The Attempt

attempt = struct("OffsetRetryCount", offsetRetryCount, ...
    "CandidateOffset_units", candidateOffset_units, ...
    "Nodes", nodes, ...
    "InitialPairSet", pairSet, ...
    "FinalCandidatePairs", recovery.EdgeCheck.CandidatePairs, ...
    "AcceptedEdges_units", recovery.EdgeCheck.AcceptedEdges_units, ...
    "RejectedEdges_units", recovery.EdgeCheck.RejectedEdges_units, ...
    "EdgeRejectionReasons", recovery.EdgeCheck.RejectionReasons, ...
    "GraphComponents", recovery.Components, ...
    "RecoverySteps", recovery.RecoverySteps, ...
    "ExhaustiveVisibilityUsed", recovery.UsedExhaustive, ...
    "ExhaustiveVisibilityFallbackUsed", ...
    recovery.UsedExhaustiveFallback, ...
    "IsConnected", recovery.IsConnected, ...
    "Cost_units", recovery.EdgeCheck.Cost_units, ...
    "VisibilityEdgeCount", recovery.EdgeCheck.AcceptedCount, ...
    "RejectedTransitionCount", recovery.EdgeCheck.RejectedCount, ...
    "VisibilityCandidatePairCount", nnz(recovery.PairMask), ...
    "EstimatedExhaustiveVisibilityWork", ...
    pairSet.EstimatedExhaustiveWork);
end

%% Section 5: Local Functions

function nodes = createVisibilityNodes(shape, start_units, goal_units, limits, candidateOffset_units, candidateLimit)
    % Create offset-boundary and workspace nodes within the node budget.
    candidateShape = shape;
    if ~isempty(shape.Vertices)
        candidateShape = polybuffer(shape, candidateOffset_units, "JointType", "miter");
    end
    rawNodes_units      = candidateShape.Vertices;
    isInsideWorkspace = rawNodes_units(:, 1) >= limits.xInterval_units(1) & rawNodes_units(:, 1) <= limits.xInterval_units(2) & rawNodes_units(:, 2) >= limits.yInterval_units(1) & rawNodes_units(:, 2) <= limits.yInterval_units(2);
    discardReasons    = repmat("", size(rawNodes_units, 1), 1);
    discardReasons(~isInsideWorkspace) = "outsideWorkspace";
    candidateNodes_units = unique(rawNodes_units(isInsideWorkspace, :), "rows", "stable");
    % Rank and truncate excess visibility nodes so the bounded search keeps the most relevant candidates.
    if size(candidateNodes_units, 1) > candidateLimit
        candidateNodes_units = selectVisibilityCandidates(candidateNodes_units, start_units, goal_units, candidateLimit);
    end
    positions_units = unique([start_units; goal_units; candidateNodes_units], "rows", "stable");
    nodes         = struct("CandidateShape", candidateShape, ...
        "RawNodes_units", rawNodes_units, ...
        "RawNodeDiscardReasons", discardReasons, ...
        "RetainedCandidateNodes_units", candidateNodes_units, ...
        "Positions_units", positions_units, ...
        "CandidateLimit", candidateLimit, ...
        "CandidateOffset_units", candidateOffset_units);
end

function selected_units = selectVisibilityCandidates(candidates_units, start_units, goal_units, count)
    % Retain global supports, endpoint access, and boundary coverage.
    endpointCount  = min(4, floor(count / 6));
    directionCount = min(16, floor(count / 3));
    uniformCount   = count - directionCount - 2 * endpointCount;
    selected       = unique(round(linspace(1, size(candidates_units, 1), uniformCount))).';
    if directionCount > 0
        angle_rad = (0:directionCount - 1).' * (2 * pi / directionCount);
        direction = [cos(angle_rad), sin(angle_rad)];
        [~, support] = max(candidates_units * direction.', [], 1);
        selected = [selected; support(:)];
    end
    % Process each reference units needed to complete select visibility candidates.
    for reference_units = [start_units; goal_units].'
        [~, order] = sort(vecnorm(candidates_units - reference_units.', 2, 2));
        selected = [selected; order(1:endpointCount)]; %#ok<AGROW>
    end
    selected     = unique(selected, "stable");
    selected_units = candidates_units(selected(1:min(count, numel(selected))), :);
end

function recovery = recoverVisibilityConnectivity(nodes, pairSet, edgeCheck, shape, edgeStart_units, edgeEnd_units)
    % Add boundary edges or all-pairs edges when the budget allows.
    positions_units  = nodes.Positions_units;
    nodeCount      = size(positions_units, 1);
    pairMask       = pairSet.PairMask;
    usedExhaustive = pairSet.UsedExhaustive;
    recoverySteps  = strings(0, 1);
    component      = conncomp(graph(isfinite(edgeCheck.Cost_units), "upper"));

    % Delaunay selection can omit consecutive offset-boundary vertices.
    if component(1) ~= component(2) && nodeCount >= 4
        [boundaryStart_units, boundaryEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(nodes.CandidateShape, 1e-12);
        [startFound, startIndex]             = ismember(boundaryStart_units, positions_units, "rows");
        [endFound, endIndex]                 = ismember(boundaryEnd_units, positions_units, "rows");
        boundaryPairs     = sort([startIndex(startFound & endFound), endIndex(startFound & endFound)], 2);
        boundaryPairs     = boundaryPairs(boundaryPairs(:, 1) ~= boundaryPairs(:, 2), :);
        augmentedPairMask = pairMask;
        augmentedPairMask(sub2ind([nodeCount nodeCount], boundaryPairs(:, 1), boundaryPairs(:, 2))) = true;
        if nnz(augmentedPairMask) > nnz(pairMask)
            pairMask  = augmentedPairMask;
            edgeCheck = evaluateVisibilityPairs(positions_units, pairMask, shape, edgeStart_units, edgeEnd_units);
            component = conncomp(graph(isfinite(edgeCheck.Cost_units), "upper"));
            recoverySteps(end + 1, 1) = "addedBoundaryPairs";
        end
    end

    % Use all-pairs visibility only within the work budget.
    usedFallback = component(1) ~= component(2) && ~usedExhaustive && pairSet.EstimatedExhaustiveWork <= pairSet.WorkBudget;
    % Record that fallback geometry supplied the proposal so downstream diagnostics can distinguish it from the primary method.
    if usedFallback
        pairMask       = triu(true(nodeCount), 1);
        edgeCheck      = evaluateVisibilityPairs(positions_units, pairMask, shape, edgeStart_units, edgeEnd_units);
        usedExhaustive = true;
        component      = conncomp(graph(isfinite(edgeCheck.Cost_units), "upper"));
        recoverySteps(end + 1, 1) = "usedExhaustivePairs";
    end
    recovery = struct("PairMask", pairMask, ...
        "EdgeCheck", edgeCheck, ...
        "Components", component, ...
        "RecoverySteps", recoverySteps, ...
        "UsedExhaustive", usedExhaustive, ...
        "UsedExhaustiveFallback", usedFallback, ...
        "IsConnected", component(1) == component(2));
end

function edgeCheck = evaluateVisibilityPairs(positions_units, pairMask, shape, edgeStart_units, edgeEnd_units)
    % Check graph edges and record the results.
    [firstNodeIndex, secondNodeIndex] = find(pairMask);
    first_units    = positions_units(firstNodeIndex, :);
    second_units   = positions_units(secondNodeIndex, :);
    isVisible    = obstacleAvoidance.search.checkVisibilitySegments(first_units, second_units, shape, edgeStart_units, edgeEnd_units);
    distance_units = vecnorm(second_units - first_units, 2, 2);
    nodeCount    = size(positions_units, 1);
    cost_units     = Inf(nodeCount);
    cost_units(1:nodeCount + 1:end) = 0;
    linearIndex = sub2ind([nodeCount nodeCount], firstNodeIndex(isVisible), secondNodeIndex(isVisible));
    cost_units(linearIndex) = distance_units(isVisible);
    cost_units          = min(cost_units, cost_units.');
    acceptedCount     = nnz(isVisible);
    rejectedCount     = nnz(~isVisible);
    acceptedEdges_units = [first_units(isVisible, :), second_units(isVisible, :)];
    rejectedEdges_units = [first_units(~isVisible, :), second_units(~isVisible, :)];
    acceptedEdges_units = acceptedEdges_units(1:min(2000, acceptedCount), :);
    rejectedEdges_units = rejectedEdges_units(1:min(2000, rejectedCount), :);
    rejectionReasons  = repmat("blockedByProposalGeometry", rejectedCount, 1);
    rejectionReasons  = rejectionReasons(1:min(2000, rejectedCount));
    edgeCheck         = struct("Cost_units", cost_units, ...
        "CandidatePairs", [firstNodeIndex secondNodeIndex], ...
        "IsVisible", isVisible, ...
        "AcceptedEdges_units", acceptedEdges_units, ...
        "RejectedEdges_units", rejectedEdges_units, ...
        "RejectionReasons", rejectionReasons, ...
        "AcceptedCount", acceptedCount, ...
        "RejectedCount", rejectedCount);
end
