function attempt = createVisibilityAttempt(shape, start_deg, goal_deg, limits, candidateOffset_deg, offsetRetryCount, workBudget)
%% Section 0: Header & Readme
% SYNTAX
%   attempt = obstacleAvoidance.search.createVisibilityAttempt( ...
%       shape, start_deg, goal_deg, limits, candidateOffset_deg, ...
%       offsetRetryCount, workBudget)
%
% PURPOSE
%   - Create, check, and recover one offset visibility-graph attempt.
%   - Return every representation needed to inspect its decisions.
%
% INPUTS
%   - shape (scalar polyshape)
%       Spatial obstacle representation used only for route proposals.
%   - start_deg, goal_deg (1-by-2 finite numeric rows)
%       Required endpoint positions in [azimuth elevation] order.
%   - limits (scalar struct)
%       Workspace intervals in degrees.
%   - candidateOffset_deg (positive finite scalar)
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
%   - Positions, offsets, bounds, and graph costs are degrees.
%

%% Section 1: Bound The Candidate Nodes

% Convert the pair/edge work budget into a node limit.

[edgeStart_deg, edgeEnd_deg] = obstacleAvoidance.geometry.boundaryToEdges(shape, 1e-12);
candidateLimit = max(2, floor(sqrt(2 * workBudget / max(1, size(edgeStart_deg, 1)))) - 2);
nodes          = createVisibilityNodes(shape, start_deg, goal_deg, limits, candidateOffset_deg, candidateLimit);

%% Section 2: Create And Check Candidate Pairs

% Check proposed edges and record both accepted and rejected connections.

nodeCount      = size(nodes.Positions_deg, 1);
pairMask       = triu(true(nodeCount), 1);
usedExhaustive = nodeCount < 4;
if nodeCount >= 4
    triangulation = delaunayTriangulation(nodes.Positions_deg);
    pairs         = sort(edges(triangulation), 2);
    pairMask      = false(nodeCount);
    pairMask(sub2ind([nodeCount nodeCount], pairs(:, 1), pairs(:, 2))) = true;
    pairMask(1:2, 3:end) = true;
    pairMask(1, 2) = true;
end
[firstNodeIndex, secondNodeIndex] = find(pairMask);
estimatedExhaustiveWork = nodeCount * (nodeCount - 1) / 2 * max(1, size(edgeStart_deg, 1));
pairSet                 = struct("PairMask", pairMask, ...
    "CandidatePairs", [firstNodeIndex secondNodeIndex], ...
    "EstimatedExhaustiveWork", estimatedExhaustiveWork, ...
    "UsedExhaustive", usedExhaustive, ...
    "WorkBudget", workBudget);
edgeCheck = evaluateVisibilityPairs(nodes.Positions_deg, pairSet.PairMask, shape, edgeStart_deg, edgeEnd_deg);

%% Section 3: Recover Missing Connectivity

% Try boundary edges and affordable all-pairs edges to connect the graph.

recovery = recoverVisibilityConnectivity(nodes, pairSet, edgeCheck, shape, edgeStart_deg, edgeEnd_deg);

%% Section 4: Assemble The Attempt

attempt = struct("OffsetRetryCount", offsetRetryCount, ...
    "CandidateOffset_deg", candidateOffset_deg, ...
    "Nodes", nodes, ...
    "InitialPairSet", pairSet, ...
    "FinalCandidatePairs", recovery.EdgeCheck.CandidatePairs, ...
    "AcceptedEdges_deg", recovery.EdgeCheck.AcceptedEdges_deg, ...
    "RejectedEdges_deg", recovery.EdgeCheck.RejectedEdges_deg, ...
    "EdgeRejectionReasons", recovery.EdgeCheck.RejectionReasons, ...
    "GraphComponents", recovery.Components, ...
    "RecoverySteps", recovery.RecoverySteps, ...
    "ExhaustiveVisibilityUsed", recovery.UsedExhaustive, ...
    "ExhaustiveVisibilityFallbackUsed", ...
    recovery.UsedExhaustiveFallback, ...
    "IsConnected", recovery.IsConnected, ...
    "Cost_deg", recovery.EdgeCheck.Cost_deg, ...
    "VisibilityEdgeCount", recovery.EdgeCheck.AcceptedCount, ...
    "RejectedTransitionCount", recovery.EdgeCheck.RejectedCount, ...
    "VisibilityCandidatePairCount", nnz(recovery.PairMask), ...
    "EstimatedExhaustiveVisibilityWork", ...
    pairSet.EstimatedExhaustiveWork);
end

%% Section 5: Local Functions

function nodes = createVisibilityNodes(shape, start_deg, goal_deg, limits, candidateOffset_deg, candidateLimit)
    % Create offset-boundary and workspace nodes within the node budget.
    candidateShape = shape;
    if ~isempty(shape.Vertices)
        candidateShape = polybuffer(shape, candidateOffset_deg, "JointType", "miter");
    end
    rawNodes_deg      = candidateShape.Vertices;
    isInsideWorkspace = rawNodes_deg(:, 1) >= limits.azimuthInterval_deg(1) & rawNodes_deg(:, 1) <= limits.azimuthInterval_deg(2) & rawNodes_deg(:, 2) >= limits.elevationInterval_deg(1) & rawNodes_deg(:, 2) <= limits.elevationInterval_deg(2);
    discardReasons    = repmat("", size(rawNodes_deg, 1), 1);
    discardReasons(~isInsideWorkspace) = "outsideWorkspace";
    candidateNodes_deg = unique(rawNodes_deg(isInsideWorkspace, :), "rows", "stable");
    % Rank and truncate excess visibility nodes so the bounded search keeps the most relevant candidates.
    if size(candidateNodes_deg, 1) > candidateLimit
        candidateNodes_deg = selectVisibilityCandidates(candidateNodes_deg, start_deg, goal_deg, candidateLimit);
    end
    positions_deg = unique([start_deg; goal_deg; candidateNodes_deg], "rows", "stable");
    nodes         = struct("CandidateShape", candidateShape, ...
        "RawNodes_deg", rawNodes_deg, ...
        "RawNodeDiscardReasons", discardReasons, ...
        "RetainedCandidateNodes_deg", candidateNodes_deg, ...
        "Positions_deg", positions_deg, ...
        "CandidateLimit", candidateLimit, ...
        "CandidateOffset_deg", candidateOffset_deg);
end

function selected_deg = selectVisibilityCandidates(candidates_deg, start_deg, goal_deg, count)
    % Retain global supports, endpoint access, and boundary coverage.
    endpointCount  = min(4, floor(count / 6));
    directionCount = min(16, floor(count / 3));
    uniformCount   = count - directionCount - 2 * endpointCount;
    selected       = unique(round(linspace(1, size(candidates_deg, 1), uniformCount))).';
    if directionCount > 0
        angle_rad = (0:directionCount - 1).' * (2 * pi / directionCount);
        direction = [cos(angle_rad), sin(angle_rad)];
        [~, support] = max(candidates_deg * direction.', [], 1);
        selected = [selected; support(:)];
    end
    % Process each reference deg needed to complete select visibility candidates.
    for reference_deg = [start_deg; goal_deg].'
        [~, order] = sort(vecnorm(candidates_deg - reference_deg.', 2, 2));
        selected = [selected; order(1:endpointCount)]; %#ok<AGROW>
    end
    selected     = unique(selected, "stable");
    selected_deg = candidates_deg(selected(1:min(count, numel(selected))), :);
end

function recovery = recoverVisibilityConnectivity(nodes, pairSet, edgeCheck, shape, edgeStart_deg, edgeEnd_deg)
    % Add boundary edges or all-pairs edges when the budget allows.
    positions_deg  = nodes.Positions_deg;
    nodeCount      = size(positions_deg, 1);
    pairMask       = pairSet.PairMask;
    usedExhaustive = pairSet.UsedExhaustive;
    recoverySteps  = strings(0, 1);
    component      = conncomp(graph(isfinite(edgeCheck.Cost_deg), "upper"));

    % Delaunay selection can omit consecutive offset-boundary vertices.
    if component(1) ~= component(2) && nodeCount >= 4
        [boundaryStart_deg, boundaryEnd_deg] = obstacleAvoidance.geometry.boundaryToEdges(nodes.CandidateShape, 1e-12);
        [startFound, startIndex]             = ismember(boundaryStart_deg, positions_deg, "rows");
        [endFound, endIndex]                 = ismember(boundaryEnd_deg, positions_deg, "rows");
        boundaryPairs     = sort([startIndex(startFound & endFound), endIndex(startFound & endFound)], 2);
        boundaryPairs     = boundaryPairs(boundaryPairs(:, 1) ~= boundaryPairs(:, 2), :);
        augmentedPairMask = pairMask;
        augmentedPairMask(sub2ind([nodeCount nodeCount], boundaryPairs(:, 1), boundaryPairs(:, 2))) = true;
        if nnz(augmentedPairMask) > nnz(pairMask)
            pairMask  = augmentedPairMask;
            edgeCheck = evaluateVisibilityPairs(positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg);
            component = conncomp(graph(isfinite(edgeCheck.Cost_deg), "upper"));
            recoverySteps(end + 1, 1) = "addedBoundaryPairs";
        end
    end

    % Use all-pairs visibility only within the work budget.
    usedFallback = component(1) ~= component(2) && ~usedExhaustive && pairSet.EstimatedExhaustiveWork <= pairSet.WorkBudget;
    % Record that fallback geometry supplied the proposal so downstream diagnostics can distinguish it from the primary method.
    if usedFallback
        pairMask       = triu(true(nodeCount), 1);
        edgeCheck      = evaluateVisibilityPairs(positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg);
        usedExhaustive = true;
        component      = conncomp(graph(isfinite(edgeCheck.Cost_deg), "upper"));
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

function edgeCheck = evaluateVisibilityPairs(positions_deg, pairMask, shape, edgeStart_deg, edgeEnd_deg)
    % Check graph edges and record the results.
    [firstNodeIndex, secondNodeIndex] = find(pairMask);
    first_deg    = positions_deg(firstNodeIndex, :);
    second_deg   = positions_deg(secondNodeIndex, :);
    isVisible    = obstacleAvoidance.search.checkVisibilitySegments(first_deg, second_deg, shape, edgeStart_deg, edgeEnd_deg);
    distance_deg = vecnorm(second_deg - first_deg, 2, 2);
    nodeCount    = size(positions_deg, 1);
    cost_deg     = Inf(nodeCount);
    cost_deg(1:nodeCount + 1:end) = 0;
    linearIndex = sub2ind([nodeCount nodeCount], firstNodeIndex(isVisible), secondNodeIndex(isVisible));
    cost_deg(linearIndex) = distance_deg(isVisible);
    cost_deg          = min(cost_deg, cost_deg.');
    acceptedCount     = nnz(isVisible);
    rejectedCount     = nnz(~isVisible);
    acceptedEdges_deg = [first_deg(isVisible, :), second_deg(isVisible, :)];
    rejectedEdges_deg = [first_deg(~isVisible, :), second_deg(~isVisible, :)];
    acceptedEdges_deg = acceptedEdges_deg(1:min(2000, acceptedCount), :);
    rejectedEdges_deg = rejectedEdges_deg(1:min(2000, rejectedCount), :);
    rejectionReasons  = repmat("blockedByProposalGeometry", rejectedCount, 1);
    rejectionReasons  = rejectionReasons(1:min(2000, rejectedCount));
    edgeCheck         = struct("Cost_deg", cost_deg, ...
        "CandidatePairs", [firstNodeIndex secondNodeIndex], ...
        "IsVisible", isVisible, ...
        "AcceptedEdges_deg", acceptedEdges_deg, ...
        "RejectedEdges_deg", rejectedEdges_deg, ...
        "RejectionReasons", rejectionReasons, ...
        "AcceptedCount", acceptedCount, ...
        "RejectedCount", rejectedCount);
end
