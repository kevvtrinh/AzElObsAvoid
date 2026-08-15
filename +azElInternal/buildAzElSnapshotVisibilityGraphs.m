function [graphs, parallelExecution] = ...
        buildAzElSnapshotVisibilityGraphs(obstacleField, selection, options)
%% Section 0: Header & Readme
% SYNTAX
%   [graphs, parallelExecution] = ...
%       azElInternal.buildAzElSnapshotVisibilityGraphs( ...
%       obstacleField, selection, options)
%**************************************************************************
% PURPOSE
%   - Build, collision-check, and solve one visibility graph at every
%     retained obstacle snapshot.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%       Complete protected obstacle history.
%   - selection (scalar struct)
%       Output from azElInternal.selectAzElVisibilitySnapshots.
%   - options (scalar struct)
%       Resolved visibility resolution and parallel-execution controls.
%**************************************************************************
% OUTPUTS
%   - graphs (structure array)
%       One stable visibility graph and shortest route per retained time.
%   - parallelExecution (scalar struct)
%       Requested mode, availability, worker count, and fallback evidence.
%**************************************************************************
% UNITS
%   - Graph positions are degrees and snapshot times are seconds.
%**************************************************************************

%% Section 1: Build Snapshot Graphs

[graphs, parallelExecution] = buildSnapshotVisibilityGraphs( ...
    obstacleField, selection.CandidatePointsAzElTime, ...
    selection.CandidateTypes, selection.CandidateObstacleIndex, ...
    selection.CandidateSampleIndex, selection.CandidateRegionIndex, ...
    selection.CandidateBoundaryGeometry, selection.StartPoint(1:2), ...
    selection.GoalPoint(1:2), selection.RetainedSnapshotTimes_s, options);
end

%% Section 2: Local Functions

function [useParallel, diagnostics] = resolveParallelExecution( ...
        requestedMode, taskCount)
%% Section 0: Header & Readme
% SYNTAX
%   [useParallel, diagnostics] = resolveParallelExecution( ...
%       requestedMode, taskCount)
%**************************************************************************
% PURPOSE
%   - Start or reuse a pool for independent visibility-reduction tasks.
%   - Return a documented serial fallback when parallel work is unavailable.
%**************************************************************************
% INPUTS
%   - requestedMode (scalar string)
%       Normalized auto, on, or off selection.
%   - taskCount (nonnegative integer scalar)
%**************************************************************************
% OUTPUTS
%   - useParallel (logical scalar)
%   - diagnostics (scalar struct)
%       Requested mode, availability, execution mode, and worker count.
%**************************************************************************
% UNITS
%   - Task and worker counts are dimensionless.
%**************************************************************************
validateattributes(taskCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});

toolboxAvailable = ...
    license("test", "Distrib_Computing_Toolbox") && ...
    exist("parpool", "file") == 2 && exist("gcp", "file") == 2;
useParallel = false;
workerCount = 0;

if taskCount <= 1
    message = "One or fewer reduction tasks; serial execution selected.";
elseif requestedMode == "off"
    message = "UseParallel is off; serial polygon reduction selected.";
elseif ~toolboxAvailable
    message = "Parallel Computing Toolbox is unavailable; serial fallback selected.";
    if requestedMode == "on"
        warning("buildAzElVisibilityRoutes:ParallelUnavailable", ...
            "%s", message);
    end
else
    try
        pool = gcp("nocreate");
        if isempty(pool)
            pool = parpool;
        end
        workerCount = pool.NumWorkers;
        useParallel = workerCount > 0;
        message = sprintf( ...
            "Parallel polygon reduction enabled with %d workers.", ...
            workerCount);
    catch parallelException
        message = "Parallel pool startup failed; serial fallback selected. " + ...
            string(parallelException.message);
        if requestedMode == "on"
            warning("buildAzElVisibilityRoutes:ParallelUnavailable", ...
                "%s", message);
        end
    end
end

diagnostics = struct( ...
    "RequestedMode", requestedMode, ...
    "ToolboxAvailable", toolboxAvailable, ...
    "Enabled", useParallel, ...
    "WorkerCount", workerCount, ...
    "TaskCount", taskCount, ...
    "Message", string(message));
end

function [graphs, parallelExecution] = buildSnapshotVisibilityGraphs( ...
        obstacleField, candidatePoints, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, snapshotTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [graphs, parallelExecution] = buildSnapshotVisibilityGraphs( ...
%       obstacleField, candidatePoints, candidateTypes, ...
%       candidateObstacleIndex, candidateSampleIndex, ...
%       candidateRegionIndex, candidateBoundaryGeometry, ...
%       startPosition_deg, goalPosition_deg, snapshotTimes_s, options)
%**************************************************************************
% PURPOSE
%   - Build one all-obstacle graph at each retained slice time.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%   - candidatePoints (N-by-3 numeric), candidateTypes (N-by-1 string)
%   - candidateObstacleIndex, candidateSampleIndex, candidateRegionIndex
%       N-by-1 provenance vectors; candidateBoundaryGeometry is N-by-1.
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%   - snapshotTimes_s (K-by-1 numeric), options (scalar struct)
%       Retained times, including slices without boundary candidates.
%**************************************************************************
% OUTPUTS
%   - graphs (structure array)
%       One stable visibility-graph record per retained time.
%   - parallelExecution (scalar struct)
%       Requested mode, availability, worker count, and fallback message.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
graphs = repmat(emptyVisibilityGraph(), 0, 1);
snapshotTimes_s = unique(snapshotTimes_s(:));
if isempty(snapshotTimes_s)
    [~, parallelExecution] = resolveParallelExecution( ...
        options.UseParallel, 0);
    return;
end
snapshotCount = numel(snapshotTimes_s);
[useParallel, parallelExecution] = resolveParallelExecution( ...
    options.UseParallel, snapshotCount);
graphs = repmat(emptyVisibilityGraph(), snapshotCount, 1);
if useParallel
    parfor snapshotIndex = 1:snapshotCount
        graphs(snapshotIndex) = buildOneSnapshotVisibilityGraph( ...
            obstacleField, candidatePoints, candidateTypes, ...
            candidateObstacleIndex, candidateSampleIndex, ...
            candidateRegionIndex, candidateBoundaryGeometry, ...
            startPosition_deg, goalPosition_deg, options, ...
            snapshotTimes_s(snapshotIndex));
    end
else
    for snapshotIndex = 1:snapshotCount
        graphs(snapshotIndex) = buildOneSnapshotVisibilityGraph( ...
            obstacleField, candidatePoints, candidateTypes, ...
            candidateObstacleIndex, candidateSampleIndex, ...
            candidateRegionIndex, candidateBoundaryGeometry, ...
            startPosition_deg, goalPosition_deg, options, ...
            snapshotTimes_s(snapshotIndex));
    end
end

% Worker output is intentionally suppressed. Printing after the loop keeps
% diagnostics deterministic even when parfor completes tasks out of order.
if options.Verbose
    fprintf("[visibility graph] %s\n", parallelExecution.Message);
    for snapshotIndex = 1:snapshotCount
        fprintf("[visibility graph] %d/%d at t=%.3f s: " + ...
            "%d active candidates, success=%d.\n", ...
            snapshotIndex, snapshotCount, ...
            snapshotTimes_s(snapshotIndex), ...
            nnz(graphs(snapshotIndex).CandidateActiveMask), ...
            graphs(snapshotIndex).Success);
    end
end
end

function graph = buildOneSnapshotVisibilityGraph( ...
        obstacleField, candidatePoints, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, options, snapshotTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   graph = buildOneSnapshotVisibilityGraph( ...
%       obstacleField, candidatePoints, candidateTypes, ...
%       candidateObstacleIndex, candidateSampleIndex, ...
%       candidateRegionIndex, candidateBoundaryGeometry, ...
%       startPosition_deg, goalPosition_deg, options, snapshotTime_s)
%**************************************************************************
% PURPOSE
%   - Select candidates at one retained time and solve their graph.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%   - candidatePoints (N-by-3 numeric), candidateTypes (N-by-1 string)
%   - candidateObstacleIndex, candidateSampleIndex, candidateRegionIndex
%       N-by-1 provenance vectors; candidateBoundaryGeometry is N-by-1.
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%   - options (scalar struct), snapshotTime_s (numeric scalar)
%**************************************************************************
% OUTPUTS
%   - graph (scalar struct)
%       Solved or failed snapshot graph with stable diagnostics.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
candidateIsOnSnapshot = false(size(candidatePoints, 1), 1);
for obstacleIndex = 1:obstacleField.ObstacleCount
    obstacle = obstacleField.Obstacles(obstacleIndex);
    obstacleTime_s = double(obstacle.TimeSeconds(:));
    if isempty(obstacleTime_s) || snapshotTime_s < obstacleTime_s(1) || ...
            snapshotTime_s > obstacleTime_s(end)
        continue;
    end
    belongsToObstacle = candidateObstacleIndex == obstacleIndex;
    availableCandidateTime_s = unique( ...
        candidatePoints(belongsToObstacle, 3));
    if isempty(availableCandidateTime_s)
        continue;
    end
    [~, nearestTimeIndex] = min(abs( ...
        availableCandidateTime_s - snapshotTime_s));
    selectedCandidateTime_s = ...
        availableCandidateTime_s(nearestTimeIndex);
    timeTolerance_s = 1e-10 * max(1, abs(selectedCandidateTime_s));
    candidateIsOnSnapshot = candidateIsOnSnapshot | ...
        (belongsToObstacle & abs(candidatePoints(:, 3) - ...
        selectedCandidateTime_s) <= timeTolerance_s);
end
globalCandidateIndex = find(candidateIsOnSnapshot);
graph = buildVisibilityGraphAtTime( ...
    obstacleField, candidatePoints(candidateIsOnSnapshot, 1:2), ...
    candidateTypes(candidateIsOnSnapshot), ...
    candidateObstacleIndex(candidateIsOnSnapshot), ...
    candidateSampleIndex(candidateIsOnSnapshot), ...
    candidateRegionIndex(candidateIsOnSnapshot), ...
    candidateBoundaryGeometry(candidateIsOnSnapshot), ...
    globalCandidateIndex, startPosition_deg, goalPosition_deg, ...
    snapshotTime_s, options);
end

function graph = emptyVisibilityGraph()
%% Section 0: Header & Readme
% SYNTAX
%   graph = emptyVisibilityGraph()
%**************************************************************************
% PURPOSE
%   - Define the stable schema for one snapshot visibility graph.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - graph (scalar struct)
%       Empty graph record used for success and failure paths.
%**************************************************************************
% UNITS
%   - Position and path cost fields are degrees; Time_s is seconds.
%**************************************************************************
graph = struct( ...
    "Success", false, ...
    "Message", "Visibility graph was not evaluated.", ...
    "Time_s", NaN, ...
    "NodePosition_deg", zeros(0, 2), ...
    "NodeType", strings(0, 1), ...
    "NodeCandidateIndex", zeros(0, 1), ...
    "NodeObstacleIndex", zeros(0, 1), ...
    "CandidateActiveMask", false(0, 1), ...
    "VisibilityTestedMask", false(0, 0), ...
    "VisibilityBlockedMask", false(0, 0), ...
    "EdgeCost_deg", zeros(0, 0), ...
    "EdgeType", strings(0, 0), ...
    "EdgeRoute_deg", {cell(0, 0)}, ...
    "PathNodeIndex", zeros(0, 1), ...
    "PathCandidateIndex", zeros(0, 1), ...
    "PathObstacleIndex", zeros(0, 1), ...
    "PathPosition_deg", zeros(0, 2), ...
    "PathEdgeType", strings(0, 1), ...
    "PathCost_deg", Inf, ...
    "PathEdgeCount", 0, ...
    "PathVisibilityEdgeCount", 0, ...
    "PathBoundaryEdgeCount", 0, ...
    "GeneratedVisibilityEdgeCount", 0, ...
    "GeneratedBoundaryEdgeCount", 0, ...
    "BoundaryRouteInputVertexCount", 0, ...
    "BoundaryRouteRetainedVertexCount", 0, ...
    "BoundaryRouteVertexReductionPercent", 0);
end

function graph = buildVisibilityGraphAtTime( ...
        obstacleField, candidatePosition_deg, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        globalCandidateIndex, ...
        startPosition_deg, goalPosition_deg, snapshotTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   graph = buildVisibilityGraphAtTime( ...
%       obstacleField, candidatePosition_deg, candidateTypes, ...
%       candidateObstacleIndex, candidateSampleIndex, ...
%       candidateRegionIndex, candidateBoundaryGeometry, ...
%       globalCandidateIndex, startPosition_deg, goalPosition_deg, ...
%       snapshotTime_s, options)
%**************************************************************************
% PURPOSE
%   - Connect visible nodes and compatible same-boundary neighbors.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%   - candidatePosition_deg (N-by-2), candidateTypes (N-by-1 string)
%   - candidateObstacleIndex, candidateSampleIndex, candidateRegionIndex,
%       globalCandidateIndex (N-by-1 provenance vectors)
%   - candidateBoundaryGeometry (N-by-1 cell array)
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%   - snapshotTime_s (numeric scalar), options (scalar struct)
%**************************************************************************
% OUTPUTS
%   - graph (scalar struct)
%       Graph matrices, selected route, and construction counts.
%**************************************************************************
% UNITS
%   - Position and edge costs are degrees; time is seconds.
%**************************************************************************
graph = emptyVisibilityGraph();
graph.Time_s = snapshotTime_s;
candidateCount = size(candidatePosition_deg, 1);
nodePosition_deg = [startPosition_deg; goalPosition_deg; ...
    candidatePosition_deg];
nodeCount = size(nodePosition_deg, 1);
nodeType = ["start"; "goal"; candidateTypes];
nodeCandidateIndex = [0; 0; globalCandidateIndex(:)];
nodeObstacleIndex = [0; 0; candidateObstacleIndex(:)];
edgeCost_deg = inf(nodeCount, nodeCount);
edgeType = strings(nodeCount, nodeCount);
edgeRoute_deg = cell(nodeCount, nodeCount);
visibilityTestedMask = false(nodeCount, nodeCount);
visibilityBlockedMask = false(nodeCount, nodeCount);
generatedVisibilityEdgeCount = 0;
generatedBoundaryEdgeCount = 0;
boundaryRouteInputVertexCount = 0;
boundaryRouteRetainedVertexCount = 0;

% Phase 1 is intentionally sparse: connect only start/goal to candidates,
% plus the direct edge. Together with cyclic boundary edges below, this
% produces a feasible upper-bound route before any candidate-pair clique is
% attempted.
endpointPair = [1 2];
for endpointPairIndex = 1:size(endpointPair, 1)
    firstNodeIndex = endpointPair(endpointPairIndex, 1);
    secondNodeIndex = endpointPair(endpointPairIndex, 2);
    [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded, ...
        edgeWasTested, edgeWasBlocked] = ...
        addVisibilityEdgeIfClear(edgeCost_deg, edgeType, edgeRoute_deg, ...
        nodePosition_deg, firstNodeIndex, secondNodeIndex, ...
        obstacleField, snapshotTime_s, options);
    [visibilityTestedMask, visibilityBlockedMask] = ...
        recordVisibilityTest(visibilityTestedMask, ...
        visibilityBlockedMask, firstNodeIndex, secondNodeIndex, ...
        edgeWasTested, edgeWasBlocked);
    generatedVisibilityEdgeCount = generatedVisibilityEdgeCount + ...
        edgeWasAdded;
end
for candidateIndex = 1:candidateCount
    candidateNodeIndex = candidateIndex + 2;
    for endpointNodeIndex = 1:2
        [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded, ...
            edgeWasTested, edgeWasBlocked] = ...
            addVisibilityEdgeIfClear( ...
            edgeCost_deg, edgeType, edgeRoute_deg, nodePosition_deg, ...
            endpointNodeIndex, candidateNodeIndex, obstacleField, ...
            snapshotTime_s, options);
        [visibilityTestedMask, visibilityBlockedMask] = ...
            recordVisibilityTest(visibilityTestedMask, ...
            visibilityBlockedMask, endpointNodeIndex, ...
            candidateNodeIndex, edgeWasTested, edgeWasBlocked);
        generatedVisibilityEdgeCount = generatedVisibilityEdgeCount + ...
            edgeWasAdded;
    end
end

% Candidates on one ring are sorted by boundary arclength. Connecting only
% cyclic neighbors preserves both sides of every obstacle without creating
% an artificial chord through its filled interior.
candidateMetadata = [candidateObstacleIndex(:), ...
    candidateSampleIndex(:), candidateRegionIndex(:)];
boundaryGroups = unique(candidateMetadata, "rows", "stable");
for boundaryGroupIndex = 1:size(boundaryGroups, 1)
    boundaryGroup = boundaryGroups(boundaryGroupIndex, :);
    candidateIsOnBoundary = all( ...
        candidateMetadata == boundaryGroup, 2);
    localCandidateIndex = find(candidateIsOnBoundary);
    if numel(localCandidateIndex) < 2
        continue;
    end
    if isempty(candidateBoundaryGeometry{localCandidateIndex(1)})
        continue;
    end
    region_deg = candidateBoundaryGeometry{localCandidateIndex(1)};
    candidateArc_deg = candidateBoundaryArcPositions( ...
        region_deg, candidatePosition_deg(localCandidateIndex, :));
    [~, boundaryOrder] = sort(candidateArc_deg);
    orderedCandidateIndex = localCandidateIndex(boundaryOrder);
    for orderedIndex = 1:numel(orderedCandidateIndex)
        nextOrderedIndex = mod( ...
            orderedIndex, numel(orderedCandidateIndex)) + 1;
        firstCandidateIndex = orderedCandidateIndex(orderedIndex);
        secondCandidateIndex = orderedCandidateIndex(nextOrderedIndex);
        firstNodeIndex = firstCandidateIndex + 2;
        secondNodeIndex = secondCandidateIndex + 2;
        [rawBoundaryRoute_deg, ~, ~] = ...
            forwardBoundaryRoute(region_deg, ...
            candidatePosition_deg(firstCandidateIndex, :), ...
            candidatePosition_deg(secondCandidateIndex, :));
        [boundaryRoute_deg, routeIsValid] = ...
            reduceBoundaryRouteForGraph(rawBoundaryRoute_deg, ...
            obstacleField, snapshotTime_s, options);
        if ~routeIsValid
            continue;
        end
        boundaryRouteInputVertexCount = ...
            boundaryRouteInputVertexCount + size(rawBoundaryRoute_deg, 1);
        boundaryRouteRetainedVertexCount = ...
            boundaryRouteRetainedVertexCount + size(boundaryRoute_deg, 1);
        boundaryStep_deg = diff(boundaryRoute_deg, 1, 1);
        boundaryDistance_deg = sum(hypot( ...
            boundaryStep_deg(:, 1), boundaryStep_deg(:, 2)));
        if boundaryDistance_deg >= ...
                edgeCost_deg(firstNodeIndex, secondNodeIndex) - 1e-10
            continue;
        end
        edgeCost_deg(firstNodeIndex, secondNodeIndex) = ...
            boundaryDistance_deg;
        edgeCost_deg(secondNodeIndex, firstNodeIndex) = ...
            boundaryDistance_deg;
        edgeType(firstNodeIndex, secondNodeIndex) = "boundary";
        edgeType(secondNodeIndex, firstNodeIndex) = "boundary";
        edgeRoute_deg{firstNodeIndex, secondNodeIndex} = ...
            boundaryRoute_deg;
        edgeRoute_deg{secondNodeIndex, firstNodeIndex} = ...
            flipud(boundaryRoute_deg);
        generatedBoundaryEdgeCount = generatedBoundaryEdgeCount + 1;
    end
end

% This public graph returns its shortest geometric route. The Euclidean
% start-plus-goal distance is therefore an admissible lower bound for this
% graph objective, and a collision-free baseline safely removes candidates
% that cannot improve it. This pruning does not certify the fastest timed
% route; that limitation is reported by the planner diagnostics.
[baselinePathNodeIndex, baselinePathCost_deg] = ...
    shortestVisibilityGraphPath(edgeCost_deg, 1, 2);
candidateActiveMask = true(candidateCount, 1);
if ~isempty(baselinePathNodeIndex) && isfinite(baselinePathCost_deg)
    startOffset_deg = candidatePosition_deg - startPosition_deg;
    goalOffset_deg = candidatePosition_deg - goalPosition_deg;
    candidateLowerBound_deg = hypot( ...
        startOffset_deg(:, 1), startOffset_deg(:, 2)) + hypot( ...
        goalOffset_deg(:, 1), goalOffset_deg(:, 2));
    costTolerance_deg = 1e-9 * max(1, baselinePathCost_deg);
    candidateActiveMask = candidateLowerBound_deg <= ...
        baselinePathCost_deg + costTolerance_deg;
    baselineCandidateNode = baselinePathNodeIndex( ...
        baselinePathNodeIndex > 2) - 2;
    candidateActiveMask(baselineCandidateNode) = true;
    inactiveNodeIndex = find(~candidateActiveMask) + 2;
    edgeCost_deg(inactiveNodeIndex, :) = Inf;
    edgeCost_deg(:, inactiveNodeIndex) = Inf;
    edgeType(inactiveNodeIndex, :) = "";
    edgeType(:, inactiveNodeIndex) = "";
    edgeRoute_deg(inactiveNodeIndex, :) = {[]};
    edgeRoute_deg(:, inactiveNodeIndex) = {[]};
end

% Phase 2 completes only the candidates that can improve this graph's
% geometric objective.
activeCandidateIndex = find(candidateActiveMask);
for firstActiveIndex = 1:numel(activeCandidateIndex) - 1
    firstCandidateIndex = activeCandidateIndex(firstActiveIndex);
    firstNodeIndex = firstCandidateIndex + 2;
    for secondActiveIndex = firstActiveIndex + 1: ...
            numel(activeCandidateIndex)
        secondCandidateIndex = activeCandidateIndex(secondActiveIndex);
        secondNodeIndex = secondCandidateIndex + 2;
        [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded, ...
            edgeWasTested, edgeWasBlocked] = ...
            addVisibilityEdgeIfClear( ...
            edgeCost_deg, edgeType, edgeRoute_deg, nodePosition_deg, ...
            firstNodeIndex, secondNodeIndex, obstacleField, ...
            snapshotTime_s, options);
        [visibilityTestedMask, visibilityBlockedMask] = ...
            recordVisibilityTest(visibilityTestedMask, ...
            visibilityBlockedMask, firstNodeIndex, secondNodeIndex, ...
            edgeWasTested, edgeWasBlocked);
        generatedVisibilityEdgeCount = generatedVisibilityEdgeCount + ...
            edgeWasAdded;
    end
end
generatedVisibilityEdgeCount = nnz(triu(edgeType == "visibility", 1));
generatedBoundaryEdgeCount = nnz(triu(edgeType == "boundary", 1));

[pathNodeIndex, pathCost_deg] = shortestVisibilityGraphPath( ...
    edgeCost_deg, 1, 2);
graph.NodePosition_deg = nodePosition_deg;
graph.NodeType = nodeType;
graph.NodeCandidateIndex = nodeCandidateIndex;
graph.NodeObstacleIndex = nodeObstacleIndex;
graph.VisibilityTestedMask = visibilityTestedMask;
graph.VisibilityBlockedMask = visibilityBlockedMask;
graph.EdgeCost_deg = edgeCost_deg;
graph.EdgeType = edgeType;
graph.EdgeRoute_deg = edgeRoute_deg;
graph.GeneratedVisibilityEdgeCount = generatedVisibilityEdgeCount;
graph.GeneratedBoundaryEdgeCount = generatedBoundaryEdgeCount;
graph.CandidateActiveMask = candidateActiveMask;
graph.BoundaryRouteInputVertexCount = boundaryRouteInputVertexCount;
graph.BoundaryRouteRetainedVertexCount = ...
    boundaryRouteRetainedVertexCount;
if boundaryRouteInputVertexCount > 0
    graph.BoundaryRouteVertexReductionPercent = 100 * max(0, ...
        1 - boundaryRouteRetainedVertexCount / ...
        boundaryRouteInputVertexCount);
end
if isempty(pathNodeIndex)
    graph.Message = "No path connects start to goal in this visibility graph.";
    return;
end

pathEdgeCount = numel(pathNodeIndex) - 1;
pathEdgeType = strings(pathEdgeCount, 1);
pathPosition_deg = zeros(0, 2);
for pathEdgeIndex = 1:pathEdgeCount
    firstNodeIndex = pathNodeIndex(pathEdgeIndex);
    secondNodeIndex = pathNodeIndex(pathEdgeIndex + 1);
    pathEdgeType(pathEdgeIndex) = edgeType( ...
        firstNodeIndex, secondNodeIndex);
    edgePath_deg = edgeRoute_deg{firstNodeIndex, secondNodeIndex};
    if isempty(pathPosition_deg)
        pathPosition_deg = edgePath_deg;
    else
        pathPosition_deg = [pathPosition_deg; ...
            edgePath_deg(2:end, :)]; %#ok<AGROW>
    end
end
graph.Success = true;
graph.Message = ...
    "Global visibility-graph Dijkstra connected start to goal.";
graph.PathNodeIndex = pathNodeIndex;
graph.PathCandidateIndex = nodeCandidateIndex(pathNodeIndex);
graph.PathObstacleIndex = nodeObstacleIndex(pathNodeIndex);
graph.PathPosition_deg = removeConsecutiveDuplicatePoints( ...
    pathPosition_deg);
graph.PathEdgeType = pathEdgeType;
graph.PathCost_deg = pathCost_deg;
graph.PathEdgeCount = pathEdgeCount;
graph.PathVisibilityEdgeCount = nnz(pathEdgeType == "visibility");
graph.PathBoundaryEdgeCount = nnz(pathEdgeType == "boundary");
end

function [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded, ...
        edgeWasTested, edgeWasBlocked] = ...
        addVisibilityEdgeIfClear(edgeCost_deg, edgeType, edgeRoute_deg, ...
        nodePosition_deg, firstNodeIndex, secondNodeIndex, ...
        obstacleField, snapshotTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeCost_deg, edgeType, edgeRoute_deg, edgeWasAdded, ...
%       edgeWasTested, edgeWasBlocked] = ...
%       addVisibilityEdgeIfClear(edgeCost_deg, edgeType, ...
%       edgeRoute_deg, nodePosition_deg, firstNodeIndex, ...
%       secondNodeIndex, obstacleField, snapshotTime_s, options)
%**************************************************************************
% PURPOSE
%   - Add a straight graph edge only when its open interior is clear.
%**************************************************************************
% INPUTS
%   - edgeCost_deg, edgeType, edgeRoute_deg (square graph matrices)
%   - nodePosition_deg (N-by-2 numeric matrix)
%   - firstNodeIndex, secondNodeIndex (positive integer scalars)
%   - obstacleField (scalar packed field), snapshotTime_s (numeric scalar)
%   - options (scalar resolved options struct)
%**************************************************************************
% OUTPUTS
%   - edgeCost_deg, edgeType, edgeRoute_deg (updated graph matrices)
%   - edgeWasAdded, edgeWasTested, edgeWasBlocked (scalar flags)
%       Distinguish a collision rejection from a pair skipped because an
%       existing boundary connection was already no more expensive.
%**************************************************************************
% UNITS
%   - Position and cost are degrees; time is seconds.
%**************************************************************************
edgeWasAdded = 0;
edgeWasTested = false;
edgeWasBlocked = false;
displacement_deg = nodePosition_deg(secondNodeIndex, :) - ...
    nodePosition_deg(firstNodeIndex, :);
distance_deg = hypot(displacement_deg(1), displacement_deg(2));
if distance_deg <= 1e-12 || distance_deg >= ...
        edgeCost_deg(firstNodeIndex, secondNodeIndex) - 1e-10
    return;
end
edgeWasTested = true;
if ~lineVisibleAtTime(obstacleField, ...
        nodePosition_deg(firstNodeIndex, :), ...
        nodePosition_deg(secondNodeIndex, :), snapshotTime_s, options)
    edgeWasBlocked = true;
    return;
end
edgeCost_deg(firstNodeIndex, secondNodeIndex) = distance_deg;
edgeCost_deg(secondNodeIndex, firstNodeIndex) = distance_deg;
edgeType(firstNodeIndex, secondNodeIndex) = "visibility";
edgeType(secondNodeIndex, firstNodeIndex) = "visibility";
edgeRoute_deg{firstNodeIndex, secondNodeIndex} = [ ...
    nodePosition_deg(firstNodeIndex, :); ...
    nodePosition_deg(secondNodeIndex, :)];
edgeRoute_deg{secondNodeIndex, firstNodeIndex} = [ ...
    nodePosition_deg(secondNodeIndex, :); ...
    nodePosition_deg(firstNodeIndex, :)];
edgeWasAdded = 1;
end

function [testedMask, blockedMask] = recordVisibilityTest( ...
        testedMask, blockedMask, firstNodeIndex, secondNodeIndex, ...
        edgeWasTested, edgeWasBlocked)
%% Section 0: Header & Readme
% SYNTAX
%   [testedMask, blockedMask] = recordVisibilityTest( ...
%       testedMask, blockedMask, firstNodeIndex, secondNodeIndex, ...
%       edgeWasTested, edgeWasBlocked)
%**************************************************************************
% PURPOSE
%   - Record one symmetric visibility-test outcome for diagnostics.
%**************************************************************************
% INPUTS
%   - testedMask, blockedMask (N-by-N logical matrices)
%   - firstNodeIndex, secondNodeIndex (positive integer scalars)
%   - edgeWasTested, edgeWasBlocked (logical scalars)
%**************************************************************************
% OUTPUTS
%   - testedMask, blockedMask (updated N-by-N logical matrices)
%**************************************************************************
% UNITS
%   - All values are dimensionless.
%**************************************************************************
if ~edgeWasTested
    return;
end
testedMask(firstNodeIndex, secondNodeIndex) = true;
testedMask(secondNodeIndex, firstNodeIndex) = true;
blockedMask(firstNodeIndex, secondNodeIndex) = edgeWasBlocked;
blockedMask(secondNodeIndex, firstNodeIndex) = edgeWasBlocked;
end

function candidateArc_deg = candidateBoundaryArcPositions( ...
        region_deg, candidatePosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   candidateArc_deg = candidateBoundaryArcPositions( ...
%       region_deg, candidatePosition_deg)
%**************************************************************************
% PURPOSE
%   - Project graph candidates onto one cyclic boundary coordinate.
%**************************************************************************
% INPUTS
%   - region_deg (N-by-2 numeric matrix)
%   - candidatePosition_deg (M-by-2 numeric matrix)
%**************************************************************************
% OUTPUTS
%   - candidateArc_deg (M-by-1 numeric vector)
%**************************************************************************
% UNITS
%   - Positions and boundary arc coordinates are degrees.
%**************************************************************************
if size(region_deg, 1) > 1 && hypot( ...
        region_deg(end, 1) - region_deg(1, 1), ...
        region_deg(end, 2) - region_deg(1, 2)) <= 1e-10
    region_deg(end, :) = [];
end
closedRegion_deg = [region_deg; region_deg(1, :)];
boundaryStep_deg = diff(closedRegion_deg, 1, 1);
edgeLength_deg = hypot( ...
    boundaryStep_deg(:, 1), boundaryStep_deg(:, 2));
perimeter_deg = sum(edgeLength_deg);
vertexArc_deg = [0; cumsum(edgeLength_deg(1:end - 1))];
candidateArc_deg = zeros(size(candidatePosition_deg, 1), 1);
for candidateIndex = 1:size(candidatePosition_deg, 1)
    candidateArc_deg(candidateIndex) = boundaryArcPosition( ...
        region_deg, edgeLength_deg, vertexArc_deg, ...
        candidatePosition_deg(candidateIndex, :), perimeter_deg);
end
end

function [pathNodeIndex, pathCost] = shortestVisibilityGraphPath( ...
        edgeCost, startNodeIndex, goalNodeIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [pathNodeIndex, pathCost] = shortestVisibilityGraphPath( ...
%       edgeCost, startNodeIndex, goalNodeIndex)
%**************************************************************************
% PURPOSE
%   - Run deterministic Dijkstra on a dense snapshot graph.
%**************************************************************************
% INPUTS
%   - edgeCost (N-by-N numeric matrix)
%       Symmetric nonnegative costs with Inf for absent edges.
%   - startNodeIndex, goalNodeIndex (positive integer scalars)
%**************************************************************************
% OUTPUTS
%   - pathNodeIndex (M-by-1 numeric vector)
%   - pathCost (nonnegative numeric scalar or Inf)
%**************************************************************************
% UNITS
%   - Path cost inherits the edge-cost units.
%**************************************************************************
nodeCount = size(edgeCost, 1);
costToReach = inf(nodeCount, 1);
hopCount = inf(nodeCount, 1);
parentNodeIndex = zeros(nodeCount, 1);
settled = false(nodeCount, 1);
costToReach(startNodeIndex) = 0;
hopCount(startNodeIndex) = 0;
for expansionIndex = 1:nodeCount
    unfinishedCost = costToReach;
    unfinishedCost(settled) = Inf;
    [currentCost, currentNodeIndex] = min(unfinishedCost);
    if ~isfinite(currentCost)
        break;
    end
    settled(currentNodeIndex) = true;
    if currentNodeIndex == goalNodeIndex
        break;
    end
    neighborNodeIndex = find(isfinite(edgeCost(currentNodeIndex, :)));
    for neighborIndex = reshape(neighborNodeIndex, 1, [])
        if settled(neighborIndex)
            continue;
        end
        trialCost = currentCost + edgeCost( ...
            currentNodeIndex, neighborIndex);
        trialHopCount = hopCount(currentNodeIndex) + 1;
        improvesCost = trialCost < costToReach(neighborIndex) - 1e-10;
        tiesCostWithFewerHops = abs( ...
            trialCost - costToReach(neighborIndex)) <= 1e-10 && ...
            trialHopCount < hopCount(neighborIndex);
        if improvesCost || tiesCostWithFewerHops
            costToReach(neighborIndex) = trialCost;
            hopCount(neighborIndex) = trialHopCount;
            parentNodeIndex(neighborIndex) = currentNodeIndex;
        end
    end
end
pathCost = costToReach(goalNodeIndex);
if ~isfinite(pathCost)
    pathNodeIndex = zeros(0, 1);
    return;
end
pathNodeIndex = goalNodeIndex;
while pathNodeIndex(1) ~= startNodeIndex
    parentIndex = parentNodeIndex(pathNodeIndex(1));
    if parentIndex == 0
        pathNodeIndex = zeros(0, 1);
        pathCost = Inf;
        return;
    end
    pathNodeIndex = [parentIndex; pathNodeIndex]; %#ok<AGROW>
end
end

function [reducedRoute_deg, success] = reduceBoundaryRouteForGraph( ...
        sourceRoute_deg, obstacleField, sampleTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [reducedRoute_deg, success] = reduceBoundaryRouteForGraph( ...
%       sourceRoute_deg, obstacleField, sampleTime_s, options)
%**************************************************************************
% PURPOSE
%   - Compact a boundary route without changing collision geometry.
%**************************************************************************
% INPUTS
%   - sourceRoute_deg (N-by-2 numeric matrix)
%   - obstacleField (scalar packed obstacle field)
%   - sampleTime_s (finite numeric scalar)
%   - options (scalar struct)
%**************************************************************************
% OUTPUTS
%   - reducedRoute_deg (M-by-2 numeric matrix)
%   - success (logical scalar)
%       True only when every reduced segment remains visible.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
% RDP proposes a compact shape, blocked chords restore source detail, and
% a final farthest-visible pass removes only redundant safe points.
sourceRoute_deg = removeConsecutiveDuplicatePoints(sourceRoute_deg);
sourceVertexCount = size(sourceRoute_deg, 1);
if sourceVertexCount <= 2 || ...
        options.BoundaryRouteReductionTolerance_deg <= 0
    reducedRoute_deg = sourceRoute_deg;
    success = routeSegmentsAreVisible( ...
        reducedRoute_deg, obstacleField, sampleTime_s, options);
    return;
end

retainedIndex = rdpPolylineIndices(sourceRoute_deg, ...
    options.BoundaryRouteReductionTolerance_deg);
retainedIndex = refineBlockedRouteSegments( ...
    sourceRoute_deg, retainedIndex, obstacleField, sampleTime_s, options);
proposedRoute_deg = sourceRoute_deg(retainedIndex, :);
if ~routeSegmentsAreVisible( ...
        proposedRoute_deg, obstacleField, sampleTime_s, options)
    reducedRoute_deg = sourceRoute_deg;
    success = routeSegmentsAreVisible( ...
        reducedRoute_deg, obstacleField, sampleTime_s, options);
    return;
end

% Greedily jump to the farthest collision-free retained point. This keeps
% true protrusions while removing points that do not constrain visibility.
reducedRoute_deg = proposedRoute_deg(1, :);
currentIndex = 1;
while currentIndex < size(proposedRoute_deg, 1)
    nextIndex = size(proposedRoute_deg, 1);
    while nextIndex > currentIndex + 1 && ~lineVisibleAtTime( ...
            obstacleField, proposedRoute_deg(currentIndex, :), ...
            proposedRoute_deg(nextIndex, :), sampleTime_s, options)
        nextIndex = nextIndex - 1;
    end
    reducedRoute_deg(end + 1, :) = ...
        proposedRoute_deg(nextIndex, :); %#ok<AGROW>
    currentIndex = nextIndex;
end
success = routeSegmentsAreVisible( ...
    reducedRoute_deg, obstacleField, sampleTime_s, options);
end

function retainedIndex = rdpPolylineIndices(position_deg, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   retainedIndex = rdpPolylineIndices(position_deg, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Return iterative Ramer-Douglas-Peucker retained indices.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 numeric matrix)
%   - tolerance_deg (nonnegative numeric scalar)
%**************************************************************************
% OUTPUTS
%   - retainedIndex (M-by-1 numeric vector)
%       Endpoint indices are always retained.
%**************************************************************************
% UNITS
%   - Position and tolerance are degrees.
%**************************************************************************
vertexCount = size(position_deg, 1);
isRetained = false(vertexCount, 1);
isRetained([1 end]) = true;
segmentStack = [1 vertexCount];
while ~isempty(segmentStack)
    firstIndex = segmentStack(end, 1);
    finalIndex = segmentStack(end, 2);
    segmentStack(end, :) = [];
    if finalIndex <= firstIndex + 1
        continue;
    end
    interiorIndex = (firstIndex + 1:finalIndex - 1).';
    distance_deg = pointToSegmentDistance( ...
        position_deg(interiorIndex, :), position_deg(firstIndex, :), ...
        position_deg(finalIndex, :));
    [maximumDistance_deg, localMaximumIndex] = max(distance_deg);
    if maximumDistance_deg <= tolerance_deg
        continue;
    end
    splitIndex = interiorIndex(localMaximumIndex);
    isRetained(splitIndex) = true;
    segmentStack = [segmentStack; ...
        firstIndex splitIndex; splitIndex finalIndex]; %#ok<AGROW>
end
retainedIndex = find(isRetained);
end

function distance_deg = pointToSegmentDistance( ...
        point_deg, firstPosition_deg, secondPosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   distance_deg = pointToSegmentDistance( ...
%       point_deg, firstPosition_deg, secondPosition_deg)
%**************************************************************************
% PURPOSE
%   - Measure point-to-closed-segment Euclidean distance.
%**************************************************************************
% INPUTS
%   - point_deg (N-by-2 numeric matrix)
%   - firstPosition_deg, secondPosition_deg (1-by-2 numeric rows)
%**************************************************************************
% OUTPUTS
%   - distance_deg (N-by-1 numeric vector)
%**************************************************************************
% UNITS
%   - Positions and distances are degrees.
%**************************************************************************
segment_deg = secondPosition_deg - firstPosition_deg;
segmentLengthSquared_deg2 = sum(segment_deg.^2);
if segmentLengthSquared_deg2 <= eps
    offset_deg = point_deg - firstPosition_deg;
    distance_deg = hypot(offset_deg(:, 1), offset_deg(:, 2));
    return;
end
projection = (point_deg - firstPosition_deg) * segment_deg.' ./ ...
    segmentLengthSquared_deg2;
projection = max(0, min(1, projection));
closestPoint_deg = firstPosition_deg + projection .* segment_deg;
offset_deg = point_deg - closestPoint_deg;
distance_deg = hypot(offset_deg(:, 1), offset_deg(:, 2));
end

function retainedIndex = refineBlockedRouteSegments( ...
        sourceRoute_deg, retainedIndex, obstacleField, sampleTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   retainedIndex = refineBlockedRouteSegments( ...
%       sourceRoute_deg, retainedIndex, obstacleField, ...
%       sampleTime_s, options)
%**************************************************************************
% PURPOSE
%   - Restore source vertices where a reduced chord is blocked.
%**************************************************************************
% INPUTS
%   - sourceRoute_deg (N-by-2 numeric matrix)
%   - retainedIndex (M-by-1 numeric vector)
%   - obstacleField (scalar packed obstacle field)
%   - sampleTime_s (finite numeric scalar)
%   - options (scalar struct)
%**************************************************************************
% OUTPUTS
%   - retainedIndex (K-by-1 numeric vector)
%       Refined stable indices into sourceRoute_deg.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
changed = true;
while changed
    changed = false;
    refinedIndex = retainedIndex(1);
    for segmentIndex = 1:numel(retainedIndex) - 1
        firstIndex = retainedIndex(segmentIndex);
        finalIndex = retainedIndex(segmentIndex + 1);
        isVisible = lineVisibleAtTime(obstacleField, ...
            sourceRoute_deg(firstIndex, :), ...
            sourceRoute_deg(finalIndex, :), sampleTime_s, options);
        if ~isVisible && finalIndex > firstIndex + 1
            interiorIndex = (firstIndex + 1:finalIndex - 1).';
            distance_deg = pointToSegmentDistance( ...
                sourceRoute_deg(interiorIndex, :), ...
                sourceRoute_deg(firstIndex, :), ...
                sourceRoute_deg(finalIndex, :));
            [~, localMaximumIndex] = max(distance_deg);
            splitIndex = interiorIndex(localMaximumIndex);
            if splitIndex <= firstIndex || splitIndex >= finalIndex
                splitIndex = floor((firstIndex + finalIndex) / 2);
            end
            refinedIndex(end + 1, 1) = splitIndex; %#ok<AGROW>
            changed = true;
        end
        refinedIndex(end + 1, 1) = finalIndex; %#ok<AGROW>
    end
    retainedIndex = unique(refinedIndex, "stable");
end
end

function visible = routeSegmentsAreVisible( ...
        route_deg, obstacleField, sampleTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   visible = routeSegmentsAreVisible( ...
%       route_deg, obstacleField, sampleTime_s, options)
%**************************************************************************
% PURPOSE
%   - Require every consecutive segment of a route to be visible.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric matrix)
%   - obstacleField (scalar packed obstacle field)
%   - sampleTime_s (finite numeric scalar)
%   - options (scalar struct)
%**************************************************************************
% OUTPUTS
%   - visible (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
visible = true;
for segmentIndex = 1:size(route_deg, 1) - 1
    if ~lineVisibleAtTime(obstacleField, ...
            route_deg(segmentIndex, :), ...
            route_deg(segmentIndex + 1, :), sampleTime_s, options)
        visible = false;
        return;
    end
end
end

function visible = lineVisibleAtTime( ...
        obstacleField, firstPosition_deg, secondPosition_deg, ...
        sampleTime_s, ~)
%% Section 0: Header & Readme
% SYNTAX
%   visible = lineVisibleAtTime(obstacleField, firstPosition_deg, ...
%       secondPosition_deg, sampleTime_s, options)
%**************************************************************************
% PURPOSE
%   - Test one open segment using exact topology intervals.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%   - firstPosition_deg, secondPosition_deg (1-by-2 numeric rows)
%   - sampleTime_s (finite numeric scalar)
%   - options (scalar struct)
%       Reserved for the shared visibility-call signature.
%**************************************************************************
% OUTPUTS
%   - visible (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
% Polygon occupancy is constant between consecutive boundary crossings, so
% one midpoint per interval is exact for the stored piecewise-linear field.
% Cost therefore scales with packed edges and actual crossings, not with
% segment length divided by VisibilitySampleStep_deg.
displacement_deg = secondPosition_deg - firstPosition_deg;
distance_deg = hypot(displacement_deg(1), displacement_deg(2));
if distance_deg <= 1e-12
    visible = true;
    return;
end
sampleFraction = visibilitySegmentIntervalMidpoints( ...
    obstacleField, firstPosition_deg, secondPosition_deg, sampleTime_s);
samplePosition_deg = firstPosition_deg + ...
    sampleFraction .* displacement_deg;
blocked = queryAzElTimeObstacle(obstacleField, ...
    samplePosition_deg(:, 1), samplePosition_deg(:, 2), ...
    repmat(sampleTime_s, numel(sampleFraction), 1), struct( ...
    "CollisionMode", "polygon", ...
    "TimePaddingSamples", 0, ...
    "BoundaryIsOccupied", false));
visible = ~any(blocked);
end

function intervalMidpoint = visibilitySegmentIntervalMidpoints( ...
        obstacleField, firstPosition_deg, secondPosition_deg, sampleTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   intervalMidpoint = visibilitySegmentIntervalMidpoints( ...
%       obstacleField, firstPosition_deg, secondPosition_deg, sampleTime_s)
%**************************************************************************
% PURPOSE
%   - Partition a segment at every packed-slice boundary crossing.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%   - firstPosition_deg, secondPosition_deg (1-by-2 numeric rows)
%   - sampleTime_s (finite numeric scalar)
%**************************************************************************
% OUTPUTS
%   - intervalMidpoint (M-by-1 numeric vector)
%       Fractions in [0, 1], one per constant-occupancy interval.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; fractions are dimensionless.
%**************************************************************************
breakpoint = [0; 1];
segmentMinimum_deg = min(firstPosition_deg, secondPosition_deg);
segmentMaximum_deg = max(firstPosition_deg, secondPosition_deg);
for obstacleIndex = 1:numel(obstacleField.Obstacles)
    obstacle = obstacleField.Obstacles(obstacleIndex);
    if obstacle.SampleCount < 1 || sampleTime_s < ...
            double(obstacle.TimeSeconds(1)) || sampleTime_s > ...
            double(obstacle.TimeSeconds(end))
        continue;
    end
    [~, sampleIndex] = min(abs( ...
        double(obstacle.TimeSeconds(:)) - sampleTime_s));
    bounds_deg = double(obstacle.BoundsDeg(sampleIndex, :));
    if any(~isfinite(bounds_deg)) || ...
            segmentMaximum_deg(1) < bounds_deg(1) || ...
            segmentMinimum_deg(1) > bounds_deg(2) || ...
            segmentMaximum_deg(2) < bounds_deg(3) || ...
            segmentMinimum_deg(2) > bounds_deg(4)
        continue;
    end
    firstEdgeIndex = double(obstacle.EdgeOffsets(sampleIndex));
    finalEdgeIndex = double(obstacle.EdgeOffsets(sampleIndex + 1)) - 1;
    if finalEdgeIndex < firstEdgeIndex
        continue;
    end
    edgeIndex = (firstEdgeIndex:finalEdgeIndex).';
    edgeStart_deg = [ ...
        double(obstacle.EdgeStartAzimuthDeg(edgeIndex)), ...
        double(obstacle.EdgeStartElevationDeg(edgeIndex))];
    edgeEnd_deg = [ ...
        double(obstacle.EdgeEndAzimuthDeg(edgeIndex)), ...
        double(obstacle.EdgeEndElevationDeg(edgeIndex))];
    edgeBreakpoint = segmentEdgeIntersectionParameters( ...
        firstPosition_deg, secondPosition_deg, ...
        edgeStart_deg, edgeEnd_deg);
    breakpoint = [breakpoint; edgeBreakpoint]; %#ok<AGROW>
end
breakpoint = sort(max(0, min(1, breakpoint)));
if numel(breakpoint) > 1
    scale = max(1, max(abs(breakpoint)));
    retain = [true; diff(breakpoint) > 1e-11 * scale];
    breakpoint = breakpoint(retain);
end
if breakpoint(1) > 0
    breakpoint = [0; breakpoint];
end
if breakpoint(end) < 1
    breakpoint(end + 1, 1) = 1;
end
intervalMidpoint = 0.5 .* (breakpoint(1:end - 1) + ...
    breakpoint(2:end));
if isempty(intervalMidpoint)
    intervalMidpoint = 0.5;
end
end

function intersectionParameter = segmentEdgeIntersectionParameters( ...
        firstPosition_deg, secondPosition_deg, ...
        edgeStart_deg, edgeEnd_deg)
%% Section 0: Header & Readme
% SYNTAX
%   intersectionParameter = segmentEdgeIntersectionParameters( ...
%       firstPosition_deg, secondPosition_deg, ...
%       edgeStart_deg, edgeEnd_deg)
%**************************************************************************
% PURPOSE
%   - Return segment parameters for crossings and collinear overlaps.
%**************************************************************************
% INPUTS
%   - firstPosition_deg, secondPosition_deg (1-by-2 numeric rows)
%   - edgeStart_deg, edgeEnd_deg (N-by-2 numeric matrices)
%**************************************************************************
% OUTPUTS
%   - intersectionParameter (M-by-1 numeric vector)
%       Candidate-segment fractions clipped to [0, 1].
%**************************************************************************
% UNITS
%   - Positions are degrees; output fractions are dimensionless.
%**************************************************************************
segmentVector_deg = secondPosition_deg - firstPosition_deg;
edgeVector_deg = edgeEnd_deg - edgeStart_deg;
edgeOffset_deg = edgeStart_deg - firstPosition_deg;
denominator_deg2 = segmentVector_deg(1) .* edgeVector_deg(:, 2) - ...
    segmentVector_deg(2) .* edgeVector_deg(:, 1);
edgeLength_deg = hypot(edgeVector_deg(:, 1), edgeVector_deg(:, 2));
segmentLength_deg = hypot(segmentVector_deg(1), segmentVector_deg(2));
tolerance_deg2 = 1e-11 .* max(1, ...
    segmentLength_deg .* edgeLength_deg);
isNonparallel = abs(denominator_deg2) > tolerance_deg2;
intersectionParameter = zeros(0, 1);
if any(isNonparallel)
    numeratorT_deg2 = edgeOffset_deg(:, 1) .* edgeVector_deg(:, 2) - ...
        edgeOffset_deg(:, 2) .* edgeVector_deg(:, 1);
    numeratorU_deg2 = edgeOffset_deg(:, 1) .* segmentVector_deg(2) - ...
        edgeOffset_deg(:, 2) .* segmentVector_deg(1);
    parameterT = numeratorT_deg2(isNonparallel) ./ ...
        denominator_deg2(isNonparallel);
    parameterU = numeratorU_deg2(isNonparallel) ./ ...
        denominator_deg2(isNonparallel);
    parameterTolerance = 1e-10;
    isOnBothSegments = parameterT >= -parameterTolerance & ...
        parameterT <= 1 + parameterTolerance & ...
        parameterU >= -parameterTolerance & ...
        parameterU <= 1 + parameterTolerance;
    intersectionParameter = parameterT(isOnBothSegments);
end

isParallel = ~isNonparallel;
if any(isParallel)
    collinearCross_deg2 = ...
        edgeOffset_deg(:, 1) .* segmentVector_deg(2) - ...
        edgeOffset_deg(:, 2) .* segmentVector_deg(1);
    isCollinear = isParallel & ...
        abs(collinearCross_deg2) <= tolerance_deg2;
    if any(isCollinear)
        segmentLengthSquared_deg2 = sum(segmentVector_deg.^2);
        collinearStart = edgeOffset_deg(isCollinear, :) * ...
            segmentVector_deg.' ./ segmentLengthSquared_deg2;
        collinearEnd = (edgeEnd_deg(isCollinear, :) - ...
            firstPosition_deg) * segmentVector_deg.' ./ ...
            segmentLengthSquared_deg2;
        overlapStart = max(0, min(collinearStart, collinearEnd));
        overlapEnd = min(1, max(collinearStart, collinearEnd));
        hasOverlap = overlapEnd >= overlapStart - 1e-10;
        intersectionParameter = [intersectionParameter; ...
            overlapStart(hasOverlap); overlapEnd(hasOverlap)];
    end
end
intersectionParameter = max(0, min(1, intersectionParameter));
end

function [route_deg, edgeCount, routeDistance_deg] = ...
        forwardBoundaryRoute(region_deg, startCandidate_deg, ...
        goalCandidate_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [route_deg, edgeCount, routeDistance_deg] = ...
%       forwardBoundaryRoute(region_deg, startCandidate_deg, ...
%       goalCandidate_deg)
%**************************************************************************
% PURPOSE
%   - Walk a polygon ring in its stored direction between contacts.
%**************************************************************************
% INPUTS
%   - region_deg (N-by-2 numeric matrix)
%   - startCandidate_deg, goalCandidate_deg (1-by-2 numeric rows)
%**************************************************************************
% OUTPUTS
%   - route_deg (M-by-2 numeric matrix)
%   - edgeCount (nonnegative integer scalar)
%   - routeDistance_deg (nonnegative numeric scalar)
%**************************************************************************
% UNITS
%   - Position and route distance are degrees.
%**************************************************************************
if size(region_deg, 1) > 1 && hypot( ...
        region_deg(end, 1) - region_deg(1, 1), ...
        region_deg(end, 2) - region_deg(1, 2)) <= 1e-10
    region_deg(end, :) = [];
end
vertexCount = size(region_deg, 1);
if vertexCount < 2
    route_deg = [startCandidate_deg; goalCandidate_deg];
    edgeCount = 1;
    routeDistance_deg = hypot( ...
        goalCandidate_deg(1) - startCandidate_deg(1), ...
        goalCandidate_deg(2) - startCandidate_deg(2));
    return;
end

closedRegion_deg = [region_deg; region_deg(1, :)];
boundaryStep_deg = diff(closedRegion_deg, 1, 1);
boundaryEdgeLength_deg = hypot( ...
    boundaryStep_deg(:, 1), boundaryStep_deg(:, 2));
perimeter_deg = sum(boundaryEdgeLength_deg);
vertexArc_deg = [0; cumsum(boundaryEdgeLength_deg(1:end - 1))];
startArc_deg = boundaryArcPosition( ...
    region_deg, boundaryEdgeLength_deg, vertexArc_deg, ...
    startCandidate_deg, perimeter_deg);
goalArc_deg = boundaryArcPosition( ...
    region_deg, boundaryEdgeLength_deg, vertexArc_deg, ...
    goalCandidate_deg, perimeter_deg);
forwardArc_deg = mod(goalArc_deg - startArc_deg, perimeter_deg);

relativeVertexArc_deg = mod(vertexArc_deg - startArc_deg, perimeter_deg);
arcTolerance_deg = max(1e-10, 1e-10 * perimeter_deg);
interiorVertex = relativeVertexArc_deg > arcTolerance_deg & ...
    relativeVertexArc_deg < forwardArc_deg - arcTolerance_deg;
interiorVertexIndex = find(interiorVertex);
[~, vertexOrder] = sort(relativeVertexArc_deg(interiorVertexIndex));
interiorVertexIndex = interiorVertexIndex(vertexOrder);
route_deg = [startCandidate_deg; ...
    region_deg(interiorVertexIndex, :); goalCandidate_deg];
route_deg = removeConsecutiveDuplicatePoints(route_deg);
edgeCount = max(0, size(route_deg, 1) - 1);
routeStep_deg = diff(route_deg, 1, 1);
routeDistance_deg = sum(hypot( ...
    routeStep_deg(:, 1), routeStep_deg(:, 2)));
end

function arcPosition_deg = boundaryArcPosition( ...
        region_deg, edgeLength_deg, vertexArc_deg, point_deg, ...
        perimeter_deg)
%% Section 0: Header & Readme
% SYNTAX
%   arcPosition_deg = boundaryArcPosition(region_deg, edgeLength_deg, ...
%       vertexArc_deg, point_deg, perimeter_deg)
%**************************************************************************
% PURPOSE
%   - Project a boundary point to the closest edge and cyclic arc length.
%**************************************************************************
% INPUTS
%   - region_deg (N-by-2 numeric matrix)
%   - edgeLength_deg, vertexArc_deg (N-by-1 numeric vectors)
%   - point_deg (1-by-2 numeric row)
%   - perimeter_deg (positive numeric scalar)
%**************************************************************************
% OUTPUTS
%   - arcPosition_deg (numeric scalar)
%**************************************************************************
% UNITS
%   - Positions, lengths, and arc coordinates are degrees.
%**************************************************************************
nextRegion_deg = region_deg([2:end 1], :);
edgeVector_deg = nextRegion_deg - region_deg;
edgeLengthSquared_deg2 = sum(edgeVector_deg.^2, 2);
pointOffset_deg = point_deg - region_deg;
projectionFraction = sum(pointOffset_deg .* edgeVector_deg, 2) ./ ...
    max(edgeLengthSquared_deg2, eps);
projectionFraction = min(max(projectionFraction, 0), 1);
projectedPoint_deg = region_deg + projectionFraction .* edgeVector_deg;
projectionError_deg2 = sum((projectedPoint_deg - point_deg).^2, 2);
[~, closestEdgeIndex] = min(projectionError_deg2);
arcPosition_deg = vertexArc_deg(closestEdgeIndex) + ...
    projectionFraction(closestEdgeIndex) * edgeLength_deg(closestEdgeIndex);
arcPosition_deg = mod(arcPosition_deg, perimeter_deg);
end

function route_deg = removeConsecutiveDuplicatePoints(route_deg)
%% Section 0: Header & Readme
% SYNTAX
%   route_deg = removeConsecutiveDuplicatePoints(route_deg)
%**************************************************************************
% PURPOSE
%   - Remove zero-length steps while retaining geometric turns.
%**************************************************************************
% INPUTS
%   - route_deg (N-by-2 numeric matrix)
%**************************************************************************
% OUTPUTS
%   - route_deg (M-by-2 numeric matrix)
%**************************************************************************
% UNITS
%   - Positions are degrees.
%**************************************************************************
if size(route_deg, 1) < 2
    return;
end
routeStep_deg = diff(route_deg, 1, 1);
keepPoint = [true; hypot( ...
    routeStep_deg(:, 1), routeStep_deg(:, 2)) > 1e-10];
route_deg = route_deg(keepPoint, :);
end

