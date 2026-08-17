function [graphs, mergedGraphCandidates, parallelExecution] = ...
        buildAzElSnapshotVisibilityGraphs(obstacleField, selection, options)
%% Section 0: Header & Readme
% SYNTAX
%   [graphs, mergedGraphCandidates, parallelExecution] = ...
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
%       Resolved visibility resolution, stage, and parallel controls.
%**************************************************************************
% OUTPUTS
%   - graphs (structure array)
%       One stable visibility graph and shortest route per retained time.
%   - mergedGraphCandidates (structure array)
%       One dense-scene alternative whose protected obstacle history is
%       merged to about one group per five source obstacles. Its sparse
%       graph retains corridor groups and nearby detour groups.
%   - parallelExecution (scalar struct)
%       Requested mode, availability, worker count, and fallback evidence.
%**************************************************************************
% UNITS
%   - Graph positions are degrees and snapshot times are seconds.
%**************************************************************************

%% Section 1: Build Snapshot Graphs

[graphs, mergedGraphCandidates, parallelExecution] = ...
    buildSnapshotVisibilityGraphs( ...
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

function [graphs, mergedGraphs, parallelExecution] = ...
        buildSnapshotVisibilityGraphs( ...
        obstacleField, candidatePoints, candidateTypes, ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, snapshotTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [graphs, mergedGraphs, parallelExecution] = ...
%       buildSnapshotVisibilityGraphs( ...
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
%   - mergedGraphs (structure array)
%       One full-history dense-scene merge attempt.
%   - parallelExecution (scalar struct)
%       Requested mode, availability, worker count, and fallback message.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
graphs = repmat(emptyVisibilityGraph(), 0, 1);
mergedGraphs = repmat(emptyVisibilityGraph(), 0, 1);
snapshotTimes_s = unique(snapshotTimes_s(:));
if isempty(snapshotTimes_s)
    [~, parallelExecution] = resolveParallelExecution( ...
        options.UseParallel, 0);
    return;
end
snapshotCount = numel(snapshotTimes_s);
if ~options.BuildFullVisibilityGraphs
    [~, parallelExecution] = resolveParallelExecution( ...
        options.UseParallel, 0);
else
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
end
if options.BuildMergedVisibilityGraph
    mergedGraph = buildMergedObstacleGraphAtTime( ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, snapshotTimes_s(1), options);
    if mergedGraph.ObstacleMergeAttempted
        mergedGraphs = mergedGraph;
    end
end

% Worker output is intentionally suppressed. Printing after the loop keeps
% diagnostics deterministic even when parfor completes tasks out of order.
if options.Verbose
    fprintf("[visibility graph] %s\n", parallelExecution.Message);
    graphCount = numel(graphs);
    for snapshotIndex = 1:graphCount
        fprintf("[visibility graph] %d/%d at t=%.3f s: " + ...
            "%d active candidates, success=%d.\n", ...
            snapshotIndex, graphCount, ...
            snapshotTimes_s(snapshotIndex), ...
            nnz(graphs(snapshotIndex).CandidateActiveMask), ...
            graphs(snapshotIndex).Success);
    end
    if graphCount == 0 && snapshotCount > 0
        fprintf("[visibility graph] Full snapshot graph construction " + ...
            "is disabled for %d retained times.\n", snapshotCount);
    end
    if ~isempty(mergedGraphs)
        fprintf("[visibility graph] merged %d obstacles to %d groups; " + ...
            "the sparse graph retained %d: success=%d.\n", ...
            mergedGraphs.SourceObstacleCount, ...
            mergedGraphs.MergedGroupCount, ...
            mergedGraphs.GraphObstacleCount, mergedGraphs.Success);
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

function mergedGraph = buildMergedObstacleGraphAtTime( ...
        candidateObstacleIndex, candidateSampleIndex, ...
        candidateRegionIndex, candidateBoundaryGeometry, ...
        startPosition_deg, goalPosition_deg, snapshotTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   mergedGraph = buildMergedObstacleGraphAtTime( ...
%       candidateObstacleIndex, candidateSampleIndex, ...
%       candidateRegionIndex, candidateBoundaryGeometry, ...
%       startPosition_deg, goalPosition_deg, snapshotTime_s, options)
%**************************************************************************
% PURPOSE
%   - Build one conservative dense-scene alternative from protected swept
%     hulls without enclosing either endpoint.
%   - Limit grouping complexity to about one group per five source
%     obstacles, then graph only corridor-relevant and neighboring groups.
%**************************************************************************
% INPUTS
%   - candidate provenance vectors and boundary geometry (N-by-1)
%       Protected obstacle regions across the retained time history.
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%   - snapshotTime_s (numeric scalar), options (scalar struct)
%**************************************************************************
% OUTPUTS
%   - mergedGraph (scalar visibility-graph record)
%       A completed graph or explicit evidence that merging was impossible.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
mergedGraph = emptyVisibilityGraph();
mergedGraph.Representation = "mergedObstacleGraph";
mergedGraph.Time_s = snapshotTime_s;
sourceObstacleIndex = unique(candidateObstacleIndex(:), "stable");
sourceObstacleIndex = sourceObstacleIndex(sourceObstacleIndex > 0);
sourceObstacleCount = numel(sourceObstacleIndex);
mergedGraph.SourceObstacleCount = sourceObstacleCount;
if sourceObstacleCount <= 4
    mergedGraph.Message = ...
        "Four or fewer obstacles; no merged graph was needed.";
    return;
end
mergedGraph.ObstacleMergeAttempted = true;

% Rounding up keeps every source obstacle represented while bounding the
% merged graph to approximately one protected group per five obstacles.
maximumMergedObstacleCount = ceil(sourceObstacleCount / 5);
mergedGraph.MaximumGraphObstacleCount = maximumMergedObstacleCount;

groupGeometry_deg = cell(sourceObstacleCount, 1);
groupMembership = cell(sourceObstacleCount, 1);
for sourceIndex = 1:sourceObstacleCount
    obstacleIndex = sourceObstacleIndex(sourceIndex);
    belongsToObstacle = candidateObstacleIndex == obstacleIndex;
    boundaryKey = [candidateSampleIndex(belongsToObstacle), ...
        candidateRegionIndex(belongsToObstacle)];
    [~, firstBoundaryRow] = unique(boundaryKey, "rows", "stable");
    obstacleBoundaryGeometry = ...
        candidateBoundaryGeometry(belongsToObstacle);
    obstacleVertex_deg = zeros(0, 2);
    for boundaryIndex = reshape(firstBoundaryRow, 1, [])
        region_deg = obstacleBoundaryGeometry{boundaryIndex};
        region_deg = region_deg(all(isfinite(region_deg), 2), :);
        obstacleVertex_deg = [obstacleVertex_deg; region_deg]; %#ok<AGROW>
    end
    [obstacleHull_deg, hullIsValid] = convexBoundary( ...
        obstacleVertex_deg);
    if ~hullIsValid
        mergedGraph.Message = "Obstacle " + obstacleIndex + ...
            " has no valid two-dimensional boundary to merge.";
        return;
    end
    if boundaryContainsEndpoint( ...
            obstacleHull_deg, startPosition_deg, goalPosition_deg)
        mergedGraph.Message = "The convex hull of obstacle " + ...
            obstacleIndex + " contains the start or goal; conservative " + ...
            "group merging is unavailable for this history.";
        return;
    end
    groupGeometry_deg{sourceIndex} = obstacleHull_deg;
    groupMembership{sourceIndex} = obstacleIndex;
end

[groupGeometry_deg, groupMembership, mergeComplete] = ...
    mergeNearestObstacleGroups(groupGeometry_deg, groupMembership, ...
    startPosition_deg, goalPosition_deg, maximumMergedObstacleCount);
mergedGraph.MergedGroupCount = numel(groupGeometry_deg);
mergedGraph.ObstacleMergeComplete = mergeComplete;
if ~mergeComplete
    mergedGraph.Message = "No endpoint-safe sequence of conservative " + ...
        "merges reached the adaptive obstacle-group limit.";
    return;
end

maximumMergedBoundaryVertexCount = 24;
for groupIndex = 1:numel(groupGeometry_deg)
    conservativeBoundary_deg = conservativeMergedBoundary( ...
        groupGeometry_deg{groupIndex}, ...
        maximumMergedBoundaryVertexCount);
    if ~boundaryContainsEndpoint(conservativeBoundary_deg, ...
            startPosition_deg, goalPosition_deg)
        groupGeometry_deg{groupIndex} = conservativeBoundary_deg;
    end
end

[selectedGroupIndex, omittedGroupIndex] = sparseMergedGroupIndices( ...
    groupGeometry_deg, startPosition_deg, goalPosition_deg);
selectedGroupGeometry_deg = groupGeometry_deg(selectedGroupIndex);
selectedGroupMembership = groupMembership(selectedGroupIndex);
omittedGroupGeometry_deg = groupGeometry_deg(omittedGroupIndex);
omittedGroupMembership = groupMembership(omittedGroupIndex);

mergedGraph.GraphObstacleCount = numel(selectedGroupIndex);
mergedGraph.MergedObstacleMembership = selectedGroupMembership;
mergedGraph.MergedObstacleGeometry_deg = selectedGroupGeometry_deg;
mergedGraph.OmittedObstacleMembership = omittedGroupMembership;
mergedGraph.OmittedObstacleGeometry_deg = omittedGroupGeometry_deg;

mergedObstacleData = cell(numel(selectedGroupGeometry_deg), 1);
candidatePosition_deg = zeros(0, 2);
candidateTypes = strings(0, 1);
candidateGroupIndex = zeros(0, 1);
candidateBoundaryGeometryForGraph = cell(0, 1);
for groupIndex = 1:numel(selectedGroupGeometry_deg)
    region_deg = selectedGroupGeometry_deg{groupIndex};
    mergedObstacleData{groupIndex} = makeAzElObstacleData( ...
        "Merged obstacle group " + groupIndex, snapshotTime_s, ...
        region_deg(:, 1), region_deg(:, 2), 0);
    vertexCount = size(region_deg, 1);
    candidatePosition_deg = [candidatePosition_deg; ...
        region_deg]; %#ok<AGROW>
    candidateTypes = [candidateTypes; ...
        repmat("mergedHullVertex", vertexCount, 1)]; %#ok<AGROW>
    candidateGroupIndex = [candidateGroupIndex; ...
        repmat(groupIndex, vertexCount, 1)]; %#ok<AGROW>
    candidateBoundaryGeometryForGraph = [ ...
        candidateBoundaryGeometryForGraph; ...
        repmat({region_deg}, vertexCount, 1)]; %#ok<AGROW>
end
mergedObstacleField = buildAzElTimeObstacleField(mergedObstacleData);
candidateCount = size(candidatePosition_deg, 1);
mergedGraph = buildVisibilityGraphAtTime( ...
    mergedObstacleField, candidatePosition_deg, candidateTypes, ...
    candidateGroupIndex, ones(candidateCount, 1), ...
    ones(candidateCount, 1), candidateBoundaryGeometryForGraph, ...
    zeros(candidateCount, 1), startPosition_deg, goalPosition_deg, ...
    snapshotTime_s, options);
mergedGraph.Representation = "mergedObstacleGraph";
mergedGraph.ObstacleMergeAttempted = true;
mergedGraph.ObstacleMergeComplete = true;
mergedGraph.SourceObstacleCount = sourceObstacleCount;
mergedGraph.MaximumGraphObstacleCount = maximumMergedObstacleCount;
mergedGraph.MergedGroupCount = numel(groupGeometry_deg);
mergedGraph.GraphObstacleCount = numel(selectedGroupGeometry_deg);
mergedGraph.MergedObstacleMembership = selectedGroupMembership;
mergedGraph.MergedObstacleGeometry_deg = selectedGroupGeometry_deg;
mergedGraph.OmittedObstacleMembership = omittedGroupMembership;
mergedGraph.OmittedObstacleGeometry_deg = omittedGroupGeometry_deg;
end

function [selectedGroupIndex, omittedGroupIndex] = ...
        sparseMergedGroupIndices( ...
        groupGeometry_deg, startPosition_deg, goalPosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [selectedGroupIndex, omittedGroupIndex] = ...
%       sparseMergedGroupIndices( ...
%       groupGeometry_deg, startPosition_deg, goalPosition_deg)
%**************************************************************************
% PURPOSE
%   - Retain merged groups that block the direct corridor.
%   - Add nearby groups on both sides of the corridor so the sparse graph
%     preserves meaningful detours instead of only the direct obstruction.
%**************************************************************************
% INPUTS
%   - groupGeometry_deg (G-by-1 cell column)
%       Closed polygon boundaries for endpoint-safe merged groups.
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%**************************************************************************
% OUTPUTS
%   - selectedGroupIndex, omittedGroupIndex (integer column vectors)
%**************************************************************************
% UNITS
%   - Position and distance are degrees.
%**************************************************************************
groupCount = numel(groupGeometry_deg);
intersectsCorridor = false(groupCount, 1);
distanceToCorridor_deg = Inf(groupCount, 1);
signedSide = zeros(groupCount, 1);
routeDirection_deg = goalPosition_deg - startPosition_deg;

for groupIndex = 1:groupCount
    boundary_deg = groupGeometry_deg{groupIndex};
    intersectsCorridor(groupIndex) = boundaryIntersectsRoute( ...
        boundary_deg, startPosition_deg, goalPosition_deg);
    distanceToCorridor_deg(groupIndex) = min(pointToSegmentDistance( ...
        boundary_deg, startPosition_deg, goalPosition_deg));
    groupCenter_deg = mean(boundary_deg, 1);
    centerOffset_deg = groupCenter_deg - startPosition_deg;
    signedSide(groupIndex) = routeDirection_deg(1) * ...
        centerOffset_deg(2) - routeDirection_deg(2) * ...
        centerOffset_deg(1);
end

seedIndex = find(intersectsCorridor);
if isempty(seedIndex)
    [~, seedIndex] = min(distanceToCorridor_deg);
end

selectedGroupIndex = seedIndex(:);
for seedPosition = 1:numel(seedIndex)
    seedGroupIndex = seedIndex(seedPosition);
    neighborIndex = nearestDetourGroupIndices( ...
        seedGroupIndex, groupGeometry_deg, signedSide);
    selectedGroupIndex = [selectedGroupIndex; ...
        neighborIndex(:)]; %#ok<AGROW>
end
selectedGroupIndex = unique(selectedGroupIndex, "sorted");
omittedGroupIndex = setdiff((1:groupCount).', selectedGroupIndex, "stable");
end

function neighborIndex = nearestDetourGroupIndices( ...
        seedGroupIndex, groupGeometry_deg, signedSide)
%% Section 0: Header & Readme
% SYNTAX
%   neighborIndex = nearestDetourGroupIndices( ...
%       seedGroupIndex, groupGeometry_deg, signedSide)
%**************************************************************************
% PURPOSE
%   - Select at most two nearby groups, preferring one on each side of the
%     start-goal corridor so both detour directions remain represented.
%**************************************************************************
% INPUTS
%   - seedGroupIndex (positive integer scalar)
%   - groupGeometry_deg (G-by-1 cell column)
%   - signedSide (G-by-1 numeric vector)
%**************************************************************************
% OUTPUTS
%   - neighborIndex (zero-to-two integer column vector)
%**************************************************************************
% UNITS
%   - Group geometry and distances are degrees.
%**************************************************************************
groupCount = numel(groupGeometry_deg);
candidateIndex = setdiff((1:groupCount).', seedGroupIndex, "stable");
if isempty(candidateIndex)
    neighborIndex = zeros(0, 1);
    return;
end

distance_deg = Inf(groupCount, 1);
seedBoundary_deg = groupGeometry_deg{seedGroupIndex};
for candidatePosition = 1:numel(candidateIndex)
    groupIndex = candidateIndex(candidatePosition);
    candidateBoundary_deg = groupGeometry_deg{groupIndex};
    displacement_deg = reshape(seedBoundary_deg, [], 1, 2) - ...
        reshape(candidateBoundary_deg, 1, [], 2);
    pairDistance_deg = hypot( ...
        displacement_deg(:, :, 1), displacement_deg(:, :, 2));
    distance_deg(groupIndex) = min(pairDistance_deg, [], "all");
end

neighborIndex = zeros(0, 1);
positiveSideIndex = candidateIndex(signedSide(candidateIndex) >= 0);
negativeSideIndex = candidateIndex(signedSide(candidateIndex) < 0);
if ~isempty(positiveSideIndex)
    [~, localIndex] = min(distance_deg(positiveSideIndex));
    neighborIndex(end + 1, 1) = positiveSideIndex(localIndex);
end
if ~isempty(negativeSideIndex)
    [~, localIndex] = min(distance_deg(negativeSideIndex));
    neighborIndex(end + 1, 1) = negativeSideIndex(localIndex);
end

% A one-sided scene still retains two nearby alternatives when available.
if numel(neighborIndex) < min(2, numel(candidateIndex))
    remainingIndex = setdiff(candidateIndex, neighborIndex, "stable");
    [~, order] = sort(distance_deg(remainingIndex), "ascend");
    neededCount = min(2 - numel(neighborIndex), numel(remainingIndex));
    neighborIndex = [neighborIndex; ...
        remainingIndex(order(1:neededCount))];
end
end

function intersectsRoute = boundaryIntersectsRoute( ...
        boundary_deg, startPosition_deg, goalPosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   intersectsRoute = boundaryIntersectsRoute( ...
%       boundary_deg, startPosition_deg, goalPosition_deg)
%**************************************************************************
% PURPOSE
%   - Identify merged groups whose protected boundary intersects the direct
%     start-goal segment.
%**************************************************************************
% INPUTS
%   - boundary_deg (N-by-2 numeric matrix)
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%**************************************************************************
% OUTPUTS
%   - intersectsRoute (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
edgeStart_deg = boundary_deg;
edgeEnd_deg = boundary_deg([2:end 1], :);
routeVector_deg = goalPosition_deg - startPosition_deg;
edgeVector_deg = edgeEnd_deg - edgeStart_deg;
edgeOffset_deg = edgeStart_deg - startPosition_deg;
denominator_deg2 = routeVector_deg(1) .* edgeVector_deg(:, 2) - ...
    routeVector_deg(2) .* edgeVector_deg(:, 1);
tolerance_deg2 = 1e-11 .* max(1, norm(routeVector_deg) .* ...
    hypot(edgeVector_deg(:, 1), edgeVector_deg(:, 2)));
isNonparallel = abs(denominator_deg2) > tolerance_deg2;
routeParameter = nan(size(denominator_deg2));
edgeParameter = nan(size(denominator_deg2));
routeParameter(isNonparallel) = ( ...
    edgeOffset_deg(isNonparallel, 1) .* ...
    edgeVector_deg(isNonparallel, 2) - ...
    edgeOffset_deg(isNonparallel, 2) .* ...
    edgeVector_deg(isNonparallel, 1)) ./ ...
    denominator_deg2(isNonparallel);
edgeParameter(isNonparallel) = ( ...
    edgeOffset_deg(isNonparallel, 1) .* routeVector_deg(2) - ...
    edgeOffset_deg(isNonparallel, 2) .* routeVector_deg(1)) ./ ...
    denominator_deg2(isNonparallel);
parameterTolerance = 1e-10;
properIntersection = isNonparallel & ...
    routeParameter >= -parameterTolerance & ...
    routeParameter <= 1 + parameterTolerance & ...
    edgeParameter >= -parameterTolerance & ...
    edgeParameter <= 1 + parameterTolerance;

isParallel = ~isNonparallel;
collinearCross_deg2 = edgeOffset_deg(:, 1) .* routeVector_deg(2) - ...
    edgeOffset_deg(:, 2) .* routeVector_deg(1);
isCollinear = isParallel & ...
    abs(collinearCross_deg2) <= tolerance_deg2;
collinearOverlap = false(size(isCollinear));
if any(isCollinear)
    routeLengthSquared_deg2 = sum(routeVector_deg.^2);
    edgeStartParameter = edgeOffset_deg(isCollinear, :) * ...
        routeVector_deg.' / routeLengthSquared_deg2;
    edgeEndParameter = (edgeEnd_deg(isCollinear, :) - ...
        startPosition_deg) * routeVector_deg.' / ...
        routeLengthSquared_deg2;
    collinearOverlap(isCollinear) = ...
        min(max(edgeStartParameter, edgeEndParameter), 1) >= ...
        max(min(edgeStartParameter, edgeEndParameter), 0) - ...
        parameterTolerance;
end
intersectsRoute = any(properIntersection | collinearOverlap);
end

function [groups_deg, memberships, complete] = ...
        mergeNearestObstacleGroups(groups_deg, memberships, ...
        startPosition_deg, goalPosition_deg, maximumGroupCount)
%% Section 0: Header & Readme
% SYNTAX
%   [groups_deg, memberships, complete] = ...
%       mergeNearestObstacleGroups(groups_deg, memberships, ...
%       startPosition_deg, goalPosition_deg, maximumGroupCount)
%**************************************************************************
% PURPOSE
%   - Repeatedly choose the endpoint-safe hull merge adding the least area.
%**************************************************************************
% INPUTS
%   - groups_deg, memberships (cell columns)
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%   - maximumGroupCount (positive integer scalar)
%**************************************************************************
% OUTPUTS
%   - groups_deg, memberships (reduced cell columns)
%   - complete (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees and merge cost is square degrees.
%**************************************************************************
while numel(groups_deg) > maximumGroupCount
    bestFirstIndex = 0;
    bestSecondIndex = 0;
    bestMergeCost_deg2 = Inf;
    bestHull_deg = zeros(0, 2);
    for firstIndex = 1:numel(groups_deg) - 1
        firstArea_deg2 = polyarea( ...
            groups_deg{firstIndex}(:, 1), ...
            groups_deg{firstIndex}(:, 2));
        for secondIndex = firstIndex + 1:numel(groups_deg)
            [mergedHull_deg, hullIsValid] = convexBoundary([ ...
                groups_deg{firstIndex}; groups_deg{secondIndex}]);
            if ~hullIsValid || boundaryContainsEndpoint( ...
                    mergedHull_deg, startPosition_deg, goalPosition_deg)
                continue;
            end
            secondArea_deg2 = polyarea( ...
                groups_deg{secondIndex}(:, 1), ...
                groups_deg{secondIndex}(:, 2));
            mergedArea_deg2 = polyarea( ...
                mergedHull_deg(:, 1), mergedHull_deg(:, 2));
            mergeCost_deg2 = mergedArea_deg2 - ...
                firstArea_deg2 - secondArea_deg2;
            if mergeCost_deg2 < bestMergeCost_deg2
                bestFirstIndex = firstIndex;
                bestSecondIndex = secondIndex;
                bestMergeCost_deg2 = mergeCost_deg2;
                bestHull_deg = mergedHull_deg;
            end
        end
    end
    if bestFirstIndex == 0
        break;
    end
    groups_deg{bestFirstIndex} = bestHull_deg;
    memberships{bestFirstIndex} = sort([ ...
        memberships{bestFirstIndex}(:); ...
        memberships{bestSecondIndex}(:)]).';
    groups_deg(bestSecondIndex) = [];
    memberships(bestSecondIndex) = [];
end
complete = numel(groups_deg) <= maximumGroupCount;
end

function [boundary_deg, isValid] = convexBoundary(position_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [boundary_deg, isValid] = convexBoundary(position_deg)
%**************************************************************************
% PURPOSE
%   - Return a duplicate-free convex boundary for conservative merging.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 numeric matrix)
%**************************************************************************
% OUTPUTS
%   - boundary_deg (M-by-2 numeric matrix), isValid (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
position_deg = unique(double(position_deg), "rows", "stable");
isValid = size(position_deg, 1) >= 3 && ...
    rank(position_deg - mean(position_deg, 1)) >= 2;
if ~isValid
    boundary_deg = zeros(0, 2);
    return;
end
hullIndex = convhull(position_deg(:, 1), position_deg(:, 2));
boundary_deg = position_deg(hullIndex(1:end - 1), :);
end

function containsEndpoint = boundaryContainsEndpoint( ...
        boundary_deg, startPosition_deg, goalPosition_deg)
%% Section 0: Header & Readme
% SYNTAX
%   containsEndpoint = boundaryContainsEndpoint( ...
%       boundary_deg, startPosition_deg, goalPosition_deg)
%**************************************************************************
% PURPOSE
%   - Reject a conservative hull when its interior or boundary covers an
%     endpoint that the original protected obstacles leave available.
%**************************************************************************
% INPUTS
%   - boundary_deg (N-by-2 numeric matrix)
%   - startPosition_deg, goalPosition_deg (1-by-2 numeric rows)
%**************************************************************************
% OUTPUTS
%   - containsEndpoint (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
[startInside, startOnBoundary] = inpolygon( ...
    startPosition_deg(1), startPosition_deg(2), ...
    boundary_deg(:, 1), boundary_deg(:, 2));
[goalInside, goalOnBoundary] = inpolygon( ...
    goalPosition_deg(1), goalPosition_deg(2), ...
    boundary_deg(:, 1), boundary_deg(:, 2));
containsEndpoint = startInside || startOnBoundary || ...
    goalInside || goalOnBoundary;
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
    "Representation", "fullObstacleGraph", ...
    "ObstacleMergeAttempted", false, ...
    "ObstacleMergeComplete", false, ...
    "SourceObstacleCount", 0, ...
    "MaximumGraphObstacleCount", 0, ...
    "MergedGroupCount", 0, ...
    "GraphObstacleCount", 0, ...
    "MergedObstacleMembership", {cell(0, 1)}, ...
    "MergedObstacleGeometry_deg", {cell(0, 1)}, ...
    "OmittedObstacleMembership", {cell(0, 1)}, ...
    "OmittedObstacleGeometry_deg", {cell(0, 1)}, ...
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
generatedBoundaryEdgeCount = 0;
boundaryRouteInputVertexCount = 0;
boundaryRouteRetainedVertexCount = 0;

% Phase 1 is intentionally sparse: connect only start/goal to candidates,
% plus the direct edge. Together with cyclic boundary edges below, this
% produces a feasible upper-bound route before any candidate-pair clique is
% attempted.
candidateNodeIndex = (3:nodeCount).';
endpointCandidatePairs = [ ...
    ones(candidateCount, 1), candidateNodeIndex; ...
    2 * ones(candidateCount, 1), candidateNodeIndex];
phaseOnePairs = [[1 2]; endpointCandidatePairs];
[edgeCost_deg, edgeType, edgeRoute_deg, visibilityTestedMask, ...
    visibilityBlockedMask, ~] = ...
    addVisibilityEdgesIfClear(edgeCost_deg, edgeType, edgeRoute_deg, ...
    visibilityTestedMask, visibilityBlockedMask, nodePosition_deg, ...
    phaseOnePairs, obstacleField, snapshotTime_s, options);

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
        boundaryRoute_deg = removeConsecutiveDuplicatePoints( ...
            rawBoundaryRoute_deg);
        routeIsValid = routeSegmentsAreVisible( ...
            boundaryRoute_deg, obstacleField, snapshotTime_s, options);
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

% Phase 2 completes only candidates that can improve this graph's
% geometric objective.
activeCandidateIndex = find(candidateActiveMask);
if numel(activeCandidateIndex) >= 2
    phaseTwoCandidatePairs = nchoosek(activeCandidateIndex, 2);
    isMergedGraph = candidateCount > 0 && all( ...
        candidateTypes == "mergedHullVertex");
    if isMergedGraph
        phaseTwoCandidatePairs = sparseMergedCandidatePairs( ...
            phaseTwoCandidatePairs, candidatePosition_deg, ...
            candidateObstacleIndex);
    end
    phaseTwoPairs = phaseTwoCandidatePairs + 2;
    [edgeCost_deg, edgeType, edgeRoute_deg, visibilityTestedMask, ...
        visibilityBlockedMask, ~] = ...
        addVisibilityEdgesIfClear( ...
        edgeCost_deg, edgeType, edgeRoute_deg, visibilityTestedMask, ...
        visibilityBlockedMask, nodePosition_deg, phaseTwoPairs, ...
        obstacleField, snapshotTime_s, options);
end
generatedVisibilityEdgeCount = nnz(triu(edgeType == "visibility", 1));
generatedBoundaryEdgeCount = nnz(triu(edgeType == "boundary", 1));

[pathNodeIndex, ~] = shortestVisibilityGraphPath( ...
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

[pathPosition_deg, pathEdgeType] = assembleGraphRoute( ...
    pathNodeIndex, edgeType, edgeRoute_deg);
[compactPathPosition_deg, compactPathIsValid] = compactSelectedRoute( ...
    pathPosition_deg, obstacleField, snapshotTime_s, options);
if compactPathIsValid
    pathPosition_deg = compactPathPosition_deg;
end
pathStep_deg = diff(pathPosition_deg, 1, 1);
pathCost_deg = sum(hypot(pathStep_deg(:, 1), pathStep_deg(:, 2)));
pathEdgeCount = numel(pathNodeIndex) - 1;
graph.Success = true;
graph.Message = "Global visibility-graph Dijkstra connected start to goal.";
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

function [compactRoute_deg, success] = compactSelectedRoute( ...
        sourceRoute_deg, obstacleField, sampleTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [compactRoute_deg, success] = compactSelectedRoute( ...
%       sourceRoute_deg, obstacleField, sampleTime_s, options)
%**************************************************************************
% PURPOSE
%   - Remove redundant vertices from only the graph's selected route.
%   - Batch every forward visibility query from the current vertex so
%     polygon sampling density does not become unnecessary motion turns.
%**************************************************************************
% INPUTS
%   - sourceRoute_deg (N-by-2 numeric matrix)
%       Collision-checked route assembled from graph edges.
%   - obstacleField (scalar packed obstacle field)
%   - sampleTime_s (finite numeric scalar)
%   - options (scalar resolved search-options struct)
%**************************************************************************
% OUTPUTS
%   - compactRoute_deg (M-by-2 numeric matrix)
%   - success (logical scalar)
%       True only when every retained segment is independently visible.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
sourceRoute_deg = removeConsecutiveDuplicatePoints(sourceRoute_deg);
compactRoute_deg = sourceRoute_deg;
success = false;
sourceVertexCount = size(sourceRoute_deg, 1);
if sourceVertexCount < 2
    return;
elseif sourceVertexCount == 2
    success = routeSegmentsAreVisible( ...
        compactRoute_deg, obstacleField, sampleTime_s, options);
    return;
end

compactRoute_deg = sourceRoute_deg(1, :);

% RDP anchors retain the route's geometric shape while preventing one
% dense source boundary from creating a candidate-by-edge Cartesian
% product. Logarithmic look-ahead points remain available between anchors,
% so a blocked long chord does not force one-vertex-at-a-time progress.
shapeAnchor = false(sourceVertexCount, 1);
shapeAnchor([1 end]) = true;
segmentStack = [1 sourceVertexCount];
shapeTolerance_deg = options.VisibilitySampleStep_deg;
while ~isempty(segmentStack)
    firstIndex = segmentStack(end, 1);
    finalIndex = segmentStack(end, 2);
    segmentStack(end, :) = [];
    if finalIndex <= firstIndex + 1
        continue;
    end
    interiorIndex = (firstIndex + 1:finalIndex - 1).';
    deviation_deg = pointToSegmentDistance( ...
        sourceRoute_deg(interiorIndex, :), ...
        sourceRoute_deg(firstIndex, :), sourceRoute_deg(finalIndex, :));
    [maximumDeviation_deg, maximumDeviationIndex] = max(deviation_deg);
    if maximumDeviation_deg <= shapeTolerance_deg
        continue;
    end
    splitIndex = interiorIndex(maximumDeviationIndex);
    shapeAnchor(splitIndex) = true;
    segmentStack = [segmentStack; ...
        firstIndex splitIndex; splitIndex finalIndex]; %#ok<AGROW>
end
shapeAnchorIndex = find(shapeAnchor);

currentIndex = 1;
while currentIndex < sourceVertexCount
    remainingVertexCount = sourceVertexCount - currentIndex;
    lookAheadCount = min(16, remainingVertexCount);
    if remainingVertexCount == 1
        lookAheadOffset = 1;
    else
        lookAheadOffset = unique(round(logspace( ...
            0, log10(remainingVertexCount), lookAheadCount))).';
    end
    candidateIndex = unique([ ...
        currentIndex + lookAheadOffset; ...
        shapeAnchorIndex(shapeAnchorIndex > currentIndex)], "sorted");
    candidateCount = numel(candidateIndex);
    firstPosition_deg = repmat( ...
        sourceRoute_deg(currentIndex, :), candidateCount, 1);
    visible = linesVisibleAtTime( ...
        obstacleField, firstPosition_deg, ...
        sourceRoute_deg(candidateIndex, :), sampleTime_s, options);
    farthestVisibleIndex = find(visible, 1, "last");
    if isempty(farthestVisibleIndex)
        nextIndex = currentIndex + 1;
    else
        nextIndex = candidateIndex(farthestVisibleIndex);
    end
    compactRoute_deg(end + 1, :) = ...
        sourceRoute_deg(nextIndex, :); %#ok<AGROW>
    currentIndex = nextIndex;
end
success = routeSegmentsAreVisible( ...
    compactRoute_deg, obstacleField, sampleTime_s, options);
end

function retainedPairIndex = sparseMergedCandidatePairs( ...
        pairIndex, candidatePosition_deg, candidateGroupIndex)
%% Section 0: Header & Readme
% SYNTAX
%   retainedPairIndex = sparseMergedCandidatePairs( ...
%       pairIndex, candidatePosition_deg, candidateGroupIndex)
%**************************************************************************
% PURPOSE
%   - Bound the cross-group visibility work in a merged dense-scene graph.
%   - Preserve local alternatives and at least two candidate connections
%     between every represented group pair before collision testing.
%**************************************************************************
% INPUTS
%   - pairIndex (P-by-2 positive integer matrix)
%   - candidatePosition_deg (N-by-2 numeric matrix)
%   - candidateGroupIndex (N-by-1 positive integer vector)
%**************************************************************************
% OUTPUTS
%   - retainedPairIndex (Q-by-2 positive integer matrix)
%**************************************************************************
% UNITS
%   - Candidate positions and pair distances are degrees.
%**************************************************************************
if isempty(pairIndex)
    retainedPairIndex = pairIndex;
    return;
end

firstGroupIndex = candidateGroupIndex(pairIndex(:, 1));
secondGroupIndex = candidateGroupIndex(pairIndex(:, 2));
crossesGroup = firstGroupIndex ~= secondGroupIndex;
pairIndex = pairIndex(crossesGroup, :);
firstGroupIndex = firstGroupIndex(crossesGroup);
secondGroupIndex = secondGroupIndex(crossesGroup);
if isempty(pairIndex)
    retainedPairIndex = pairIndex;
    return;
end

pairOffset_deg = candidatePosition_deg(pairIndex(:, 1), :) - ...
    candidatePosition_deg(pairIndex(:, 2), :);
pairDistance_deg = hypot(pairOffset_deg(:, 1), pairOffset_deg(:, 2));
retainPair = false(size(pairIndex, 1), 1);

% Eight local cross-group neighbors preserve alternatives around blocked
% nearest connections without restoring the quadratic candidate clique.
maximumLocalNeighborCount = 8;
activeCandidateIndex = unique(pairIndex(:), "stable");
for activeIndex = reshape(activeCandidateIndex, 1, [])
    incidentPair = pairIndex(:, 1) == activeIndex | ...
        pairIndex(:, 2) == activeIndex;
    incidentRow = find(incidentPair);
    [~, localOrder] = sort(pairDistance_deg(incidentRow), "ascend");
    retainedCount = min(maximumLocalNeighborCount, numel(incidentRow));
    retainPair(incidentRow(localOrder(1:retainedCount))) = true;
end

groupPair = sort([firstGroupIndex, secondGroupIndex], 2);
uniqueGroupPair = unique(groupPair, "rows", "stable");
minimumConnectionsPerGroupPair = 2;
for groupPairIndex = 1:size(uniqueGroupPair, 1)
    belongsToGroupPair = all( ...
        groupPair == uniqueGroupPair(groupPairIndex, :), 2);
    groupPairRow = find(belongsToGroupPair);
    [~, localOrder] = sort(pairDistance_deg(groupPairRow), "ascend");
    retainedCount = min( ...
        minimumConnectionsPerGroupPair, numel(groupPairRow));
    retainPair(groupPairRow(localOrder(1:retainedCount))) = true;
end

retainedPairIndex = pairIndex(retainPair, :);
end

function conservativeBoundary_deg = conservativeMergedBoundary( ...
        sourceBoundary_deg, maximumVertexCount)
%% Section 0: Header & Readme
% SYNTAX
%   conservativeBoundary_deg = conservativeMergedBoundary( ...
%       sourceBoundary_deg, maximumVertexCount)
%**************************************************************************
% PURPOSE
%   - Bound merged-graph vertex count with a convex outer approximation.
%   - Preserve every source point inside the returned polygon so geometric
%     reduction cannot clip a protected obstacle corner.
%**************************************************************************
% INPUTS
%   - sourceBoundary_deg (N-by-2 numeric matrix)
%       Convex protected boundary.
%   - maximumVertexCount (integer scalar at least four)
%**************************************************************************
% OUTPUTS
%   - conservativeBoundary_deg (M-by-2 numeric matrix)
%       Source boundary or a containing support polygon.
%**************************************************************************
% UNITS
%   - Position is degrees; support directions are dimensionless.
%**************************************************************************
conservativeBoundary_deg = sourceBoundary_deg;
if size(sourceBoundary_deg, 1) <= maximumVertexCount
    return;
end

angle_rad = (0:maximumVertexCount - 1).' * ...
    (2 * pi / maximumVertexCount);
normal = [cos(angle_rad), sin(angle_rad)];
support_deg = max(sourceBoundary_deg * normal.', [], 1).';
roundoffGuard_deg = 64 * eps(max(1, max(abs(sourceBoundary_deg), [], "all")));
support_deg = support_deg + roundoffGuard_deg;
candidateBoundary_deg = zeros(maximumVertexCount, 2);

for normalIndex = 1:maximumVertexCount
    nextNormalIndex = mod(normalIndex, maximumVertexCount) + 1;
    normalPair = normal([normalIndex nextNormalIndex], :);
    if abs(det(normalPair)) <= 1e-12
        return;
    end
    candidateBoundary_deg(normalIndex, :) = (normalPair \ ...
        support_deg([normalIndex nextNormalIndex])).';
end

[candidateBoundary_deg, boundaryIsValid] = convexBoundary( ...
    candidateBoundary_deg);
if ~boundaryIsValid
    return;
end
[inside, onBoundary] = inpolygon( ...
    sourceBoundary_deg(:, 1), sourceBoundary_deg(:, 2), ...
    candidateBoundary_deg(:, 1), candidateBoundary_deg(:, 2));
if all(inside | onBoundary)
    conservativeBoundary_deg = candidateBoundary_deg;
end
end

function [routePosition_deg, routeEdgeType] = assembleGraphRoute( ...
        routeNodeIndex, edgeType, edgeRoute_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [routePosition_deg, routeEdgeType] = assembleGraphRoute( ...
%       routeNodeIndex, edgeType, edgeRoute_deg)
%**************************************************************************
% PURPOSE
%   - Expand one node path into its routed geometry and edge labels.
%**************************************************************************
% INPUTS
%   - routeNodeIndex (N-by-1 node-index vector)
%   - edgeType (square string matrix)
%   - edgeRoute_deg (square cell matrix of routed positions)
%**************************************************************************
% OUTPUTS
%   - routePosition_deg (M-by-2 numeric matrix)
%   - routeEdgeType (N-1-by-1 string vector)
%**************************************************************************
% UNITS
%   - Position is degrees; edge labels are dimensionless.
%**************************************************************************
edgeCount = numel(routeNodeIndex) - 1;
routeEdgeType = strings(edgeCount, 1);
routePosition_deg = zeros(0, 2);
for edgeIndex = 1:edgeCount
    firstNodeIndex = routeNodeIndex(edgeIndex);
    secondNodeIndex = routeNodeIndex(edgeIndex + 1);
    routeEdgeType(edgeIndex) = edgeType(firstNodeIndex, secondNodeIndex);
    edgePosition_deg = edgeRoute_deg{firstNodeIndex, secondNodeIndex};
    if isempty(edgePosition_deg)
        error("buildAzElSnapshotVisibilityGraphs:MissingEdgeRoute", ...
            "A retained graph edge has no routed position geometry.");
    end
    if isempty(routePosition_deg)
        routePosition_deg = edgePosition_deg;
    else
        routePosition_deg = [routePosition_deg; ...
            edgePosition_deg(2:end, :)]; %#ok<AGROW>
    end
end
routePosition_deg = removeConsecutiveDuplicatePoints(routePosition_deg);
end

function [edgeCost_deg, edgeType, edgeRoute_deg, testedMask, ...
        blockedMask, addedEdgeCount] = addVisibilityEdgesIfClear( ...
        edgeCost_deg, edgeType, edgeRoute_deg, testedMask, blockedMask, ...
        nodePosition_deg, nodePairIndex, obstacleField, snapshotTime_s, ...
        options)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeCost_deg, edgeType, edgeRoute_deg, testedMask, ...
%       blockedMask, addedEdgeCount] = addVisibilityEdgesIfClear( ...
%       edgeCost_deg, edgeType, edgeRoute_deg, testedMask, blockedMask, ...
%       nodePosition_deg, nodePairIndex, obstacleField, snapshotTime_s, ...
%       options)
%**************************************************************************
% PURPOSE
%   - Batch straight-edge collision queries at one resolved snapshot time.
%**************************************************************************
% INPUTS
%   - edgeCost_deg, edgeType, edgeRoute_deg (square graph matrices)
%   - testedMask, blockedMask (square logical matrices)
%   - nodePosition_deg (N-by-2 numeric matrix)
%   - nodePairIndex (M-by-2 positive integer matrix)
%   - obstacleField (scalar packed field), snapshotTime_s (numeric scalar)
%   - options (scalar resolved options struct)
%**************************************************************************
% OUTPUTS
%   - Updated graph matrices and visibility masks.
%   - addedEdgeCount (nonnegative integer scalar)
%**************************************************************************
% UNITS
%   - Position and cost are degrees; time is seconds.
%**************************************************************************
addedEdgeCount = 0;
if isempty(nodePairIndex)
    return;
end
firstNodeIndex = nodePairIndex(:, 1);
secondNodeIndex = nodePairIndex(:, 2);
firstPosition_deg = nodePosition_deg(firstNodeIndex, :);
secondPosition_deg = nodePosition_deg(secondNodeIndex, :);
displacement_deg = secondPosition_deg - firstPosition_deg;
distance_deg = hypot(displacement_deg(:, 1), displacement_deg(:, 2));
matrixIndex = sub2ind(size(edgeCost_deg), ...
    firstNodeIndex, secondNodeIndex);
pairNeedsTest = distance_deg > 1e-12 & ...
    distance_deg < edgeCost_deg(matrixIndex) - 1e-10;
if ~any(pairNeedsTest)
    return;
end
testedPairIndex = find(pairNeedsTest);
pairIsVisible = linesVisibleAtTime(obstacleField, ...
    firstPosition_deg(pairNeedsTest, :), ...
    secondPosition_deg(pairNeedsTest, :), snapshotTime_s, options);
testedFirstNodeIndex = firstNodeIndex(pairNeedsTest);
testedSecondNodeIndex = secondNodeIndex(pairNeedsTest);
testedForwardIndex = sub2ind(size(testedMask), ...
    testedFirstNodeIndex, testedSecondNodeIndex);
testedReverseIndex = sub2ind(size(testedMask), ...
    testedSecondNodeIndex, testedFirstNodeIndex);
testedMask([testedForwardIndex; testedReverseIndex]) = true;
blockedValue = ~pairIsVisible;
blockedMask(testedForwardIndex) = blockedValue;
blockedMask(testedReverseIndex) = blockedValue;

addedPairIndex = testedPairIndex(pairIsVisible);
for addedIndex = reshape(addedPairIndex, 1, [])
    firstIndex = firstNodeIndex(addedIndex);
    secondIndex = secondNodeIndex(addedIndex);
    edgeCost_deg(firstIndex, secondIndex) = distance_deg(addedIndex);
    edgeCost_deg(secondIndex, firstIndex) = distance_deg(addedIndex);
    edgeType(firstIndex, secondIndex) = "visibility";
    edgeType(secondIndex, firstIndex) = "visibility";
    edgeRoute_deg{firstIndex, secondIndex} = [ ...
        nodePosition_deg(firstIndex, :); ...
        nodePosition_deg(secondIndex, :)];
    edgeRoute_deg{secondIndex, firstIndex} = [ ...
        nodePosition_deg(secondIndex, :); ...
        nodePosition_deg(firstIndex, :)];
end
addedEdgeCount = numel(addedPairIndex);
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
if size(route_deg, 1) == 2
    visible = lineVisibleAtTime(obstacleField, ...
        route_deg(1, :), route_deg(2, :), sampleTime_s, options);
else
    segmentVisible = linesVisibleAtTime(obstacleField, ...
        route_deg(1:end - 1, :), route_deg(2:end, :), ...
        sampleTime_s, options);
    visible = all(segmentVisible);
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
visible = linesVisibleAtTime(obstacleField, firstPosition_deg, ...
    secondPosition_deg, sampleTime_s, struct());
end

function visible = linesVisibleAtTime( ...
        obstacleField, firstPosition_deg, secondPosition_deg, ...
        sampleTime_s, ~)
%% Section 0: Header & Readme
% SYNTAX
%   visible = linesVisibleAtTime(obstacleField, firstPosition_deg, ...
%       secondPosition_deg, sampleTime_s, options)
%**************************************************************************
% PURPOSE
%   - Test independent open segments through the maintained continuous
%     moving-obstacle collision kernel.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%   - firstPosition_deg, secondPosition_deg (N-by-2 numeric matrices)
%   - sampleTime_s (finite numeric scalar)
%   - options (scalar struct)
%       Reserved for the shared visibility-call signature.
%**************************************************************************
% OUTPUTS
%   - visible (N-by-1 logical vector)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
segmentCount = size(firstPosition_deg, 1);
visible = true(segmentCount, 1);
if segmentCount == 0
    return;
end
packedObstacles = obstacleField.Obstacles;
for segmentIndex = 1:segmentCount
    segment_deg = [ ...
        firstPosition_deg(segmentIndex, :); ...
        secondPosition_deg(segmentIndex, :)];
    if norm(diff(segment_deg, 1, 1)) <= 1e-12
        continue;
    end
    for obstacleIndex = 1:numel(packedObstacles)
        segmentOccupied = azElInternal.queryPackedMovingObstacle( ...
            packedObstacles(obstacleIndex), sampleTime_s, segment_deg, ...
            false, 0);
        if any(segmentOccupied)
            visible(segmentIndex) = false;
            break;
        end
    end
end
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

