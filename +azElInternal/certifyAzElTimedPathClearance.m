function [provenClear, diagnostics] = certifyAzElTimedPathClearance( ...
        obstacleField, time_s, position_deg, clearanceBuffer_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [provenClear, diagnostics] = ...
%       azElInternal.certifyAzElTimedPathClearance( ...
%       obstacleField, time_s, position_deg, clearanceBuffer_deg)
%**************************************************************************
% PURPOSE
%   - Prove that each timed path chord stays farther than a specified
%     spatial buffer from every linearly moving polygon edge.
%   - Return an unresolved result when obstacle topology changes or the
%     conservative subdivision limit cannot prove separation.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%       Protected static or time-varying polygon geometry.
%   - time_s (N-element numeric vector)
%       Finite, nondecreasing path times.
%   - position_deg (N-by-2 numeric array)
%       Linear path-chord endpoints in [azimuth elevation] order.
%   - clearanceBuffer_deg (scalar or N-1 element numeric vector)
%       Required nonnegative distance from each path chord to all edges.
%**************************************************************************
% OUTPUTS
%   - provenClear (logical scalar)
%       True only when all edge-distance intervals have a positive proof.
%   - diagnostics (scalar struct)
%       Stable subdivision counts and unresolved path-segment information.
%**************************************************************************
% UNITS
%   - Position and clearance use degrees. Time uses seconds.
%**************************************************************************

%% Section 1: Validate The Packed Path And Buffer

if ~isstruct(obstacleField) || ~isscalar(obstacleField) || ...
        ~isfield(obstacleField, "Format") || ...
        string(obstacleField.Format) ~= "AzElTimeObstacleField"
    error("certifyAzElTimedPathClearance:InvalidObstacleField", ...
        "obstacleField must be a packed AzElTimeObstacleField scalar.");
end
validateattributes(time_s, {'numeric'}, ...
    {'real','finite','vector','nonempty'});
validateattributes(position_deg, {'numeric'}, ...
    {'real','finite','2d','ncols',2,'nonempty'});
time_s = double(time_s(:));
position_deg = double(position_deg);
if numel(time_s) ~= size(position_deg, 1)
    error("certifyAzElTimedPathClearance:PathSizeMismatch", ...
        "time_s must contain one value for each position_deg row.");
end
if any(diff(time_s) < 0)
    error("certifyAzElTimedPathClearance:TimeNotNondecreasing", ...
        "time_s must be nondecreasing.");
end
pathSegmentCount = max(0, numel(time_s) - 1);
validateattributes(clearanceBuffer_deg, {'numeric'}, ...
    {'real','finite','nonnegative','vector'});
clearanceBuffer_deg = double(clearanceBuffer_deg(:));
if isscalar(clearanceBuffer_deg)
    clearanceBuffer_deg = repmat( ...
        clearanceBuffer_deg, pathSegmentCount, 1);
elseif numel(clearanceBuffer_deg) ~= pathSegmentCount
    error("certifyAzElTimedPathClearance:BufferSizeMismatch", ...
        "clearanceBuffer_deg must be scalar or contain %d values.", ...
        pathSegmentCount);
end

%% Section 2: Prove Clearance Against Each Moving Edge

maximumSubdivisionDepth = 18;
maximumBoxesPerEdge = 20000;
unresolvedPathSegmentMask = false(pathSegmentCount, 1);
topologyMismatchIntervalCount = 0;
evaluatedEdgeIntervalCount = 0;
broadPhaseRejectedEdgeIntervalCount = 0;
subdivisionBoxCount = 0;
maximumDepthReached = 0;
minimumProvenLowerBound_deg = Inf;
packedObstacles = obstacleField.Obstacles;

for pathSegmentIndex = 1:pathSegmentCount
    segmentStartTime_s = time_s(pathSegmentIndex);
    segmentEndTime_s = time_s(pathSegmentIndex + 1);
    segmentStart_deg = position_deg(pathSegmentIndex, :);
    segmentEnd_deg = position_deg(pathSegmentIndex + 1, :);
    requiredClearance_deg = clearanceBuffer_deg(pathSegmentIndex);
    for obstacleIndex = 1:numel(packedObstacles)
        packedObstacle = packedObstacles(obstacleIndex);
        obstacleTime_s = double(packedObstacle.TimeSeconds(:));
        overlapStart_s = max(segmentStartTime_s, obstacleTime_s(1));
        overlapEnd_s = min(segmentEndTime_s, obstacleTime_s(end));
        if overlapEnd_s < overlapStart_s
            continue;
        end
        interiorTimes_s = obstacleTime_s( ...
            obstacleTime_s > overlapStart_s & ...
            obstacleTime_s < overlapEnd_s);
        intervalTimes_s = unique([overlapStart_s; interiorTimes_s; ...
            overlapEnd_s]);
        if isscalar(intervalTimes_s)
            intervalTimes_s = repmat(intervalTimes_s, 2, 1);
        end
        for intervalIndex = 1:numel(intervalTimes_s) - 1
            intervalTime_s = intervalTimes_s( ...
                intervalIndex:intervalIndex + 1);
            intervalPosition_deg = interpolatePathSegment( ...
                [segmentStartTime_s; segmentEndTime_s], ...
                [segmentStart_deg; segmentEnd_deg], intervalTime_s);
            [firstEdgeSet, lastEdgeSet, topologyMatches] = ...
                intervalEdgeSets(packedObstacle, intervalTime_s);
            edgeSetsMatch = topologyMatches && ...
                isequal(size(firstEdgeSet), size(lastEdgeSet));
            if ~edgeSetsMatch
                [mismatchIsClear, mismatchDetails] = ...
                    proveMismatchedTopologyClearance( ...
                    intervalPosition_deg, firstEdgeSet, lastEdgeSet, ...
                    requiredClearance_deg, maximumSubdivisionDepth, ...
                    maximumBoxesPerEdge);
                evaluatedEdgeIntervalCount = evaluatedEdgeIntervalCount + ...
                    mismatchDetails.EvaluatedEdgeIntervalCount;
                broadPhaseRejectedEdgeIntervalCount = ...
                    broadPhaseRejectedEdgeIntervalCount + ...
                    mismatchDetails.BroadPhaseRejectedEdgeIntervalCount;
                subdivisionBoxCount = subdivisionBoxCount + ...
                    mismatchDetails.SubdivisionBoxCount;
                maximumDepthReached = max(maximumDepthReached, ...
                    mismatchDetails.MaximumDepthReached);
                minimumProvenLowerBound_deg = min( ...
                    minimumProvenLowerBound_deg, ...
                    mismatchDetails.MinimumProvenLowerBound_deg);
                if ~mismatchIsClear
                    topologyMismatchIntervalCount = ...
                        topologyMismatchIntervalCount + 1;
                    unresolvedPathSegmentMask(pathSegmentIndex) = true;
                    break;
                end
                continue;
            end
            [candidateEdgeMask, broadPhaseLowerBound_deg] = ...
                sweptEdgeCandidateMask(intervalPosition_deg, ...
                firstEdgeSet, lastEdgeSet, requiredClearance_deg);
            broadPhaseRejectedEdgeIntervalCount = ...
                broadPhaseRejectedEdgeIntervalCount + ...
                nnz(~candidateEdgeMask);
            if any(~candidateEdgeMask)
                minimumProvenLowerBound_deg = min( ...
                    minimumProvenLowerBound_deg, ...
                    min(broadPhaseLowerBound_deg(~candidateEdgeMask)));
            end
            candidateEdgeIndex = find(candidateEdgeMask);
            for candidateIndex = 1:numel(candidateEdgeIndex)
                edgeIndex = candidateEdgeIndex(candidateIndex);
                evaluatedEdgeIntervalCount = ...
                    evaluatedEdgeIntervalCount + 1;
                [edgeIsClear, edgeDetails] = proveEdgeClearance( ...
                    intervalPosition_deg, firstEdgeSet(edgeIndex, :), ...
                    lastEdgeSet(edgeIndex, :), requiredClearance_deg, ...
                    maximumSubdivisionDepth, maximumBoxesPerEdge);
                subdivisionBoxCount = subdivisionBoxCount + ...
                    edgeDetails.SubdivisionBoxCount;
                maximumDepthReached = max(maximumDepthReached, ...
                    edgeDetails.MaximumDepthReached);
                minimumProvenLowerBound_deg = min( ...
                    minimumProvenLowerBound_deg, ...
                    edgeDetails.MinimumProvenLowerBound_deg);
                if ~edgeIsClear
                    unresolvedPathSegmentMask(pathSegmentIndex) = true;
                    break;
                end
            end
            if unresolvedPathSegmentMask(pathSegmentIndex)
                break;
            end
        end
        if unresolvedPathSegmentMask(pathSegmentIndex)
            break;
        end
    end
end

%% Section 3: Assemble Stable Diagnostics

provenClear = ~any(unresolvedPathSegmentMask);
if isinf(minimumProvenLowerBound_deg)
    minimumProvenLowerBound_deg = NaN;
end
diagnostics = struct( ...
    "ProvenClear", provenClear, ...
    "UnresolvedPathSegmentMask", unresolvedPathSegmentMask, ...
    "UnresolvedPathSegmentCount", nnz(unresolvedPathSegmentMask), ...
    "TopologyMismatchIntervalCount", topologyMismatchIntervalCount, ...
    "EvaluatedEdgeIntervalCount", evaluatedEdgeIntervalCount, ...
    "BroadPhaseRejectedEdgeIntervalCount", ...
    broadPhaseRejectedEdgeIntervalCount, ...
    "SubdivisionBoxCount", subdivisionBoxCount, ...
    "MaximumDepthReached", maximumDepthReached, ...
    "MaximumSubdivisionDepth", maximumSubdivisionDepth, ...
    "MaximumBoxesPerEdge", maximumBoxesPerEdge, ...
    "MinimumProvenLowerBound_deg", minimumProvenLowerBound_deg);
end

%% Section 4: Local Functions

function positionAtTime_deg = interpolatePathSegment( ...
        segmentTime_s, segmentPosition_deg, queryTime_s)
% PURPOSE
%   - Interpolate one linear path chord at requested times.
if segmentTime_s(2) <= segmentTime_s(1)
    positionAtTime_deg = repmat( ...
        segmentPosition_deg(1, :), numel(queryTime_s), 1);
    return;
end
fraction = (queryTime_s(:) - segmentTime_s(1)) / ...
    (segmentTime_s(2) - segmentTime_s(1));
positionAtTime_deg = segmentPosition_deg(1, :) + fraction .* ...
    (segmentPosition_deg(2, :) - segmentPosition_deg(1, :));
end

function [firstEdgeSet, lastEdgeSet, topologyMatches] = ...
        intervalEdgeSets(packedObstacle, intervalTime_s)
% PURPOSE
%   - Interpolate corresponding edge rows at an obstacle-time subinterval.
obstacleTime_s = double(packedObstacle.TimeSeconds(:));
middleTime_s = mean(intervalTime_s);
lowerSliceIndex = find(obstacleTime_s <= middleTime_s, 1, "last");
upperSliceIndex = find(obstacleTime_s >= middleTime_s, 1, "first");
if isempty(lowerSliceIndex)
    lowerSliceIndex = 1;
end
if isempty(upperSliceIndex)
    upperSliceIndex = numel(obstacleTime_s);
end
if lowerSliceIndex == upperSliceIndex
    firstEdgeSet = packedSliceEdges(packedObstacle, lowerSliceIndex);
    lastEdgeSet = firstEdgeSet;
    topologyMatches = true;
    return;
end
topologyMatches = ...
    numel(packedObstacle.TopologyMatchesNext) >= lowerSliceIndex && ...
    logical(packedObstacle.TopologyMatchesNext(lowerSliceIndex));
if ~topologyMatches
    firstEdgeSet = packedSliceEdges(packedObstacle, lowerSliceIndex);
    lastEdgeSet = packedSliceEdges(packedObstacle, upperSliceIndex);
    return;
end
lowerEdges = packedSliceEdges(packedObstacle, lowerSliceIndex);
upperEdges = packedSliceEdges(packedObstacle, upperSliceIndex);
if ~isequal(size(lowerEdges), size(upperEdges))
    firstEdgeSet = lowerEdges;
    lastEdgeSet = upperEdges;
    topologyMatches = false;
    return;
end
timeWidth_s = obstacleTime_s(upperSliceIndex) - ...
    obstacleTime_s(lowerSliceIndex);
firstFraction = (intervalTime_s(1) - ...
    obstacleTime_s(lowerSliceIndex)) / timeWidth_s;
lastFraction = (intervalTime_s(2) - ...
    obstacleTime_s(lowerSliceIndex)) / timeWidth_s;
firstEdgeSet = lowerEdges + firstFraction * (upperEdges - lowerEdges);
lastEdgeSet = lowerEdges + lastFraction * (upperEdges - lowerEdges);
end

function [isClear, diagnostics] = proveMismatchedTopologyClearance( ...
        pathPosition_deg, firstEdgeSet, lastEdgeSet, ...
        requiredClearance_deg, maximumDepth, maximumBoxCount)
% PURPOSE
%   - Prove clearance from the union of both topology endpoint polygons.
if isempty(firstEdgeSet) && isempty(lastEdgeSet)
    isClear = true;
    diagnostics = struct( ...
        "EvaluatedEdgeIntervalCount", 0, ...
        "BroadPhaseRejectedEdgeIntervalCount", 0, ...
        "SubdivisionBoxCount", 0, ...
        "MaximumDepthReached", 0, ...
        "MinimumProvenLowerBound_deg", Inf);
    return;
end
endpointEdgeSet = [firstEdgeSet; lastEdgeSet];
evaluatedEdgeIntervalCount = 0;
subdivisionBoxCount = 0;
maximumDepthReached = 0;
minimumProvenLowerBound_deg = Inf;
isClear = true;
[candidateEdgeMask, broadPhaseLowerBound_deg] = ...
    sweptEdgeCandidateMask(pathPosition_deg, endpointEdgeSet, ...
    endpointEdgeSet, requiredClearance_deg);
broadPhaseRejectedEdgeIntervalCount = nnz(~candidateEdgeMask);
if any(~candidateEdgeMask)
    minimumProvenLowerBound_deg = min( ...
        broadPhaseLowerBound_deg(~candidateEdgeMask));
end
candidateEdgeIndex = find(candidateEdgeMask);
for candidateIndex = 1:numel(candidateEdgeIndex)
    edgeIndex = candidateEdgeIndex(candidateIndex);
    evaluatedEdgeIntervalCount = evaluatedEdgeIntervalCount + 1;
    [edgeIsClear, edgeDetails] = proveEdgeClearance( ...
        pathPosition_deg, endpointEdgeSet(edgeIndex, :), ...
        endpointEdgeSet(edgeIndex, :), requiredClearance_deg, ...
        maximumDepth, maximumBoxCount);
    subdivisionBoxCount = subdivisionBoxCount + ...
        edgeDetails.SubdivisionBoxCount;
    maximumDepthReached = max(maximumDepthReached, ...
        edgeDetails.MaximumDepthReached);
    minimumProvenLowerBound_deg = min( ...
        minimumProvenLowerBound_deg, ...
        edgeDetails.MinimumProvenLowerBound_deg);
    if ~edgeIsClear
        isClear = false;
        break;
    end
end
diagnostics = struct( ...
    "EvaluatedEdgeIntervalCount", evaluatedEdgeIntervalCount, ...
    "BroadPhaseRejectedEdgeIntervalCount", ...
    broadPhaseRejectedEdgeIntervalCount, ...
    "SubdivisionBoxCount", subdivisionBoxCount, ...
    "MaximumDepthReached", maximumDepthReached, ...
    "MinimumProvenLowerBound_deg", minimumProvenLowerBound_deg);
end

function [candidateEdgeMask, distanceLowerBound_deg] = ...
        sweptEdgeCandidateMask(pathPosition_deg, firstEdgeSet, ...
        lastEdgeSet, requiredClearance_deg)
% PURPOSE
%   - Reject swept edges whose bounding boxes prove required separation.
edgeCount = size(firstEdgeSet, 1);
if edgeCount == 0
    candidateEdgeMask = false(0, 1);
    distanceLowerBound_deg = zeros(0, 1);
    return;
end
pathMinimum_deg = min(pathPosition_deg, [], 1);
pathMaximum_deg = max(pathPosition_deg, [], 1);
edgeAzimuth_deg = [firstEdgeSet(:, [1 3]), ...
    lastEdgeSet(:, [1 3])];
edgeElevation_deg = [firstEdgeSet(:, [2 4]), ...
    lastEdgeSet(:, [2 4])];
edgeMinimum_deg = [min(edgeAzimuth_deg, [], 2), ...
    min(edgeElevation_deg, [], 2)];
edgeMaximum_deg = [max(edgeAzimuth_deg, [], 2), ...
    max(edgeElevation_deg, [], 2)];
lowerGap_deg = pathMinimum_deg - edgeMaximum_deg;
upperGap_deg = edgeMinimum_deg - pathMaximum_deg;
componentGap_deg = max(cat(3, lowerGap_deg, upperGap_deg, ...
    zeros(edgeCount, 2)), [], 3);
distanceLowerBound_deg = hypot( ...
    componentGap_deg(:, 1), componentGap_deg(:, 2));
coordinateScale_deg = max(1, max(abs([pathPosition_deg(:); ...
    firstEdgeSet(:); lastEdgeSet(:)])));
numericalTolerance_deg = 100 * eps(coordinateScale_deg);
candidateEdgeMask = distanceLowerBound_deg <= ...
    requiredClearance_deg + numericalTolerance_deg;
end

function edgeSet = packedSliceEdges(packedObstacle, sliceIndex)
% PURPOSE
%   - Extract one packed slice edge matrix.
firstEdgeIndex = double(packedObstacle.EdgeOffsets(sliceIndex));
lastEdgeIndex = double(packedObstacle.EdgeOffsets(sliceIndex + 1)) - 1;
if lastEdgeIndex < firstEdgeIndex
    edgeSet = zeros(0, 4);
    return;
end
edgeSet = [ ...
    double(packedObstacle.EdgeStartAzimuthDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeStartElevationDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeEndAzimuthDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeEndElevationDeg( ...
    firstEdgeIndex:lastEdgeIndex))];
end

function [provenClear, diagnostics] = proveEdgeClearance( ...
        pathPosition_deg, firstEdge_deg, lastEdge_deg, ...
        requiredClearance_deg, maximumDepth, maximumBoxCount)
% PURPOSE
%   - Bound one bilinear timed point-to-edge distance by subdivision.
boxStack = zeros(maximumBoxCount, 5);
stackCount = 1;
boxStack(1, :) = [0 1 0 1 0];
processedBoxCount = 0;
maximumDepthReached = 0;
minimumProvenLowerBound_deg = Inf;
provenClear = true;
while stackCount > 0
    box = boxStack(stackCount, :);
    stackCount = stackCount - 1;
    processedBoxCount = processedBoxCount + 1;
    timeBounds = box(1:2);
    edgeBounds = box(3:4);
    depth = box(5);
    maximumDepthReached = max(maximumDepthReached, depth);
    relativeCorner_deg = relativeCorners( ...
        pathPosition_deg, firstEdge_deg, lastEdge_deg, ...
        timeBounds, edgeBounds);
    componentLower_deg = min(relativeCorner_deg, [], 1);
    componentUpper_deg = max(relativeCorner_deg, [], 1);
    componentDistance_deg = max([componentLower_deg; ...
        -componentUpper_deg; zeros(1, 2)], [], 1);
    distanceLowerBound_deg = norm(componentDistance_deg);
    coordinateScale_deg = max(1, max(abs(relativeCorner_deg), [], "all"));
    numericalTolerance_deg = 100 * eps(coordinateScale_deg);
    if distanceLowerBound_deg > ...
            requiredClearance_deg + numericalTolerance_deg
        minimumProvenLowerBound_deg = min( ...
            minimumProvenLowerBound_deg, distanceLowerBound_deg);
        continue;
    end
    if depth >= maximumDepth || ...
            processedBoxCount + stackCount + 2 > maximumBoxCount
        provenClear = false;
        break;
    end
    timeVariation_deg = max([ ...
        norm(relativeCorner_deg(3, :) - relativeCorner_deg(1, :)), ...
        norm(relativeCorner_deg(4, :) - relativeCorner_deg(2, :))]);
    edgeVariation_deg = max([ ...
        norm(relativeCorner_deg(2, :) - relativeCorner_deg(1, :)), ...
        norm(relativeCorner_deg(4, :) - relativeCorner_deg(3, :))]);
    if timeVariation_deg >= edgeVariation_deg
        splitValue = mean(timeBounds);
        firstBox = [timeBounds(1), splitValue, edgeBounds, depth + 1];
        secondBox = [splitValue, timeBounds(2), edgeBounds, depth + 1];
    else
        splitValue = mean(edgeBounds);
        firstBox = [timeBounds, edgeBounds(1), splitValue, depth + 1];
        secondBox = [timeBounds, splitValue, edgeBounds(2), depth + 1];
    end
    boxStack(stackCount + 1, :) = firstBox;
    boxStack(stackCount + 2, :) = secondBox;
    stackCount = stackCount + 2;
end
diagnostics = struct( ...
    "SubdivisionBoxCount", processedBoxCount, ...
    "MaximumDepthReached", maximumDepthReached, ...
    "MinimumProvenLowerBound_deg", minimumProvenLowerBound_deg);
end

function relativeCorner_deg = relativeCorners( ...
        pathPosition_deg, firstEdge_deg, lastEdge_deg, ...
        timeBounds, edgeBounds)
% PURPOSE
%   - Evaluate the bilinear point-to-moving-edge vector at four box corners.
relativeCorner_deg = zeros(4, 2);
cornerIndex = 0;
for timeParameter = timeBounds
    pathPoint_deg = pathPosition_deg(1, :) + timeParameter * ...
        (pathPosition_deg(2, :) - pathPosition_deg(1, :));
    edgeAtTime_deg = firstEdge_deg + timeParameter * ...
        (lastEdge_deg - firstEdge_deg);
    edgeStart_deg = edgeAtTime_deg(1:2);
    edgeEnd_deg = edgeAtTime_deg(3:4);
    for edgeParameter = edgeBounds
        cornerIndex = cornerIndex + 1;
        edgePoint_deg = edgeStart_deg + edgeParameter * ...
            (edgeEnd_deg - edgeStart_deg);
        relativeCorner_deg(cornerIndex, :) = ...
            pathPoint_deg - edgePoint_deg;
    end
end
end
