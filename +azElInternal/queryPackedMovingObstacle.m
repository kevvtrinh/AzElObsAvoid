function [isOccupied, isInsideBounds, blockingSliceIndex] = ...
        queryPackedMovingObstacle(packedObstacle, time_s, position_deg, ...
        boundaryIsOccupied, timePaddingSamples)
%% Section 0: Header & Readme
% SYNTAX
%   [isOccupied, isInsideBounds, blockingSliceIndex] = ...
%       azElInternal.queryPackedMovingObstacle( ...
%       packedObstacle, time_s, position_deg, ...
%       boundaryIsOccupied, timePaddingSamples)
%**************************************************************************
% PURPOSE
%   - Check a point or one linear timed segment against a polygon whose
%     corresponding vertices move linearly between source samples.
%   - Treat topology changes as discrete source geometry and check both
%     adjacent shapes over a crossing interval.
%**************************************************************************
% INPUTS
%   - packedObstacle (scalar packed-obstacle struct)
%   - time_s (scalar or two-element finite numeric vector)
%       One query time or the endpoints of a nondecreasing time interval.
%   - position_deg (1-by-2 or 2-by-2 finite numeric matrix)
%       Point or linear segment in [azimuth elevation] order.
%   - boundaryIsOccupied (logical scalar)
%   - timePaddingSamples (nonnegative integer scalar)
%       Neighboring source slices checked as additional static uncertainty.
%**************************************************************************
% OUTPUTS
%   - isOccupied, isInsideBounds (logical scalars)
%       Polygon and broad-phase occupancy over the complete query.
%   - blockingSliceIndex (nonnegative integer scalar)
%       First source interval or padded slice that blocks, or zero.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Validate The Query

validateattributes(time_s, {'numeric'}, ...
    {'real', 'finite', 'vector', 'nonempty'});
validateattributes(position_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
validateattributes(timePaddingSamples, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
time_s = double(time_s(:));
position_deg = double(position_deg);
if isscalar(time_s)
    if size(position_deg, 1) ~= 1
        error("queryPackedMovingObstacle:PointSizeMismatch", ...
            "A scalar time requires one position row.");
    end
elseif numel(time_s) == 2
    if size(position_deg, 1) ~= 2 || time_s(2) < time_s(1)
        error("queryPackedMovingObstacle:SegmentSizeMismatch", ...
            "Two times require two position rows and nondecreasing time.");
    end
else
    error("queryPackedMovingObstacle:InvalidTimeCount", ...
        "time_s must contain one point time or two segment endpoint times.");
end
boundaryIsOccupied = logical(boundaryIsOccupied);

%% Section 2: Check A Point Or Timed Segment

isOccupied = false;
isInsideBounds = false;
blockingSliceIndex = 0;
if packedObstacle.SampleCount < 1
    return;
end
obstacleTime_s = double(packedObstacle.TimeSeconds(:));
if time_s(end) < obstacleTime_s(1) || time_s(1) > obstacleTime_s(end)
    return;
end

if isscalar(time_s)
    [edgeSets, sourceSliceIndex] = edgeSetsAtTime( ...
        packedObstacle, time_s, timePaddingSamples);
    for edgeSetIndex = 1:numel(edgeSets)
        edgeSet = edgeSets{edgeSetIndex};
        if isempty(edgeSet)
            continue;
        end
        bounds_deg = [min(edgeSet(:, [1 3]), [], "all"), ...
            max(edgeSet(:, [1 3]), [], "all"), ...
            min(edgeSet(:, [2 4]), [], "all"), ...
            max(edgeSet(:, [2 4]), [], "all")];
        insideBounds = pointInsideBounds(position_deg, bounds_deg);
        isInsideBounds = isInsideBounds || insideBounds;
        if insideBounds && pointOccupiedByEdges( ...
                position_deg, edgeSet, boundaryIsOccupied)
            isOccupied = true;
            blockingSliceIndex = sourceSliceIndex(edgeSetIndex);
            return;
        end
    end
    return;
end

overlapStart_s = max(time_s(1), obstacleTime_s(1));
overlapEnd_s = min(time_s(2), obstacleTime_s(end));
if overlapStart_s > overlapEnd_s
    return;
end
if time_s(2) == time_s(1)
    [edgeSets, sourceSliceIndex] = edgeSetsAtTime( ...
        packedObstacle, time_s(1), timePaddingSamples);
    for edgeSetIndex = 1:numel(edgeSets)
        [blocked, boundsHit] = staticSegmentHitsEdges( ...
            position_deg, edgeSets{edgeSetIndex}, boundaryIsOccupied);
        isInsideBounds = isInsideBounds || boundsHit;
        if blocked
            isOccupied = true;
            blockingSliceIndex = sourceSliceIndex(edgeSetIndex);
            return;
        end
    end
    return;
end
breakTime_s = unique([overlapStart_s; ...
    obstacleTime_s(obstacleTime_s > overlapStart_s & ...
    obstacleTime_s < overlapEnd_s); overlapEnd_s]);
if isscalar(breakTime_s)
    breakTime_s = [breakTime_s; breakTime_s];
end

for intervalIndex = 1:numel(breakTime_s) - 1
    intervalTime_s = breakTime_s(intervalIndex:intervalIndex + 1);
    intervalPosition_deg = interpolateSegmentPosition( ...
        time_s, position_deg, intervalTime_s);
    [firstEdgeSet, firstSliceIndex, topologyMatches] = ...
        primaryEdgesAtTime(packedObstacle, intervalTime_s(1));
    [lastEdgeSet, ~, lastTopologyMatches] = ...
        primaryEdgesAtTime(packedObstacle, intervalTime_s(2));
    topologyMatches = topologyMatches && lastTopologyMatches && ...
        isequal(size(firstEdgeSet), size(lastEdgeSet));
    if topologyMatches
        [blocked, boundsHit] = linearSegmentHitsMovingEdges( ...
            intervalPosition_deg, firstEdgeSet, lastEdgeSet, ...
            boundaryIsOccupied);
    else
        [blockedAtFirstShape, firstBoundsHit] = ...
            linearSegmentHitsMovingEdges(intervalPosition_deg, ...
            firstEdgeSet, firstEdgeSet, boundaryIsOccupied);
        [blockedAtLastShape, lastBoundsHit] = ...
            linearSegmentHitsMovingEdges(intervalPosition_deg, ...
            lastEdgeSet, lastEdgeSet, boundaryIsOccupied);
        blocked = blockedAtFirstShape || blockedAtLastShape;
        boundsHit = firstBoundsHit || lastBoundsHit;
    end
    isInsideBounds = isInsideBounds || boundsHit;
    if blocked
        isOccupied = true;
        blockingSliceIndex = firstSliceIndex;
        return;
    end

    if timePaddingSamples <= 0
        continue;
    end
    middleTime_s = mean(intervalTime_s);
    [lowerSliceIndex, upperSliceIndex] = ...
        bracketingSliceIndices(obstacleTime_s, middleTime_s);
    paddedSliceIndex = unique(max(1, lowerSliceIndex - ...
        timePaddingSamples):min(packedObstacle.SampleCount, ...
        upperSliceIndex + timePaddingSamples));
    for sliceIndex = paddedSliceIndex
        staticEdges = packedSliceEdges(packedObstacle, sliceIndex);
        [blocked, boundsHit] = linearSegmentHitsMovingEdges( ...
            intervalPosition_deg, staticEdges, staticEdges, ...
            boundaryIsOccupied);
        isInsideBounds = isInsideBounds || boundsHit;
        if blocked
            isOccupied = true;
            blockingSliceIndex = sliceIndex;
            return;
        end
    end
end
end

%% Section 3: Local Functions

function [edgeSets, sourceSliceIndex] = edgeSetsAtTime( ...
        packedObstacle, queryTime_s, timePaddingSamples)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeSets, sourceSliceIndex] = edgeSetsAtTime( ...
%       packedObstacle, queryTime_s, timePaddingSamples)
%**************************************************************************
% PURPOSE
%   - Return the interpolated polygon and requested neighboring slices.
%**************************************************************************
% INPUTS
%   - packedObstacle (scalar struct), queryTime_s (scalar seconds)
%   - timePaddingSamples (nonnegative integer scalar)
%**************************************************************************
% OUTPUTS
%   - edgeSets (cell array), sourceSliceIndex (integer column)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
[primaryEdges, lowerSliceIndex] = ...
    primaryEdgesAtTime(packedObstacle, queryTime_s);
edgeSets = {primaryEdges};
sourceSliceIndex = lowerSliceIndex;
if timePaddingSamples <= 0
    return;
end
obstacleTime_s = double(packedObstacle.TimeSeconds(:));
[lowerSliceIndex, upperSliceIndex] = ...
    bracketingSliceIndices(obstacleTime_s, queryTime_s);
paddedSliceIndex = unique(max(1, lowerSliceIndex - ...
    timePaddingSamples):min(packedObstacle.SampleCount, ...
    upperSliceIndex + timePaddingSamples));
for sliceIndex = paddedSliceIndex
    staticEdges = packedSliceEdges(packedObstacle, sliceIndex);
    if isequal(staticEdges, primaryEdges)
        continue;
    end
    edgeSets{end + 1, 1} = staticEdges; %#ok<AGROW>
    sourceSliceIndex(end + 1, 1) = sliceIndex; %#ok<AGROW>
end
end

function [edgeSet, lowerSliceIndex, topologyMatches] = ...
        primaryEdgesAtTime(packedObstacle, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [edgeSet, lowerSliceIndex, topologyMatches] = ...
%       primaryEdgesAtTime(packedObstacle, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Interpolate corresponding vertices or select the nearest discrete
%     source shape when polygon topology changes.
%**************************************************************************
% INPUTS
%   - packedObstacle (scalar struct), queryTime_s (scalar seconds)
%**************************************************************************
% OUTPUTS
%   - edgeSet (N-by-4), lowerSliceIndex (integer)
%   - topologyMatches (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
obstacleTime_s = double(packedObstacle.TimeSeconds(:));
[lowerSliceIndex, upperSliceIndex, fraction] = ...
    bracketingSliceIndices(obstacleTime_s, queryTime_s);
if lowerSliceIndex == upperSliceIndex
    [vertices_deg, finiteVertex] = ...
        packedSliceVertices(packedObstacle, lowerSliceIndex);
    edgeSet = edgesFromSeparatedVertices(vertices_deg, finiteVertex);
    topologyMatches = true;
    return;
end
[firstVertices_deg, firstFinite] = ...
    packedSliceVertices(packedObstacle, lowerSliceIndex);
[lastVertices_deg, lastFinite] = ...
    packedSliceVertices(packedObstacle, upperSliceIndex);
topologyMatches = isequal(size(firstVertices_deg), ...
    size(lastVertices_deg)) && isequal(firstFinite, lastFinite);
if topologyMatches
    interpolatedVertices_deg = firstVertices_deg;
    interpolatedVertices_deg(firstFinite, :) = ...
        (1 - fraction) * firstVertices_deg(firstFinite, :) + ...
        fraction * lastVertices_deg(lastFinite, :);
    edgeSet = edgesFromSeparatedVertices( ...
        interpolatedVertices_deg, firstFinite);
else
    if fraction < 0.5
        edgeSet = edgesFromSeparatedVertices( ...
            firstVertices_deg, firstFinite);
    else
        edgeSet = edgesFromSeparatedVertices( ...
            lastVertices_deg, lastFinite);
    end
end
end

function [lowerSliceIndex, upperSliceIndex, fraction] = ...
        bracketingSliceIndices(obstacleTime_s, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [lowerSliceIndex, upperSliceIndex, fraction] = ...
%       bracketingSliceIndices(obstacleTime_s, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Locate the closed source-time interval containing one query.
%**************************************************************************
% INPUTS
%   - obstacleTime_s (strictly increasing column), queryTime_s (scalar)
%**************************************************************************
% OUTPUTS
%   - lowerSliceIndex, upperSliceIndex (integers), fraction (0 through 1)
%**************************************************************************
% UNITS
%   - Input time is seconds; fraction is dimensionless.
%**************************************************************************
upperSliceIndex = find(obstacleTime_s >= queryTime_s, 1, "first");
if isempty(upperSliceIndex)
    upperSliceIndex = numel(obstacleTime_s);
end
lowerSliceIndex = find(obstacleTime_s <= queryTime_s, 1, "last");
if isempty(lowerSliceIndex)
    lowerSliceIndex = 1;
end
if lowerSliceIndex == upperSliceIndex
    fraction = 0;
else
    fraction = (queryTime_s - obstacleTime_s(lowerSliceIndex)) / ...
        (obstacleTime_s(upperSliceIndex) - ...
        obstacleTime_s(lowerSliceIndex));
    fraction = min(1, max(0, fraction));
end
end

function [vertices_deg, finiteVertex] = ...
        packedSliceVertices(packedObstacle, sliceIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [vertices_deg, finiteVertex] = ...
%       packedSliceVertices(packedObstacle, sliceIndex)
%**************************************************************************
% PURPOSE
%   - Recover one NaN-separated packed polygon slice.
%**************************************************************************
% INPUTS
%   - packedObstacle (scalar struct), sliceIndex (positive integer)
%**************************************************************************
% OUTPUTS
%   - vertices_deg (N-by-2), finiteVertex (N-by-1 logical)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
firstVertexIndex = double(packedObstacle.SliceOffsets(sliceIndex));
lastVertexIndex = double( ...
    packedObstacle.SliceOffsets(sliceIndex + 1)) - 1;
if lastVertexIndex < firstVertexIndex
    vertices_deg = zeros(0, 2);
    finiteVertex = false(0, 1);
    return;
end
vertices_deg = [double(packedObstacle.AzimuthDeg( ...
    firstVertexIndex:lastVertexIndex)), ...
    double(packedObstacle.ElevationDeg( ...
    firstVertexIndex:lastVertexIndex))];
finiteVertex = all(isfinite(vertices_deg), 2);
end

function edgeSet = packedSliceEdges(packedObstacle, sliceIndex)
%% Section 0: Header & Readme
% SYNTAX
%   edgeSet = packedSliceEdges(packedObstacle, sliceIndex)
%**************************************************************************
% PURPOSE
%   - Recover one packed slice as [startAz startEl endAz endEl] rows.
%**************************************************************************
% INPUTS
%   - packedObstacle (scalar struct), sliceIndex (positive integer)
%**************************************************************************
% OUTPUTS
%   - edgeSet (N-by-4 numeric matrix)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
firstEdgeIndex = double(packedObstacle.EdgeOffsets(sliceIndex));
lastEdgeIndex = double( ...
    packedObstacle.EdgeOffsets(sliceIndex + 1)) - 1;
if lastEdgeIndex < firstEdgeIndex
    edgeSet = zeros(0, 4);
    return;
end
edgeSet = [double(packedObstacle.EdgeStartAzimuthDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeStartElevationDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeEndAzimuthDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeEndElevationDeg( ...
    firstEdgeIndex:lastEdgeIndex))];
end

function edgeSet = edgesFromSeparatedVertices(vertices_deg, finiteVertex)
%% Section 0: Header & Readme
% SYNTAX
%   edgeSet = edgesFromSeparatedVertices(vertices_deg, finiteVertex)
%**************************************************************************
% PURPOSE
%   - Convert NaN-separated polygon rings to an explicit edge matrix.
%**************************************************************************
% INPUTS
%   - vertices_deg (N-by-2), finiteVertex (N-by-1 logical)
%**************************************************************************
% OUTPUTS
%   - edgeSet (M-by-4 numeric matrix)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
transitions = diff([false; finiteVertex; false]);
ringStart = find(transitions == 1);
ringStop = find(transitions == -1) - 1;
edgeSet = zeros(0, 4);
for ringIndex = 1:numel(ringStart)
    ring = vertices_deg(ringStart(ringIndex):ringStop(ringIndex), :);
    if size(ring, 1) < 3
        continue;
    end
    nextRing = ring([2:end 1], :);
    edgeSet = [edgeSet; ring, nextRing]; %#ok<AGROW>
end
end

function positionAtTime_deg = interpolateSegmentPosition( ...
        segmentTime_s, segmentPosition_deg, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   positionAtTime_deg = interpolateSegmentPosition( ...
%       segmentTime_s, segmentPosition_deg, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Evaluate one linear timed path segment at requested times.
%**************************************************************************
% INPUTS
%   - segmentTime_s (2-vector), segmentPosition_deg (2-by-2)
%   - queryTime_s (numeric vector)
%**************************************************************************
% OUTPUTS
%   - positionAtTime_deg (N-by-2 numeric matrix)
%**************************************************************************
% UNITS
%   - Time is seconds and position is degrees.
%**************************************************************************
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

function [isOccupied, boundsHit] = staticSegmentHitsEdges( ...
        segmentPosition_deg, edgeSet, boundaryIsOccupied)
%% Section 0: Header & Readme
% SYNTAX
%   [isOccupied, boundsHit] = staticSegmentHitsEdges( ...
%       segmentPosition_deg, edgeSet, boundaryIsOccupied)
%**************************************************************************
% PURPOSE
%   - Check a spatial segment against one static polygon without invoking
%     the moving-edge polynomial solver.
%**************************************************************************
% INPUTS
%   - segmentPosition_deg (2-by-2), edgeSet (N-by-4 numeric)
%   - boundaryIsOccupied (logical scalar)
%**************************************************************************
% OUTPUTS
%   - isOccupied, boundsHit (logical scalars)
%**************************************************************************
% UNITS
%   - Position is degrees; intersection parameters are dimensionless.
%**************************************************************************
isOccupied = false;
boundsHit = false;
if isempty(edgeSet)
    return;
end
segmentStart_deg = segmentPosition_deg(1, :);
segmentEnd_deg = segmentPosition_deg(2, :);
segmentDelta_deg = segmentEnd_deg - segmentStart_deg;
edgeStart_deg = edgeSet(:, 1:2);
edgeEnd_deg = edgeSet(:, 3:4);
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
polygonBounds_deg = [min(edgeSet(:, [1 3]), [], "all"), ...
    max(edgeSet(:, [1 3]), [], "all"), ...
    min(edgeSet(:, [2 4]), [], "all"), ...
    max(edgeSet(:, [2 4]), [], "all")];
segmentBounds_deg = [min(segmentPosition_deg(:, 1)), ...
    max(segmentPosition_deg(:, 1)), ...
    min(segmentPosition_deg(:, 2)), ...
    max(segmentPosition_deg(:, 2))];
scale_deg = max(1, max(abs([polygonBounds_deg, segmentBounds_deg])));
tolerance_deg = 1e-11 * scale_deg;
boundsHit = segmentBounds_deg(2) >= polygonBounds_deg(1) - tolerance_deg && ...
    segmentBounds_deg(1) <= polygonBounds_deg(2) + tolerance_deg && ...
    segmentBounds_deg(4) >= polygonBounds_deg(3) - tolerance_deg && ...
    segmentBounds_deg(3) <= polygonBounds_deg(4) + tolerance_deg;
if ~boundsHit
    return;
end

segmentLength_deg = norm(segmentDelta_deg);
if segmentLength_deg <= tolerance_deg
    isOccupied = pointOccupiedByEdges( ...
        segmentStart_deg, edgeSet, boundaryIsOccupied);
    return;
end
offset_deg = edgeStart_deg - segmentStart_deg;
denominator_deg2 = segmentDelta_deg(1) .* edgeDelta_deg(:, 2) - ...
    segmentDelta_deg(2) .* edgeDelta_deg(:, 1);
edgeLength_deg = hypot(edgeDelta_deg(:, 1), edgeDelta_deg(:, 2));
denominatorTolerance_deg2 = tolerance_deg .* ...
    max(1, max(segmentLength_deg, edgeLength_deg));
isNonparallel = abs(denominator_deg2) > denominatorTolerance_deg2;
segmentParameter = nan(size(denominator_deg2));
edgeParameter = nan(size(denominator_deg2));
segmentParameter(isNonparallel) = ( ...
    offset_deg(isNonparallel, 1) .* edgeDelta_deg(isNonparallel, 2) - ...
    offset_deg(isNonparallel, 2) .* edgeDelta_deg(isNonparallel, 1)) ./ ...
    denominator_deg2(isNonparallel);
edgeParameter(isNonparallel) = ( ...
    offset_deg(isNonparallel, 1) .* segmentDelta_deg(2) - ...
    offset_deg(isNonparallel, 2) .* segmentDelta_deg(1)) ./ ...
    denominator_deg2(isNonparallel);
parameterTolerance = tolerance_deg / segmentLength_deg;
properContact = isNonparallel & ...
    segmentParameter >= -parameterTolerance & ...
    segmentParameter <= 1 + parameterTolerance & ...
    edgeParameter >= -parameterTolerance & ...
    edgeParameter <= 1 + parameterTolerance;
contactParameter = segmentParameter(properContact);

offsetCrossSegment_deg2 = offset_deg(:, 1) .* segmentDelta_deg(2) - ...
    offset_deg(:, 2) .* segmentDelta_deg(1);
isCollinear = ~isNonparallel & abs(offsetCrossSegment_deg2) <= ...
    tolerance_deg * max(1, segmentLength_deg);
if any(isCollinear)
    segmentLengthSquared_deg2 = dot(segmentDelta_deg, segmentDelta_deg);
    firstCollinearParameter = ...
        offset_deg(isCollinear, :) * segmentDelta_deg.' / ...
        segmentLengthSquared_deg2;
    lastCollinearParameter = (edgeEnd_deg(isCollinear, :) - ...
        segmentStart_deg) * segmentDelta_deg.' / ...
        segmentLengthSquared_deg2;
    overlaps = max(firstCollinearParameter, lastCollinearParameter) >= ...
        -parameterTolerance & min(firstCollinearParameter, ...
        lastCollinearParameter) <= 1 + parameterTolerance;
    contactParameter = [contactParameter; ...
        firstCollinearParameter(overlaps); ...
        lastCollinearParameter(overlaps)];
end
contactParameter = unique(min(1, max(0, contactParameter)));
if boundaryIsOccupied && ~isempty(contactParameter)
    isOccupied = true;
    return;
end
breakParameter = unique([0; contactParameter; 1]);
testParameter = breakParameter;
if numel(breakParameter) > 1
    testParameter = unique([testParameter; ...
        0.5 * (breakParameter(1:end - 1) + breakParameter(2:end))]);
end
for parameter = reshape(testParameter, 1, [])
    point_deg = segmentStart_deg + parameter * segmentDelta_deg;
    if pointOccupiedByEdges(point_deg, edgeSet, boundaryIsOccupied)
        isOccupied = true;
        return;
    end
end
end

function [isOccupied, boundsHit] = linearSegmentHitsMovingEdges( ...
        segmentPosition_deg, firstEdgeSet, lastEdgeSet, ...
        boundaryIsOccupied)
%% Section 0: Header & Readme
% SYNTAX
%   [isOccupied, boundsHit] = linearSegmentHitsMovingEdges( ...
%       segmentPosition_deg, firstEdgeSet, lastEdgeSet, ...
%       boundaryIsOccupied)
%**************************************************************************
% PURPOSE
%   - Exactly partition a linear point/moving-edge interval at every
%     boundary contact and test polygon occupancy between contacts.
%**************************************************************************
% INPUTS
%   - segmentPosition_deg (2-by-2)
%   - firstEdgeSet, lastEdgeSet (matching N-by-4 edge matrices)
%   - boundaryIsOccupied (logical scalar)
%**************************************************************************
% OUTPUTS
%   - isOccupied, boundsHit (logical scalars)
%**************************************************************************
% UNITS
%   - Position is degrees; interval parameters are dimensionless.
%**************************************************************************
isOccupied = false;
boundsHit = false;
if isempty(firstEdgeSet) || ~isequal(size(firstEdgeSet), size(lastEdgeSet))
    return;
end
allCoordinates_deg = [firstEdgeSet(:); lastEdgeSet(:); ...
    segmentPosition_deg(:)];
coordinateScale_deg = max(1, max(abs(allCoordinates_deg)));
tolerance = 1e-11 * coordinateScale_deg;
eventParameter = [0; 1];
pointStart_deg = segmentPosition_deg(1, :);
pointDelta_deg = diff(segmentPosition_deg, 1, 1);
segmentMinimum_deg = min(segmentPosition_deg, [], 1);
segmentMaximum_deg = max(segmentPosition_deg, [], 1);
edgeMinimumAzimuth_deg = min([firstEdgeSet(:, 1), ...
    firstEdgeSet(:, 3), lastEdgeSet(:, 1), lastEdgeSet(:, 3)], [], 2);
edgeMaximumAzimuth_deg = max([firstEdgeSet(:, 1), ...
    firstEdgeSet(:, 3), lastEdgeSet(:, 1), lastEdgeSet(:, 3)], [], 2);
edgeMinimumElevation_deg = min([firstEdgeSet(:, 2), ...
    firstEdgeSet(:, 4), lastEdgeSet(:, 2), lastEdgeSet(:, 4)], [], 2);
edgeMaximumElevation_deg = max([firstEdgeSet(:, 2), ...
    firstEdgeSet(:, 4), lastEdgeSet(:, 2), lastEdgeSet(:, 4)], [], 2);
possibleContact = edgeMaximumAzimuth_deg >= ...
    segmentMinimum_deg(1) - tolerance & ...
    edgeMinimumAzimuth_deg <= segmentMaximum_deg(1) + tolerance & ...
    edgeMaximumElevation_deg >= segmentMinimum_deg(2) - tolerance & ...
    edgeMinimumElevation_deg <= segmentMaximum_deg(2) + tolerance;
rootEdgeIndex = find(possibleContact);
edgeStart0_deg = firstEdgeSet(rootEdgeIndex, 1:2);
edgeEnd0_deg = firstEdgeSet(rootEdgeIndex, 3:4);
edgeStartDelta_deg = lastEdgeSet(rootEdgeIndex, 1:2) - edgeStart0_deg;
edgeEndDelta_deg = lastEdgeSet(rootEdgeIndex, 3:4) - edgeEnd0_deg;
relativeStart_deg = pointStart_deg - edgeStart0_deg;
relativeDelta_deg = pointDelta_deg - edgeStartDelta_deg;
edgeVector0_deg = edgeEnd0_deg - edgeStart0_deg;
edgeVectorDelta_deg = edgeEndDelta_deg - edgeStartDelta_deg;
quadraticCoefficient = ...
    relativeDelta_deg(:, 1) .* edgeVectorDelta_deg(:, 2) - ...
    relativeDelta_deg(:, 2) .* edgeVectorDelta_deg(:, 1);
linearCoefficient = ...
    relativeDelta_deg(:, 1) .* edgeVector0_deg(:, 2) - ...
    relativeDelta_deg(:, 2) .* edgeVector0_deg(:, 1) + ...
    relativeStart_deg(:, 1) .* edgeVectorDelta_deg(:, 2) - ...
    relativeStart_deg(:, 2) .* edgeVectorDelta_deg(:, 1);
constantCoefficient = ...
    relativeStart_deg(:, 1) .* edgeVector0_deg(:, 2) - ...
    relativeStart_deg(:, 2) .* edgeVector0_deg(:, 1);
coefficientScale = max(1, max(abs([quadraticCoefficient, ...
    linearCoefficient, constantCoefficient]), [], 2));
activeTolerance = tolerance .* coefficientScale;
firstRoot = nan(size(quadraticCoefficient));
secondRoot = nan(size(quadraticCoefficient));
isLinear = abs(quadraticCoefficient) <= activeTolerance & ...
    abs(linearCoefficient) > activeTolerance;
firstRoot(isLinear) = -constantCoefficient(isLinear) ./ ...
    linearCoefficient(isLinear);
isQuadratic = abs(quadraticCoefficient) > activeTolerance;
discriminant = linearCoefficient.^2 - ...
    4 * quadraticCoefficient .* constantCoefficient;
hasRealRoots = isQuadratic & discriminant >= -activeTolerance;
rootDiscriminant = sqrt(max(0, discriminant(hasRealRoots)));
firstRoot(hasRealRoots) = (-linearCoefficient(hasRealRoots) - ...
    rootDiscriminant) ./ (2 * quadraticCoefficient(hasRealRoots));
secondRoot(hasRealRoots) = (-linearCoefficient(hasRealRoots) + ...
    rootDiscriminant) ./ (2 * quadraticCoefficient(hasRealRoots));
edgeIndex = (1:numel(rootEdgeIndex)).';
candidateParameter = [firstRoot; secondRoot];
candidateEdgeIndex = [edgeIndex; edgeIndex];
validRoot = isfinite(candidateParameter) & ...
    candidateParameter >= -tolerance & ...
    candidateParameter <= 1 + tolerance;
candidateParameter = min(1, max(0, candidateParameter(validRoot)));
candidateEdgeIndex = candidateEdgeIndex(validRoot);
if ~isempty(candidateParameter)
    candidatePoint_deg = pointStart_deg + ...
        candidateParameter .* pointDelta_deg;
    candidateEdgeStart_deg = edgeStart0_deg(candidateEdgeIndex, :) + ...
        candidateParameter .* edgeStartDelta_deg(candidateEdgeIndex, :);
    candidateEdgeEnd_deg = edgeEnd0_deg(candidateEdgeIndex, :) + ...
        candidateParameter .* edgeEndDelta_deg(candidateEdgeIndex, :);
    candidateEdgeVector_deg = ...
        candidateEdgeEnd_deg - candidateEdgeStart_deg;
    lengthSquared_deg2 = sum(candidateEdgeVector_deg.^2, 2);
    projection = sum((candidatePoint_deg - candidateEdgeStart_deg) .* ...
        candidateEdgeVector_deg, 2) ./ lengthSquared_deg2;
    validContact = lengthSquared_deg2 > tolerance^2 & ...
        projection >= -tolerance & projection <= 1 + tolerance;
    eventParameter = [eventParameter; ...
        candidateParameter(validContact)];
end
eventParameter = unique(min(1, max(0, eventParameter)));
testParameter = eventParameter;
if numel(eventParameter) > 1
    testParameter = unique([testParameter; 0.5 * ( ...
        eventParameter(1:end - 1) + eventParameter(2:end))]);
end
for parameter = reshape(testParameter, 1, [])
    point_deg = pointStart_deg + parameter * pointDelta_deg;
    edgeSet = firstEdgeSet + parameter * ...
        (lastEdgeSet - firstEdgeSet);
    bounds_deg = [min(edgeSet(:, [1 3]), [], "all"), ...
        max(edgeSet(:, [1 3]), [], "all"), ...
        min(edgeSet(:, [2 4]), [], "all"), ...
        max(edgeSet(:, [2 4]), [], "all")];
    insideBounds = pointInsideBounds(point_deg, bounds_deg);
    boundsHit = boundsHit || insideBounds;
    if insideBounds && pointOccupiedByEdges( ...
            point_deg, edgeSet, boundaryIsOccupied)
        isOccupied = true;
        return;
    end
end
end

function isOccupied = pointOccupiedByEdges( ...
        point_deg, edgeSet, boundaryIsOccupied)
%% Section 0: Header & Readme
% SYNTAX
%   isOccupied = pointOccupiedByEdges( ...
%       point_deg, edgeSet, boundaryIsOccupied)
%**************************************************************************
% PURPOSE
%   - Apply odd-even polygon occupancy and the requested boundary policy.
%**************************************************************************
% INPUTS
%   - point_deg (1-by-2), edgeSet (N-by-4)
%   - boundaryIsOccupied (logical scalar)
%**************************************************************************
% OUTPUTS
%   - isOccupied (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
edgeStart_deg = edgeSet(:, 1:2);
edgeEnd_deg = edgeSet(:, 3:4);
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
startsAbove = edgeStart_deg(:, 2) > point_deg(2);
endsAbove = edgeEnd_deg(:, 2) > point_deg(2);
verticalStraddle = startsAbove ~= endsAbove;
rayCrosses = false(size(verticalStraddle));
rayCrosses(verticalStraddle) = point_deg(1) < ...
    edgeStart_deg(verticalStraddle, 1) + ( ...
    point_deg(2) - edgeStart_deg(verticalStraddle, 2)) .* ...
    edgeDelta_deg(verticalStraddle, 1) ./ ...
    edgeDelta_deg(verticalStraddle, 2);
inside = mod(nnz(rayCrosses), 2) == 1;
edgeLength_deg = hypot(edgeDelta_deg(:, 1), edgeDelta_deg(:, 2));
coordinateTolerance_deg = 1e-10 .* max(1, edgeLength_deg);
crossProduct_deg2 = (point_deg(1) - edgeStart_deg(:, 1)) .* ...
    edgeDelta_deg(:, 2) - (point_deg(2) - edgeStart_deg(:, 2)) .* ...
    edgeDelta_deg(:, 1);
onBoundary = any(abs(crossProduct_deg2) <= ...
    coordinateTolerance_deg .* max(1, edgeLength_deg) & ...
    point_deg(1) >= min(edgeStart_deg(:, 1), edgeEnd_deg(:, 1)) - ...
    coordinateTolerance_deg & ...
    point_deg(1) <= max(edgeStart_deg(:, 1), edgeEnd_deg(:, 1)) + ...
    coordinateTolerance_deg & ...
    point_deg(2) >= min(edgeStart_deg(:, 2), edgeEnd_deg(:, 2)) - ...
    coordinateTolerance_deg & ...
    point_deg(2) <= max(edgeStart_deg(:, 2), edgeEnd_deg(:, 2)) + ...
    coordinateTolerance_deg);
if boundaryIsOccupied
    isOccupied = inside || onBoundary;
else
    isOccupied = inside && ~onBoundary;
end
end

function inside = pointInsideBounds(point_deg, bounds_deg)
%% Section 0: Header & Readme
% SYNTAX
%   inside = pointInsideBounds(point_deg, bounds_deg)
%**************************************************************************
% PURPOSE
%   - Test one point against a finite axis-aligned box with roundoff guard.
%**************************************************************************
% INPUTS
%   - point_deg (1-by-2), bounds_deg (1-by-4)
%**************************************************************************
% OUTPUTS
%   - inside (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees.
%**************************************************************************
scale_deg = max(1, max(abs(bounds_deg)));
tolerance_deg = 1e-12 * scale_deg;
inside = all(isfinite(bounds_deg)) && ...
    point_deg(1) >= bounds_deg(1) - tolerance_deg && ...
    point_deg(1) <= bounds_deg(2) + tolerance_deg && ...
    point_deg(2) >= bounds_deg(3) - tolerance_deg && ...
    point_deg(2) <= bounds_deg(4) + tolerance_deg;
end
