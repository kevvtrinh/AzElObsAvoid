function [isOccupied, collisionDetails] = queryAzElTimedPathCollision( ...
        obstacleField, time_s, position_deg, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   optionDefaults = queryAzElTimedPathCollision()
%   isOccupied = queryAzElTimedPathCollision( ...
%       obstacleField, time_s, position_deg)
%   [isOccupied, collisionDetails] = queryAzElTimedPathCollision( ...
%       obstacleField, time_s, position_deg, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Test both trajectory samples and every connecting line segment against
%     the complete packed polygon geometry.
%   - Preserve the nearest obstacle-slice and time-padding policy used by
%     queryAzElTimeObstacle without allowing clear endpoints to hide a
%     polygon crossing between samples.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field or canonical obstacle data)
%       Unpacked obstacle data is packed before the continuous query.
%   - time_s (numeric scalar or N-element vector)
%       A scalar applies one time to the complete spatial path. A vector must
%       be finite and nondecreasing, with one value per position row.
%   - position_deg (N-by-2 finite numeric array)
%       Ordered [azimuth elevation] trajectory samples. Connecting rows are
%       interpreted as line segments and checked continuously in space.
%   - optionOverrides (scalar struct, optional; default struct())
%       .TimePaddingSamples is a nonnegative integer (default 0).
%       .BoundaryIsOccupied controls exact polygon contact (default true).
%**************************************************************************
% OUTPUTS
%   - isOccupied (N-by-1 logical vector)
%       Row one reports its sample. Later rows report their sample or the
%       inbound segment, so any(isOccupied) validates the complete path.
%   - collisionDetails (scalar struct)
%       Stable sample/segment masks, blocking obstacle and source-slice
%       indices, interpreted time/position histories, and resolved options.
%**************************************************************************
% UNITS
%   - Position is degrees and numeric time is seconds from ReferenceTime.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

defaultOptions = struct( ...
    "TimePaddingSamples", 0, ...
    "BoundaryIsOccupied", true);
if nargin == 0
    isOccupied = defaultOptions;
    collisionDetails = struct();
    return;
end
if nargin < 4 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("queryAzElTimedPathCollision:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
unknownOptionFields = setdiff( ...
    fieldnames(optionOverrides), fieldnames(defaultOptions), "stable");
if ~isempty(unknownOptionFields)
    warning("queryAzElTimedPathCollision:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(string(unknownOptionFields), ", "));
    optionOverrides = rmfield(optionOverrides, unknownOptionFields);
end
resolvedOptions = defaultOptions;
providedOptionFields = fieldnames(optionOverrides);
for optionIndex = 1:numel(providedOptionFields)
    optionName = providedOptionFields{optionIndex};
    if ~isempty(optionOverrides.(optionName))
        resolvedOptions.(optionName) = optionOverrides.(optionName);
    end
end
validateattributes(resolvedOptions.TimePaddingSamples, {'numeric'}, ...
    {'scalar','integer','nonnegative'});
validateattributes(resolvedOptions.BoundaryIsOccupied, ...
    {'logical','numeric'}, {'scalar'});
resolvedOptions.BoundaryIsOccupied = ...
    logical(resolvedOptions.BoundaryIsOccupied);

validateattributes(position_deg, {'numeric'}, ...
    {'real','finite','2d','ncols',2,'nonempty'});
position_deg = double(position_deg);
pathPointCount = size(position_deg, 1);
validateattributes(time_s, {'numeric'}, {'real','finite','nonempty'});
time_s = double(time_s(:));
if isscalar(time_s)
    time_s = repmat(time_s, pathPointCount, 1);
elseif numel(time_s) ~= pathPointCount
    error("queryAzElTimedPathCollision:TimeSizeMismatch", ...
        "time_s must be scalar or have %d values for position_deg.", ...
        pathPointCount);
end
if any(diff(time_s) < 0)
    error("queryAzElTimedPathCollision:TimeNotNondecreasing", ...
        "time_s must be nondecreasing along position_deg.");
end
isPackedInput = isstruct(obstacleField) && isscalar(obstacleField) && ...
    isfield(obstacleField, "Format") && any( ...
    string(obstacleField.Format) == [ ...
    "AzElTimeObstacleField", "AzElTimeObstacleWorkspace"]);
if ~isPackedInput
    obstacleField = buildAzElTimeObstacleField(obstacleField);
end

%% Section 2: Check Samples & Continuous Connecting Segments

pointOptions = struct( ...
    "CollisionMode", "polygon", ...
    "TimePaddingSamples", resolvedOptions.TimePaddingSamples, ...
    "BoundaryIsOccupied", resolvedOptions.BoundaryIsOccupied);
[sampleOccupied, sampleBlockingObstacleIndex] = ...
    queryAzElTimeObstacle(obstacleField, position_deg(:, 1), ...
    position_deg(:, 2), time_s, pointOptions);
sampleOccupied = logical(sampleOccupied(:));
sampleBlockingObstacleIndex = uint32( ...
    sampleBlockingObstacleIndex(:));
segmentOccupied = false(max(0, pathPointCount - 1), 1);
segmentBlockingObstacleIndex = zeros( ...
    size(segmentOccupied), "uint32");
segmentBlockingSliceIndex = zeros( ...
    size(segmentOccupied), "uint32");

packedObstacles = obstacleField.Obstacles;
for segmentIndex = 1:pathPointCount - 1
    segmentStartTime_s = time_s(segmentIndex);
    segmentEndTime_s = time_s(segmentIndex + 1);
    segmentStart_deg = position_deg(segmentIndex, :);
    segmentEnd_deg = position_deg(segmentIndex + 1, :);
    for obstacleIndex = 1:numel(packedObstacles)
        packedObstacle = packedObstacles(obstacleIndex);
        obstacleTime_s = double(packedObstacle.TimeSeconds(:));
        if isempty(obstacleTime_s) || ...
                segmentEndTime_s < obstacleTime_s(1) || ...
                segmentStartTime_s > obstacleTime_s(end)
            continue;
        end
        [nearestStartTime_s, nearestEndTime_s] = ...
            nearestSliceActivationIntervals(obstacleTime_s);
        for sliceIndex = 1:packedObstacle.SampleCount
            firstNearestIndex = max(1, sliceIndex - ...
                resolvedOptions.TimePaddingSamples);
            lastNearestIndex = min(packedObstacle.SampleCount, ...
                sliceIndex + resolvedOptions.TimePaddingSamples);
            activeStartTime_s = ...
                nearestStartTime_s(firstNearestIndex);
            activeEndTime_s = nearestEndTime_s(lastNearestIndex);
            overlapStartTime_s = max( ...
                segmentStartTime_s, activeStartTime_s);
            overlapEndTime_s = min( ...
                segmentEndTime_s, activeEndTime_s);
            if overlapStartTime_s > overlapEndTime_s
                continue;
            end
            if segmentEndTime_s > segmentStartTime_s
                overlapParameter = ([overlapStartTime_s; ...
                    overlapEndTime_s] - segmentStartTime_s) ./ ...
                    (segmentEndTime_s - segmentStartTime_s);
            else
                overlapParameter = [0; 1];
            end
            activeStart_deg = segmentStart_deg + ...
                overlapParameter(1) .* ...
                (segmentEnd_deg - segmentStart_deg);
            activeEnd_deg = segmentStart_deg + ...
                overlapParameter(2) .* ...
                (segmentEnd_deg - segmentStart_deg);
            if segmentIntersectsOccupiedSlice( ...
                    packedObstacle, sliceIndex, activeStart_deg, ...
                    activeEnd_deg, resolvedOptions.BoundaryIsOccupied)
                segmentOccupied(segmentIndex) = true;
                segmentBlockingObstacleIndex(segmentIndex) = ...
                    uint32(obstacleIndex);
                segmentBlockingSliceIndex(segmentIndex) = ...
                    uint32(sliceIndex);
                break;
            end
        end
        if segmentOccupied(segmentIndex)
            break;
        end
    end
end

%% Section 3: Assemble Stable Diagnostics

isOccupied = sampleOccupied;
if pathPointCount > 1
    isOccupied(2:end) = isOccupied(2:end) | segmentOccupied;
end
blockingObstacleIndex = sampleBlockingObstacleIndex;
blockingSliceIndex = zeros(pathPointCount, 1, "uint32");
for pathPointIndex = 2:pathPointCount
    inboundSegmentIndex = pathPointIndex - 1;
    if ~sampleOccupied(pathPointIndex) && ...
            segmentOccupied(inboundSegmentIndex)
        blockingObstacleIndex(pathPointIndex) = ...
            segmentBlockingObstacleIndex(inboundSegmentIndex);
        blockingSliceIndex(pathPointIndex) = ...
            segmentBlockingSliceIndex(inboundSegmentIndex);
    end
end
collisionDetails = struct( ...
    "SampleOccupied", sampleOccupied, ...
    "SegmentOccupied", segmentOccupied, ...
    "BlockingObstacleIndex", blockingObstacleIndex, ...
    "BlockingSliceIndex", blockingSliceIndex, ...
    "SegmentBlockingObstacleIndex", segmentBlockingObstacleIndex, ...
    "SegmentBlockingSliceIndex", segmentBlockingSliceIndex, ...
    "time_s", time_s, ...
    "position_deg", position_deg, ...
    "Options", resolvedOptions);
end

%% Section 4: Local Functions

function [activationStartTime_s, activationEndTime_s] = ...
        nearestSliceActivationIntervals(obstacleTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [activationStartTime_s, activationEndTime_s] = ...
%       nearestSliceActivationIntervals(obstacleTime_s)
%**************************************************************************
% PURPOSE
%   - Return the clipped Voronoi time interval owned by every source slice.
%**************************************************************************
% INPUTS
%   - obstacleTime_s (strictly increasing numeric column vector)
%**************************************************************************
% OUTPUTS
%   - activationStartTime_s, activationEndTime_s (numeric column vectors)
%**************************************************************************
% UNITS
%   - Time is seconds.
%**************************************************************************
if isscalar(obstacleTime_s)
    activationStartTime_s = obstacleTime_s;
    activationEndTime_s = obstacleTime_s;
    return;
end
midpointTime_s = 0.5 .* ( ...
    obstacleTime_s(1:end - 1) + obstacleTime_s(2:end));
activationStartTime_s = [obstacleTime_s(1); midpointTime_s];
activationEndTime_s = [midpointTime_s; obstacleTime_s(end)];
end

function isOccupied = segmentIntersectsOccupiedSlice( ...
        packedObstacle, sliceIndex, segmentStart_deg, segmentEnd_deg, ...
        boundaryIsOccupied)
%% Section 0: Header & Readme
% SYNTAX
%   isOccupied = segmentIntersectsOccupiedSlice( ...
%       packedObstacle, sliceIndex, segmentStart_deg, segmentEnd_deg, ...
%       boundaryIsOccupied)
%**************************************************************************
% PURPOSE
%   - Prove whether one complete spatial segment enters one packed slice.
%**************************************************************************
% INPUTS
%   - packedObstacle (scalar packed obstacle record)
%   - sliceIndex (positive integer source-slice index)
%   - segmentStart_deg, segmentEnd_deg (1-by-2 positions)
%   - boundaryIsOccupied (logical scalar)
%**************************************************************************
% OUTPUTS
%   - isOccupied (logical scalar)
%**************************************************************************
% UNITS
%   - Position is degrees; all intersection parameters are dimensionless.
%**************************************************************************
sliceBounds_deg = double(packedObstacle.BoundsDeg(sliceIndex, :));
if any(~isfinite(sliceBounds_deg))
    isOccupied = false;
    return;
end
segmentMinimum_deg = min(segmentStart_deg, segmentEnd_deg);
segmentMaximum_deg = max(segmentStart_deg, segmentEnd_deg);
coordinateScale_deg = max(1, max(abs([ ...
    sliceBounds_deg, segmentStart_deg, segmentEnd_deg])));
coordinateTolerance_deg = 1e-10 * coordinateScale_deg;
boxesOverlap = segmentMaximum_deg(1) >= ...
    sliceBounds_deg(1) - coordinateTolerance_deg && ...
    segmentMinimum_deg(1) <= ...
    sliceBounds_deg(2) + coordinateTolerance_deg && ...
    segmentMaximum_deg(2) >= ...
    sliceBounds_deg(3) - coordinateTolerance_deg && ...
    segmentMinimum_deg(2) <= ...
    sliceBounds_deg(4) + coordinateTolerance_deg;
if ~boxesOverlap
    isOccupied = false;
    return;
end

firstEdgeIndex = double(packedObstacle.EdgeOffsets(sliceIndex));
lastEdgeIndex = double( ...
    packedObstacle.EdgeOffsets(sliceIndex + 1)) - 1;
if firstEdgeIndex > lastEdgeIndex
    isOccupied = false;
    return;
end
edgeStart_deg = [ ...
    double(packedObstacle.EdgeStartAzimuthDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeStartElevationDeg( ...
    firstEdgeIndex:lastEdgeIndex))];
edgeEnd_deg = [ ...
    double(packedObstacle.EdgeEndAzimuthDeg( ...
    firstEdgeIndex:lastEdgeIndex)), ...
    double(packedObstacle.EdgeEndElevationDeg( ...
    firstEdgeIndex:lastEdgeIndex))];

[intersectionParameter, touchesBoundary] = ...
    segmentEdgeIntersectionParameters(segmentStart_deg, segmentEnd_deg, ...
    edgeStart_deg, edgeEnd_deg, coordinateTolerance_deg);
if boundaryIsOccupied && touchesBoundary
    isOccupied = true;
    return;
end
breakParameter = sort([0; intersectionParameter(:); 1]);
parameterTolerance = coordinateTolerance_deg / max( ...
    coordinateTolerance_deg, norm(segmentEnd_deg - segmentStart_deg));
keepParameter = [true; diff(breakParameter) > parameterTolerance];
breakParameter = breakParameter(keepParameter);
breakParameter = min(1, max(0, breakParameter));
if isscalar(breakParameter)
    testParameter = breakParameter;
else
    intervalMidpoint = 0.5 .* ( ...
        breakParameter(1:end - 1) + breakParameter(2:end));
    testParameter = unique([breakParameter; intervalMidpoint]);
end
testPosition_deg = segmentStart_deg + testParameter .* ...
    (segmentEnd_deg - segmentStart_deg);
isOccupied = any(pointsOccupiedByEdges( ...
    testPosition_deg, edgeStart_deg, edgeEnd_deg, ...
    boundaryIsOccupied, coordinateTolerance_deg));
end

function [intersectionParameter, touchesBoundary] = ...
        segmentEdgeIntersectionParameters(segmentStart_deg, segmentEnd_deg, ...
        edgeStart_deg, edgeEnd_deg, coordinateTolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [intersectionParameter, touchesBoundary] = ...
%       segmentEdgeIntersectionParameters(segmentStart_deg, segmentEnd_deg, ...
%       edgeStart_deg, edgeEnd_deg, coordinateTolerance_deg)
%**************************************************************************
% PURPOSE
%   - Locate proper and collinear contacts between a segment and edge set.
%**************************************************************************
% INPUTS
%   - Segment endpoints and M-by-2 edge endpoints in degrees.
%   - coordinateTolerance_deg (nonnegative scalar numerical tolerance)
%**************************************************************************
% OUTPUTS
%   - intersectionParameter (numeric column vector in [0,1])
%   - touchesBoundary (logical scalar)
%**************************************************************************
% UNITS
%   - Position and tolerance are degrees; parameters are dimensionless.
%**************************************************************************
segmentDelta_deg = segmentEnd_deg - segmentStart_deg;
segmentLength_deg = norm(segmentDelta_deg);
if segmentLength_deg <= coordinateTolerance_deg
    intersectionParameter = zeros(0, 1);
    touchesBoundary = false;
    return;
end
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
offset_deg = edgeStart_deg - segmentStart_deg;
denominator_deg2 = segmentDelta_deg(1) .* edgeDelta_deg(:, 2) - ...
    segmentDelta_deg(2) .* edgeDelta_deg(:, 1);
edgeLength_deg = hypot(edgeDelta_deg(:, 1), edgeDelta_deg(:, 2));
denominatorTolerance_deg2 = coordinateTolerance_deg .* ...
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
parameterTolerance = coordinateTolerance_deg / segmentLength_deg;
properContact = isNonparallel & ...
    segmentParameter >= -parameterTolerance & ...
    segmentParameter <= 1 + parameterTolerance & ...
    edgeParameter >= -parameterTolerance & ...
    edgeParameter <= 1 + parameterTolerance;
intersectionParameter = segmentParameter(properContact);

offsetCrossSegment_deg2 = offset_deg(:, 1) .* segmentDelta_deg(2) - ...
    offset_deg(:, 2) .* segmentDelta_deg(1);
isCollinear = ~isNonparallel & ...
    abs(offsetCrossSegment_deg2) <= ...
    coordinateTolerance_deg * max(1, segmentLength_deg);
if any(isCollinear)
    segmentLengthSquared_deg2 = dot(segmentDelta_deg, segmentDelta_deg);
    collinearStartParameter = ( ...
        offset_deg(isCollinear, :) * segmentDelta_deg.') ./ ...
        segmentLengthSquared_deg2;
    collinearEndOffset_deg = ...
        edgeEnd_deg(isCollinear, :) - segmentStart_deg;
    collinearEndParameter = ( ...
        collinearEndOffset_deg * segmentDelta_deg.') ./ ...
        segmentLengthSquared_deg2;
    collinearMinimum = min( ...
        collinearStartParameter, collinearEndParameter);
    collinearMaximum = max( ...
        collinearStartParameter, collinearEndParameter);
    overlapsSegment = collinearMaximum >= -parameterTolerance & ...
        collinearMinimum <= 1 + parameterTolerance;
    intersectionParameter = [intersectionParameter; ...
        collinearStartParameter(overlapsSegment); ...
        collinearEndParameter(overlapsSegment)];
end
intersectionParameter = min(1, max(0, intersectionParameter));
touchesBoundary = ~isempty(intersectionParameter);
end

function isOccupied = pointsOccupiedByEdges( ...
        position_deg, edgeStart_deg, edgeEnd_deg, ...
        boundaryIsOccupied, coordinateTolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   isOccupied = pointsOccupiedByEdges( ...
%       position_deg, edgeStart_deg, edgeEnd_deg, ...
%       boundaryIsOccupied, coordinateTolerance_deg)
%**************************************************************************
% PURPOSE
%   - Apply the packed query's odd-even and boundary policy to one slice.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 points)
%   - edgeStart_deg, edgeEnd_deg (M-by-2 packed polygon edges)
%   - boundaryIsOccupied (logical scalar)
%   - coordinateTolerance_deg (nonnegative scalar)
%**************************************************************************
% OUTPUTS
%   - isOccupied (N-by-1 logical vector)
%**************************************************************************
% UNITS
%   - Position and tolerance are degrees.
%**************************************************************************
pointCount = size(position_deg, 1);
isOccupied = false(pointCount, 1);
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
edgeLength_deg = hypot(edgeDelta_deg(:, 1), edgeDelta_deg(:, 2));
for pointIndex = 1:pointCount
    point_deg = position_deg(pointIndex, :);
    edgeStartsAbove = edgeStart_deg(:, 2) > point_deg(2);
    edgeEndsAbove = edgeEnd_deg(:, 2) > point_deg(2);
    verticalStraddle = edgeStartsAbove ~= edgeEndsAbove;
    rayCrossesEdge = false(size(verticalStraddle));
    rayCrossesEdge(verticalStraddle) = point_deg(1) < ...
        edgeStart_deg(verticalStraddle, 1) + ( ...
        point_deg(2) - edgeStart_deg(verticalStraddle, 2)) .* ...
        edgeDelta_deg(verticalStraddle, 1) ./ ...
        edgeDelta_deg(verticalStraddle, 2);
    isInside = mod(nnz(rayCrossesEdge), 2) == 1;
    crossProduct_deg2 = (point_deg(1) - edgeStart_deg(:, 1)) .* ...
        edgeDelta_deg(:, 2) - ...
        (point_deg(2) - edgeStart_deg(:, 2)) .* edgeDelta_deg(:, 1);
    crossTolerance_deg2 = coordinateTolerance_deg .* ...
        max(1, edgeLength_deg);
    isOnEdge = abs(crossProduct_deg2) <= crossTolerance_deg2 & ...
        point_deg(1) >= min(edgeStart_deg(:, 1), edgeEnd_deg(:, 1)) - ...
        coordinateTolerance_deg & ...
        point_deg(1) <= max(edgeStart_deg(:, 1), edgeEnd_deg(:, 1)) + ...
        coordinateTolerance_deg & ...
        point_deg(2) >= min(edgeStart_deg(:, 2), edgeEnd_deg(:, 2)) - ...
        coordinateTolerance_deg & ...
        point_deg(2) <= max(edgeStart_deg(:, 2), edgeEnd_deg(:, 2)) + ...
        coordinateTolerance_deg;
    if boundaryIsOccupied
        isOccupied(pointIndex) = isInside || any(isOnEdge);
    else
        isOccupied(pointIndex) = isInside && ~any(isOnEdge);
    end
end
end
