function [isOccupied, blockingObstacleIndex, queryDetails] = queryAzElTimeObstacle( ...
        obstacleField, azimuth_deg, ...
        elevation_deg, queryTime, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   optionDefaults = queryAzElTimeObstacle()
%   isOccupied = queryAzElTimeObstacle( ...
%       obstacleField, azimuth_deg, elevation_deg, queryTime)
%   [isOccupied, blockingObstacleIndex, queryDetails] = queryAzElTimeObstacle( ...
%       obstacleField, azimuth_deg, ...
%       elevation_deg, queryTime, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Test broadcast azimuth/elevation/time points against packed polygons.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar struct or canonical obstacle data)
%       Preferred and legacy packed-format tags are accepted. Unpacked
%       obstacle data is packed before querying.
%   - azimuth_deg (numeric array)
%       Query azimuth coordinates.
%   - elevation_deg (numeric array)
%       Query elevation coordinates.
%   - queryTime (numeric array or datetime array)
%       Seconds from ReferenceTime or absolute UTC datetimes.
%   - optionOverrides (scalar struct)
%       CollisionMode, TimePaddingSamples, BoundsMarginDeg, and
%       SafetyMarginDeg control the broad and narrow phases.
%**************************************************************************
% OUTPUTS
%   - isOccupied (logical array)
%       True where any obstacle contains or clears the query point.
%   - blockingObstacleIndex (numeric array)
%       One-based blocking obstacle index, or zero when unoccupied.
%   - queryDetails (scalar struct)
%       Per-query sample indices, names, times, mode, and safety margin.
%**************************************************************************
% UNITS
%   - azimuth_deg, elevation_deg, BoundsMarginDeg, and SafetyMarginDeg are
%     degrees. Numeric queryTime values are seconds.

%% Section 1: Validate Inputs & Apply Defaults
defaultOptions = defaultQueryAzElTimeObstacleOptions();
if nargin == 0
    isOccupied = defaultOptions;
    blockingObstacleIndex = [];
    queryDetails = struct();
    return;
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("queryAzElTimeObstacle:InvalidOptions", ...
        "options must be a scalar struct.");
end
unknownOptionFields = setdiff( ...
    fieldnames(optionOverrides), fieldnames(defaultOptions), "stable");
if ~isempty(unknownOptionFields)
    warning("queryAzElTimeObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s.", ...
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
collisionMode = lower(string(resolvedOptions.CollisionMode));
if ~any(collisionMode == ["polygon", "bounds"])
    error("queryAzElTimeObstacle:InvalidMode", ...
        "CollisionMode must be 'polygon' or 'bounds'.");
end
validateattributes(resolvedOptions.TimePaddingSamples, {'numeric'}, ...
    {'scalar', 'integer', 'nonnegative'});
validateattributes(resolvedOptions.BoundsMarginDeg, {'numeric'}, ...
    {'vector', 'numel', 2, 'real', 'finite', 'nonnegative'});
validateattributes(resolvedOptions.SafetyMarginDeg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
isPackedInput = isstruct(obstacleField) && isscalar(obstacleField) && ...
    isfield(obstacleField, "Format") && any( ...
    string(obstacleField.Format) == [ ...
    "AzElTimeObstacleField", "AzElTimeObstacleWorkspace"]);
if ~isPackedInput
    obstacleField = buildAzElTimeObstacleField(obstacleField);
end
obstacleFieldHasRequiredFields = isstruct(obstacleField) && ...
    isfield(obstacleField, "Format") && ...
    any(string(obstacleField.Format) == [ ...
    "AzElTimeObstacleField", "AzElTimeObstacleWorkspace"]) && ...
    isfield(obstacleField, "Obstacles") && ...
    isfield(obstacleField, "ReferenceTime");
if ~obstacleFieldHasRequiredFields
    error("queryAzElTimeObstacle:InvalidObstacleField", ...
        "Use buildAzElTimeObstacleField to create the obstacle field.");
end

%% Section 2: Broadcast Query Arrays
if isdatetime(queryTime)
    queryTime.TimeZone = "UTC";
    broadcastTime_s = seconds(queryTime - obstacleField.ReferenceTime);
elseif isnumeric(queryTime)
    broadcastTime_s = double(queryTime);
else
    error("queryAzElTimeObstacle:InvalidTime", ...
        "queryTime must be datetime or seconds from ReferenceTime.");
end
broadcastValues = {double(azimuth_deg), double(elevation_deg), ...
    double(broadcastTime_s)};
queryElementCounts = zeros(1, 3);
for queryValueIndex = 1:3
    queryElementCounts(queryValueIndex) = numel( ...
        broadcastValues{queryValueIndex});
end
nonScalarCounts = queryElementCounts(queryElementCounts > 1);
if isempty(nonScalarCounts)
    outputSize = size(broadcastValues{1});
    targetQueryCount = 1;
elseif any(nonScalarCounts ~= nonScalarCounts(1))
    error("queryAzElTimeObstacle:SizeMismatch", ...
        "Non-scalar azimuth, elevation, and time inputs must have equal size.");
else
    targetQueryCount = nonScalarCounts(1);
    shapeSourceIndex = find( ...
        queryElementCounts == targetQueryCount, 1);
    outputSize = size(broadcastValues{shapeSourceIndex});
end
for queryValueIndex = 1:3
    if queryElementCounts(queryValueIndex) == 1
        broadcastValues{queryValueIndex} = repmat( ...
            broadcastValues{queryValueIndex}, targetQueryCount, 1);
    elseif ~isequal(size(broadcastValues{queryValueIndex}), outputSize)
        broadcastValues{queryValueIndex} = reshape( ...
            broadcastValues{queryValueIndex}, outputSize);
    end
    columnValue = broadcastValues{queryValueIndex}(:);
    broadcastValues{queryValueIndex} = columnValue;
end
queryAzimuth_deg = broadcastValues{1};
queryElevation_deg = broadcastValues{2};
queryTime_s = broadcastValues{3};
queryCount = numel(queryAzimuth_deg);
isOccupied = false(queryCount, 1);
blockingObstacleIndex = zeros(queryCount, 1, "uint32");

%% Section 3: Test Packed Obstacles
packedObstacles = obstacleField.Obstacles;
for packedObstacleIndex = 1:numel(packedObstacles)
    % Once one obstacle claims a query, later obstacles cannot change its
    % boolean result or diagnostic owner.
    isUnresolvedQuery = ~isOccupied & isfinite(queryAzimuth_deg) & ...
        isfinite(queryElevation_deg) & isfinite(queryTime_s);
    if ~any(isUnresolvedQuery)
        break;
    end
    packedObstacle = packedObstacles(packedObstacleIndex);
    obstacleTime_s = double(packedObstacle.TimeSeconds);
    obstacleBounds_deg = packedObstacle.BoundsDeg;
    sliceEdgeOffsets = packedObstacle.EdgeOffsets;
    edgeStartAzimuth_deg = packedObstacle.EdgeStartAzimuthDeg;
    edgeStartElevation_deg = packedObstacle.EdgeStartElevationDeg;
    edgeEndAzimuth_deg = packedObstacle.EdgeEndAzimuthDeg;
    edgeEndElevation_deg = packedObstacle.EdgeEndElevationDeg;

    % Uniform time bases map arithmetically. Irregular time bases use nearest
    % interpolation, but neither path extrapolates outside the obstacle span.
    nearestSampleIndex = zeros(size(queryTime_s));
    isInsideObstacleTimeSpan = false(size(queryTime_s));
    if packedObstacle.SampleCount > 0
        isInsideObstacleTimeSpan = queryTime_s >= obstacleTime_s(1) & ...
            queryTime_s <= obstacleTime_s(end);
        if packedObstacle.SampleCount == 1
            nearestSampleIndex(isInsideObstacleTimeSpan) = 1;
        elseif packedObstacle.IsUniformTime
            nearestSampleIndex(isInsideObstacleTimeSpan) = round(( ...
                queryTime_s(isInsideObstacleTimeSpan) - obstacleTime_s(1)) ./ ...
                packedObstacle.TimeStepSeconds) + 1;
        else
            sampleNumbers = (1:packedObstacle.SampleCount).';
            nearestSampleIndex(isInsideObstacleTimeSpan) = interp1( ...
                obstacleTime_s, sampleNumbers, ...
                queryTime_s(isInsideObstacleTimeSpan), "nearest");
        end
        nearestSampleIndex = round(nearestSampleIndex);
    end
    isEligibleQuery = isUnresolvedQuery & isInsideObstacleTimeSpan;

    % Padding checks neighboring source slices rather than inventing an
    % interpolated polygon between measured boundaries.
    for sampleOffset = -resolvedOptions.TimePaddingSamples: ...
            resolvedOptions.TimePaddingSamples
        candidateSampleIndex = nearestSampleIndex + sampleOffset;
        isCandidateQuery = isEligibleQuery & candidateSampleIndex >= 1 & ...
            candidateSampleIndex <= packedObstacle.SampleCount & ~isOccupied;
        candidateRows = find(isCandidateQuery);
        if isempty(candidateRows)
            continue;
        end

        % --- Conservative Slice-Bounds Reject -----------------------------
        sampledBounds_deg = double(obstacleBounds_deg( ...
            candidateSampleIndex(candidateRows), :));
        broadPhaseMargin_deg = resolvedOptions.BoundsMarginDeg + ...
            resolvedOptions.SafetyMarginDeg;
        minimumAzimuth_deg = ...
            sampledBounds_deg(:, 1) - broadPhaseMargin_deg(1);
        maximumAzimuth_deg = ...
            sampledBounds_deg(:, 2) + broadPhaseMargin_deg(1);
        minimumElevation_deg = ...
            sampledBounds_deg(:, 3) - broadPhaseMargin_deg(2);
        maximumElevation_deg = ...
            sampledBounds_deg(:, 4) + broadPhaseMargin_deg(2);
        isInsideSliceBounds = all(isfinite(sampledBounds_deg), 2) & ...
            queryAzimuth_deg(candidateRows) >= minimumAzimuth_deg & ...
            queryAzimuth_deg(candidateRows) <= maximumAzimuth_deg & ...
            queryElevation_deg(candidateRows) >= minimumElevation_deg & ...
            queryElevation_deg(candidateRows) <= maximumElevation_deg;
        collisionRows = candidateRows(isInsideSliceBounds);
        if isempty(collisionRows)
            continue;
        end
        if collisionMode == "bounds"
            newlyBlockedRows = collisionRows(~isOccupied(collisionRows));
            isOccupied(newlyBlockedRows) = true;
            blockerNumber = uint32(packedObstacleIndex);
            blockingObstacleIndex(newlyBlockedRows) = blockerNumber;
            continue;
        end

        % --- Expand Ragged Edge Ranges ------------------------------------
        collisionSamples = candidateSampleIndex(collisionRows);
        edgeCounts = double(sliceEdgeOffsets(collisionSamples + 1) - ...
            sliceEdgeOffsets(collisionSamples));
        edgeCounts = edgeCounts(:);
        queriesHaveEdges = edgeCounts > 0;
        if ~any(queriesHaveEdges)
            continue;
        end
        activeRows = collisionRows(queriesHaveEdges);
        activeSamples = collisionSamples(queriesHaveEdges);
        activeEdgeCounts = edgeCounts(queriesHaveEdges);
        activeQueryCount = numel(activeSamples);
        totalEdgeCount = sum(activeEdgeCounts);
        edgeOwner = repelem((1:activeQueryCount).', activeEdgeCounts);
        % repelem can preserve a row-shaped repetition argument and return
        % a matrix. Every later edge array is packed as a column, so force
        % the owner map to the same one-edge-per-row invariant before it is
        % used for indexing or accumarray grouping.
        edgeOwner = edgeOwner(:);
        precedingEdgeCounts = cumsum([0; activeEdgeCounts(1:end - 1)]);
        ownerBaseOffset = repelem(precedingEdgeCounts, activeEdgeCounts);
        withinOwnerOffset = (0:totalEdgeCount - 1).' - ...
            ownerBaseOffset(:);
        firstEdgeIndex = double(sliceEdgeOffsets(activeSamples));
        repeatedFirstEdgeIndex = repelem( ...
            firstEdgeIndex(:), activeEdgeCounts);
        packedEdgeIndex = repeatedFirstEdgeIndex(:) + withinOwnerOffset;

        activeEdgeStartAzimuth_deg = double( ...
            edgeStartAzimuth_deg(packedEdgeIndex));
        activeEdgeStartElevation_deg = double( ...
            edgeStartElevation_deg(packedEdgeIndex));
        activeEdgeEndAzimuth_deg = double( ...
            edgeEndAzimuth_deg(packedEdgeIndex));
        activeEdgeEndElevation_deg = double( ...
            edgeEndElevation_deg(packedEdgeIndex));
        ownerQueryRows = activeRows(edgeOwner);
        activeQueryAzimuth_deg = queryAzimuth_deg(ownerQueryRows);
        activeQueryElevation_deg = queryElevation_deg(ownerQueryRows);
        activeEdgeStartAzimuth_deg = activeEdgeStartAzimuth_deg(:);
        activeEdgeStartElevation_deg = activeEdgeStartElevation_deg(:);
        activeEdgeEndAzimuth_deg = activeEdgeEndAzimuth_deg(:);
        activeEdgeEndElevation_deg = activeEdgeEndElevation_deg(:);
        activeQueryAzimuth_deg = activeQueryAzimuth_deg(:);
        activeQueryElevation_deg = activeQueryElevation_deg(:);

        % --- Odd-Even Crossing With Boundary Inclusion --------------------
        edgeStartsAboveQuery = ...
            activeEdgeStartElevation_deg > activeQueryElevation_deg;
        edgeEndsAboveQuery = ...
            activeEdgeEndElevation_deg > activeQueryElevation_deg;
        verticalStraddle = edgeStartsAboveQuery ~= edgeEndsAboveQuery;
        rayCrossesEdge = false(totalEdgeCount, 1);
        rayCrossesEdge(verticalStraddle) = activeQueryAzimuth_deg( ...
            verticalStraddle) < ...
            activeEdgeStartAzimuth_deg(verticalStraddle) + ( ...
            activeQueryElevation_deg(verticalStraddle) - ...
            activeEdgeStartElevation_deg(verticalStraddle)) .* ( ...
            activeEdgeEndAzimuth_deg(verticalStraddle) - ...
            activeEdgeStartAzimuth_deg(verticalStraddle)) ./ ( ...
            activeEdgeEndElevation_deg(verticalStraddle) - ...
            activeEdgeStartElevation_deg(verticalStraddle));
        crossingCount = accumarray( ...
            edgeOwner, double(rayCrossesEdge), ...
            [activeQueryCount 1], @sum, 0);

        edgeAzimuthDelta_deg = ...
            activeEdgeEndAzimuth_deg - activeEdgeStartAzimuth_deg;
        edgeElevationDelta_deg = ...
            activeEdgeEndElevation_deg - activeEdgeStartElevation_deg;
        edgeLength_deg = hypot( ...
            edgeAzimuthDelta_deg, edgeElevationDelta_deg);
        crossProduct_deg2 = ( ...
            activeQueryAzimuth_deg - activeEdgeStartAzimuth_deg) .* ...
            edgeElevationDelta_deg - ( ...
            activeQueryElevation_deg - activeEdgeStartElevation_deg) .* ...
            edgeAzimuthDelta_deg;
        % The 1e-7 relative tolerance absorbs single-precision packing
        % error. Separate coordinate and cross-product tolerances preserve
        % dimensional consistency for both boundary tests.
        coordinateTolerance_deg = 1e-7 .* max(1, edgeLength_deg);
        crossProductTolerance_deg2 = ...
            coordinateTolerance_deg .* max(1, edgeLength_deg);
        queryOnEdge = ...
            abs(crossProduct_deg2) <= crossProductTolerance_deg2 & ...
            activeQueryAzimuth_deg >= min( ...
            activeEdgeStartAzimuth_deg, activeEdgeEndAzimuth_deg) - ...
            coordinateTolerance_deg & ...
            activeQueryAzimuth_deg <= max( ...
            activeEdgeStartAzimuth_deg, activeEdgeEndAzimuth_deg) + ...
            coordinateTolerance_deg & ...
            activeQueryElevation_deg >= min( ...
            activeEdgeStartElevation_deg, activeEdgeEndElevation_deg) - ...
            coordinateTolerance_deg & ...
            activeQueryElevation_deg <= max( ...
            activeEdgeStartElevation_deg, activeEdgeEndElevation_deg) + ...
            coordinateTolerance_deg;
        boundaryCount = accumarray( ...
            edgeOwner, double(queryOnEdge), ...
            [activeQueryCount 1], @sum, 0);
        isInsideByOddParity = mod(crossingCount, 2) == 1;
        isOnPolygonBoundary = boundaryCount > 0;
        activeQueriesBlocked = isInsideByOddParity | isOnPolygonBoundary;

        % --- Optional Euclidean Edge Clearance ----------------------------
        if resolvedOptions.SafetyMarginDeg > 0
            edgeLength_deg2 = edgeAzimuthDelta_deg.^2 + ...
                edgeElevationDelta_deg.^2;
            minimumDistance_deg2 = inf(totalEdgeCount, 1);
            % Adjacent azimuth images keep clearance continuous at the
            % conventional -180/180 seam.
            for azimuthShift_deg = [-360 0 360]
                shiftedQueryAzimuth_deg = ...
                    activeQueryAzimuth_deg + azimuthShift_deg;
                edgeFraction = (( ...
                    shiftedQueryAzimuth_deg - ...
                    activeEdgeStartAzimuth_deg) .* ...
                    edgeAzimuthDelta_deg + ( ...
                    activeQueryElevation_deg - ...
                    activeEdgeStartElevation_deg) .* ...
                    edgeElevationDelta_deg) ./ max(edgeLength_deg2, eps);
                edgeFraction = min(max(edgeFraction, 0), 1);
                closestAzimuth_deg = activeEdgeStartAzimuth_deg + ...
                    edgeFraction .* edgeAzimuthDelta_deg;
                closestElevation_deg = activeEdgeStartElevation_deg + ...
                    edgeFraction .* edgeElevationDelta_deg;
                distance_deg2 = ( ...
                    shiftedQueryAzimuth_deg - closestAzimuth_deg).^2 + ...
                    (activeQueryElevation_deg - closestElevation_deg).^2;
                minimumDistance_deg2 = min( ...
                    minimumDistance_deg2, distance_deg2);
            end
            safetyMargin_deg2 = resolvedOptions.SafetyMarginDeg^2;
            edgeWithinMargin = minimumDistance_deg2 <= safetyMargin_deg2;
            nearEdgeCount = accumarray( ...
                edgeOwner, double(edgeWithinMargin), ...
                [activeQueryCount 1], @sum, 0);
            activeQueriesBlocked = activeQueriesBlocked | nearEdgeCount > 0;
        end

        newlyBlockedRows = activeRows(activeQueriesBlocked & ...
            ~isOccupied(activeRows));
        isOccupied(newlyBlockedRows) = true;
        blockerNumber = uint32(packedObstacleIndex);
        blockingObstacleIndex(newlyBlockedRows) = blockerNumber;
    end
end

%% Section 4: Assemble The Output
isOccupied = reshape(isOccupied, outputSize);
blockingObstacleIndex = reshape(blockingObstacleIndex, outputSize);
if nargout >= 3
    obstacleNames = strings(queryCount, 1);
    flatBlockingObstacleIndex = blockingObstacleIndex(:);
    for packedObstacleIndex = 1:numel(packedObstacles)
        belongsToObstacle = flatBlockingObstacleIndex == packedObstacleIndex;
        obstacleName = packedObstacles(packedObstacleIndex).Name;
        obstacleNames(belongsToObstacle) = obstacleName;
    end
    queryDetails = struct( ...
        "ObstacleName", reshape(obstacleNames, outputSize), ...
        "QueryTimeSeconds", reshape(queryTime_s, outputSize), ...
        "CollisionMode", collisionMode, ...
        "SafetyMarginDeg", resolvedOptions.SafetyMarginDeg, ...
        "Options", resolvedOptions);
end
end

%% Section 5: Local Functions
function options = defaultQueryAzElTimeObstacleOptions()
%% Section 0: Header & Readme
% SYNTAX
%   options = defaultQueryAzElTimeObstacleOptions()
%**************************************************************************
% PURPOSE
%   - Keep collision-query defaults in one source of truth.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Fully populated query options.
%**************************************************************************
% UNITS
%   - BoundsMarginDeg and SafetyMarginDeg are degrees.
%   - TimePaddingSamples is dimensionless.
options = struct( ...
    "CollisionMode", "polygon", ...
    "TimePaddingSamples", 0, ...
    "BoundsMarginDeg", [0 0], ...
    "SafetyMarginDeg", 0);
end
