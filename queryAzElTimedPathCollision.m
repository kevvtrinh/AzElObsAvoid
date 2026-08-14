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
    {'logical','numeric'}, {'real','finite','scalar'});
if isnumeric(resolvedOptions.BoundaryIsOccupied) && ...
        ~any(resolvedOptions.BoundaryIsOccupied == [0 1])
    error("queryAzElTimedPathCollision:InvalidBoundaryPolicy", ...
        "BoundaryIsOccupied must be scalar logical or binary numeric.");
end
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
isSameTimePath = pathPointCount > 1 && all(time_s == time_s(1));
if isSameTimePath
    for obstacleIndex = 1:numel(packedObstacles)
        packedObstacle = packedObstacles(obstacleIndex);
        [obstacleSegmentOccupied, ~, obstacleBlockingSliceIndex] = ...
            azElInternal.queryPackedMovingObstacle( ...
            packedObstacle, time_s(1), position_deg, ...
            resolvedOptions.BoundaryIsOccupied, ...
            resolvedOptions.TimePaddingSamples);
        newlyBlocked = obstacleSegmentOccupied & ~segmentOccupied;
        segmentOccupied = segmentOccupied | obstacleSegmentOccupied;
        segmentBlockingObstacleIndex(newlyBlocked) = uint32(obstacleIndex);
        segmentBlockingSliceIndex(newlyBlocked) = uint32( ...
            obstacleBlockingSliceIndex(newlyBlocked));
    end
else
    for segmentIndex = 1:pathPointCount - 1
        segmentStartTime_s = time_s(segmentIndex);
        segmentEndTime_s = time_s(segmentIndex + 1);
        segmentStart_deg = position_deg(segmentIndex, :);
        segmentEnd_deg = position_deg(segmentIndex + 1, :);
        for obstacleIndex = 1:numel(packedObstacles)
            packedObstacle = packedObstacles(obstacleIndex);
            [segmentIsOccupied, ~, blockingSliceIndex] = ...
                azElInternal.queryPackedMovingObstacle( ...
                packedObstacle, [segmentStartTime_s; segmentEndTime_s], ...
                [segmentStart_deg; segmentEnd_deg], ...
                resolvedOptions.BoundaryIsOccupied, ...
                resolvedOptions.TimePaddingSamples);
            if segmentIsOccupied
                segmentOccupied(segmentIndex) = true;
                segmentBlockingObstacleIndex(segmentIndex) = ...
                    uint32(obstacleIndex);
                segmentBlockingSliceIndex(segmentIndex) = ...
                    uint32(blockingSliceIndex);
                break;
            end
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
