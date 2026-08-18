function [isOccupied, blockingObstacleIndex, queryDetails] = ...
        queryAzElTimeObstacle(obstacles, azimuth_deg, elevation_deg, ...
        queryTime, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = queryAzElTimeObstacle()
%   isOccupied = queryAzElTimeObstacle( ...
%       obstacles, azimuth_deg, elevation_deg, queryTime)
%   [isOccupied, blockingObstacleIndex, queryDetails] = ...
%       queryAzElTimeObstacle(obstacles, azimuth_deg, elevation_deg, ...
%       queryTime, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Query canonical protected polygon geometry at explicit times.
%   - Return signed-clearance diagnostics from the same geometry queried.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle array, nested cells, or [])
%       Safety margins must already be present in protected geometry.
%   - azimuth_deg, elevation_deg (numeric arrays or scalars)
%       Non-scalar inputs must have equal element counts.
%   - queryTime (numeric seconds or datetime array)
%       Numeric values use the supplied ReferenceTime only for provenance.
%   - optionOverrides (scalar struct, optional; default struct())
%       BoundaryIsOccupied is logical (default true).
%       ClearanceTolerance_deg is nonnegative (default 1e-10).
%       ReferenceTime is a datetime scalar (default Unix epoch).
%**************************************************************************
% OUTPUTS
%   - isOccupied (logical array)
%       True for protected-interior points and, by default, boundary points.
%   - blockingObstacleIndex (uint32 array)
%       First blocking obstacle index, or zero when clear.
%   - queryDetails (scalar struct)
%       Signed minimum clearance, blocker names, times, margins, and options.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Numeric time is seconds.
%**************************************************************************

%% Section 1: Resolve Options And Obstacles

defaults = struct( ...
    "BoundaryIsOccupied", true, ...
    "ClearanceTolerance_deg", 1e-10, ...
    "ReferenceTime", datetime(1970, 1, 1, 0, 0, 0, ...
    "TimeZone", "UTC"));
if nargin == 0
    isOccupied = defaults;
    blockingObstacleIndex = [];
    queryDetails = struct();
    return;
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("queryAzElTimeObstacle:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
[options, unknownNames] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("queryAzElTimeObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.BoundaryIsOccupied = azElInternal.normalizeLogicalScalar( ...
    options.BoundaryIsOccupied, "BoundaryIsOccupied", ...
    "queryAzElTimeObstacle:InvalidBoundaryPolicy");
validateattributes(options.ClearanceTolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
if ~isdatetime(options.ReferenceTime) || ...
        ~isscalar(options.ReferenceTime) || isnat(options.ReferenceTime)
    error("queryAzElTimeObstacle:InvalidReferenceTime", ...
        "ReferenceTime must be one finite datetime scalar.");
end
options.ReferenceTime.TimeZone = "UTC";
obstacles = combineAzElObstacles(obstacles);

%% Section 2: Broadcast Query Arrays

if isdatetime(queryTime)
    queryTime.TimeZone = "UTC";
    queryTime_s = seconds(queryTime - options.ReferenceTime);
elseif isnumeric(queryTime)
    queryTime_s = double(queryTime);
else
    error("queryAzElTimeObstacle:InvalidTime", ...
        "queryTime must be numeric seconds or a datetime array.");
end
[queryAzimuth_deg, queryElevation_deg, queryTime_s, outputSize] = ...
    broadcastQueries(azimuth_deg, elevation_deg, queryTime_s);
queryCount = numel(queryTime_s);
isOccupied = false(queryCount, 1);
blockingObstacleIndex = zeros(queryCount, 1, "uint32");
minimumClearance_deg = Inf(queryCount, 1);
nearestObstacleIndex = zeros(queryCount, 1, "uint32");

%% Section 3: Query Interpolated Protected Geometry

clearanceTolerance_deg = double(options.ClearanceTolerance_deg);
for queryIndex = 1:queryCount
    if ~all(isfinite([queryAzimuth_deg(queryIndex), ...
            queryElevation_deg(queryIndex), queryTime_s(queryIndex)]))
        minimumClearance_deg(queryIndex) = NaN;
        continue;
    end
    point_deg = [queryAzimuth_deg(queryIndex), ...
        queryElevation_deg(queryIndex)];
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacleShapeAtTime( ...
            obstacles(obstacleIndex), queryTime_s(queryIndex));
        clearance_deg = azElInternal.pointPolygonClearance( ...
            shape, point_deg);
        if clearance_deg < minimumClearance_deg(queryIndex)
            minimumClearance_deg(queryIndex) = clearance_deg;
            nearestObstacleIndex(queryIndex) = uint32(obstacleIndex);
        end
        isBoundaryBlocked = options.BoundaryIsOccupied && ...
            clearance_deg <= clearanceTolerance_deg;
        isInteriorBlocked = clearance_deg < -clearanceTolerance_deg;
        if blockingObstacleIndex(queryIndex) == 0 && ...
                (isBoundaryBlocked || isInteriorBlocked)
            isOccupied(queryIndex) = true;
            blockingObstacleIndex(queryIndex) = uint32(obstacleIndex);
        end
    end
end

%% Section 4: Assemble Outputs

isOccupied = reshape(isOccupied, outputSize);
blockingObstacleIndex = reshape(blockingObstacleIndex, outputSize);
minimumClearance_deg = reshape(minimumClearance_deg, outputSize);
nearestObstacleIndex = reshape(nearestObstacleIndex, outputSize);
obstacleNames = strings(outputSize);
for obstacleIndex = 1:numel(obstacles)
    obstacleNames(nearestObstacleIndex == obstacleIndex) = ...
        obstacles(obstacleIndex).targetName;
end
queryDetails = struct( ...
    "MinimumClearance_deg", minimumClearance_deg, ...
    "NearestObstacleIndex", nearestObstacleIndex, ...
    "NearestObstacleName", obstacleNames, ...
    "QueryTime_s", reshape(queryTime_s, outputSize), ...
    "ObstacleSafetyMargins_deg", reshape( ...
    [obstacles.safetyMargin_deg], [], 1), ...
    "Options", options);
end

%% Section 5: Local Functions

function [azimuth_deg, elevation_deg, time_s, outputSize] = ...
        broadcastQueries(azimuth_deg, elevation_deg, time_s)
% PURPOSE
%   - Apply scalar expansion while preserving one caller-provided shape.
values = {double(azimuth_deg), double(elevation_deg), double(time_s)};
elementCounts = zeros(1, 3);
for valueIndex = 1:3
    elementCounts(valueIndex) = numel(values{valueIndex});
end
nonScalarCounts = elementCounts(elementCounts > 1);
if isempty(nonScalarCounts)
    outputSize = size(values{1});
    queryCount = 1;
elseif any(nonScalarCounts ~= nonScalarCounts(1))
    error("queryAzElTimeObstacle:SizeMismatch", ...
        "Non-scalar azimuth, elevation, and time inputs must have " + ...
        "equal element counts.");
else
    queryCount = nonScalarCounts(1);
    shapeSourceIndex = find(elementCounts == queryCount, 1, "first");
    outputSize = size(values{shapeSourceIndex});
end
for valueIndex = 1:3
    if elementCounts(valueIndex) == 1
        values{valueIndex} = repmat(values{valueIndex}, queryCount, 1);
    end
    values{valueIndex} = values{valueIndex}(:);
end
azimuth_deg = values{1};
elevation_deg = values{2};
time_s = values{3};
end
