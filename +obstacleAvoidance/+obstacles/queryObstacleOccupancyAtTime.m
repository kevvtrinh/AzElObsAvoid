function [isOccupied, blockingObstacleIndex, queryDetails] = ...
        queryObstacleOccupancyAtTime(obstacles, azimuth_deg, elevation_deg, ...
        queryTime, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime()
%   isOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, azimuth_deg, elevation_deg, queryTime)
%   [isOccupied, blockingObstacleIndex, queryDetails] = ...
%       obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, azimuth_deg, elevation_deg, queryTime, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Query protected polygon occupancy at explicit physical times.
%   - Return signed-clearance and nearest-obstacle diagnostics.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle array, nested cells, or [])
%   - azimuth_deg, elevation_deg (numeric arrays or scalars)
%   - queryTime (numeric seconds or datetime array)
%   - optionOverrides (scalar struct, optional; default struct())
%       BoundaryIsOccupied, ClearanceTolerance_deg, and ReferenceTime.
%**************************************************************************
% OUTPUTS
%   - isOccupied (logical array)
%   - blockingObstacleIndex (uint32 array)
%       First blocker in caller order, or zero when clear.
%   - queryDetails (scalar struct)
%       Signed clearance, nearest obstacle, times, margins, and options.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Numeric time is seconds.
%**************************************************************************

%% Section 1: Resolve Options And Queries

defaults = struct("BoundaryIsOccupied", true, "ClearanceTolerance_deg", 1e-10, "ReferenceTime", ...
    datetime(1970, 1, 1, 0, 0, 0, "TimeZone", "UTC"));
if nargin == 0
    isOccupied = defaults;
    blockingObstacleIndex = [];
    queryDetails = struct();
    return;
end
if nargin ~= 4 && nargin ~= 5
    error("queryObstacleOccupancyAtTime:InvalidCall", ...
        "Use zero inputs, four query inputs, or four inputs plus options.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("queryObstacleOccupancyAtTime:InvalidOptions", "options must be a scalar struct.");
end
[options, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("queryObstacleOccupancyAtTime:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options.BoundaryIsOccupied = obstacleAvoidance.input.normalizeLogicalScalar( ...
    options.BoundaryIsOccupied, "BoundaryIsOccupied", ...
    "queryObstacleOccupancyAtTime:InvalidBoundaryPolicy");
validateattributes(options.ClearanceTolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
if ~isdatetime(options.ReferenceTime) || ~isscalar(options.ReferenceTime) || ...
        isnat(options.ReferenceTime)
    error("queryObstacleOccupancyAtTime:InvalidReferenceTime", ...
        "ReferenceTime must be one finite datetime scalar.");
end
options.ReferenceTime.TimeZone = "UTC";
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
if isdatetime(queryTime)
    queryTime.TimeZone = "UTC";
    queryTime_s = seconds(queryTime - options.ReferenceTime);
elseif isnumeric(queryTime)
    queryTime_s = double(queryTime);
else
    error("queryObstacleOccupancyAtTime:InvalidTime", ...
        "queryTime must be numeric seconds or datetime.");
end
[azimuth_deg, elevation_deg, queryTime_s, outputSize] = ...
    broadcastQueries(azimuth_deg, elevation_deg, queryTime_s);

%% Section 2: Evaluate Each Distinct Time Once

isOccupied = false(numel(queryTime_s), 1);
blockingObstacleIndex = zeros(numel(queryTime_s), 1, "uint32");
minimumClearance_deg = Inf(numel(queryTime_s), 1);
nearestObstacleIndex = zeros(numel(queryTime_s), 1, "uint32");
finiteQuery = isfinite(azimuth_deg) & isfinite(elevation_deg) & isfinite(queryTime_s);
minimumClearance_deg(~finiteQuery) = NaN;
uniqueTime_s = unique(queryTime_s(finiteQuery));
tolerance_deg = double(options.ClearanceTolerance_deg);
if isempty(obstacles)
    obstacleBounds_deg = zeros(0, 4);
else
    preparation = [obstacles.InternalPreparation];
    obstacleBounds_deg = vertcat(preparation.HistoryBounds_deg);
end
for timeIndex = 1:numel(uniqueTime_s)
    queryIndices = find(finiteQuery & queryTime_s == uniqueTime_s(timeIndex));
    for obstacleIndex = 1:numel(obstacles)
        candidate = queryIndices;
        if nargout < 2
            candidate = candidate(~isOccupied(candidate));
            bound_deg = obstacleBounds_deg(obstacleIndex, :);
            inBounds = azimuth_deg(candidate) >= bound_deg(1) - tolerance_deg & ...
                azimuth_deg(candidate) <= bound_deg(2) + tolerance_deg & ...
                elevation_deg(candidate) >= bound_deg(3) - tolerance_deg & ...
                elevation_deg(candidate) <= bound_deg(4) + tolerance_deg;
            candidate = candidate(inBounds);
        end
        if isempty(candidate)
            continue;
        end
        shape = obstacleAvoidance.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), uniqueTime_s(timeIndex));
        points_deg = [azimuth_deg(candidate), elevation_deg(candidate)];
        clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance(shape, points_deg);
        if nargout >= 2
            priorClearance_deg = minimumClearance_deg(candidate);
            closer = clearance_deg < priorClearance_deg;
            priorClearance_deg(closer) = clearance_deg(closer);
            minimumClearance_deg(candidate) = priorClearance_deg;
            nearestObstacleIndex(candidate(closer)) = uint32(obstacleIndex);
        end
        blocked = clearance_deg < -tolerance_deg | ...
            (options.BoundaryIsOccupied & clearance_deg <= tolerance_deg);
        firstBlocker = blocked & blockingObstacleIndex(candidate) == 0;
        blockingObstacleIndex(candidate(firstBlocker)) = uint32(obstacleIndex);
        isOccupied(candidate(blocked)) = true;
    end
end

%% Section 3: Assemble Stable Outputs

isOccupied = reshape(isOccupied, outputSize);
if nargout < 2
    return;
end
blockingObstacleIndex = reshape(blockingObstacleIndex, outputSize);
minimumClearance_deg = reshape(minimumClearance_deg, outputSize);
nearestObstacleIndex = reshape(nearestObstacleIndex, outputSize);
obstacleNames = strings(outputSize);
for obstacleIndex = 1:numel(obstacles)
    obstacleNames(nearestObstacleIndex == obstacleIndex) = obstacles(obstacleIndex).targetName;
end
queryDetails = struct("MinimumClearance_deg", minimumClearance_deg, ...
    "NearestObstacleIndex", nearestObstacleIndex, "NearestObstacleName", obstacleNames, ...
    "QueryTime_s", reshape(queryTime_s, outputSize), "ObstacleSafetyMargins_deg", ...
    reshape([obstacles.safetyMargin_deg], [], 1), "Options", options);
end

function [azimuth_deg, elevation_deg, time_s, outputSize] = ...
        broadcastQueries(azimuth_deg, elevation_deg, time_s)
% Apply scalar expansion and retain the first nonscalar input shape.
values = {double(azimuth_deg), double(elevation_deg), double(time_s)};
counts = [numel(values{1}), numel(values{2}), numel(values{3})];
queryCount = max(counts);
if any(counts ~= 1 & counts ~= queryCount)
    error("queryObstacleOccupancyAtTime:SizeMismatch", ...
        "Non-scalar azimuth, elevation, and time must have equal counts.");
end
outputSize = size(values{find(counts == queryCount, 1)});
for valueIndex = 1:3
    if counts(valueIndex) == 1
        values{valueIndex} = repmat(values{valueIndex}, queryCount, 1);
    end
    values{valueIndex} = values{valueIndex}(:);
end
[azimuth_deg, elevation_deg, time_s] = deal(values{:});
end
