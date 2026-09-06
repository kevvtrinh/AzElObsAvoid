function [isOccupied, blockingObstacleIndex, queryDetails] = queryPreparedObstacles(obstacles, azimuth_deg, elevation_deg, queryTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [isOccupied, blockingObstacleIndex, queryDetails] = ...
%       queryPreparedObstacles(obstacles, azimuth_deg, elevation_deg, queryTime_s, options)
% PURPOSE
%   Check positions against prepared obstacles without rebuilding their geometry.
% INPUTS
%   prepared obstacles, numeric position/time queries, and optional boundary/clearance settings.
% OUTPUTS
%   occupied flags, first blocking obstacle indices, and optional clearance details.
% UNITS
%   Positions are degrees; time is seconds; derivatives retain physical units.

%% Section 1: Evaluate Prepared Inputs
if nargin < 5, options = struct(); end
if ~isfield(options, "BoundaryIsOccupied"), options.BoundaryIsOccupied = true; end
if ~isfield(options, "ClearanceTolerance_deg"), options.ClearanceTolerance_deg = 1e-10; end
[azimuth_deg, elevation_deg, queryTime_s, outputSize] = broadcastQueries(azimuth_deg, elevation_deg, queryTime_s);

%% Section 2: Evaluate Each Distinct Geometry Once

isOccupied = false(numel(queryTime_s), 1);
if nargout >= 2
    blockingObstacleIndex = zeros(numel(queryTime_s), 1, "uint32");
end
if nargout >= 3
    minimumClearance_deg = Inf(numel(queryTime_s), 1);
    nearestObstacleIndex = zeros(numel(queryTime_s), 1, "uint32");
end
finiteQuery = isfinite(azimuth_deg) & isfinite(elevation_deg) & isfinite(queryTime_s);
if nargout >= 3, minimumClearance_deg(~finiteQuery) = NaN; end
uniqueTime_s  = unique(queryTime_s(finiteQuery));
tolerance_deg = double(options.ClearanceTolerance_deg);
if isempty(obstacles)
    obstacleBounds_deg = zeros(0, 4);
else
    preparation        = [obstacles.InternalPreparation];
    obstacleBounds_deg = vertcat(preparation.HistoryBounds_deg);
end
% Evaluate each obstacle against the current geometry or motion.
for obstacleIndex = 1:numel(obstacles)
    obstacle                = obstacles(obstacleIndex);
    obstacleIsTimeInvariant = obstacle.InternalPreparation.IsTimeInvariant;
    obstacleQueryTime_s     = uniqueTime_s;
    activeQuery             = finiteQuery;
    if obstacleIsTimeInvariant
        % Query a static shape once for all points within its active times.
        obstacleTime_s      = double(obstacle.time_s(:));
        obstacleQueryTime_s = obstacleTime_s(1);
        activeQuery         = finiteQuery & (isscalar(obstacleTime_s) | (queryTime_s >= obstacleTime_s(1) & queryTime_s <= obstacleTime_s(end)));
    end
    % Process each time in temporal order and accumulate its result.
    for timeIndex = 1:numel(obstacleQueryTime_s)
        queryIndices = find(activeQuery & (obstacleIsTimeInvariant | queryTime_s == obstacleQueryTime_s(timeIndex)));
        candidate    = queryIndices;
        if nargout < 3
            candidate = candidate(~isOccupied(candidate));
            bound_deg = obstacleBounds_deg(obstacleIndex, :);
            inBounds  = azimuth_deg(candidate) >= bound_deg(1) - tolerance_deg & azimuth_deg(candidate) <= bound_deg(2) + tolerance_deg & elevation_deg(candidate) >= bound_deg(3) - tolerance_deg & elevation_deg(candidate) <= bound_deg(4) + tolerance_deg;
            candidate = candidate(inBounds);
        end
        if isempty(candidate)
            continue;
        end
        [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, obstacleQueryTime_s(timeIndex));
        points_deg    = [azimuth_deg(candidate), elevation_deg(candidate)];
        clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance(shape, points_deg, geometry);
        if nargout >= 3
            priorClearance_deg = minimumClearance_deg(candidate);
            closer             = clearance_deg < priorClearance_deg;
            priorClearance_deg(closer) = clearance_deg(closer);
            minimumClearance_deg(candidate) = priorClearance_deg;
            nearestObstacleIndex(candidate(closer)) = uint32(obstacleIndex);
        end
        blocked = clearance_deg < -tolerance_deg | (options.BoundaryIsOccupied & clearance_deg <= tolerance_deg);
        if nargout >= 2
            firstBlocker = blocked & blockingObstacleIndex(candidate) == 0;
            blockingObstacleIndex(candidate(firstBlocker)) = uint32(obstacleIndex);
        end
        isOccupied(candidate(blocked)) = true;
    end
end

%% Section 3: Assemble Stable Outputs

isOccupied = reshape(isOccupied, outputSize);
if nargout < 2
    return;
end
blockingObstacleIndex = reshape(blockingObstacleIndex, outputSize);
if nargout < 3, return; end
minimumClearance_deg = reshape(minimumClearance_deg, outputSize);
nearestObstacleIndex = reshape(nearestObstacleIndex, outputSize);
obstacleNames        = strings(outputSize);
% Evaluate each obstacle against the current geometry or motion.
for obstacleIndex = 1:numel(obstacles)
    obstacleNames(nearestObstacleIndex == obstacleIndex) = obstacles(obstacleIndex).targetName;
end
queryDetails = struct("MinimumClearance_deg", minimumClearance_deg, ...
    "NearestObstacleIndex", nearestObstacleIndex, "NearestObstacleName", obstacleNames, ...
    "QueryTime_s", reshape(queryTime_s, outputSize), "ObstacleSafetyMargins_deg", ...
    reshape([obstacles.safetyMargin_deg], [], 1), "Options", options);
end

function [azimuth_deg, elevation_deg, time_s, outputSize] = broadcastQueries(azimuth_deg, elevation_deg, time_s)
    % Apply scalar expansion and retain the first nonscalar input shape.
    values     = {double(azimuth_deg), double(elevation_deg), double(time_s)};
    counts     = [numel(values{1}), numel(values{2}), numel(values{3})];
    queryCount = max(counts);
    if any(counts ~= 1 & counts ~= queryCount)
        error("queryObstacleOccupancyAtTime:SizeMismatch", "Non-scalar azimuth, elevation, and time must have equal counts.");
    end
    outputSize = size(values{find(counts == queryCount, 1)});
    % Process each value needed to complete broadcast queries.
    for valueIndex = 1:3
        if counts(valueIndex) == 1
            values{valueIndex} = repmat(values{valueIndex}, queryCount, 1);
        end
        values{valueIndex} = values{valueIndex}(:);
    end
    [azimuth_deg, elevation_deg, time_s] = deal(values{:});
end
