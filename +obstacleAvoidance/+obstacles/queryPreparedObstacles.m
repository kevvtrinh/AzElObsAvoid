function [isOccupied, blockingObstacleIndex, queryDetails] = queryPreparedObstacles(obstacles, x_units, y_units, queryTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [isOccupied, blockingObstacleIndex, queryDetails] = ...
%       queryPreparedObstacles(obstacles, x_units, y_units, queryTime_s, options)
% PURPOSE
%   Check positions against prepared obstacles without rebuilding their geometry.
% INPUTS
%   prepared obstacles, numeric position/time queries, and optional boundary/clearance settings.
% OUTPUTS
%   occupied flags, first blocking obstacle indices, and optional clearance details.
% UNITS
%   Positions are coordinate units; time is seconds; derivatives retain physical units.

%% Section 1: Evaluate Prepared Inputs
if nargin < 5, options = struct(); end
if ~isfield(options, "BoundaryIsOccupied"), options.BoundaryIsOccupied = true; end
if ~isfield(options, "ClearanceTolerance_units"), options.ClearanceTolerance_units = 1e-10; end
[x_units, y_units, queryTime_s, outputSize] = broadcastQueries(x_units, y_units, queryTime_s);

%% Section 2: Evaluate Each Distinct Geometry Once

isOccupied = false(numel(queryTime_s), 1);
if nargout >= 2
    blockingObstacleIndex = zeros(numel(queryTime_s), 1, "uint32");
end
if nargout >= 3
    minimumClearance_units = Inf(numel(queryTime_s), 1);
    nearestObstacleIndex = zeros(numel(queryTime_s), 1, "uint32");
end
finiteQuery = isfinite(x_units) & isfinite(y_units) & isfinite(queryTime_s);
if nargout >= 3, minimumClearance_units(~finiteQuery) = NaN; end
uniqueTime_s  = unique(queryTime_s(finiteQuery));
tolerance_units = double(options.ClearanceTolerance_units);
if isempty(obstacles)
    obstacleBounds_units = zeros(0, 4);
else
    preparation        = [obstacles.InternalPreparation];
    obstacleBounds_units = vertcat(preparation.HistoryBounds_units);
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
            bound_units = obstacleBounds_units(obstacleIndex, :);
            inBounds  = x_units(candidate) >= bound_units(1) - tolerance_units & x_units(candidate) <= bound_units(2) + tolerance_units & y_units(candidate) >= bound_units(3) - tolerance_units & y_units(candidate) <= bound_units(4) + tolerance_units;
            candidate = candidate(inBounds);
        end
        if isempty(candidate)
            continue;
        end
        [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, obstacleQueryTime_s(timeIndex));
        points_units    = [x_units(candidate), y_units(candidate)];
        clearance_units = obstacleAvoidance.geometry.pointPolygonClearance(shape, points_units, geometry);
        if nargout >= 3
            priorClearance_units = minimumClearance_units(candidate);
            closer             = clearance_units < priorClearance_units;
            priorClearance_units(closer) = clearance_units(closer);
            minimumClearance_units(candidate) = priorClearance_units;
            nearestObstacleIndex(candidate(closer)) = uint32(obstacleIndex);
        end
        blocked = clearance_units < -tolerance_units | (options.BoundaryIsOccupied & clearance_units <= tolerance_units);
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
minimumClearance_units = reshape(minimumClearance_units, outputSize);
nearestObstacleIndex = reshape(nearestObstacleIndex, outputSize);
obstacleNames        = strings(outputSize);
% Evaluate each obstacle against the current geometry or motion.
for obstacleIndex = 1:numel(obstacles)
    obstacleNames(nearestObstacleIndex == obstacleIndex) = obstacles(obstacleIndex).targetName;
end
queryDetails = struct("MinimumClearance_units", minimumClearance_units, ...
    "NearestObstacleIndex", nearestObstacleIndex, "NearestObstacleName", obstacleNames, ...
    "QueryTime_s", reshape(queryTime_s, outputSize), "ObstacleSafetyMargins_units", ...
    reshape([obstacles.safetyMargin_units], [], 1), "Options", options);
end

function [x_units, y_units, time_s, outputSize] = broadcastQueries(x_units, y_units, time_s)
    % Apply scalar expansion and retain the first nonscalar input shape.
    values     = {double(x_units), double(y_units), double(time_s)};
    counts     = [numel(values{1}), numel(values{2}), numel(values{3})];
    queryCount = max(counts);
    if any(counts ~= 1 & counts ~= queryCount)
        error("queryObstacleOccupancyAtTime:SizeMismatch", "Non-scalar x, y, and time must have equal counts.");
    end
    outputSize = size(values{find(counts == queryCount, 1)});
    % Process each value needed to complete broadcast queries.
    for valueIndex = 1:3
        if counts(valueIndex) == 1
            values{valueIndex} = repmat(values{valueIndex}, queryCount, 1);
        end
        values{valueIndex} = values{valueIndex}(:);
    end
    [x_units, y_units, time_s] = deal(values{:});
end
