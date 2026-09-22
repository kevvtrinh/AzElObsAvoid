function [occupied, blockingIndex] = queryPreparedOccupancy( ...
    obstacles, x_units, y_units, time_s, boundaryIsOccupied)
%% Section 0: Header & Readme
% SYNTAX
%   [occupied, blockingIndex] = ...
%       obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
%       obstacles, x_units, y_units, time_s, boundaryIsOccupied)
%**************************************************************************
% PURPOSE
%   - Check each point against the prepared obstacle shapes at its time.
%     Return the first obstacle covering that point, including its margin.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array)
%       Histories already checked and prepared for the requested time range.
%   - x_units (numeric array)
%       Query x-coordinates.
%   - y_units (numeric array)
%       Query y-coordinates matching x_units.
%   - time_s (numeric scalar or array)
%       One time for all points, or an array matching x_units and y_units.
%   - boundaryIsOccupied (logical scalar)
%       Whether points on a protected boundary count as occupied.
%**************************************************************************
% OUTPUTS
%   - occupied (logical array)
%       True for occupied points; same size as x_units.
%   - blockingIndex (integer array)
%       Index of the first obstacle covering each point, or 0 if clear.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Assign A Time To Each Query Point

if isscalar(time_s)
    time_s = repmat(time_s, size(x_units));
end
occupied      = false(size(x_units));
blockingIndex = zeros(size(x_units), 'uint32');
if isempty(time_s)
    return;
end

%% Section 2: Check Unblocked Points Against Each Obstacle

queryTimes_s = unique(time_s(:));
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    % Identical boundary samples use the same polygon for every active time.
    % Require exact equality so even a small movement gets the timed check.
    samplesAreExactlyEqual = obstacle.InternalPreparation.SamplesExactlyEqual;
    if samplesAreExactlyEqual
        queryIsActive = true(size(time_s));
        % One sample applies at all times. Several identical samples still
        % limit the obstacle to the time range covered by its history.
        if numel(obstacle.time_s) > 1
            queryIsActive = time_s >= obstacle.time_s(1) & time_s <= obstacle.time_s(end);
        end
        % Once a point is blocked, keep its first blocking obstacle index.
        pointIndicesToCheck = find(queryIsActive & ~occupied);
        [pointIsInside, pointIsOnBoundary] = inpolygon( ...
            x_units(pointIndicesToCheck), y_units(pointIndicesToCheck), ...
            obstacle.x_units{1}, obstacle.y_units{1});
        % inpolygon includes boundary points. Apply the caller's boundary rule.
        pointIsBlocked      = pointIsInside & (~pointIsOnBoundary | boundaryIsOccupied);
        blockedPointIndices = pointIndicesToCheck(pointIsBlocked);

        occupied(blockedPointIndices)      = true;
        blockingIndex(blockedPointIndices) = obstacleIndex;
        continue;
    end

    % Reuse shapes for repeated times. The cache belongs to this obstacle
    % and preparation checks that it still matches the supplied boundaries.
    obstacleShapeCache = [];
    cacheIsAvailable   = isfield(obstacle.InternalPreparation, 'QueryGeometryCache');
    if cacheIsAvailable
        obstacleShapeCache = obstacle.InternalPreparation.QueryGeometryCache;
    end
    cachedShapeDetails = cell(size(queryTimes_s));
    if cacheIsAvailable
        timeCacheKeys = num2cell(queryTimes_s);
        timeIsCached  = isKey(obstacleShapeCache, timeCacheKeys);
        cachedShapeDetails(timeIsCached) = values(obstacleShapeCache, timeCacheKeys(timeIsCached));
    end

    % Points at the same time share one shape calculation.
    for timeIndex = 1:numel(queryTimes_s)
        pointIndicesToCheck = find(time_s == queryTimes_s(timeIndex) & ~occupied);
        if isempty(pointIndicesToCheck)
            continue;
        end
        % At a sample time, use its supplied protected boundary. Between
        % samples, the prepared model provides an interpolated boundary or
        % an enclosure covering the obstacle over that interval.
        if ~isempty(cachedShapeDetails{timeIndex})
            shapeDetails = cachedShapeDetails{timeIndex};
        else
            [~, shapeDetails] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
                obstacle, queryTimes_s(timeIndex), true, false);
            if cacheIsAvailable && ...
                    obstacleShapeCache.Count < obstacle.InternalPreparation.QueryGeometryCacheCapacity
                obstacleShapeCache(queryTimes_s(timeIndex)) = struct( ...
                    'Active',  shapeDetails.Active, ...
                    'x_units', shapeDetails.x_units, ...
                    'y_units', shapeDetails.y_units); %#ok<AGROW> Cache capacity checked above.
            end
        end
        % An obstacle outside its active history, or with no boundary at
        % this time, cannot block these points.
        if ~shapeDetails.Active
            continue;
        end
        [pointIsInside, pointIsOnBoundary] = inpolygon( ...
            x_units(pointIndicesToCheck), y_units(pointIndicesToCheck), shapeDetails.x_units, shapeDetails.y_units);
        pointIsBlocked      = pointIsInside & (~pointIsOnBoundary | boundaryIsOccupied);
        blockedPointIndices = pointIndicesToCheck(pointIsBlocked);

        occupied(blockedPointIndices)      = true;
        blockingIndex(blockedPointIndices) = obstacleIndex;
    end
end
end
