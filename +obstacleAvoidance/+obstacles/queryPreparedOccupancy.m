function [occupied, blockingIndex] = queryPreparedOccupancy( ...
    obstacles, x_units, y_units, time_s, boundaryIsOccupied)
%% Section 0: Header & Readme
% SYNTAX
%   [occupied, blockingIndex] = ...
%       obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
%       obstacles, x_units, y_units, time_s, boundaryIsOccupied)
%**************************************************************************
% PURPOSE
%   - Query occupancy from an already prepared obstacle snapshot.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array)
%       Source-checked prepared obstacle histories.
%   - x_units (numeric array)
%       Query x-coordinates.
%   - y_units (numeric array)
%       Query y-coordinates matching x_units.
%   - time_s (numeric scalar or array)
%       Query times; a scalar broadcasts to the coordinate-array size.
%   - boundaryIsOccupied (logical scalar)
%       Whether points on a protected boundary count as occupied.
%**************************************************************************
% OUTPUTS
%   - occupied (logical array)
%       Occupancy for each query.
%   - blockingIndex (integer array)
%       First blocking obstacle index for each query.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Broadcast Query Times

if isscalar(time_s)
    time_s = repmat(time_s, size(x_units));
end
occupied      = false(size(x_units));
blockingIndex = zeros(size(x_units), 'uint32');
if isempty(time_s)
    return;
end

%% Section 2: Batch Identical Geometry And Equal-Time Queries

queryTimes_s = unique(time_s(:));
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    % Require exact source equality, not tolerance-based shape equivalence.
    samplesAreExactlyEqual = obstacle.InternalPreparation.SamplesExactlyEqual;
    if samplesAreExactlyEqual
        queryIsActive = true(size(time_s));
        if numel(obstacle.time_s) > 1
            queryIsActive = time_s >= obstacle.time_s(1) & time_s <= obstacle.time_s(end);
        end
        availableIndices = find(queryIsActive & ~occupied);
        [inside, onBoundary] = inpolygon( ...
            x_units(availableIndices), y_units(availableIndices), ...
            obstacle.x_units{1}, obstacle.y_units{1});
        freshIndices = availableIndices(inside & (~onBoundary | boundaryIsOccupied));
        occupied(freshIndices)      = true;
        blockingIndex(freshIndices) = obstacleIndex;
        continue;
    end
    geometryCache    = [];
    cacheIsAvailable = isfield(obstacle.InternalPreparation, 'QueryGeometryCache');
    if cacheIsAvailable
        geometryCache = obstacle.InternalPreparation.QueryGeometryCache;
    end
    cachedGeometry = cell(size(queryTimes_s));
    if cacheIsAvailable
        queryKeys         = num2cell(queryTimes_s);
        cachedTimeIndices = isKey(geometryCache, queryKeys);
        cachedGeometry(cachedTimeIndices) = values(geometryCache, queryKeys(cachedTimeIndices));
    end
    for timeIndex = 1:numel(queryTimes_s)
        availableIndices = find(time_s == queryTimes_s(timeIndex) & ~occupied);
        if isempty(availableIndices)
            continue;
        end
        % The shared evaluator returns the swept-cell union at interior times
        % and authoritative normalized geometry at sample times.
        if ~isempty(cachedGeometry{timeIndex})
            geometry = cachedGeometry{timeIndex};
        else
            [~, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
                obstacle, queryTimes_s(timeIndex), true, false);
            if cacheIsAvailable && ...
                    geometryCache.Count < obstacle.InternalPreparation.QueryGeometryCacheCapacity
                geometryCache(queryTimes_s(timeIndex)) = struct( ...
                    'Active', geometry.Active, 'x_units', geometry.x_units, 'y_units', geometry.y_units); %#ok<AGROW> Bounded map insertion.
            end
        end
        if ~geometry.Active
            continue;
        end
        [inside, onBoundary] = inpolygon( ...
            x_units(availableIndices), y_units(availableIndices), geometry.x_units, geometry.y_units);
        queryHitsObstacle = inside & (~onBoundary | boundaryIsOccupied);
        freshIndices      = availableIndices(queryHitsObstacle);
        occupied(freshIndices)      = true;
        blockingIndex(freshIndices) = obstacleIndex;
    end
end
end
