function [occupied, blockingIndex] = queryObstacleOccupancyAtTime(obstacles, x_units, y_units, time_s, options)
%% Section 0: Header & Readme
% SYNTAX: [occupied, blockingIndex] = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles,x,y,time,options)
% PURPOSE: Query the original protected history model at physical times.
% INPUTS: Canonical obstacles and equally sized coordinate/time arrays.
%         A scalar time broadcasts. BoundaryIsOccupied defaults to true.
% OUTPUTS: Logical occupancy and first blocking obstacle index.
% UNITS: Seconds and coordinate units.

%% Section 1: Validate Query Coordinates
if nargin < 5, options = struct(); end
validateattributes(x_units, {'numeric'}, {'real','finite'});
validateattributes(y_units, {'numeric'}, {'real','finite','size',size(x_units)});
validateattributes(time_s, {'numeric'}, {'real','finite'});
if isscalar(time_s), time_s = repmat(time_s, size(x_units)); end
assert(isequal(size(time_s),size(x_units)), 'queryObstacleOccupancyAtTime:SizeMismatch', 'Query arrays must have equal sizes.');
boundaryOccupied = true;
if isfield(options,'BoundaryIsOccupied'), boundaryOccupied = options.BoundaryIsOccupied; end
boundaryOccupied = obstacleAvoidance.input.normalizeLogicalScalar(boundaryOccupied, 'BoundaryIsOccupied', 'queryObstacleOccupancyAtTime:InvalidBoundaryPolicy');
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles);
occupied = false(size(x_units)); blockingIndex = zeros(size(x_units),'uint32');

%% Section 2: Batch Identical Geometry And Equal-Time Queries
queryTimes_s = unique(time_s(:));
for j = 1:numel(obstacles)
    obstacle = obstacles(j);
    % Require exact source equality, not tolerance-based shape equivalence.
    identical = all(cellfun(@(x,y) isequaln(x,obstacle.x_units{1}) && ...
        isequaln(y,obstacle.y_units{1}),obstacle.x_units,obstacle.y_units));
    if identical
        active = true(size(time_s));
        if numel(obstacle.time_s)>1
            active = time_s>=obstacle.time_s(1) & time_s<=obstacle.time_s(end);
        end
        indices = find(active & ~occupied);
        [inside,on] = inpolygon(x_units(indices),y_units(indices),obstacle.x_units{1},obstacle.y_units{1});
        fresh = indices(inside & (~on | boundaryOccupied));
        occupied(fresh) = true; blockingIndex(fresh) = j;
        continue;
    end
    for k = 1:numel(queryTimes_s)
        indices = find(time_s == queryTimes_s(k) & ~occupied);
        if isempty(indices), continue; end
        [~, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacles(j),queryTimes_s(k),true);
        if ~geometry.Active, continue; end
        [inside, on] = inpolygon(x_units(indices),y_units(indices),geometry.x_units,geometry.y_units);
        hit = inside & (~on | boundaryOccupied);
        fresh = indices(hit & ~occupied(indices));
        occupied(fresh) = true; blockingIndex(fresh) = j;
    end
end
end
