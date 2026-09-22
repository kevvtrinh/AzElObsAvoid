function [occupied, blockingIndex] = queryObstacleOccupancyAtTime( ...
    obstacles, x_units, y_units, time_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [occupied, blockingIndex] = ...
%       obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, x_units, y_units, time_s)
%   [occupied, blockingIndex] = ...
%       obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, x_units, y_units, time_s, options)
%**************************************************************************
% PURPOSE
%   - Check whether each point is inside an obstacle at its specified time,
%     including the obstacle's safety margin. This checks the given points;
%     it does not check the path between them.
%**************************************************************************
% INPUTS
%   - obstacles (standard obstacle array)
%       Obstacle histories to prepare over the range of requested times.
%   - x_units (numeric array)
%       Query x-coordinates.
%   - y_units (numeric array)
%       Query y-coordinates matching x_units.
%   - time_s (numeric scalar or array)
%       A single time applies to every supplied point. Otherwise, time_s
%       must have the same size as x_units and y_units.
%   - options (scalar struct, optional; default struct())
%       BoundaryIsOccupied defaults to true: a point on the protected
%       boundary counts as occupied. Use false to count only interior points.
%**************************************************************************
% OUTPUTS
%   - occupied (logical array)
%       True for occupied points; same size as x_units.
%   - blockingIndex (integer array)
%       Index of the first obstacle covering each point, or 0 if clear.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Check Query Coordinates And The Boundary Option

if nargin < 5
    options = struct();
end
validateattributes(x_units, {'numeric'}, {'real', 'finite'});
validateattributes(y_units, {'numeric'}, {'real', 'finite', 'size', size(x_units)});
validateattributes(time_s, {'numeric'}, {'real', 'finite'});
if isscalar(time_s)
    time_s = repmat(time_s, size(x_units));
end
assert(isequal(size(time_s), size(x_units)), ...
    'queryObstacleOccupancyAtTime:SizeMismatch', 'Query arrays must have equal sizes.');

boundaryIsOccupied = true;
if isfield(options, 'BoundaryIsOccupied')
    boundaryIsOccupied = options.BoundaryIsOccupied;
end
boundaryIsOccupied = obstacleAvoidance.input.normalizeLogicalScalar( ...
    boundaryIsOccupied, 'BoundaryIsOccupied', ...
    'queryObstacleOccupancyAtTime:InvalidBoundaryPolicy');

%% Section 2: Check Points Against Prepared Obstacles

occupied      = false(size(x_units));
blockingIndex = zeros(size(x_units), 'uint32');
if isempty(time_s)
    return;
end
% Prepare the geometry needed between the earliest and latest query times.
% The point check then selects each obstacle's shape at each requested time.
obstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
    obstacles, [min(time_s(:)), max(time_s(:))]);
[occupied, blockingIndex] = obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
    obstacles, x_units, y_units, time_s, boundaryIsOccupied);
end
