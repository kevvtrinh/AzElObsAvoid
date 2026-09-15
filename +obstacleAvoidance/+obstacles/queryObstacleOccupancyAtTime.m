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
%   - Query protected obstacle occupancy at physical times.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle array)
%       Obstacles to prepare over the query-time range.
%   - x_units (numeric array)
%       Query x-coordinates.
%   - y_units (numeric array)
%       Query y-coordinates matching x_units.
%   - time_s (numeric scalar or array)
%       Query times; a scalar broadcasts to the coordinate-array size.
%   - options (scalar struct, optional; default struct())
%       BoundaryIsOccupied defaults to true. Scalar time broadcasts.
%**************************************************************************
% OUTPUTS
%   - occupied (logical array)
%       Occupancy for each query.
%   - blockingIndex (integer array)
%       First blocking obstacle index for each query. Invalid input throws
%       an error.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Validate Query Coordinates

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
occupied      = false(size(x_units));
blockingIndex = zeros(size(x_units), 'uint32');
if isempty(time_s)
    return;
end
obstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
    obstacles, [min(time_s(:)), max(time_s(:))]);

%% Section 2: Query The Prepared Snapshot

[occupied, blockingIndex] = obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
    obstacles, x_units, y_units, time_s, boundaryIsOccupied);
end
