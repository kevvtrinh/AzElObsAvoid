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
occupied = false(size(x_units)); blockingIndex = zeros(size(x_units),'uint32');
if isempty(time_s), return; end
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles,[min(time_s(:)),max(time_s(:))]);

%% Section 2: Query The Prepared Snapshot
[occupied, blockingIndex] = obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
    obstacles,x_units,y_units,time_s,boundaryOccupied);
end
