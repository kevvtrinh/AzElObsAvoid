function [isOccupied, blockingObstacleIndex, queryDetails] = queryObstacleOccupancyAtTime(obstacles, x_units, y_units, queryTime, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime()
%   isOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, x_units, y_units, queryTime)
%   [isOccupied, blockingObstacleIndex, queryDetails] = ...
%       obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, x_units, y_units, queryTime, optionOverrides)
%
% PURPOSE
%   - Query protected polygon occupancy at explicit physical times.
%   - Return signed-clearance and nearest-obstacle diagnostics.
%
% INPUTS
%   - obstacles (canonical obstacle array, nested cells, or [])
%   - x_units, y_units (numeric arrays or scalars)
%   - queryTime (numeric seconds or datetime array)
%   - optionOverrides (scalar struct, optional; default struct())
%       BoundaryIsOccupied, ClearanceTolerance_units, and ReferenceTime.
%
% OUTPUTS
%   - isOccupied (logical array)
%   - blockingObstacleIndex (uint32 array)
%       First blocker in caller order, or zero when clear.
%   - queryDetails (scalar struct)
%       Signed clearance, nearest obstacle, times, margins, and options.
%
% UNITS
%   - Position and clearance are coordinate units. Numeric time is seconds.
%

%% Section 1: Resolve Options And Queries

defaults = struct("BoundaryIsOccupied", true, "ClearanceTolerance_units", 1e-10, "ReferenceTime", ...
    datetime(1970, 1, 1, 0, 0, 0, "TimeZone", "UTC"));
if nargin == 0
    isOccupied            = defaults;
    blockingObstacleIndex = [];
    queryDetails          = struct();
    return;
end
if nargin ~= 4 && nargin ~= 5
    error("queryObstacleOccupancyAtTime:InvalidCall", "Use zero inputs, four query inputs, or four inputs plus options.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("queryObstacleOccupancyAtTime:InvalidOptions", "options must be a scalar struct.");
end
[options, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("queryObstacleOccupancyAtTime:UnknownOptions", "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options.BoundaryIsOccupied = obstacleAvoidance.input.normalizeLogicalScalar(options.BoundaryIsOccupied, "BoundaryIsOccupied", "queryObstacleOccupancyAtTime:InvalidBoundaryPolicy");
validateattributes(options.ClearanceTolerance_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
if ~isdatetime(options.ReferenceTime) || ~isscalar(options.ReferenceTime) || isnat(options.ReferenceTime)
    error("queryObstacleOccupancyAtTime:InvalidReferenceTime", "ReferenceTime must be one finite datetime scalar.");
end
options.ReferenceTime.TimeZone = "UTC";
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles);
if isdatetime(queryTime)
    queryTime.TimeZone = "UTC";
    queryTime_s = seconds(queryTime - options.ReferenceTime);
elseif isnumeric(queryTime)
    queryTime_s = double(queryTime);
else
    error("queryObstacleOccupancyAtTime:InvalidTime", "queryTime must be numeric seconds or datetime.");
end
if nargout < 2
    isOccupied = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, x_units, y_units, queryTime_s, options);
elseif nargout == 2
    [isOccupied, blockingObstacleIndex] = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, x_units, y_units, queryTime_s, options);
else
    [isOccupied, blockingObstacleIndex, queryDetails] = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, x_units, y_units, queryTime_s, options);
end
end
