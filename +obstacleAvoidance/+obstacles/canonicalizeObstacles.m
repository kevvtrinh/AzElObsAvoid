function obstacles = canonicalizeObstacles(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.canonicalizeObstacles(obstacles)
%**************************************************************************
% PURPOSE
%   - Turn any public obstacle input into canonical obstacle records in
%     caller order.
%**************************************************************************
% INPUTS
%   - obstacles (empty, struct array, or nested cell array)
%       Canonical records, nested cells of records, or static polygon structs
%       with Vertices_units and optional Name and SafetyMargin_units.
%**************************************************************************
% OUTPUTS
%   - obstacles (column struct array)
%       Canonical records; empty input is returned unchanged. Invalid input
%       throws an error.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Normalize The Public Format

if iscell(obstacles)
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
if ~isempty(obstacles) && isstruct(obstacles) && isfield(obstacles, 'Vertices_units')
    canonicalObstacles = cell(numel(obstacles), 1);
    for obstacleIndex = 1:numel(obstacles)
        vertices_units = obstacles(obstacleIndex).Vertices_units;
        validateattributes(vertices_units, {'numeric'}, {'2d', 'ncols', 2, 'real', 'finite'});
        obstacleName       = "obstacle " + obstacleIndex;
        safetyMargin_units = 0;
        if isfield(obstacles, 'Name')
            obstacleName = obstacles(obstacleIndex).Name;
        end
        if isfield(obstacles, 'SafetyMargin_units')
            safetyMargin_units = obstacles(obstacleIndex).SafetyMargin_units;
        end
        canonicalObstacles{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
            obstacleName, 0, vertices_units(:, 1), vertices_units(:, 2), safetyMargin_units);
    end
    obstacles = vertcat(canonicalObstacles{:});
end
end
