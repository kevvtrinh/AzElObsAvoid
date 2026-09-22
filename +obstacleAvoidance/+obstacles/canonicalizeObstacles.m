function obstacles = canonicalizeObstacles(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.canonicalizeObstacles(obstacles)
%**************************************************************************
% PURPOSE
%   - Convert supported public obstacle inputs into standard records in
%     caller order.
%**************************************************************************
% INPUTS
%   - obstacles (empty, struct array, or nested cell array)
%       Standard obstacle records, optionally grouped in nested cells, or a
%       flat array of static polygon structs with Vertices_units and optional
%       Name and SafetyMargin_units.
%**************************************************************************
% OUTPUTS
%   - obstacles (struct array or empty)
%       Converted records form a column array; existing standard records keep
%       their shape. Empty input stays empty. Invalid converted inputs throw;
%       remaining record checks happen during obstacle preparation.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Normalize The Public Format

% Turn groups such as {A, {B, C}} into one column of records: [A; B; C].
if iscell(obstacles)
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end

% A Vertices_units input describes a static polygon. Give it the same
% fields as obstacle histories so later planning stages use one format.
if ~isempty(obstacles) && isstruct(obstacles) && isfield(obstacles, 'Vertices_units')
    convertedObstacles = cell(numel(obstacles), 1);
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
        % One sample means the obstacle is stationary for all times; time = 0
        % labels that sample. createObstacle includes the safety margin once.
        convertedObstacles{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
            obstacleName, 0, vertices_units(:, 1), vertices_units(:, 2), safetyMargin_units);
    end
    obstacles = vertcat(convertedObstacles{:});
end
end
