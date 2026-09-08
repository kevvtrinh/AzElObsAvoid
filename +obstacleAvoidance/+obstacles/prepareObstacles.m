function preparedObstacles = prepareObstacles(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles)
%
% PURPOSE
%   - Validate static convex polygon obstacles.
%   - Apply each safety margin exactly once and retain original geometry.
%
% INPUTS
%   - obstacles (struct array or [])
%       Each element requires Vertices_units (N-by-2). Optional fields are
%       Name and SafetyMargin_units.
%
% OUTPUTS
%   - preparedObstacles (column struct array)
%       Original and protected convex vertices plus the protected polyshape.
%
% UNITS
%   - Vertices and safety margins are coordinate units.

%% Section 1: Validate The Collection

template = struct("Name", "", ...
    "OriginalVertices_units", zeros(0, 2), ...
    "SafetyMargin_units", 0, ...
    "ProtectedVertices_units", zeros(0, 2), ...
    "ProtectedShape", polyshape());
if isempty(obstacles)
    preparedObstacles = repmat(template, 0, 1);
    return;
end
if ~isstruct(obstacles) || ~isvector(obstacles)
    error("prepareObstacles:InvalidCollection", "obstacles must be a struct vector or empty.");
end

%% Section 2: Prepare Each Static Convex Polygon

preparedObstacles = repmat(template, numel(obstacles), 1);
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    if ~isfield(obstacle, "Vertices_units")
        error("prepareObstacles:MissingVertices", "Obstacle %d requires Vertices_units.", obstacleIndex);
    end
    vertices_units = double(obstacle.Vertices_units);
    if ~isnumeric(obstacle.Vertices_units) || size(vertices_units, 2) ~= 2 || size(vertices_units, 1) < 3 || any(~isfinite(vertices_units), "all")
        error("prepareObstacles:InvalidVertices", "Obstacle %d vertices must be a finite N-by-2 array with at least three rows.", obstacleIndex);
    end
    name = "obstacle " + obstacleIndex;
    if isfield(obstacle, "Name") && ~isempty(obstacle.Name)
        name = string(obstacle.Name);
    end
    if ~isscalar(name) || ismissing(name) || strlength(strtrim(name)) == 0
        error("prepareObstacles:InvalidName", "Obstacle %d Name must be nonempty scalar text.", obstacleIndex);
    end
    safetyMargin_units = 0;
    if isfield(obstacle, "SafetyMargin_units") && ~isempty(obstacle.SafetyMargin_units)
        safetyMargin_units = obstacle.SafetyMargin_units;
    end
    validateattributes(safetyMargin_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});

    originalShape = polyshape(vertices_units(:, 1), vertices_units(:, 2), "Simplify", true, "KeepCollinearPoints", true);
    if originalShape.NumRegions ~= 1 || originalShape.NumHoles ~= 0 || area(originalShape) <= 0
        error("prepareObstacles:InvalidPolygon", "Obstacle %d must define one nonzero-area polygon without holes.", obstacleIndex);
    end
    originalVertices_units = finiteVertices(originalShape);
    requireConvex(originalVertices_units, originalShape, obstacleIndex);
    protectedShape = originalShape;
    if safetyMargin_units > 0
        protectedShape = polybuffer(originalShape, double(safetyMargin_units), "JointType", "square");
    end
    protectedVertices_units = finiteVertices(protectedShape);
    requireConvex(protectedVertices_units, protectedShape, obstacleIndex);

    preparedObstacles(obstacleIndex).Name                       = name;
    preparedObstacles(obstacleIndex).OriginalVertices_units     = originalVertices_units;
    preparedObstacles(obstacleIndex).SafetyMargin_units         = double(safetyMargin_units);
    preparedObstacles(obstacleIndex).ProtectedVertices_units    = protectedVertices_units;
    preparedObstacles(obstacleIndex).ProtectedShape             = protectedShape;
end
end

%% Section 3: Local Functions

function vertices_units = finiteVertices(shape)
    % Return one open finite ring from a validated single-region shape.
    vertices_units = double(shape.Vertices);
    vertices_units = vertices_units(all(isfinite(vertices_units), 2), :);
    if size(vertices_units, 1) > 1 && isequal(vertices_units(1, :), vertices_units(end, :))
        vertices_units(end, :) = [];
    end
end

function requireConvex(vertices_units, shape, obstacleIndex)
    % BMTP accepts convex exclusion regions, so reject concave input here.
    hullIndex = convhull(vertices_units(:, 1), vertices_units(:, 2));
    hullShape = polyshape(vertices_units(hullIndex(1:end - 1), 1), vertices_units(hullIndex(1:end - 1), 2), "Simplify", false);
    areaScale_units2 = max(1, area(hullShape));
    if abs(area(hullShape) - area(shape)) > 1024 * eps(areaScale_units2)
        error("prepareObstacles:NonconvexObstacle", "Obstacle %d is concave. The empty core accepts convex polygons only.", obstacleIndex);
    end
end
