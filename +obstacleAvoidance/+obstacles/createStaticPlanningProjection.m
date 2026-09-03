function [planningObstacles, projection] = ...
        createStaticPlanningProjection(obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [planningObstacles, projection] = ...
%       obstacleAvoidance.obstacles.createStaticPlanningProjection( ...
%       obstacles, startTime_s, endTime_s)
%**************************************************************************
% PURPOSE
%   - Create a conservative static planning projection for complete obstacle
%     histories without changing the authoritative validation geometry.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared obstacle struct array)
%       Protected histories are used exactly once; moving histories are
%       enclosed by a convex hull of every protected sample vertex.
%   - startTime_s, endTime_s (finite numeric scalars)
%       Inclusive request horizon with endTime_s not before startTime_s.
%**************************************************************************
% OUTPUTS
%   - planningObstacles (canonical obstacle struct array)
%       Static exact obstacles and conservative moving-history surrogates.
%   - projection (scalar struct)
%       Source mapping, construction method, and planning boundaries.
%**************************************************************************
% UNITS
%   - Position and boundary coordinates are degrees; time is seconds.
%**************************************************************************

%% Section 1: Validate And Normalize The Projection Request

validateattributes(startTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(endTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>=', startTime_s});
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
recordTemplate = struct( ...
    "SourceObstacleIndex", 0, "SourceName", "", ...
    "IsExactStaticGeometry", false, "Method", "", ...
    "Boundary_deg", zeros(0, 2), ...
    "HistoryBounds_deg", [NaN NaN NaN NaN]);
records = repmat(recordTemplate, numel(obstacles), 1);
projected = cell(numel(obstacles), 1);

%% Section 2: Create Exact Static Or Conservative Swept Geometry

for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    isStatic = obstacleAvoidance.obstacles.queryStaticHorizon( ...
        obstacle, startTime_s, endTime_s);
    records(obstacleIndex).SourceObstacleIndex = obstacleIndex;
    records(obstacleIndex).SourceName = string(obstacle.targetName);
    records(obstacleIndex).HistoryBounds_deg = ...
        obstacle.InternalPreparation.HistoryBounds_deg;
    if isStatic
        projected{obstacleIndex} = ...
            obstacleAvoidance.obstacles.createObstacle(obstacle);
        records(obstacleIndex).IsExactStaticGeometry = true;
        records(obstacleIndex).Method = "exactStaticHistory";
        records(obstacleIndex).Boundary_deg = [ ...
            obstacle.az_deg{1}, obstacle.el_deg{1}];
        continue;
    end

    historyVertices_deg = zeros(0, 2);
    for sampleIndex = 1:numel(obstacle.time_s)
        vertices_deg = [ ...
            obstacle.az_deg{sampleIndex}, obstacle.el_deg{sampleIndex}];
        historyVertices_deg = [historyVertices_deg; ...
            vertices_deg(all(isfinite(vertices_deg), 2), :)]; %#ok<AGROW>
    end
    historyVertices_deg = unique(historyVertices_deg, "rows", "stable");
    if size(historyVertices_deg, 1) < 3
        error("createStaticPlanningProjection:InsufficientGeometry", ...
            "Obstacle %d must retain at least three finite history vertices.", ...
            obstacleIndex);
    end
    hullIndex = convhull(historyVertices_deg(:, 1), ...
        historyVertices_deg(:, 2));
    boundary_deg = historyVertices_deg(hullIndex(1:end - 1), :);
    projectionTime_s = [startTime_s; endTime_s];
    if startTime_s == endTime_s
        projectionTime_s = startTime_s;
    end
    projected{obstacleIndex} = ...
        obstacleAvoidance.obstacles.createObstacle( ...
        string(obstacle.targetName) + " planning projection", ...
        projectionTime_s, boundary_deg(:, 1), boundary_deg(:, 2), 0);
    records(obstacleIndex).Method = ...
        "conservativeProtectedHistoryConvexHull";
    records(obstacleIndex).Boundary_deg = boundary_deg;
end
planningObstacles = ...
    obstacleAvoidance.obstacles.combineObstacles(projected{:});

%% Section 3: Assemble Projection Provenance

projection = struct( ...
    "Method", "staticProtectedHistoryProjection", ...
    "StartTime_s", startTime_s, "EndTime_s", endTime_s, ...
    "IsTimeDependent", false, ...
    "ContinuousContainmentBasis", ...
    "convexHullOfAllProtectedHistoryVertices", ...
    "Records", records);
end
