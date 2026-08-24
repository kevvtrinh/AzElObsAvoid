function expandedRoute_deg = expandRouteClearance( ...
        route_deg, obstacles, queryTime_s, clearanceTarget_deg, ...
        routeExpansionFraction)
%% Section 0: Header & Readme
% Move low-clearance interior route vertices outward from protected geometry.

expandedRoute_deg = route_deg;

% Endpoints are fixed by the request, so only interior vertices may move.
for routeIndex = 2:size(route_deg, 1) - 1
    point_deg = route_deg(routeIndex, :);
    nearestDistance_deg = Inf;
    nearestBoundaryPoint_deg = [NaN NaN];
    nearestSpan_deg = 0;

    % Find the protected obstacle that most restricts this route vertex.
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), queryTime_s);
        vertices_deg = shape.Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        [distance_deg, boundaryPoint_deg] = ...
            azElInternal.geometry.pointPolygonClearance(shape, point_deg);
        if distance_deg < nearestDistance_deg
            nearestDistance_deg = distance_deg;
            nearestBoundaryPoint_deg = boundaryPoint_deg;
            nearestSpan_deg = norm( ...
                max(vertices_deg, [], 1) - min(vertices_deg, [], 1));
        end
    end
    desiredClearance_deg = clearanceTarget_deg + ...
        routeExpansionFraction * nearestSpan_deg;
    outwardDirection = point_deg - nearestBoundaryPoint_deg;
    if nearestDistance_deg < desiredClearance_deg && ...
            norm(outwardDirection) > eps
        expandedRoute_deg(routeIndex, :) = point_deg + ...
            (desiredClearance_deg - nearestDistance_deg) * ...
            outwardDirection / norm(outwardDirection);
    end
end
end
