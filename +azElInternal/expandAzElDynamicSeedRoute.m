function [route_deg, maximumExpansion_deg] = expandAzElDynamicSeedRoute( ...
        seed, obstacles, initialTime_s, goalTime_s)
%% Section 0: Header & Readme
% PURPOSE
%   - Add time-local protected clearance to moving-seed interior vertices
%     without changing endpoints, route order, or explicit stationary spans.
%**************************************************************************

route_deg = seed.position_deg;
duration_s = min(goalTime_s - initialTime_s, seed.EstimatedDuration_s);
for routeIndex = 2:size(route_deg, 1) - 1
    previousSpanIsHold = ...
        norm(route_deg(routeIndex, :) - route_deg(routeIndex - 1, :)) <= 1e-12;
    nextSpanIsHold = ...
        norm(route_deg(routeIndex + 1, :) - route_deg(routeIndex, :)) <= 1e-12;
    if previousSpanIsHold || nextSpanIsHold
        continue;
    end
    queryTime_s = initialTime_s + seed.tau(routeIndex) * duration_s;
    point_deg = route_deg(routeIndex, :);
    [distance_deg, boundaryPoint_deg, obstacleSpan_deg] = ...
        nearestActiveBoundary(point_deg, obstacles, queryTime_s);
    desiredClearance_deg = 0.02 + 0.02 * obstacleSpan_deg;
    direction = point_deg - boundaryPoint_deg;
    if distance_deg < desiredClearance_deg && norm(direction) > eps
        route_deg(routeIndex, :) = point_deg + ...
            (desiredClearance_deg - distance_deg) * direction / norm(direction);
    end
end
maximumExpansion_deg = max( ...
    vecnorm(route_deg - seed.position_deg, 2, 2), [], "omitmissing");
end

function [bestDistance_deg, bestBoundaryPoint_deg, bestSpan_deg] = ...
        nearestActiveBoundary(point_deg, obstacles, queryTime_s)
bestDistance_deg = Inf;
bestBoundaryPoint_deg = [NaN NaN];
bestSpan_deg = 0;
for obstacleIndex = 1:numel(obstacles)
    shape = azElInternal.obstacleShapeAtTime( ...
        obstacles(obstacleIndex), queryTime_s);
    [azimuth_deg, elevation_deg] = boundary(shape);
    finiteRow = isfinite(azimuth_deg) & isfinite(elevation_deg);
    runStart = find(finiteRow & [true; ~finiteRow(1:end - 1)]);
    runEnd = find(finiteRow & [~finiteRow(2:end); true]);
    for runIndex = 1:numel(runStart)
        ring_deg = [ ...
            azimuth_deg(runStart(runIndex):runEnd(runIndex)), ...
            elevation_deg(runStart(runIndex):runEnd(runIndex))];
        ringShape = polyshape(ring_deg(:, 1), ring_deg(:, 2), ...
            "Simplify", false);
        [distance_deg, boundaryPoint_deg] = ...
            azElInternal.pointPolygonClearance(ringShape, point_deg);
        if distance_deg < bestDistance_deg
            bestDistance_deg = distance_deg;
            bestBoundaryPoint_deg = boundaryPoint_deg;
            bestSpan_deg = norm(max(ring_deg, [], 1) - min(ring_deg, [], 1));
        end
    end
end
end
