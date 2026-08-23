function containsAllObstacles = seedEnvelopeContainsObstacles( ...
        boundary_deg, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   containsAllObstacles = azElPlannerMethods.hs3.internal.validation.seedEnvelopeContainsObstacles( ...
%       boundary_deg, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Verify that convex seed-envelope regions contain obstacle histories.
%**************************************************************************
% INPUTS
%   - boundary_deg (N-by-2 numeric array)
%       Paired nonfinite rows can separate envelope regions.
%   - obstacles (canonical protected obstacle struct array)
%       One convex envelope region must contain each complete history.
%   - tolerance_deg (nonnegative finite scalar)
%       Outward numerical containment tolerance.
%**************************************************************************
% OUTPUTS
%   - containsAllObstacles (logical scalar)
%       True only when all regions are convex and all histories are covered.
%**************************************************************************
% UNITS
%   - Boundary coordinates and tolerance are degrees.
%**************************************************************************

%% Section 1: Validate Optional Envelope Geometry

validateattributes(tolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
containsAllObstacles = false;
if isempty(boundary_deg) || ~isnumeric(boundary_deg) || ...
        size(boundary_deg, 2) ~= 2
    return;
end
boundary_deg = double(boundary_deg);
if any(xor(isfinite(boundary_deg(:, 1)), ...
        isfinite(boundary_deg(:, 2))))
    return;
end
envelopeShape = polyshape( ...
    boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
envelopeRegions = regions(envelopeShape);
regionCount = numel(envelopeRegions);
if regionCount < 1
    return;
end

%% Section 2: Verify Convex Regions

containmentTolerance_deg = max(1e-9, tolerance_deg);
bufferedRegionVertices_deg = cell(regionCount, 1);

% Validate and buffer every envelope region independently before assigning any
% obstacle history to it.
for regionIndex = 1:regionCount
    regionVertices_deg = envelopeRegions(regionIndex).Vertices;
    regionVertices_deg = regionVertices_deg( ...
        all(isfinite(regionVertices_deg), 2), :);
    if size(regionVertices_deg, 1) < 3
        return;
    end
    hullIndex = convhull( ...
        regionVertices_deg(:, 1), regionVertices_deg(:, 2));
    hullShape = polyshape(regionVertices_deg(hullIndex, :));
    areaTolerance_deg2 = containmentTolerance_deg * ...
        max(1, perimeter(hullShape));
    if abs(area(hullShape) - area(envelopeRegions(regionIndex))) > ...
            areaTolerance_deg2
        return;
    end
    bufferedShape = polybuffer( ...
        envelopeRegions(regionIndex), containmentTolerance_deg);
    bufferedRegionVertices_deg{regionIndex} = bufferedShape.Vertices;
end

%% Section 3: Assign Each Complete History To One Region

% Keep each obstacle's complete history inside one region so disconnected
% envelopes cannot certify a trajectory by splitting one history across them.
for obstacleIndex = 1:numel(obstacles)
    containingRegionFound = false;
    obstacle = obstacles(obstacleIndex);
    position_deg = [vertcat(obstacle.az_deg{:}), ...
        vertcat(obstacle.el_deg{:})];
    position_deg = position_deg(all(isfinite(position_deg), 2), :);

    % Test the complete finite history against every validated envelope region.
    for regionIndex = 1:regionCount
        boundary = bufferedRegionVertices_deg{regionIndex};
        [isInside, isOnBoundary] = inpolygon( ...
            position_deg(:, 1), position_deg(:, 2), ...
            boundary(:, 1), boundary(:, 2));
        if ~isempty(position_deg) && all(isInside | isOnBoundary)
            containingRegionFound = true;
            break;
        end
    end
    if ~containingRegionFound
        return;
    end
end
containsAllObstacles = true;
end
