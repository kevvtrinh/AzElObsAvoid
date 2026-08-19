function containsAllObstacles = seedEnvelopeContainsObstacles( ...
        boundary_deg, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   containsAllObstacles = ...
%       azElInternal.seedEnvelopeContainsObstacles( ...
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
bufferedRegions = cell(regionCount, 1);
for regionIndex = 1:regionCount
    regionVertices_deg = envelopeRegions(regionIndex).Vertices;
    regionVertices_deg = ...
        regionVertices_deg(all(isfinite(regionVertices_deg), 2), :);
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
    bufferedRegions{regionIndex} = polybuffer( ...
        envelopeRegions(regionIndex), containmentTolerance_deg);
end

%% Section 3: Assign Each Complete History To One Region

for obstacleIndex = 1:numel(obstacles)
    containingRegionFound = false;
    for regionIndex = 1:regionCount
        allSlicesInside = true;
        for sampleIndex = 1:numel(obstacles(obstacleIndex).az_deg)
            position_deg = [ ...
                obstacles(obstacleIndex).az_deg{sampleIndex}(:), ...
                obstacles(obstacleIndex).el_deg{sampleIndex}(:)];
            position_deg = ...
                position_deg(all(isfinite(position_deg), 2), :);
            if isempty(position_deg) || ~all(isinterior( ...
                    bufferedRegions{regionIndex}, ...
                    position_deg(:, 1), position_deg(:, 2)))
                allSlicesInside = false;
                break;
            end
        end
        if allSlicesInside
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
