function containsAllObstacles = seedEnvelopeContainsObstacles( ...
        boundary_deg, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   containsAllObstacles = ...
%       azElInternal.seedEnvelopeContainsObstacles( ...
%       boundary_deg, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Verify that the complete seed-envelope union contains obstacle histories.
%**************************************************************************
% INPUTS
%   - boundary_deg (N-by-2 numeric array)
%       Paired nonfinite rows can separate envelope regions.
%   - obstacles (canonical protected obstacle struct array)
%       Every history vertex must lie in the complete envelope union.
%   - tolerance_deg (nonnegative finite scalar)
%       Outward numerical containment tolerance.
%**************************************************************************
% OUTPUTS
%   - containsAllObstacles (logical scalar)
%       True only when the complete obstacle histories are covered.
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

%% Section 2: Buffer The Complete Envelope Union

containmentTolerance_deg = max(1e-9, tolerance_deg);
bufferedEnvelopeShape = polybuffer( ...
    envelopeShape, containmentTolerance_deg);

%% Section 3: Verify Every Complete History Polygon

for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    for sampleIndex = 1:numel(obstacle.time_s)
        obstacleShape = azElInternal.obstacleShapeAtTime( ...
            obstacle, obstacle.time_s(sampleIndex));
        if isempty(obstacleShape.Vertices)
            return;
        end
        uncoveredShape = subtract(obstacleShape, bufferedEnvelopeShape);
        areaTolerance_deg2 = 256 * eps(max(1, area(obstacleShape)));
        if area(uncoveredShape) > areaTolerance_deg2
            return;
        end
    end
end
containsAllObstacles = true;
end
