function containsAllObstacles = seedEnvelopeContainsObstacles(boundary_deg, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   containsAllObstacles = azElPlannerMethods.corridor.internal.validation.seedEnvelopeContainsObstacles( ...
%       boundary_deg, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Verify that a proposed static envelope contains every protected obstacle
%     slice. This prevents a reduced seed-only shape from becoming an unsafe
%     certificate source.
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

% Invalid optional certificate data returns false rather than throwing because
% the planner can report an ordinary candidate-validation failure.
validateattributes(tolerance_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
containsAllObstacles = false;
if isempty(boundary_deg) || ~isnumeric(boundary_deg) || size(boundary_deg, 2) ~= 2
    return;
end
boundary_deg = double(boundary_deg);
if any(xor(isfinite(boundary_deg(:, 1)), isfinite(boundary_deg(:, 2))))
    return;
end
envelopeShape = polyshape( boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
envelopeRegions = regions(envelopeShape);
regionCount = numel(envelopeRegions);
if regionCount < 1
    return;
end

%% Section 2: Buffer The Complete Envelope Union

% The small outward buffer absorbs polygon Boolean roundoff only; protected
% obstacle margins were already applied by public obstacle construction.
containmentTolerance_deg = max(1e-9, tolerance_deg);
bufferedEnvelopeShape = polybuffer( envelopeShape, containmentTolerance_deg);

%% Section 3: Verify Every Complete History Polygon

% Area subtraction checks full polygons rather than vertices alone. A concave
% edge can leave the envelope even when all sampled vertices appear contained.
% Check each obstacle independently so one covered source cannot mask another.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);

    % Verify every protected source-time slice, not merely the first or last.
    for sampleIndex = 1:numel(obstacle.time_s)
        obstacleShape = azElPlannerMethods.corridor.internal.obstacles.shapeAtTime( obstacle, obstacle.time_s(sampleIndex));
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
