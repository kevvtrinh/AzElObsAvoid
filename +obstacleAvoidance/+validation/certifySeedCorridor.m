function [certified, minimumClearance_units] = certifySeedCorridor(trajectory, obstacles, tolerance_units)
%% Section 0: Header & Readme
% SYNTAX
%   [certified, minimumClearance_units] = ...
%       obstacleAvoidance.validation.certifySeedCorridor( ...
%       trajectory, obstacles, tolerance_units)
%
% PURPOSE
%   - Independently verify complete obstacle-envelope containment, support
%     integrity, and continuous Bernstein separation for a seed corridor.
%
% INPUTS
%   - trajectory (scalar struct)
%       Polynomial, SeedCorridorBoundary_units, and SeedCorridor are required.
%   - obstacles (prepared protected obstacle struct array)
%       Complete histories that the supplied envelope must contain.
%   - tolerance_units (nonnegative numeric scalar)
%       Certificate comparison tolerance.
%
% OUTPUTS
%   - certified (logical scalar)
%       True only when every segment/region record passes.
%   - minimumClearance_units (numeric scalar)
%       Smallest continuous certified clearance, or NaN on failure.
%
% UNITS
%   - Geometry, clearance, and tolerance are coordinate units.
%

%% Section 1: Validate Complete Certificate Evidence

validateattributes(tolerance_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
certified            = false;
minimumClearance_units = NaN;
requiredFields       = {'Polynomial', 'SeedCorridorBoundary_units', 'SeedCorridor'};
if ~isstruct(trajectory) || ~isscalar(trajectory) || ~all(isfield(trajectory, requiredFields)) || isempty(trajectory.SeedCorridorBoundary_units) || isempty(trajectory.SeedCorridor)
    return;
end
boundary_units = double(trajectory.SeedCorridorBoundary_units);
if size(boundary_units, 2) ~= 2 || any(xor(isfinite(boundary_units(:, 1)), isfinite(boundary_units(:, 2))))
    return;
end
shape        = polyshape(boundary_units(:, 1), boundary_units(:, 2), "Simplify", true);
regions      = obstacleAvoidance.geometry.convexPolygonRegions(shape);
corridor     = trajectory.SeedCorridor;
segmentCount = trajectory.Polynomial.SegmentCount;
regionCount  = numel(regions);
if segmentCount < 1 || regionCount < 1 || numel(corridor) ~= segmentCount * regionCount || ~seedEnvelopeContainsObstacles(boundary_units, obstacles, tolerance_units)
    return;
end
pairIndex = [[corridor.SegmentIndex].', [corridor.RegionIndex].'];
expected  = [reshape(repelem(1:segmentCount, regionCount), [], 1), ...
    repmat((1:regionCount).', segmentCount, 1)];
if ~isequal(sortrows(pairIndex), expected)
    return;
end

%% Section 2: Verify Supports And Continuous Separation

supportTolerance_units = max(1e-9, 10 * tolerance_units);
% Process each corridor needed to verify seed corridor.
for corridorIndex = 1:numel(corridor)
    record = corridor(corridorIndex);
    if abs(norm(record.Normal) - 1) > 1e-9 || record.Clearance_units < 0
        return;
    end
    vertices_units       = regions(record.RegionIndex).Vertices;
    vertices_units       = vertices_units(all(isfinite(vertices_units), 2), :);
    verifiedOffset_units = max(vertices_units * record.Normal.');
    if abs(verifiedOffset_units - record.BoundaryOffset_units) > supportTolerance_units
        return;
    end
end
coefficientCount        = size(trajectory.Polynomial.positionPower_units, 3);
corridorCount           = numel(corridor);
segmentIndex            = [corridor.SegmentIndex].';
normal                  = vertcat(corridor.Normal);
selectedPower_units       = trajectory.Polynomial.positionPower_units(segmentIndex, :, :);
xPower_units        = reshape(selectedPower_units(:, 1, :), corridorCount, coefficientCount);
yPower_units      = reshape(selectedPower_units(:, 2, :), corridorCount, coefficientCount);
projectionPower_units     = normal(:, 1) .* xPower_units + normal(:, 2) .* yPower_units;
projectionBernstein_units = convertPowerToBernstein(projectionPower_units.');
offset_units              = [corridor.BoundaryOffset_units] + ...
    [corridor.Clearance_units];
inequalityMatrix_units = offset_units - projectionBernstein_units;
inequality_units       = inequalityMatrix_units(:);
if isempty(inequality_units) || any(~isfinite(inequality_units)) || any(inequality_units > tolerance_units)
    return;
end
requiredClearance_units = reshape(repelem([corridor.Clearance_units], coefficientCount), [], 1);
clearance_units         = requiredClearance_units - inequality_units;
minimumClearance_units  = min(clearance_units, [], "all");
certified             = true;
end

%% Section 3: Local Functions

function coefficient = convertPowerToBernstein(powerCoefficient)
    % Independently convert ascending powers on [0, 1] for certification.
    degree = size(powerCoefficient, 1) - 1;
    persistent conversionMatrixByDegree
    if numel(conversionMatrixByDegree) <= degree || isempty(conversionMatrixByDegree{degree + 1})
        conversionMatrix = pascal(degree + 1, 1);
        conversionMatrix = conversionMatrix ./ conversionMatrix(end, :);
        conversionMatrixByDegree{degree + 1} = conversionMatrix;
    end
    coefficient = conversionMatrixByDegree{degree + 1} * double(powerCoefficient);
end

function containsAllObstacles = seedEnvelopeContainsObstacles(boundary_units, obstacles, tolerance_units)
    % Verify one envelope region contains every complete obstacle history.
    validateattributes(tolerance_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
    containsAllObstacles = false;
    if isempty(boundary_units) || ~isnumeric(boundary_units) || size(boundary_units, 2) ~= 2
        return;
    end
    boundary_units = double(boundary_units);
    if any(xor(isfinite(boundary_units(:, 1)), isfinite(boundary_units(:, 2))))
        return;
    end
    shape           = polyshape(boundary_units(:, 1), boundary_units(:, 2), "Simplify", true);
    envelopeRegions = regions(shape);
    if isempty(envelopeRegions)
        return;
    end
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:numel(envelopeRegions)
        envelopeRegions(regionIndex) = polybuffer(envelopeRegions(regionIndex), max(1e-9, tolerance_units));
    end
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        obstacle    = obstacles(obstacleIndex);
        preparation = obstacle.InternalPreparation;
        if preparation.IsTimeInvariant
            sweptShape = preparation.StaticShape;
        else
            vertices_units = zeros(0, 2);
            % Process each sample in temporal order and accumulate its result.
            for sampleIndex = 1:numel(obstacle.x_units)
                sample_units = [obstacle.x_units{sampleIndex}(:), ...
                    obstacle.y_units{sampleIndex}(:)];
                vertices_units = [vertices_units; ...
                    sample_units(all(isfinite(sample_units), 2), :)]; %#ok<AGROW>
            end
            vertices_units = unique(vertices_units, "rows", "stable");
            if size(vertices_units, 1) < 3
                return;
            end
            hullIndex  = convhull(vertices_units(:, 1), vertices_units(:, 2));
            sweptShape = polyshape(vertices_units(hullIndex(1:end - 1), :), "Simplify", false, "KeepCollinearPoints", true);
        end
        areaTolerance_units2 = 256 * eps(max(1, area(sweptShape)));
        isContained        = false;
        % Process each geometric region while constructing or checking the region topology.
        for regionIndex = 1:numel(envelopeRegions)
            if area(subtract(sweptShape, envelopeRegions(regionIndex))) <= areaTolerance_units2
                isContained = true;
                break;
            end
        end
        if ~isContained
            return;
        end
    end
    containsAllObstacles = true;
end
