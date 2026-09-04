function [certified, minimumClearance_deg] = certifySeedCorridor( ...
        trajectory, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [certified, minimumClearance_deg] = ...
%       obstacleAvoidance.validation.certifySeedCorridor( ...
%       trajectory, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Independently verify complete obstacle-envelope containment, support
%     integrity, and continuous Bernstein separation for a seed corridor.
%**************************************************************************
% INPUTS
%   - trajectory (scalar struct)
%       Polynomial, SeedCorridorBoundary_deg, and SeedCorridor are required.
%   - obstacles (canonical protected obstacle struct array)
%       Complete histories that the supplied envelope must contain.
%   - tolerance_deg (nonnegative numeric scalar)
%       Certificate comparison tolerance.
%**************************************************************************
% OUTPUTS
%   - certified (logical scalar)
%       True only when every segment/region record passes.
%   - minimumClearance_deg (numeric scalar)
%       Smallest continuous certified clearance, or NaN on failure.
%**************************************************************************
% UNITS
%   - Geometry, clearance, and tolerance are degrees.
%**************************************************************************

%% Section 1: Validate Complete Certificate Evidence

validateattributes(tolerance_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
certified = false;
minimumClearance_deg = NaN;
requiredFields = {'Polynomial', 'SeedCorridorBoundary_deg', 'SeedCorridor'};
if ~isstruct(trajectory) || ~isscalar(trajectory) || ...
        ~all(isfield(trajectory, requiredFields)) || ...
        isempty(trajectory.SeedCorridorBoundary_deg) || isempty(trajectory.SeedCorridor)
    return;
end
boundary_deg = double(trajectory.SeedCorridorBoundary_deg);
if size(boundary_deg, 2) ~= 2 || ...
        any(xor(isfinite(boundary_deg(:, 1)), isfinite(boundary_deg(:, 2))))
    return;
end
shape = polyshape(boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
regions = obstacleAvoidance.geometry.convexPolygonRegions(shape);
corridor = trajectory.SeedCorridor;
segmentCount = trajectory.Polynomial.SegmentCount;
regionCount = numel(regions);
if segmentCount < 1 || regionCount < 1 || ...
        numel(corridor) ~= segmentCount * regionCount || ...
        ~seedEnvelopeContainsObstacles( ...
        boundary_deg, obstacles, tolerance_deg)
    return;
end
pairIndex = [[corridor.SegmentIndex].', [corridor.RegionIndex].'];
expected = [repelem((1:segmentCount).', regionCount), ...
    repmat((1:regionCount).', segmentCount, 1)];
if ~isequal(sortrows(pairIndex), expected)
    return;
end

%% Section 2: Verify Supports And Continuous Separation

supportTolerance_deg = max(1e-9, 10 * tolerance_deg);
for corridorIndex = 1:numel(corridor)
    record = corridor(corridorIndex);
    if abs(norm(record.Normal) - 1) > 1e-9 || record.Clearance_deg < 0
        return;
    end
    vertices_deg = regions(record.RegionIndex).Vertices;
    vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
    verifiedOffset_deg = max(vertices_deg * record.Normal.');
    if abs(verifiedOffset_deg - record.BoundaryOffset_deg) > supportTolerance_deg
        return;
    end
end
coefficientCount = size(trajectory.Polynomial.positionPower_deg, 3);
corridorCount = numel(corridor);
segmentIndex = [corridor.SegmentIndex].';
normal = vertcat(corridor.Normal);
selectedPower_deg = ...
    trajectory.Polynomial.positionPower_deg(segmentIndex, :, :);
azimuthPower_deg = reshape( ...
    selectedPower_deg(:, 1, :), corridorCount, coefficientCount);
elevationPower_deg = reshape( ...
    selectedPower_deg(:, 2, :), corridorCount, coefficientCount);
projectionPower_deg = normal(:, 1) .* azimuthPower_deg + ...
    normal(:, 2) .* elevationPower_deg;
projectionBernstein_deg = ...
    convertPowerToBernstein(projectionPower_deg.');
offset_deg = [corridor.BoundaryOffset_deg] + ...
    [corridor.Clearance_deg];
inequalityMatrix_deg = offset_deg - projectionBernstein_deg;
inequality_deg = inequalityMatrix_deg(:);
if isempty(inequality_deg) || any(~isfinite(inequality_deg)) || ...
        any(inequality_deg > tolerance_deg)
    return;
end
clearance_deg = repelem([corridor.Clearance_deg].', coefficientCount) - inequality_deg;
minimumClearance_deg = min(clearance_deg);
certified = true;
end

%% Section 3: Local Functions

function coefficient = convertPowerToBernstein(powerCoefficient)
% Independently convert ascending powers on [0, 1] for certification.
degree = size(powerCoefficient, 1) - 1;
persistent conversionMatrixByDegree
if numel(conversionMatrixByDegree) <= degree || ...
        isempty(conversionMatrixByDegree{degree + 1})
    conversionMatrix = pascal(degree + 1, 1);
    conversionMatrix = conversionMatrix ./ conversionMatrix(end, :);
    conversionMatrixByDegree{degree + 1} = conversionMatrix;
end
coefficient = conversionMatrixByDegree{degree + 1} * ...
    double(powerCoefficient);
end

function containsAllObstacles = seedEnvelopeContainsObstacles( ...
        boundary_deg, obstacles, tolerance_deg)
% Verify one envelope region contains every complete obstacle history.
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
shape = polyshape( ...
    boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
envelopeRegions = regions(shape);
if isempty(envelopeRegions)
    return;
end
for regionIndex = 1:numel(envelopeRegions)
    envelopeRegions(regionIndex) = polybuffer( ...
        envelopeRegions(regionIndex), max(1e-9, tolerance_deg));
end
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacleAvoidance.obstacles.prepareDynamic( ...
        obstacles(obstacleIndex));
    preparation = obstacle.InternalPreparation;
    if preparation.IsTimeInvariant
        sweptShape = preparation.StaticShape;
    else
        vertices_deg = zeros(0, 2);
        for sampleIndex = 1:numel(obstacle.az_deg)
            sample_deg = [obstacle.az_deg{sampleIndex}(:), ...
                obstacle.el_deg{sampleIndex}(:)];
            vertices_deg = [vertices_deg; ...
                sample_deg(all(isfinite(sample_deg), 2), :)]; %#ok<AGROW>
        end
        vertices_deg = unique(vertices_deg, "rows", "stable");
        if size(vertices_deg, 1) < 3
            return;
        end
        hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
        sweptShape = polyshape( ...
            vertices_deg(hullIndex(1:end - 1), :), ...
            "Simplify", false, "KeepCollinearPoints", true);
    end
    areaTolerance_deg2 = 256 * eps(max(1, area(sweptShape)));
    isContained = false;
    for regionIndex = 1:numel(envelopeRegions)
        if area(subtract(sweptShape, envelopeRegions(regionIndex))) <= ...
                areaTolerance_deg2
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
