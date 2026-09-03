function containsAllObstacles = seedEnvelopeContainsObstacles( ...
        boundary_deg, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   containsAllObstacles = ...
%       obstacleAvoidance.validation.seedEnvelopeContainsObstacles( ...
%       boundary_deg, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Verify that one connected envelope region contains each obstacle's
%     complete conservative continuous history.
%**************************************************************************
% INPUTS
%   - boundary_deg (N-by-2 numeric array)
%       Paired nonfinite rows may separate envelope regions.
%   - obstacles (canonical protected obstacle struct array)
%       Source histories are rebuilt rather than trusting stale preparation.
%   - tolerance_deg (nonnegative numeric scalar)
%       Outward comparison tolerance.
%**************************************************************************
% OUTPUTS
%   - containsAllObstacles (logical scalar)
%       True only when every complete history is contained.
%**************************************************************************
% UNITS
%   - Boundary coordinates and tolerance are degrees.
%**************************************************************************

%% Section 1: Resolve Connected Envelope Regions

validateattributes(tolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
containsAllObstacles = false;
if isempty(boundary_deg) || ~isnumeric(boundary_deg) || size(boundary_deg, 2) ~= 2
    return;
end
boundary_deg = double(boundary_deg);
if any(xor(isfinite(boundary_deg(:, 1)), isfinite(boundary_deg(:, 2))))
    return;
end
shape = polyshape(boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
envelopeRegions = regions(shape);
if isempty(envelopeRegions)
    return;
end
for regionIndex = 1:numel(envelopeRegions)
    envelopeRegions(regionIndex) = polybuffer(envelopeRegions(regionIndex), ...
        max(1e-9, tolerance_deg));
end

%% Section 2: Verify Source-Derived Continuous Sweeps

for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacleAvoidance.obstacles.prepareDynamic(obstacles(obstacleIndex));
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
        sweptShape = polyshape(vertices_deg(hullIndex(1:end - 1), :), ...
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
