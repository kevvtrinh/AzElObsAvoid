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
if segmentCount < 1 || regionCount < 1 || numel(corridor) ~= segmentCount * regionCount || ...
        ~obstacleAvoidance.validation.seedEnvelopeContainsObstacles( ...
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
inequality_deg = obstacleAvoidance.search.seedCorridorInequality( ...
    trajectory.Polynomial, corridor);
if isempty(inequality_deg) || any(~isfinite(inequality_deg)) || ...
        any(inequality_deg > tolerance_deg)
    return;
end
coefficientCount = size(trajectory.Polynomial.positionPower_deg, 3);
clearance_deg = repelem([corridor.Clearance_deg].', coefficientCount) - inequality_deg;
minimumClearance_deg = min(clearance_deg);
certified = true;
end
