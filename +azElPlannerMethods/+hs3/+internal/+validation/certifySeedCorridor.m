function [certified, minimumClearance_deg] = certifySeedCorridor( ...
        trajectory, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [certified, minimumClearance_deg] = azElPlannerMethods.hs3.internal.validation.certifySeedCorridor( ...
%       trajectory, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Independently certify a complete polynomial outside seed envelopes.
%**************************************************************************
% INPUTS
%   - trajectory (scalar candidate or planner-result struct)
%       Polynomial, SeedCorridorBoundary_deg, and SeedCorridor are used.
%   - obstacles (canonical protected obstacle struct array)
%       Every protected history slice must lie in one convex envelope region.
%   - tolerance_deg (nonnegative finite scalar)
%       Numerical tolerance for containment, support, and polynomial checks.
%**************************************************************************
% OUTPUTS
%   - certified (logical scalar)
%       True only when geometry containment and continuous separation pass.
%   - minimumClearance_deg (numeric scalar)
%       Smallest certified supporting-half-space clearance, or NaN.
%**************************************************************************
% UNITS
%   - Geometry, tolerance, and clearance are degrees.
%**************************************************************************

%% Section 1: Validate The Optional Certificate Schema

validateattributes(tolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
certified = false;
minimumClearance_deg = NaN;
requiredFields = {'Polynomial', 'SeedCorridorBoundary_deg', ...
    'SeedCorridor'};
if ~isstruct(trajectory) || ~isscalar(trajectory) || ...
        ~all(isfield(trajectory, requiredFields)) || ...
        isempty(trajectory.SeedCorridorBoundary_deg) || ...
        isempty(trajectory.SeedCorridor)
    return;
end
boundary_deg = double(trajectory.SeedCorridorBoundary_deg);
if size(boundary_deg, 2) ~= 2 || ...
        any(xor(isfinite(boundary_deg(:, 1)), ...
        isfinite(boundary_deg(:, 2))))
    return;
end
envelopeShape = polyshape( ...
    boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
envelopeRegions = regions(envelopeShape);
segmentCount = trajectory.Polynomial.SegmentCount;
regionCount = numel(envelopeRegions);
corridor = trajectory.SeedCorridor;
if segmentCount < 1 || regionCount < 1 || ...
        numel(corridor) ~= segmentCount * regionCount
    return;
end

%% Section 2: Verify Convex Envelope Containment

if ~azElPlannerMethods.hs3.internal.validation.seedEnvelopeContainsObstacles( ...
        boundary_deg, obstacles, tolerance_deg)
    return;
end

%% Section 3: Verify Supports & Complete Polynomial Separation

pairIndex = [[corridor.SegmentIndex].', [corridor.RegionIndex].'];
expectedPairIndex = zeros(segmentCount * regionCount, 2);
writeIndex = 0;

% Enumerate every expected segment-region pair in the same deterministic order
% used by corridor construction.
for segmentIndex = 1:segmentCount
    % Pair this segment with every convex envelope region.
    for regionIndex = 1:regionCount
        writeIndex = writeIndex + 1;
        expectedPairIndex(writeIndex, :) = [segmentIndex regionIndex];
    end
end
if ~isequal(sortrows(pairIndex), expectedPairIndex)
    return;
end
supportTolerance_deg = max(1e-9, 10 * tolerance_deg);

% Recompute every recorded support from source geometry before trusting the
% polynomial separation certificate.
for corridorIndex = 1:numel(corridor)
    record = corridor(corridorIndex);
    if norm(record.Normal) < 1 - 1e-9 || ...
            norm(record.Normal) > 1 + 1e-9 || ...
            record.Clearance_deg < 0
        return;
    end
    regionVertices_deg = envelopeRegions(record.RegionIndex).Vertices;
    regionVertices_deg = regionVertices_deg( ...
        all(isfinite(regionVertices_deg), 2), :);
    verifiedOffset_deg = max(regionVertices_deg * record.Normal.');
    if abs(verifiedOffset_deg - record.BoundaryOffset_deg) > ...
            supportTolerance_deg
        return;
    end
end
inequality_deg = azElInternal.seedCorridorInequality( ...
    trajectory.Polynomial, corridor);
if isempty(inequality_deg) || any(~isfinite(inequality_deg)) || ...
        any(inequality_deg > tolerance_deg)
    return;
end
coefficientCount = size(trajectory.Polynomial.positionPower_deg, 3);
clearance_deg = zeros(size(inequality_deg));

% Convert each corridor inequality block back to certified clearance so the
% minimum report covers every Bernstein coefficient.
for corridorIndex = 1:numel(corridor)
    rows = (corridorIndex - 1) * coefficientCount + ...
        (1:coefficientCount);
    clearance_deg(rows) = corridor(corridorIndex).Clearance_deg - ...
        inequality_deg(rows);
end
minimumClearance_deg = min(clearance_deg);
certified = true;
end
