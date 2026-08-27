function [certified, minimumClearance_deg] = certifySeedCorridor(trajectory, obstacles, tolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [certified, minimumClearance_deg] = obstacleAvoidance.search.certifySeedCorridor( ...
%       trajectory, obstacles, tolerance_deg)
%**************************************************************************
% PURPOSE
%   - Independently verify a solver-supplied static corridor certificate.
%     Certification checks field completeness, obstacle-envelope containment,
%     support-plane integrity, and continuous polynomial separation in that
%     order; any missing or inconsistent evidence fails closed.
%**************************************************************************
% INPUTS
%   - trajectory (scalar struct), polynomial and corridor certificate data.
%   - obstacles (canonical struct array), protected obstacle histories.
%   - tolerance_deg (nonnegative scalar), certificate comparison tolerance.
%**************************************************************************
% OUTPUTS
%   - certified (logical scalar), true only when every certificate passes.
%   - minimumClearance_deg (scalar), smallest certified corridor clearance.
%**************************************************************************
% UNITS
%   - Geometry, clearance, and tolerance are degrees.
%**************************************************************************

%% Section 1: Validate The Optional Certificate Data

% Fail closed: missing fields, malformed boundaries, or incomplete
% segment/region coverage cannot establish continuous obstacle separation.

% A certificate is optional for candidate families that use exact dynamic
% validation. When present, however, every segment/region pair must be supplied.
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
if size(boundary_deg, 2) ~= 2 || any(xor(isfinite(boundary_deg(:, 1)), isfinite(boundary_deg(:, 2))))
    return;
end
envelopeShape = polyshape( boundary_deg(:, 1), boundary_deg(:, 2), "Simplify", true);
envelopeRegions = obstacleAvoidance.geometry.convexPolygonRegions(envelopeShape);
segmentCount = trajectory.Polynomial.SegmentCount;
regionCount = numel(envelopeRegions);
corridor = trajectory.SeedCorridor;
if segmentCount < 1 || regionCount < 1 || numel(corridor) ~= segmentCount * regionCount
    return;
end

%% Section 2: Verify Convex Envelope Containment

% A support line derived from the seed envelope is meaningful only when that
% envelope contains every protected obstacle position over the full history.

% Corridor supports are useful only if their source envelope truly contains
% every protected obstacle history used by the planner.
if ~obstacleAvoidance.search.seedEnvelopeContainsObstacles( boundary_deg, obstacles, tolerance_deg)
    return;
end

%% Section 3: Verify Supports & Complete Polynomial Separation

% Recreate support offsets from the supplied geometry, verify unit normals,
% and then use Bernstein coefficients to prove separation throughout each
% complete polynomial segment. No sampled-point success is treated as proof.

% Recompute each support offset from geometry rather than trusting solver data.
% Then use Bernstein bounds to cover every normalized time within each span.
pairIndex = [[corridor.SegmentIndex].', [corridor.RegionIndex].'];
expectedPairIndex = zeros(segmentCount * regionCount, 2);
writeIndex = 0;

% Enumerate every segment/region pair that a complete certificate must contain.
for segmentIndex = 1:segmentCount

    % Add all convex envelope regions for this polynomial segment.
    for regionIndex = 1:regionCount
        writeIndex = writeIndex + 1;
        expectedPairIndex(writeIndex, :) = [segmentIndex regionIndex];
    end
end
if ~isequal(sortrows(pairIndex), expectedPairIndex)
    return;
end
supportTolerance_deg = max(1e-9, 10 * tolerance_deg);
% The absolute floor covers polygon arithmetic near zero; the relative term
% keeps this check consistent with the caller's geometric tolerance.

% Recompute and verify every stored support plane against its source region.
for corridorIndex = 1:numel(corridor)
    record = corridor(corridorIndex);
    if norm(record.Normal) < 1 - 1e-9 || norm(record.Normal) > 1 + 1e-9 || record.Clearance_deg < 0
        return;
    end
    regionVertices_deg = envelopeRegions(record.RegionIndex).Vertices;
    regionVertices_deg = regionVertices_deg(all(isfinite(regionVertices_deg), 2), :);
    verifiedOffset_deg = max(regionVertices_deg * record.Normal.');
    if abs(verifiedOffset_deg - record.BoundaryOffset_deg) > supportTolerance_deg
        return;
    end
end
inequality_deg = obstacleAvoidance.search.seedCorridorInequality( ...
    trajectory.Polynomial, corridor);
if isempty(inequality_deg) || any(~isfinite(inequality_deg)) || any(inequality_deg > tolerance_deg)
    return;
end
coefficientCount = size(trajectory.Polynomial.positionPower_deg, 3);
clearance_deg = zeros(size(inequality_deg));

% Convert each corridor record's Bernstein inequalities into certified clearance values.
for corridorIndex = 1:numel(corridor)
    rows = (corridorIndex - 1) * coefficientCount + (1:coefficientCount);
    clearance_deg(rows) = corridor(corridorIndex).Clearance_deg - inequality_deg(rows);
end
minimumClearance_deg = min(clearance_deg);
certified = true;
end
