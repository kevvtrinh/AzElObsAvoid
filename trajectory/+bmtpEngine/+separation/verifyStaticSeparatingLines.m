function planes = verifyStaticSeparatingLines(planes, controlPoint_units, regions_units, roundoffReserve_units, target_units)
%% Section 0: Header & Readme
% SYNTAX
%   planes = bmtpEngine.separation.verifyStaticSeparatingLines(planes, ...
%       controlPoint_units, regions_units, roundoffReserve_units, target_units)
%**************************************************************************
% PURPOSE
%   - Batch the scalar Bernstein plane bounds for static cells, then apply
%     the same bmtpEngine.separation.proveSeparation decision as the scalar verifier.
%**************************************************************************
% INPUTS
%   - planes (R-element struct array)
%       One separating plane per convex region.
%   - controlPoint_units (N-by-2 numeric array)
%       Common Bezier control points for one motion span.
%   - regions_units (R-by-1 cell array)
%       Static convex exclusion-region vertices.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%   - target_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
%**************************************************************************
% OUTPUTS
%   - planes (R-element struct array)
%       Planes with corrected offsets, signed gaps, and verification states.
%**************************************************************************
% UNITS
%   - Position, offsets, target, reserve, and gap are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Evaluate All Obstacle And Curve Product Coefficients

regionCount = numel(regions_units);
assert(numel(planes) == regionCount, ...
    'bmtpEngine:InvalidPlaneBatch', ...
    'Every static source region requires a plane.');
if regionCount == 0
    return
end

normalPages  = reshape([planes.Normal], 2, 2, regionCount);
firstNormals = reshape(normalPages(1, :, :), 2, regionCount).';
lastNormals  = reshape(normalPages(2, :, :), 2, regionCount).';
offsets_units = reshape([planes.Offset_units], 2, regionCount).';
vertices_units = vertcat(regions_units{:});
regionIndexByVertex = repelem((1:regionCount).', ...
    cellfun(@(region) size(region, 1), regions_units));
regionIndexByVertex = regionIndexByVertex(:);

firstSide_units = sum(vertices_units .* firstNormals(regionIndexByVertex, :), 2);
lastSide_units  = sum(vertices_units .* lastNormals(regionIndexByVertex, :), 2);
obstacleSide_units = min([firstSide_units + offsets_units(regionIndexByVertex, 1), ...
    (lastSide_units + firstSide_units + sum(offsets_units(regionIndexByVertex, :), 2)) / 2, ...
    lastSide_units + offsets_units(regionIndexByVertex, 2)], [], 2);
minimumObstacle_units = accumarray(regionIndexByVertex, obstacleSide_units, ...
    [regionCount, 1], @min);

degree = size(controlPoint_units, 1) - 1;
beta   = (0:degree + 1).' / (degree + 1);
alpha  = 1 - beta;
firstProjection_units = controlPoint_units(:, 1) * firstNormals(:, 1).' + ...
    controlPoint_units(:, 2) * firstNormals(:, 2).';
lastProjection_units = controlPoint_units(:, 1) * lastNormals(:, 1).' + ...
    controlPoint_units(:, 2) * lastNormals(:, 2).';
product_units = alpha .* [firstProjection_units; zeros(1, regionCount)] + ...
    beta .* [zeros(1, regionCount); lastProjection_units] + ...
    alpha * offsets_units(:, 1).' + beta * offsets_units(:, 2).';
maximumTrajectory_units = max(product_units, [], 1).';
maximumNormalNorm = max( ...
    [vecnorm(firstNormals, 2, 2), vecnorm(lastNormals, 2, 2)], [], 2);

%% Section 2: Apply The Shared Correction And Acceptance Decision

% Gather the same maximum absolute coordinate that the scalar tolerance
% calculation measures for one pair, here for every region in one pass.
scale_units = accumarray(regionIndexByVertex, max(abs(vertices_units), [], 2), ...
    [regionCount, 1], @max);
scale_units = max([scale_units, max(abs(offsets_units), [], 2), ...
    repmat(max(1, max(abs(controlPoint_units), [], 'all')), regionCount, 1)], [], 2);
[offsets_units, signedGap_units, verified] = bmtpEngine.separation.proveSeparation( ...
    minimumObstacle_units, maximumTrajectory_units, maximumNormalNorm, ...
    offsets_units, 16 * eps(scale_units), roundoffReserve_units, target_units);

offsetCells = num2cell(offsets_units, 2);
gapCells    = num2cell(signedGap_units);
flags       = num2cell(verified);
[planes.Offset_units]    = offsetCells{:};
[planes.SignedGap_units] = gapCells{:};
[planes.Verified]        = flags{:};
end
