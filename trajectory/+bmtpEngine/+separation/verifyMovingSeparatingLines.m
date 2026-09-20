function planes = verifyMovingSeparatingLines(planes, controlPoint_units, ...
        firstRegions_units, lastRegions_units, reserve_units, target_units)
%% Section 0: Header & Readme
% SYNTAX
%   planes = bmtpEngine.separation.verifyMovingSeparatingLines(planes, ...
%       controlPoint_units, firstRegions_units, lastRegions_units, ...
%       reserve_units, target_units)
%**************************************************************************
% PURPOSE
%   - Batch the scalar Bernstein plane bounds for affine moving cells.
%**************************************************************************
% INPUTS
%   - planes (R-element struct array)
%       One separating plane per convex moving region.
%   - controlPoint_units (N-by-2 numeric array)
%       Common Bezier control points over the checked physical interval.
%   - firstRegions_units, lastRegions_units (R-by-1 cell arrays)
%       Corresponding convex-region vertices at the interval endpoints.
%   - reserve_units, target_units (nonnegative numeric scalars)
%       Required trajectory reserve and obstacle-side target.
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

regionCount = numel(firstRegions_units);
assert(numel(planes) == regionCount && numel(lastRegions_units) == regionCount, ...
    'bmtpEngine:InvalidMovingPlaneBatch', ...
    'Every moving source region requires one plane and two endpoint regions.');
if regionCount == 0
    return
end

vertexCounts     = cellfun(@(region) size(region, 1), firstRegions_units);
lastVertexCounts = cellfun(@(region) size(region, 1), lastRegions_units);
regionsAreValid  = all(vertexCounts >= 3) && isequal(vertexCounts, lastVertexCounts) && ...
    all(cellfun(@(region) isnumeric(region) && size(region, 2) == 2 && ...
    all(isfinite(region), 'all'), firstRegions_units)) && ...
    all(cellfun(@(region) isnumeric(region) && size(region, 2) == 2 && ...
    all(isfinite(region), 'all'), lastRegions_units));
assert(regionsAreValid, ...
    'bmtpEngine:InvalidMovingPlaneBatch', ...
    'Moving-region endpoint vertices must be finite corresponding convex polygons.');
normalPages  = reshape([planes.Normal], 2, 2, regionCount);
firstNormals = reshape(normalPages(1, :, :), 2, regionCount).';
lastNormals  = reshape(normalPages(2, :, :), 2, regionCount).';
offsets_units = reshape([planes.Offset_units], 2, regionCount).';
firstVertices_units = vertcat(firstRegions_units{:});
lastVertices_units  = vertcat(lastRegions_units{:});
regionIndexByVertex = repelem((1:regionCount).', vertexCounts);
regionIndexByVertex = regionIndexByVertex(:);

firstSide_units = sum(firstVertices_units .* firstNormals(regionIndexByVertex, :), 2);
lastSide_units  = sum(lastVertices_units .* lastNormals(regionIndexByVertex, :), 2);
crossSide_units = sum(firstVertices_units .* lastNormals(regionIndexByVertex, :), 2) + ...
    sum(lastVertices_units .* firstNormals(regionIndexByVertex, :), 2);
obstacleSide_units = min([firstSide_units + offsets_units(regionIndexByVertex, 1), ...
    (crossSide_units + sum(offsets_units(regionIndexByVertex, :), 2)) / 2, ...
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

vertexScale_units = max([max(abs(firstVertices_units), [], 2), ...
    max(abs(lastVertices_units), [], 2)], [], 2);
scale_units = accumarray(regionIndexByVertex, vertexScale_units, ...
    [regionCount, 1], @max);
scale_units = max([scale_units, max(abs(offsets_units), [], 2), ...
    repmat(max(1, max(abs(controlPoint_units), [], 'all')), regionCount, 1)], [], 2);
[offsets_units, signedGap_units, verified] = bmtpEngine.separation.proveSeparation( ...
    minimumObstacle_units, maximumTrajectory_units, maximumNormalNorm, ...
    offsets_units, 16 * eps(scale_units), reserve_units, target_units);

offsetCells = num2cell(offsets_units, 2);
gapCells    = num2cell(signedGap_units);
flags       = num2cell(verified);
[planes.Offset_units]    = offsetCells{:};
[planes.SignedGap_units] = gapCells{:};
[planes.Verified]        = flags{:};
end
