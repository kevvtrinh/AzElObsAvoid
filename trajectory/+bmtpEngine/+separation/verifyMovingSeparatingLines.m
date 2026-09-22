function separatingPlanes = verifyMovingSeparatingLines(separatingPlanes, controlPoint_units, ...
    startRegions_units, endRegions_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   separatingPlanes = bmtpEngine.separation.verifyMovingSeparatingLines( ...
%       separatingPlanes, controlPoint_units, startRegions_units, endRegions_units, ...
%       roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Check one curve against several moving convex regions together. Use
%     the same whole-interval bounds and acceptance rules as the one-line check.
%**************************************************************************
% INPUTS
%   - separatingPlanes (R-element struct array)
%       One separating line per convex region whose vertices move linearly.
%   - controlPoint_units (N-by-2 numeric array)
%       Common Bezier control points over the checked physical interval.
%   - startRegions_units, endRegions_units (R-by-1 cell arrays)
%       Corresponding convex-region vertices at the interval endpoints.
%   - roundoffReserve_units, separationTarget_units (nonnegative numeric scalars)
%       Required trajectory roundoff reserve and obstacle-side target.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (R-element struct array)
%       Updated offsets and gap bounds. Verified is true only where every
%       separation condition passes for the entire interval.
%**************************************************************************
% UNITS
%   - Position, offsets, target, reserve, and gap are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Check Matching Regions And Group Their Vertices

regionCount = numel(startRegions_units);
assert(numel(separatingPlanes) == regionCount && numel(endRegions_units) == regionCount, ...
    'bmtpEngine:InvalidMovingPlaneBatch', ...
    'Every moving source region requires one plane and two endpoint regions.');
if regionCount == 0
    return
end

startVertexCounts = cellfun(@(region) size(region, 1), startRegions_units);
endVertexCounts   = cellfun(@(region) size(region, 1), endRegions_units);
regionsAreValid   = all(startVertexCounts >= 3) && isequal(startVertexCounts, endVertexCounts) && ...
    all(cellfun(@(region) isnumeric(region) && size(region, 2) == 2 && ...
    all(isfinite(region), 'all'), startRegions_units)) && ...
    all(cellfun(@(region) isnumeric(region) && size(region, 2) == 2 && ...
    all(isfinite(region), 'all'), endRegions_units));
assert(regionsAreValid, ...
    'bmtpEngine:InvalidMovingPlaneBatch', ...
    'Moving-region endpoint vertices must be finite corresponding convex polygons.');
% Stack the regions for one calculation. Keep the region index of each
% vertex so every projection uses its own line normal and offset.
normalByEndpointAndRegion = reshape([separatingPlanes.Normal], 2, 2, regionCount);
startNormals             = reshape(normalByEndpointAndRegion(1, :, :), 2, regionCount).';
endNormals               = reshape(normalByEndpointAndRegion(2, :, :), 2, regionCount).';

lineOffset_units    = reshape([separatingPlanes.Offset_units], 2, regionCount).';
startVertices_units = vertcat(startRegions_units{:});
endVertices_units   = vertcat(endRegions_units{:});
regionIndexByVertex = repelem((1:regionCount).', startVertexCounts);
regionIndexByVertex = regionIndexByVertex(:);

%% Section 2: Bound Both Sides Of Each Line Throughout The Interval

% A linearly moving vertex x a linearly changing normal gives a quadratic.
% Its three Bernstein coefficients bound its value for the whole interval,
% including between the endpoints. Take the lowest bound over each region.
startSideWithoutOffset_units = sum(startVertices_units .* startNormals(regionIndexByVertex, :), 2);
endSideWithoutOffset_units   = sum(endVertices_units .* endNormals(regionIndexByVertex, :), 2);
crossSideWithoutOffset_units = sum(startVertices_units .* endNormals(regionIndexByVertex, :), 2) + ...
    sum(endVertices_units .* startNormals(regionIndexByVertex, :), 2);
minimumVertexSide_units = min([startSideWithoutOffset_units + lineOffset_units(regionIndexByVertex, 1), ...
    (crossSideWithoutOffset_units + sum(lineOffset_units(regionIndexByVertex, :), 2)) / 2, ...
    endSideWithoutOffset_units + lineOffset_units(regionIndexByVertex, 2)], [], 2);
minimumObstacleSide_units = accumarray(regionIndexByVertex, minimumVertexSide_units, ...
    [regionCount, 1], @min);

% For the degree-D curve, the corresponding product has degree D + 1.
% Its largest coefficient bounds the curve-side value without time sampling.
degree             = size(controlPoint_units, 1) - 1;
endNormalWeights   = (0:degree + 1).' / (degree + 1);
startNormalWeights = 1 - endNormalWeights;
startNormalProjection_units = controlPoint_units(:, 1) * startNormals(:, 1).' + ...
    controlPoint_units(:, 2) * startNormals(:, 2).';
endNormalProjection_units = controlPoint_units(:, 1) * endNormals(:, 1).' + ...
    controlPoint_units(:, 2) * endNormals(:, 2).';
curveSideCoefficients_units = startNormalWeights .* [startNormalProjection_units; zeros(1, regionCount)] + ...
    endNormalWeights .* [zeros(1, regionCount); endNormalProjection_units] + ...
    startNormalWeights * lineOffset_units(:, 1).' + endNormalWeights * lineOffset_units(:, 2).';
maximumTrajectorySide_units = max(curveSideCoefficients_units, [], 1).';
maximumNormalLength = max( ...
    [vecnorm(startNormals, 2, 2), vecnorm(endNormals, 2, 2)], [], 2);

%% Section 3: Shift The Lines When Possible And Apply All Gap Checks

% Size the rounding allowance from each region, its offsets and the curve.
% The shared check may shift both offsets equally, but all required curve
% and obstacle gaps must still pass after that shift.

vertexScale_units = max([max(abs(startVertices_units), [], 2), ...
    max(abs(endVertices_units), [], 2)], [], 2);
geometryScale_units = accumarray(regionIndexByVertex, vertexScale_units, ...
    [regionCount, 1], @max);
geometryScale_units = max([geometryScale_units, max(abs(lineOffset_units), [], 2), ...
    repmat(max(1, max(abs(controlPoint_units), [], 'all')), regionCount, 1)], [], 2);
[lineOffset_units, signedGap_units, separationIsVerified] = bmtpEngine.separation.proveSeparation( ...
    minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalLength, ...
    lineOffset_units, 16 * eps(geometryScale_units), roundoffReserve_units, separationTarget_units);

% Put each batch result back into its original line record.
offsetCells       = num2cell(lineOffset_units, 2);
gapCells          = num2cell(signedGap_units);
verificationCells = num2cell(separationIsVerified);
[separatingPlanes.Offset_units]    = offsetCells{:};
[separatingPlanes.SignedGap_units] = gapCells{:};
[separatingPlanes.Verified]        = verificationCells{:};
end
