function separatingPlanes = verifyStaticSeparatingLines( ...
    separatingPlanes, controlPoint_units, regions_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   separatingPlanes = bmtpEngine.separation.verifyStaticSeparatingLines( ...
%       separatingPlanes, controlPoint_units, regions_units, ...
%       roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Check one curve against several static convex regions together. Use
%     the same whole-interval bounds and acceptance rules as the one-line check.
%**************************************************************************
% INPUTS
%   - separatingPlanes (R-element struct array)
%       One separating line per static convex obstacle region.
%   - controlPoint_units (N-by-2 numeric array)
%       Common Bezier control points for one trajectory segment.
%   - regions_units (R-by-1 cell array)
%       Vertices of each static convex obstacle region.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%   - separationTarget_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
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

%% Section 1: Group The Regions And Their Line Data

regionCount = numel(regions_units);
assert(numel(separatingPlanes) == regionCount, ...
    'bmtpEngine:InvalidPlaneBatch', ...
    'Every static source region requires a plane.');
if regionCount == 0
    return
end

% Stack the vertices for one calculation, retaining each vertex's region
% index so its projection uses the correct line normal and offset.
normalByEndpointAndRegion = reshape([separatingPlanes.Normal], 2, 2, regionCount);
startNormals             = reshape(normalByEndpointAndRegion(1, :, :), 2, regionCount).';
endNormals               = reshape(normalByEndpointAndRegion(2, :, :), 2, regionCount).';

lineOffset_units = reshape([separatingPlanes.Offset_units], 2, regionCount).';
vertices_units   = vertcat(regions_units{:});
regionIndexByVertex = repelem((1:regionCount).', ...
    cellfun(@(region) size(region, 1), regions_units));
regionIndexByVertex = regionIndexByVertex(:);

%% Section 2: Bound Both Sides Of Each Line Throughout The Interval

% A static vertex has a line-side value that changes linearly. The middle
% coefficient below is the average of its endpoint values; all three follow
% the same representation used for linearly moving regions.
startSideWithoutOffset_units = sum(vertices_units .* startNormals(regionIndexByVertex, :), 2);
endSideWithoutOffset_units   = sum(vertices_units .* endNormals(regionIndexByVertex, :), 2);
minimumVertexSide_units = min([startSideWithoutOffset_units + lineOffset_units(regionIndexByVertex, 1), ...
    (endSideWithoutOffset_units + startSideWithoutOffset_units + sum(lineOffset_units(regionIndexByVertex, :), 2)) / 2, ...
    endSideWithoutOffset_units + lineOffset_units(regionIndexByVertex, 2)], [], 2);
minimumObstacleSide_units = accumarray(regionIndexByVertex, minimumVertexSide_units, ...
    [regionCount, 1], @min);

% Curve degree D and line-normal degree 1 produce degree D + 1.
% The largest Bernstein coefficient bounds the curve-side value everywhere.
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
geometryScale_units = accumarray(regionIndexByVertex, max(abs(vertices_units), [], 2), ...
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
