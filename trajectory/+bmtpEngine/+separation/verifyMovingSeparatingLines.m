function separatingPlanes = verifyMovingSeparatingLines(separatingPlanes, controlPoint_units, ...
    startRegions_units, endRegions_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   separatingPlanes = bmtpEngine.separation.verifyMovingSeparatingLines( ...
%       separatingPlanes, controlPoint_units, startRegions_units, endRegions_units, ...
%       roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Check one motion curve against several moving regions in one batch.
%     Each region keeps its own line; batching repeats the same full-time
%     bounds and acceptance rules used for one line.
%**************************************************************************
% INPUTS
%   - separatingPlanes (R-element struct array)
%       One line per convex region, with normal and offset at both ends of
%       the interval. Each changes linearly between those ends.
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier controls for the same motion interval against all R regions.
%   - startRegions_units, endRegions_units (R-by-1 cell arrays)
%       Matching convex polygon vertices at the interval endpoints. Vertex
%       i at the start moves linearly to vertex i at the end.
%   - roundoffReserve_units, separationTarget_units (nonnegative numeric scalars)
%       Required curve-side rounding margin and obstacle-side target.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (R-element struct array)
%       Same line records with updated offsets and conservative SignedGap
%       bounds. Verified is true only when the whole-interval check passes.
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
% Stack all vertices for one calculation. Record which region owns each
% vertex so each projection uses that region's normal and offset.
normalByEndpointAndRegion = reshape([separatingPlanes.Normal], 2, 2, regionCount);
startNormals             = reshape(normalByEndpointAndRegion(1, :, :), 2, regionCount).';
endNormals               = reshape(normalByEndpointAndRegion(2, :, :), 2, regionCount).';

lineOffset_units    = reshape([separatingPlanes.Offset_units], 2, regionCount).';
startVertices_units = vertcat(startRegions_units{:});
endVertices_units   = vertcat(endRegions_units{:});
regionIndexByVertex = repelem((1:regionCount).', startVertexCounts);
regionIndexByVertex = regionIndexByVertex(:);

%% Section 2: Bound Both Sides Of Each Line Throughout The Interval

% A linearly moving vertex dotted with a linearly changing normal makes a
% quadratic in time fraction. Its three Bezier coefficients bound its
% line-side value between endpoints. Take the smallest coefficient for
% each vertex, then the smallest of those bounds for each region.
startSideWithoutOffset_units = sum(startVertices_units .* startNormals(regionIndexByVertex, :), 2);
endSideWithoutOffset_units   = sum(endVertices_units .* endNormals(regionIndexByVertex, :), 2);
crossSideWithoutOffset_units = sum(startVertices_units .* endNormals(regionIndexByVertex, :), 2) + ...
    sum(endVertices_units .* startNormals(regionIndexByVertex, :), 2);
minimumVertexSide_units = min([startSideWithoutOffset_units + lineOffset_units(regionIndexByVertex, 1), ...
    (crossSideWithoutOffset_units + sum(lineOffset_units(regionIndexByVertex, :), 2)) / 2, ...
    endSideWithoutOffset_units + lineOffset_units(regionIndexByVertex, 2)], [], 2);
minimumObstacleSide_units = accumarray(regionIndexByVertex, minimumVertexSide_units, ...
    [regionCount, 1], @min);

% A degree-D curve dotted with a linear normal has degree D+1. The
% largest of its D+2 Bezier coefficients bounds the curve-side value for
% the whole interval, including between any plotted or sampled points.
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
% A normal interpolated between two endpoint normals cannot be longer
% than the longer endpoint normal.
maximumNormalLength = max( ...
    [vecnorm(startNormals, 2, 2), vecnorm(endNormals, 2, 2)], [], 2);

%% Section 3: Shift The Lines When Possible And Apply All Gap Checks

% Scale the rounding allowance to each region's coordinates, its offsets,
% and the common curve. proveSeparation may shift both offsets equally,
% but every required margin must still pass after that shift.

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
