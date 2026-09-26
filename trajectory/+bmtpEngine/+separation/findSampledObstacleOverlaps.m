function sampledOverlapPairs = findSampledObstacleOverlaps(controlPoint_units, regions_units, ...
    regionMinimum_units, regionMaximum_units, regionActiveBySegment)
%% Section 0: Header & Readme
% SYNTAX
%   sampledOverlapPairs = bmtpEngine.separation.findSampledObstacleOverlaps(controlPoint_units, ...
%       regions_units, regionMinimum_units, regionMaximum_units, regionActiveBySegment)
%**************************************************************************
% PURPOSE
%   - Flag motion-segment/static-obstacle pairs with a sampled position on
%     or inside the protected polygon. The optimizer uses these flags to
%     revisit separating lines. A clear sample set does not prove the full
%     curve clear; independent full-curve checks are still required.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Bezier controls for S motion segments, with x/y in the last axis.
%   - regions_units (R-by-1 cell array)
%       Prepared, static convex polygons, one N-by-2 vertex array per cell.
%   - regionMinimum_units (R-by-2 numeric array)
%       Smallest x and y coordinate of each polygon.
%   - regionMaximum_units (R-by-2 numeric array)
%       Largest x and y coordinate of each polygon.
%   - regionActiveBySegment (S-by-R logical array)
%       True for pairs to check; false pairs are skipped.
%**************************************************************************
% OUTPUTS
%   - sampledOverlapPairs (S-by-R logical array)
%       True if at least one sampled point touches or lies inside the
%       polygon. False does not certify clearance between samples.
%**************************************************************************
% UNITS
%   - Position and region bounds are coordinate units.
%**************************************************************************

%% Section 1: Check Curve Samples Against Applicable Obstacles

% Sample 1201 equally spaced curve fractions, including 0 and 1. Equal
% fraction steps need not cover equal travel distances.
% These samples guide optimization; they do not set output plot spacing.
sampleCount         = 1201;
segmentCount        = size(controlPoint_units, 1);
sampledOverlapPairs = false(segmentCount, numel(regions_units));
segmentFractions    = linspace(0, 1, sampleCount).';
for segmentIndex = 1:segmentCount
    sampledPosition_units = evaluateBezier(squeeze(controlPoint_units(segmentIndex, :, :)), segmentFractions);
    sampleMinimum_units   = min(sampledPosition_units, [], 1);
    sampleMaximum_units   = max(sampledPosition_units, [], 1);
    % If the sample x/y range misses a polygon's x/y box, no sampled point
    % can lie in that polygon. A box overlap alone does not mean collision;
    % check the actual polygon for every surviving active pair.
    regionIsActive   = regionActiveBySegment(segmentIndex, :).';
    minimumXOverlaps = regionMinimum_units(:, 1) <= sampleMaximum_units(1);
    maximumXOverlaps = regionMaximum_units(:, 1) >= sampleMinimum_units(1);
    minimumYOverlaps = regionMinimum_units(:, 2) <= sampleMaximum_units(2);
    maximumYOverlaps = regionMaximum_units(:, 2) >= sampleMinimum_units(2);
    boundsOverlap    = regionIsActive & minimumXOverlaps & maximumXOverlaps & ...
        minimumYOverlaps & maximumYOverlaps;
    for regionIndex = reshape(find(boundsOverlap), 1, [])
        obstacleVertices_units = regions_units{regionIndex};
        % Boundary contact counts as overlap as well as an interior point.
        [sampleIsInside, sampleIsOnBoundary] = inpolygon(sampledPosition_units(:, 1), sampledPosition_units(:, 2), ...
            obstacleVertices_units(:, 1), obstacleVertices_units(:, 2));
        sampledOverlapPairs(segmentIndex, regionIndex) = any(sampleIsInside | sampleIsOnBoundary);
    end
end
end

%% Section 2: Local Functions

function sampledPosition_units = evaluateBezier(controlPoint_units, segmentFractions)
    % Repeatedly blend adjacent controls at each requested fraction until
    % one curve point remains (de Casteljau evaluation). At fraction 0.25,
    % each blend is 0.75 x left control + 0.25 x right control.
    degree                     = size(controlPoint_units, 1) - 1;
    segmentFractions           = reshape(double(segmentFractions), [], 1, 1);
    interpolatedControls_units = repmat(reshape(controlPoint_units, 1, degree + 1, []), numel(segmentFractions), 1, 1);
    for interpolationLevel = 1:degree
        interpolatedControls_units = (1 - segmentFractions) .* interpolatedControls_units(:, 1:end - 1, :) + ...
            segmentFractions .* interpolatedControls_units(:, 2:end, :);
    end
    sampledPosition_units = reshape(interpolatedControls_units(:, 1, :), ...
        numel(segmentFractions), size(controlPoint_units, 2));
end
