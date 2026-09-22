function sampledOverlapPairs = findSampledObstacleOverlaps(controlPoint_units, regions_units, ...
    regionMinimum_units, regionMaximum_units, regionActiveBySegment)
%% Section 0: Header & Readme
% SYNTAX
%   sampledOverlapPairs = bmtpEngine.separation.findSampledObstacleOverlaps(controlPoint_units, ...
%       regions_units, regionMinimum_units, regionMaximum_units, regionActiveBySegment)
%**************************************************************************
% PURPOSE
%   - Find curve samples that touch or enter an obstacle, so the optimizer
%     knows which separating lines to update. Samples can miss a collision
%     between them, so the complete curve still needs its motion checks.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points.
%   - regions_units (R-by-1 cell array)
%       Protected convex obstacle polygons.
%   - regionMinimum_units (R-by-2 numeric array)
%       Cached minimum region bounds.
%   - regionMaximum_units (R-by-2 numeric array)
%       Cached maximum region bounds.
%   - regionActiveBySegment (S-by-R logical array)
%       Applicable curve-region pairs.
%**************************************************************************
% OUTPUTS
%   - sampledOverlapPairs (S-by-R logical array)
%       True where at least one sample touches or lies inside that obstacle.
%**************************************************************************
% UNITS
%   - Position and region bounds are coordinate units.
%**************************************************************************

%% Section 1: Check Curve Samples Against Applicable Obstacles

% Sample 1201 equally spaced fractions of each segment, including both ends.
% This sampling density belongs to the optimizer, not the output plot spacing.
sampleCount         = 1201;
segmentCount        = size(controlPoint_units, 1);
sampledOverlapPairs = false(segmentCount, numel(regions_units));
segmentFractions    = linspace(0, 1, sampleCount).';
for segmentIndex = 1:segmentCount
    sampledPosition_units = evaluateBezier(squeeze(controlPoint_units(segmentIndex, :, :)), segmentFractions);
    sampleMinimum_units   = min(sampledPosition_units, [], 1);
    sampleMaximum_units   = max(sampledPosition_units, [], 1);
    % Skip obstacle boxes that cannot contain any of these sampled points.
    % The surviving obstacles still need the polygon containment check below.
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
