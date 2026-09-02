function collisionPairs = findSampledObstacleOverlaps( ...
        controlPoint_deg, regions_deg, regionMinimum_deg, ...
        regionMaximum_deg, regionActiveBySegment, sampleCount)
%% Section 0: Header & Readme
% SYNTAX
%   collisionPairs = bmtpEngine.findSampledObstacleOverlaps( ...
%       controlPoint_deg, regions_deg, regionMinimum_deg, ...
%       regionMaximum_deg, regionActiveBySegment, sampleCount)
%**************************************************************************
% PURPOSE
%   - Identify sampled Bezier span and convex-region overlaps that require
%     separating-line updates during optimization.
%   - Never treat the sampled result as a final acceptance certificate.
%**************************************************************************
% INPUTS
%   - controlPoint_deg (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points.
%   - regions_deg (R-by-1 cell array)
%       Convex exclusion polygons.
%   - regionMinimum_deg, regionMaximum_deg (R-by-2 numeric arrays)
%       Cached region bounds.
%   - regionActiveBySegment (S-by-R logical array)
%       Applicable curve-region pairs.
%   - sampleCount (positive integer scalar)
%       Samples per curve span.
%**************************************************************************
% OUTPUTS
%   - collisionPairs (S-by-R logical array)
%       Sampled-overlap tags used only to guide later optimization.
%**************************************************************************
% UNITS
%   - Position and region bounds are degrees.
%**************************************************************************

%% Section 1: Check Sampled Span And Region Overlaps

segmentCount = size(controlPoint_deg, 1);
collisionPairs = false(segmentCount, numel(regions_deg));
tau = linspace(0, 1, sampleCount).';
for segmentIndex = 1:segmentCount
    position_deg = evaluateBezier( ...
        squeeze(controlPoint_deg(segmentIndex, :, :)), tau);
    sampleMinimum_deg = min(position_deg, [], 1);
    sampleMaximum_deg = max(position_deg, [], 1);
    overlaps = regionActiveBySegment(segmentIndex, :).' & ...
        regionMinimum_deg(:, 1) <= sampleMaximum_deg(1) & ...
        regionMaximum_deg(:, 1) >= sampleMinimum_deg(1) & ...
        regionMinimum_deg(:, 2) <= sampleMaximum_deg(2) & ...
        regionMaximum_deg(:, 2) >= sampleMinimum_deg(2);
    for regionIndex = reshape(find(overlaps), 1, [])
        vertices_deg = regions_deg{regionIndex};
        [inside, on] = inpolygon(position_deg(:, 1), ...
            position_deg(:, 2), vertices_deg(:, 1), vertices_deg(:, 2));
        collisionPairs(segmentIndex, regionIndex) = any(inside | on);
    end
end
end

%% Section 2: Local Functions

function position_deg = evaluateBezier(controlPoint_deg, tau)
% Evaluate samples through one vectorized de Casteljau recurrence.
degree = size(controlPoint_deg, 1) - 1;
tau = reshape(double(tau), [], 1, 1);
work_deg = repmat(reshape(controlPoint_deg, 1, degree + 1, []), ...
    numel(tau), 1, 1);
for levelIndex = 1:degree
    work_deg = (1 - tau) .* work_deg(:, 1:end - 1, :) + ...
        tau .* work_deg(:, 2:end, :);
end
position_deg = reshape(work_deg(:, 1, :), ...
    numel(tau), size(controlPoint_deg, 2));
end
