function collisionPairs = findSampledObstacleOverlaps(controlPoint_units, regions_units, regionMinimum_units, regionMaximum_units, regionActiveBySegment, sampleCount)
%% Section 0: Header & Readme
% SYNTAX
%   collisionPairs = bmtpEngine.findSampledObstacleOverlaps( ...
%       controlPoint_units, regions_units, regionMinimum_units, ...
%       regionMaximum_units, regionActiveBySegment, sampleCount)
%
% PURPOSE
%   - Identify sampled Bezier span and convex-region overlaps that require
%     separating-line updates during optimization.
%   - Never treat the sampled result as a final acceptance certificate.
%
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier control points.
%   - regions_units (R-by-1 cell array)
%       Convex exclusion polygons.
%   - regionMinimum_units, regionMaximum_units (R-by-2 numeric arrays)
%       Cached region bounds.
%   - regionActiveBySegment (S-by-R logical array)
%       Applicable curve-region pairs.
%   - sampleCount (positive integer scalar)
%       Samples per curve span.
%
% OUTPUTS
%   - collisionPairs (S-by-R logical array)
%       Sampled-overlap tags used only to guide later optimization.
%
% UNITS
%   - Position and region bounds are coordinate units.
%

%% Section 1: Check Sampled Span And Region Overlaps

segmentCount   = size(controlPoint_units, 1);
collisionPairs = false(segmentCount, numel(regions_units));
tau            = linspace(0, 1, sampleCount).';
% Process each segment while assembling the complete motion or interval result.
for segmentIndex = 1:segmentCount
    position_units      = evaluateBezier(squeeze(controlPoint_units(segmentIndex, :, :)), tau);
    sampleMinimum_units = min(position_units, [], 1);
    sampleMaximum_units = max(position_units, [], 1);
    overlaps          = regionActiveBySegment(segmentIndex, :).' & regionMinimum_units(:, 1) <= sampleMaximum_units(1) & regionMaximum_units(:, 1) >= sampleMinimum_units(1) & regionMinimum_units(:, 2) <= sampleMaximum_units(2) & regionMaximum_units(:, 2) >= sampleMinimum_units(2);
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = reshape(find(overlaps), 1, [])
        vertices_units = regions_units{regionIndex};
        [inside, on] = inpolygon(position_units(:, 1), position_units(:, 2), vertices_units(:, 1), vertices_units(:, 2));
        collisionPairs(segmentIndex, regionIndex) = any(inside | on);
    end
end
end

%% Section 2: Local Functions

function position_units = evaluateBezier(controlPoint_units, tau)
    % Evaluate samples through one vectorized de Casteljau recurrence.
    degree   = size(controlPoint_units, 1) - 1;
    tau      = reshape(double(tau), [], 1, 1);
    work_units = repmat(reshape(controlPoint_units, 1, degree + 1, []), numel(tau), 1, 1);
    % Repeat the level alternatives needed to refine the current solution.
    for levelIndex = 1:degree
        work_units = (1 - tau) .* work_units(:, 1:end - 1, :) + tau .* work_units(:, 2:end, :);
    end
    position_units = reshape(work_units(:, 1, :), numel(tau), size(controlPoint_units, 2));
end
